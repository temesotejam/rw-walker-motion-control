// Evaluate the last applied pulse in a position-equipped RWLOG with the same
// wheel-work estimator as the browser. All offsets are from the driver's OFF.
const fs = require('node:fs');
const wheel = require('../../web/wheel_work.js');

function nearest(all, key, t) {
  return all.reduce((a, b) => Math.abs(a[key] - t) <= Math.abs(b[key] - t) ? a : b);
}

function windowWork(report, offUs, startMs, endMs) {
  // OFF itself is already a split in the analyzer. Other boundaries use the
  // nearest position timestamp so consecutive windows cover exactly the same data.
  const edge = ms => ms === 0 ? offUs : nearest(report.kinematics, 'time_us', offUs + ms * 1000).time_us;
  const left = edge(startMs), right = edge(endMs);
  const inside = report.intervals.filter(i => i.measurement && i.start_us >= left && i.end_us <= right);
  const valid = inside.filter(i => i.valid);
  const duration = valid.reduce((sum, i) => sum + i.end_us - i.start_us, 0);
  return {
    range_ms: [startMs, endMs],
    first_ms: inside.length ? (inside[0].start_us - offUs) / 1000 : null,
    last_ms: inside.length ? (inside.at(-1).end_us - offUs) / 1000 : null,
    covered_ms: duration / 1000,
    missing_intervals: inside.length - valid.length,
    observed_work_uJ: valid.length ? valid.reduce((sum, i) => sum + i.work_j * 1e6, 0) : null,
  };
}

function analyze(file) {
  const {rows, metadata} = wheel.parse(fs.readFileSync(file));
  if (metadata.wheel_work_velocity_source !== 'position_local_quadratic_5_points')
    throw Error('Cumulative wheel position is required');
  return [5, 9].map(fitPoints => {
    const report = wheel.analyze(rows, metadata, {positionFitPoints: fitPoints});
    const pulse = report.pulses.at(-1);
    if (!pulse || !pulse.end_observed) throw Error('Last pulse has no acknowledged end');
    const off = pulse.end_us;
    const speeds = [0, 15, 50, 100, 145].map(offset => {
      const t = off + offset * 1000;
      // A rejected point is kept as missing even if the next valid point is nearby.
      const k = nearest(report.kinematics, 'time_us', t);
      const reg = nearest(rows.filter(r => r.speed_valid), 'speed_t', t);
      return {offset_ms: offset, position_ms: (k.time_us - off) / 1000,
        position_rpm: k.valid && Math.abs(k.time_us - t) <= 10000 ? k.relative_speed_rpm : null,
        register_ms: (reg.speed_t - off) / 1000,
        register_rpm: Math.abs(reg.speed_t - t) <= 15000 ? reg.speed_rpm : null};
    });
    const post = [[0, 15], [15, 145], [0, 145]].map(([a, b]) => windowWork(report, off, a, b));
    return {fit_points: fitPoints, pulse_index: pulse.index, current_mA: pulse.current_mA,
      command_ms: (off - pulse.start_us) / 1000, command_covered_ms: pulse.covered_us / 1000,
      command_work_uJ: pulse.covered_us ? pulse.observed_net_j * 1e6 : null, speeds, post};
  });
}

if (process.argv.length < 3) throw Error('Pass one or more raw position-equipped RWLOGs');
for (const file of process.argv.slice(2)) console.log(JSON.stringify({file, result: analyze(file)}, null, 2));
