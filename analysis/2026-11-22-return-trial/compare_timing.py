"""Compare both observers on the recorded trajectories up to their first decisions.

The historical run's profile is replaced for this comparison. A packet without a
saved motion snapshot is not fabricated. Both observers get the same available
IMU samples between snapshots and the same raw camera image as of each sample.
A changed decision terminates that observer's counterfactual for this half-cycle.
The next half-cycle starts from its recorded state, not the preceding prediction.
"""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from replay_small_arc import rows

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-11-22-walking-simulator'))
import return_model

FLAGS = ('timestamp_valid', 'frame_valid', 'zero_ready', 'left_valid', 'right_valid',
         'left_control_range_ok', 'right_control_range_ok')


def compare(folder, exe):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    energy = meta['walking_energy']
    if energy['stored'] != energy['total'] or len(energy['records']) != energy['stored']:
        raise ValueError(f'{folder.name}: incomplete walking commands')
    frames = sorted((f for f in meta['foot_frames'] if f['state_id'] == 3 and f['control_observed_us']),
                    key=lambda f: f['control_observed_us'])
    epoch, = {f['frame_us'] - f['measurement_time_us'] for f in frames if f['timestamp_valid']}
    ready = [f['control_observed_us'] for f in frames]
    image_by_seq = {f['sequence']: f['frame_us'] for f in frames}
    period = meta['foot_periodic']['period_us']
    packets = {int(r['slot_us']): r for r in rows(folder, 'foot_periodic')}
    shadows = defaultdict(list)
    for s in rows(folder, 'support_transfer_shadow'):
        capture = epoch + ((int(s['capture_us32']) - epoch) & 0xffffffff)
        shadows[int(s['peak_ms'])].append((capture, s))
    imu = []
    for r in rows(folder, 'timeseries'):
        if r['state_id'] != '3' or not r['body_rate_sample_time_us'] or not r['imu_sample_age_us']:
            continue
        if not all(r[k] for k in ('pitch_mekf_detector_relative_deg', 'gy_dps', 'mekf_bias_y_dps')):
            continue
        elapsed = (int(r['body_rate_sample_time_us']) - epoch) & 0xffffffff
        # The first controller rows can still refer to the last gyro sample before t=0.
        if elapsed > 600_000_000:
            continue
        sample = epoch + elapsed
        if imu and sample <= imu[-1][1]:
            continue
        age = int(r['imu_sample_age_us'])
        rate = (float(r['gy_dps']) - float(r['mekf_bias_y_dps'])) * meta['mekf_gyro_y_scale']
        imu.append((sample + age, sample, int(r['t_test_ms']), r['pitch_mekf_detector_relative_deg'], rate))
    if not imu:
        raise ValueError(f'{folder.name}: no running IMU observations')
    imu.sort()
    imu_times = [v[0] for v in imu]
    peaks = {p['peak_time_ms']: p for p in meta['energy_control_autonomous_peak_events']}
    crossing_events = meta['energy_control_autonomous_zero_cross_events']
    episodes = []
    return_model.load_sections(folder, sections=(), episodes=episodes, foot_packets=[])
    stream = []

    def put(kind, *fields):
        stream.append(kind + ' ' + ' '.join(str(v) if v not in (None, '') else 'nan' for v in fields))

    for e in episodes:
        peak = e['peak_ms']
        end = e['end_ms']
        commands = [c for c in energy['records'] if c['peak_ms'] == peak and
                    (c['executed'] or c['return_event']) and c['time_ms'] < end]
        first = min(commands, key=lambda c: c['time_ms'], default=None)
        e['archived_decision_ms'] = first['time_ms'] if first else None
        e['archived_decision'] = ('return' if first['return_event'] else 'main') if first else None
        e['archived_decision_executed'] = first['executed'] if first else None
        # A main issued at the observed crossing cannot account for reaching that
        # crossing. Treat milliseconds within one tick of the endpoint as coincident.
        e['recorded_pulse_before_end'] = any(c['output_executed'] and
            c['pulse_start_ms'] < end - 1 and c['pulse_end_ms'] > peak for c in crossing_events)
        start = e['peak_available_ms']
        stop = min(end, first['time_ms'] if first else end)
        events = []
        # These saved inputs were taken before that tick's output decision. Their exact IMU
        # timestamps are not archived. Use capture as a fresh sample proxy on these ticks.
        for at, s in shadows[peak]:
            now = int(s['time_ms'])
            if start < now < stop:
                events.append((at, 0, (at, now, s['lateral_deg'], s['rate_dps'], s)))
        for at, sample, now, angle, rate in imu[bisect_left(imu_times, epoch + (start-2)*1000):
                                                bisect_right(imu_times, epoch + (stop+2)*1000)]:
            if start < now < stop:
                events.append((at, 1, (sample, now, angle, rate, None)))
        tau = meta['support_transfer_shadow']['rise_tau_s']
        put('K', peak, epoch, peaks[peak]['peak_amplitude_deg'], peaks[peak]['target_energy_j'], tau)
        last_frame = last_sample = 0
        allowed = False
        last_capture = 0
        packet_ids = set()
        stale_sample_count = 0
        for at, source, (sample, now, angle, rate, s) in sorted(events):
            if source and sample <= last_sample:
                continue
            if source and last_sample and sample-last_sample > 10000:
                stale_sample_count += 1
            last_sample = sample
            fi = bisect_right(ready, at) - 1
            if fi < 0:
                continue
            f = frames[fi]
            if f['sequence'] != last_frame:
                last_frame = f['sequence']
                flags = sum(1 << k for k,key in enumerate(FLAGS) if f[key])
                put('F', flags, last_frame, f['frame_us'], f['control_observed_us'],
                    max(0, f['measurement_time_us']//1000), max(0, (ready[fi]-epoch)//1000),
                    f['left_deg'], f['right_deg'])
            if s is not None:
                count = int(s['packet'])
                p = packets[count*period]
                packet_ids.add(count)
                left, right = int(p['left_sequence']), int(p['right_sequence'])
                put('P', count, epoch+int(p['slot_us']), epoch+int(p['publish_us']),
                    left, image_by_seq.get(left, 0), p['left_observed_deg'],
                    right, image_by_seq.get(right, 0), p['right_observed_deg'], s['fore_aft_deg'])
                # InvalidFoot additionally requires two finite slopes; the actual observer
                # checks the two source images and measured angles instead.
                allowed = not int(s['baseline_flags']) and not (int(s['gate_mask']) & ~4)
                last_capture = at
            put('I', now, sample & 0xffffffff, at-sample, int(allowed and at-last_capture <= period),
                angle, rate, s['accel_dps2'] if s is not None else None)
        e['saved_slots'] = len(packet_ids)
        e['missing_interior_slots'] = (max(packet_ids)-min(packet_ids)+1-len(packet_ids)) if packet_ids else 0
        e['saved_imu_gaps_over_10ms'] = stale_sample_count
        e['old'] = e['new'] = None

    result = subprocess.run([exe], input='\n'.join(stream)+'\n', text=True, capture_output=True, check=True)
    by_peak = {e['peak_ms']: e for e in episodes}
    for line in result.stdout.splitlines():
        peak, variant, now, evidence, request, packet = line.split()
        by_peak[int(peak)]['new' if int(variant) else 'old'] = dict(
            ms=int(now), evidence=int(evidence), request_uJ=float(request), packet=int(packet))
    return episodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path, help='Parents of converted per-run folders')
    parser.add_argument('--json-out', type=Path)
    args = parser.parse_args()
    folders = sorted({h.parent for root in args.roots for h in root.glob('*/header.json')})
    if not folders:
        parser.error('No converted run folders found')
    data = []
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp)/'compare_observers.exe'
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-I', HERE.parents[1]/'src',
                        HERE/'compare_observers.cpp', '-o', exe], check=True)
        for folder in folders:
            data.extend(compare(folder, exe))
    changes = [e for e in data if e['new'] != e['old']]
    print('Right returns', len(data), 'outcome / prior pulse',
          dict(Counter((e['outcome'], e['recorded_pulse_before_end']) for e in data)))
    print('No saved decision slots', sum(not e['saved_slots'] for e in data),
          'returns with skipped interior slots', sum(bool(e['missing_interior_slots']) for e in data))
    coasts = [e for e in data if e['outcome'] == 'crossed' and not e['recorded_pulse_before_end']]
    print('Unpowered crossings:', len(coasts), 'with new predictive decisions:',
          sum(e['new'] is not None and e['new']['evidence'] == 6 for e in coasts),
          'with skipped interior slots:', sum(bool(e['missing_interior_slots']) for e in coasts))
    for e in changes:
        print(e['run'], e['peak_ms'], e['old'], '->', e['new'], e['outcome'], e['end_ms'],
              'actual decision', e['archived_decision_ms'], 'prior pulse', e['recorded_pulse_before_end'])
    if args.json_out:
        args.json_out.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')


if __name__ == '__main__':
    main()
