'use strict';
// Pure analysis of frozen frame-delivery snapshots. No device writes or fits.
const PoseComparison = (() => {
  const limits = Object.freeze({samples:5, interval_ms:800, min_span_us:2500000,
    max_span_us:12000000, max_age_ms:300, max_sensor_age_us:300000,
    max_gyro_dps:2, max_accel_error_g:0.08, max_tilt_spread_deg:0.8,
    max_marker_spread_px:3, max_poses:12, max_trace:600});
  const wrap = x => ((x + 180) % 360 + 360) % 360 - 180;
  const mean = a => a.reduce((s,x) => s + x, 0) / a.length;
  const vector = v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
  function orientation(q) {
    if (!q || ![q.w,q.x,q.y,q.z].every(Number.isFinite)) throw Error('姿勢データが不完全です');
    const n = Math.hypot(q.w,q.x,q.y,q.z);
    if (Math.abs(n - 1) > 0.01) throw Error('姿勢データの長さが不正です');
    const [w,x,y,z] = [q.w,q.x,q.y,q.z].map(v => v / n);
    const up = [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
    const deg = 180 / Math.PI;
    return {roll_deg:Math.atan2(up[1],up[2])*deg,
      pitch_deg:Math.atan2(-up[0],Math.hypot(up[1],up[2]))*deg};
  }
  function sameSession(a,b) {
    return a.boot_id === b.boot_id && a.run_id === b.run_id && a.revision === b.revision &&
      a.right_zero_x === b.right_zero_x && a.left_zero_x === b.left_zero_x;
  }
  function validateFrame(m) {
    if (!m || !Number.isInteger(m.boot_id) || !Number.isInteger(m.run_id) ||
        typeof m.revision !== 'string' || m.state_id !== 2)
      throw Error('測定前の待機状態で取得してください');
    if (!m.timestamp_valid || !Number.isFinite(m.age_ms) || m.age_ms < 0 || m.age_ms > limits.max_age_ms ||
        !Number.isSafeInteger(m.frame_us) || !Number.isSafeInteger(m.delivered_us) || m.frame_us <= 0 ||
        m.delivered_us < m.frame_us || m.delivered_us - m.frame_us > limits.max_sensor_age_us ||
        !Number.isInteger(m.sequence) || !m.zero_ready || !m.right_valid || !m.left_valid ||
        !m.right?.valid || !m.left?.valid ||
        ![m.right_deg,m.left_deg,m.right_zero_x,m.left_zero_x,m.right.x,m.left.x].every(Number.isFinite))
      throw Error('左右マーカーと新しい画像を確認してください');
    const k = m.mekf, input = k?.inputs;
    if (!k?.valid || !k.fresh || k.frame !== 'mekf' || k.euler_order !== 'ZYX' ||
        !Number.isFinite(k.age_us) || k.age_us < 0 || k.age_us > limits.max_sensor_age_us ||
        !input?.valid || input.frame !== 'mekf' || input.axis_order !== 'xyz' ||
        !Number.isFinite(input.accel_age_us) || input.accel_age_us < 0 || input.accel_age_us > limits.max_sensor_age_us ||
        !vector(input.accel_g) || !vector(input.gyro_dps) || !vector(input.gyro_bias_dps))
      throw Error('新しいIMU診断が必要です。ファームウェアと接続を確認してください');
    orientation(k.quaternion);
    const rate = Math.hypot(...input.gyro_dps.map((v,i) => v - input.gyro_bias_dps[i]));
    if (rate > limits.max_gyro_dps || Math.abs(Math.hypot(...input.accel_g) - 1) > limits.max_accel_error_g)
      throw Error('動きを検出しました。姿勢を保って再取得してください');
  }
  function summarize(frames, baseline = false) {
    if (frames.length !== limits.samples) throw Error('静止姿勢の試料が不足しています');
    frames.forEach(validateFrame);
    const first = frames[0], last = frames[frames.length - 1];
    if (!frames.every(m => sameSession(m,first))) throw Error('再起動・測定・ゼロ点の変更を検出しました。基準を取り直してください');
    for (let i=1;i<frames.length;i++) {
      if (frames[i].sequence === frames[i-1].sequence || frames[i].frame_us <= frames[i-1].frame_us ||
          frames[i].delivered_us <= frames[i-1].delivered_us)
        throw Error('画像が更新されていません');
    }
    const span = last.delivered_us - first.delivered_us;
    if (span < limits.min_span_us || span > limits.max_span_us) throw Error('取得間隔が不適切です。再取得してください');
    const angles = frames.map(m => orientation(m.mekf.quaternion));
    const summary = {boot_id:first.boot_id, run_id:first.run_id, revision:first.revision,
      right_zero_x:first.right_zero_x, left_zero_x:first.left_zero_x,
      first_frame_us:first.frame_us, last_frame_us:last.frame_us, span_us:span,
      right_in_range:frames.every(m => m.right_in_range === true),
      left_in_range:frames.every(m => m.left_in_range === true), spread:{}};
    for (const [key,values,maxSpread] of [
      ...['roll_deg','pitch_deg'].map(key => [key,
        angles.map(a => angles[0][key] + wrap(a[key]-angles[0][key])),
        limits.max_tilt_spread_deg]),
      ['right_x',frames.map(m => m.right.x),limits.max_marker_spread_px],
      ['left_x',frames.map(m => m.left.x),limits.max_marker_spread_px]]) {
      summary[key] = mean(values); summary.spread[key] = Math.max(...values) - Math.min(...values);
      if ( summary.spread[key] > maxSpread) throw Error('姿勢または足の位置が動いています。静止させて再取得してください');
    }
    for (const side of ['right','left']) summary[side+'_deg'] = mean(frames.map(m => m[side+'_deg']));
    const accelRoll = frames.map(m => Math.atan2(m.mekf.inputs.accel_g[1],m.mekf.inputs.accel_g[2])*180/Math.PI);
    summary.accel_roll_deg = mean(accelRoll.map(v => accelRoll[0] + wrap(v-accelRoll[0])));
    if (baseline && (Math.abs(summary.roll_deg)>6 || Math.abs(summary.pitch_deg)>6 ||
        Math.abs(summary.right_deg)>3 || Math.abs(summary.left_deg)>3))
      throw Error('基準は胴体と両足を直立させて取得してください');
    return summary;
  }
  function compare(base,pose) {
    if (!sameSession(base,pose) || pose.first_frame_us <= base.last_frame_us)
      throw Error('基準との取得条件が変わりました。記録を保存し、比較をやり直してください');
    const delta = {roll_deg:wrap(pose.roll_deg-base.roll_deg), pitch_deg:wrap(pose.pitch_deg-base.pitch_deg),
      accel_roll_deg:wrap(pose.accel_roll_deg-base.accel_roll_deg),
      right_deg:pose.right_deg-base.right_deg, left_deg:pose.left_deg-base.left_deg};
    const flags = [];
    if (Math.abs(delta.pitch_deg)>2) flags.push('sideways_changed');
    if (!base.right_in_range || !pose.right_in_range || !base.left_in_range || !pose.left_in_range) flags.push('outside_range');
    if (Math.abs(delta.roll_deg)>30) flags.push('large_tilt');
    return {delta, right_residual_deg:delta.right_deg+delta.roll_deg,
      left_residual_deg:delta.left_deg+delta.roll_deg, planar_check:flags.every(f => f === 'outside_range'), flags};
  }
  return {limits, wrap, orientation, sameSession, validateFrame, summarize, compare};
})();
if (typeof module !== 'undefined') module.exports = PoseComparison;
