"""Audit body axes, camera availability and wheel history around a reported fall.

The user's direction report is external evidence. Euler attitude, gyro heading
and relative marker angles do not establish a ground angle, contact time or a
causal mechanical explanation. Native energy selection is audited separately.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

from analyze_v14_trial_energy import ROOT, plt, stats


def read_csv(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def truth(value):
    return str(value).lower() in ['1', 'true']


def number(value):
    return float(value) if value not in ['', None] else None


def evidence(path, needle):
    data = (ROOT / path).read_bytes()
    lines = data.decode('utf8').splitlines()
    return {'path': path, 'line': next(i + 1 for i, line in enumerate(lines) if needle in line),
            'sha256': hashlib.sha256(data).hexdigest()}


def at(rows, time_ms):
    return min(rows, key=lambda r: abs(r['time_ms'] - time_ms))


def first_crossing(rows, key, value):
    for a, b in zip(rows, rows[1:]):
        if a[key] < value <= b[key]:
            return {'first_sample_ms': b['time_ms'], 'value': b[key],
                    'previous_sample_ms': a['time_ms'], 'previous_value': a[key],
                    'linear_crossing_ms': a['time_ms'] + (b['time_ms'] - a['time_ms']) *
                        (value - a[key]) / (b[key] - a[key])}
    return None


def analyze(path, user_observation):
    metadata_blob = (path / 'metadata.json').read_bytes()
    meta = json.loads(metadata_blob)
    header = json.loads((path / 'header.json').read_text(encoding='utf8'))
    source = read_csv(path / 'timeseries.csv')
    active = [r for r in source if r['state_id'] == '3']
    terminal = [r for r in source if r['state_id'] == '5']
    assert active and terminal, 'This analysis requires active and ESTOP samples'
    scale = meta['mekf_gyro_y_scale']
    rows = []
    # The historical upright vector is used by tilt_stop, rather than the run's
    # initial Euler roll. Euler-only reconstruction is a rounded diagnostic.
    upright = [-.021626, .033568, .999202]
    for r in active + terminal:
        roll = float(r['roll_mekf_abs_deg']); pitch = float(r['pitch_mekf_abs_deg'])
        rr, pp = math.radians(roll), math.radians(pitch)
        gravity = [-math.sin(pp), math.sin(rr) * math.cos(pp), math.cos(rr) * math.cos(pp)]
        up = sum(a * b for a, b in zip(gravity, upright))
        forward = gravity[1] * upright[2] - gravity[2] * upright[1]
        row = {'time_ms': float(r['t_test_ms']), 'state_id': int(r['state_id']),
               'log_time_s': float(r['log_time_s']), 'lateral_mekf_pitch_deg': pitch,
               'fore_aft_mekf_roll_deg': roll,
               'fore_aft_reference_projection_deg': math.degrees(math.atan2(forward, up)),
               'gyro_heading_diagnostic_deg': number(r['gyro_heading_deg']),
               'gyro_heading_valid': truth(r['gyro_heading_valid']),
               'body_gyro_x_dps': -(float(r['gx_dps']) - float(r['mekf_bias_x_dps'])),
               'body_gyro_y_dps': (float(r['gy_dps']) - float(r['mekf_bias_y_dps'])) * scale,
               'body_gyro_z_dps': -(float(r['gz_dps']) - float(r['mekf_bias_z_dps'])),
               'raw_gx_dps': float(r['gx_dps']), 'raw_gy_dps': float(r['gy_dps']), 'raw_gz_dps': float(r['gz_dps']),
               'gx_log_clipped': abs(float(r['gx_dps'])) >= 327.67 - 1e-6,
               'acc_norm_g': float(r['acc_norm_g']), 'accel_confidence': float(r['mekf_accel_confidence']),
               'accel_residual_deg': float(r['mekf_accel_residual_deg']),
               'accel_update_used': truth(r['mekf_accel_used']),
               'motor_command_ma': float(r['motor_cmd_mA']), 'current_telemetry_ma': float(r['roller_actual_current_mA']),
               'wheel_speed_rpm': float(r['roller_speed_rpm']), 'wheel_speed_valid': truth(r['roller_speed_valid']),
               'battery_mv': float(r['roller_battery_mV']), 'pulse_id': int(r['pulse_id']),
               'pulse_active': truth(r['pulse_active']), 'imu_sample_age_us': float(r['imu_sample_age_us'])}
        rows.append(row)
    end_ms = rows[-1]['time_ms']
    arows = [r for r in rows if r['state_id'] == 3]
    pose = {r['sequence']: r for r in read_csv(path / 'foot_pose_prediction.csv')}
    feet = []
    epochs = []
    for f in read_csv(path / 'foot_angles.csv'):
        if not f['measurement_time_us'] or float(f['measurement_time_us']) < 0:
            continue
        epoch = int(f['frame_us']) - int(f['measurement_time_us'])
        epochs.append(epoch)
        ready_ms = number(pose[f['sequence']]['ready_ms']) if f['sequence'] in pose else None
        fr = {'sequence': int(f['sequence']), 'state_id': int(f['state_id']),
              'measurement_epoch_us': epoch, 'frame_ms': int(f['measurement_time_us']) / 1000,
              'delivered_ms': (int(f['delivered_us']) - epoch) / 1000,
              'recognition_ready_ms': ready_ms,
              'control_observed_ms': (int(f['control_observed_us']) - epoch) / 1000 if f['control_observed_us'] else None,
              'processing_ms': int(f['processing_us']) / 1000}
        assert ready_ms is None or ready_ms >= fr['frame_ms']
        for side in ['left', 'right']:
            fr[side + '_deg'] = number(f[side + '_deg'])
            fr[side + '_reason'] = f[side + '_reason']
            fr[side + '_control_range_ok'] = truth(f[side + '_control_range_ok'])
            fr[side + '_usable'] = all(truth(f[key]) for key in
                ['timestamp_valid', 'frame_valid', 'zero_ready', side + '_valid', side + '_control_range_ok'])
        feet.append(fr)
    assert len(set(epochs)) == 1, 'Do not mix frame epochs or assume run_start is measurement start'
    epoch = epochs[0]
    threshold_rows = arows + terminal_rows(rows)
    thresholds = {str(value): first_crossing(threshold_rows, 'fore_aft_mekf_roll_deg', value)
                  for value in [5, 10, 15, 20, 30, 45, 60, 90]}
    # Final low-angle departure, separate from earlier brief excursions >5 deg.
    final_below_five = next(r for r in reversed(arows) if r['fore_aft_mekf_roll_deg'] <= 5)
    final_segment = [r for r in rows if r['time_ms'] >= final_below_five['time_ms']]
    final_thresholds = {str(value): first_crossing(final_segment, 'fore_aft_mekf_roll_deg', value)
                        for value in [5, 10, 15, 20, 30, 45, 60, 90]}
    outputs = [a for a in meta['walking_energy']['records'] if a['executed']]
    last_outputs = []
    for a in outputs:
        if a['time_ms'] < end_ms - 3000:
            continue
        item = {key: a[key] for key in ['time_ms', 'side', 'rescue', 'return_event', 'output_start_ms',
            'output_end_ms', 'actual_elapsed_ms', 'guarded_ms', 'completion_reason']}
        for key in ['budget_kind', 'return_source', 'return_dropout', 'halfcycle_used_ms', 'halfcycle_remaining_ms']:
            if key in a:
                item[key] = a[key]
        for phase, time in [('start', a['output_start_ms']), ('end', a['output_end_ms'])]:
            item[phase + '_nearest_attitude'] = at(rows, time)
        last_outputs.append(item)
    windows = {}
    for name, begin, end in [('first_10s', 0, 10000), ('10_to_15s', 10000, 15000),
                            ('15_to_16_3s', 15000, 16300), ('16_3_to_16_8s', 16300, 16800),
                            ('last_186ms', 16800, end_ms + 1)]:
        group = [r for r in rows if begin <= r['time_ms'] < end]
        windows[name] = {'begin_ms': begin, 'end_ms': end,
            **{key: stats([r[key] for r in group]) for key in
               ['lateral_mekf_pitch_deg', 'fore_aft_mekf_roll_deg', 'body_gyro_x_dps',
                'wheel_speed_rpm', 'acc_norm_g', 'accel_confidence']}}
    clipped = [r for r in rows if r['gx_log_clipped']]
    positive_confidence = max(i for i, r in enumerate(rows) if r['accel_confidence'] > 0)
    confidence_tail = rows[positive_confidence + 1:]
    missing_left = [f for f in feet if 16000 <= f['frame_ms'] <= end_ms and not f['left_usable']]
    resumed_left = next((f for f in feet if missing_left and f['frame_ms'] > missing_left[-1]['frame_ms'] and f['left_usable']), None)
    review = {'build': meta['diagnostic_build_id'], 'user_observation': user_observation,
        'terminal_state': meta['terminal_state'], 'metadata_sha256': hashlib.sha256(metadata_blob).hexdigest(),
        'measurement_epoch_us': epoch, 'header_run_start_us': header['run_start_us'],
        'measurement_epoch_minus_run_start_us': epoch - header['run_start_us'],
        'epoch_method': 'frame_us minus measurement_time_us; identical across all measurement frames',
        'axis_mapping': {'raw_to_body_accel': '(-ax,+ay,-az)', 'raw_to_body_gyro': '(-gx,+gy*scale,-gz)',
            'gyro_y_scale': scale, 'lateral_control_angle': 'pitch_mekf_abs_deg; MEKF ZYX pitch about body Y',
            'fore_aft_observation': 'roll_mekf_abs_deg; MEKF ZYX roll about body X',
            'third_attitude_trace': 'gyro_heading_deg is independent integrated gyro diagnostic; no magnetic or ground reference',
            'body_rate_semantics': 'Mapped raw gyro minus archived live MEKF bias; body components are not Euler angle derivatives',
            'foot_semantics': 'Camera-to-body relative calibrated marker angles; not ground-relative leg angles',
            'guard_projection': 'Diagnostic reconstructed from rounded archived roll/pitch and historical upright vector; not exact quaternion guard replay'},
        'source_evidence': [evidence('src/experiment_runner.cpp', 'mekf6::Vec3 mekfAccelFromRaw'),
            evidence('src/experiment_runner.cpp', 'tilt_guard_.reason'), evidence('src/main.cpp', 'p.pose.fore_aft_deg='),
            evidence('src/mekf6.cpp', 'EulerDeg Mekf6::eulerDegFromQuaternion'),
            evidence('src/log_sample_encoder.cpp', 'row.roll_mekf_abs_cdeg = log_quantization'),
            evidence('src/log_quantization.cpp', 'if (value < -32767.0f)'),
            evidence('src/tilt_stop.h', 'return up <= 0.0f'),
            evidence('src/upright_pose_guide.h', 'static constexpr float REF_AX'),
            evidence('src/gyro_heading_diagnostic.cpp', 'heading_.update(')],
        'first_estop_row': rows[-1], 'last_running_row': arows[-1],
        'first_roll_threshold_crossings': thresholds, 'final_excursion_roll_threshold_crossings': final_thresholds,
        'sample_snapshots': [at(rows, time) for time in [16000, 16200, 16368, 16400, 16500, 16600, 16700, 16803, 16851, 16900, end_ms]],
        'windows': windows, 'last_outputs': last_outputs,
        'final_last_output_to_estop_ms': end_ms - outputs[-1]['output_end_ms'],
        'logging_clipped_raw_gx': {'count': len(clipped), 'first_ms': clipped[0]['time_ms'] if clipped else None,
            'last_ms': clipped[-1]['time_ms'] if clipped else None,
            'limit_abs_dps': 327.67, 'interpretation': 'int16 log quantization clips raw gyro; does not establish sensor saturation or true rate above this limit'},
        'terminal_zero_accel_confidence': {'first_ms': confidence_tail[0]['time_ms'] if confidence_tail else None,
            'count': len(confidence_tail), 'max_norm_g': max(r['acc_norm_g'] for r in rows),
            'max_norm_time_ms': max(rows, key=lambda r: r['acc_norm_g'])['time_ms'],
            'interpretation': 'Acceleration becomes dynamically inconsistent with gravity; late attitude mainly relies on gyro prediction'},
        'final_foot_observations': {'left_missing_frames': missing_left, 'left_reacquired_frame': resumed_left,
            'last_frames': feet[-12:], 'left_missing_reasons': dict(Counter(f['left_reason'] for f in missing_left)),
            'control_observed_note': 'Only saved positive timestamps identify sampled observation; blank timestamps do not prove non-use'},
        'latency': {key: meta['control_latency'][key] for key in
            ['control_ready_age', 'decision_age', 'done_age', 'io_failures', 'io_max_us', 'overrun_detail']},
        'observations': ['The reported backward fall coincides with a large MEKF fore/aft roll excursion, while lateral pitch remains small at ESTOP.',
            'Fore/aft roll and body-X angular velocity grow before the final main pulse; the final pulse therefore cannot alone explain the onset.',
            'The last main pulse starts during an already large fore/aft excursion; lateral-only eligibility does not imply fore/aft stability.',
            'A left-marker dropout overlaps the beginning of the final excursion; its later relative angle is not backfilled into the missing interval.'],
        'cause_candidates_not_proven': ['Foot support geometry or contact evolution may alter fore/aft stability.',
            'Wheel/body/foot coupling or timing may contribute to the excursion.',
            'Missing foot observations can hide support-related state, but do not establish a missed ground-contact event.'],
        'limits': ['User supplies backward direction; no video determines physical fall onset, first contact, forward travel or ground plane.',
            'No calibrated fore/aft actuator response or ground-relative foot angle is reconstructed.',
            'Coincidence of a pulse and body motion is not causal identification.',
            'ESTOP timestamp has millisecond quantization and differs from a physical impact time.',
            'Large dynamic acceleration and gyro log clipping limit the terminal numerical interpretation.']}
    (path / 'backward_fall_review.json').write_text(json.dumps(review, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf8')
    for name, data in [('backward_fall_timeseries.csv', rows), ('backward_fall_foot_frames.csv', feet)]:
        with (path / name).open('w', encoding='utf8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(data[0])); writer.writeheader(); writer.writerows(data)
    plot(path, rows, feet, last_outputs, end_ms, meta['diagnostic_build_id'])
    print(json.dumps({'build': review['build'], 'end_ms': end_ms,
        'terminal_reason': meta['terminal_state']['last_error'],
        'final_fore_aft_roll_deg': rows[-1]['fore_aft_mekf_roll_deg'],
        'epoch_us': epoch, 'raw_gx_clipped_first_ms': review['logging_clipped_raw_gx']['first_ms'],
        'final_thresholds': final_thresholds}, indent=2))
    return review


def terminal_rows(rows):
    return [r for r in rows if r['state_id'] == 5]


def plot(path, rows, feet, pulses, end_ms, build):
    for detail in [False, True]:
        fig, axes = plt.subplots(6, 1, figsize=(13, 14), sharex=True, constrained_layout=True)
        t = [r['time_ms'] / 1000 for r in rows]
        axes[0].plot(t, [r['fore_aft_mekf_roll_deg'] for r in rows], color='tab:red', lw=1, label='fore/aft MEKF roll (body X)')
        axes[0].plot(t, [r['lateral_mekf_pitch_deg'] for r in rows], color='black', lw=.8, label='lateral MEKF pitch (body Y)')
        axes[0].axhline(90, color='tab:red', ls=':', lw=.7, label='90 deg raw Euler reference')
        axes[0].set_ylabel('Body Euler angle [deg]')
        for key, color, label in [('body_gyro_x_dps', 'tab:red', 'body gyro X (fore/aft)'),
                                  ('body_gyro_y_dps', 'tab:blue', 'body gyro Y (lateral)'),
                                  ('body_gyro_z_dps', 'tab:green', 'body gyro Z')]:
            axes[1].plot(t, [r[key] for r in rows], lw=.7, color=color, label=label)
        clip = [r for r in rows if r['gx_log_clipped']]
        if clip:
            axes[1].scatter([r['time_ms'] / 1000 for r in clip], [r['body_gyro_x_dps'] for r in clip],
                            s=9, marker='x', color='black', label='raw gx log clipped')
        axes[1].set_ylabel('Mapped body gyro [deg/s]')
        for side, color in [('left', 'tab:blue'), ('right', 'tab:orange')]:
            q = [f for f in feet if f['state_id'] == 3 and f['recognition_ready_ms'] is not None]
            axes[2].plot([f['recognition_ready_ms'] / 1000 for f in q],
                [f[side + '_deg'] if f[side + '_usable'] else math.nan for f in q],
                '.-', color=color, ms=3, lw=.7, label=side + ' relative foot, at recognition-ready')
        missing = [f for f in feet if f['state_id'] == 3 and not f['left_usable'] and f['recognition_ready_ms'] is not None]
        axes[2].scatter([f['recognition_ready_ms'] / 1000 for f in missing], [0] * len(missing),
                        color='tab:red', marker='x', s=20, label='left unavailable')
        axes[2].set_ylabel('Relative foot [deg]')
        axes[3].plot(t, [r['wheel_speed_rpm'] for r in rows], color='tab:purple', lw=.8, label='wheel speed telemetry')
        axes[3].set_ylabel('Wheel [rpm]')
        current_ax = axes[3].twinx()
        current_ax.plot(t, [r['motor_command_ma'] for r in rows], color='grey', alpha=.4, lw=.6, label='command [mA]')
        current_ax.plot(t, [r['current_telemetry_ma'] for r in rows], color='tab:green', alpha=.6, lw=.6, label='current telemetry [mA]')
        current_ax.set_ylabel('Current [mA]')
        current_ax.legend(loc='upper right', fontsize=7)
        axes[4].plot(t, [r['acc_norm_g'] for r in rows], color='tab:purple', lw=.8, label='acceleration norm [g]')
        axes[4].plot(t, [r['accel_confidence'] for r in rows], color='tab:green', lw=.8, label='MEKF accel confidence [0..1]')
        axes[4].set_ylabel('Accel norm / confidence')
        axes[5].plot(t, [r['gyro_heading_diagnostic_deg'] if r['gyro_heading_valid'] else math.nan for r in rows],
                     color='tab:olive', lw=.9, label='integrated gyro heading diagnostic (drifts; no ground reference)')
        axes[5].set_ylabel('Heading diagnostic [deg]'); axes[5].set_xlabel('Measurement time [s]')
        for ax in axes:
            if detail:
                for pulse in pulses:
                    if pulse['output_start_ms'] >= 15500:
                        ax.axvspan(pulse['output_start_ms'] / 1000, pulse['output_end_ms'] / 1000,
                                   color='tab:purple' if pulse['rescue'] else 'grey', alpha=.09)
            ax.axvline(end_ms / 1000, color='tab:red', ls='--', lw=.7)
            ax.grid(alpha=.2); ax.legend(loc='upper left', fontsize=7, ncol=2)
            ax.set_xlim(max(0, end_ms / 1000 - 1.8) if detail else 0, end_ms / 1000 + .03)
        if detail:
            axes[0].set_ylim(-12, 100)
        fig.suptitle(f'{build} / {path.name}: reported backward fall\n'
            'Red dashed: ESTOP. Foot gaps are not interpolated. Grey/purple bands: main/return software pulses.' if detail else
            f'{build} / {path.name}: reported backward fall\n'
            'MEKF body axes and relative foot observations; no ground-contact or causal inference')
        fig.savefig(path / ('backward_fall_detail.png' if detail else 'backward_fall_timeline.png'),
                    dpi=150, bbox_inches='tight', pad_inches=.12)
        plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_directory', type=Path)
    parser.add_argument('--user-observation',
                        default='User confirms the robot fell backward; whether the feet slipped is unknown. No new video.')
    args = parser.parse_args()
    analyze(args.converted_directory, args.user_observation)
