#!/usr/bin/env python3
"""Replay saved RWLOG inputs into the real phase observer/cap implementation.

Example: python tools/replay_phase_cap.py --exe /tmp/test_phase_cap_replay RUN.rwlog
The RWLOG remains in its original location; no data is copied into this repo.
Replaying recorded states stops on the first change; it cannot predict future
foot motion, body peaks, gyro speed or safety after a changed motor command.
"""
import argparse
import json
import math
import struct
import subprocess
from pathlib import Path

from convert_rwlog_to_csv import expand_tables, parse_header, sample_format_for_version, verify_crc


def load_log(path):
    data = path.read_bytes()
    h = parse_header(data)
    if h['format_version'] < 53 or h['format_version'] > 56:
        raise ValueError('supported metadata/telemetry format is RWLOG v53-v56')
    if h['crc_offset'] + 4 != len(data) or not verify_crc(data, h):
        raise ValueError('RWLOG CRC/length invalid')
    if h['header_size'] + h['metadata_json_size'] != h['samples_offset']:
        raise ValueError('invalid metadata offsets')
    if h['samples_offset'] + h['sample_count'] * h['log_sample_size'] > h['crc_offset']:
        raise ValueError('invalid sample offsets')
    if h['log_sample_size'] != struct.calcsize(sample_format_for_version(h['format_version'])):
        raise ValueError('unexpected sample format')
    m = expand_tables(json.loads(data[h['header_size']:h['samples_offset']].decode('utf-8')))
    if m.get('metadata_event_detail_truncated'):
        raise ValueError('event detail truncated; refusing to infer missing history')
    for key in ('foot_frames', 'energy_control_autonomous_peak_events', 'energy_control_autonomous_zero_cross_events'):
        if not isinstance(m.get(key), list) or not m[key]:
            raise ValueError(f'missing {key}')
    return h, m


def number(value, default=0):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else default


def events_for(h, m, extra_delay, poll_delay):
    # frame measurement_time_us is in the motor-run epoch. delivered_us is the
    # camera-frame acquisition time, before detection/publishing, NOT the
    # consumer notification time. The recorded processing_us covers detection
    # and preview; publication overhead remains unknown. Probe 0..25ms extra
    # for the consumer's periodic snapshot poll. The precise poll phase is not
    # logged and the actual snapshot can skip frames: this is an approximation.
    sync_ms = m.get('led_sync_start_total_ms')
    if not isinstance(sync_ms, int) or sync_ms < 0:
        raise ValueError('missing recorded START sync duration')
    events = []
    for f in m['foot_frames']:
        if f.get('run_id') != h.get('run_id'):
            continue
        exposed = number(f.get('measurement_time_us'), -1)
        frame_us = number(f.get('frame_us'), -1)
        delivered_us = number(f.get('delivered_us'), -1)
        processing = number(f.get('processing_us'), -1)
        if not f.get('timestamp_valid') or exposed < 0 or frame_us < 0 or delivered_us < frame_us or processing < 0:
            continue
        delivered = exposed + (delivered_us - frame_us) + processing + poll_delay*1000
        # The same C++ rawFrame gate and fixed calibrated angle as runtime.
        # Foot angle validity depends on boot zero and is not marker validity.
        frame_ok = bool(f.get('frame_valid') and f.get('state_id') == 3)
        lv = frame_ok and f.get('left_reason') == 'detected'
        rv = frame_ok and f.get('right_reason') == 'detected'
        lx, rx = number(f.get('left_x')), number(f.get('right_x'))
        if not isinstance(f.get('left_x'),(int,float)) or not math.isfinite(f['left_x']): lv = False
        if not isinstance(f.get('right_x'),(int,float)) or not math.isfinite(f['right_x']): rv = False
        events.append((int(delivered / 1000), 0,
                       f"B {int(delivered/1000)} {int(exposed/1000)} {int(lv)} {int(rv)} {lx} {rx}"))
    for p in m['energy_control_autonomous_peak_events']:
        t = p.get('peak_time_ms'); notified = p.get('peak_telemetry_capture_time_ms')
        side = p.get('physical_peak_side'); amplitude = p.get('peak_amplitude_deg')
        if not isinstance(t, int) or not isinstance(notified, int) or notified < t or side not in (-1, 1):
            raise ValueError('missing or inconsistent measured peak notification')
        notified += extra_delay
        events.append((notified, 1, f'P {notified} {t} {side} {number(amplitude)}'))
    for z in m['energy_control_autonomous_zero_cross_events']:
        if z.get('event_kind') == 'strong_start_kick':
            continue
        t, side = z.get('zero_cross_time_ms'), z.get('physical_next_peak_side')
        if not isinstance(t, int) or side not in (-1, 1):
            raise ValueError('invalid zero-cross event')
        target = z.get('target_energy_j')
        if not isinstance(target, (int, float)) or not math.isfinite(target) or target <= 0:
            # A missing target is deliberately passed as invalid, as at runtime.
            target = 0
        if side == 1:
            # Later phase-cap versions explicitly record the ordinary choice.
            # Reusing the already-restricted solver width as the baseline
            # would silently erase a historical intervention.
            phase_baseline = number(z.get('foot_phase_baseline_ms'))
            # v1.0.58's low-peak selector logs its ordinary pre-intervention
            # width separately. If it differs from history the C++ replay stops.
            baseline = (phase_baseline if phase_baseline > 0 else
                        number(z.get('low_peak_base_width_ms')) if z.get('low_peak_selected') else
                        number(z.get('solver_selected_integer_width_ms')))
            if z.get('mpc_alternate'):
                events.append((t, 2, f'X {t}'))
                continue
        else:
            baseline = number(z.get('solver_selected_integer_width_ms'))
        ma, sign = int(number(z.get('command_current_mA'))), int(number(z.get('q_command_direction')))
        width = int(number(z.get('pulse_width_ms')))
        speed_fresh = z.get('pre_input_wheel_speed_valid') and number(z.get('pre_input_wheel_speed_age_us'), 99999999) <= 30000
        voltage = number(z.get('vbat_mV'))
        allowed = bool(z.get('valid') and z.get('work_control_branch') == 5 and speed_fresh and 7000 <= voltage <= 8120)
        cap = int(number(z.get('work_brake_cap_ms')))
        if side == 1 and z.get('foot_phase_selected'):
            # This field reflects the selected restricted candidate, not the
            # pre-intervention limit. The ordinary choice proves at least its
            # own width was allowed; the precise larger cap is not logged.
            cap = max(cap, int(baseline))
        if z.get('low_peak_selected') and isinstance(z.get('low_peak_base_prediction_mj'), (int, float)):
            base_prediction = number(z.get('low_peak_base_prediction_mj'))
        else:
            base_prediction = number(z.get('predicted_energy_j')) * 1000 - number(z.get('low_peak_applied_bias_mj'))
        model = [number(z.get(k)) for k in ('startup_target_deg', 'zero_cross_abs_rate_dps',
                 'pre_input_wheel_speed_rpm', 'vbat_mV', 'previous_peak_amplitude_deg', 'i0_estimated_mA',
                 'predicted_current_goal_mA', 'predicted_current_rise_tau_s')]
        events.append((t, 2, f"Z {t} {side} {int(baseline)} {ma} {sign} {cap} {int(allowed)} {target*1000} "
                              f"{int(bool(z.get('valid')))} {int(bool(z.get('output_executed')))} "
                              f"{int(number(z.get('reason_code'),-1))} {int(number(z.get('work_control_branch')))} {ma} {sign} {width} "
                              f"{int(bool(speed_fresh))} {int(bool(z.get('side_mismatch_diagnostic')))} "
                              f"{number(z.get('target_peak_deg'))} "
                              + ' '.join(map(str, model + [base_prediction]))))
    events.sort(key=lambda e: (e[0], e[1]))
    return events


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('rwlog', type=Path)
    p.add_argument('--exe', required=True, type=Path, help='compiled test_phase_cap_replay executable')
    p.add_argument('--peak-extra-delay-ms', type=int, nargs='+', default=[0, 25, 75])
    p.add_argument('--poll-extra-delay-ms', type=int, nargs='+', default=[0, 25],
                   help='camera consumer polling delay, after recorded processing')
    args = p.parse_args()
    h, m = load_log(args.rwlog)
    for delay in args.peak_extra_delay_ms:
        if delay < 0:
            raise ValueError('extra peak notification delay must be non-negative')
        for poll_delay in args.poll_extra_delay_ms:
            if poll_delay < 0:
                raise ValueError('poll delay must be non-negative')
            events = events_for(h, m, delay, poll_delay)
            stream = f"R {h['run_id']}\n" + '\n'.join(e[2] for e in events) + '\n'
            result = subprocess.run([str(args.exe.resolve())], input=stream, text=True, capture_output=True, check=True)
            print(f'peak_extra_delay_ms={delay} poll_extra_delay_ms={poll_delay} {result.stdout.strip()}')
    print('CRC valid; each replay stops before any changed history; candidate cap is not an executed motor command.')


if __name__ == '__main__':
    main()
