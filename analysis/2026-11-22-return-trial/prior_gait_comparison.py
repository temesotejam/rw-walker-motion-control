"""Compare the measured initial posture and the first leftward returns of walking trials."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'analysis/2026-11-20-smallstep-baseline'))
sys.path.insert(0, str(ROOT / 'tools'))
from summarize_start import summarize
from score_observed_transfers import gyro_samples, measurement_epoch
from pulse_release import charge, trace


def first_passive_return(times, rates, angles, peak, first_output, initial_angle):
    """Compare the first return at fixed travel and at the pre-kick angle, before the next output."""
    targets = [('one_degree_from_peak', peak['peak_amplitude_deg']-1.0),
               ('prestart_angle', initial_angle), ('mechanical_zero', 0.0)]
    result = {name: None for name, _ in targets}
    for i in range(1, len(times)):
        if times[i] >= first_output:
            break
        if (times[i-1] < peak['peak_time_ms'] or times[i]-times[i-1]>10
                or angles[i-1] is None or angles[i] is None):
            continue
        hi, lo = angles[i-1], angles[i]
        for name, target in targets:
            if target is None or result[name] is not None or not hi > target >= lo:
                continue
            f = (hi-target)/(hi-lo)
            result[name] = dict(time_ms=round(times[i-1]+f*(times[i]-times[i-1]), 2),
                                speed_dps=round(-(rates[i-1]+f*(rates[i]-rates[i-1])), 2))
    return result


def acknowledged_pulse(event, records, signals):
    """Match a recorded walking pulse to the driver's signed, acknowledged command edges."""
    side = event['physical_next_peak_side']
    commands = []
    for time_ms, value in signals['command']:
        if not commands or value != commands[-1][1]:
            commands.append((time_ms, value))
    start_ms = event['pulse_start_ms']
    end_ms = event['pulse_end_ms']
    matches = [(on, off, current) for (on, current), (off, zero) in zip(commands, commands[1:])
               if current * side > 0 and zero == 0 and abs(on-start_ms) <= 8 and abs(off-end_ms) <= 8]
    if len(matches) != 1:
        raise ValueError(f'No unique acknowledged pulse at {start_ms} ms')
    on, off, current = matches[0]
    work = [r for r in records if r['executed'] and r['side'] == side and
            r['output_start_ms'] == start_ms and abs(r['output_end_ms']-off) <= 8]
    if len(work) != 1:
        raise ValueError(f'No unique work record at {start_ms} ms')
    predicted = work[0].get('elapsed_work_pred_j')
    quiet_tail = not any(value and off < t < off+100 for t, value in commands)
    integral = charge(signals['current'], on, off+100) if quiet_tail else None
    return dict(event=event['event_kind'], on_ms=on, off_ms=off, command_mA=current,
                commanded_width_ms=event['pulse_width_ms'], acknowledged_duration_ms=round(off-on, 3),
                charge_through_off_plus_100ms_mA_s=round(integral, 3) if integral is not None else None,
                elapsed_work_pred_uJ=round(predicted*1e6, 3) if predicted is not None else None)


def first_rightward_return(folder, events, records, peak, times, rates, angles):
    """Use only samples strictly before the first rightward command and after the initial left pulse."""
    main = [e for e in events if e['output_executed'] and e['event_kind'] != 'strong_start_kick']
    if len(main) < 2 or [e['physical_next_peak_side'] for e in main[:2]] != [-1, 1]:
        return None
    if peak['physical_peak_side'] != -1:
        return None
    _, signals = trace(folder)
    left, right = [acknowledged_pulse(e, records, signals) for e in main[:2]]
    lower = max(left['off_ms'], peak['peak_time_ms'])
    upper = right['on_ms']
    if not lower < upper:
        return None
    targets = dict(one_degree_from_left_peak=1.0 - peak['peak_amplitude_deg'], minus_one_deg=-1.0)
    result = {name: None for name in targets}
    for i in range(1, len(times)):
        if times[i] >= upper:
            break
        if times[i-1] < lower or times[i]-times[i-1] > 10:
            continue
        before, after = angles[i-1:i+1]
        if before is None or after is None:
            continue
        for name, target in targets.items():
            if result[name] is not None or not before < target <= after:
                continue
            f = (target-before)/(after-before)
            result[name] = dict(time_ms=round(times[i-1]+f*(times[i]-times[i-1]), 2),
                                speed_dps=round(rates[i-1]+f*(rates[i]-rates[i-1]), 2),
                                bracket_ms=[times[i-1], times[i]])
    return dict(left_input=left, right_input=right, rightward_before_right_input=result)


def compare(folder):
    start = summarize(folder)
    meta = json.loads((folder / 'metadata.json').read_bytes())
    epoch = measurement_epoch(meta)
    with (folder / 'timeseries.csv').open(encoding='utf-8-sig', newline='') as stream:
        times, rates, angles = gyro_samples(meta, csv.DictReader(stream), epoch)
    # Use the measured leftward crossing even when an early pulse consumed the
    # firmware ZERO event. Limit each right-to-left peak interval to one crossing.
    speeds = []
    peaks = meta['energy_control_autonomous_peak_events']
    for a, b in zip(peaks, peaks[1:]):
        if a['physical_peak_side'] != 1 or b['physical_peak_side'] != -1:
            continue
        lo, hi = a['peak_time_ms'], b['peak_time_ms']
        for k, t in enumerate(times[1:], 1):
            if t > min(4000, hi):
                break
            if times[k-1] < max(1000, lo) or t-times[k-1] > 10:
                continue
            before, after = angles[k-1:k+1]
            if before is not None and after is not None and before > 0 >= after:
                fraction = before / (before-after)
                speeds.append(-(rates[k-1] + fraction*(rates[k]-rates[k-1])))
                break
    events = meta['energy_control_autonomous_zero_cross_events']
    first_outputs = [e for e in events if e['output_executed'] and e['pulse_start_ms'] <= peaks[0]['peak_time_ms']]
    if (peaks[0]['physical_peak_side'] != 1 or len(first_outputs) != 1
            or first_outputs[0]['event_kind'] != 'strong_start_kick'):
        raise ValueError(f'{folder}: expected a right first peak with only the kick before it')
    first = first_outputs[0]
    next_output = min((e['pulse_start_ms'] for e in events if e['output_executed'] and
                       e['event_kind']!='strong_start_kick'), default=float('inf'))
    initial = start['prestart_roll_median_deg']
    first_return = first_passive_return(times, rates, angles, peaks[0], next_output, initial)
    right_return = (first_rightward_return(folder, events, meta['walking_energy']['records'], peaks[1],
                                           times, rates, angles) if len(peaks) >= 2 else None)
    right_limits = [(r['time_ms'], r['right_arrival_base_ms'], r['guarded_ms'])
                    for r in meta['walking_energy']['records'] if r.get('right_arrival_eligible') and
                    r['executed'] and r['guarded_ms'] < r['right_arrival_base_ms']]
    return dict(run=folder.name, profile=meta.get('walking_profile',meta['diagnostic_build_id']),
                prestart_abs_deg=initial, first_return_before_next_input=first_return,
                first_left_input_and_right_return=right_return,
                first_right_peak_deg=start['first_peak_signed_deg'],
                first_zero_ms=start['first_center_crossing_interval_ms'],
                last_peak_ms=start['last_accepted_body_peak_ms'], kick_mA=first['command_current_mA'],
                kick_ms=first['pulse_width_ms'], left_crossings_1_to_4s_deg_s=speeds,
                actual_right_limit_time_base_final_ms=right_limits,
                median_left_crossing_deg_s=median(speeds) if speeds else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path, help='One converted RWLOG directory per trial')
    args = parser.parse_args()
    print(json.dumps([compare(f) for f in args.folders], indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
