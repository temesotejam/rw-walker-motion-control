"""Read a converted RWLOG and compare startup motion and actual command selection."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median


def first_zero_window(rows, peaks):
    # An early main pulse can consume the firmware event for this half-cycle.
    # Independently bracket the centre using the recorded detector angle. Its
    # two decimal places cannot distinguish a rounded zero from either sign.
    if not peaks:
        return None
    peak = peaks[0]
    side = peak['physical_peak_side']
    start = peak['peak_time_ms']
    end = peaks[1]['peak_time_ms'] if len(peaks) > 1 else float('inf')
    last_same_side = None
    for row in rows:
        t = int(row['t_test_ms'])
        if t < start or not row['pitch_mekf_detector_relative_deg']:
            continue
        if t > end:
            break
        oriented_angle = side * float(row['pitch_mekf_detector_relative_deg'])
        if oriented_angle > 0:
            last_same_side = t
        elif oriented_angle < 0:
            if last_same_side is not None:
                return [last_same_side, t]
            break
    return None


def summarize(folder):
    metadata = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    with (folder / 'timeseries.csv').open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    first = next((i for i, r in enumerate(rows) if r['state_id'] == '3'), None)
    if first is None:
        raise ValueError(f'{folder} has no measurement interval')
    origin = int(rows[first]['body_rate_sample_time_us'] or 0)
    before = {}
    for row in reversed(rows[:first]):
        if row['state_id'] != '6':  # Countdown immediately preceding this run.
            break
        stamp = int(row['body_rate_sample_time_us'] or 0)
        if not stamp or not origin:
            continue
        if ((origin - stamp) & 0xffffffff) > 250_000:
            break
        if row['pitch_mekf_abs_deg'] and stamp != origin:
            before[stamp] = float(row['pitch_mekf_abs_deg'])
    measurement = [r for r in rows[first:] if r['state_id'] == '3']
    peaks = metadata['energy_control_autonomous_peak_events']
    events = metadata['energy_control_autonomous_zero_cross_events']
    widths = {}
    for event in events:
        if (event['output_executed'] and event['event_kind'] != 'strong_start_kick'
                and 0 <= event['pulse_start_ms'] < 5000):
            widths.setdefault(event['event_kind'], []).append(event['pulse_width_ms'])
    return {
        'trial': folder.name,
        'build': metadata.get('diagnostic_build_id'),
        'walking_profile': metadata.get('walking_profile'),
        'prestart_roll_median_deg': median(before.values()) if before else None,
        'first_peak_ms': peaks[0]['peak_time_ms'] if peaks else None,
        'first_peak_signed_deg': peaks[0]['physical_peak_side'] * peaks[0]['peak_amplitude_deg'] if peaks else None,
        'first_center_crossing_interval_ms': first_zero_window(measurement, peaks),
        'first_5s_executed_command_widths_ms': widths,
        'last_accepted_body_peak_ms': peaks[-1]['peak_time_ms'] if peaks else None,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', nargs='+', type=Path, help='Directories from convert_rwlog_to_csv.py')
    args = parser.parse_args()
    print(json.dumps([summarize(f) for f in args.folders], indent=2, ensure_ascii=False))
