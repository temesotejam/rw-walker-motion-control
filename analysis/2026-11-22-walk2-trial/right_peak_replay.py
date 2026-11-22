"""Evaluate the existing right-arrival width limiter at 5 degrees on archived trajectories."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-20-ab-trials'))
import verify_arrival


def rows_for(folder):
    run = json.loads((folder/'header.json').read_bytes())['run_start_us']
    metadata = json.loads((folder/'metadata.json').read_bytes())
    events = metadata['energy_control_autonomous_zero_cross_events']
    peaks = metadata['energy_control_autonomous_peak_events']
    records = metadata['walking_energy']['records']
    for event in events:
        if (event['event_kind'] != 'energy_control_zero_cross' or event['physical_next_peak_side'] != 1 or
                event['startup_target_deg'] != 5.0):
            continue
        zero, previous = event['zero_cross_time_ms'], event['previous_peak_time_ms']
        matches = [w for w in records if w['peak_ms'] == previous and abs(w['time_ms']-zero) <= 8 and
                   w['side'] == 1 and not w['optional'] and not w['rescue']]
        if len(matches) != 1:
            raise ValueError(f'Expected one right main at {run}/{zero}')
        record, = matches
        if record['assisted_cycle'] or not record['valid']:
            continue
        powered = bool(event['output_executed'])
        if powered and not (record['planned_duration_completed'] and record['completion_valid']):
            continue
        width = event['pulse_width_ms'] if powered else 0
        following = [p for p in peaks if p['peak_time_ms'] > zero]
        peak = following[0] if following else None
        # Inspect the original path only for outcome scoring; the limiter below
        # takes only start-of-main state, target, electrical model and width.
        scoreable = bool(peak and peak['physical_peak_side'] == 1 and peak['peak_time_ms'] - zero <= 750)
        if scoreable and any(e['output_executed'] and e is not event and
                             previous < e['zero_cross_time_ms'] < peak['peak_time_ms'] for e in events):
            scoreable = False
        yield dict(run=run, zero_ms=zero, rate_dps=record['start_rate_dps'],
                   previous_peak_deg=event['previous_peak_amplitude_deg'], target_deg=5.,
                   i0_ma=record['start_i0_ma'], goal_ma=record['start_goal_ma'], tau_s=record['start_tau_s'],
                   original_ms=width, right_peak_deg=peak['peak_amplitude_deg'] if scoreable else None,
                   continued=bool(len(following)>1 and following[1]['physical_peak_side'] == -1 and
                                  following[1]['peak_time_ms'] - peak['peak_time_ms'] <= 1000) if scoreable else None)


def audit(folders):
    rows = [r for folder in folders for r in rows_for(folder)]
    choices = verify_arrival.firmware_choices(rows)
    counts = {'fit_8_runs': Counter(), 'other_runs': Counter()}
    for row, (eligible, width) in zip(rows, choices):
        row['eligible'] = eligible
        row['candidate_ms'] = width
        counter = counts['fit_8_runs' if row['run'] in verify_arrival.cal.TRAIN else 'other_runs']
        counter['ordinary_5deg'] += 1
        if not row['original_ms'] or not eligible:
            continue
        counter['eligible_powered'] += 1
        if width >= row['original_ms']:
            continue
        counter['cut'] += 1
        peak = row['right_peak_deg']
        if peak is None:
            counter['cut_missing_peak'] += 1
        else:
            counter['cut_above' if peak > row['target_deg'] else 'cut_at_or_below'] += 1
            counter['cut_returned' if row['continued'] else 'cut_without_next_left'] += 1
    return dict(roots=len(folders), counts=counts, rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    args = parser.parse_args()
    folders = {p.resolve() for root in args.roots for p in root.iterdir() if (p/'metadata.json').is_file()}
    ids = [json.loads((p/'header.json').read_bytes())['run_start_us'] for p in folders]
    if len(ids) != len(set(ids)):
        parser.error('Duplicate recording across roots')
    print(json.dumps(audit(sorted(folders)), indent=2, allow_nan=False))
