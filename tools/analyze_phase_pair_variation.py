#!/usr/bin/env python3
"""Describe adjacent two-cycle differences in previously exported RWLOG data.

Input directories each contain metadata.json and foot_angles.csv as generated
by the project's RWLOG export tools. This is an observational calculation;
it does not estimate a controlled intervention effect or measurement noise.

Example (Windows):
  python tools/analyze_phase_pair_variation.py C:/path/run139 C:/path/run175
"""

import argparse
import csv
import json
import statistics
from pathlib import Path


def analyze(directory, start_ms, end_ms, max_delta_ms):
    with (directory / 'metadata.json').open(encoding='utf-8') as stream:
        metadata = json.load(stream)
    frames = []
    with (directory / 'foot_angles.csv').open(encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            if (row['state_id'] == '3' and row['timestamp_valid'] == 'True'
                    and row['left_valid'] == 'True' and row['right_valid'] == 'True'
                    and int(row['measurement_time_us']) >= 0):
                frames.append((int(row['measurement_time_us']) / 1000,
                               float(row['left_deg']), float(row['right_deg'])))
    if not frames:
        raise ValueError(f'{directory}: no eligible frames')

    matched = []
    for peak in metadata['energy_control_autonomous_peak_events']:
        time = peak['peak_time_ms']
        frame = min(frames, key=lambda item: abs(item[0] - time))
        if abs(frame[0] - time) <= max_delta_ms:
            matched.append((time, peak['physical_peak_side'], frame[1], frame[2], frame[0]))

    events = metadata['energy_control_autonomous_zero_cross_events']
    cycles = []
    for a, b, c in zip(matched, matched[1:], matched[2:]):
        if not (a[1] == -1 and b[1] == 1 and c[1] == -1
                and start_ms < a[0] and c[0] < end_ms
                and len({a[4], b[4], c[4]}) == 3):
            continue
        left = b[2] - (a[2] + c[2]) / 2
        right = (a[3] + c[3]) / 2 - b[3]
        in_cycle = [e for e in events if e.get('event_kind') == 'energy_control_zero_cross'
                    and a[0] < e.get('zero_cross_time_ms', 0) < c[0]]
        positive = [e for e in in_cycle if e['physical_next_peak_side'] == 1]
        negative = [e for e in in_cycle if e['physical_next_peak_side'] == -1]
        cycles.append((a[0], c[0], left - right, positive, negative))

    all_windows, stable_windows, exact_windows = [], [], []
    for i in range(len(cycles) - 3):
        group = cycles[i:i + 4]
        if any(group[j][1] != group[j + 1][0] for j in range(3)):
            continue
        difference = (group[2][2] + group[3][2] - group[0][2] - group[1][2]) / 2
        all_windows.append(difference)
        if any(len(c[3]) != 1 or len(c[4]) != 1 for c in group):
            continue
        positive, negative = [c[3][0] for c in group], [c[4][0] for c in group]
        if not all(e.get('valid') and e.get('output_executed') for e in positive + negative):
            continue
        if any(len({(e['command_current_mA'], e['work_control_branch']) for e in side}) != 1
               for side in (positive, negative)):
            continue
        widths = [[e['pulse_width_ms'] for e in side] for side in (positive, negative)]
        if all(max(w) - min(w) <= 5 for w in widths):
            stable_windows.append(difference)
        if all(len(set(w)) == 1 for w in widths):
            exact_windows.append(difference)

    def report(label, values):
        magnitudes = sorted(abs(v) for v in values)
        if not values:
            print(f'  {label}: n=0')
            return
        # Match the prior calculation: nearest observation for the 90th percentile.
        q90 = magnitudes[round((len(magnitudes) - 1) * .9)]
        print(f'  {label}: n={len(values)}, abs_median={statistics.median(magnitudes):.3f} deg, '
              f'abs_q90={q90:.3f} deg, abs_max={max(magnitudes):.3f} deg, '
              f'delta<=-0.10={sum(v <= -.1 for v in values)}, '
              f'delta>=+0.10={sum(v >= .1 for v in values)}')

    print(f'{directory}: {len(cycles)} matched cycles')
    report('all contiguous windows', all_windows)
    report('both branches/currents equal, each width within 5 ms', stable_windows)
    report('both branches/currents and widths equal', exact_windows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--start-ms', type=int, default=5000)
    parser.add_argument('--end-ms', type=int, default=20000)
    parser.add_argument('--max-delta-ms', type=int, default=100)
    args = parser.parse_args()
    for directory in args.directories:
        analyze(directory, args.start_ms, args.end_ms, args.max_delta_ms)
    print('Descriptive only: windows overlap; input and state changes are not isolated.')


if __name__ == '__main__':
    main()
