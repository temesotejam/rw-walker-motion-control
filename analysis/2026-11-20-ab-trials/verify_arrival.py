"""Audit recorded right-peak predictions, width sensitivity and the following left peak.

Pass one folder containing the ten converted recordings, or each recording folder
explicitly. Predictions always use the command recorded in the original run.
"""
import argparse
import csv
import json
import math
import subprocess
import tempfile
from collections import defaultdict
from pathlib import Path

import numpy as np

import calibration as cal


def recording_folders(paths):
    if len(paths) == 1 and not (paths[0] / 'header.json').is_file():
        paths = sorted(p for p in paths[0].iterdir() if (p / 'header.json').is_file())
    folders = {}
    for path in paths:
        run = json.loads((path / 'header.json').read_text(encoding='utf-8'))['run_start_us']
        if run not in cal.TRAIN | cal.HOLD:
            continue
        if run in folders:
            raise ValueError(f'Duplicate {run}; pass the ten individual recording folders')
        folders[run] = path
    if set(folders) != cal.TRAIN | cal.HOLD:
        raise ValueError(f'Missing recordings: {(cal.TRAIN | cal.HOLD) - set(folders)}')
    return folders


def firmware_choices(rows):
    fields = 'rate_dps previous_peak_deg target_deg i0_ma goal_ma tau_s original_ms'.split()
    payload = ''.join(' '.join(str(row[k]) for k in fields) + '\n' for row in rows)
    with tempfile.TemporaryDirectory() as directory:
        exe = Path(directory) / 'verify_arrival.exe'
        subprocess.run(['g++', '-std=c++17', '-O2', '-I', cal.HERE.parents[1] / 'src',
                        cal.HERE / 'calibration.cpp', '-o', exe], check=True)
        output = subprocess.run([exe], input=payload, capture_output=True, text=True, check=True)
    lines = output.stdout.splitlines()
    if len(lines) != len(rows):
        raise ValueError('Incomplete firmware replay')
    return [(bool(int(line.split()[0])), int(line.split()[1])) for line in lines]


def compare_errors(label, triples):
    # Each tuple is (run, old signed error, new signed error), in degrees.
    if not triples:
        return
    runs, old, new = map(np.asarray, zip(*triples))
    print(f'{label}: runs={len(set(runs))} n={len(runs)} '
          f'MAE={abs(old).mean():.3f}/{abs(new).mean():.3f} '
          f'new_closer={sum(abs(new) < abs(old))} '
          f'bias={old.mean():+.3f}/{new.mean():+.3f} '
          f'max_abs={max(abs(old)):.3f}/{max(abs(new)):.3f}')


def width_for_fit(row, coefficients):
    # Same model and recorded current conditions; exhaustively check the finite
    # integer widths so the sensitivity result is independent of the C++ solver.
    widths = np.arange(row['original_ms'] + 1)
    duration_s = widths / 1000.
    tau = row['tau_s']
    charge = row['goal_ma'] * duration_s + (row['i0_ma'] - row['goal_ma']) * tau * -np.expm1(-duration_s / tau)
    omega = math.radians(row['rate_dps'])
    energy = coefficients[0] + coefficients[1] * omega ** 2 + coefficients[2] * omega * charge
    return int(np.argmin(np.where(charge >= 0, abs(energy - cal.potential(row['target_deg'])), np.inf)))


def audit_predictions(rows):
    train = [row for row in rows if row['run'] in cal.TRAIN and 0 <= row['q_mas'] <= cal.Q_LIMIT]
    full_fit = cal.fit(train)
    omitted = {run: cal.fit([r for r in train if r['run'] != run]) for run in sorted(cal.TRAIN)}
    # Fit the same nonnegative model with its charge term removed. This checks
    # whether Q contributes predictive information beyond crossing speed.
    unpowered = [dict(row, q_mas=0.) for row in train]
    speed_only = {run: cal.fit([r for r in unpowered if r['run'] != run]) for run in sorted(cal.TRAIN)}
    speed_only_full = cal.fit(unpowered)
    print('Charge removed, full prior fit [intercept, speed squared, charge interaction]:', speed_only_full)
    held = [row for row in rows if row['run'] in cal.HOLD]
    compared = defaultdict(list)
    print('Signed error = prediction minus measured peak. Metrics in degrees, old/new.')
    for row, (eligible, active_width) in zip(train + held, firmware_choices(train + held)):
        run = row['run']
        group = 'prior LORO' if run in omitted else 'recent'
        fit = omitted.get(run, full_fit)
        new_peak = cal.angle(fit @ cal.features(row))
        errors = (run, row['old_predicted_deg'] - row['next_peak_deg'], new_peak - row['next_peak_deg'])
        speed_fit = speed_only.get(run, speed_only_full)
        without_charge = cal.angle(speed_fit @ cal.features(dict(row, q_mas=0.)))
        ablated_errors = (run, without_charge - row['next_peak_deg'], errors[-1])
        powered = row['original_ms'] > 0
        compared[group + ' all'].append(errors)
        compared[group + ' all, charge removed vs full'].append(ablated_errors)
        compared[group + (' powered' if powered else ' coast')].append(errors)
        if powered and eligible and row['target_deg'] == 4.5:
            compared[group + ' eligible startup powered'].append(errors)
            compared[f'{group} eligible startup powered run {run}'].append(errors)
            compared[group + ' eligible startup powered, charge removed vs full'].append(ablated_errors)
        if run in cal.HOLD and powered and eligible and row['target_deg'] == 4.5:
            choices = [width_for_fit(row, c) for c in omitted.values()]
            print(f'{run} at {row["zero_ms"]} ms: recorded {row["original_ms"]} ms; '
                  f'firmware {active_width} ms; omitting each prior run {min(choices)}..{max(choices)} ms')
    for label, triples in compared.items():
        compare_errors(label, triples)


def audit_returns(folders):
    # A subsequent opposite body peak is measurable; a completed floor step is not.
    groups = defaultdict(list)
    for run, folder in folders.items():
        data = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
        peaks = data['energy_control_autonomous_peak_events']
        with (folder / 'timeseries.csv').open(encoding='utf-8', newline='') as f:
            end_ms = max(int(s['t_test_ms']) for s in csv.DictReader(f) if s['state_id'] == '3')
        for index, peak in enumerate(peaks):
            t = peak['peak_time_ms']
            if peak['physical_peak_side'] != 1 or t + 1000 > end_ms:
                continue
            following = peaks[index + 1] if index + 1 < len(peaks) else None
            returned = bool(following and following['physical_peak_side'] == -1
                            and following['peak_time_ms'] <= t + 1000)
            side_group = 'prior' if run in cal.TRAIN else 'recent'
            height = '>=7deg' if peak['peak_amplitude_deg'] >= 7 else '<7deg'
            entry = (run, t, peak['peak_amplitude_deg'], returned)
            groups[(side_group, 'all', height)].append(entry)
            if peak['target_peak_deg'] == 4.5:
                groups[(side_group, 'startup', height)].append(entry)
    for key, group in sorted(groups.items()):
        missed = [(run, ms, round(deg, 3)) for run, ms, deg, ok in group if not ok]
        print(*key, f'next left peak within 1 s {len(group)-len(missed)}/{len(group)}; missing={missed}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', type=Path, nargs='+')
    folders = recording_folders(parser.parse_args().recordings)
    extracted = cal.extract(folders.values())
    with (cal.HERE / 'right_arrivals.csv').open(encoding='utf-8', newline='') as table:
        saved = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(table)]
    if extracted != saved:
        raise ValueError('Extracted right-arrival rows differ from the checked-in table')
    print(f'{len(extracted)} extracted rows match the saved input.')
    audit_predictions(extracted)
    audit_returns(folders)
