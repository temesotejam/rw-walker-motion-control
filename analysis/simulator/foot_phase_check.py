"""Check whether adding both measured foot rates improves the ZERO-arrival predictor."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

import numpy as np

from foot_kinematics import FootHistory, enabled
import return_model


SECTIONS = (4.5, 3., 2.)


def camera_checks(folder, history, episodes):
    """Score at the exposure time of a later valid camera image, revealed only for scoring."""
    peaks = sorted(episodes, key=lambda p: p['peak_ms'])
    peak_times = [e['peak_ms'] for e in peaks]
    checks = []
    for frame in return_model.read_rows(folder, 'foot_angles'):
        if frame['state_id'] != '3' or not all(enabled(frame[k]) for k in ('timestamp_valid', 'frame_valid', 'zero_ready')):
            continue
        at_us = int(frame['measurement_time_us'])
        t = at_us / 1000
        peak = bisect_right(peak_times, t) - 1
        # Offline score only the measured first inward returns. Neither endpoint nor
        # later camera data are passed to the packet or angular-rate predictor.
        if peak < 0 or not peaks[peak]['peak_available_ms'] <= t <= peaks[peak]['end_ms']:
            continue
        state = history.at(t)
        for side in ('left', 'right'):
            sample = getattr(state, side)
            if (not sample or sample.rate_dps is None or sample.age_ms <= 0 or
                    not enabled(frame[side + '_valid']) or not enabled(frame[side + '_control_range_ok'])):
                continue
            actual = float(frame[side + '_deg'])
            issued = sample.published_est_deg
            if issued is None or not math.isfinite(actual):
                continue
            checks.append((side, sample.angle_deg, issued,
                           sample.rate_dps * sample.age_ms / 1000, actual))
    return checks


def camera_score(checks, ids):
    """Learn one nonnegative shrinkage of the extrapolation on other reference runs."""
    errors = {side: [] for side in ('left', 'right')}
    gains = {}
    for rid in ids:
        train_ids = [r for r in return_model.PRIOR if r != rid]
        # Equal weight per training recording. A single shared coefficient for both feet.
        numerator, denominator = 0., 0.
        for r in train_ids:
            data = checks[r]
            if data:
                numerator += sum(extra * (actual - base) for _, base, _, extra, actual in data) / len(data)
                denominator += sum(extra * extra for _, _, _, extra, _ in data) / len(data)
        gain = max(0., min(1., numerator / denominator)) if denominator else 0.
        gains[str(rid)] = round(gain, 3)
        for side, base, issued, extra, actual in checks[rid]:
            errors[side].append([abs(base - actual), abs(issued - actual),
                                 abs(base + gain * extra - actual)])
    scores = {}
    for side, data in {**errors, 'both': errors['left'] + errors['right']}.items():
        if data:
            mae = np.mean(data, axis=0)
            scores[side] = {'n': len(data), 'hold_mae_deg': round(float(mae[0]), 4),
                            'firmware_mae_deg': round(float(mae[1]), 4),
                            'shrunk_rate_mae_deg': round(float(mae[2]), 4)}
    return {'scores': scores, 'fitted_gain_per_test_run': gains}


def feature_row(obs, feet):
    s = obs.state
    pair = feet.at(s.time_ms)
    if (not pair.left or not pair.right or pair.left.rate_dps is None or pair.right.rate_dps is None
            or pair.left.published_est_deg is None or pair.right.published_est_deg is None):
        return None
    return (s.speed, s.acceleration, pair.left.angle_deg - pair.right.angle_deg,
            pair.left.rate_dps, pair.right.rate_dps, pair.left.published_est_deg - pair.right.published_est_deg)


def arrival(train, query, mode):
    """One nearest observation per independent run, using only training-fold IQRs."""
    if len({obs.state.run for obs, _ in train}) < 3:
        return None
    columns = {'baseline': [0, 1, 2], 'phase': [0, 1, 2, 3, 4], 'published_estimate': [0, 1, 5]}[mode]
    values = np.array([x for _, x in train])[:, columns]
    floor = np.array([1., 10., .25, 1., 1., .25])[columns]
    scale = np.maximum(np.quantile(values, .75, axis=0) - np.quantile(values, .25, axis=0), floor)
    delta = (values - np.array(query)[columns]) / scale
    distance = np.sqrt(np.mean(delta * delta, axis=1))
    neighbors, runs = [], set()
    for idx in np.argsort(distance):
        obs = train[idx][0]
        if obs.state.run in runs:
            continue
        runs.add(obs.state.run)
        neighbors.append(obs)
        if len(neighbors) == 3:
            break
    return dict(speed_dps=sum(o.zero_speed for o in neighbors) / 3,
                energy_per_inertia=sum(.5 * math.radians(o.zero_speed)**2 for o in neighbors) / 3,
                in_range=bool(distance[idx] <= 1),
                warning=bool(distance[idx] <= 1 and sum(o.outcome == 'reversed' for o in neighbors) >= 2))


def section_scores(rows, section):
    sample = [r for r in rows if r['section_deg'] == section]
    paired = [r for r in sample if r['phase'] is not None and r['baseline'] is not None]
    scores = {'eligible': len(sample), 'paired': len(paired),
              'reversals': sum(r['outcome'] == 'reversed' for r in paired)}
    for name in ('baseline', 'phase', 'published_estimate'):
        if not paired:
            scores[name] = {}
            continue
        scores[name] = dict(
            speed_mae_dps=round(float(np.mean([abs(r[name]['speed_dps'] - r['zero_speed_dps']) for r in paired])), 3),
            energy_per_inertia_mae=round(float(np.mean([abs(r[name]['energy_per_inertia']
                           - .5 * math.radians(r['zero_speed_dps'])**2) for r in paired])), 6),
            detected=sum(r[name]['warning'] and r['outcome'] == 'reversed' for r in paired),
            false_warnings=sum(r[name]['warning'] and r['outcome'] == 'crossed' for r in paired),
            in_range=sum(r[name]['in_range'] for r in paired))
    return scores


def run(recordings_root):
    examples = []
    camera = {}
    for rid in (*return_model.PRIOR, *return_model.RECENT):
        folder = Path(recordings_root) / str(rid)
        episodes = []
        rows, _ = return_model.load_sections(folder, SECTIONS, episodes)
        feet = FootHistory.from_folder(folder)
        examples.extend((obs, feature_row(obs, feet)) for obs in return_model.coast_reference(rows))
        camera[rid] = camera_checks(folder, feet, episodes)
    report = {'features': ['inward speed', 'inward acceleration', 'left minus right measured angle',
                           'left foot rate', 'right foot rate', 'left minus right firmware-estimated angle'],
              'sections_deg': list(SECTIONS)}
    for split, ids in [('prior_leave_one_run_out', return_model.PRIOR), ('held_out', return_model.RECENT)]:
        decisions = []
        for obs, x in examples:
            s = obs.state
            if s.run not in ids:
                continue
            train = [(o, feature) for o, feature in examples if feature is not None and o.state.section == s.section
                     and o.state.run in return_model.PRIOR and o.state.run != s.run]
            decisions.append(dict(run=s.run, peak_ms=s.peak_ms, time_ms=s.time_ms, section_deg=s.section,
                                  outcome=obs.outcome, zero_speed_dps=obs.zero_speed,
                                  left_deg=s.left if math.isfinite(s.left) else None,
                                  right_deg=s.right if math.isfinite(s.right) else None,
                                  max_image_age_ms=s.foot_age_ms if math.isfinite(s.foot_age_ms) else None,
                                  left_rate_dps=x[3] if x else None, right_rate_dps=x[4] if x else None,
                                  **{mode: arrival(train, x, mode) if x else None
                                     for mode in ('baseline', 'phase', 'published_estimate')}))
        report[split] = {'by_section': {str(s): section_scores(decisions, s) for s in SECTIONS},
                         'per_run': {str(r): {str(s): section_scores([d for d in decisions if d['run'] == r], s)
                                             for s in SECTIONS} for r in ids},
                         'decisions': decisions, 'camera_angle_at_exposure': camera_score(camera, ids)}
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings_root', type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.recordings_root), indent=2, allow_nan=False))
