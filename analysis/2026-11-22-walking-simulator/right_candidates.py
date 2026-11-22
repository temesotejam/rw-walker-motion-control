"""Compare guarded right-going ZERO widths using actual prior runs as local support."""
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path
from statistics import mean, median

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-20-ab-trials'))
import verify_arrival as arrival

cal = arrival.cal
RETURN_WINDOW_MS = 1000
RATE_WINDOW_DPS = 3.0
INCOMING_WINDOW_DEG = 1.0
CHARGE_WINDOW_MAS = 1.0


def command_charge(row, width):
    """Predicted electrical charge for a command, using the decision-time current model."""
    duration = width / 1000
    tau = row['tau_s']
    return row['goal_ma'] * duration + (row['i0_ma'] - row['goal_ma']) * tau * -math.expm1(-duration / tau)


def return_observations(folders, rows):
    """Pair each accepted right arrival with the next accepted peak and later inputs."""
    observations = {}
    recorded_right_zeros = 0
    for run, folder in folders.items():
        data = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
        peaks = data['energy_control_autonomous_peak_events']
        recorded_right_zeros += sum(e['event_kind'] == 'energy_control_zero_cross' and e['physical_next_peak_side'] == 1
                                    for e in data['energy_control_autonomous_zero_cross_events'])
        pulses = [e['zero_cross_time_ms'] for e in data['energy_control_autonomous_zero_cross_events']
                  if e['output_executed']]
        with (folder / 'timeseries.csv').open(encoding='utf8', newline='') as stream:
            end = max(int(r['t_test_ms']) for r in csv.DictReader(stream) if r['state_id'] == '3')
        for row in rows:
            if row['run'] != run:
                continue
            zero = row['zero_ms']
            idx = next(i for i, peak in enumerate(peaks) if peak['peak_time_ms'] > zero)
            peak = peaks[idx]
            if peak['physical_peak_side'] != 1 or peak['peak_amplitude_deg'] != row['next_peak_deg']:
                raise ValueError(f'Right arrival labels do not match run {run} at {zero}')
            t = peak['peak_time_ms']
            following = peaks[idx+1] if idx+1 < len(peaks) else None
            returned = bool(following and following['physical_peak_side'] == -1
                            and following['peak_time_ms'] <= min(t + RETURN_WINDOW_MS, end))
            label = returned if returned or end >= t + RETURN_WINDOW_MS else None
            stop = following['peak_time_ms'] if returned else min(t + RETURN_WINDOW_MS, end)
            observations[run, zero] = {
                'right_peak_deg': row['next_peak_deg'], 'right_peak_ms': t, 'next_left': label,
                'further_command': any(t <= pulse < stop for pulse in pulses)}
    return observations, recorded_right_zeros


def matching_charge(peers, row, width):
    # Coast is a distinct action: a nearby positive Q must not count as coast evidence.
    q = command_charge(row, width)
    return [r for r in peers if bool(r['original_ms']) == bool(width) and abs(r['q_mas']-q) <= CHARGE_WINDOW_MAS]


def support_summary(matches, observations):
    outcomes = [observations[r['run'], r['zero_ms']] for r in matches]
    return {
        'events': len(matches), 'runs': len({r['run'] for r in matches}),
        'observed_width_range_ms': [min(r['original_ms'] for r in matches), max(r['original_ms'] for r in matches)] if matches else None,
        'right_peak_range_deg': [min(r['next_peak_deg'] for r in matches), max(r['next_peak_deg'] for r in matches)] if matches else None,
        'median_right_peak_deg': round(median(r['next_peak_deg'] for r in matches), 3) if matches else None,
        'right_peak_below_target': sum(r['next_peak_deg'] < r['target_deg'] for r in matches),
        'next_left_yes': sum(x['next_left'] is True for x in outcomes),
        'next_left_no': sum(x['next_left'] is False for x in outcomes),
        'next_left_censored': sum(x['next_left'] is None for x in outcomes),
        'with_further_command': sum(x['further_command'] for x in outcomes),
        'examples': [[r['run'], r['zero_ms']] for r in matches],
    }


def less_severe_cut(row, current_width, peers):
    """First less shortened width with >=2 donor runs; otherwise report the first with one.

    Search uses state, width and modeled Q only. Return/peak observations never rank widths.
    """
    first_single = None
    for width in range(current_width, row['original_ms'] + 1):
        matches = matching_charge(peers, row, width)
        runs = {r['run'] for r in matches}
        if len(runs) >= 2:
            return width, matches
        if runs and first_single is None:
            first_single = width, matches
    return first_single or (None, [])


def run(recordings_root: Path) -> dict:
    folders = arrival.recording_folders([Path(recordings_root)])
    rows = cal.extract(folders.values())
    prior = [r for r in rows if r['run'] in cal.TRAIN and 0 <= r['q_mas'] <= cal.Q_LIMIT]
    fitted = {test: cal.fit([r for r in prior if r['run'] != test]) for test in cal.TRAIN}
    prior_fit = cal.fit(prior)
    observations, recorded = return_observations(folders, rows)
    results = []
    errors = {}
    for row, (eligible, current_width) in zip(rows, arrival.firmware_choices(rows)):
        run_id = row['run']
        peers = [r for r in prior if r['run'] != run_id and r['target_deg'] == row['target_deg']
                 and abs(r['rate_dps'] - row['rate_dps']) <= RATE_WINDOW_DPS
                 and abs(r['previous_peak_deg'] - row['previous_peak_deg']) <= INCOMING_WINDOW_DEG]
        applied = matching_charge(peers, row, row['original_ms'])
        outcome = observations[run_id, row['zero_ms']]
        fit = fitted.get(run_id, prior_fit)
        fitted_peak = cal.angle(fit @ cal.features(row))
        split = 'prior_held_one_run_out' if run_id in cal.TRAIN else 'two_recent_runs'
        if eligible and row['original_ms'] > 0:
            groups = ['eligible_powered']
            if row['target_deg'] == 4.5:
                groups.append('eligible_startup_powered')
            if len({r['run'] for r in applied}) >= 2:
                groups.append('applied_charge_supported_by_two_runs')
            for group in groups:
                audit = errors.setdefault((split, group), [])
                audit.append((run_id, abs(fitted_peak - row['next_peak_deg']),
                              abs(median(r['next_peak_deg'] for r in applied)-row['next_peak_deg']) if applied else None))
        current = matching_charge(peers, row, current_width)
        if eligible and current_width < row['original_ms']:
            alternative, matches = less_severe_cut(row, current_width, peers)
        else:
            alternative, matches = None, []
        comparison_charge = command_charge(row, alternative) if alternative is not None else None
        current_charge = command_charge(row, current_width)
        results.append({
            'run': run_id, 'zero_ms': row['zero_ms'], 'split': split, 'eligible': eligible,
            'target_deg': row['target_deg'], 'original_ms': row['original_ms'], 'arrival1_ms': current_width,
            'comparison_ms': alternative, 'observed': outcome,
            'original_prediction_deg': round(fitted_peak, 3),
            'arrival1_predicted_deg': round(cal.angle(fit @ cal.features(dict(row, q_mas=current_charge))), 3)
            if eligible else None,
            'comparison_predicted_deg': round(cal.angle(fit @ cal.features(dict(row, q_mas=comparison_charge))), 3)
            if alternative is not None else None,
            'applied_support': support_summary(applied, observations),
            'arrival1_support': support_summary(current, observations),
            'comparison_support': support_summary(matches, observations)})
    metrics = {}
    for (split, group), entries in errors.items():
        local = [e for e in entries if e[2] is not None]
        metrics.setdefault(split, {})[group] = {
            'events': len(entries), 'runs': len({e[0] for e in entries}),
            'actual_command_energy_mae_deg': round(mean(e[1] for e in entries), 3),
            'local_median_events': len(local),
            'energy_mae_on_local_subset_deg': round(mean(e[1] for e in local), 3) if local else None,
            'local_median_mae_deg': round(mean(e[2] for e in local), 3) if local else None}
    return {
        'state_match': {'rate_tolerance_dps': RATE_WINDOW_DPS, 'incoming_peak_tolerance_deg': INCOMING_WINDOW_DEG,
                        'predicted_charge_tolerance_mA_s': CHARGE_WINDOW_MAS, 'same_target': True},
        'next_accepted_left_window_ms_after_right_peak': RETURN_WINDOW_MS,
        'rows': len(rows), 'recorded_ordinary_right_zeros': recorded,
        'without_analyzable_right_arrival': recorded - len(rows), 'fit_rows': len(prior),
        'eligible_rows': sum(r['eligible'] for r in results),
        'observed_outcome_counts': dict(Counter(str(r['observed']['next_left']) for r in results)),
        'validation': metrics, 'decisions': results}
