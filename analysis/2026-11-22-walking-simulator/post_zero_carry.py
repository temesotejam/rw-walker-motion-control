"""Conditional replay from an actual leftward ZERO to its already latched receiving CAD angle."""
from collections import Counter
import argparse
import json
import math
from pathlib import Path
from statistics import mean, median

from receiving_carry import CarryReplay
from foot_kinematics import FootHistory
import return_model


# Fixed local comparison widths, in deg/s, deg, mA*s, ms, and deg respectively.
TOLERANCES = {'zero_speed_dps': 3., 'foot_gap_deg': 2., 'charge_mas': 1.,
              'width_ms': 10., 'terminal_deg': .25}


def compare_state(ref, row):
    if ref['run'] == row['run'] or ref['powered'] != row['powered'] or ref['assisted'] != row['assisted']:
        return None
    ds = [abs(ref[k] - row[k]) / tol for k, tol in TOLERANCES.items()]
    return sum(d*d for d in ds) if max(ds) <= 1 else None


def score_prediction(references, row):
    # One nearest actual command per independent donor run, never a second sample
    # from the same run. No extrapolated speed for an unsupported command.
    donors = {}
    for ref in references:
        if ref['status'] not in ('crossed', 'reversed'):
            continue
        d = compare_state(ref, row)
        if d is not None and (ref['run'] not in donors or d < donors[ref['run']][0]):
            donors[ref['run']] = d, ref
    chosen = [ref for _, ref in sorted(donors.values(), key=lambda item: item[0])[:3]]
    result = {'donors': [[ref['run'], ref['zero_ms']] for ref in chosen]}
    if len(chosen) < 2:
        return dict(result, status='insufficient_similar_runs')
    # Empirical change in rotational kinetic energy per unit inertia at the endpoint.
    # A turn before reaching the target has zero arrival speed by definition.
    change = mean(ref['arrival_dps']**2 - ref['zero_speed_dps']**2 for ref in chosen)
    return dict(result, status='supported', predicted_dps=math.sqrt(max(0, row['zero_speed_dps']**2 + change)),
                donor_carry_count=sum(ref['arrival_dps'] >= return_model.CARRY_SPEED_DPS for ref in chosen))


def load_rows(folder):
    rid = json.loads((folder / 'header.json').read_text(encoding='utf-8'))['run_start_us']
    meta = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    geom = CarryReplay(folder)
    feet = FootHistory.from_folder(folder)
    archive = geom.archive
    if (archive.get('truncated') or archive['stored'] != archive['total'] or
            meta.get('metadata_event_detail_truncated') or meta.get('energy_control_autonomous_event_overflow')):
        raise ValueError(f'Incomplete command/event archive for run {rid}')
    rows = []
    for event in meta['energy_control_autonomous_zero_cross_events']:
        if event['event_kind'] != 'energy_control_zero_cross' or event['physical_next_peak_side'] != -1:
            continue
        t, peak = event['zero_cross_time_ms'], event['previous_peak_time_ms']
        plan = geom.at(t)
        row = dict(run=rid, zero_ms=t, peak_ms=peak, status=plan['status'])
        rows.append(row)
        if plan['terminal_deg'] is None:
            continue
        if plan.get('peak_ms') != peak:
            row['status'] = 'different_confirmed_peak'
            continue
        matches = [a for a in archive['records'] if a['peak_ms'] == peak and a['side'] == -1
                   and not a['rescue'] and not a['optional'] and abs(a['time_ms'] - t) <= 8]
        if len(matches) != 1:
            row['status'] = 'no_unique_main_command'
            continue
        action = matches[0]
        powered = bool(event['output_executed'])
        if (powered != bool(action['executed']) or (powered and
                (action['guarded_ms'] != event['pulse_width_ms'] or action['output_start_ms'] != event['pulse_start_ms']))):
            row['status'] = 'ambiguous_command'
            continue
        charge = action['q_mas'] if powered else 0.
        pair = feet.at(t)
        if not pair.left or not pair.right or charge is None or not math.isfinite(charge):
            row['status'] = 'missing_state_or_charge'
            continue
        row.update(zero_speed_dps=-event['zero_cross_rate_dps'], terminal_deg=plan['terminal_deg'],
                   command_decision_ms=action['time_ms'], powered=powered, assisted=action['assisted_cycle'],
                   width_ms=action['guarded_ms'] if powered else 0, charge_mas=charge,
                   foot_gap_deg=pair.left.angle_deg - pair.right.angle_deg)
        observation = geom.follow(plan)
        row['status'] = observation['status']
        if row['status'] not in ('crossed', 'reversed'):
            continue
        end = observation['confirmed_ms']
        others = [a for a in archive['records'] if a is not action and a['executed']
                  and a['output_start_ms'] < end and a['output_end_ms'] > t]
        if others:
            row['status'] = 'another_pulse'
            continue
        if powered and (not action['completion_valid'] or not action['planned_duration_completed']):
            row['status'] = 'incomplete_pulse'
            continue
        row['arrival_dps'] = observation['inward_speed_dps'] if row['status'] == 'crossed' else 0.
        row['observed_end_ms'] = observation['end_ms']
    return rows


def evaluate(train, test):
    cases, errors = [], []
    for row in test:
        if 'foot_gap_deg' not in row:
            continue
        prediction = score_prediction(train, row)
        cases.append(dict(row, prediction=prediction))
        if prediction['status'] == 'supported' and 'arrival_dps' in row:
            broad = [r for r in train if r['run'] != row['run'] and 'arrival_dps' in r
                     and r['assisted'] == row['assisted'] and r['powered'] == row['powered']]
            errors.append([abs(row['zero_speed_dps'] - row['arrival_dps']),
                           abs(prediction['predicted_dps'] - row['arrival_dps']),
                           abs(median(r['arrival_dps'] for r in broad) - row['arrival_dps'])])
    return {'all_left_zeros': len(test), 'statuses': dict(Counter(row['status'] for row in test)),
            'observable_endpoints': sum('arrival_dps' in r for r in test),
            'observed_below_carry': sum(r['arrival_dps'] < return_model.CARRY_SPEED_DPS for r in test if 'arrival_dps' in r),
            'supported_scored': len(errors), 'constant_speed_mae_dps': mean(x[0] for x in errors) if errors else None,
            'empirical_energy_mae_dps': mean(x[1] for x in errors) if errors else None,
            'same_action_median_mae_dps': mean(x[2] for x in errors) if errors else None, 'cases': cases}


def reference_rows(root):
    return [row for rid in return_model.PRIOR for row in load_rows(root / str(rid))]


def run(root):
    root = Path(root)
    reference = reference_rows(root)
    held = [row for rid in return_model.RECENT for row in load_rows(root / str(rid))]
    return {'tolerances': TOLERANCES, 'prior_leave_one_run_out': evaluate(reference, reference),
            'held_out': evaluate(reference, held)}


def evaluate_trial(root, trial):
    rows = load_rows(Path(trial))
    if any(r['run'] in return_model.PRIOR for r in rows):
        raise ValueError('New trial is in the training set')
    return evaluate(reference_rows(Path(root)), rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.root), indent=2, allow_nan=False))
