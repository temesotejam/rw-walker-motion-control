"""Retrospective comparison of adding a +4.5° warning to the +3°/+2° return watch."""
import argparse
from collections import Counter
import json
from pathlib import Path

import return_model


EARLY_SECTION = 4.5
SECTIONS = (EARLY_SECTION, *return_model.SECTIONS)


def compare_episodes(episodes):
    """Use the same no-further-command endpoint for both watches.

    A missing or unsupported prediction is an abstention and cannot warn.
    A case with no usable IMU section has no exposure anchor. A pulse before
    reversal confirmation invalidates both comparisons, even if the later
    two-section model could have started again after the pulse ended.
    """
    rows = []
    for episode in episodes:
        states = episode['observations']
        evidence = episode['score_status']
        if episode['outcome'] == 'censored':
            evidence = 'censored'
        eligible = evidence in ('scored', 'outside_reference', 'missing_prediction')
        alarms = [s for s in states if s['warning']]
        two = next((s['time_ms'] for s in alarms if s['section_deg'] != EARLY_SECTION), None)
        three = next((s['time_ms'] for s in alarms), None)
        early = next((s for s in states if s['section_deg'] == EARLY_SECTION), {})
        intervening = states[0]['interfering_pulse_starts_ms'] if states else []
        row = {
            'run': episode['run'], 'peak_ms': episode['peak_ms'],
            'outcome': episode['outcome'], 'end_ms': episode['end_ms'],
            'end_angle_deg': episode['end_angle_deg'], 'score_status': evidence,
            'eligible': eligible, 'interfering_pulse_starts_ms': intervening,
            'early_section_ms': early.get('time_ms'), 'early_supported': early.get('supported'),
            'early_warning_ms': early.get('time_ms') if early.get('warning') else None,
            'early_third_distance_iqr': early.get('third_distance_iqr'),
            'early_cross_fraction': early.get('cross_fraction'),
            'two_warning_ms': two, 'three_warning_ms': three,
            'two_lead_ms': None if two is None else round(episode['end_ms'] - two, 2),
            'three_lead_ms': None if three is None else round(episode['end_ms'] - three, 2),
        }
        rows.append(row)
    return rows


def summarize(rows):
    scored = [r for r in rows if r['eligible']]
    totals = Counter(r['outcome'] for r in scored)
    result = {
        'all_peaks': len(rows), 'scored': len(scored),
        'reversed': totals['reversed'], 'crossed': totals['crossed'],
        'status_counts': dict(Counter(r['score_status'] for r in rows)),
        'outside_reference_or_missing_counted_as_no_alarm': sum(
            r['score_status'] in ('outside_reference', 'missing_prediction') for r in scored),
    }
    for name in ('two', 'three'):
        warning = name + '_warning_ms'
        leads = [r[name + '_lead_ms'] for r in scored if r['outcome'] == 'reversed' and r[warning] is not None]
        false_alarms = sum(r['outcome'] == 'crossed' and r[warning] is not None for r in scored)
        result[name] = {
            'detected_reversals': len(leads), 'missed_reversals': totals['reversed'] - len(leads),
            'false_warnings': false_alarms, 'correct_crossings': totals['crossed'] - false_alarms,
            'detected_reversal_leads_ms': leads,
        }
    result['newly_detected_reversals'] = sum(r['outcome'] == 'reversed' and
        r['two_warning_ms'] is None and r['three_warning_ms'] is not None for r in scored)
    result['additional_false_warnings'] = sum(r['outcome'] == 'crossed' and
        r['two_warning_ms'] is None and r['three_warning_ms'] is not None for r in scored)
    result['earlier_by_ms_on_common_true_alarms'] = [
        r['two_warning_ms'] - r['three_warning_ms'] for r in scored
        if r['outcome'] == 'reversed' and r['two_warning_ms'] is not None
    ]
    return result


def read_recording(folder, expected_id=None):
    run_id = json.loads((folder / 'header.json').read_text(encoding='utf8'))['run_start_us']
    if expected_id is not None and run_id != expected_id:
        raise ValueError(f'Run identity does not match folder {expected_id}.')
    peaks = []
    states, _ = return_model.load_sections(folder, SECTIONS, peaks)
    return run_id, states, peaks


def evaluate_new(recordings_root, trial):
    """Apply the fixed earlier eight-run reference to an independently converted trial."""
    root = Path(recordings_root)
    run_id, states, peaks = read_recording(Path(trial))
    if run_id in return_model.PRIOR:
        raise ValueError('This is a training recording. Use the leave-one-run-out comparison.')
    reference = []
    for prior_id in return_model.PRIOR:
        _, prior_states, _ = read_recording(root / str(prior_id), prior_id)
        reference.extend(prior_states)
    replay = return_model.evaluate_episodes(reference, states, peaks)
    rows = compare_episodes(replay['episodes'])
    return {'run': run_id, 'training_run_ids': list(return_model.PRIOR),
            'comparison': summarize(rows), 'episodes': rows}


def run(recordings_root, test_runs=return_model.RECENT):
    test_runs = tuple(int(n) for n in test_runs)
    if set(test_runs).intersection(return_model.PRIOR) or len(set(test_runs)) != len(test_runs):
        raise ValueError('Test runs must be unique and independent of the earlier eight runs.')
    root = Path(recordings_root)
    reference, recorded, peaks = [], {}, {}
    for run_id in return_model.PRIOR + test_runs:
        folder = root / str(run_id)
        _, recorded[run_id], peaks[run_id] = read_recording(folder, run_id)
        if run_id in return_model.PRIOR:
            reference.extend(recorded[run_id])
    result = {
        'early_section_deg': EARLY_SECTION, 'base_sections_deg': list(return_model.SECTIONS),
        'selection': 'Retrospective hypothesis after inspecting earlier missing-section reversals.',
        'training_run_ids': list(return_model.PRIOR), 'test_run_ids': list(test_runs),
    }
    for split, run_ids in [('prior_leave_one_run_out', return_model.PRIOR), ('held_out', test_runs)]:
        rows = []
        per_run = {}
        for run_id in run_ids:
            replay = return_model.evaluate_episodes(reference, recorded[run_id], peaks[run_id])
            episodes = compare_episodes(replay['episodes'])
            per_run[str(run_id)] = summarize(episodes)
            rows.extend(episodes)
        result[split] = {'comparison': summarize(rows), 'per_run': per_run, 'episodes': rows}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings_root', type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.recordings_root), allow_nan=False, indent=2))
