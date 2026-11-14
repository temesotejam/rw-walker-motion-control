"""Audit v16 software pulse completion and nominal-energy/foot observations.

Replay uses the repository's C++ selectors, not a second Python controller.
Queue-to-stop duration and model work are not physical delivery measurements.
Camera angles are relative observations; they do not establish foot contact,
forward displacement, an absolute energy-neutral angle, or barrier work.
"""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from analyze_v14_trial_energy import ROOT, potential, stats, tau_from_metadata, plt
from analyze_v15_trial_energy import compile_model, reconstruct, invert_potential


COMPLETION_CPP = r'''
#include "energy_pulse_completion.h"
#include <iostream>
#include <iomanip>
int main(){
  unsigned id; int side; energy_pulse_completion::Input in;
  std::cout << std::setprecision(12);
  while(std::cin >> id >> side >> in.rate_dps >> in.initial_current_ma
        >> in.goal_current_ma >> in.tau_s >> in.available_j >> in.request_j
        >> in.actual_elapsed_ms >> in.planned_ms){
    in.side=side; const auto out=energy_pulse_completion::evaluate(in);
    std::cout << id << ' ' << out.valid << ' ' << out.work_valid << ' '
      << out.q_mas << ' ' << out.work_j << ' ' << out.arrival_j << ' '
      << out.unmet_j << ' ' << out.current_end_ma << '\n';
  }
}
'''


def read_csv(path):
    with path.open(encoding='utf8', newline='') as stream:
        return list(csv.DictReader(stream))


def replay_completion(temp, audits):
    blob = (ROOT / 'src/energy_pulse_completion.h').read_bytes()
    (temp / 'energy_pulse_completion.h').write_bytes(blob)
    (temp / 'completion.cpp').write_text(COMPLETION_CPP, encoding='utf8')
    exe = temp / 'completion.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(temp), str(temp / 'completion.cpp'), '-o', str(exe)], check=True)
    fields = ['start_rate_dps', 'start_i0_ma', 'start_goal_ma', 'start_tau_s',
              'available_j', 'request_j', 'actual_elapsed_ms', 'guarded_ms']
    sent = [a for a in audits if a['executed']]
    lines = [' '.join(map(str, [i, a['side']] + [a[k] for k in fields])) for i, a in enumerate(sent)]
    output = subprocess.run([str(exe)], input='\n'.join(lines) + '\n', text=True,
                            capture_output=True, check=True).stdout
    result_fields = ['elapsed_q_pred_mas', 'elapsed_work_pred_j', 'elapsed_arrival_pred_j',
                     'elapsed_unmet_pred_j', 'elapsed_current_end_pred_ma']
    errors = {k: [] for k in result_fields}
    for line in output.splitlines():
        values = line.split()
        a = sent[int(values[0])]
        assert bool(int(values[1])) == a['completion_valid']
        assert bool(int(values[2])) == a['completion_work_valid']
        for key, value in zip(result_fields, values[3:]):
            error = float(value) - a[key]
            errors[key].append(error)
            assert abs(error) < (2e-9 if key.endswith('_j') else 2e-5), (key, error)
    assert len(output.splitlines()) == len(sent)
    return hashlib.sha256(blob).hexdigest(), {k: max(map(abs, v), default=0) for k, v in errors.items()}


def analyze(path, user_observation=None):
    meta = json.loads((path / 'metadata.json').read_text(encoding='utf8'))
    assert meta['walking_energy']['branch'] == 21
    assert meta['walking_energy']['completion_revision'] == 'actual_elapsed_pulse_completion_v16'
    audits = meta['walking_energy']['records']
    assert not meta['walking_energy']['truncated'], 'Incomplete audit cannot support all-pulse claims'
    events = [e for e in meta['energy_control_autonomous_zero_cross_events'] if e['work_control_branch'] == 21]
    assert len(audits) == len(events)
    peaks = meta['energy_control_autonomous_peak_events']
    peak_times = [p['peak_time_ms'] for p in peaks]
    support = {a['time_ms']: a for a in meta['foot_support_assist']['records']}
    rows = []
    for a, e in zip(audits, events):
        t = a['time_ms']
        assert 0 <= t - e['zero_cross_time_ms'] <= 5 and a['side'] == e['physical_next_peak_side']
        prior = next((p for p in reversed(peaks) if p['peak_time_ms'] < t and p['physical_peak_side'] == a['side']), None)
        same = prior['peak_amplitude_deg'] if prior and t - prior['peak_time_ms'] <= 3000 else None
        body = e['detector_angle_after_deg'] if a['optional'] or a['rescue'] else 0.
        # The foot backend projects the angle at dispatch. Recover only its
        # model magnitude from the logged nominal U, not a measured CAD angle.
        if t in support and a['valid']:
            body = (1 if body >= 0 else -1) * invert_potential(a['potential_j'])
        row = dict(a)
        row.update({'candidate_valid': a['valid'], 'candidate_requested_ms': a['requested_ms'],
                    'candidate_guarded_ms': a['guarded_ms'], 'candidate_q_mas': a['q_mas'],
                    'candidate_work_j': a['predicted_work_j'], 'rate_dps': e['zero_cross_abs_rate_dps'],
                    'model_body_deg': body, 'target_deg': e['startup_target_deg'],
                    'previous_peak_deg': e['previous_peak_amplitude_deg'], 'same_side_peak_deg': same,
                    'aligned_i0_ma': e['aligned_initial_current_mA'],
                    'goal_current_ma': e['predicted_current_goal_mA'], 'cap_ms': e['work_brake_cap_ms'],
                    'event_kind': e['event_kind'], 'foot_timing': t in support,
                    'prediction_usable': False, 'prediction_exclusion': 'unsent'})
        if a['executed']:
            assert a['output_end_ms'] - a['output_start_ms'] == a['actual_elapsed_ms']
            row['prediction_exclusion'] = 'no_future_peak'
            k = bisect.bisect_right(peak_times, t)
            if k < len(peaks):
                p = peaks[k]
                intervening = any(x['executed'] and x is not a and
                                  t < x['output_start_ms'] < p['peak_time_ms'] for x in audits)
                if intervening:
                    row['prediction_exclusion'] = 'intervening_output'
                elif p['physical_peak_side'] != a['side'] or not p['pending_command_matched']:
                    row['prediction_exclusion'] = 'side_or_pending_mismatch'
                elif p['pending_q_command_mA_s'] is None or abs(p['pending_q_command_mA_s'] - a['elapsed_q_pred_mas']) > 2e-5:
                    row['prediction_exclusion'] = 'completion_Q_mismatch'
                else:
                    observed = potential(p['peak_amplitude_deg'])
                    row.update({'prediction_usable': True, 'prediction_exclusion': '',
                                'next_peak_ms': p['peak_time_ms'], 'observed_peak_deg': p['peak_amplitude_deg'],
                                'observed_nominal_peak_j': observed,
                                'nominal_arrival_error_j': observed - a['elapsed_arrival_pred_j']})
        rows.append(row)
    with tempfile.TemporaryDirectory(prefix='walker_v16_') as folder:
        temp = Path(folder)
        exe, hashes = compile_model(temp)
        reconstruct(rows, exe, tau_from_metadata(meta))
        completion_hash, completion_errors = replay_completion(temp, audits)
        hashes['energy_pulse_completion.h'] = completion_hash
    reconstructed = [r for r in rows if 'selected_width_error_ms' in r]
    assert len(reconstructed) == sum(a['valid'] for a in audits)
    assert all(r['selected_width_error_ms'] == 0 for r in reconstructed)
    assert all(abs(r['guarded_work_error_j']) < 2e-8 for r in reconstructed)
    assert all(r['ceil_minimum_verified'] for r in reconstructed)
    outputs = [a for a in audits if a['executed']]
    matched = [r for r in rows if r['prediction_usable']]
    series = [r for r in read_csv(path / 'timeseries.csv') if r['state_id'] == '3']
    final_peak = peaks[-1]['peak_time_ms']
    after = [r for r in series if float(r['t_test_ms']) >= final_peak]
    minimum = min(after, key=lambda r: float(r['pitch_mekf_detector_relative_deg']))
    tail = [r for r in series if 25000 <= float(r['t_test_ms']) < 30000]
    feet = read_csv(path / 'foot_angles.csv')
    pose = {r['sequence']: r for r in read_csv(path / 'foot_pose_prediction.csv')}
    qualified = {side: [] for side in ['left', 'right']}
    availability = []
    for f in feet:
        if not f['measurement_time_us'] or f['sequence'] not in pose:
            continue
        availability.append({'frame_ms': float(f['measurement_time_us']) / 1000,
                             'ready_ms': float(pose[f['sequence']]['ready_ms']),
                             'left_deg': f['left_deg'] or None, 'right_deg': f['right_deg'] or None,
                             'left_reason': f['left_reason'], 'right_reason': f['right_reason']})
        for side in qualified:
            if (f[side + '_valid'] in ['1', 'True', 'true'] and
                    f[side + '_in_range'] in ['1', 'True', 'true']):
                qualified[side].append({'frame_ms': float(f['measurement_time_us']) / 1000,
                                        'ready_ms': float(pose[f['sequence']]['ready_ms']),
                                        'angle_deg': float(f[side + '_deg'])})
    assert all(qualified.values()), 'No qualified camera angles; do not publish an empty foot plot'
    frame_ages = []
    for a in audits:
        completed = [f for f in availability if f['ready_ms'] <= a['time_ms']]
        if completed:
            frame_ages.append(a['time_ms'] - max(completed, key=lambda f: f['ready_ms'])['frame_ms'])
    final_completed = max((f for f in availability if f['ready_ms'] <= outputs[-1]['time_ms']), key=lambda f: f['ready_ms'])
    active_feet = [f for f in feet if f['state_id'] == '3']
    latency = meta['control_latency']
    groups = {}
    for name, group in [('all', matched), ('stage8_plus', [r for r in matched if r['target_deg'] == 8 and r['side'] == 1]),
                        ('stage8_minus', [r for r in matched if r['target_deg'] == 8 and r['side'] == -1])]:
        groups[name] = {'count': len(group), 'nominal_arrival_error_mj': stats([r['nominal_arrival_error_j'] * 1000 for r in group])}
    review = {
        'build': meta['diagnostic_build_id'], 'archive': {k: meta['walking_energy'][k] for k in
            ['archive_storage', 'archive_available', 'stored', 'total', 'truncated']},
        'terminal_state': meta['terminal_state'], 'cpp_source_sha256': hashes,
        'candidate_reconstruction': {'count': len(reconstructed), 'all_widths_match': True,
                                     'all_minimum_or_cap_verified': True,
                                     'max_guarded_work_error_j': max(abs(r['guarded_work_error_j']) for r in reconstructed)},
        'completion_replay_max_errors': completion_errors,
        'outputs': {'count': len(outputs), 'normal': sum(not a['optional'] and not a['rescue'] for a in outputs),
                    'early': sum(a['optional'] for a in outputs), 'rescue': sum(a['rescue'] for a in outputs),
                    'all_timer': all(a['completion_reason'] == 1 for a in outputs),
                    'all_elapsed_equal_guarded': all(a['actual_elapsed_ms'] == a['guarded_ms'] for a in outputs),
                    'actual_width_counts_ms': dict(Counter(a['actual_elapsed_ms'] for a in outputs)),
                    'modeled_unmet_remaining': sum(a['elapsed_unmet_pred_j'] > 1e-9 for a in outputs),
                    'model_extrapolation': sum(a['extrapolated'] for a in outputs),
                    'last': outputs[-1]},
        'prediction_groups': groups,
        'final_stall': {'last_peak_ms': final_peak, 'last_peak_deg': peaks[-1]['peak_amplitude_deg'],
                        'minimum_after_last_peak_deg': float(minimum['pitch_mekf_detector_relative_deg']),
                        'minimum_time_ms': float(minimum['t_test_ms']),
                        'tail25_to30_body_deg': stats([float(r['pitch_mekf_detector_relative_deg']) for r in tail]),
                        'foot_tail25_to30_deg': {s: stats([f['angle_deg'] for f in q if 25000 <= f['frame_ms'] < 30000]) for s, q in qualified.items()},
                        'user_observation': user_observation,
                        'source_gates': {'foot_early_abs_angle_max_deg': 5, 'generic_early_abs_angle_max_deg': 1.5,
                                         'active_rescue_abs_angle_max_deg': 3.25, 'ordinary_requires_zero_crossing': True}},
        'latency': {k: latency[k] for k in ['control_ready_age', 'decision_age', 'done_age', 'io_failures', 'io_max_us']},
        'control_overrun_detail': latency['overrun_detail'],
        'foot_recognition_processing_ms': stats([float(f['processing_us']) / 1000 for f in active_feet]),
        'foot_frame_to_ready_ms': stats([f['ready_ms'] - f['frame_ms'] for f in availability
                                         if 0 <= f['frame_ms'] < 30000]),
        'latest_completed_frame_age_at_decisions_ms': stats(frame_ages),
        'last_output_latest_completed_frame': final_completed,
        'interpretation': 'All energies and arrival errors use the frozen nominal fixed-zero rocker/empirical model. '
                          'These are not measured mechanical work or an identified moving energy-neutral potential. '
                          'No ground contact, absolute foot-to-CAD zero, forward travel, or causal plant counterfactual is inferred.'}
    (path / 'v16_energy_review.json').write_text(json.dumps(review, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf8')
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (path / 'v16_energy_events.csv').open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    plot(path, series, qualified, matched, final_peak, meta['diagnostic_build_id'])
    print(json.dumps({k: review[k] for k in ['candidate_reconstruction', 'completion_replay_max_errors', 'prediction_groups']}, indent=2))
    return review


def plot(path, series, feet, matched, final_peak, build):
    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, constrained_layout=True)
    t = [float(r['t_test_ms']) / 1000 for r in series]
    axes[0].plot(t, [float(r['pitch_mekf_detector_relative_deg']) for r in series], color='black', lw=.9)
    axes[0].axhline(5, color='grey', ls=':', label='foot timing gate +/-5 deg')
    axes[0].axhline(-5, color='grey', ls=':')
    axes[0].axhline(0, color='grey', lw=.6)
    axes[0].set_ylabel('Body lateral angle [deg]')
    for side, color in [('left', 'tab:blue'), ('right', 'tab:orange')]:
        q = [r for r in feet[side] if 0 <= r['frame_ms'] <= 30000]
        axes[1].plot([r['ready_ms'] / 1000 for r in q], [r['angle_deg'] for r in q], '.-',
                     color=color, ms=2, lw=.7, label=side + ' (at recognition-ready time)')
    axes[1].set_ylabel('Relative foot angle [deg]')
    for side, color, label in [(1, 'tab:red', '+'), (-1, 'tab:blue', '-')]:
        group = [r for r in matched if r['side'] == side]
        axes[2].scatter([r['time_ms'] / 1000 for r in group], [r['elapsed_arrival_pred_j'] * 1000 for r in group],
                        s=20, marker='x', color=color, label='predicted arrival ' + label)
        axes[2].scatter([r['next_peak_ms'] / 1000 for r in group], [r['observed_nominal_peak_j'] * 1000 for r in group],
                        s=15, facecolors='none', edgecolors=color, label='nominal U at next peak ' + label)
    axes[2].axhline(potential(8) * 1000, color='grey', ls=':', label='nominal target 8 deg')
    axes[2].set_ylabel('Nominal energy [mJ]')
    axes[3].plot(t, [float(r['motor_cmd_mA']) for r in series], lw=.7, color='grey', label='command [mA]')
    axes[3].plot(t, [float(r['roller_actual_current_mA']) for r in series], lw=.6, color='tab:green', label='current telemetry [mA]')
    axes[3].set_ylabel('Current [mA]')
    axes[3].set_xlabel('Measurement time [s]')
    for ax in axes:
        ax.axvspan(final_peak / 1000, 30, color='tab:red', alpha=.06)
        ax.axvline(final_peak / 1000, color='tab:red', ls='--', lw=.7)
        ax.grid(alpha=.2)
        ax.legend(loc='upper left', fontsize=8, ncol=2)
        ax.set_xlim(0, 30)
    fig.suptitle(f'RWLOG {path.name} / {build}: pulse completion and foot observations\n'
                 'Foot angles shown when recognized; energy is a nominal model, not measured barrier work')
    fig.savefig(path / 'v16_energy_foot_timeline.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_directory', type=Path)
    parser.add_argument('--user-observation', help='External observation; never inferred from energy traces')
    args = parser.parse_args()
    analyze(args.converted_directory, args.user_observation)
