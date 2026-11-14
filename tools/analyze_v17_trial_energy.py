"""Replay frozen v17 policy and software completion; audit return aid versus ZERO.

Nominal peak U and empirical arrival work are model diagnostics, not measured
mechanical work. Camera angles alone do not establish contact or forward travel.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from analyze_v14_trial_energy import ROOT, potential, stats, tau_from_metadata, plt
from analyze_v15_trial_energy import FIELDS
from analyze_v16_trial_energy import read_csv, replay_completion

CPP = r'''
#include "walking_return_energy.h"
#include <iostream>
#include <iomanip>
namespace e=walking_return_energy;
void print(const e::Decision& d){std::cout<<' '<<d.valid<<' '<<d.width_ms<<' '<<d.q_mas<<' '
 <<d.predicted_work_j<<' '<<d.predicted_arrival_j<<' '<<d.unmet_j<<' '<<d.saturated<<' '<<d.request_j;}
int main(){lateral_energy::Geometry g;unsigned id;char command;std::cout<<std::setprecision(12);
 e::ArrivalFeedback feedback;
 while(std::cin>>command>>id){
  if(command=='D'){
   e::Input in;unsigned width;int side;float tau,same;
   std::cin>>side>>in.rate_dps>>in.body_angle_deg>>in.target_deg>>in.previous_peak_deg>>same
     >>in.initial_current_ma>>in.goal_current_ma>>tau>>in.cap_ms>>width>>in.arrival_bias_j>>in.return_work_j;
   in.side=side;in.same_side_peak_deg=same<0?NAN:same;
   lateral_energy::CurrentTable table;if(!table.prepare(tau))return 2;
   auto selected=e::decide(g,table,in);auto guarded=e::evaluateWidth(g,table,in,width);
   auto lower=e::evaluateWidth(g,table,in,selected.width_ms?selected.width_ms-1:0);
   std::cout<<command<<' '<<id;print(selected);print(guarded);print(lower);
   std::cout<<' '<<e::earlyAdmissible(guarded)<<' '<<selected.available_j<<' '<<selected.peak_correction_j<<'\n';
  }else if(command=='F'){
   int side;unsigned now;float peak,predicted,applied,target;
   std::cin>>side>>now>>peak>>predicted>>applied>>target;
   feedback.observe(side,now,lateral_energy::potential(g,peak),predicted,applied,target,true);
  }else if(command=='B'){
   int side;unsigned now;float target;std::cin>>side>>now>>target;
   std::cout<<command<<' '<<id<<' '<<feedback.bias(side,now,target)<<'\n';
  }
 }
}
'''


def truth(value):
    return str(value).lower() in ['1', 'true']


def compile_model(temp):
    hashes = {}
    for name in ['walking_return_energy.h', 'walking_energy_policy_v15.h',
                 'walking_energy_policy.h', 'lateral_energy_control.h', 'realtime_code.h']:
        blob = (ROOT / 'src' / name).read_bytes()
        (temp / name).write_bytes(blob)
        hashes[name] = hashlib.sha256(blob).hexdigest()
    (temp / 'v17.cpp').write_text(CPP, encoding='utf8')
    exe = temp / 'v17.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(temp), str(temp / 'v17.cpp'), '-o', str(exe)], check=True)
    return exe, hashes


def reconstruct(rows, exe, tau):
    lines = []
    for i, r in enumerate(rows):
        assert r['candidate_valid']
        values = [i, r['side'], r['rate_dps'], r['model_body_deg'], r['target_deg'],
                  r['previous_peak_deg'], r['same_side_peak_deg'] if r['same_side_peak_deg'] is not None else -1,
                  r['aligned_i0_ma'], r['goal_current_ma'], tau, r['cap_ms'],
                  r['candidate_guarded_ms'], r['arrival_bias_j'], r['return_work_j']]
        lines.append('D ' + ' '.join(map(str, values)))
    output = subprocess.run([str(exe)], input='\n'.join(lines) + '\n', text=True,
                            capture_output=True, check=True).stdout.splitlines()
    assert len(output) == len(rows)
    for line in output:
        values = line.split()
        assert len(values) == 5 + 3 * len(FIELDS)
        r = rows[int(values[1])]
        for i, prefix in enumerate(['selected_', 'guarded_', 'lower_']):
            for key, value in zip(FIELDS, values[2 + i * len(FIELDS):2 + (i + 1) * len(FIELDS)]):
                x = float(value)
                r[prefix + key] = bool(int(x)) if key in ['valid', 'saturated'] else int(x) if key == 'width_ms' else x
        r['reconstructed_early_admissible'] = bool(int(values[-3]))
        r['reconstructed_available_j'] = float(values[-2])
        r['reconstructed_peak_correction_j'] = float(values[-1])
        r['selected_width_error_ms'] = r['selected_width_ms'] - r['candidate_requested_ms']
        r['guarded_work_error_j'] = r['guarded_work_j'] - r['candidate_work_j']
        r['guarded_arrival_error_j'] = r['guarded_arrival_j'] - r['predicted_arrival_j']
        r['ceil_minimum_verified'] = (r['selected_saturated'] or r['selected_request_j'] == 0 or
            (r['selected_work_j'] + 1e-10 >= r['selected_request_j'] and r['lower_work_j'] < r['selected_request_j']))


def matched_peak(row, audits, peaks):
    candidates = [p for p in peaks if p['peak_time_ms'] > row['time_ms']]
    if not candidates:
        return None, 'no_future_peak'
    p = candidates[0]
    if any(a['executed'] and row['time_ms'] < a['output_start_ms'] < p['peak_time_ms'] for a in audits if a['time_ms'] != row['time_ms']):
        return p, 'intervening_output'
    if p['physical_peak_side'] != row['side'] or not p['pending_command_matched']:
        return p, 'side_or_pending_mismatch'
    if p['pending_q_command_mA_s'] is None or abs(p['pending_q_command_mA_s'] - row['elapsed_q_pred_mas']) > 2e-5:
        return p, 'completion_Q_mismatch'
    return p, ''


def replay_feedback(rows, peaks, exe):
    commands = []
    observed = []
    for i, r in enumerate(rows):
        commands.append((r['time_ms'], 1, 'B ' + ' '.join(map(str, [i, r['side'], r['time_ms'], r['target_j']]))))
        if (r['prediction_usable'] and not r['return_event'] and r['completion_reason'] == 1 and
                r['completion_valid'] and r['completion_work_valid'] and r['next_peak_ms'] - r['time_ms'] <= 1500):
            # IMU health is not separately archived at this exact peak. Successful
            # chronological bias agreement validates this inferred learning set.
            commands.append((r['next_peak_ms'], 0, 'F ' + ' '.join(map(str, [i, r['side'], r['next_peak_ms'],
                r['observed_peak_deg'], r['elapsed_arrival_pred_j'], r['arrival_bias_j'], r['target_j']]))))
            observed.append({'decision_ms': r['time_ms'], 'peak_ms': r['next_peak_ms'], 'side': r['side']})
    output = subprocess.run([str(exe)], input='\n'.join(c[2] for c in sorted(commands)) + '\n', text=True,
                            capture_output=True, check=True).stdout.splitlines()
    assert len(output) == len(rows)
    for line in output:
        _, idx, value = line.split()
        r = rows[int(idx)]
        r['replayed_arrival_bias_j'] = float(value)
        r['arrival_bias_replay_error_j'] = float(value) - r['arrival_bias_j']
    error = max(abs(r['arrival_bias_replay_error_j']) for r in rows)
    assert error < 1e-8, error
    return {'learning_set_inferred_from_completion_and_peak_matching': observed,
            'queries': len(rows), 'max_abs_bias_error_j': error,
            'tolerance_j': 1e-8, 'rounding_note': 'Archived peak amplitude and energy fields have decimal rounding.'}


def zero_after(row, series):
    # Crossing sign is opposite the commanded NEXT peak side.
    start = [s for s in series if row['output_end_ms'] <= float(s['t_test_ms']) <= row.get('next_peak_ms', 30000)]
    for a, b in zip(start, start[1:]):
        aa, bb = [float(s['pitch_mekf_detector_relative_deg']) for s in [a, b]]
        if aa * row['side'] <= 0 and bb * row['side'] >= 0 and aa != bb:
            ratio = -aa / (bb - aa)
            lerp = lambda key: float(a[key]) + ratio * (float(b[key]) - float(a[key]))
            return {'time_ms': lerp('t_test_ms'), 'bracket_ms': [float(a['t_test_ms']), float(b['t_test_ms'])],
                    'body_rate_dps': lerp('physical_roll_rate_dps'),
                    'current_telemetry_ma': lerp('roller_actual_current_mA'),
                    'vbat_mv': lerp('roller_battery_mV'), 'interpolation': 'linear_between_sampled_sign_change'}
    return None


def foot_review(path, rows):
    feet = read_csv(path / 'foot_angles.csv')
    pose = {f['sequence']: f for f in read_csv(path / 'foot_pose_prediction.csv')}
    frames = []
    qualified = {'left': [], 'right': []}
    for f in feet:
        if f['sequence'] not in pose or not f['measurement_time_us'] or float(f['measurement_time_us']) < 0:
            continue
        fr = {'frame_ms': float(f['measurement_time_us']) / 1000,
              'ready_ms': float(pose[f['sequence']]['ready_ms']),
              'sequence': int(f['sequence']), 'active': f['state_id'] == '3'}
        for side in qualified:
            fr[side + '_deg'] = float(f[side + '_deg']) if f[side + '_deg'] else None
            fr[side + '_reason'] = f[side + '_reason']
            fr[side + '_strict_range'] = truth(f[side + '_in_range'])
            fr[side + '_control_range'] = truth(f[side + '_control_range_ok'])
            fr[side + '_extrapolation_deg'] = float(f[side + '_extrapolation_deg']) if f[side + '_extrapolation_deg'] else None
            fr[side + '_usable'] = all(truth(f[k]) for k in ['timestamp_valid', 'frame_valid', 'zero_ready', side + '_valid', side + '_control_range_ok'])
            if fr[side + '_usable']:
                qualified[side].append(dict(fr, angle_deg=fr[side + '_deg']))
        fr['both_usable'] = fr['left_usable'] and fr['right_usable']
        frames.append(fr)
    active = [f for f in frames if f['active']]
    extended = {s: [f for f in active if f[s + '_usable'] and not f[s + '_strict_range']] for s in qualified}
    final_gap = [f for f in active if 24000 <= f['frame_ms'] <= 24800]
    after_return = [f for f in active if 24214 <= f['frame_ms'] <= 24450]
    latest = max((f for f in frames if f['ready_ms'] <= rows[-1]['time_ms']), key=lambda f: f['ready_ms'])
    summary = {'active_frames': len(active), 'control_usable_frames': {s: sum(f[s + '_usable'] for f in active) for s in qualified},
               'strict_range_exceeded_but_control_accepted_frames': {s: len(q) for s, q in extended.items()},
               'accepted_extrapolation_deg': {s: stats([f[s + '_extrapolation_deg'] for f in q]) for s, q in extended.items()},
               'terminal_window_frames': final_gap, 'last_output_latest_completed_frame': latest,
               'final_return_usable_frames': [f for f in after_return if f['both_usable']],
               'interpretation': 'Strict calibration range flags remain false for permitted extrapolation; no angle clipping is inferred.'}
    return summary, qualified, frames


def analyze(path, user_observation=None):
    meta_blob = (path / 'metadata.json').read_bytes()
    meta = json.loads(meta_blob)
    assert meta['walking_energy']['branch'] == 22 and meta['walking_energy']['revision'] == 'walking_return_energy_v17'
    assert not meta['walking_energy']['truncated']
    audits = meta['walking_energy']['records']
    events = [e for e in meta['energy_control_autonomous_zero_cross_events'] if e['work_control_branch'] == 22]
    assert len(audits) == len(events)
    peaks = meta['energy_control_autonomous_peak_events']
    rows = []
    for a, e in zip(audits, events):
        t = a['time_ms']
        assert 0 <= t - e['zero_cross_time_ms'] <= 5 and a['side'] == e['physical_next_peak_side']
        prior = next((p for p in reversed(peaks) if p['peak_time_ms'] < t and p['physical_peak_side'] == a['side']), None)
        same = prior['peak_amplitude_deg'] if prior and t - prior['peak_time_ms'] <= 3000 else None
        r = dict(a)
        r.update({'candidate_valid': a['valid'], 'candidate_requested_ms': a['requested_ms'],
                  'candidate_guarded_ms': a['guarded_ms'], 'candidate_q_mas': a['q_mas'],
                  'candidate_work_j': a['predicted_work_j'], 'rate_dps': e['zero_cross_abs_rate_dps'],
                  'model_body_deg': e['detector_angle_after_deg'] if a['optional'] or a['rescue'] else 0.,
                  'target_deg': e['startup_target_deg'], 'previous_peak_deg': e['previous_peak_amplitude_deg'],
                  'same_side_peak_deg': same, 'aligned_i0_ma': e['aligned_initial_current_mA'],
                  'goal_current_ma': e['predicted_current_goal_mA'], 'cap_ms': e['work_brake_cap_ms'],
                  'event_kind': e['event_kind'], 'event_index': e['event_index'],
                  'prediction_usable': False, 'prediction_exclusion': 'unsent'})
        if a['executed']:
            assert a['output_end_ms'] - a['output_start_ms'] == a['actual_elapsed_ms']
            p, exclusion = matched_peak(r, audits, peaks)
            r['prediction_exclusion'] = exclusion
            if not exclusion:
                observed = potential(p['peak_amplitude_deg'])
                r.update({'prediction_usable': True, 'next_peak_ms': p['peak_time_ms'],
                          'observed_peak_deg': p['peak_amplitude_deg'], 'observed_nominal_peak_j': observed,
                          'nominal_arrival_error_j': observed - a['elapsed_arrival_pred_j']})
        rows.append(r)
    series = [s for s in read_csv(path / 'timeseries.csv') if s['state_id'] == '3']
    outputs = [r for r in rows if r['executed']]
    matched = [r for r in rows if r['prediction_usable']]
    with tempfile.TemporaryDirectory(prefix='walker_v17_audit_') as folder:
        temp = Path(folder)
        exe, hashes = compile_model(temp)
        reconstruct(rows, exe, tau_from_metadata(meta))
        completion_hash, completion_errors = replay_completion(temp, audits)
        hashes['energy_pulse_completion.h'] = completion_hash
        feedback = replay_feedback(rows, peaks, exe)
        rescue = []
        crossing_candidates = []
        for r in outputs:
            if not r['return_event']:
                continue
            zero = zero_after(r, series)
            entry = {k: r[k] for k in ['time_ms', 'side', 'requested_ms', 'guarded_ms', 'actual_elapsed_ms',
                'return_work_j', 'request_j', 'elapsed_work_pred_j', 'elapsed_arrival_pred_j',
                'elapsed_unmet_pred_j', 'prediction_usable', 'next_peak_ms', 'observed_peak_deg',
                'observed_nominal_peak_j', 'nominal_arrival_error_j'] if k in r}
            entry['geometric_zero_after_completion'] = zero
            if zero:
                d = dict(r)
                d.update({'time_ms': zero['time_ms'], 'model_body_deg': 0., 'rate_dps': abs(zero['body_rate_dps']),
                          'aligned_i0_ma': zero['current_telemetry_ma'] * r['side'], 'return_work_j': 0.,
                          'candidate_requested_ms': 0, 'candidate_guarded_ms': 0, 'candidate_work_j': 0.,
                          'predicted_arrival_j': 0., 'candidate_valid': True})
                crossing_candidates.append(d)
            rescue.append(entry)
        reconstruct(crossing_candidates, exe, tau_from_metadata(meta))
        for entry, d in zip([r for r in rescue if r['geometric_zero_after_completion']], crossing_candidates):
            entry['geometric_zero_after_completion'].update({
                'diagnostic_corrected_available_j': d['reconstructed_available_j'],
                'diagnostic_request_j': d['selected_request_j'], 'diagnostic_minimum_or_cap_width_ms': d['selected_width_ms'],
                'diagnostic_unmet_at_80ms_j': d['selected_unmet_j'],
                'diagnostic_method': 'Frozen v17 at interpolated measured crossing, no new return budget; no admission or new plant trajectory replay.'})
            d['cap_ms'] = max(0, 80 - entry['actual_elapsed_ms'])
        reconstruct(crossing_candidates, exe, tau_from_metadata(meta))
        for entry, d in zip([r for r in rescue if r['geometric_zero_after_completion']], crossing_candidates):
            entry['geometric_zero_after_completion'].update({
                'diagnostic_remaining_halfcycle_cap_ms': d['cap_ms'],
                'diagnostic_selected_with_remaining_cap_ms': d['selected_width_ms'],
                'diagnostic_work_with_remaining_cap_j': d['selected_work_j'],
                'diagnostic_unmet_with_remaining_cap_j': d['selected_unmet_j']})
    assert all(r['selected_width_error_ms'] == 0 for r in rows)
    assert all(abs(r['guarded_work_error_j']) < 2e-8 for r in rows)
    assert all(r['ceil_minimum_verified'] for r in rows)
    foot, qualified, frames = foot_review(path, rows)
    final_peak = peaks[-1]['peak_time_ms']
    after = [s for s in series if float(s['t_test_ms']) >= final_peak]
    minimum = min(after, key=lambda s: float(s['pitch_mekf_detector_relative_deg']))
    tail = [s for s in series if 25000 <= float(s['t_test_ms']) < 30000]
    groups = {}
    for name, group in [('all', matched), ('nonreturn', [r for r in matched if not r['return_event']]),
                        ('return', [r for r in matched if r['return_event']]),
                        ('stage8_plus_nonreturn', [r for r in matched if r['target_deg'] == 8 and r['side'] == 1 and not r['return_event']]),
                        ('stage8_minus_nonreturn', [r for r in matched if r['target_deg'] == 8 and r['side'] == -1 and not r['return_event']])]:
        groups[name] = {'count': len(group), 'nominal_arrival_error_mj': stats([r['nominal_arrival_error_j'] * 1000 for r in group]),
                        'peak_deg': stats([r['observed_peak_deg'] for r in group])}
    active_vbat = stats([float(s['roller_battery_mV']) for s in series])
    previous = json.loads((path.parent / '28820005/metadata.json').read_text(encoding='utf8'))
    prev_series = [s for s in read_csv(path.parent / '28820005/timeseries.csv') if s['state_id'] == '3']
    review = {'build': meta['diagnostic_build_id'], 'metadata_sha256': hashlib.sha256(meta_blob).hexdigest(),
        'archive': {k: meta['walking_energy'][k] for k in ['archive_storage', 'archive_available', 'stored', 'total', 'truncated']},
        'terminal_state': meta['terminal_state'], 'cpp_source_sha256': hashes,
        'candidate_reconstruction': {'count': len(rows), 'all_widths_match': True, 'all_minimum_or_cap_verified': True,
            'max_guarded_work_error_j': max(abs(r['guarded_work_error_j']) for r in rows),
            'max_guarded_arrival_error_j': max(abs(r['guarded_arrival_error_j']) for r in rows)},
        'completion_replay_max_errors': completion_errors, 'causal_arrival_bias_replay': feedback,
        'outputs': {'count': len(outputs), 'normal': sum(not r['optional'] and not r['rescue'] for r in outputs),
            'early': sum(r['optional'] for r in outputs), 'rescue': sum(r['rescue'] for r in outputs),
            'all_timer': all(r['completion_reason'] == 1 for r in outputs),
            'elapsed_minus_guarded_counts_ms': dict(Counter(r['actual_elapsed_ms'] - r['guarded_ms'] for r in outputs)),
            'modeled_unmet_remaining': sum(r['elapsed_unmet_pred_j'] > 1e-9 for r in outputs), 'last': outputs[-1]},
        'prediction_matching': dict(Counter(r['prediction_exclusion'] or 'matched' for r in rows)),
        'prediction_groups': groups, 'return_rescues': rescue, 'foot_observations': foot,
        'latency': {k: meta['control_latency'][k] for k in
                    ['control_ready_age', 'decision_age', 'done_age', 'io_failures', 'io_max_us', 'overrun_detail']},
        'final_stall': {'last_peak_ms': final_peak, 'last_peak_deg': peaks[-1]['peak_amplitude_deg'],
            'minimum_after_last_peak_deg': float(minimum['pitch_mekf_detector_relative_deg']),
            'minimum_time_ms': float(minimum['t_test_ms']),
            'tail25_to30_body_deg': stats([float(s['pitch_mekf_detector_relative_deg']) for s in tail]),
            'foot_tail25_to30_deg': {s: stats([f['angle_deg'] for f in q if 25000 <= f['frame_ms'] < 30000]) for s, q in qualified.items()},
            'user_observation': user_observation},
        'previous_trial_comparison': {'prior_build': previous['diagnostic_build_id'],
            'prior_last_peak_ms': previous['energy_control_autonomous_peak_events'][-1]['peak_time_ms'],
            'current_last_peak_ms': final_peak, 'current_active_battery_mv': active_vbat,
            'prior_active_battery_mv': stats([float(s['roller_battery_mV']) for s in prev_series]),
            'limits': 'Different battery and realized trajectory; last accepted body peak is not walking duration or causal improvement.'},
        'limits': ['Software queue-to-stop completion and predicted charge/work do not measure physical delivery.',
            'Nominal fixed-zero rocker U does not identify moving-foot potential, energy-neutral angle or true barrier work.',
            'Interpolated ZERO diagnostics use recorded states and do not reproduce new admission, actuator guards or motion.',
            'No video, ground contact, support alternation or forward displacement is established by this log.']}
    (path / 'v17_energy_review.json').write_text(json.dumps(review, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf8')
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (path / 'v17_energy_events.csv').open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    with (path / 'v17_foot_frames.csv').open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted(set().union(*(f.keys() for f in frames))))
        writer.writeheader(); writer.writerows(frames)
    plot(path, series, matched, rescue, frames, final_peak, float(minimum['t_test_ms']), meta['diagnostic_build_id'])
    print(json.dumps({k: review[k] for k in ['candidate_reconstruction', 'completion_replay_max_errors', 'prediction_groups']}, indent=2))
    return review


def plot(path, series, matched, rescue, frames, final_peak, minimum_ms, build):
    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, constrained_layout=True)
    t = [float(s['t_test_ms']) / 1000 for s in series]
    axes[0].plot(t, [float(s['pitch_mekf_detector_relative_deg']) for s in series], color='black', lw=.8, label='body angle')
    axes[0].axhline(0, color='grey', lw=.6)
    axes[0].set_ylabel('Body angle [deg]')
    for s, color in [('left', 'tab:blue'), ('right', 'tab:orange')]:
        q = [f for f in frames if f['active']]
        axes[1].plot([f['ready_ms'] / 1000 for f in q],
                     [f[s + '_deg'] if f[s + '_usable'] else float('nan') for f in q],
                     '.-', color=color, ms=2, lw=.6, label=s + ' relative foot at ready time')
    axes[1].set_ylabel('Foot angle [deg]')
    for side, color, label in [(1, 'tab:red', '+'), (-1, 'tab:blue', '-')]:
        q = [r for r in matched if r['side'] == side]
        axes[2].scatter([r['time_ms'] / 1000 for r in q], [r['elapsed_arrival_pred_j'] * 1000 for r in q], s=20, marker='x', color=color, label='predicted arrival ' + label)
        axes[2].scatter([r['next_peak_ms'] / 1000 for r in q], [r['observed_nominal_peak_j'] * 1000 for r in q], s=16, facecolors='none', edgecolors=color, label='nominal U at next peak ' + label)
    axes[2].axhline(potential(8) * 1000, color='grey', ls=':', label='nominal 8 deg target')
    axes[2].set_ylabel('Nominal energy [mJ]')
    axes[3].plot(t, [float(s['motor_cmd_mA']) for s in series], color='grey', lw=.6, label='command')
    axes[3].plot(t, [float(s['roller_actual_current_mA']) for s in series], color='tab:green', lw=.6, label='current telemetry')
    axes[3].set_ylabel('Current [mA]'); axes[3].set_xlabel('Measurement time [s]')
    for ax in axes:
        for r in rescue:
            ax.axvline(r['time_ms'] / 1000, color='tab:purple', alpha=.25, lw=.6)
        ax.axvspan(final_peak / 1000, 30, color='tab:red', alpha=.05)
        ax.axvline(final_peak / 1000, color='tab:red', ls='--', lw=.7)
        ax.grid(alpha=.2); ax.legend(loc='upper left', fontsize=8, ncol=2); ax.set_xlim(0, 30)
    fig.suptitle(f'RWLOG {path.name} / {build}\nPurple: return-aid decisions. Nominal energy is not measured mechanical work.')
    fig.savefig(path / 'v17_energy_foot_timeline.png', dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, [float(s['pitch_mekf_detector_relative_deg']) for s in series], lw=1, color='black', label='body angle')
    axes[0].set_ylabel('Body [deg]')
    axes[1].plot(t, [float(s['physical_roll_rate_dps']) for s in series], lw=.8, label='body angular rate')
    axes[1].set_ylabel('Rate [deg/s]')
    for s, color in [('left', 'tab:blue'), ('right', 'tab:orange')]:
        q = [f for f in frames if f['active']]
        axes[2].plot([f['ready_ms'] / 1000 for f in q],
                     [f[s + '_deg'] if f[s + '_usable'] else float('nan') for f in q],
                     '.-', ms=3, lw=.7, color=color, label=s + ' usable foot')
    invalid = [f for f in frames if f['active'] and not f['both_usable']]
    axes[2].scatter([f['ready_ms'] / 1000 for f in invalid], [0] * len(invalid), color='tab:red', marker='x', s=22, label='one or both feet unavailable')
    axes[2].set_ylabel('Foot [deg]'); axes[2].set_xlabel('Measurement / recognition-ready time [s]')
    for ax in axes:
        ax.axvline(final_peak / 1000, color='tab:red', ls='--', lw=.8, label='last peak')
        ax.axvline(minimum_ms / 1000, color='tab:purple', ls=':', lw=.8, label='return minimum')
        ax.set_xlim((final_peak - 414) / 1000, (final_peak + 786) / 1000)
        ax.grid(alpha=.2); ax.legend(fontsize=8, ncol=3)
    fig.suptitle('Final return: body motion and causally available foot observations')
    fig.savefig(path / 'v17_final_return_detail.png', dpi=160); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_directory', type=Path)
    parser.add_argument('--user-observation')
    args = parser.parse_args()
    analyze(args.converted_directory, args.user_observation)
