"""Audit recorded v18 aid/main decisions, software envelope and peak forecasts.

Native replay uses the same source headers with separate explicit-J aid and
ordinary arrival selectors. No changed trajectory, measured mechanical work,
ground-forward direction or physical energy-neutral angle is reconstructed.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import zlib

from analyze_v14_trial_energy import ROOT, potential, stats, tau_from_metadata, plt
from analyze_v15_trial_energy import FIELDS, invert_potential
from analyze_v16_trial_energy import read_csv, replay_completion

CPP = r'''
#include "half_cycle_energy_budget.h"
#include <iostream>
#include <iomanip>
namespace e=walking_return_energy;
namespace b=half_cycle_energy_budget;
void print(const e::Decision& d){std::cout<<' '<<d.valid<<' '<<d.width_ms<<' '<<d.q_mas<<' '
 <<d.predicted_work_j<<' '<<d.predicted_arrival_j<<' '<<d.unmet_j<<' '<<d.saturated<<' '<<d.request_j;}
int main(){lateral_energy::Geometry g;unsigned id;char command;std::cout<<std::setprecision(12);
 e::ArrivalFeedback feedback;b::Ledger ledger;
 while(std::cin>>command>>id){
  if(command=='D'){
   e::Input in;unsigned width;int side,aid;float tau,same;
   std::cin>>aid>>side>>in.rate_dps>>in.body_angle_deg>>in.target_deg>>in.previous_peak_deg>>same
     >>in.initial_current_ma>>in.goal_current_ma>>tau>>in.cap_ms>>width>>in.arrival_bias_j>>in.return_work_j;
   in.side=side;in.same_side_peak_deg=same<0?NAN:same;
   lateral_energy::CurrentTable table;if(!table.prepare(tau))return 2;
   auto selected=aid?b::decideReturnAid(g,table,in):e::decide(g,table,in);
   auto guarded=aid?b::evaluateReturnAidWidth(g,table,in,width):e::evaluateWidth(g,table,in,width);
   const auto lower_width=selected.width_ms?selected.width_ms-1:0;
   auto lower=aid?b::evaluateReturnAidWidth(g,table,in,lower_width):e::evaluateWidth(g,table,in,lower_width);
   std::cout<<command<<' '<<id;print(selected);print(guarded);print(lower);
   std::cout<<' '<<e::earlyAdmissible(guarded)<<' '<<selected.available_j<<' '<<selected.peak_correction_j<<'\n';
  }else if(command=='L'){
   unsigned peak,width,elapsed;int aid,accept;
   std::cin>>peak>>aid>>accept>>width>>elapsed;
   const bool source=ledger.sourcePeak(peak);const auto before=ledger.remaining();
   bool accepted=false,completed=false;
   if(accept)accepted=ledger.accepted(aid?b::Kind::ReturnAid:b::Kind::Main,width);
   const auto reserved=ledger.remaining();
   if(accept && width)completed=ledger.complete(elapsed);
   std::cout<<command<<' '<<id<<' '<<source<<' '<<before<<' '<<accepted<<' '<<reserved
      <<' '<<completed<<' '<<ledger.remaining()<<' '<<ledger.spentMs()
      <<' '<<ledger.aidUsed()<<' '<<ledger.mainUsed()<<'\n';
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


def native_model(temp):
    names = ['half_cycle_energy_budget.h', 'walking_return_energy.h', 'walking_energy_policy_v15.h',
             'walking_energy_policy.h', 'lateral_energy_control.h', 'realtime_code.h']
    hashes = {}
    for name in names:
        blob = (ROOT / 'src' / name).read_bytes()
        (temp / name).write_bytes(blob)
        hashes[name] = hashlib.sha256(blob).hexdigest()
    (temp / 'v18.cpp').write_text(CPP, encoding='utf8')
    exe = temp / 'v18.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(temp), str(temp / 'v18.cpp'), '-o', str(exe)], check=True)
    return exe, hashes


def run_native(exe, commands):
    return subprocess.run([str(exe)], input='\n'.join(commands) + '\n', text=True,
                          capture_output=True, check=True).stdout.splitlines()


def reconstruct(rows, exe, tau):
    commands = []
    for i, r in enumerate(rows):
        assert r['valid'] and r['cap_ms'] > 0
        values = [i, int(r['return_event']), r['side'], r['rate_dps'], r['model_body_deg'],
                  r['target_deg'], r['previous_peak_deg'],
                  r['same_side_peak_deg'] if r['same_side_peak_deg'] is not None else -1,
                  r['aligned_i0_ma'], r['goal_current_ma'], tau, r['cap_ms'],
                  r['guarded_ms'], r['arrival_bias_j'], r['return_work_j']]
        commands.append('D ' + ' '.join(map(str, values)))
    output = run_native(exe, commands)
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
        r['selected_width_error_ms'] = r['selected_width_ms'] - r['requested_ms']
        r['guarded_work_error_j'] = r['guarded_work_j'] - r['predicted_work_j']
        r['guarded_arrival_error_j'] = r['guarded_arrival_j'] - r['predicted_arrival_j']
        r['ceil_minimum_verified'] = (r['selected_saturated'] or r['selected_request_j'] == 0 or
            (r['selected_work_j'] + 1e-10 >= r['selected_request_j'] and r['lower_work_j'] < r['selected_request_j']))
    assert all(r['selected_width_error_ms'] == 0 for r in rows)
    assert all(abs(r['guarded_work_error_j']) < 2e-8 for r in rows)
    assert all(abs(r['guarded_arrival_error_j']) < 2e-8 for r in rows)
    assert all(r['ceil_minimum_verified'] for r in rows)


def replay_ledger(rows, exe):
    commands = []
    for i, r in enumerate(rows):
        accepted = r['executed'] or (not r['optional'] and not r['return_event'] and r['valid'] and r['coast'])
        r['accepted_main_coast'] = accepted and not r['executed']
        commands.append('L ' + ' '.join(map(str, [i, r['peak_ms'], int(r['return_event']),
                        int(accepted), r['guarded_ms'] if r['executed'] else 0, r['actual_elapsed_ms']])))
    for line in run_native(exe, commands):
        v = list(map(int, line.split()[1:]));r = rows[v[0]]
        _, source, before, accepted, reserved, completed, after, spent, aid_used, main_used = v
        assert source and before == r['envelope_before_ms'] and after == r['envelope_after_ms'], (r['time_ms'], v)
        assert bool(accepted) == (r['executed'] or r['accepted_main_coast'])
        assert bool(completed) == r['executed']
        assert r['cap_ms'] <= before and (not r['executed'] or r['guarded_ms'] <= r['cap_ms'])
        r.update({'replayed_envelope_before_ms': before, 'replayed_reserved_remaining_ms': reserved,
                  'replayed_envelope_after_ms': after, 'replayed_spent_ms': spent,
                  'replayed_aid_used': bool(aid_used), 'replayed_main_used': bool(main_used)})
    groups = []
    for peak in sorted(set(r['peak_ms'] for r in rows)):
        group = [r for r in rows if r['peak_ms'] == peak]
        outputs = [r for r in group if r['executed']]
        aid = [r for r in outputs if r['return_event']]
        mains = [r for r in group if (r['executed'] or r['accepted_main_coast']) and not r['return_event']]
        assert len(aid) <= 1 and len(mains) <= 1
        assert sum(r['actual_elapsed_ms'] for r in outputs) <= 80
        assert not aid or (len(mains) == 1 and mains[0]['time_ms'] > aid[0]['output_end_ms'])
        groups.append({'source_peak_ms': peak, 'side': group[0]['side'], 'candidates': len(group),
            'aid_ms': aid[0]['time_ms'] if aid else None, 'aid_elapsed_ms': aid[0]['actual_elapsed_ms'] if aid else 0,
            'main_ms': mains[0]['time_ms'] if mains else None,
            'main_elapsed_ms': mains[0]['actual_elapsed_ms'] if mains else 0,
            'main_guarded_ms': mains[0]['guarded_ms'] if mains else None,
            'main_request_j': mains[0]['request_j'] if mains else None,
            'main_unmet_j': mains[0]['elapsed_unmet_pred_j'] if mains and mains[0]['executed'] else None,
            'total_elapsed_ms': sum(r['actual_elapsed_ms'] for r in outputs),
            'remaining_ms': group[-1]['envelope_after_ms'],
            'aid_to_main_gap_ms': mains[0]['time_ms'] - aid[0]['output_end_ms'] if aid and mains else None})
    return groups


def match_peaks(rows, peaks):
    for r in rows:
        r['prediction_usable'] = False
        r['prediction_exclusion'] = 'aid_not_arrival_forecast' if r['return_event'] else 'unsent'
        if r['return_event'] or not (r['executed'] or r['accepted_main_coast']):
            continue
        future = [p for p in peaks if p['peak_time_ms'] > r['time_ms']]
        if not future:
            r['prediction_exclusion'] = 'no_future_peak'
            continue
        p = future[0]
        if any(x['executed'] and r['time_ms'] < x['output_start_ms'] < p['peak_time_ms'] for x in rows if x is not r):
            r['prediction_exclusion'] = 'intervening_output'
            continue
        if p['physical_peak_side'] != r['side'] or not p['pending_command_matched']:
            r['prediction_exclusion'] = 'side_or_pending_mismatch'
            continue
        q = r['elapsed_q_pred_mas'] if r['executed'] else 0.
        assert abs(p['pending_q_command_mA_s'] - q) < 2e-5
        forecast = r['elapsed_arrival_pred_j'] if r['executed'] else r['predicted_arrival_j']
        observed = potential(p['peak_amplitude_deg'])
        assert abs(observed - p['measured_energy_j']) < 6e-9
        r.update({'prediction_usable': True, 'prediction_exclusion': '',
            'next_peak_ms': p['peak_time_ms'], 'observed_peak_deg': p['peak_amplitude_deg'],
            'observed_nominal_peak_j': observed, 'arrival_forecast_j': forecast,
            'forecast_peak_deg': invert_potential(forecast), 'nominal_arrival_error_j': observed - forecast,
            'peak_minus_stage_target_deg': p['peak_amplitude_deg'] - r['target_deg'],
            'peak_residual_observed': p['model_residual_observed'],
            'peak_confirmed_ms': max(p['peak_time_ms'], p['peak_telemetry_capture_time_ms'])})
        if r['assisted_cycle']:
            assert not p['model_residual_observed'], 'Assisted cycle unexpectedly trained arrival feedback'


def replay_feedback(rows, exe):
    commands = []
    learning = []
    for i, r in enumerate(rows):
        if not r['return_event']:
            commands.append((r['time_ms'], 1, 'B ' + ' '.join(map(str, [i, r['side'], r['time_ms'], r['target_j']]))))
        if r['prediction_usable'] and r['peak_residual_observed'] and r['executed'] and not r['assisted_cycle']:
            assert r['completion_reason'] == 1 and r['completion_valid'] and r['completion_work_valid']
            commands.append((r['peak_confirmed_ms'], 0, 'F ' + ' '.join(map(str, [i, r['side'],
                r['next_peak_ms'], r['observed_peak_deg'], r['elapsed_arrival_pred_j'], r['arrival_bias_j'], r['target_j']]))))
            learning.append(r['time_ms'])
    errors = []
    for line in run_native(exe, [x[2] for x in sorted(commands)]):
        _, idx, bias = line.split();r = rows[int(idx)]
        r['replayed_arrival_bias_j'] = float(bias)
        r['arrival_bias_replay_error_j'] = float(bias) - r['arrival_bias_j']
        errors.append(abs(r['arrival_bias_replay_error_j']))
    assert max(errors, default=0) < 1e-8
    return {'queries': len(errors), 'learning_main_times_ms': learning, 'max_abs_error_j': max(errors, default=0),
            'assisted_cycles_excluded': sum(r['assisted_cycle'] and r['prediction_usable'] for r in rows)}


def compact(r):
    fields = ['time_ms', 'peak_ms', 'side', 'executed', 'return_event', 'optional', 'assisted_cycle',
              'return_evidence', 'return_foot_age_ms', 'model_body_deg', 'rate_dps', 'target_deg',
              'requested_ms', 'guarded_ms', 'actual_elapsed_ms', 'output_start_ms', 'output_end_ms',
              'completion_reason', 'envelope_before_ms', 'envelope_after_ms', 'cap_ms', 'available_j',
              'request_j', 'elapsed_work_pred_j', 'elapsed_arrival_pred_j', 'elapsed_unmet_pred_j',
              'next_peak_ms', 'observed_peak_deg', 'forecast_peak_deg', 'nominal_arrival_error_j',
              'arrival_bias_j', 'prediction_exclusion', 'pre_input_wheel_rpm']
    return {k: r[k] for k in fields if k in r}


def analyze(path, source_rwlog, user_observation):
    meta = json.loads((path / 'metadata.json').read_text(encoding='utf8'))
    header = json.loads((path / 'header.json').read_text(encoding='utf8'))
    assert meta['diagnostic_build_id'] == 'walking-energy-transfer-v18'
    assert meta['walking_energy']['branch'] == 23 and meta['walking_energy']['revision'] == 'walking_energy_transfer_v18'
    assert not meta['walking_energy']['truncated']
    raw = source_rwlog.read_bytes()
    stored_crc = struct.unpack_from('<I', raw, header['crc_offset'])[0]
    computed_crc = zlib.crc32(raw[:header['crc_offset']]) & 0xffffffff
    assert stored_crc == computed_crc and len(raw) == header['crc_offset'] + 4
    audits = meta['walking_energy']['records']
    events = [e for e in meta['energy_control_autonomous_zero_cross_events'] if e['work_control_branch'] == 23]
    peaks = meta['energy_control_autonomous_peak_events']
    support = {a['time_ms']: a for a in meta['foot_support_assist']['records']}
    assert len(audits) == len(events)
    rows = []
    for a, e in zip(audits, events):
        t = a['time_ms'];assert 0 <= t - e['zero_cross_time_ms'] <= 5
        assert a['side'] == e['physical_next_peak_side']
        prior = next((p for p in reversed(peaks) if p['peak_time_ms'] < t and p['physical_peak_side'] == a['side']), None)
        same = prior['peak_amplitude_deg'] if prior and t - prior['peak_time_ms'] <= 3000 else None
        body = e['detector_angle_after_deg']
        if t in support:
            body = math.copysign(invert_potential(a['potential_j']), body)
        r = dict(a)
        r.update({'rate_dps': e['zero_cross_abs_rate_dps'], 'model_body_deg': body,
            'interpolated_zero_ms': e['zero_cross_time_ms'],
            'decision_aligned_i0_ma': e['aligned_initial_current_mA'],
            'decision_physical_signed_i0_ma': a['side'] * e['aligned_initial_current_mA'],
            'target_deg': e['startup_target_deg'], 'previous_peak_deg': e['previous_peak_amplitude_deg'],
            'same_side_peak_deg': same, 'aligned_i0_ma': e['aligned_initial_current_mA'],
            'goal_current_ma': e['predicted_current_goal_mA'], 'cap_ms': e['work_brake_cap_ms'],
            'pre_input_wheel_rpm': e['pre_input_wheel_speed_rpm'],
            'pre_input_wheel_age_us': e['pre_input_wheel_speed_age_us'],
            'event_candidate_peak_forecast_deg': e['predicted_next_peak_amplitude_deg'],
            'event_kind': e['event_kind'],
            'foot_dispatch_angle_recovered_from_nominal_U': t in support})
        if a['executed']:
            assert a['output_end_ms'] - a['output_start_ms'] == a['actual_elapsed_ms']
        rows.append(r)
    with tempfile.TemporaryDirectory(prefix='v18_trial_energy_') as folder:
        temp = Path(folder);exe, hashes = native_model(temp)
        reconstruct(rows, exe, tau_from_metadata(meta))
        cycles = replay_ledger(rows, exe)
        completion_hash, completion_errors = replay_completion(temp, audits)
        hashes['energy_pulse_completion.h'] = completion_hash
        match_peaks(rows, peaks)
        feedback = replay_feedback(rows, exe)
    samples = [r for r in read_csv(path / 'timeseries.csv') if r['state_id'] in ['3', '5']]
    active = [r for r in samples if r['state_id'] == '3']
    outputs = [r for r in rows if r['executed']]
    main_outputs = [r for r in outputs if not r['return_event']]
    matched = [r for r in rows if r['prediction_usable']]
    aids = [r for r in outputs if r['return_event']]
    thresholds = {}
    for threshold in [5, 10, 15, 20, 30, 45, 60, 90]:
        found = next((s for s in samples if float(s['t_test_ms']) >= 16000 and abs(float(s['roll_mekf_abs_deg'])) >= threshold), None)
        thresholds[str(threshold)] = {k: found[k] for k in ['t_test_ms', 'state_id', 'roll_mekf_abs_deg',
            'pitch_mekf_detector_relative_deg', 'pulse_active', 'motor_cmd_mA']} if found else None
    last_main = main_outputs[-1]
    nearest_main = min(samples, key=lambda s: abs(float(s['t_test_ms']) - last_main['time_ms']))
    summary = {
        'identity': {'build': meta['diagnostic_build_id'], 'revision': meta['walking_energy']['revision'],
            'branch': 23, 'format_version': header['format_version'], 'sample_count': header['sample_count'],
            'rwlog_sha256': hashlib.sha256(raw).hexdigest(), 'crc_valid': stored_crc == computed_crc,
            'metadata_sha256': hashlib.sha256((path / 'metadata.json').read_bytes()).hexdigest()},
        'user_observation': user_observation, 'terminal_state': meta['terminal_state'],
        'active_end_ms': max(float(s['t_test_ms']) for s in active), 'accepted_peaks': len(peaks),
        'counts': {'candidates': len(rows), 'outputs': len(outputs), 'main_outputs': len(main_outputs),
            'ordinary_main_outputs': sum(not r['optional'] for r in main_outputs),
            'early_main_outputs': sum(r['optional'] for r in main_outputs), 'return_aids': len(aids),
            'accepted_main_coasts': sum(r['accepted_main_coast'] for r in rows),
            'completion_reasons': dict(Counter(r['completion_reason'] for r in outputs)),
            'return_evidence': dict(Counter(r['return_evidence'] for r in aids)),
            'planned_elapsed_mismatch': sum(r['actual_elapsed_ms'] != r['guarded_ms'] for r in outputs),
            'model_extrapolated_outputs': sum(r['extrapolated'] for r in outputs),
            'main_unmet_outputs': sum(r['elapsed_unmet_pred_j'] > 1e-9 for r in main_outputs)},
        'native_source_replay': {'header_sha256': hashes, 'selector_rows': len(rows),
            'maximum_width_error_ms': max(abs(r['selected_width_error_ms']) for r in rows),
            'maximum_work_error_j': max(abs(r['guarded_work_error_j']) for r in rows),
            'maximum_arrival_error_j': max(abs(r['guarded_arrival_error_j']) for r in rows),
            'all_integer_minima_verified': all(r['ceil_minimum_verified'] for r in rows),
            'completion_rows': len(outputs), 'completion_max_abs_errors': completion_errors,
            'ledger_all_rows_matched': True, 'arrival_feedback': feedback},
        'shared_envelope': {'source_peaks': len(cycles), 'maximum_actual_ms': max(c['total_elapsed_ms'] for c in cycles),
            'over_80ms': sum(c['total_elapsed_ms'] > 80 for c in cycles),
            'assisted_half_cycles': [c for c in cycles if c['aid_ms'] is not None]},
        'prediction_matching': {'main_forecasts': len(matched),
            'exclusions': dict(Counter(r['prediction_exclusion'] for r in rows if not r['prediction_usable'])),
            'by_side': {str(sign): {'all': stats([r['nominal_arrival_error_j'] for r in matched if r['side'] == sign]),
                'unassisted': stats([r['nominal_arrival_error_j'] for r in matched if r['side'] == sign and not r['assisted_cycle']]),
                'assisted': stats([r['nominal_arrival_error_j'] for r in matched if r['side'] == sign and r['assisted_cycle']])} for sign in [-1, 1]},
            'largest_underprediction': compact(max(matched, key=lambda r: r['nominal_arrival_error_j'])),
            'largest_overprediction': compact(min(matched, key=lambda r: r['nominal_arrival_error_j']))},
        'late_input_support': [{k: r[k] for k in ['time_ms', 'interpolated_zero_ms', 'output_start_ms',
            'output_end_ms', 'side', 'model_body_deg', 'rate_dps', 'decision_aligned_i0_ma',
            'decision_physical_signed_i0_ma', 'start_i0_ma', 'pre_input_wheel_rpm', 'pre_input_wheel_age_us',
            'passive_rate_supported', 'work_rate_supported', 'charge_supported', 'early_state_transfer',
            'extrapolated', 'completion_extrapolated', 'elapsed_q_pred_mas', 'elapsed_work_pred_j',
            'arrival_bias_j', 'common_correction_j', 'redistribution_j', 'event_candidate_peak_forecast_deg']}
            for r in rows if r['time_ms'] in [15581, 15954, 16367, 16801]],
        'late_sequence': [compact(r) for r in rows if r['time_ms'] >= 14500],
        'late_peaks': [p for p in peaks if p['peak_time_ms'] >= 14300],
        'roll_mekf_late_thresholds': thresholds,
        'last_main_nearest_sample': {k: nearest_main[k] for k in ['t_test_ms', 'roll_mekf_abs_deg',
            'pitch_mekf_detector_relative_deg', 'physical_roll_rate_dps', 'gx_dps', 'acc_norm_g', 'mekf_accel_confidence']},
        'last_sample': {k: samples[-1][k] for k in ['t_test_ms', 'state_id', 'roll_mekf_abs_deg',
            'pitch_mekf_detector_relative_deg', 'physical_roll_rate_dps', 'gx_dps', 'acc_norm_g', 'mekf_accel_confidence']},
        'limits': ['Nominal peak U and empirical pulse/arrival work are model diagnostics, not measured mechanical work.',
            'Aid predicted_arrival_J is local return-demand fulfillment and is excluded from next-peak forecasts.',
            'Actual elapsed counts software queue acceptance to stop request, not physical current edges.',
            'Foot, support, ground-forward direction and physical neutral/inertia are not identified by this audit.',
            'The late rising roll_mekf tilt precedes the final main; temporal order alone cannot assign fall causality.',
            'The source snapshot is hashed; native replay verifies recorded decisions, not a changed plant trajectory.']}
    return summary, rows, cycles, samples, peaks


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w', encoding='utf8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields);writer.writeheader();writer.writerows(rows)


def plot(path, rows, samples, peaks):
    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True, constrained_layout=True)
    t = [float(s['t_test_ms']) / 1000 for s in samples]
    axes[0].plot(t, [float(s['pitch_mekf_detector_relative_deg']) for s in samples], color='#2459a7', lw=1)
    axes[0].scatter([p['peak_time_ms']/1000 for p in peaks],
        [p['physical_peak_side']*p['peak_amplitude_deg'] for p in peaks], color='#2459a7', s=12)
    axes[0].axhline(8, color='gray', ls='--', lw=.7);axes[0].axhline(-8, color='gray', ls='--', lw=.7)
    axes[0].set_ylabel('Controlled angle (deg)')
    axes[1].plot(t, [float(s['roll_mekf_abs_deg']) for s in samples], color='#ba421e', lw=1)
    axes[1].axhline(90, color='gray', ls='--', lw=.7);axes[1].set_ylabel('roll_mekf (deg)')
    for r in rows:
        if not r['executed']:continue
        color='#c35a22' if r['return_event'] else '#22679b'
        axes[2].bar(r['output_start_ms']/1000, r['side'], width=r['actual_elapsed_ms']/1000,
                    align='edge', color=color, alpha=.8)
        axes[3].scatter(r['time_ms']/1000, r['elapsed_unmet_pred_j']*1000,
                        color=color, marker='^' if r['return_event'] else 'o', s=20)
    axes[2].set_ylabel('Command sign\norange=aid');axes[2].set_yticks([-1, 0, 1])
    axes[3].set_ylabel('Model unmet (mJ)');axes[3].set_xlabel('Run time (s)')
    axes[0].set_title('62788986: v18 source-verified software energy audit; user reports backward fall')
    for ax in axes:ax.grid(alpha=.2);ax.axvline(16.218, color='#7a4590', ls=':', lw=.8)
    fig.savefig(path / 'v18_energy_timeline.png', dpi=150);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--rwlog', type=Path, required=True)
    parser.add_argument('--user-observation', default='User explicitly reported a backward fall.')
    args = parser.parse_args()
    summary, rows, cycles, samples, peaks = analyze(args.trial, args.rwlog, args.user_observation)
    (args.trial / 'v18_energy_audit.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf8')
    write_csv(args.trial / 'v18_energy_events.csv', rows)
    write_csv(args.trial / 'v18_energy_half_cycles.csv', cycles)
    plot(args.trial, rows, samples, peaks)
    print(json.dumps({k: summary[k] for k in ['identity', 'terminal_state', 'counts', 'shared_envelope', 'prediction_matching']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
