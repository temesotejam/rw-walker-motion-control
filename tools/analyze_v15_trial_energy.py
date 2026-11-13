"""Audit v15 candidate admission, planned output and sampled pulse lifecycle.

An executed transport and a fulfilled MODEL budget do not prove that the pulse
lasted its planned width. This audit separates candidate/model fields from
sampled command-active intervals. Absolute-current diagnostic Q excludes pulse
edges and is not the model's signed/net Q or measured mechanical work.
"""
import argparse
import bisect
from collections import Counter,defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

from analyze_v14_trial_energy import ROOT,potential,stats,tau_from_metadata,plt

FIELDS=['valid','width_ms','q_mas','work_j','arrival_j','unmet_j','saturated','request_j']
CPP=r'''
#include "walking_energy_policy_v15.h"
#include <iostream>
#include <iomanip>
namespace e=walking_energy_policy_v15;
void print(const e::Decision& d){std::cout<<' '<<d.valid<<' '<<d.width_ms<<' '<<d.q_mas<<' '
 <<d.predicted_work_j<<' '<<d.predicted_arrival_j<<' '<<d.unmet_j<<' '<<d.saturated<<' '<<d.request_j;}
int main(){lateral_energy::Geometry g;unsigned id,cap,width;int side;float rate,angle,target,previous,same,i0,goal,tau;
 std::cout<<std::setprecision(12);
 while(std::cin>>id>>side>>rate>>angle>>target>>previous>>same>>i0>>goal>>tau>>cap>>width){
  lateral_energy::CurrentTable table;if(!table.prepare(tau))return 2;
  e::Input in;in.side=side;in.rate_dps=rate;in.body_angle_deg=angle;in.target_deg=target;
  in.previous_peak_deg=previous;in.same_side_peak_deg=same<0?NAN:same;
  in.initial_current_ma=i0;in.goal_current_ma=goal;in.cap_ms=cap;
  auto selected=e::decide(g,table,in);auto guarded=e::evaluateWidth(g,table,in,width);
  auto lower=e::evaluateWidth(g,table,in,selected.width_ms?selected.width_ms-1:0);
  std::cout<<id;print(selected);print(guarded);print(lower);std::cout<<' '<<e::earlyAdmissible(guarded)<<'\n';
 }
}
'''


def number(x):
    try:return float(x)
    except (TypeError,ValueError):return math.nan


def finite(x):return x is not None and math.isfinite(number(x))


def invert_potential(energy):
    lo=0.;hi=17.45
    for _ in range(60):
        mid=(lo+hi)/2
        if potential(mid)<energy:lo=mid
        else:hi=mid
    return (lo+hi)/2


def compile_model(temp):
    hashes={}
    for name in ['walking_energy_policy_v15.h','walking_energy_policy.h','lateral_energy_control.h','realtime_code.h']:
        blob=(ROOT/'src'/name).read_bytes();(temp/name).write_bytes(blob);hashes[name]=hashlib.sha256(blob).hexdigest()
    (temp/'audit.cpp').write_text(CPP,encoding='utf8');exe=temp/'audit.exe'
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(temp),str(temp/'audit.cpp'),'-o',str(exe)],check=True)
    return exe,hashes


def reconstruct(rows,exe,tau):
    usable=[r for r in rows if r['candidate_valid'] and all(finite(r[k]) for k in
        ['rate_dps','model_body_deg','target_deg','previous_peak_deg','aligned_i0_ma','goal_current_ma'])]
    lines=[]
    for i,r in enumerate(usable):
        values=[i,r['side'],r['rate_dps'],r['model_body_deg'],r['target_deg'],r['previous_peak_deg'],
                r['same_side_peak_deg'] if r['same_side_peak_deg'] is not None else -1,
                r['aligned_i0_ma'],r['goal_current_ma'],tau,r['cap_ms'],r['candidate_guarded_ms']]
        lines.append(' '.join(map(str,values)))
    result=subprocess.run([str(exe)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    for line in result.stdout.splitlines():
        values=line.split();assert len(values)==2+3*len(FIELDS);r=usable[int(values[0])]
        for i,prefix in enumerate(['selected_','guarded_','lower_']):
            for key,value in zip(FIELDS,values[1+i*len(FIELDS):1+(i+1)*len(FIELDS)]):
                x=float(value);r[prefix+key]=bool(int(x)) if key in ['valid','saturated'] else int(x) if key=='width_ms' else x if math.isfinite(x) else None
        r['reconstructed_early_admissible']=bool(int(values[-1]))
        if not r['selected_valid']:
            r['reconstruction_invalid']=True
            continue
        r['selected_width_error_ms']=r['selected_width_ms']-r['candidate_requested_ms']
        r['guarded_q_error_mas']=r['guarded_q_mas']-r['candidate_q_mas']
        r['guarded_work_error_j']=r['guarded_work_j']-r['candidate_work_j']
        r['ceil_minimum_verified']=(r['selected_saturated'] or r['selected_request_j']==0 or
            (r['selected_work_j']+1e-10>=r['selected_request_j'] and
             r['lower_work_j']<r['selected_request_j']))


def sampled_pulse(event,pulse_id,samples,probes):
    group=samples.get(pulse_id,[]);start=event['pulse_start_ms'];width=event['pulse_width_ms'];end=start+width
    after=[s for s in group if number(s['t_test_ms'])>=start]
    active=[s for s in after if s['pulse_active']=='1' and number(s['motor_cmd_mA'])!=0]
    result={'pulse_id':pulse_id,'active_sample_count':len(active),'programmed_width_ms':width,
            'programmed_end_ms':end,'programmed_end_is_actual_completion':False,
            'software_duration_lower_proxy_ms':0.,'software_duration_upper_proxy_ms':None}
    if active:
        last=number(active[-1]['t_test_ms']);first_off=next((s for s in after if number(s['t_test_ms'])>=last and s['pulse_active']=='0'),None)
        result.update({'first_active_ms':number(active[0]['t_test_ms']),'last_active_ms':last,
                       'software_duration_lower_proxy_ms':max(0,last-start-1),
                       'software_duration_upper_proxy_ms':number(first_off['t_test_ms'])-start+1 if first_off else None,
                       'sampled_lifecycle':'active_near_programmed_end' if last>=end-6 else 'active_but_short_or_unresolved'})
    elif after:
        upper=number(after[0]['t_test_ms'])-start+1
        result.update({'first_after_start_ms':number(after[0]['t_test_ms']),'software_duration_upper_proxy_ms':upper,
                       'sampled_lifecycle':'collapsed_before_first_sample' if after[0]['pulse_active']=='0' and number(after[0]['motor_cmd_mA'])==0 and upper<min(width,10)
                       else 'no_active_sample_unresolved'})
    else:result['sampled_lifecycle']='no_post_start_sample'
    capture=event.get('pre_input_capture_time_us',0)
    zero=next((s for s in after if number(s['roller_applied_command_time_us'])>capture and number(s['roller_applied_current_mA'])==0),None)
    result['new_zero_command_shadow_observed']=zero is not None
    if zero:
        result['zero_command_shadow_us']=int(number(zero['roller_applied_command_time_us']))
        result['zero_command_shadow_sequence']=int(number(zero['roller_applied_command_sequence']))
    expected=event['q_command_direction']*event['command_current_mA']
    on=next((s for s in after if number(s['roller_applied_command_time_us'])>capture and
             number(s['roller_applied_current_mA'])==expected),None)
    result['new_nonzero_command_shadow_observed']=on is not None
    if on:
        on_us=int(number(on['roller_applied_command_time_us']))
        off=next((s for s in after if number(s['roller_applied_command_time_us'])>on_us and number(s['roller_applied_current_mA'])==0),None)
        result['nonzero_command_shadow_us']=on_us
        if off:
            off_us=int(number(off['roller_applied_command_time_us']))
            result['zero_after_nonzero_command_shadow_us']=off_us
            result['applied_command_ack_interval_ms']=(off_us-on_us)/1000
    result['zero_ack_delay_after_preinput_snapshot_ms']=(result['zero_command_shadow_us']-capture)/1000 if zero else None
    probe=probes.get(pulse_id)
    if probe and zero and result['zero_command_shadow_us']>=probe['pulse_start_us']:
        result['probe_start_us']=probe['pulse_start_us']
        result['acknowledged_zero_after_probe_start_us']=result['zero_command_shadow_us']-probe['pulse_start_us']
    fresh=[s for s in active if s['roller_current_valid']=='1' and number(s['roller_current_sample_time_us'])>capture]
    q=[s for s in fresh if s['roller_q_meas_observed_valid']=='1' and finite(s['roller_q_meas_observed_mA_s'])]
    result['fresh_active_current_samples']=len({s['roller_current_sample_time_us'] for s in fresh})
    result['fresh_active_q_diagnostic_samples']=len(q)
    result['partial_absolute_q_diagnostic_mas']=number(q[-1]['roller_q_meas_observed_mA_s']) if q else None
    result['fresh_active_abs_current_max_ma']=max((abs(number(s['roller_actual_current_mA'])) for s in fresh),default=None)
    result['stale_prior_pulse_q_rows_ignored']=sum(s['roller_q_meas_observed_valid']=='1' and
        (number(s['roller_current_sample_time_us'])<=capture or s['pulse_active']!='1') for s in after[:3])
    return result


def analyze(path,exe,hashes):
    blob=(path/'metadata.json').read_bytes();meta=json.loads(blob)
    assert meta['diagnostic_build_id']=='early-energy-main-v15'
    audits=meta['walking_energy']['records'];all_events=meta['energy_control_autonomous_zero_cross_events']
    events=[e for e in all_events if e['work_control_branch']==20];assert len(audits)==len(events)
    peaks=meta['energy_control_autonomous_peak_events'];pt=[p['peak_time_ms'] for p in peaks]
    with (path/'timeseries.csv').open(encoding='utf8',newline='') as f:series=list(csv.DictReader(f))
    samples=defaultdict(list)
    for sample in series:
        if sample['state_id'] in ['3','4','5']:samples[int(sample['pulse_id'])].append(sample)
    output_events=[e for e in all_events if e['output_executed']]
    output_ids={e['event_index']:i for i,e in enumerate(output_events,1)}
    assert max(samples)==len(output_events),'Cannot infer pulse_id from output ordinal'
    probes={r['pulse_id']:r for r in meta['v46k_timing_probe_events']}
    foot={r['time_ms']:r for r in meta['foot_support_assist']['records']}
    rows=[]
    for a,event in zip(audits,events):
        t=a['time_ms'];side=a['side'];previous=event['previous_peak_time_ms'];support=foot.get(t)
        assert side==event['physical_next_peak_side'] and 0<=t-event['zero_cross_time_ms']<=5
        prior=next((p for p in reversed(peaks) if p['peak_time_ms']<t and p['physical_peak_side']==side),None)
        same=prior['peak_amplitude_deg'] if prior and t-prior['peak_time_ms']<=3000 else None
        body=event['detector_angle_after_deg'] if a['optional'] or a['rescue'] else 0.
        body_source='raw_detector_angle_after' if a['optional'] else 'normal_ZERO_0'
        if support and finite(a['potential_j']):
            body=math.copysign(invert_potential(a['potential_j']),body);body_source='magnitude_inverted_from_logged_nominal_U_for_foot_dispatch_projection'
        goal=event.get('predicted_current_goal_mA')
        if not finite(goal):
            sat=meta['model_i_sat_at_reference_mA']+meta['model_i_sat_slope_mA_per_v']*(event['vbat_mV']/1000-meta['model_vbat_reference_v'])
            goal=1200/(1+(1200/sat)**meta['model_i_sat_exponent'])**(1/meta['model_i_sat_exponent'])
        i0=event.get('aligned_initial_current_mA')
        if not finite(i0):i0=side*event['i0_estimated_mA']
        r={'event_index':event['event_index'],'time_ms':t,'zero_cross_ms':event['zero_cross_time_ms'],
           'peak_ms':a['peak_ms'],'side':side,'optional':a['optional'],'foot_candidate':support is not None,
           'rescue':a['rescue'],'executed':a['executed'],'event_valid':event['valid'],'candidate_valid':a['valid'],
           'candidate_coast':a['coast'],'candidate_saturated':a['saturated'],'extrapolated':a['extrapolated'],
           'passive_rate_supported':a['passive_rate_supported'],'work_rate_supported':a['work_rate_supported'],
           'charge_supported':a['charge_supported'],'early_budget_checked':a['early_budget_checked'],
           'early_budget_admitted':a['early_budget_admitted'],'candidate_requested_ms':a['requested_ms'],
           'candidate_guarded_ms':a['guarded_ms'],'guard_reduction_ms':a['requested_ms']-a['guarded_ms'],
           'candidate_q_mas':a['q_mas'],'candidate_work_j':a['predicted_work_j'],'candidate_arrival_j':a['predicted_arrival_j'],
           'candidate_unmet_j':a['unmet_j'],'request_j':a['request_j'],'target_j':a['target_j'],'available_j':a['available_j'],
           'output_q_pred_mas':a['output_q_pred_mas'],'output_work_pred_j':a['output_work_pred_j'],
           'unmet_after_command_j':a['unmet_after_command_j'],'event_q_mas':event.get('q_command_mA_s'),
           'event_work_j':event.get('predicted_input_work_j'),'event_unmet_j':event.get('unapplied_energy_request_j'),
           'rate_dps':event['zero_cross_abs_rate_dps'],'model_body_deg':body,'body_input_source':body_source,
           'target_deg':event['startup_target_deg'],'previous_peak_deg':event['previous_peak_amplitude_deg'],
           'previous_peak_age_ms':t-previous,'same_side_peak_deg':same,'aligned_i0_ma':i0,'goal_current_ma':goal,
           'cap_ms':event['work_brake_cap_ms'] if event['work_brake_cap_ms'] else 80,
           'cap_input_source':'raw_event' if event['work_brake_cap_ms'] else 'nominal80_inferred_unwritten_event_field',
           'output_start_ms':a['output_start_ms'],
           'event_reason':event['reason'],'admission_reason_inferred':True,'prediction_usable':False,'prediction_exclusion':'unsent_candidate'}
        if support:r.update({'foot_backend_reason':support['backend_reason'],'foot_selector_reason':support['backend_selector_reason']})
        if a['executed']:
            r.update(sampled_pulse(event,output_ids[event['event_index']],samples,probes))
            r['output_start_lag_ms']=a['output_start_ms']-t
            r['event_duration_matches_candidate']=event['pulse_end_ms']-event['pulse_start_ms']==a['guarded_ms']
            r['admission_reason']='early_budget_admitted' if a['optional'] else 'normal_drive'
            r['admission_reason_inferred']=False
        elif a['optional'] and a['coast']:r['admission_reason']='early_candidate_surplus_coast';r['admission_reason_inferred']=False
        elif a['optional'] and a['early_budget_checked']:r['admission_reason']='budget_rejected_pending_reconstruction'
        elif support:r['admission_reason']='foot_timing_recheck_'+meta['foot_support_assist']['reason_names'][support['backend_selector_reason']];r['admission_reason_inferred']=False
        elif not a['valid'] and r['previous_peak_age_ms']>1500:r['admission_reason']='preceding_peak_stale_inference'
        elif not a['optional'] and a['coast']:r['admission_reason']='normal_surplus_coast';r['admission_reason_inferred']=False
        else:r['admission_reason']='prebudget_rejection_unresolved'
        # A no-send candidate has no pending prediction. A valid ordinary
        # coast can be compared if its pending Q matches the first next peak.
        if a['executed'] or (not a['optional'] and a['coast'] and event['valid']):
            k=bisect.bisect_right(pt,t);r['prediction_exclusion']='no_future_peak'
            if k<len(peaks):
                peak=peaks[k];stop=peak['peak_time_ms']
                later=[e for e in all_events if e['output_executed'] and e['event_index']!=event['event_index'] and t<e['pulse_start_ms']<stop]
                r.update({'next_peak_ms':stop,'next_peak_delay_ms':stop-t,'observed_peak_deg':peak['peak_amplitude_deg'],
                          'observed_peak_j':potential(peak['peak_amplitude_deg']),'next_peak_pending_matched':peak['pending_command_matched']})
                if later:r['prediction_exclusion']='intervening_output'
                elif peak['physical_peak_side']!=side:r['prediction_exclusion']='first_peak_wrong_side'
                elif not peak['pending_command_matched']:r['prediction_exclusion']='pending_not_matched'
                elif peak['pending_q_command_mA_s'] is None or abs(peak['pending_q_command_mA_s']-a['output_q_pred_mas'])>.0001:r['prediction_exclusion']='pending_Q_mismatch'
                else:
                    r['prediction_usable']=True;r['prediction_exclusion']=''
                    r['planned_arrival_error_j']=r['observed_peak_j']-a['predicted_arrival_j']
                    r['observed_target_energy_ratio']=r['observed_peak_j']/a['target_j']
        rows.append(r)
    reconstruct(rows,exe,tau_from_metadata(meta))
    for r in rows:
        if r['admission_reason']=='budget_rejected_pending_reconstruction':
            r['admission_reason']='cap_infeasible_inference' if r.get('selected_saturated') else 'post_guard_budget_shortfall_inference'
        if r['optional'] and not r['executed']:
            normal=next((x for x in rows if x['time_ms']>r['time_ms'] and x['peak_ms']==r['peak_ms'] and not x['optional']),None)
            r['same_peak_later_normal_found']=normal is not None
            if normal:r.update({'later_normal_ms':normal['time_ms'],'later_normal_delay_ms':normal['time_ms']-r['time_ms'],
                                'later_normal_executed':normal['executed'],'later_normal_valid':normal['event_valid']})
    outputs=[r for r in rows if r['executed']];unsent=[r for r in rows if r['optional'] and not r['executed']]
    full=[r for r in outputs if r['sampled_lifecycle']=='active_near_programmed_end']
    collapsed=[r for r in outputs if r['sampled_lifecycle']=='collapsed_before_first_sample']
    matched=[r for r in rows if r['prediction_usable']]
    groups={'all_planned':matched,'normal_full_sampled':[r for r in matched if r in full],
            'collapsed_planned_not_calibration':[r for r in matched if r in collapsed],
            'early_planned':[r for r in matched if r['optional']],
            'stage8_full_plus':[r for r in matched if r in full and r['target_deg']==8 and r['side']>0],
            'stage8_full_minus':[r for r in matched if r in full and r['target_deg']==8 and r['side']<0],
            'stage8_collapsed_plus_not_calibration':[r for r in matched if r in collapsed and r['target_deg']==8 and r['side']>0],
            'stage8_collapsed_minus_not_calibration':[r for r in matched if r in collapsed and r['target_deg']==8 and r['side']<0]}
    per_peak=Counter(r['peak_ms'] for r in outputs)
    terminal_samples=[s for s in series if s['state_id'] in ['4','5']]
    terminal_ms=number(terminal_samples[0]['t_test_ms']) if terminal_samples else None
    stage_peaks=[p for p in peaks if p['target_peak_deg']==8]
    summary={'case':path.name,'scope':__doc__,'metadata_sha256':hashlib.sha256(blob).hexdigest(),'source_sha256':hashes,
      'terminal':meta['terminal_state'],'counts':{'energy_records':len(rows),'outputs':len(outputs),'normal_outputs':sum(not r['optional'] for r in outputs),
         'early_outputs':sum(r['optional'] for r in outputs),'early_unsent':len(unsent),'peaks':len(peaks),
         'candidate_coast':sum(r['candidate_coast'] for r in rows),'saturated_candidates':sum(r['candidate_saturated'] for r in rows),
         'guard_reductions':sum(r['guard_reduction_ms']>0 for r in rows),'guard_increases':sum(r['guard_reduction_ms']<0 for r in rows),
         'full_sampled_outputs':len(full),'collapsed_before_first_sample':len(collapsed),
         'early_outputs_no_active_samples':sum(r['optional'] and r['active_sample_count']==0 for r in outputs),
         'duplicate_output_peak_ids':sum(n>1 for n in per_peak.values()),
         'early_unsent_later_same_peak_normal':sum(r['same_peak_later_normal_found'] for r in unsent),
         'early_unsent_later_normal_output':sum(r.get('later_normal_executed',False) for r in unsent)},
      'terminal_pulse_timing':{'terminal_sample_ms':terminal_ms,
          'last_programmed_end_ms':max(r['programmed_end_ms'] for r in outputs),
          'programmed_end_after_terminal':sum(r['programmed_end_ms']>terminal_ms for r in outputs) if terminal_ms is not None else None,
          'last_programmed_end_to_terminal_ms':terminal_ms-max(r['programmed_end_ms'] for r in outputs) if terminal_ms is not None else None},
      'peak_sequence':{'max_accepted_peak_interval_ms':max(b-a for a,b in zip(pt,pt[1:])),
          'first_stage8_peak_ms':stage_peaks[0]['peak_time_ms'] if stage_peaks else None,
          'stage8_plus_deg':stats([p['peak_amplitude_deg'] for p in stage_peaks if p['physical_peak_side']>0]),
          'stage8_minus_deg':stats([p['peak_amplitude_deg'] for p in stage_peaks if p['physical_peak_side']<0])},
      'admission_reasons':dict(Counter(r['admission_reason'] for r in rows)),
      'lifecycle_by_start_lag':dict(Counter(str((r['output_start_lag_ms'],r['sampled_lifecycle'])) for r in outputs)),
      'early_executed_width_ms':stats([r['candidate_guarded_ms'] for r in outputs if r['optional']]),
      'early_executed_model_margin_j':stats([r['candidate_work_j']-r['request_j'] for r in outputs if r['optional']]),
      'output_start_lag_ms':stats([r['output_start_lag_ms'] for r in outputs]),
      'collapsed_software_duration_upper_proxy_ms':stats([r['software_duration_upper_proxy_ms'] for r in collapsed]),
      'applied_command_ack_intervals':{'full_sampled_ms':stats([r.get('applied_command_ack_interval_ms') for r in full]),
          'collapsed_ms':stats([r.get('applied_command_ack_interval_ms') for r in collapsed]),
          'collapsed_nonzero_edge_not_sampled':sum(not r['new_nonzero_command_shadow_observed'] for r in collapsed),
          'collapsed_zero_ack_after_snapshot_ms':stats([r['zero_ack_delay_after_preinput_snapshot_ms'] for r in collapsed])},
      'same_peak_normal_followup_delay_ms':stats([r['later_normal_delay_ms'] for r in unsent if r['same_peak_later_normal_found']]),
      'policy_reconstruction':{'n':sum('selected_width_error_ms' in r for r in rows),
          'selected_width_differences':dict(Counter(r['selected_width_error_ms'] for r in rows if 'selected_width_error_ms' in r)),
          'q_error_max_mas':max(abs(r['guarded_q_error_mas']) for r in rows if 'guarded_q_error_mas' in r),
          'work_error_max_j':max(abs(r['guarded_work_error_j']) for r in rows if 'guarded_work_error_j' in r),
          'ceil_failures':sum(not r['ceil_minimum_verified'] for r in rows if 'ceil_minimum_verified' in r)},
      'output_label_checks':{'unsent_nonzero_output_q_or_work':sum((r['output_q_pred_mas']!=0 or r['output_work_pred_j']!=0) for r in rows if not r['executed']),
          'executed_event_width_differs_from_candidate':sum(not r['event_duration_matches_candidate'] for r in outputs),
          'early_admitted_under_model_budget':sum(r['candidate_work_j']+1e-9<r['request_j'] for r in outputs if r['optional'])},
      'current_diagnostics':{'full_outputs_with_fresh_partial_Q':sum(r['partial_absolute_q_diagnostic_mas'] is not None for r in full),
          'collapsed_outputs_with_fresh_partial_Q':sum(r['partial_absolute_q_diagnostic_mas'] is not None for r in collapsed),
          'stale_prior_pulse_Q_rows_ignored':sum(r['stale_prior_pulse_q_rows_ignored'] for r in outputs),
          'partial_Q_semantics':meta['roller_q_meas_observed_semantics'],
          'first_normal_probe_zero_command':[{k:r[k] for k in ['pulse_id','output_start_ms','programmed_width_ms','acknowledged_zero_after_probe_start_us']} for r in outputs if 'acknowledged_zero_after_probe_start_us' in r]},
      'prediction_exclusions':dict(Counter(r['prediction_exclusion'] or 'usable' for r in rows)),
      'prediction_groups':{name:{'n':len(rs),'planned_arrival_error_mj':stats([r['planned_arrival_error_j']*1000 for r in rs]),
                                'observed_peak_deg':stats([r['observed_peak_deg'] for r in rs]),
                                'target_energy_ratio':stats([r['observed_target_energy_ratio'] for r in rs])} for name,rs in groups.items()},
      'peaks_above9p5_after_full_output':[{'input_ms':r['time_ms'],'width_ms':r['candidate_guarded_ms'],'side':r['side'],
                                        'peak_ms':r['next_peak_ms'],'peak_deg':r['observed_peak_deg']} for r in full if r.get('observed_peak_deg',0)>9.5],
      'invalid_and_special_records':[r for r in rows if not r['candidate_valid'] or r['foot_candidate']],
      'limits':['Model prediction is not physically realized Q/work when pulse lifecycle collapsed.',
                'Software-row duration proxies include timestamp quantization and cross-core snapshot skew; they are not strict hardware-edge bounds.',
                'setCurrentMa acknowledges enqueue; nonzero/zero applied-command shadow timestamps, where both sampled, bound device-command acknowledgement interval, not mechanical work.',
                'Zero-command shadow is an acknowledged software command, not measured shaft work or zero residual current.',
                'Admission reason is inferred unless an explicit flag/foot timing reason is available; IMU freshness/deadline rejection is not directly logged.',
                'Moving-foot neutral U and independent ground-forward walking remain unidentified; terminal tilt cause requires video.']}
    (path/'energy_audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf8')
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with (path/'energy_audit.csv').open('w',encoding='utf8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(rows)
    plot(path,rows,peaks)
    print(path.name,json.dumps(summary['counts']),json.dumps(summary['admission_reasons']))
    print('lifecycle',summary['lifecycle_by_start_lag'],'reconstruction',summary['policy_reconstruction'])
    return summary


def plot(path,rows,peaks):
    fig,axs=plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    ax=axs[0,0]
    for side,color in [(1,'tab:blue'),(-1,'tab:orange')]:
        p=[x for x in peaks if x['physical_peak_side']==side]
        ax.plot([x['peak_time_ms']/1000 for x in p],[x['peak_amplitude_deg'] for x in p],'.-',color=color,label=f'peak {side:+}')
    ax.step([p['peak_time_ms']/1000 for p in peaks],[p['target_peak_deg'] for p in peaks],where='post',color='black',ls='--',label='scheduled target')
    ax.set(title='Accepted body peaks (not a foot-contact assessment)',xlabel='Test time [s]',ylabel='Peak amplitude [deg]');ax.legend(fontsize=8)
    ax=axs[0,1];outputs=[r for r in rows if r['executed']]
    ax.plot([r['time_ms']/1000 for r in outputs],[r['programmed_width_ms'] for r in outputs],'.-',color='gray',alpha=.5,label='programmed width')
    for lifecycle,color in [('active_near_programmed_end','tab:blue'),('collapsed_before_first_sample','tab:red')]:
        p=[r for r in outputs if r['sampled_lifecycle']==lifecycle]
        ax.scatter([r['time_ms']/1000 for r in p],[r.get('applied_command_ack_interval_ms',r['software_duration_upper_proxy_ms']) for r in p],s=24,color=color,label=lifecycle)
    ax.set(title='Programmed width versus ACK interval / status-time proxy',xlabel='Test time [s]',ylabel='Duration [ms]');ax.legend(fontsize=7)
    ax=axs[1,0];p=[r for r in rows if r['candidate_valid']]
    ax.plot([r['time_ms']/1000 for r in p],[r['request_j']*1000 for r in p],'.-',label='model request')
    ax.plot([r['time_ms']/1000 for r in p],[r['output_work_pred_j']*1000 for r in p],'.-',label='recorded planned output work')
    ax.set(title='Logged model fields; planned work is NOT realized dose',xlabel='Test time [s]',ylabel='Model energy [mJ]');ax.legend(fontsize=8)
    ax=axs[1,1];p=[r for r in rows if r['prediction_usable']]
    for lifecycle,color,marker in [('active_near_programmed_end','tab:blue','o'),('collapsed_before_first_sample','tab:red','x')]:
        group=[r for r in p if r.get('sampled_lifecycle')==lifecycle]
        ax.scatter([r['candidate_arrival_j']*1000 for r in group],[r['observed_peak_j']*1000 for r in group],s=24,color=color,marker=marker,label=lifecycle)
    top=max([r['candidate_arrival_j']*1000 for r in p]+[r['observed_peak_j']*1000 for r in p])*1.05
    ax.plot([0,top],[0,top],'k--',lw=1);ax.set(xlim=(0,top),ylim=(0,top),aspect='equal',
        title='Next-peak comparison split by observed command lifecycle',xlabel='PLANNED nominal arrival [mJ]',ylabel='Observed nominal peak U [mJ]');ax.legend(fontsize=7)
    fig.suptitle(f'{path.name}: v15 admission and pulse lifecycle audit (no refit from collapsed pulses)')
    fig.savefig(path/'energy_audit.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('cases',nargs='*',default=['59727228','192259301']);args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='v15_trial_energy_') as temp:
        exe,hashes=compile_model(Path(temp))
        for case in args.cases:
            path=Path(case) if Path(case).is_dir() else ROOT/'analysis/2026-11-13-walking'/case
            analyze(path,exe,hashes)


if __name__=='__main__':main()
