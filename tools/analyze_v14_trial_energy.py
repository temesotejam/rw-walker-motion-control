"""Audit recorded v14 decisions and chronological next-peak predictions.

Counterfactual caps are evaluated at the SAME recorded input states. They do
not re-run branch eligibility, timing, sensor delay, or new physical motion.
Observed peak energy uses the firmware's nominal mechanical-zero rocker U;
it does not identify moving-foot neutral angle or ground-forward walking.
"""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "walking_energy_policy.h"
#include <iostream>
#include <iomanip>
namespace e=walking_energy_policy;
void print(const e::Decision& d){
 std::cout<<' '<<d.valid<<' '<<d.width_ms<<' '<<d.q_mas<<' '<<d.predicted_work_j
  <<' '<<d.predicted_arrival_j<<' '<<d.unmet_j<<' '<<d.saturated<<' '<<d.request_j;
}
int main(){
 lateral_energy::Geometry g;unsigned id,cap,width;int side;float rate,angle,target,previous,same,i0,goal,tau;
 std::cout<<std::setprecision(12);
 while(std::cin>>id>>side>>rate>>angle>>target>>previous>>same>>i0>>goal>>tau>>cap>>width){
  lateral_energy::CurrentTable table;if(!table.prepare(tau))return 2;
  e::Input in;in.side=side;in.rate_dps=rate;in.body_angle_deg=angle;in.target_deg=target;
  in.previous_peak_deg=previous;in.same_side_peak_deg=same<0?NAN:same;
  in.initial_current_ma=i0;in.goal_current_ma=goal;in.cap_ms=cap;
  auto selected=e::decide(g,table,in);auto actual=e::evaluateWidth(g,table,in,width);
  in.cap_ms=15;auto c15=e::decide(g,table,in);in.cap_ms=80;auto c80=e::decide(g,table,in);
  std::cout<<id;print(selected);print(actual);print(c15);print(c80);std::cout<<'\n';
 }
}
'''
MODEL_FIELDS=['valid','width_ms','q_mas','work_j','arrival_j','unmet_j','saturated','request_j']


def potential(degrees):
    a=abs(degrees)*math.pi/180;mass_g=.1997*9.80665;h=.120;r=.150;x=.005
    center=math.sqrt(r*r-x*x)
    if a>math.asin(.045/r)+1e-6:return math.nan
    dh=(-2*h*math.sin(.5*a)**2+x*math.sin(a) if a<=math.asin(x/r)
        else r-center+2*(center-h)*math.sin(.5*a)**2)
    return mass_g*dh


def stats(values):
    a=[x for x in values if x is not None and math.isfinite(x)]
    if not a:return {'n':0}
    return {'n':len(a),'mean':statistics.mean(a),'median':statistics.median(a),
            'min':min(a),'max':max(a),'rmse':math.sqrt(statistics.mean(x*x for x in a)),
            'mae':statistics.mean(abs(x) for x in a)}


def energy_totals(rows,prefix=''):
    k=lambda name:prefix+name
    width_key=k('width_ms') if prefix else 'actual_width_ms'
    return {'n':len(rows),'sum_request_mj':sum(r[k('request_j')] for r in rows)*1000,
            'sum_q_mas':sum(r[k('q_mas')] for r in rows),
            'sum_work_mj':sum(r[k('work_j')] for r in rows)*1000,
            'sum_unmet_mj':sum(r[k('unmet_j')] for r in rows)*1000,
            'cap_limited':sum(r[k('saturated')] for r in rows),
            'coast':sum(r[width_key]==0 for r in rows),
            'width_ms':stats([r[width_key] for r in rows])}


def tau_from_metadata(meta):
    return (meta['model_tau_rise_min_ms']+(meta['model_tau_rise_max_ms']-meta['model_tau_rise_min_ms'])/
            (1+(1200/meta['model_tau_rise_u_mA'])**meta['model_tau_rise_exponent']))/1000


def build(executable_dir):
    names=['walking_energy_policy.h','lateral_energy_control.h','realtime_code.h']
    hashes={}
    for name in names:
        blob=(ROOT/'src'/name).read_bytes();(executable_dir/name).write_bytes(blob)
        hashes[name]=hashlib.sha256(blob).hexdigest()
    (executable_dir/'audit.cpp').write_text(CPP,encoding='utf8')
    exe=executable_dir/'audit.exe'
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(executable_dir),
                    str(executable_dir/'audit.cpp'),'-o',str(exe)],check=True)
    return exe,hashes


def model_checks(exe,rows,tau):
    lines=[]
    for i,r in enumerate(rows):
        values=[i,r['side'],r['rate_dps'],r['body_angle_deg'],r['target_deg'],r['previous_peak_deg'],
                r['same_side_peak_deg'] if r['same_side_peak_deg'] is not None else -1,
                r['aligned_i0_ma'],r['goal_current_ma'],tau,r['cap_ms'],r['actual_width_ms']]
        lines.append(' '.join(map(str,values)))
    result=subprocess.run([str(exe)],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
    for line in result.stdout.splitlines():
        cols=line.split();r=rows[int(cols[0])];assert len(cols)==1+4*len(MODEL_FIELDS)
        for i,prefix in enumerate(['reselected_','recomputed_','cap15_','cap80_']):
            for name,value in zip(MODEL_FIELDS,cols[1+i*len(MODEL_FIELDS):1+(i+1)*len(MODEL_FIELDS)]):
                r[prefix+name]=(bool(int(value)) if name in ['valid','saturated'] else
                                int(value) if name=='width_ms' else float(value))
        r['q_recompute_error_mas']=r['recomputed_q_mas']-r['q_mas']
        r['work_recompute_error_j']=r['recomputed_work_j']-r['work_j']
        r['selected_width_recompute_difference_ms']=r['reselected_width_ms']-r['requested_width_ms']
        r['cap80_minus_actual_width_ms']=r['cap80_width_ms']-r['actual_width_ms']


def analyze(path,exe,hashes):
    raw=(path/'metadata.json').read_bytes();meta=json.loads(raw)
    assert meta['diagnostic_build_id']=='foot-support-energy-v14'
    decisions=meta['walking_energy']['records'];events=meta['energy_control_autonomous_zero_cross_events']
    peaks=meta['energy_control_autonomous_peak_events'];times=[p['peak_time_ms'] for p in peaks]
    energy_events=[e for e in events if e['work_control_branch']==19]
    assert len(energy_events)==len(decisions)
    rows=[]
    for d,e in zip(decisions,energy_events):
        t=d['time_ms'];side=d['side']
        # Normal ZERO records retain the interpolated crossing timestamp;
        # the backend audit uses its dispatch time (0..3ms later here).
        assert side==e['physical_next_peak_side'] and 0<=t-e['zero_cross_time_ms']<=5
        old_same=next((p for p in reversed(peaks) if p['peak_time_ms']<t and p['physical_peak_side']==side),None)
        same=(old_same['peak_amplitude_deg'] if old_same and t-old_same['peak_time_ms']<=3000 else None)
        r={'time_ms':t,'zero_cross_time_ms':e['zero_cross_time_ms'],
           'audit_minus_cross_ms':t-e['zero_cross_time_ms'],
           'side':side,'event_kind':e['event_kind'],'optional':d['optional'],'rescue':d['rescue'],
           'executed':d['executed'],'coast':d['coast'],'valid':d['valid'],'saturated':d['saturated'],
           'extrapolated':d['extrapolated'],'passive_rate_supported':d['passive_rate_supported'],
           'work_rate_supported':d['work_rate_supported'],'charge_supported':d['charge_supported'],
           'early_state_transfer':d['early_state_transfer'],'intercept_rolloff':d['passive_intercept_rolloff'],
           'requested_width_ms':d['requested_ms'],'actual_width_ms':d['guarded_ms'],
           'cap_ms':e['work_brake_cap_ms'],'rate_dps':e['zero_cross_abs_rate_dps'],
           'body_angle_deg':e['detector_angle_after_deg'] if d['optional'] or d['rescue'] else 0.,
           'target_deg':e['startup_target_deg'],'previous_peak_deg':e['previous_peak_amplitude_deg'],
           'same_side_peak_deg':same,'aligned_i0_ma':e['aligned_initial_current_mA'],
           'goal_current_ma':e['predicted_current_goal_mA'],'q_mas':d['q_mas'],
           'available_j':d['available_j'],'target_j':d['target_j'],
           'common_correction_j':d['common_correction_j'],'redistribution_j':d['redistribution_j'],
           'request_j':d['request_j'],'work_j':d['predicted_work_j'],'arrival_j':d['predicted_arrival_j'],
           'unmet_j':d['unmet_j'],'pulse_start_ms':e['pulse_start_ms'],'pulse_end_ms':e['pulse_end_ms'],
           'predicted_peak_deg':e['predicted_next_peak_amplitude_deg'],
           'guard_reduction_ms':d['requested_ms']-d['guarded_ms'],
           'accounting_arrival_error_j':d['predicted_arrival_j']-d['available_j']-d['predicted_work_j'],
           'accounting_unmet_error_j':d['unmet_j']-max(0,d['request_j']-d['predicted_work_j'])}
        k=bisect.bisect_right(times,t)
        r['prediction_usable']=False;r['prediction_exclusion']='no_future_peak'
        if k<len(peaks):
            peak=peaks[k];tp=peak['peak_time_ms']
            intervening=[x for x in events if x['output_executed'] and x['pulse_start_ms']>t and x['pulse_start_ms']<tp
                         and x['event_index']!=e['event_index']]
            newer=[x for x in decisions if t<x['time_ms']<tp]
            r.update({'next_peak_time_ms':tp,'next_peak_delay_ms':tp-t,'next_peak_side':peak['physical_peak_side'],
                      'next_peak_deg':peak['peak_amplitude_deg'],'next_peak_energy_j':potential(peak['peak_amplitude_deg']),
                      'next_peak_pending_matched':peak['pending_command_matched'],
                      'intervening_executed_pulses':len(intervening),'intervening_decisions':len(newer),
                      'first_intervening_pulse_ms':intervening[0]['pulse_start_ms'] if intervening else None})
            if intervening:r['prediction_exclusion']='intervening_executed_pulse'
            elif peak['physical_peak_side']!=side:r['prediction_exclusion']='first_peak_wrong_side'
            elif tp-t>1500:r['prediction_exclusion']='peak_after_1500ms'
            elif not peak['pending_command_matched']:r['prediction_exclusion']='pending_not_matched'
            elif peak['pending_q_command_mA_s'] is None or abs(peak['pending_q_command_mA_s']-d['q_mas'])>.0001:
                r['prediction_exclusion']='pending_Q_mismatch'
            elif newer:r['prediction_exclusion']='pending_overwritten_by_new_decision'
            else:
                r['prediction_usable']=True;r['prediction_exclusion']=''
                r['arrival_error_j']=r['next_peak_energy_j']-r['arrival_j']
                r['peak_error_deg']=peak['peak_amplitude_deg']-r['predicted_peak_deg']
                r['observed_target_energy_ratio']=r['next_peak_energy_j']/r['target_j']
                r['observed_minus_available_j']=r['next_peak_energy_j']-r['available_j']
        rows.append(r)
    model_checks(exe,rows,tau_from_metadata(meta))
    matched=[r for r in rows if r['prediction_usable']]
    optional=[r for r in rows if r['optional']]
    stage8=[r for r in rows if r['target_deg']==8]
    stage8matched=[r for r in matched if r['target_deg']==8]
    groups={'all':matched,'plus':[r for r in matched if r['side']>0],'minus':[r for r in matched if r['side']<0],
            'optional':[r for r in matched if r['optional']],'normal':[r for r in matched if not r['optional']],
            'stage8':stage8matched,'stage8_plus':[r for r in stage8matched if r['side']>0],
            'stage8_minus':[r for r in stage8matched if r['side']<0]}
    prediction={name:{'arrival_error_mj':stats([r['arrival_error_j']*1000 for r in group]),
                      'peak_error_deg':stats([r['peak_error_deg'] for r in group]),
                      'observed_peak_deg':stats([r['next_peak_deg'] for r in group]),
                      'observed_target_energy_ratio':stats([r['observed_target_energy_ratio'] for r in group])}
                for name,group in groups.items()}
    counterfactual={name:{'actual':energy_totals(group),'cap15':energy_totals(group,'cap15_'),
                         'cap80':energy_totals(group,'cap80_'),
                         'cap80_larger_than_actual':sum(r['cap80_width_ms']>r['actual_width_ms'] for r in group)}
                    for name,group in [('all',rows),('optional',optional),('stage8',stage8),
                                       ('stage8_optional',[r for r in stage8 if r['optional']])]}
    stage_progression=[]
    for stage in sorted({p['target_peak_deg'] for p in peaks}):
        ps=[p for p in peaks if p['target_peak_deg']==stage]
        stage_progression.append({'target_deg':stage,'first_peak_ms':ps[0]['peak_time_ms'],
                                  'last_peak_ms':ps[-1]['peak_time_ms'],'n':len(ps),
                                  'plus_deg':stats([p['peak_amplitude_deg'] for p in ps if p['physical_peak_side']>0]),
                                  'minus_deg':stats([p['peak_amplitude_deg'] for p in ps if p['physical_peak_side']<0])})
    summary={'case':path.name,'build':meta['diagnostic_build_id'],'scope':__doc__,
             'source_sha256':hashes,'metadata_sha256':hashlib.sha256(raw).hexdigest(),
             'counts':{'energy_records':len(rows),'peaks':len(peaks),'events':len(events),
                       **{k:sum(bool(r[k]) for r in rows) for k in ['executed','optional','rescue','coast','saturated',
                          'extrapolated','passive_rate_supported','work_rate_supported','charge_supported',
                          'early_state_transfer','intercept_rolloff']},
                       'normal':sum(not r['optional'] for r in rows),
                       'guard_reductions':sum(r['guard_reduction_ms']>0 for r in rows),
                       'guard_increases':sum(r['guard_reduction_ms']<0 for r in rows),
                       'chronological_prediction_pairs':len(matched)},
             'prediction_exclusions':dict(Counter(r['prediction_exclusion'] or 'usable' for r in rows)),
             'event_width_counts':dict(Counter(str((r['event_kind'],r['actual_width_ms'])) for r in rows)),
             'energy_equation_absolute_errors':{k:max(abs(r[k]) for r in rows) for k in
                ['accounting_arrival_error_j','accounting_unmet_error_j','q_recompute_error_mas','work_recompute_error_j']},
             'reselected_width_differences':dict(Counter(r['selected_width_recompute_difference_ms'] for r in rows)),
             'foot_support':{'stored':meta['foot_support_assist']['stored'],'total':meta['foot_support_assist']['total'],
                             'check_reasons':dict(zip(meta['foot_support_assist']['reason_names'],meta['foot_support_assist']['candidate_check_counts']))},
             'prediction':prediction,'counterfactual_caps':counterfactual,'stage_progression':stage_progression,
             'coast_records':[r for r in rows if r['coast']],
             'limits':['Nominal mechanical-zero rocker U, not moving-foot neutral energy.',
                       'Pending match and no intervening command establish a bookkeeping comparison, not isolated causal identification.',
                       'Passive fit is conditional on previous controller coast; powered response is an inherited empirical transfer.',
                       'Cap comparisons hold measured states fixed and do not simulate future gait or authorize execution.',
                       'An alternating accepted body peak sequence does not establish alternating foot contact or forward progress.']}
    (path/'energy_audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf8')
    columns=list(dict.fromkeys(k for r in rows for k in r))
    with (path/'energy_audit.csv').open('w',encoding='utf8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(rows)
    plot(path,rows,peaks)
    print(path.name,json.dumps(summary['counts']),json.dumps(summary['prediction_exclusions']))
    for name in ['all','stage8_plus','stage8_minus']:
        print(name,prediction[name])
    return summary


def plot(path,rows,peaks):
    fig,axs=plt.subplots(2,2,figsize=(14,9),constrained_layout=True)
    ax=axs[0,0]
    for side,color in [(1,'tab:blue'),(-1,'tab:orange')]:
        p=[x for x in peaks if x['physical_peak_side']==side]
        ax.plot([x['peak_time_ms']/1000 for x in p],[x['peak_amplitude_deg'] for x in p],'.-',color=color,label=f'peak side {side:+}')
    ax.step([p['peak_time_ms']/1000 for p in peaks],[p['target_peak_deg'] for p in peaks],where='post',color='black',ls='--',label='scheduled target')
    ax.set(title='Accepted body peaks and scheduled target',xlabel='Test time [s]',ylabel='Peak amplitude [deg]');ax.legend(fontsize=8)
    t=[r['time_ms']/1000 for r in rows];ax=axs[0,1]
    ax.plot(t,[r['request_j']*1000 for r in rows],'.-',label='requested new work')
    ax.plot(t,[r['work_j']*1000 for r in rows],'.-',label='predicted actual pulse work')
    ax.plot(t,[r['unmet_j']*1000 for r in rows],'.-',label='unmet after actual guard')
    ax.set(title='Logged model energy at each decision',xlabel='Test time [s]',ylabel='Energy [mJ]');ax.legend(fontsize=8)
    ax=axs[1,0]
    ax.plot(t,[r['cap80_width_ms'] for r in rows],'.-',color='tab:green',alpha=.65,label='cap80 at same recorded state')
    ax.plot(t,[r['actual_width_ms'] for r in rows],'o',ms=3,color='black',label='actual width')
    ax.axhline(15,color='tab:red',ls=':',label='optional timing cap15')
    ax.set(title='Actual inputs and cap-only counterfactual',xlabel='Test time [s]',ylabel='Width [ms]');ax.legend(fontsize=8)
    ax=axs[1,1];matched=[r for r in rows if r['prediction_usable']]
    for side,color in [(1,'tab:blue'),(-1,'tab:orange')]:
        for optional,marker in [(True,'o'),(False,'x')]:
            p=[r for r in matched if r['side']==side and r['optional']==optional]
            ax.scatter([r['arrival_j']*1000 for r in p],[r['next_peak_energy_j']*1000 for r in p],s=24,
                       color=color,marker=marker,label=f'{side:+} '+('early' if optional else 'normal'))
    top=max([r['arrival_j']*1000 for r in matched]+[r['next_peak_energy_j']*1000 for r in matched])*1.05
    ax.plot([0,top],[0,top],'k--',lw=1);ax.set(xlim=(0,top),ylim=(0,top),aspect='equal',
        title='First next peak; matching pending Q, no intervening pulse',xlabel='Predicted nominal arrival energy [mJ]',ylabel='Observed nominal peak energy [mJ]')
    ax.legend(fontsize=8)
    fig.suptitle(f'{path.name}: v14 energy audit (same-state cap comparison; not a gait simulation)')
    fig.savefig(path/'energy_audit.png',dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('cases',nargs='*',default=['172594684','294107213'])
    args=p.parse_args()
    with tempfile.TemporaryDirectory(prefix='v14_trial_energy_') as temp:
        exe,hashes=build(Path(temp))
        for case in args.cases:
            path=Path(case) if Path(case).is_dir() else ROOT/'analysis/2026-11-13-walking'/case
            analyze(path,exe,hashes)


if __name__=='__main__':main()
