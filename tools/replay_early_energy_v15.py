"""Compare v14 and v15 input sizing at recorded decision opportunities.

The recorded trajectory, target schedule, inputs and wheel/peak history remain
fixed. Deferred opportunities do not generate a new trajectory or a later ZERO
command here. Recorded-next-peak timing is a margin marker, not validation of
the changed pulse, gait, foot contact or forward travel.
"""
import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
CPP=r'''
#include "walking_energy_policy_v15.h"
#include "wheel_cycle_guard.h"
#include "tilt_priority.h"
#include <iostream>
#include <iomanip>
namespace v14=walking_energy_policy;
namespace v15=walking_energy_policy_v15;
void print(const v14::Decision& d){std::cout<<' '<<d.valid<<' '<<d.width_ms<<' '<<d.request_j
 <<' '<<d.q_mas<<' '<<d.predicted_work_j<<' '<<d.unmet_j<<' '<<d.saturated<<' '<<d.coast
 <<' '<<d.extrapolated<<' '<<d.rate_supported<<' '<<d.charge_supported<<' '<<d.early_state_transfer;}
int main(){
 lateral_energy::Geometry geometry;lateral_energy::CurrentTable table;
 wheel_cycle_guard::Controller wheel;tilt_priority::Controller timing;
 char kind;std::cout<<std::setprecision(12);
 while(std::cin>>kind){
  if(kind=='T'){float tau;std::cin>>tau;if(!table.prepare(tau))return 2;}
  else if(kind=='P'){int side,matched;unsigned ms;float angle;
   std::cin>>side>>ms>>angle>>matched;wheel.peak(side,angle,ms,matched);timing.peak(side,ms,angle);}
  else if(kind=='C'){int side,fresh,executed,optional,valid;unsigned ms,width;float rpm,target;
   std::cin>>side>>ms>>width>>rpm>>fresh>>target>>executed>>optional>>valid;
   if(valid && (executed || !optional)){
    if(executed)wheel.limit(side,rpm,fresh,ms,width);
    tilt_priority::Choice c;c.selected=true;c.direction=side;c.coast=!executed;
    c.width_ms=width;c.current_ma=executed?1200:0;c.target_deg=target;
    timing.crossing(side,ms);timing.confirm(side,ms,c,true,true,false);
   }
  }else if(kind=='D'){
   unsigned id,ms;int side,early,rescue,foot,fresh;float rate,angle,target,previous,same,i0,goal,rpm;
   std::cin>>id>>ms>>side>>early>>rescue>>foot>>rate>>angle>>target>>previous>>same>>i0>>goal>>rpm>>fresh;
   v14::Input in;in.side=side;in.rate_dps=rate;in.body_angle_deg=angle;in.target_deg=target;
   in.previous_peak_deg=previous;in.same_side_peak_deg=same<0?NAN:same;
   in.initial_current_ma=i0;in.goal_current_ma=goal;
   const unsigned cap=timing.timingCap(side,ms,rescue?15:80);
   in.cap_ms=early && !foot && !rescue && cap>15?15:cap;
   const auto old=v14::decide(geometry,table,in);auto old_guard=wheel;
   const auto old_width=old.valid && !old.coast?old_guard.limit(side,rpm,fresh,ms,old.width_ms):0;
   const auto old_actual=v14::evaluateWidth(geometry,table,in,old_width);
   in.cap_ms=cap;const auto candidate=v15::decide(geometry,table,in);auto new_guard=wheel;
   const auto actual_width=candidate.valid && !candidate.coast?new_guard.limit(side,rpm,fresh,ms,candidate.width_ms):0;
   const auto actual=v15::evaluateWidth(geometry,table,in,actual_width);
   std::cout<<id<<' '<<cap;print(old);print(old_actual);print(candidate);print(actual);
   std::cout<<' '<<v15::earlyAdmissible(actual)<<'\n';
  }else return 3;
 }
}
'''
FIELDS=['valid','width_ms','request_j','q_mas','work_j','unmet_j','saturated','coast',
        'extrapolated','rate_supported','charge_supported','early_state_transfer']


def find_case(case):
    direct=Path(case)
    if direct.is_dir():return direct
    for directory in ['2026-11-13-walking','2026-11-11-walking']:
        path=ROOT/'analysis'/directory/case
        if (path/'metadata.json').exists():return path
    raise FileNotFoundError(case)


def finite(x):return isinstance(x,(float,int)) and math.isfinite(x)


def current_goal(meta,volts):
    sat=meta['model_i_sat_at_reference_mA']+meta['model_i_sat_slope_mA_per_v']*(volts-meta['model_vbat_reference_v'])
    return 1200/(1+(1200/sat)**meta['model_i_sat_exponent'])**(1/meta['model_i_sat_exponent'])


def rise_tau(meta):
    return (meta['model_tau_rise_min_ms']+(meta['model_tau_rise_max_ms']-meta['model_tau_rise_min_ms'])/
            (1+(1200/meta['model_tau_rise_u_mA'])**meta['model_tau_rise_exponent']))/1000


def load_case(path):
    raw=(path/'metadata.json').read_bytes();meta=json.loads(raw)
    events=meta['energy_control_autonomous_zero_cross_events'];peaks=meta['energy_control_autonomous_peak_events']
    peak_times=[p['peak_time_ms'] for p in peaks]
    audits=meta.get('walking_energy',{}).get('records',[])
    foot_times={f['time_ms'] for f in meta.get('foot_support_assist',{}).get('records',[])}
    energy_events=[e for e in events if e.get('work_control_branch')==19]
    audit_by_index={e['event_index']:a for e,a in zip(energy_events,audits)}
    if audits:assert len(audits)==len(energy_events)
    stream=[];rows=[];skipped=Counter()
    for peak in peaks:
        t=peak['peak_time_ms']
        stream.append((t,0,f"P {peak['physical_peak_side']} {t} {peak['peak_amplitude_deg']} {int(peak['pending_command_matched'])}"))
    for e in events:
        if e['event_kind']=='strong_start_kick':skipped['startup_fixed_kick']+=1;continue
        a=audit_by_index.get(e['event_index']);t=a['time_ms'] if a else e['zero_cross_time_ms']
        side=e['physical_next_peak_side'];kind=e['event_kind']
        early=bool(a['optional']) if a else kind!='energy_control_zero_cross'
        rescue=bool(a['rescue']) if a else 'rescue' in kind
        foot=t in foot_times or e['zero_cross_time_ms'] in foot_times or e.get('work_control_branch') in [16,17,18]
        previous=e.get('previous_peak_amplitude_deg');rate=e.get('zero_cross_abs_rate_dps')
        target=e.get('startup_target_deg');goal=e.get('predicted_current_goal_mA')
        if not finite(goal) and e.get('vbat_mV',0)>0:goal=current_goal(meta,e['vbat_mV']/1000)
        angle=e.get('detector_angle_after_deg') if early or rescue else 0.
        i0=e.get('aligned_initial_current_mA')
        if not finite(i0) and finite(e.get('i0_estimated_mA')):i0=side*e['i0_estimated_mA']
        if not finite(target):
            k=bisect.bisect_left(peak_times,t)-1
            target=peaks[k]['target_peak_deg'] if k>=0 else None
        if not all(finite(x) for x in [previous,rate,target,goal,angle,i0]):skipped['missing_pre_input']+=1;continue
        prior=next((p for p in reversed(peaks) if p['peak_time_ms']<t and p['physical_peak_side']==side),None)
        same=prior['peak_amplitude_deg'] if prior and t-prior['peak_time_ms']<=3000 else -1
        rpm=e.get('pre_input_wheel_speed_rpm');fresh=bool(e.get('pre_input_wheel_speed_valid')) and finite(rpm) and e.get('pre_input_wheel_speed_age_us',999999)<=50000
        if not finite(rpm):rpm=0.
        incoming=e.get('previous_peak_time_ms',0)
        state_fresh=bool(incoming) and 0<=t-incoming<=1500
        row={'case':path.name,'event_index':e['event_index'],'time_ms':t,'event_kind':kind,'early':early,'rescue':rescue,'foot':foot,
             'side':side,'rate_dps':rate,'body_angle_deg':angle,'target_deg':target,'previous_peak_deg':previous,
             'same_side_peak_deg':same if same>=0 else None,'aligned_i0_ma':i0,'goal_current_ma':goal,
             'wheel_rpm':rpm,'wheel_fresh':fresh,'incoming_peak_fresh':state_fresh,
             'recorded_width_ms':e['pulse_width_ms'],'recorded_executed':e['output_executed'],
             'recorded_peak_timestamp_semantics':'first accepted next peak on the old trajectory; not a new-pulse prediction'}
        k=bisect.bisect_right(peak_times,t)
        if k<len(peaks):
            row['recorded_next_peak_time_ms']=peaks[k]['peak_time_ms'];row['recorded_next_peak_side']=peaks[k]['physical_peak_side']
            row['recorded_next_peak_delay_ms']=peaks[k]['peak_time_ms']-t
            row['recorded_next_peak_matches_intended']=peaks[k]['physical_peak_side']==side
        ident=len(rows);rows.append(row)
        stream.append((t,1,f'D {ident} {t} {side} {int(early)} {int(rescue)} {int(foot)} {rate} {angle} {target} {previous} {same} {i0} {goal} {rpm} {int(fresh)}'))
        stream.append((t,2,f"C {side} {t} {e['pulse_width_ms']} {rpm} {int(fresh)} {target} {int(e['output_executed'])} {int(early or rescue)} {int(e['valid'])}"))
    return meta,rows,skipped,[item[2] for item in sorted(stream,key=lambda x:(x[0],x[1]))],hashlib.sha256(raw).hexdigest()


def parse_output(output,rows):
    for line in output.splitlines():
        cols=line.split();assert len(cols)==3+4*len(FIELDS)
        row=rows[int(cols[0])];row['timing_cap_ms']=int(cols[1])
        for i,prefix in enumerate(['v14_','v14_guarded_','v15_','v15_guarded_']):
            for key,value in zip(FIELDS,cols[2+i*len(FIELDS):2+(i+1)*len(FIELDS)]):
                number=float(value)
                row[prefix+key]=(bool(int(number)) if key in ['valid','saturated','coast','extrapolated','rate_supported','charge_supported','early_state_transfer'] else int(number) if key=='width_ms'
                                 else number if math.isfinite(number) else None)
        row['v15_early_budget_admissible']=bool(int(cols[-1]))
        if not row['incoming_peak_fresh'] or not row['wheel_fresh']:action='invalid_recorded_precondition'
        elif not row['v15_guarded_valid']:action='invalid_model_or_guard'
        elif row['v15_guarded_coast']:action='coast_no_early_pulse'
        elif row['early'] and not row['rescue'] and not row['v15_early_budget_admissible']:
            action='defer_guard_budget' if row['v15_guarded_width_ms']<row['v15_width_ms'] and not row['v15_saturated'] else 'defer_infeasible_budget'
        else:action='energy_input_candidate'
        row['v15_action']=action
        row['ordinary_opportunity_retained']=bool(row['early'] and action!='energy_input_candidate')
        if 'recorded_next_peak_delay_ms' in row:
            row['recorded_peak_margin_after_v15_width_ms']=row['recorded_next_peak_delay_ms']-row['v15_guarded_width_ms']
    return rows


def group_summary(rows):
    allowed=[r for r in rows if r['v15_action']=='energy_input_candidate']
    margins=[r['recorded_peak_margin_after_v15_width_ms'] for r in allowed if r.get('recorded_next_peak_matches_intended')]
    deferred=[r for r in rows if r['v15_action'] in ['defer_guard_budget','defer_infeasible_budget']]
    return {'n':len(rows),'actions':dict(Counter(r['v15_action'] for r in rows)),
            'v14_guarded_budget_covered':sum(r['v14_guarded_valid'] and r['v14_guarded_work_j']+1e-9>=r['v14_guarded_request_j'] and not r['v14_guarded_coast'] for r in rows),
            'v15_admissible_width_ms':{'min':min((r['v15_guarded_width_ms'] for r in allowed),default=None),
                                     'max':max((r['v15_guarded_width_ms'] for r in allowed),default=None)},
            'v15_candidate_above15':sum(r['v15_guarded_width_ms']>15 for r in allowed),
            'v15_candidate_extrapolated':sum(r['v15_guarded_extrapolated'] for r in allowed),
            'v15_candidate_rate_unsupported':sum(not r['v15_guarded_rate_supported'] for r in allowed),
            'v15_candidate_q_unsupported':sum(not r['v15_guarded_charge_supported'] for r in allowed),
            'sum_v14_guarded_q_mas':sum(r['v14_guarded_q_mas'] for r in rows if r['v14_guarded_q_mas'] is not None),
            'sum_v15_admissible_q_mas':sum(r['v15_guarded_q_mas'] for r in allowed),
            'sum_v14_guarded_unmet_mj':sum(r['v14_guarded_unmet_j']*1000 for r in rows if r['v14_guarded_unmet_j'] is not None),
            'sum_v15_admissible_unmet_mj':sum(r['v15_guarded_unmet_j']*1000 for r in allowed),
            'sum_v15_deferred_request_mj':sum(r['v15_guarded_request_j']*1000 for r in deferred),
            'min_recorded_peak_delay_ms':min((r['recorded_next_peak_delay_ms'] for r in allowed if 'recorded_next_peak_delay_ms' in r),default=None),
            'min_recorded_peak_margin_after_width_ms':min(margins,default=None),
            'recorded_margin_under30ms':sum(x<30 for x in margins),
            'ordinary_opportunity_retained':sum(r['ordinary_opportunity_retained'] for r in rows)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cases',nargs='+',default=['172594684','294107213'])
    p.add_argument('--out',type=Path);args=p.parse_args();results={}
    headers=['walking_energy_policy_v15.h','walking_energy_policy.h','lateral_energy_control.h','realtime_code.h','wheel_cycle_guard.h','tilt_priority.h']
    sources={name:(ROOT/'src'/name).read_bytes() for name in headers}
    hashes={name:hashlib.sha256(blob).hexdigest() for name,blob in sources.items()}
    with tempfile.TemporaryDirectory(prefix='early_energy_v15_') as temp:
        temp=Path(temp)
        for name,blob in sources.items():(temp/name).write_bytes(blob)
        (temp/'replay.cpp').write_text(CPP,encoding='utf8');exe=temp/'replay.exe'
        subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(temp),str(temp/'replay.cpp'),'-o',str(exe)],check=True)
        for case in args.cases:
            path=find_case(case);meta,rows,skipped,lines,data_hash=load_case(path)
            result=subprocess.run([str(exe)],input=f'T {rise_tau(meta)}\n'+'\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
            parse_output(result.stdout,rows)
            assert all(not (r['early'] and not r['rescue'] and r['v15_action']=='energy_input_candidate') or
                       (r['v15_early_budget_admissible'] and 0<r['v15_guarded_width_ms']<=80 and
                        r['v15_guarded_work_j']+1e-9>=r['v15_guarded_request_j']) for r in rows)
            summary={'all':group_summary(rows),'early':group_summary([r for r in rows if r['early'] and not r['rescue']]),
                     'stage8_early':group_summary([r for r in rows if r['early'] and not r['rescue'] and r['target_deg']==8]),
                     'stage8_early_plus':group_summary([r for r in rows if r['early'] and not r['rescue'] and r['target_deg']==8 and r['side']>0]),
                     'stage8_early_minus':group_summary([r for r in rows if r['early'] and not r['rescue'] and r['target_deg']==8 and r['side']<0]),
                     'rescue':group_summary([r for r in rows if r['rescue']]),'skipped':dict(skipped)}
            results[path.name]={'metadata_sha256':data_hash,'summary':summary,'decisions':rows}
            brief={name:{k:summary[name][k] for k in ['n','actions','v15_admissible_width_ms',
                    'v15_candidate_extrapolated','min_recorded_peak_margin_after_width_ms']}
                   for name in ['early','stage8_early']}
            brief['stage8_side_actions']={str(side):summary[name]['actions'] for side,name in
                                          [(1,'stage8_early_plus'),(-1,'stage8_early_minus')]}
            brief['skipped']=summary['skipped'];print(path.name,json.dumps(brief,allow_nan=False))
    artifact={'scope':__doc__,'source_sha256':hashes,'cases':results,
              'energy_origin':'unchanged nominal mechanical-zero rocker U; moving-foot neutral point is not identified',
              'history':'only old recorded commands update timing/wheel guard; new candidates do not generate new history',
              'not_replayed':['complete generic/foot timing eligibility and source selection',
                              '5ms dispatch deadline and 10ms IMU-sample guards under new processing time',
                              'transport write latency, later ordinary ZERO or new physical motion']}
    if args.out:
        args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(artifact,indent=2,allow_nan=False),encoding='utf8')


if __name__=='__main__':main()
