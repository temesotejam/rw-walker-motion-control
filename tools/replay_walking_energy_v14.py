"""Energy-v14 snapshot audit on recorded trajectories, not new closed-loop motion.

Each old decision supplies its preceding peak, recorded pre-output rate/current/
battery and previous same-side peak. Normal ZERO uses body angle0 as the real
v14 runner does; early opportunities use the saved support audit or latest past
body sample. Old
commands/peaks force-update only the wheel guard. The future response to a new
width, full branch eligibility and precise scheduler timing are not simulated.
"""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
CASES=['97855607','241508524','611622951','141282685','43750941',
       '137267573','267557373','27908524','64040782','84815132','57814129','199015154']
FIELDS=['valid','coast','reason','width_ms','guarded_width_ms','peak_energy_j',
        'same_side_peak_energy_j','potential_j','effective_kinetic_j','fitted_offset_j',
        'available_j','target_j','peak_correction_j','deficit_j','request_j','q_mas',
        'predicted_work_j','predicted_arrival_j','unmet_j','maximum_work_j','saturated',
        'rate_supported','charge_supported','extrapolated','early_state_transfer',
        'coast_residual_q_mas','coast_residual_modelled','inbound_loss_proxy_j',
        'guarded_valid','guarded_q_mas','guarded_work_j','guarded_arrival_j','guarded_unmet_j',
        'passive_rate_supported','work_rate_supported','passive_intercept_rolloff','passive_omega2_coefficient',
        'mean_peak_energy_j','side_imbalance_j','common_correction_j','redistribution_j']
BOOL={'valid','coast','saturated','rate_supported','charge_supported','extrapolated',
      'early_state_transfer','coast_residual_modelled','guarded_valid',
      'passive_rate_supported','work_rate_supported','passive_intercept_rolloff'}
INTEGER={'reason','width_ms','guarded_width_ms'}
CPP=r'''
#include "walking_energy_policy.h"
#include "wheel_cycle_guard.h"
#include <iostream>
#include <iomanip>
#include <cmath>
int main(){
 lateral_energy::Geometry geometry;lateral_energy::CurrentTable table;
 wheel_cycle_guard::Controller wheel;char kind;std::cout<<std::setprecision(10);
 while(std::cin>>kind){
  if(kind=='T'){float tau;std::cin>>tau;if(!table.prepare(tau))return 3;}
  else if(kind=='M'){for(int side:{-1,1}){const auto p=walking_energy_policy::passiveCoefficients(side);
   const auto w=lateral_energy::coefficients(side);
   std::cout<<side<<' '<<p.offset_j<<' '<<p.omega2_j<<' '<<p.rate_min_dps<<' '<<p.rate_max_dps
    <<' '<<w.rate_min_dps<<' '<<w.rate_max_dps<<' '<<w.charge_max_mAs<<'\n';}}
  else if(kind=='P'){int side,matched;float angle;unsigned ms;
   std::cin>>side>>angle>>ms>>matched;wheel.peak(side,angle,ms,matched);}
  else if(kind=='C'){int side,fresh;float rpm;unsigned ms,width;
   std::cin>>side>>rpm>>fresh>>ms>>width;wheel.limit(side,rpm,fresh,ms,width);}
  else if(kind=='D'){
   unsigned id,now,cap;int side,fresh;float rate,angle,target,previous,same,i0,goal,rpm;
   std::cin>>id>>now>>side>>rate>>angle>>target>>previous>>same>>i0>>goal>>cap>>rpm>>fresh;
   walking_energy_policy::Input in;in.side=side;in.rate_dps=rate;in.body_angle_deg=angle;
   in.target_deg=target;in.previous_peak_deg=previous;in.same_side_peak_deg=same<0?NAN:same;
   in.initial_current_ma=i0;in.goal_current_ma=goal;in.cap_ms=cap;
   auto d=walking_energy_policy::decide(geometry,table,in);auto preview=wheel;
   const unsigned guarded=d.valid?preview.limit(side,rpm,fresh,now,d.width_ms):0;
   const auto actual=walking_energy_policy::evaluateWidth(geometry,table,in,guarded);
   std::cout<<id<<' '<<d.valid<<' '<<d.coast<<' '<<int(d.reason)<<' '<<d.width_ms<<' '<<guarded<<' '
    <<d.peak_energy_j<<' '<<d.same_side_peak_energy_j<<' '<<d.potential_j<<' '<<d.effective_kinetic_j<<' '
    <<d.fitted_offset_j<<' '<<d.available_j<<' '<<d.target_j<<' '<<d.peak_correction_j<<' '
    <<d.deficit_j<<' '<<d.request_j<<' '<<d.q_mas<<' '<<d.predicted_work_j<<' '<<d.predicted_arrival_j<<' '
    <<d.unmet_j<<' '<<d.maximum_work_j<<' '<<d.saturated<<' '<<d.rate_supported<<' '<<d.charge_supported<<' '
    <<d.extrapolated<<' '<<d.early_state_transfer<<' '<<d.coast_residual_q_mas<<' '<<d.coast_residual_modelled<<' '
    <<d.inbound_loss_proxy_j<<' '<<actual.valid<<' '<<actual.q_mas<<' '<<actual.predicted_work_j<<' '
    <<actual.predicted_arrival_j<<' '<<actual.unmet_j<<' '<<d.passive_rate_supported<<' '
    <<d.work_rate_supported<<' '<<d.passive_intercept_rolloff<<' '<<d.passive_omega2_coefficient<<' '
    <<d.mean_peak_energy_j<<' '<<d.side_imbalance_j<<' '<<d.common_correction_j<<' '<<d.redistribution_j<<'\n';
  }else return 4;
 }
}
'''


def read(path):
    with path.open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))


def number(x, default=math.nan):
    try:return float(x)
    except (ValueError,TypeError):return default


def find_case(case):
    for date in ['2026-11-13-walking','2026-11-11-walking']:
        p=ROOT/'analysis'/date/case
        if (p/'timeseries.csv').exists():return p
    raise FileNotFoundError(case)


def current_config():
    source=(ROOT/'src/config.h').read_text(encoding='utf8')
    names=['MODEL_I_SAT_AT_REFERENCE_MA','MODEL_I_SAT_SLOPE_MA_PER_V','MODEL_I_SAT_EXPONENT',
           'MODEL_VBAT_REFERENCE_V','MODEL_TAU_RISE_MIN_MS','MODEL_TAU_RISE_MAX_MS',
           'MODEL_TAU_RISE_U_MA','MODEL_TAU_RISE_EXPONENT']
    values={n:float(re.search(r'\b'+n+r'\s*=\s*([0-9.]+)f',source).group(1)) for n in names}
    return values


def goal_current(cfg,volts):
    sat=cfg['MODEL_I_SAT_AT_REFERENCE_MA']+cfg['MODEL_I_SAT_SLOPE_MA_PER_V']*(volts-cfg['MODEL_VBAT_REFERENCE_V'])
    return 1200/(1+(1200/sat)**cfg['MODEL_I_SAT_EXPONENT'])**(1/cfg['MODEL_I_SAT_EXPONENT'])


def rise_tau(cfg):
    return (cfg['MODEL_TAU_RISE_MIN_MS']+(cfg['MODEL_TAU_RISE_MAX_MS']-cfg['MODEL_TAU_RISE_MIN_MS'])/
            (1+(1200/cfg['MODEL_TAU_RISE_U_MA'])**cfg['MODEL_TAU_RISE_EXPONENT']))/1000


def snapshot_data(path,cfg):
    events=read(path/'energy_control_autonomous_zero_cross_events.csv')
    peaks=read(path/'energy_control_autonomous_peak_events.csv')
    samples=[r for r in read(path/'timeseries.csv') if r['state_id']=='3' and number(r['t_test_ms'])>=0]
    times=[number(r['t_test_ms']) for r in samples]
    meta=json.loads((path/'metadata.json').read_text(encoding='utf8'))
    support={r['time_ms']:r for r in meta.get('foot_support_assist',{}).get('records',[])}
    peaktimes=[number(r['peak_time_ms']) for r in peaks]
    records=[];skipped=Counter();stream=[]
    for peak in peaks:
        ms=int(number(peak['peak_time_ms']));side=int(peak['physical_peak_side'])
        stream.append((ms,0,f"P {side} {peak['peak_amplitude_deg']} {ms} {int(peak['pending_command_matched']=='True')}"))
    for event in events:
        ms=number(event['zero_cross_time_ms']);side=int(event['physical_next_peak_side'])
        if event['event_kind']=='strong_start_kick':skipped['start_kick_no_preceding_peak']+=1;continue
        previous=number(event['previous_peak_amplitude_deg']);rate=number(event['zero_cross_abs_rate_dps'])
        battery=number(event['vbat_mV']);i0=number(event['i0_estimated_mA'])
        j=bisect.bisect_right(times,ms)-1;k=bisect.bisect_left(peaktimes,ms)-1
        if j<0 or k<0 or not all(math.isfinite(x) for x in [ms,previous,rate,battery,i0]) or battery<=0:
            skipped['missing_pre_input_or_past_state']+=1;continue
        prior=samples[j];stage=number(peaks[k]['target_peak_deg'])
        same_peak=next((p for p in reversed(peaks[:k+1]) if int(p['physical_peak_side'])==side),None)
        same=number(same_peak['peak_amplitude_deg']) if same_peak and ms-number(same_peak['peak_time_ms'])<=3000 else -1
        exact=support.get(int(ms))
        if exact:angle=exact['body_angle_deg'];angle_source='saved_support_audit'
        elif event['event_kind']=='energy_control_zero_cross':angle=0.;angle_source='recorded_zero_cross'
        else:angle=number(prior['pitch_mekf_detector_relative_deg']);angle_source='latest_past_body_sample'
        cap=80 if event['event_kind']=='energy_control_zero_cross' or exact else 15
        rpm=number(event['pre_input_wheel_speed_rpm']);wheel_valid=(event['pre_input_wheel_speed_valid']=='True'
                    and number(event['pre_input_wheel_speed_age_us'])<=50000 and math.isfinite(rpm))
        if not math.isfinite(rpm):rpm=0
        ident=len(records)
        record={'case':path.name,'time_ms':int(ms),'event_kind':event['event_kind'],
                'old_decision_valid':event['valid']=='True',
                'old_output_executed':event['output_executed']=='True','old_width_ms':int(event['pulse_width_ms']),
                'old_branch':event['work_control_branch'],'side':side,'body_angle_deg':angle,
                'body_source':angle_source,'past_body_sample_age_ms':ms-times[j],
                'rate_dps':rate,'previous_peak_deg':previous,'same_side_peak_deg':same if same>=0 else None,
                'target_deg':stage,'target_semantics':'last recorded peak stage; discrete ramp update may differ by one step',
                'cap_ms':cap,'aligned_i0_ma':side*i0,'goal_current_ma':goal_current(cfg,battery/1000),
                'battery_mv':battery,'wheel_rpm':rpm,'wheel_fresh':wheel_valid,
                'state_fresh':ms-times[j]<=10 or exact is not None or angle_source=='recorded_zero_cross'}
        records.append(record)
        stream.append((ms,1,f'D {ident} {int(ms)} {side} {rate} {angle} {stage} {previous} {same} {side*i0} {record["goal_current_ma"]} {cap} {rpm} {int(wheel_valid)}'))
        if event['valid']=='True':
            stream.append((ms,2,f'C {side} {rpm} {int(wheel_valid)} {int(ms)} {int(event["pulse_width_ms"])}'))
    return records,skipped,[s[2] for s in sorted(stream,key=lambda x:(x[0],x[1]))]


def parse_output(output,records):
    for line in output.splitlines():
        cols=line.split();ident=int(cols[0]);assert len(cols)==len(FIELDS)+1,(len(cols),len(FIELDS))
        r=records[ident]
        for key,value in zip(FIELDS,cols[1:]):
            n=float(value);r[key]=(bool(int(n)) if key in BOOL else int(n) if key in INTEGER else n if math.isfinite(n) else None)
    return records


def sensitivity(executable,cfg):
    """Matched inputs isolate energy-domain peak/rate effects, not real gait."""
    specifications=[('baseline',55,6,8),('weak_same_peak',55,2,8),
                    ('large_same_peak',55,11,8),('slow_rate',25,6,8),
                    ('fast_rate',80,6,8),('changed_incoming_peak',55,6,11)]
    records=[];lines=[f'T {rise_tau(cfg)}']
    for i,(name,rate,same,previous) in enumerate(specifications):
        records.append({'name':name,'rate_dps':rate,'same_side_peak_deg':same,
                        'previous_peak_deg':previous,'target_deg':8})
        lines.append(f'D {i} 1000 1 {rate} 0 8 {previous} {same} 0 {goal_current(cfg,7.5)} 80 0 0')
    result=subprocess.run([str(executable)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    parse_output(result.stdout,records);r={x['name']:x for x in records}
    assert all(x['valid'] and x['guarded_valid'] for x in records)
    assert r['weak_same_peak']['peak_correction_j']>r['large_same_peak']['peak_correction_j']
    assert r['weak_same_peak']['width_ms']>r['large_same_peak']['width_ms']
    assert r['slow_rate']['available_j']<r['fast_rate']['available_j']
    assert r['slow_rate']['width_ms']>r['fast_rate']['width_ms'] and r['fast_rate']['coast']
    # BOTH completed peaks enter the common energy and side redistribution.
    # The incoming-to-arrival discrepancy remains a separate audit only.
    assert r['changed_incoming_peak']['width_ms']!=r['baseline']['width_ms']
    assert r['changed_incoming_peak']['common_correction_j']!=r['baseline']['common_correction_j']
    assert r['changed_incoming_peak']['redistribution_j']!=r['baseline']['redistribution_j']
    assert r['changed_incoming_peak']['inbound_loss_proxy_j']>r['baseline']['inbound_loss_proxy_j']
    assert all(abs(x['peak_correction_j']-x['common_correction_j']-x['redistribution_j'])<1e-9 for x in records)
    return records


def summarize(records):
    return {'snapshots':len(records),'valid':sum(r['valid'] for r in records),
            'invalid':sum(not r['valid'] for r in records),
            'drive':sum(r['valid'] and not r['coast'] for r in records),
            'coast':sum(r['valid'] and r['coast'] for r in records),
            'cap_limited':sum(r['saturated'] for r in records),
            'rate_unsupported':sum(r['valid'] and not r['rate_supported'] for r in records),
            'passive_rate_unsupported':sum(r['valid'] and not r['passive_rate_supported'] for r in records),
            'work_rate_unsupported':sum(r['valid'] and not r['work_rate_supported'] for r in records),
            'passive_intercept_rolloff':sum(r['valid'] and r['passive_intercept_rolloff'] for r in records),
            'charge_unsupported':sum(r['valid'] and not r['charge_supported'] for r in records),
            'any_extrapolation':sum(r['extrapolated'] for r in records),
            'early_state_transfer':sum(r['valid'] and r['early_state_transfer'] for r in records),
            'old_decision_valid':sum(r['old_decision_valid'] for r in records),
            'wheel_guard_changed':sum(r['width_ms']!=r['guarded_width_ms'] for r in records),
            'post_guard_invalid':sum(r['valid'] and not r['guarded_valid'] for r in records),
            'post_guard_negative_q':sum(r['guarded_q_mas'] is not None and r['guarded_q_mas']<0 for r in records),
            'state_not_fresh':sum(not r['state_fresh'] for r in records)}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cases',nargs='+',default=CASES)
    p.add_argument('--out',type=Path,default=ROOT/'analysis/2026-11-13-walking/v14_energy_snapshot_replay.json')
    a=p.parse_args();cfg=current_config();results={};headers=['walking_energy_policy.h','lateral_energy_control.h','realtime_code.h','wheel_cycle_guard.h']
    sources={name:(ROOT/'src'/name).read_bytes() for name in headers}
    hashes={name:hashlib.sha256(source).hexdigest() for name,source in sources.items()}
    with tempfile.TemporaryDirectory(prefix='walking_energy_replay_') as tmp:
        tmp=Path(tmp);(tmp/'replay.cpp').write_text(CPP,encoding='utf8')
        for name,source in sources.items():(tmp/name).write_bytes(source)
        subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-I'+str(tmp),str(tmp/'replay.cpp'),'-o',str(tmp/'replay.exe')],check=True)
        model_output=subprocess.run([str(tmp/'replay.exe')],input='M\n',text=True,capture_output=True,check=True)
        model_fields=['passive_offset_j','passive_omega2_j','passive_rate_min_dps','passive_rate_max_dps',
                      'work_rate_min_dps','work_rate_max_dps','work_charge_max_mas']
        models={cols[0]:dict(zip(model_fields,map(float,cols[1:]))) for cols in
                (line.split() for line in model_output.stdout.splitlines())}
        checks=sensitivity(tmp/'replay.exe',cfg)
        for case in a.cases:
            records,skipped,lines=snapshot_data(find_case(case),cfg)
            data=f'T {rise_tau(cfg)}\n'+'\n'.join(lines)+'\n'
            result=subprocess.run([str(tmp/'replay.exe')],input=data,text=True,capture_output=True,check=True)
            records=parse_output(result.stdout,records)
            assert all('valid' in r for r in records)
            summary=summarize(records);summary['skipped']=dict(skipped)
            summary['by_cap_ms']={str(cap):summarize([r for r in records if r['cap_ms']==cap]) for cap in [80,15]}
            assert summary['post_guard_invalid']==0 and summary['post_guard_negative_q']==0
            assert all(not r['valid'] or
                       abs(r['guarded_arrival_j']-r['available_j']-r['guarded_work_j'])<1e-9 for r in records)
            case_path=find_case(case)
            provenance={name:hashlib.sha256((case_path/name).read_bytes()).hexdigest() for name in
                        ['timeseries.csv','energy_control_autonomous_zero_cross_events.csv',
                         'energy_control_autonomous_peak_events.csv','metadata.json']}
            results[case]={'summary':summary,'input_sha256':provenance,'decisions':records}
            print(case,{k:v for k,v in summary.items() if k!='by_cap_ms'})
    out={'scope':__doc__,'source_sha256':hashes,'current_model_config':cfg,'tau_1200_s':rise_tau(cfg),
         'model_coefficients_and_support':models,'counterfactual_checks':checks,'cases':results}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(out,indent=2,allow_nan=False),encoding='utf8')
    md=['# v14 energy snapshot audit','',__doc__,'',
        'The nominal rocker potential, conditional no-new-pulse arrival fit and inherited pulse-Q response are model estimates. Effective kinetic energy is not a measured body-inertia estimate; pulse-Q work is not measured shaft work. Potential still uses the fixed nominal rocker and mechanical angle0 reference; a moving-foot-pose-dependent potential minimum or energy-neutral angle is not implemented here. Old target stages are snapshots, not a replay of the new staged closed loop. See [passive-arrival audit](v14_passive_energy_audit.md) for the first-four-log training / latest-two-log held-out split.','',
        '| Log | Snapshots | Invalid | Drive | Coast | Cap-limited | Rate outside fit | Q outside fit | Any extrapolation | Wheel changes |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for case,result in results.items():
        s=result['summary'];md.append('| '+case+' | '+' | '.join(str(s[k]) for k in ['snapshots','invalid','drive','coast','cap_limited','rate_unsupported','charge_unsupported','any_extrapolation','wheel_guard_changed'])+' |')
    md+=['','## Normal / foot cap80 snapshots, separated from speculative cap15','',
         '| Log | Cap80 opportunities | Cap-limited | Coast | Q outside fit | Cap15 opportunities | Cap15 cap-limited |',
         '|---|---:|---:|---:|---:|---:|---:|']
    for case,result in results.items():
        normal=result['summary']['by_cap_ms']['80'];early=result['summary']['by_cap_ms']['15']
        md.append(f"| {case} | {normal['snapshots']} | {normal['cap_limited']} | {normal['coast']} | {normal['charge_unsupported']} | {early['snapshots']} | {early['cap_limited']} |")
    md+=['','The cap80 and cap15 limits are eligibility classes, not predictions of executed commands. Cap saturation is a residual energy deficit under the selected model. Q/rate extrapolation, transfer of a zero-cross fit to a finite body angle, and signed residual-current model gaps remain explicit.','',
         '## Model support','',
         '| Side | Passive intercept | Passive omega² coefficient | Passive rate band | Pulse-work rate band | Pulse-Q maximum |',
         '|---|---:|---:|---:|---:|---:|']
    for side,m in models.items():
        md.append(f"| {side} | {m['passive_offset_j']*1000:.6f} mJ | {m['passive_omega2_j']*1000:.6f} mJ/(rad/s)² | {m['passive_rate_min_dps']:.4f}..{m['passive_rate_max_dps']:.4f} deg/s | {m['work_rate_min_dps']:.4f}..{m['work_rate_max_dps']:.4f} deg/s | {m['work_charge_max_mas']:.6f} mAs |")
    md+=['','The positive passive intercept represents conditional stored-energy/coupling effects in past coast observations. Below20 deg/s it is scaled by (rate/20)² as an unvalidated extrapolation. The core accepts finite positive rates so low-speed rescue remains possible; the separate foot timing selector retains its own minimum rate. Above the observed passive band the same monotone quadratic is extrapolated, with flags. For the minus side the passive and pulse-work rate bands do not overlap, so joint rate support is absent.','',
         '## Matched-input sensitivity checks','',
         'Current angle0 deg, target8 deg, side+, battery7.5 V and I0=0 are held fixed. Both completed peak energies and the current rate actively alter the energy deficit and selected width. Common energy correction uses the mean of the two peaks; redistribution uses their signed imbalance. The incoming-to-arrival discrepancy remains a separate diagnostic and is not subtracted a second time.','',
         '| Input change | Rate | Same-side peak | Available | Peak correction | Request | Width | Coast |',
         '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in checks:
        md.append(f"| {r['name']} | {r['rate_dps']} deg/s | {r['same_side_peak_deg']} deg | {r['available_j']*1000:.4f} mJ | {r['peak_correction_j']*1000:.4f} mJ | {r['request_j']*1000:.4f} mJ | {r['width_ms']} ms | {r['coast']} |")
    md+=['','## Foot-support snapshots from recent experiments','',
         '| Log / time | Old command | New energy width | Guarded | Available | Target | Peak correction | Request | Predicted work | Unmet after guard |','|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for case,result in results.items():
        if case not in ['97855607','241508524','611622951','141282685']:continue
        for r in result['decisions']:
            if r['body_source']!='saved_support_audit':continue
            fmt=lambda key:f"{r[key]*1000:.4f} mJ" if r[key] is not None else '-'
            md.append(f"| {case} / {r['time_ms']/1000:.3f}s | {r['old_width_ms']}ms ({r['old_output_executed']}) | {r['width_ms']}ms | {r['guarded_width_ms']}ms | "+' | '.join(fmt(k) for k in ['available_j','target_j','peak_correction_j','request_j','predicted_work_j','guarded_unmet_j'])+' |')
    md+=['','This table compares eligibility/energy sizing at recorded decision opportunities. It does not establish how many new commands will execute, whether the gait improves, or physical energy conservation. The backend may reject stale/invalid telemetry, timing, direction or source changes. Signed residual current is audited but its coast-tail arrival contribution is unmodelled.','']
    md+=['## Source snapshot','']+[f'- `{name}` SHA256 `{digest}`' for name,digest in hashes.items()]+['']
    a.out.with_suffix('.md').write_text('\n'.join(md),encoding='utf8')


if __name__=='__main__':main()
