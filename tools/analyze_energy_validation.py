#!/usr/bin/env python3
"""Audit requested/limited work and matched next-peak predictions in RWLOGs.

Accepts RWLOGs or folders produced by convert_rwlog_to_csv.py. Historical
q_unclamped fields are deliberately ignored: prior firmware logged clipped Q.
This audits the combined model; residuals alone do not identify brake work.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from convert_rwlog_to_csv import convert


def finite(x):
    try:
        return math.isfinite(float(x))
    except (ValueError, TypeError):
        return False


def inverse(omega, a, b, work):
    disc=(a*omega)**2+4*b*work
    if disc < 0:
        return None
    denominator=a*omega+math.sqrt(disc)
    return 2000*work/denominator if denominator>0 else (0 if work==0 else None)


def required_width(q_mAs, initial, goal, tau):
    """Diagnostic only: no extension of legal controller pulse limits."""
    if not all(finite(x) for x in (q_mAs, initial, goal, tau)) or tau<=0 or goal<=0:
        return None
    if q_mAs==0:
        return 0
    sign=1 if q_mAs>0 else -1
    def charge(ms):
        t=ms*.001
        return sign*goal*t+(initial-sign*goal)*tau*(-math.expm1(-t/tau))
    if sign*charge(100)<abs(q_mAs):
        return None
    lo, hi=0,100
    while hi-lo>1:
        mid=(hi+lo)//2
        if sign*charge(mid)<abs(q_mAs):
            lo=mid
        else:
            hi=mid
    return min((lo,hi),key=lambda ms: abs(charge(ms)-q_mAs))


def average(values):
    values=[x for x in values if finite(x)]
    return statistics.mean(values) if values else None


def analyze(folder, start_ms=5000):
    meta=json.loads((folder/'metadata.json').read_text(encoding='utf-8'))
    with (folder/'timeseries.csv').open(encoding='utf-8',newline='') as f:
        samples=list(csv.DictReader(f))
    zero=sorted([z for z in meta['energy_control_autonomous_zero_cross_events']
                 if z['event_kind']=='energy_control_zero_cross'],key=lambda z:z['zero_cross_time_ms'])
    peaks=meta['energy_control_autonomous_peak_events']
    rows=[]
    unmatched=0
    for index,z in enumerate(zero):
        t=z['zero_cross_time_ms']
        if t<start_ms:
            continue
        next_t=zero[index+1]['zero_cross_time_ms'] if index+1<len(zero) else math.inf
        matches=[p for p in peaks if t<p['peak_time_ms']<next_t
                 and p['physical_peak_side']==z['physical_next_peak_side']
                 and p['pending_command_matched']]
        if len(matches)!=1 or not z['valid']:
            unmatched+=1
            continue
        p=matches[0]
        target=z['target_energy_j']
        bounded_i=max(-.5*target,min(.5*target,z['integral_energy_j']))
        raw=target-z['passive_energy_j']+bounded_i-z.get('prediction_bias_j',0)
        q=inverse(math.radians(z['zero_cross_abs_rate_dps']),z['work_omega_q_coefficient'],
                  z['work_q2_coefficient'],raw)
        if meta.get('energy_request_audit_revision'):
            raw=z['unclamped_energy_request_j']
            q=z['q_unclamped_mA_s'] if z['unclamped_inverse_valid'] else None
        work=z['predicted_energy_j']-z['passive_energy_j']
        observed_prediction=z.get('predicted_observed_energy_j',z['predicted_energy_j'])
        width=required_width(q,z.get('aligned_initial_current_mA'),
                             z.get('predicted_current_goal_mA'),z.get('predicted_current_rise_tau_s'))
        currents=[]
        observed_q=[]
        seq=set()
        for s in samples:
            if z['pulse_width_ms']==0:
                break
            if not z['pulse_start_ms']<=float(s['t_test_ms'])<=z['pulse_end_ms']+50:
                continue
            age=(int(s['roller_current_sample_time_us'])-z['pre_input_capture_time_us']) & 0xffffffff
            if age>(z['pulse_width_ms']+100)*1000:
                continue
            if s['roller_q_meas_observed_valid']=='1' and finite(s['roller_q_meas_observed_mA_s']):
                observed_q.append(float(s['roller_q_meas_observed_mA_s']))
            if s['pulse_active']!='1' or s['roller_current_valid']!='1':
                continue
            sequence=s['roller_current_sequence']
            if sequence in seq or not finite(s['roller_actual_current_mA']):
                continue
            # Reject samples taken before the event, including previous-pulse Q.
            seq.add(sequence)
            currents.append(abs(float(s['roller_actual_current_mA'])))
        row=dict(source=folder.name,firmware=meta['firmware_revision'],
                 adaptation_enabled=meta.get('peak_error_adaptation_enabled',False),
                 brake_cap_ms=meta['energy_control_autonomous_negative_side_brake_width_max_ms'],
                 t_ms=t,side=z['physical_next_peak_side'],mode=z.get('adaptation_mode',
                 1 if z['q_command_mA_s']>0 else -1 if z['q_command_mA_s']<0 else 0),
                 rate_dps=z['zero_cross_abs_rate_dps'],wheel_rpm=z.get('pre_input_wheel_speed_rpm'),
                 battery_mV=z['vbat_mV'],target_deg=z['target_peak_deg'],peak_deg=p['peak_amplitude_deg'],
                 required_work_j=raw,required_q_mAs=q,required_width_within_100ms_ms=width,
                 required_width_not_available=width is None,
                 selected_width_ms=z['pulse_width_ms'],selected_q_mAs=z['q_command_mA_s'],
                 selected_model_work_j=work,unapplied_request_j=raw-work,
                 saturation=bool(z['q_saturated_upper'] or z['q_saturated_lower']),
                 bias_j=z.get('prediction_bias_j',0),model_peak_energy_j=z['predicted_energy_j'],
                 actual_peak_energy_j=p['measured_energy_j'],
                 model_residual_j=p['measured_energy_j']-z['predicted_energy_j'],
                 corrected_residual_j=p['measured_energy_j']-observed_prediction,
                 model_angle_error_deg=p['peak_amplitude_deg']-z['predicted_next_peak_amplitude_deg'],
                 current_max_sample_mA=max(currents) if currents else None,
                 current_unique_active_samples=len(currents),
                 current_observed_partial_q_mAs=max(observed_q) if observed_q else None,
                 peak_wheel_rpm=p.get('peak_wheel_speed_rpm'),peak_speed_age_us=p.get('peak_speed_age_us'))
        # Counterfactual passive energy is predicted, never a measured work.
        rows.append(row)
    summaries=[]
    for side in (-1,1):
        rr=[r for r in rows if r['side']==side]
        summaries.append(dict(source=folder.name,side=side,n=len(rr),unmatched=unmatched,
            brake_cap_ms=meta['energy_control_autonomous_negative_side_brake_width_max_ms'],
            adaptation_enabled=meta.get('peak_error_adaptation_enabled',False),
            saturation_count=sum(r['saturation'] for r in rr),
            mean_peak_deg=average([r['peak_deg'] for r in rr]),
            mean_required_q_mAs=average([r['required_q_mAs'] for r in rr]),
            mean_selected_q_mAs=average([r['selected_q_mAs'] for r in rr]),
            mean_unapplied_request_j=average([r['unapplied_request_j'] for r in rr]),
            mean_model_residual_j=average([r['model_residual_j'] for r in rr]),
            mean_abs_model_error_deg=average([abs(r['model_angle_error_deg']) for r in rr]),
            mean_current_max_mA=average([r['current_max_sample_mA'] for r in rr]),
            mean_pre_wheel_rpm=average([r['wheel_rpm'] for r in rr]),
            metadata_truncated=meta.get('metadata_event_detail_truncated',False),
            terminal_state=meta.get('terminal_state',{}).get('state')))
    return rows,summaries


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sources',nargs='+',type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--start-ms',type=int,default=5000)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    all_rows,all_summaries=[],[]
    for source in args.sources:
        folder=source
        if source.is_file():
            folder=args.out/source.stem
            # convert() verifies CRC and raises before writing invalid input.
            convert(source,folder)
        rows,summaries=analyze(folder,args.start_ms)
        all_rows+=rows;all_summaries+=summaries
    (args.out/'summary.json').write_text(json.dumps(all_summaries,indent=2,ensure_ascii=False),encoding='utf-8')
    for name,items in [('events.csv',all_rows),('summary.csv',all_summaries)]:
        with (args.out/name).open('w',encoding='utf-8-sig',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(items[0]) if items else [])
            writer.writeheader();writer.writerows(items)
    for r in all_summaries:
        print(f"{r['source']} side={r['side']:+d} n={r['n']} cap={r['brake_cap_ms']} "
              f"sat={r['saturation_count']} peak={r['mean_peak_deg']} "
              f"unapplied_J={r['mean_unapplied_request_j']} residual_J={r['mean_model_residual_j']}")


if __name__=='__main__':
    main()
