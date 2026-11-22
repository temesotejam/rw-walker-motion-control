"""Compare left-transfer carry with prediction1 at the same recorded decisions.

The new work calculation runs only inside an accepted left aid. Replay the
production camera track, reserve, reference, wheel guard and work selector.
Historical half-cycles are independent conditional comparisons; later motions
are the original measurements even when an earlier decision would be different.
"""
import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
SIM = HERE.parent / '2026-11-22-walking-simulator'
sys.path.insert(0, str(SIM))
import return_history_replay
import return_model


def read_rows(folder, name):
    with (folder / (name + '.csv')).open(encoding='utf8', newline='') as f:
        return list(csv.DictReader(f))


def approach_sensitivity(fields):
    if len(fields)!=12:
        raise ValueError('Expected three crossing forecasts at each of three horizons')
    width,horizon,gap,*times=map(float,fields)
    forecasts=[]
    for offset,duration in zip((0,3,6),(horizon,60.,150.)):
        observed,estimated,tilted=(value if math.isfinite(value) else None for value in times[offset:offset+3])
        forecasts.append(dict(horizon_ms=duration, observed_fixed_pitch=observed,
                              estimated_fixed_pitch=estimated, estimated_changing_pitch=tilted))
    return dict(width_without_main_reserve_ms=int(width), nominal_horizon_ms=horizon,
                measured_left_minus_right_height_mm=gap, predicted_height_crossings_ms=forecasts)


def replay(folder, executable, legacy, audit_approach=False):
    meta = json.loads((folder/'metadata.json').read_bytes())
    energy = meta['walking_energy']
    records = energy['records']
    events = meta['energy_control_autonomous_zero_cross_events']
    peaks = meta['energy_control_autonomous_peak_events']
    if (energy['truncated'] or len(records) != energy['total'] or len(events) != len(records)+1
            or meta.get('metadata_event_detail_truncated') or meta.get('energy_control_autonomous_event_overflow')):
        raise ValueError(f'{folder}: incomplete commands')
    arcs = {a['row_index']: a for a in energy.get('arc_aids', [])}
    raw = [f for f in meta['foot_frames'] if f['state_id']==3 and f['control_observed_us']]
    epoch, = {f['frame_us']-f['measurement_time_us'] for f in raw if f['timestamp_valid']}
    imu = [(int(r['t_test_ms']), int(r['imu_sample_age_us'])) for r in read_rows(folder, 'timeseries')
           if r['state_id']=='3' and r['imu_sample_age_us']]
    imu_ms = [t for t,_ in imu]
    episodes = []
    return_model.load_sections(folder, sections=(), episodes=episodes, foot_packets=[])
    for ep in episodes:
        ep['pulse_before_end'] = any(e['output_executed'] and e['pulse_start_ms'] < ep['end_ms']-1
                                     and e['pulse_end_ms'] > ep['peak_ms'] for e in events)
    by_peak = {e['peak_ms']:e for e in episodes}
    # Reuse the current production selector for the already-certified arc path,
    # which runs unchanged in both conditions (e.g. the 14:40 trial).
    previous = return_history_replay.replay(folder,legacy)
    previous_ms = {d['decision_ms']: d['crossing_selected_ms'] for d in previous['decisions']}
    stream, decisions = [], {}

    def append(stamp, order, kind, *args):
        stream.append((stamp,order,kind+' '+' '.join('nan' if v is None else str(v) for v in args)))

    for f in raw:
        append(f['control_observed_us'],0,'F',f['sequence'],f['frame_us'],f['control_observed_us'],
               int(f['timestamp_valid'] and f['zero_ready']), int(f['left_valid']), int(f['left_control_range_ok']),
               int(f['right_valid']), int(f['right_control_range_ok']),f['left_deg'],f['right_deg'])
    for p in read_rows(folder,'foot_periodic'):
        append(epoch+int(p['publish_us']),1,'S',epoch+int(p['publish_us']),p['left_sequence'],p['right_sequence'])
    completed = [(w,e) for w,e in zip(records,events[1:]) if w['executed'] and w['output_end_ms']]
    for p in peaks:
        t, seen = p['peak_time_ms'], p['peak_telemetry_capture_time_ms']
        recent = [(w,e) for w,e in completed if w['output_end_ms'] < seen and w['output_start_ms'] < t]
        w,e = recent[-1] if recent else ({},{})
        j = bisect_right(imu_ms, seen)-1
        fresh = j>=0 and seen-imu_ms[j]<=10 and imu[j][1]<=10000
        powered = p['pending_command_matched'] and (p['pending_q_command_mA_s'] or 0)>0 and fresh
        match = (fresh and p['pending_command_matched'] and w.get('assisted_cycle') and not w['optional']
                 and not w['rescue'] and not w['return_event'] and w['completion_reason']==1
                 and w['completion_valid'] and w['completion_work_valid'] and w['side']==p['physical_peak_side']
                 and e['event_kind']=='energy_control_zero_cross')
        full = all(w.get(k) for k in ('planned_duration_completed','completion_width_supported','charge_supported'))
        append(epoch+1000*seen,2,'P',t,p['physical_peak_side'],int(bool(powered)),int(bool(match)),
               e.get('zero_cross_time_ms',0),int(full),p['peak_amplitude_deg'],w.get('start_rate_dps'),w.get('target_j'))
    for i,(w,e) in enumerate(zip(records,events[1:])):
        t, peak, side = w['time_ms'], w['peak_ms'], w['side']
        if peak!=e['previous_peak_time_ms'] or w['executed']!=e['output_executed']:
            raise ValueError(f'{folder}: misaligned commands at {t}')
        stamp = e['pre_input_capture_time_us'] or epoch+t*1000
        if not w['return_event']:
            # An accepted coast copies the untouched guard. Only actual main
            # pulses update its last side and speed.
            if w['executed']:
                append(stamp,3,'M',t,side,w['requested_ms'],e['pre_input_wheel_speed_rpm'])
                if w['completion_reason']==1 and w['completion_valid']:
                    append(epoch+w['output_end_ms']*1000,0,'R',side,int(w['assisted_cycle']),w['actual_elapsed_ms'])
            continue
        if side!=-1 or not w['executed']:
            continue
        ep=by_peak.get(peak,{})
        arc=arcs.get(i,{})
        d=dict(ms=t,peak_ms=peak,evidence=w['return_evidence'],observed_ms=w['guarded_ms'],
               observed_work_uJ=w['predicted_work_j']*1e6,outcome=ep.get('outcome'),
               prediction1_ms=previous_ms.get(t,w['guarded_ms']),archive_source=arc.get('arc_foot_source'),
               archive_reason=arc.get('arc_aid_reason'),archive_reserve=arc.get('arc_main_reserve_ms'),
               archive_base=arc.get('arc_base_guarded_ms'),envelope_ms=w['envelope_before_ms'])
        decisions[t]=d
        append(stamp,5,'C')
        if w['return_evidence']!=1 or not w['fore_aft_allowed'] or w['return_foot_age_ms']>75:
            d['new_ms']=d['prediction1_ms']
            continue
        matches=[f for f in raw if f['measurement_time_us']//1000==t-w['return_foot_age_ms']
                 and f['control_observed_us']<=stamp and all(f[k] for k in (
                     'timestamp_valid','frame_valid','zero_ready','left_valid','right_valid',
                     'left_control_range_ok','right_control_range_ok'))]
        if len(matches)!=1:
            d['unreconstructed']='unmatched observed frame'
            continue
        f=matches[0]
        recent=[p for p in peaks if p['physical_peak_side']==-1 and p['peak_telemetry_capture_time_ms']<t
                and t-p['peak_time_ms']<=3000]
        append(stamp,4,'A',t,peak,stamp,f['sequence'],f['frame_us'],f['control_observed_us'],
               w['periodic_foot_packet'],e['work_brake_cap_ms'],w['envelope_before_ms'],
               f['left_deg'],f['right_deg'],w['fore_aft_angle_deg'],w['fore_aft_rate_dps'],e['detector_angle_after_deg'],
               e['zero_cross_abs_rate_dps'],w['return_deceleration_dps2'],e['startup_target_deg'],
               e['previous_peak_amplitude_deg'],recent[-1]['peak_amplitude_deg'] if recent else None,
               e['aligned_initial_current_mA'],e['predicted_current_goal_mA'],e['predicted_current_rise_tau_s'],
               e['pre_input_wheel_speed_rpm'],w['return_work_j'])
    payload=str(epoch)+'\n'+'\n'.join(s for _,_,s in sorted(stream))+'\n'
    command=[executable,'--audit-approach'] if audit_approach else [executable]
    result=subprocess.run(command,input=payload,text=True,capture_output=True)
    if result.returncode:
        raise ValueError(f'{folder}: adapter {result.returncode}: {result.stderr}')
    for line in result.stdout.splitlines():
        fields=line.split()
        (t,src,reason,res,inv,base,zero,speed,transfer,terminal,arc_j,ref_j,new_inv,width,work)=fields[:15]
        d=decisions[int(t)]
        if fields[15:]:
            d['approach_sensitivity']=approach_sensitivity(fields[15:])
        if (d['archive_source'] is not None and int(src)!=d['archive_source']
                or d['archive_reserve'] is not None and int(res)!=d['archive_reserve']
                or d['archive_base'] is not None and int(base)!=d['archive_base']):
            raise ValueError(f'{folder.name}/{t}: mismatch in foot, reserve or pre-arc wheel guard: {line}; {d}')
        # Older firmware used a stricter arc certificate after the receiving
        # pivot. It can report UncertainPivot where prediction1 says Eligible.
        if d['archive_reason'] is not None and (d['archive_reason']==3)!=(int(reason)==3):
            raise ValueError(f'{folder.name}/{t}: original geometry did not reproduce')
        if int(reason)==3:
            # This branch first became active in left_transfer_carry_v1.
            d['prediction1_ms']=int(base)
            expected=width if meta.get('walking_profile')=='left_transfer_carry_v1' else base
            if d['observed_ms']!=int(expected):
                raise ValueError(f'{folder.name}/{t}: recorded profile width mismatch')
        # For old versions without recorded arc decisions, the retained v1 arc
        # extension may be unknown. A different geometry path cannot trigger the
        # tested extension; a same-width result there does not reconstruct v1.
        known_v1=(int(reason)==3 or int(reason)==5 and d['ms'] in previous_ms)
        if not known_v1:
            d['prediction1_ms']=None
        d.update(source=int(src),initial_arc_reason=int(reason),baseline_inverse_ms=int(inv),
                 baseline_pre_arc_ms=int(base),reserve_ms=int(res),ref_zero_ms=int(zero),ref_dps=float(speed),
                 transfer_ms=None if not math.isfinite(float(transfer)) else float(transfer),
                 terminal_deg=None if not math.isfinite(float(terminal)) else float(terminal),
                 arc_request_uJ=float(arc_j)*1e6 if math.isfinite(float(arc_j)) else None,
                 ref_request_uJ=float(ref_j)*1e6,
                 candidate_inverse_ms=int(new_inv),new_work_uJ=float(work)*1e6 if int(reason)==3 else None,
                 new_ms=int(width) if int(reason)==3 else d['prediction1_ms'] if known_v1 else None,
                 other_path_unchanged=int(reason)!=3)
    return dict(run=folder.name, episodes=episodes, aids=list(decisions.values()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots',type=Path,nargs='+',help='Roots containing converted per-trial folders')
    parser.add_argument('--json-out',type=Path)
    parser.add_argument('--audit-approach',action='store_true',
                        help='Evaluate removing only the main reservation and list CAD crossing forecasts')
    args=parser.parse_args()
    folders=sorted({f.parent for root in args.roots for f in root.glob('*/header.json')})
    if not folders:
        parser.error('No converted run folders')
    with tempfile.TemporaryDirectory() as tmp:
        flags=['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-DWALKING_RETURN_HISTORY=1',
               '-I',HERE.parents[1]/'src']
        adapter,prior=Path(tmp)/'transfer_carry.exe',Path(tmp)/'history.exe'
        subprocess.run([*flags,HERE/'transfer_carry_selector.cpp','-o',adapter],check=True)
        subprocess.run([*flags,SIM/'return_history_selector.cpp','-o',prior],check=True)
        runs=[replay(folder,adapter,prior,args.audit_approach) for folder in folders]
    episodes=[e for r in runs for e in r['episodes']]
    aids=[dict(run=r['run'],**a) for r in runs for a in r['aids']]
    print('Runs:',len(runs),'right peaks:',len(episodes),'outcomes:',
          Counter((e['outcome'],e['pulse_before_end']) for e in episodes))
    print('Left aids:',len(aids),'unmatched:',sum('unreconstructed' in a for a in aids),
          'new-path candidates:',sum(a.get('initial_arc_reason')==3 for a in aids))
    for a in aids:
        if a['run'] in ('135284989','70067320','195727101') or (
                a.get('new_ms') is not None and a['new_ms']!=a['prediction1_ms']):
            print(a)
    if args.json_out:
        args.json_out.write_text(json.dumps(runs,indent=2,allow_nan=False),encoding='utf8')


if __name__=='__main__':
    main()
