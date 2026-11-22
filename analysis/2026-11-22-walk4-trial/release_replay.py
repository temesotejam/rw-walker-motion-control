"""Check the carry-release detector on original driven intervals, using their measured input and motor edges."""
import argparse
from bisect import bisect_right
from collections import Counter
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-11-22-walking-simulator'))
import aid_current_delivery
from receiving_carry import CarryReplay


def receiving_pair(motion, peak, capture, packets, by_sequence, exe):
    """Require two preceding adjacent retained CAD slots with real, advancing exposures and positive losses."""
    recent = sorted((s for s in motion.by_peak.get(peak,()) if 0<=capture-s[0]*1000<=75000), key=lambda s:s[0])[-2:]
    if len(recent)!=2:
        return None
    history = []
    for at, _, row in recent:
        if not row['accel_dps2'] or float(row['accel_dps2'])<=0 or not row['rate_dps'] or float(row['rate_dps'])>=0:
            return None
        p=packets[int(row['packet'])-1]
        seq=int(p['left_sequence'])
        f=by_sequence.get(seq)
        if not (f and seq==int(p['right_sequence']) and f['timestamp_valid'] and f['frame_valid'] and f['zero_ready']
                and f['left_valid'] and f['right_valid'] and f['left_control_range_ok'] and f['right_control_range_ok']
                and f['control_observed_us']<=motion.clock+int(p['publish_us'])
                and 0<=int(p['publish_us'])<=at*1000+1 and 0<=motion.clock+at*1000-f['frame_us']<=75000
                and p['left_observed_deg'] and p['right_observed_deg'] and row['left_deg'] and row['right_deg']):
            return None
        command='G {} {} {} {} {} {}'.format(p['left_observed_deg'],p['right_observed_deg'],row['left_deg'],
                                             row['right_deg'],row['fore_aft_deg'],row['lateral_deg'])
        history.append((int(p['slot_us']),f['frame_us'],float(row['accel_dps2']),command))
    if history[1][0]-history[0][0]!=25000 or history[1][1]<=history[0][1]:
        return None
    observed=subprocess.check_output([exe],input='\n'.join(h[3] for h in history)+'\n',text=True).splitlines()
    return [h[2] for h in history] if observed==['1','1'] else None


def audit(folder, exe):
    meta = json.loads((folder/'metadata.json').read_bytes())
    events = meta['walking_energy']['records']
    aids = [(i,r) for i,r in enumerate(events) if r['return_event'] and r['side']==-1 and r['executed']
            and r.get('return_evidence') not in (7,8)]
    if not aids:
        return []
    motion = CarryReplay(folder)
    driver = {d['start_ms']: d for d in aid_current_delivery.evaluate_trial(folder, motion)}
    raw = sorted((f for f in meta['foot_frames'] if f['state_id']==3 and f['control_observed_us']),
                 key=lambda f:f['control_observed_us'])
    ready = [f['control_observed_us'] for f in raw]
    by_sequence = {f['sequence']:f for f in raw}
    with (folder/'foot_periodic.csv').open(newline='') as source:
        packets = list(csv.DictReader(source))
    edges = meta['energy_control_autonomous_zero_cross_events'][1:]
    if len(edges)!=len(events):
        raise ValueError(f'{folder}: decisions do not align')
    with (folder/'timeseries.csv').open(newline='') as source:
        imu = [s for s in csv.DictReader(source) if s['state_id']=='3' and s['body_rate_sample_time_us']
               and s['imu_sample_age_us'] and s['pitch_mekf_detector_relative_deg']]
    cases = []
    for i,row in aids:
        d = driver.get(row['output_start_ms'], {})
        output_start = row['output_start_ms']
        edge = edges[i]
        capture = motion.clock + ((edge['pre_input_capture_time_us']-motion.clock) & 0xffffffff)
        index = bisect_right(ready,capture)-1
        f = raw[index] if index>=0 else None
        outcome = motion.follow(dict(peak_ms=row['peak_ms'], decision_ms=output_start, terminal_deg=0))
        record = dict(run=folder.name, decision_ms=row['time_ms'], first_elapsed_ms=row['actual_elapsed_ms'],
                      outcome=outcome['status'], release_signal_ms=None, checked=False)
        cases.append(record)
        if not (f and f['timestamp_valid'] and f['frame_valid'] and f['zero_ready'] and f['left_control_range_ok']
                and f['left_valid'] and 0<=capture-f['frame_us']<=75000 and 'driver_off_ms' in d
                and row['fore_aft_angle_deg'] is not None and row['return_deceleration_dps2']>0):
            continue
        # These are first pulses. Apply the later-pulse release criterion only
        # as an analogue, using the last observed foot and the pre-pulse loss.
        # It did not govern the actual duration or the ensuing original path.
        pitch = f['left_deg'] + row['fore_aft_angle_deg']
        commands = []
        times = []
        for s in imu:
            t = int(s['t_test_ms'])
            if not output_start<t<row['output_end_ms']:
                continue
            stamp = int(s['body_rate_sample_time_us'])
            at = ((stamp-motion.clock) & 0xffffffff)*.001
            age = int(s['imu_sample_age_us'])
            if not (d['driver_on_ms']<=at<=at+age*.001<=d['driver_off_ms']):
                continue
            angle=float(s['pitch_mekf_detector_relative_deg'])
            rate=(float(s['gy_dps'])-float(s['mekf_bias_y_dps']))*meta['mekf_gyro_y_scale']
            commands.append(f"I {stamp} {age} {t-output_start} {angle} {rate}")
            times.append(t)
        def first_release(decel):
            answer = subprocess.check_output([exe],input='\n'.join([f'A {pitch} {decel}',*commands])+'\n',text=True).splitlines()
            if len(answer)!=len(times)+1:
                raise ValueError('incomplete detector output')
            record['terminal_deg']=float(answer[0])
            return next((t for t,v in zip(times,answer[1:]) if v=='1'),None)
        record['checked'] = True
        record['samples'] = len(times)
        record['release_signal_ms'] = first_release(row['return_deceleration_dps2'])
        record['actual_zero_speed_dps']=outcome.get('inward_speed_dps')
        pair=receiving_pair(motion,row['peak_ms'],capture-motion.clock,packets,by_sequence,exe)
        if pair:
            record['two_receiving_slots_dps2']=pair
            record['release_lower_ms']=first_release(min(pair))
            record['release_upper_ms']=first_release(max(pair))
            if record['release_upper_ms']:
                arc=motion.follow(dict(peak_ms=row['peak_ms'],decision_ms=record['release_upper_ms'],
                                       terminal_deg=record['terminal_deg']))
                record['arc_outcome']=arc['status']
                record['arc_speed_dps']=arc.get('inward_speed_dps')
                record['arc_recorded_command']=arc['recorded_command']
    return cases


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots',nargs='+',type=Path)
    args=parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/'release.exe'
        subprocess.run(['g++','-std=c++17','-O2','-I',HERE.parents[1]/'src',HERE/'release_probe.cpp','-o',exe], check=True)
        cases=[c for folder in sorted({p.parent for root in args.roots for p in root.glob('*/header.json')})
               for c in audit(folder,exe)]
    print('First aid intervals:',len(cases),'with source/driver timing:',sum(c['checked'] for c in cases))
    print('Intervals with at least two logged ticks:', sum(c.get('samples',0)>=2 for c in cases))
    print('Original outcomes when release signalled:',dict(Counter(c['outcome'] for c in cases if c['release_signal_ms'])))
    print('Intervals with two recent receiving slots and positive losses:',sum('two_receiving_slots_dps2' in c for c in cases))
    for field in ('release_lower_ms','release_upper_ms'):
        print(field,dict(Counter(c['outcome'] for c in cases if c.get(field))))
    for case in cases:
        if case.get('release_upper_ms') or case['run']=='33406765':
            print(json.dumps(case,allow_nan=False))
