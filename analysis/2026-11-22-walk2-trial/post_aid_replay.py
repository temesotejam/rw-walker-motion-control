"""Re-run the firmware post-aid decision on archived observation and command histories."""
import argparse
from bisect import bisect_right
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-11-22-return-trial'))
from compare_timing import FLAGS, rows
import receiving_carry


def compare(folder, exe):
    motion = receiving_carry.CarryReplay(folder)
    meta = json.loads((folder/'metadata.json').read_bytes())
    epoch = motion.clock
    energy = meta['walking_energy']
    events = energy['records']
    edges = meta['energy_control_autonomous_zero_cross_events'][1:]
    if energy['stored'] != energy['total'] or len(events) != len(edges) or not epoch:
        raise ValueError(f'{folder}: cannot align retained events')
    pe = meta['energy_control_autonomous_peak_events']
    packet_rows = rows(folder, 'foot_periodic')
    # count is the ordinal of a publication, independent of skipped nominal slots.
    packets = dict(enumerate(packet_rows, 1))
    raw = sorted((f for f in meta['foot_frames'] if f['state_id'] == 3 and f['control_observed_us']),
                 key=lambda f: f['control_observed_us'])
    ready = [f['control_observed_us'] for f in raw]
    by_seq = {}
    for f in raw:
        key = f['sequence']
        by_seq[key] = f if key not in by_seq else None
    imu, wheels = [], []
    prior_sample = prior_rpm = None
    for r in rows(folder, 'timeseries'):
        if r['state_id'] != '3':
            continue
        if r['roller_speed_sample_time_us'] and r['roller_speed_valid'] == '1':
            at = epoch + ((int(r['roller_speed_sample_time_us']) - epoch) & 0xffffffff)
            completed = epoch + ((int(r['roller_speed_read_end_us']) - epoch) & 0xffffffff)
            if at != prior_rpm and completed >= at:
                wheels.append((completed, at, float(r['roller_speed_rpm'])))
                prior_rpm = at
        if not all(r[k] for k in ('body_rate_sample_time_us','imu_sample_age_us',
                                   'pitch_mekf_detector_relative_deg','gy_dps','mekf_bias_y_dps')):
            continue
        sample = epoch + ((int(r['body_rate_sample_time_us']) - epoch) & 0xffffffff)
        if sample > epoch+60_000_000 or sample == prior_sample:
            continue
        age = int(r['imu_sample_age_us'])
        rate = (float(r['gy_dps'])-float(r['mekf_bias_y_dps']))*meta['mekf_gyro_y_scale']
        imu.append((sample+age,sample,age,int(r['t_test_ms']),float(r['pitch_mekf_detector_relative_deg']),rate))
        prior_sample = sample
    imu.sort(); wheels.sort()
    imu_times = [p[0] for p in imu]
    wheel_times = [p[0] for p in wheels]
    stream, cases, snapshot, relaxed_slots = [], {}, {}, {}

    def put(tag, *values):
        stream.append(tag+' '+' '.join('nan' if v is None or v == '' else str(v) for v in values))

    for i, (w, e) in enumerate(zip(events, edges)):
        if not (w['return_event'] and w['side'] == -1 and w['executed']):
            continue
        peak, end = w['peak_ms'], w['output_end_ms']
        actual = motion.follow(dict(peak_ms=peak,decision_ms=w['output_start_ms'],terminal_deg=0.))
        case = dict(run=folder.name, peak_ms=peak, end_ms=end, first_width_ms=w['guarded_ms'],
                    remaining_ms=w['envelope_after_ms'], outcome=actual['status'], event_ms=actual.get('end_ms'),
                    saved_slots=0, skipped_slots=0, no_current_provenance=0, imu_gaps_over_10ms=0, decision=None)
        if peak in cases:
            raise ValueError(f'{folder}: ambiguous previous aid at {peak}')
        cases[peak] = case
        q = w.get('elapsed_q_pred_mas')
        # Production verifies modeled charge against the left-column fit bound.
        case['armed'] = False
        if not (w['completion_reason'] == 1 and w['planned_duration_completed'] and w['completion_work_valid'] and
                q is not None and w['elapsed_work_pred_j'] > 0 and w['envelope_after_ms'] > 0):
            continue
        prior_peaks = [p for p in pe if p['peak_telemetry_capture_time_ms'] <= end]
        same = [p for p in prior_peaks if p['physical_peak_side'] == -1 and end-p['peak_time_ms']<=3000]
        put('A',peak,end,epoch,w['envelope_after_ms'],w['start_rate_dps'],w['elapsed_work_pred_j'],q,
            w['target_j'],w['start_tau_s'],e['startup_target_deg'],e['previous_peak_amplitude_deg'],
            same[-1]['peak_amplitude_deg'] if same else None)
        for p in prior_peaks:
            cutoff = p['peak_telemetry_capture_time_ms']
            j = bisect_right(imu_times,epoch+cutoff*1000)-1
            fresh = j >= 0 and 0 <= cutoff-imu[j][3] <= 10 and imu[j][2] <= 10000
            paired = fresh and p['pending_command_matched'] and (p['pending_q_command_mA_s'] or 0)>0
            put('H',p['physical_peak_side'],p['peak_time_ms'],int(paired),p['peak_amplitude_deg'])
        mains = [(c,z) for c,z in zip(events[:i],edges[:i]) if c['executed'] and not c['return_event']]
        if mains:
            c,z = mains[-1]
            put('M',c['side'],c['time_ms'],z['pre_input_wheel_speed_rpm'])
        other = [c['time_ms'] for c in events[i+1:] if c['peak_ms']==peak and
                 (c['executed'] or (not c['optional'] and not c['rescue'] and not c['return_event']))]
        stop = min([end+151]+other+[p['peak_telemetry_capture_time_ms'] for p in pe if p['peak_time_ms']>peak])
        lo, hi = epoch+end*1000, epoch+stop*1000
        ticks = [(epoch+round(at*1000), 0, r) for at,_,r in motion.by_peak.get(peak,()) if end<=int(r['time_ms'])<stop]
        ticks.extend((p[0],1,p) for p in imu[bisect_right(imu_times,lo):bisect_right(imu_times,hi)]
                     if end<=p[3]<stop)
        counts = []
        last_imu = None
        for at, kind, data in sorted(ticks,key=lambda k:(k[0],k[1])):
            if kind:
                _,sample,age,now,angle,rate = data
                if last_imu is not None and sample-last_imu > 10000:
                    case['imu_gaps_over_10ms'] += 1
                last_imu = sample
                put('I',now,sample & 0xffffffff,age,at,angle,rate)
                continue
            # CarryReplay stores CAD timestamps in measurement-relative milliseconds.
            r = data
            at = epoch + ((int(r['capture_us32']) - epoch) & 0xffffffff)
            now, count = int(r['time_ms']), int(r['packet'])
            packet = packets.get(count)
            if packet is None:
                continue
            publish = epoch+int(packet['publish_us'])
            if publish > at:
                raise ValueError(f'{folder}/{count}: future packet')
            counts.append(count)
            sides = []
            for side in ('left','right'):
                seq = int(packet[side+'_sequence'])
                f = by_seq.get(seq)
                image = f['frame_us'] if f and f['control_observed_us'] <= publish else 0
                # The CAD snapshot keeps more digits of that packet's estimate
                # than the rounded periodic export.
                estimate=r[side+'_deg']
                exported=packet[side+'_est_deg']
                if exported and estimate and abs(float(estimate)-float(exported))>.001:
                    raise ValueError(f'{folder}/{count}: periodic and CAD estimates disagree')
                sides.extend((seq,image,packet[side+'_observed_deg'],estimate))
            fi=bisect_right(ready,at)-1
            f=raw[fi] if fi>=0 else None
            raw_state = [sum(1<<j for j,k in enumerate(FLAGS) if f[k]),max(0,f['measurement_time_us']//1000),
                         max(0,(f['control_observed_us']-epoch)//1000),f['left_deg'],f['right_deg']] if f else [0,0,0,None,None]
            gates=int(r['gate_mask'])
            fresh_current = r.get('current_after_zero_ms','') not in ('',None) and gates == 0
            if 'current_after_zero_ms' not in r:
                case['no_current_provenance']+=1
            wi=bisect_right(wheel_times,at)-1
            rpm=wheels[wi][2] if wi>=0 and 0<=at-wheels[wi][1]<=50000 else None
            # Actual age is not stored per CAD row. Its stale-IMU flag is the
            # firmware's same <=10 ms comparison; the observer only uses this boolean.
            active=int(not (gates & (16|512)) and not (int(r['baseline_flags']) & (1|4)))
            put('S',at,now,int(r['capture_us32']),10001 if gates&1 else 0,active,count,
                epoch+int(packet['slot_us']),publish,*sides,*raw_state,r['lateral_deg'],r['rate_dps'],
                r['fore_aft_deg'],r['accel_dps2'],r['i0_ma'] if fresh_current else None,r['goal_ma'],rpm,r['cap_ms'])
            # Sensitivity for early logs: do not invent a current or timestamp.
            # Use the already-recorded i0 only, even without proof its read
            # followed the last zero. This can over-accept on those old runs.
            if 'current_after_zero_ms' not in r and gates==0 and r['i0_ma']!='':
                fields=stream[-1].split()
                fields[-4]=r['i0_ma']
                relaxed_slots[len(stream)-1]=' '.join(fields)
            snapshot[(peak,now)] = dict(speed_dps=-float(r['rate_dps']),angle_deg=float(r['lateral_deg']),
                                        packet=count,i0_ma=float(r['i0_ma']) if fresh_current else None,
                                        goal_ma=float(r['goal_ma']),rpm=rpm,guarded_envelope_ms=int(r['cap_ms']))
        case['saved_slots']=len(set(counts))
        case['skipped_slots']=max(counts)-min(counts)+1-len(set(counts)) if counts else 0
    def run(lines):
        return subprocess.run([exe],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True).stdout.splitlines()

    if relaxed_slots:
        for line in run([relaxed_slots.get(i,row) for i,row in enumerate(stream)]):
            if not line.startswith('A '):
                fields=line.split()
                cases[int(fields[0])]['with_unverified_current_ms']=int(fields[1])
    for line in run(stream):
        if line.startswith('A '):
            cases[int(line.split()[1])]['armed']=True
            continue
        peak, now, req, cap, wlo, whi, worklo, workhi=line.split()
        p,t=int(peak),int(now)
        if cases[p]['decision'] is not None: raise ValueError('Multiple second aids on one half-cycle')
        cases[p]['decision']=dict(time_ms=t,request_uJ=1e6*float(req),loss_cap_ms=int(cap),
                                  min_width_ms=int(wlo) if int(whi)>=0 else None,
                                  max_width_ms=int(whi) if int(whi)>=0 else None,
                                  gain_over_coast_uJ=[1e6*float(worklo),1e6*float(workhi)],**snapshot[p,t])
    return list(cases.values())


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots',nargs='+',type=Path)
    parser.add_argument('--json-out',type=Path)
    args=parser.parse_args()
    folders={p.parent for root in args.roots for p in root.glob('*/header.json')}
    if not folders: parser.error('No converted recordings')
    with tempfile.TemporaryDirectory() as tmp:
        exe=Path(tmp)/'post_aid.exe'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',HERE.parents[1]/'src',
                        HERE/'post_aid_selector.cpp','-o',exe],check=True)
        data=[row for folder in sorted(folders) for row in compare(folder,exe)]
    print('Trials',len(folders),'aids',len(data), 'outcomes/selected',
          dict(Counter((e['outcome'],bool(e['decision'])) for e in data)))
    for e in data:
        if e['decision']: print(json.dumps(e, allow_nan=False))
    if args.json_out: args.json_out.write_text(json.dumps(data,allow_nan=False,indent=2),encoding='utf8')
