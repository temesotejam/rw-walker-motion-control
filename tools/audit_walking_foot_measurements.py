"""Audit archived walking foot measurements; never operate hardware.

Input is an existing convert_rwlog_to_csv output plus its original RWLOG.
Frame time is a driver timestamp, not a calibrated exposure timestamp.
Camera-relative angles and recognition validity do not establish true angles.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import zlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from analyze_foot_latency import analyze, stats

REASONS = ['Ready', 'Warmup', 'Invalid', 'Timestamp', 'Gap', 'Jump', 'Stale', 'Horizon']


def audit(folder, original):
    raw = original.read_bytes()
    header = json.loads((folder / 'header.json').read_text(encoding='utf8'))
    meta = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
    assert len(raw) == header['crc_offset'] + 4
    assert zlib.crc32(raw[:-4]) == int.from_bytes(raw[-4:], 'little')
    frames = [r for r in meta['foot_frames'] if r['state_id'] == 3]
    samples = [r for r in csv.DictReader((folder / 'timeseries.csv').open(encoding='utf8')) if r['state_id'] == '3']
    epoch_set = {r['frame_us'] - r['measurement_time_us'] for r in frames}
    assert len(epoch_set) == 1
    epoch = epoch_set.pop()
    end = float(samples[-1]['time_s'])
    both = [r for r in frames if r['left_valid'] and r['right_valid']]
    receipts = [r for r in both if r['control_observed_us'] > 0]
    assert all(a['sequence'] < b['sequence'] and a['frame_us'] < b['frame_us']
               for a, b in zip(frames, frames[1:]))
    assert all(a['control_observed_us'] < b['control_observed_us'] for a,b in zip(receipts, receipts[1:]))
    gaps = [{'time_s': (b['control_observed_us']-epoch)/1e6,
             'gap_ms': (b['control_observed_us']-a['control_observed_us'])/1000}
            for a,b in zip(receipts, receipts[1:])]
    windows = {}
    for width in [1, 5]:
        windows[str(width)] = [
            {'start_s': i*width, 'end_s': (i+1)*width,
             'valid_receipts': sum(i*width <= (r['control_observed_us']-epoch)/1e6 < (i+1)*width for r in receipts)}
            for i in range(int(end//width))]
        for w in windows[str(width)]:
            w['hz'] = w['valid_receipts']/width
    latency, _, _ = analyze(folder)
    angles = {}
    def calibrated(points, x):
        points = sorted(points)
        i = next((i for i in range(len(points)-1) if x <= points[i+1][0]), len(points)-2)
        (x0,y0),(x1,y1) = points[i:i+2]
        return y0+(x-x0)*(y1-y0)/(x1-x0)
    for side in ['left','right']:
        valid = [r for r in frames if r[side+'_valid']]
        points = meta['foot_observation']['calibration'][side+'_points']
        zero = meta['foot_observation'][side+'_zero_x']
        mapping_errors = [abs(r[side+'_deg']-(calibrated(points,r[side+'_x'])-calibrated(points,zero))) for r in valid]
        assert max(mapping_errors)<0.0001
        lo, hi = [fn(valid, key=lambda r:r[side+'_deg']) for fn in [min,max]]
        def point(r):
            return {'time_s':r['measurement_time_us']/1e6,'deg':r[side+'_deg'],
                    'x':r[side+'_x'],'scan_y':r[side+'_scan_y']}
        changes = [{'time_s':b['measurement_time_us']/1e6,
                    'dt_ms':(b['frame_us']-a['frame_us'])/1000,
                    'delta_deg':b[side+'_deg']-a[side+'_deg'],
                    'rate_dps':(b[side+'_deg']-a[side+'_deg'])*1e6/(b['frame_us']-a['frame_us']),
                    'from_deg':a[side+'_deg'],'to_deg':b[side+'_deg']}
                   for a,b in zip(frames,frames[1:]) if a[side+'_valid'] and b[side+'_valid']]
        angles[side] = {'valid':len(valid),'min':point(lo),'max':point(hi),'final':point(valid[-1]),
                        'calibration_recalculation_abs_error_deg':stats(mapping_errors),
                        'reasons':dict(Counter(r[side+'_reason'] for r in frames)),
                        'control_range_rejected':sum(not r[side+'_control_range_ok'] for r in valid),
                        'adjacent_abs_delta_deg':stats(abs(r['delta_deg']) for r in changes),
                        'adjacent_rate_exceeds_80dps':sum(abs(r['rate_dps'])>80 for r in changes),
                        'largest_changes':sorted(changes,key=lambda r:abs(r['delta_deg']),reverse=True)[:8]}
    sequences = {r['sequence'] for r in frames}
    predictions = {}
    for name in ['foot_pose_prediction','foot_local_prediction']:
        rows = [r for r in meta[name]['records'] if r['sequence'] in sequences]
        predictions[name] = {'counts':dict(Counter(REASONS[r['reason']] for r in rows)),
                             'ready_percent':100*sum(r['reason']==0 for r in rows)/len(rows),
                             'last_reasons':[{'sequence':r['sequence'],'reason':REASONS[r['reason']]} for r in rows[-6:]]}
    events = meta['walking_energy']['records']
    executed = [r for r in events if r['executed']]
    aids = [r for r in executed if r['return_event']]
    decisions = []
    all_receipts = [r for r in frames if r['control_observed_us']>0]
    for e in aids:
        available = [r for r in all_receipts if r['control_observed_us']<=epoch+e['time_ms']*1000]
        last = max(available,key=lambda r:r['control_observed_us']) if available else None
        decisions.append({k:e[k] for k in ['time_ms','return_evidence','return_foot_age_ms','actual_elapsed_ms','request_j','output_work_pred_j']})
        decisions[-1]['latest_recorded_foot'] = ({k:last[k] for k in ['sequence','measurement_time_us','left_deg','right_deg','left_valid','right_valid']} if last else None)
    checks = {'crc_and_length':True,'unique_epoch':True,'monotone_frames_and_receipts':True,
              'foot_archive_no_overflow':not meta['foot_observation']['overflow'],
              'energy_archive_complete':len(events)==meta['walking_energy']['total'] and not meta['walking_energy']['truncated'],
              'output_completion_records_consistent':all(r['completion_valid'] and r['output_end_ms']-r['output_start_ms']==r['actual_elapsed_ms'] for r in executed)}
    assert all(checks.values()), checks
    result = {'source':str(original),'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw),
              'build':meta['diagnostic_build_id'],'run_id':header['run_id'],'terminal':meta['terminal_state'],
              'measurement_end_s':end,'active_frames':len(frames),'both_valid':len(both),
              'both_valid_percent':100*len(both)/len(frames),'valid_receipts':len(receipts),
              'missing_receipt_frames':sum(r['control_observed_us']<=0 for r in frames),
              'valid_frame_hz_over_active_duration':len(both)/end,
              'valid_receipt_hz_over_receipt_span':(len(receipts)-1)*1e6/(receipts[-1]['control_observed_us']-receipts[0]['control_observed_us']),
              'valid_receipt_gap_ms':stats(r['gap_ms'] for r in gaps),
              'gaps_over_50ms':[r for r in gaps if r['gap_ms']>50],
              'complete_windows':windows,'latency':latency,'angles':angles,'predictions':predictions,
              'energy':{'decisions':len(events),'executed':len(executed),'return_aids':decisions,
                        'last_output_end_ms':max(r['output_end_ms'] for r in executed),
                        'max_commanded_ms':max(r['guarded_ms'] for r in executed),
                        'max_actual_elapsed_ms':max(r['actual_elapsed_ms'] for r in executed),
                        'elapsed_overshoot_ms':stats(r['actual_elapsed_ms']-r['guarded_ms'] for r in executed)},
              'fore_aft_guard':meta['walking_energy']['fore_aft_motion_guard'],'checks':checks,
              'control_deadline':meta['v46p_control_worker']['v46u_deadline'],
              'limits':['Angles are camera/body-relative, not ground-referenced or independently validated.',
                        'Driver timestamps are not calibrated exposure times.',
                        'Positive recognition does not prove correct marker association.',
                        'Latest-recorded causal foot association is not a per-decision frame ID.',
                        'No replay establishes physical effects of alternative controls.']}
    (folder/'foot_measurement_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf8')
    fig,ax=plt.subplots(4,1,figsize=(13,10),sharex=True,constrained_layout=True)
    t=[r['measurement_time_us']/1e6 for r in frames]
    for side,color in [('left','#d56a24'),('right','#16856a')]:
        ax[0].plot(t,[r[side+'_deg'] if r[side+'_valid'] else float('nan') for r in frames],label=side,color=color,lw=1)
        invalid=[r for r in frames if not r[side+'_valid']]
        ax[0].scatter([r['measurement_time_us']/1e6 for r in invalid],[-2 if side=='left' else -3]*len(invalid),marker='x',color=color,s=20)
    ax[0].set_ylabel('Observed foot (deg)');ax[0].legend();ax[0].set_title(folder.name+': actual archived measurements (crosses = invalid)')
    st=[float(r['time_s']) for r in samples]
    ax[1].plot(st,[float(r['roll_mekf_abs_deg']) for r in samples],label='fore/aft (MEKF roll)')
    ax[1].plot(st,[float(r['pitch_mekf_abs_deg']) for r in samples],label='lateral (MEKF pitch)',alpha=.7)
    ax[1].set_ylabel('Body (deg)');ax[1].legend()
    known=[r for r in frames if r['control_observed_us']>0]
    ax[2].plot([r['measurement_time_us']/1e6 for r in known],[(r['control_observed_us']-r['frame_us'])/1000 for r in known],label='Frame to first control',lw=.8)
    ax[2].plot([r['time_s'] for r in gaps],[r['gap_ms'] for r in gaps],label='Both-valid receipt interval',lw=.6,alpha=.6)
    ax[2].axhline(25,color='gray',ls='--',lw=.7);ax[2].set_ylabel('Time (ms)');ax[2].legend()
    ax[3].plot(st,[float(r['motor_cmd_mA']) for r in samples],label='Command',lw=.8)
    ax[3].plot(st,[float(r['roller_actual_current_mA']) for r in samples],label='Readback',lw=.8,alpha=.7)
    ax[3].set_ylabel('Current (mA)');ax[3].legend();ax[3].set_xlabel('Measurement time (s)')
    for a in ax:a.grid(alpha=.2);a.set_xlim(0,end)
    fig.savefig(folder/'foot_measurement_audit.png',dpi=145);plt.close(fig)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--rwlog',type=Path,required=True)
    args=parser.parse_args();r=audit(args.folder,args.rwlog)
    print(json.dumps({k:r[k] for k in ['build','measurement_end_s','active_frames','both_valid','both_valid_percent','valid_receipt_hz_over_receipt_span','valid_receipt_gap_ms','gaps_over_50ms','predictions','checks']},indent=2))
