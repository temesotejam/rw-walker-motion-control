"""Separate recorded foot-camera latency stages without filling missing times.

Driver frame time is capture/DMA-start time, not calibrated exposure time.
Pose compute includes local compute. v20 raw ready_us is a prepublication
timestamp; older pose ready_ms values have 1ms quantization. Older control
observations are sparse, rank-sampled snapshot-return timestamps.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
SOURCES = {
    'bundled_camera_commit': 'f0bb429',
    'bundled_versions': str(Path.home() / '.platformio/packages/framework-arduinoespressif32@3.20016.0/tools/sdk/versions.txt'),
    'camera_hal': 'https://github.com/espressif/esp32-camera/blob/f0bb429/driver/cam_hal.c',
    'camera_s3': 'https://github.com/espressif/esp32-camera/blob/f0bb429/target/esp32s3/ll_cam.c',
    'camera_readme': 'https://github.com/espressif/esp32-camera/blob/f0bb429/README.md',
}
METRICS = ['frame_interval_ms', 'delivery_interval_ms', 'driver_to_delivery_ms',
    'recognition_block_ms', 'pose_total_ms', 'local_inside_pose_ms', 'pose_history_and_overhead_ms',
    'capture_wait_ms', 'row_copy_ms', 'detector_ms', 'delivered_to_release_ms', 'release_to_ready_ms',
    'driver_to_ready_exact_ms', 'ready_to_control_exact_ms',
    'driver_to_ready_lower_ms', 'ready_to_control_upper_ms', 'driver_to_control_ms', 'delivery_to_control_ms']


def number(value):
    if value is None or value == '':
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values, fraction):
    values = sorted(values)
    position = (len(values)-1)*fraction
    low = int(position);high = min(len(values)-1, low+1)
    return values[low] + (values[high]-values[low])*(position-low)


def stats(values):
    values = list(values)
    known = [x for x in values if x is not None and math.isfinite(x)]
    out = {'n': len(known), 'missing': len(values)-len(known)}
    if known:
        out.update({'min': min(known), 'median': statistics.median(known),
            'p90': percentile(known,.90), 'p95': percentile(known,.95),
            'p99': percentile(known,.99), 'max': max(known), 'mean': statistics.mean(known)})
    return out


def frames(meta, trial):
    raw = meta['foot_frames']
    pose = {r['sequence']:r for r in meta.get('foot_pose_prediction',{}).get('records',[])}
    local = {r['sequence']:r for r in meta.get('foot_local_prediction',{}).get('records',[])}
    epoch_counts = Counter(r['frame_us']-r['measurement_time_us'] for r in raw
        if r.get('timestamp_valid') and number(r.get('measurement_time_us')) is not None and r['measurement_time_us']>=0)
    epoch = epoch_counts.most_common(1)[0][0] if epoch_counts else None
    rows = [];previous = None
    for index, f in enumerate(raw):
        p = pose.get(f['sequence'],{});l = local.get(f['sequence'],{})
        frame = number(f.get('frame_us'));delivered = number(f.get('delivered_us'))
        control = number(f.get('control_observed_us'))
        if control is not None and control <= 0:
            control = None
        timestamp_ok = f.get('timestamp_valid') and frame is not None and frame>0 and delivered is not None and delivered>=frame
        ready_ms = number(p.get('ready_ms'))
        measurement_us = number(f.get('measurement_time_us'))
        raw_ready = number(f.get('ready_us'))
        if 'ready_us' in f:
            # A present raw zero/invalid value remains unavailable; do not
            # replace it with a different stage's older rounded timestamp.
            ready = raw_ready if raw_ready is not None and raw_ready>0 else None
            ready_source = 'ready_us' if ready is not None else 'unavailable'
            ready_error = 0 if ready is not None else None
        else:
            ready = epoch+ready_ms*1000 if epoch is not None and timestamp_ok and measurement_us is not None and measurement_us>=0 and ready_ms is not None and ready_ms>0 else None
            ready_source = 'pose_ready_ms' if ready is not None else 'unavailable'
            ready_error = 1000 if ready is not None else None
        released = number(f.get('released_us'))
        if released is not None and released<=0:
            released = None  # no borrowed frame was returned
        compute = number(p.get('compute_us'));local_compute = number(l.get('compute_us'))
        processing = number(f.get('processing_us'))
        capture_wait = number(f.get('capture_wait_us'));row_copy = number(f.get('row_copy_us'))
        recognition = number(f.get('recognition_us'))
        r = {'trial':trial,'index':index,'sequence':f['sequence'],'state_id':f['state_id'],
            'measurement_time_us':measurement_us,'frame_us':frame,'delivered_us':delivered,
            'ready_lower_us':ready,'ready_upper_us':ready+ready_error if ready is not None else None,
            'ready_source':ready_source,'ready_quantization_us':ready_error,'released_us':released,
            'row_cache_used':f.get('row_cache_used'),
            'control_observed_us':control,'timestamp_valid':bool(timestamp_ok),
            'frame_valid':bool(f.get('frame_valid')),'zero_ready':bool(f.get('zero_ready')),
            'left_valid':bool(f.get('left_valid')),'right_valid':bool(f.get('right_valid')),
            'left_reason':f.get('left_reason'),'right_reason':f.get('right_reason'),
            'pose_reason':p.get('reason'),'local_reason':l.get('reason'),
            'frame_interval_ms':(frame-previous['frame_us'])/1000 if timestamp_ok and previous and previous['timestamp_valid'] and frame>previous['frame_us'] else None,
            'delivery_interval_ms':(delivered-previous['delivered_us'])/1000 if delivered is not None and previous and delivered>previous['delivered_us'] else None,
            'driver_to_delivery_ms':(delivered-frame)/1000 if timestamp_ok else None,
            'recognition_block_ms':processing/1000 if processing is not None and processing>=0 else None,
            'capture_wait_ms':capture_wait/1000 if capture_wait is not None and capture_wait>=0 else None,
            'row_copy_ms':row_copy/1000 if row_copy is not None and row_copy>=0 else None,
            'detector_ms':recognition/1000 if recognition is not None and recognition>=0 else None,
            'delivered_to_release_ms':(released-delivered)/1000 if released is not None and delivered is not None else None,
            'release_to_ready_ms':(ready-released)/1000 if released is not None and ready is not None else None,
            'driver_to_ready_exact_ms':(ready-frame)/1000 if ready_source=='ready_us' and timestamp_ok else None,
            'ready_to_control_exact_ms':(control-ready)/1000 if ready_source=='ready_us' and control is not None else None,
            'pose_total_ms':compute/1000 if compute is not None and compute>=0 else None,
            'local_inside_pose_ms':local_compute/1000 if local_compute is not None and local_compute>=0 else None,
            'pose_history_and_overhead_ms':(compute-local_compute)/1000 if compute is not None and local_compute is not None and compute>=local_compute else None,
            'driver_to_ready_lower_ms':(ready-frame)/1000 if ready is not None and timestamp_ok else None,
            'ready_to_control_upper_ms':(control-ready)/1000 if control is not None and ready is not None else None,
            'driver_to_control_ms':(control-frame)/1000 if control is not None and timestamp_ok else None,
            'delivery_to_control_ms':(control-delivered)/1000 if control is not None and delivered is not None else None,
            'pose_local_ready_equal':p.get('ready_ms')==l.get('ready_ms') if p and l else None}
        rows.append(r);previous = r
    return rows, epoch, epoch_counts


def analyze(path):
    blob = (path/'metadata.json').read_bytes();meta = json.loads(blob)
    rows, epoch, epoch_counts = frames(meta,path.name)
    active = [r for r in rows if r['state_id']==3]
    # Availability is an inferred camera-side lower bound, not proof that the
    # control poll has already copied a particular unarchived frame.
    ready = sorted((r for r in active if r['ready_upper_us'] is not None),key=lambda r:r['ready_upper_us'])
    decisions = []
    for a in meta.get('walking_energy',{}).get('records',[]):
        now = epoch+a['time_ms']*1000 if epoch is not None else None
        available = [r for r in ready if now is not None and r['ready_upper_us']<=now]
        latest = available[-1] if available else None
        decisions.append({'trial':path.name,'decision_ms':a['time_ms'],
            'latest_completed_sequence':latest['sequence'] if latest else None,
            'latest_completed_driver_age_ms':(now-latest['frame_us'])/1000 if latest else None,
            'latest_completed_both_markers_valid':latest['left_valid'] and latest['right_valid'] if latest else None,
            'return_aid_archived_frame_age_ms':a.get('return_foot_age_ms') if a.get('return_event') else None})
    sparse = meta.get('foot_control_observations',{})
    observed = [r for r in active if r['control_observed_us'] is not None]
    summary = {'trial':path.name,'build':meta.get('diagnostic_build_id'),
        'metadata_sha256':hashlib.sha256(blob).hexdigest(),'frames_total':len(rows),'active_frames':len(active),
        'state_counts':dict(Counter(r['state_id'] for r in rows)),
        'measurement_epoch_us':epoch,'distinct_measured_epochs':len(epoch_counts),
        'active_valid_frames':sum(r['frame_valid'] for r in active),
        'active_valid_timestamps':sum(r['timestamp_valid'] for r in active),
        'ready_timestamp_sources':dict(Counter(r['ready_source'] for r in active)),
        'active_both_markers_valid':sum(r['left_valid'] and r['right_valid'] for r in active),
        'active_marker_dropout_frames':sum(not(r['left_valid'] and r['right_valid']) for r in active),
        'active_marker_reasons':{side:dict(Counter(r[side+'_reason'] for r in active)) for side in ['left','right']},
        'stages_active_ms':{k:stats(r[k] for r in active) for k in METRICS},
        'stages_both_markers_valid_ms':{k:stats(r[k] for r in active if r['left_valid'] and r['right_valid']) for k in METRICS},
        'stages_marker_dropout_ms':{k:stats(r[k] for r in active if not(r['left_valid'] and r['right_valid'])) for k in METRICS},
        'receipt_archive':{'known_active':len(observed),'missing_active':len(active)-len(observed),
            'metadata':{k:v for k,v in sparse.items() if k!='rows'},
            'sampling':('v20 raw per-frame receipts; zero/missing timestamps remain unknown' if any('ready_us' in f for f in meta['foot_frames']) else
                'up to64 uniform ranks over valid observations; not first64; omitted per-frame times remain unknown'),
            'known_measurement_time_bounds_ms':[
                min(r['measurement_time_us'] for r in observed)/1000,
                max(r['measurement_time_us'] for r in observed)/1000] if observed else None},
        'latest_completed_driver_age_at_decisions_ms':stats(r['latest_completed_driver_age_ms'] for r in decisions),
        'return_aid_archived_frame_age_ms':stats(r['return_aid_archived_frame_age_ms'] for r in decisions if r['return_aid_archived_frame_age_ms'] is not None),
        'checks':{'pose_local_ready_mismatch':sum(r['pose_local_ready_equal'] is False for r in active),
            'local_compute_exceeds_pose_total':sum(r['pose_history_and_overhead_ms'] is None and r['pose_total_ms'] is not None and r['local_inside_pose_ms'] is not None for r in active),
            'observed_before_delivery':sum(r['delivery_to_control_ms'] is not None and r['delivery_to_control_ms']<0 for r in active),
            'ready_before_delivery':sum(r['ready_upper_us'] is not None and r['delivered_us'] is not None and r['ready_upper_us']<r['delivered_us'] for r in active),
            'released_before_delivery':sum(r['delivered_to_release_ms'] is not None and r['delivered_to_release_ms']<0 for r in active),
            'ready_before_release':sum(r['release_to_ready_ms'] is not None and r['release_to_ready_ms']<0 for r in active),
            'observed_before_ready_lower':sum(r['ready_to_control_upper_ms'] is not None and r['ready_to_control_upper_ms']<0 for r in active)}}
    assert not any(summary['checks'].values()),summary['checks']
    return summary,rows,decisions


def write_csv(path,rows):
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',encoding='utf8',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def plot(path,summaries,all_rows):
    fig,axes = plt.subplots(3,1,figsize=(12,11),constrained_layout=True)
    names = [s['trial'] for s in summaries]
    colors = ['#4a78a8','#c68c41','#6d9d75']
    bottoms = [0.]*len(summaries)
    for key,label,color in zip(['driver_to_delivery_ms','recognition_block_ms','pose_total_ms'],
            ['Driver capture/start to delivery','Processing block (row copy included in v20)','Pose total (includes local)'],colors):
        heights = [s['stages_active_ms'][key].get('median',0) for s in summaries]
        axes[0].bar(names,heights,bottom=bottoms,label=label,color=color)
        bottoms = [a+b for a,b in zip(bottoms,heights)]
    axes[0].scatter(names,[s['stages_active_ms']['driver_to_control_ms'].get('median',float('nan')) for s in summaries],
        color='black',marker='x',s=55,label='Recorded first control snapshot receipt')
    axes[0].set_ylabel('Milliseconds (medians)');axes[0].legend(fontsize=8,loc='lower right')
    axes[0].set_title('Foot-camera latency baseline: separate stages; no missing timestamps imputed')
    if any(s['stages_active_ms']['pose_total_ms']['missing'] for s in summaries):
        axes[0].text(.01,.98,'Missing optional pose stages omitted from bars',transform=axes[0].transAxes,va='top',fontsize=8)
    for index,name in enumerate(names):
        rows = [r for r in all_rows if r['trial']==name and r['state_id']==3]
        intervals=[r for r in rows if r['frame_interval_ms'] is not None and r['measurement_time_us'] is not None]
        x = [r['measurement_time_us']/1e6 for r in intervals]
        axes[1].plot(x,[r['frame_interval_ms'] for r in intervals],'.',ms=3,label=name)
        known = [r['driver_to_control_ms'] for r in rows if r['driver_to_control_ms'] is not None]
        axes[2].hist(known,bins=12,alpha=.4,label=name+' known receipt age')
    axes[1].axhline(66.667,color='gray',ls='--',lw=.8,label='15Hz application target')
    if any('ready_us' in s['ready_timestamp_sources'] for s in summaries):
        axes[1].axhline(40,color='gray',ls=':',lw=.8,label='v20 25Hz worker minimum period')
    axes[1].set_ylabel('Driver frame interval (ms)');axes[1].set_xlabel('Measurement-relative frame time (s)');axes[1].legend(fontsize=8)
    axes[2].set_xlabel('Driver start timestamp to first control receipt (ms)');axes[2].set_ylabel('Known sampled frames');axes[2].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(path/'foot_latency_baseline.png',dpi=150);plt.close(fig)


def self_test():
    meta = {'foot_frames':[{'sequence':1,'state_id':3,'frame_us':1001000,'delivered_us':1051000,
        'measurement_time_us':1000,'timestamp_valid':True,'frame_valid':True,'processing_us':12000}],
        'foot_pose_prediction':{'records':[{'sequence':1,'ready_ms':64,'compute_us':200}]},
        'foot_local_prediction':{'records':[{'sequence':1,'ready_ms':64,'compute_us':50}]}}
    rows,_,_ = frames(meta,'synthetic');r=rows[0]
    assert r['driver_to_delivery_ms']==50 and r['recognition_block_ms']==12
    assert r['pose_total_ms']==.2 and r['local_inside_pose_ms']==.05 and r['pose_history_and_overhead_ms']==.15
    assert r['driver_to_control_ms'] is None and r['ready_to_control_upper_ms'] is None
    assert r['ready_source']=='pose_ready_ms' and r['ready_quantization_us']==1000
    assert r['ready_lower_us']==1064000 and r['ready_upper_us']==1065000
    assert r['detector_ms'] is None and r['released_us'] is None
    meta['foot_frames'][0].update({'ready_us':1064789,'released_us':1051200,
        'control_observed_us':1066000,'capture_wait_us':2200,'row_copy_us':100,'recognition_us':12500,'row_cache_used':True})
    r=frames(meta,'v20 raw')[0][0]
    assert r['ready_source']=='ready_us' and r['ready_quantization_us']==0
    assert r['ready_lower_us']==1064789 and r['ready_upper_us']==1064789
    assert r['driver_to_ready_exact_ms']==63.789 and r['ready_to_control_exact_ms']==1.211
    assert r['capture_wait_ms']==2.2 and r['row_copy_ms']==.1 and r['detector_ms']==12.5
    assert r['delivered_to_release_ms']==.2 and r['release_to_ready_ms']==13.589
    # Raw ready survives truncated optional predictor tables and absent epoch.
    meta.pop('foot_pose_prediction');meta.pop('foot_local_prediction')
    meta['foot_frames'][0]['measurement_time_us']=-1
    r=frames(meta,'v20 truncated')[0][0]
    assert r['ready_lower_us']==1064789 and r['pose_total_ms'] is None
    # Missing release/receipt are not fabricated zero-latency observations.
    meta['foot_frames'][0].update({'released_us':0,'control_observed_us':0,'ready_us':0})
    r=frames(meta,'v20 missing')[0][0]
    assert r['released_us'] is None and r['driver_to_control_ms'] is None and r['ready_lower_us'] is None
    assert stats([None,1,2])['missing']==1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials',type=Path,nargs='+')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();self_test()
    summaries=[];rows=[];decisions=[]
    for trial in args.trials:
        s,r,d=analyze(trial);summaries.append(s);rows.extend(r);decisions.extend(d)
    args.out.mkdir(parents=True,exist_ok=True)
    versions_path=Path(SOURCES['bundled_versions'])
    sdk_versions=versions_path.read_text(encoding='utf8') if versions_path.is_file() else None
    result = {'sources':SOURCES,'sdk_versions_text':sdk_versions,'trials':summaries,
        'source_snapshot_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [ROOT/'src/camera_coexistence.cpp',ROOT/'src/foot_observer.cpp',ROOT/'src/foot_tracking_config.h',ROOT/'src/main.cpp']},
        'interpretation':[
            'Driver frame timestamp is recorded by cam_start_frame after DMA start; exposure/rolling shutter timing is uncalibrated.',
            'Driver-to-delivery includes acquisition plus driver/task transfer, not exposure duration measured separately.',
            'processing_us covers both detectors, zero/angle conversion and optional preview; active runs skip the preview copy.',
            'pose_compute_us includes local_compute_us. Never add them as independent stages.',
            'v20 raw ready_us overrides optional pose ready_ms, including when predictor tables are truncated. This is recognition/projection complete before the publication lock, not exact publication or exposure.',
            'Older ready_ms is truncated: true ready lies in [ready_lower,ready_upper); ready-to-control values are upper bounds within1ms. Raw ready_us has zero quantization interval and separate exact-stage metrics.',
            'v20 released_us records return of a borrowed driver buffer; zero means no returned frame. capture_wait, row_copy, detector, release-to-ready and ready-to-control are kept separate; processing_us includes more than detector time.',
            'Exact publication time is absent. Ready-to-control combines publication overhead and control polling; first control receipt precedes runner.observe calls.',
            'Missing receipt timestamps are unknown. Older metadata stores at most64 uniformly ranked observations; v20 raw frame rows retain all recorded receipts.',
            'Latest completed frame age at decisions is a camera-availability inference, not proof of the actual per-decision frame received by control.',
            'Single-fb priority3 cam_task queues completed frame then immediately seeks free buffer before priority1 consumer can return it; early return alone need not remove skipped VSYNC.',
            'Two grayscale buffers are accepted by the pinned driver code, but its README recommends multiple buffers only for JPEG and warns of CPU/PSRAM load.',
            'GRAB_LATEST with fb_count2 creates queue length1; it replaces queued old frames, never a borrowed buffer. It is not a calibrated exposure-age guarantee.',
            'Pinned S3 cam_take resets DMA after any empty receive, even timeout0: do not introduce repeated cam_take(0) draining while capture is active.',
            'At the configured20MHz the pinned driver uses internal DMA staging and CPU copy to PSRAM; its old psram_mode is enabled only at16MHz.',
            'A second QVGA grayscale fb adds76800bytes of PSRAM plus metadata; raw capture/copy/ISR load can rise with delivered cadence.',
            'Reducing consumer period, caching sparse rows and returning the borrowed fb early can reduce ownership/load; faster raw capture and IMU deadlines require hardware measurements.']}
    (args.out/'foot_latency_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    write_csv(args.out/'foot_latency_frames.csv',rows);write_csv(args.out/'foot_latency_decisions.csv',decisions)
    plot(args.out,summaries,rows)
    for s in summaries:
        print(s['trial'],'frames',s['active_frames'],'receipt',s['receipt_archive']['known_active'],'dropout',s['active_marker_dropout_frames'])
        for k in ['frame_interval_ms','driver_to_delivery_ms','recognition_block_ms','pose_total_ms','driver_to_ready_lower_ms','driver_to_control_ms','ready_to_control_upper_ms']:
            print(' ',k,s['stages_active_ms'][k])


if __name__=='__main__':
    main()
