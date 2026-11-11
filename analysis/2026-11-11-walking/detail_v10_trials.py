"""Readable trial detail; body-angle alignment is approximate, never contact truth."""
from pathlib import Path
import csv,json,hashlib
from collections import Counter
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
CASES={'137267573':'20_26_36','267557373':'20_28_46'}
def read(p):
    with p.open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def good(r):
    return all(r[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range'])

for case,name in CASES.items():
    out=ROOT/case
    result=json.loads((out/'v10_analysis.json').read_text(encoding='utf8'))
    feet=read(out/'foot_angles.csv');cad=read(out/'cad_support_shadow.csv')
    peaks=read(out/'energy_control_autonomous_peak_events.csv')
    offsets=[float(x['log_time_us'])/1e6-float(x['measurement_time_us'])/1e6 for x in feet if int(x['measurement_time_us'])>=0]
    offset=float(np.median(offsets))
    ft=np.array([float(x['log_time_us'])/1e6-offset for x in feet])
    video=Path(r'C:\Users\arika\Pictures\Camera Roll')/('WIN_20261111_'+name+'_Pro.mp4')
    log=Path(r'C:\Users\arika\Downloads')/('energy_control_autonomous_run_'+('2' if case=='137267573' else '3')+'_'+case+'.rwlog')
    cap=cv2.VideoCapture(str(video))
    # Two complete lateral cycles, sampled every 100 ms.
    dt=result['approx_motion_fit']['video_minus_test_s']
    for start in [10,22]:
        fig,axes=plt.subplots(6,4,figsize=(12,16))
        for k,a in enumerate(axes.flat):
            sec=dt+start+k*.1;cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,im=cap.read()
            a.axis('off')
            if ok:a.imshow(cv2.cvtColor(im[210:655,470:880],cv2.COLOR_BGR2RGB))
            a.set_title(f'video {sec:.2f} s')
        fig.suptitle('100 ms samples | log correspondence uses body motion, not LED sync')
        fig.tight_layout();fig.savefig(out/f'video_detail_{start}s.png',dpi=110);plt.close(fig)
    cap.release()
    optical=np.loadtxt(out/'video_body_markers.csv',delimiter=',',skiprows=1)
    ts=read(out/'timeseries.csv');t=np.array([float(x['log_time_s'])-offset for x in ts]);a=np.array([float(x['pitch_mekf_detector_relative_deg']) for x in ts])
    fig,axes=plt.subplots(3,1,figsize=(14,10),constrained_layout=True)
    for ax,(lo,hi) in zip(axes,[(0,30),(10,13),(22,25)]):
        ax.plot(t,a,label='IMU detector',lw=1)
        ax.plot(optical[:,0]-dt,(optical[:,1]-result['approx_motion_fit']['bias_deg'])/result['approx_motion_fit']['gain'],label='white markers, affine fit',lw=1,alpha=.7)
        ax.set_xlim(lo,hi);ax.set_ylim(-10,8);ax.grid(alpha=.3);ax.legend();ax.set_xlabel('Approximate measurement time [s]');ax.set_ylabel('Body angle [deg]')
    fig.savefig(out/'motion_alignment.png',dpi=130);plt.close(fig)
    measurement=[c for c,f in zip(cad,feet) if f['state_id']=='3' and 0<=float(f['measurement_time_us'])<=30000000]
    cycles=[]
    for cycle in result['cycles']:
        i=np.where((ft>=cycle['start_s'])&(ft<cycle['end_s']))[0]
        r={'start_s':cycle['start_s'],'end_s':cycle['end_s'],'frames':len(i),'both_valid':sum(good(feet[j]) for j in i)}
        for side in ['left','right']:
            vals=[float(feet[j][side+'_deg']) for j in i if good(feet[j])]
            r[side+'_span_deg']=max(vals)-min(vals) if len(vals)>=3 else None
            r[side+'_max_deg']=max(vals) if len(vals)>=3 else None
        cycles.append(r)
    with (out/'body_cycle_foot_spans.csv').open('w',encoding='utf8',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(cycles[0]));w.writeheader();w.writerows(cycles)
    strict=[feet[i] for i in range(len(feet)) if 2<=ft[i]<=28 and good(feet[i])]
    excursion_intervals={}
    for side in ['left','right']:
        # Descriptive camera-relative forward peaks, not counted ground steps.
        hits=[(float(r['measurement_time_us'])/1e6,float(r[side+'_deg'])) for r in strict if float(r[side+'_deg'])>.3]
        groups=[]
        for pair in hits:
            if not groups or pair[0]-groups[-1][-1][0]>.4:groups.append([])
            groups[-1].append(pair)
        centers=[max(g,key=lambda q:q[1]) for g in groups]
        excursion_intervals[side]={'median_s':float(np.median(np.diff([x[0] for x in centers]))),'peaks_s_deg':centers,
                                  'definition':'valid camera angle >0.3deg, clusters separated by >0.4s; descriptive, not contact/steps'}
    ev=read(out/'energy_control_autonomous_zero_cross_events.csv')
    direction_counts=Counter((r['event_kind'],r['physical_next_peak_side'],r['pulse_width_ms']) for r in ev if r['output_executed']=='True')
    fig,axes=plt.subplots(3,1,figsize=(14,9),sharex=True,constrained_layout=True)
    for side,col in [('left','tab:blue'),('right','tab:red')]:
        axes[0].plot(ft,[float(f[side+'_deg']) if good(f) else np.nan for f in feet],'.-',label=side,color=col)
    axes[0].set_ylabel('Camera-relative foot [deg]');axes[0].legend()
    axes[1].plot(t,a,color='purple');axes[1].set_ylabel('Body detector [deg]')
    for r in ev:
        if r['output_executed']=='True':
            axes[2].vlines(float(r['pulse_start_ms'])/1000,0,float(r['pulse_width_ms'])*(1 if r['physical_next_peak_side']=='1' else -1),color='green' if r['event_kind']=='energy_control_zero_cross' else 'orange')
    axes[2].set_ylabel('Command width x side [ms]');axes[2].set_xlabel('Measurement time [s]')
    for ax in axes:ax.set_xlim(8,14);ax.grid(alpha=.3)
    fig.suptitle('Foot peaks can repeat every two body cycles; angle is not ground displacement')
    fig.savefig(out/'foot_body_input_detail.png',dpi=150);plt.close(fig)
    details={'video_sha256':hashlib.sha256(video.read_bytes()).hexdigest(),'rwlog_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
      'measurement_foot_frames':len(measurement),'measurement_reason_counts':dict(Counter(c['reason'] for c in measurement)),
      'ready_compute_median_us':float(np.median([float(c['compute_us']) for c in cad if c['reason']=='0'])),
      'ready_age_median_ms':float(np.median([float(c['age_ms']) for c in cad if c['reason']=='0'])),
      'ready_horizon_median_ms':float(np.median([float(c['horizon_ms']) for c in cad if c['reason']=='0'])),
      'body_cycle_median_s':float(np.median(np.diff([float(p['peak_time_ms'])/1000 for p in peaks if p['physical_peak_side']=='-1']))),
      'forward_excursion_intervals':excursion_intervals,
      'executed_event_kind_side_width_counts':{'|'.join(k):v for k,v in direction_counts.items()},
      'cycle_span_median_deg':{s:float(np.median([c[s+'_span_deg'] for c in cycles if c[s+'_span_deg'] is not None and c['start_s']>=2 and c['end_s']<=28])) for s in ['left','right']},
      'raw_relative_foot_range_2_28s_deg':{s:[min(float(f[s+'_deg']) for f in strict),max(float(f[s+'_deg']) for f in strict)] for s in ['left','right']},
      'peak_amplitude_median_deg':{s:float(np.median([float(p['peak_amplitude_deg']) for p in peaks if p['physical_peak_side']==s and 2000<=float(p['peak_time_ms'])<=28000])) for s in ['1','-1']}}
    (out/'v10_detail.json').write_text(json.dumps(details,ensure_ascii=False,indent=2),encoding='utf8')
    active=[r for r in feet if r['state_id']=='3']
    observed=[r for r in active if r['control_observed_us']]
    def stat(values):
        return dict(zip(['min','median','p95','max'],map(float,np.percentile(values,[0,50,95,100]))))
    latency={
      'driver_frame_to_receive_ms':stat([(int(r['delivered_us'])-int(r['frame_us']))/1000 for r in active]),
      'angle_processing_ms':stat([int(r['processing_us'])/1000 for r in active]),
      'driver_frame_to_angle_ready_estimate_ms':stat([(int(r['delivered_us'])-int(r['frame_us'])+int(r['processing_us']))/1000 for r in active]),
      'frame_interval_ms':stat(np.diff([int(r['frame_us']) for r in active])/1000),
      'driver_frame_to_first_control_ms':stat([(int(r['control_observed_us'])-int(r['frame_us']))/1000 for r in observed]),
      'receive_to_first_control_ms':stat([(int(r['control_observed_us'])-int(r['delivered_us']))/1000 for r in observed]),
      'control_observed_records':len(observed),
      'control_observation_metadata':{k:v for k,v in json.loads((out/'metadata.json').read_text(encoding='utf8'))['foot_control_observations'].items() if k!='rows'},
      'timestamp_semantics':'camera_driver_frame_timestamp_not_verified_exposure_time',
      'caution':'First control snapshot only, strided subset of observations. Reuse age and motor response are additional.'}
    (out/'foot_latency.json').write_text(json.dumps(latency,indent=2),encoding='utf8')
    print(case,json.dumps({'latency':latency,'body_cycle_median_s':details['body_cycle_median_s']},ensure_ascii=False))
