"""Two v10 recordings: diagnostics and optical body-marker motion matching.
Motion matching is approximate, not LED or contact-force synchronization.
"""
from pathlib import Path
import bisect,csv,json,hashlib,math,statistics
from collections import Counter
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
CASES={'137267573':r'C:\Users\arika\Pictures\Camera Roll\WIN_20261111_20_26_36_Pro.mp4',
       '267557373':r'C:\Users\arika\Pictures\Camera Roll\WIN_20261111_20_28_46_Pro.mp4'}
def read(p):
    with p.open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def n(v):
    try:return float(v)
    except (ValueError,TypeError):return math.nan
def good(f):
    return all(f[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range'])

for case,videopath in CASES.items():
    out=ROOT/case;s=read(out/'timeseries.csv');feet=read(out/'foot_angles.csv');cad=read(out/'cad_support_shadow.csv')
    ev=read(out/'energy_control_autonomous_zero_cross_events.csv');peaks=read(out/'energy_control_autonomous_peak_events.csv')
    meta=json.loads((out/'metadata.json').read_text(encoding='utf8'))
    offset=float(np.median([n(x['log_time_s'])-n(x['t_test_ms'])/1000 for x in s if .1<n(x['t_test_ms'])/1000<29.9]))
    st=np.array([n(x['log_time_s'])-offset for x in s]);angles=np.array([n(x['pitch_mekf_detector_relative_deg']) for x in s])
    ft=np.array([n(x['log_time_us'])/1e6-offset for x in feet]);index={int(x['sequence']):i for i,x in enumerate(feet)}
    errors=[];hold_errors=[];target_rows=[]
    for row in cad:
        if row['reason']!='0':continue
        i=index[int(row['sequence'])];target=ft[i]+n(row['horizon_ms'])/1000;j=bisect.bisect_left(ft,target)
        if not 0<j<len(feet) or not good(feet[j-1]) or not good(feet[j]) or ft[j]-ft[j-1]>.2:continue
        q=(target-ft[j-1])/(ft[j]-ft[j-1])
        for side in ['left','right']:
            actual=(1-q)*n(feet[j-1][side+'_deg'])+q*n(feet[j][side+'_deg'])
            pred=n(row['predicted_'+side+'_deg']);err=abs(actual-pred);hold=abs(actual-n(feet[i][side+'_deg']))
            errors.append(err);hold_errors.append(hold)
            target_rows.append({'sequence':int(row['sequence']),'side':side,'target_measurement_s':target,'predicted':pred,'interpolated_camera':actual,'abs_error':err,'hold_abs_error':hold})
    # Actual metadata prediction, not a replay of assumed timing.
    fig,ax=plt.subplots(4,1,figsize=(15,11),sharex=True,constrained_layout=True)
    for side,col in [('left','tab:blue'),('right','tab:red')]:
        vals=[n(f[side+'_deg']) if good(f) else np.nan for f in feet]
        ax[0].plot(ft,vals,'.-',color=col,label=side,markersize=2)
        targets=[ft[index[int(r['sequence'])]]+n(r['horizon_ms'])/1000 for r in cad if r['reason']=='0']
        preds=[n(r['predicted_'+side+'_deg']) for r in cad if r['reason']=='0']
        ax[0].scatter(targets,preds,color=col,s=8,marker='x',alpha=.5)
    ax[0].legend();ax[0].set_ylabel('Foot angle [deg]\nline: measured; x: prediction')
    ax[1].plot(st,angles,lw=.7,color='purple');ax[1].set_ylabel('Body detector angle [deg]')
    ax[2].plot(st,[n(x['physical_roll_rate_dps']) for x in s],lw=.5,color='tab:orange');ax[2].set_ylabel('Body angular rate [deg/s]')
    for sign,c in [('plus','green'),('minus','brown')]:
        tt=[ft[index[int(x['sequence'])]]+n(x['horizon_ms'])/1000 for x in cad]
        val=[n(x['switch_sign_'+sign+'_deg']) if x['geometry_valid']=='True' else np.nan for x in cad]
        ax[3].plot(tt,val,'.',label='CAD sign '+sign,color=c,markersize=3)
    ax[3].plot(st,angles,color='purple',lw=.7,alpha=.5,label='body');ax[3].legend();ax[3].set_ylabel('Nominal CAD switch [deg]');ax[3].set_xlabel('Measurement time [s]')
    for a in ax:a.set_xlim(0,30);a.grid(alpha=.3)
    fig.suptitle(case+' | CAD roots are uncalibrated hypotheses, not measured contact')
    fig.savefig(out/'timeline.png',dpi=150);plt.close(fig)
    # Cycle summaries use complete body cycles, not ground step lengths.
    neg=[n(x['peak_time_ms'])/1000 for x in peaks if x['physical_peak_side']=='-1']
    cycles=[]
    for a,b in zip(neg,neg[1:]):
        if not .4<b-a<1.5:continue
        f=[feet[i] for i in range(len(feet)) if a<=ft[i]<b]
        vals={side:[n(x[side+'_deg']) for x in f if good(x)] for side in ['left','right']}
        cycles.append({'start_s':a,'end_s':b,'both_valid':sum(good(x) for x in f),'frames':len(f),
          **{side+'_span_deg':max(v)-min(v) if len(v)>=3 else None for side,v in vals.items()}})
    cap=cv2.VideoCapture(videopath);fps=cap.get(cv2.CAP_PROP_FPS);duration=cap.get(cv2.CAP_PROP_FRAME_COUNT)/fps
    optical=[];frame_id=0;last=None
    times=list(range(0,int(duration),3));thumbs=[]
    while True:
        ok,frame=cap.read()
        if not ok:break
        t=frame_id/fps
        if frame_id%90==0:thumbs.append((t,cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
        # Body upper white circular markers: excludes feet and wheel.
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,(0,0,180),(179,95,255));mask[:160]=0;mask[470:]=0;mask[:,:420]=0;mask[:,880:]=0
        nc,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
        pts=[]
        for i in range(1,nc):
            x,y,w,h,area=stats[i]
            if 55<=area<=700 and .65<=w/h<=1.55 and area/(w*h)>.45:pts.append(centroids[i])
        options=[]
        for p in pts:
            for q in pts:
                dx=q[0]-p[0];dy=q[1]-p[1]
                if 150<dx<300 and abs(dy)<75:
                    mid=(p+q)/2;score=abs(dx-210)*.15+abs(mid[0]-650)*.05+abs(mid[1]-290)*.03
                    if last is not None:score+=np.linalg.norm(mid-last)*.5
                    options.append((score,p,q,mid))
        if options:
            _,p,q,mid=min(options,key=lambda a:a[0]);last=mid
            optical.append([t,math.degrees(math.atan2(q[1]-p[1],q[0]-p[0])),float(mid[0]),float(mid[1])])
        frame_id+=1
    cap.release()
    fig,axes=plt.subplots((len(thumbs)+3)//4,4,figsize=(16,3*((len(thumbs)+3)//4)))
    for a in axes.flat:a.axis('off')
    for a,(t,im) in zip(axes.flat,thumbs):a.imshow(im);a.set_title(f'video {t:.2f}s')
    fig.tight_layout();fig.savefig(out/'video_contact.png',dpi=100);plt.close(fig)
    optical=np.array(optical);np.savetxt(out/'video_body_markers.csv',optical,delimiter=',',header='video_s,image_marker_line_deg,mid_x_px,mid_y_px',comments='')
    # Fit only a constant offset and affine angle calibration. Periodic aliases remain possible.
    candidates=[]
    for dt in np.arange(2,13,.01):
        t=optical[:,0]-dt;mask=(t>=-.5)&(t<=30.5)
        x=np.interp(t[mask],st,angles);y=optical[mask,1]
        coef=np.linalg.lstsq(np.c_[x,np.ones(len(x))],y,rcond=None)[0];res=y-coef[0]*x-coef[1]
        candidates.append((float(np.mean(res*res)),float(dt),float(coef[0]),float(coef[1])))
    best=min(candidates);top=sorted(candidates)[:4]
    summary={'build':meta.get('diagnostic_build_id'),'foot_frames':len(feet),'cad_records':len(cad),
             'reason_counts':dict(Counter(x['reason'] for x in cad)),'compute_median_us':statistics.median(n(x['compute_us']) for x in cad),
             'compute_max_us':max(n(x['compute_us']) for x in cad),'prediction_comparison_values':len(errors),
             'prediction_mae_deg':statistics.mean(errors) if errors else None,'prediction_p95_deg':float(np.percentile(errors,95)) if errors else None,
             'prediction_max_deg':max(errors) if errors else None,'hold_last_mae_deg':statistics.mean(hold_errors) if hold_errors else None,
             'peaks':len(peaks),'last_peak_s':n(peaks[-1]['peak_time_ms'])/1000,
             'target_counts':dict(Counter(x['target_peak_deg'] for x in peaks)),
             'output_counts':dict(Counter(x['event_kind'] for x in ev if x['output_executed']=='True')),
             'video_duration_s':duration,'video_markers_frames':len(optical),'video_frames':frame_id,
             'approx_motion_fit':{'mse':best[0],'video_minus_test_s':best[1],'gain':best[2],'bias_deg':best[3],
                                  'scope':'constant-offset optical body angle fit; not LED/exposure/contact synchronization','top':top},
             'cycles':cycles,'prediction_comparison':target_rows}
    (out/'v10_analysis.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf8')
    print(case,json.dumps({k:v for k,v in summary.items() if k not in ['cycles','prediction_comparison']},ensure_ascii=False))
