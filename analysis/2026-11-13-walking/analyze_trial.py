"""v11 log/video comparison. Motion alignment is approximate, not LED sync."""
from pathlib import Path
import csv,hashlib,json,math
from collections import Counter
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'141282685'
VIDEO=Path(r'C:\Users\arika\Pictures\Camera Roll\WIN_20261113_11_32_46_Pro.mp4')
LOG=Path(r'C:\Users\arika\Downloads\energy_control_autonomous_run_2_141282685.rwlog')
def read(name):
    with (OUT/name).open(encoding='utf8',newline='') as f:return list(csv.DictReader(f))
def n(x):
    try:return float(x)
    except (TypeError,ValueError):return math.nan
def good(f):
    return all(f[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range'])

s=read('timeseries.csv');feet=read('foot_angles.csv');pose=read('foot_pose_prediction.csv')
peaks=read('energy_control_autonomous_peak_events.csv');events=read('energy_control_autonomous_zero_cross_events.csv')
assist=read('foot_support_assist.csv')
offset=float(np.median([n(r['log_time_s'])-n(r['t_test_ms'])/1000 for r in s if .1<n(r['t_test_ms'])/1000<29.9]))
t=np.array([n(r['log_time_s'])-offset for r in s]);angle=np.array([n(r['pitch_mekf_detector_relative_deg']) for r in s])
ft=np.array([n(r['measurement_time_us'])/1e6 for r in feet]);ix={int(f['sequence']):i for i,f in enumerate(feet)}
fig,axes=plt.subplots(4,1,figsize=(15,12),sharex=True,constrained_layout=True)
for side,col in [('left','tab:blue'),('right','tab:red')]:
    axes[0].plot(ft,[n(f[side+'_deg']) if good(f) else np.nan for f in feet],'.-',color=col,label=side,ms=2)
    axes[0].scatter([(n(r['ready_ms'])+30)/1000 for r in pose if r['reason']=='0'],[n(r['predicted_'+side+'_deg']) for r in pose if r['reason']=='0'],marker='x',s=8,color=col,alpha=.4)
axes[0].set_ylabel('Foot angle [deg]\nline measured, x forecast +30ms');axes[0].legend()
axes[1].plot(t,angle,lw=.8,color='purple');axes[1].set_ylabel('Body detector angle [deg]')
axes[2].plot(t,[n(r['physical_roll_rate_dps']) for r in s],lw=.6,color='tab:orange');axes[2].set_ylabel('Body rate [deg/s]')
for e in events:
    if e['output_executed']=='True':
        tm=n(e['pulse_start_ms'])/1000;w=n(e['pulse_width_ms'])*n(e['physical_next_peak_side'])
        col='red' if e['work_control_branch']=='16' else 'green' if e['event_kind']=='energy_control_zero_cross' else 'gray'
        axes[3].vlines(tm,0,w,color=col,lw=2 if col=='red' else 1)
axes[3].set_ylabel('Executed width x side [ms]\nred: new support assist');axes[3].set_xlabel('Measurement time [s]')
for ax in axes:
    ax.set_xlim(0,30);ax.grid(alpha=.3)
    for a in assist:ax.axvline(n(a['time_ms'])/1000,color='red',alpha=.25,lw=.8)
fig.savefig(OUT/'timeline.png',dpi=150);plt.close(fig)

cap=cv2.VideoCapture(str(VIDEO));fps=cap.get(cv2.CAP_PROP_FPS);optical=[];last=None;frame_id=0
while True:
    ok,frame=cap.read()
    if not ok:break
    mask=cv2.inRange(cv2.cvtColor(frame,cv2.COLOR_BGR2HSV),(0,0,180),(179,95,255))
    mask[:180]=0;mask[520:]=0;mask[:,:420]=0;mask[:,880:]=0
    nc,labels,stats,centroids=cv2.connectedComponentsWithStats(mask);points=[]
    for j in range(1,nc):
        x,y,w,h,a=stats[j]
        if 55<=a<=900 and .65<=w/h<=1.55 and a/(w*h)>.45:points.append(centroids[j])
    opts=[]
    for p in points:
        for q in points:
            dx=q[0]-p[0];dy=q[1]-p[1]
            if 150<dx<320 and abs(dy)<85:
                mid=(p+q)/2;score=abs(dx-210)*.15+abs(mid[0]-650)*.05+abs(mid[1]-290)*.03
                if last is not None:score+=np.linalg.norm(mid-last)*.5
                opts.append((score,p,q,mid))
    if opts:
        _,p,q,mid=min(opts,key=lambda x:x[0]);last=mid
        optical.append([frame_id/fps,math.degrees(math.atan2(q[1]-p[1],q[0]-p[0])),mid[0],mid[1]])
    frame_id+=1
cap.release();optical=np.array(optical)
np.savetxt(OUT/'video_body_markers.csv',optical,delimiter=',',header='video_s,image_marker_line_deg,mid_x_px,mid_y_px',comments='')
candidates=[]
for dt in np.arange(5,15,.01):
    vt=optical[:,0]-dt;mask=(vt>=-1)&(vt<=32)
    # A slow image/IMU zero difference is a nuisance fit, never a contact cue.
    x=np.interp(vt[mask],t,angle);y=optical[mask,1]
    coef=np.linalg.lstsq(np.c_[x,np.ones(len(x)),vt[mask]],y,rcond=None)[0]
    resid=y-coef[0]*x-coef[1]-coef[2]*vt[mask]
    candidates.append((float(np.mean(resid*resid)),float(dt),*map(float,coef)))
best=min(candidates);mse,dt,gain,bias,drift=best
aliases=[c for c in sorted(candidates) if abs(c[1]-dt)>.3][:5]
fig,axes=plt.subplots(3,1,figsize=(14,9),constrained_layout=True)
for ax,(lo,hi) in zip(axes,[(0,32),(6.8,11.5),(26,30)]):
    vt=optical[:,0]-dt
    ax.plot(t,angle,lw=1,label='IMU detector')
    ax.plot(vt,(optical[:,1]-bias-drift*vt)/gain,lw=1,label='white marker line, affine/drift fit',alpha=.75)
    ax.set_xlim(lo,hi);ax.set_ylim(-11,9);ax.grid(alpha=.3);ax.legend();ax.set_ylabel('Body [deg]')
axes[-1].set_xlabel('Approximate measurement time [s]')
fig.savefig(OUT/'motion_alignment.png',dpi=140);plt.close(fig)
cap=cv2.VideoCapture(str(VIDEO))
for name,start,count,step in [('assist_7429',6.95,24,.075),('assist_10652',10.15,24,.075),('stop_27622',27.0,32,.1)]:
    fig,axes=plt.subplots((count+3)//4,4,figsize=(12,3*((count+3)//4)))
    for k,ax in enumerate(axes.flat):
        ax.axis('off')
        if k>=count:continue
        test=start+k*step;sec=test+dt;cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,im=cap.read()
        if ok:ax.imshow(cv2.cvtColor(im[230:690,470:850],cv2.COLOR_BGR2RGB))
        ax.set_title(f'video {sec:.2f}s / approx test {test:.2f}s',fontsize=9)
    fig.suptitle('Body-motion alignment only; no LED/contact synchronization')
    fig.tight_layout(rect=(0,0,1,.97));fig.savefig(OUT/('video_'+name+'.png'),dpi=110);plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(12,8))
for ax,test in zip(axes.flat,[27.45,27.65,27.85,28.05,28.5,29.1]):
    sec=test+dt;cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,im=cap.read();ax.axis('off')
    if ok:ax.imshow(cv2.cvtColor(im[230:690,470:850],cv2.COLOR_BGR2RGB))
    ax.set_title(f'video {sec:.2f}s / approx test {test:.2f}s')
fig.suptitle('End: foot posture remains asymmetric while rocking shrinks')
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'stop_selected_frames.png',dpi=140);plt.close(fig)
cap.release()
# Detailed end-of-trial variables reveal detector centre versus foot posture.
fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True,constrained_layout=True)
axes[0].plot(t,angle,color='purple');axes[0].axhline(0,color='black',lw=.5)
for side,col in [('left','tab:blue'),('right','tab:red')]:
    axes[1].plot(ft,[n(f[side+'_deg']) if f[side+'_valid']=='True' and f[side+'_in_range']=='True' else np.nan for f in feet],'.-',color=col,label=side)
    axes[2].plot([ft[ix[int(r['sequence'])]] for r in pose],[n(r['confidence']) for r in pose],'.-',color='green')
axes[1].legend();axes[0].set_ylabel('Body [deg]');axes[1].set_ylabel('Camera foot [deg]');axes[2].set_ylabel('History score')
axes[3].plot(t,[n(r['physical_roll_rate_dps']) for r in s],color='tab:orange',lw=.6);axes[3].set_ylabel('Body rate [deg/s]');axes[3].set_xlabel('Measurement time [s]')
for ax in axes:ax.set_xlim(26,30);ax.grid(alpha=.3)
fig.savefig(OUT/'stop_detail.png',dpi=150);plt.close(fig)
summary={'video_sha256':hashlib.sha256(VIDEO.read_bytes()).hexdigest(),'rwlog_sha256':hashlib.sha256(LOG.read_bytes()).hexdigest(),
    'video_fps':fps,'video_frames':frame_id,'tracked_marker_frames':len(optical),'log_to_measurement_s':offset,
    'approx_video_minus_measurement_s':dt,'motion_fit_mse_deg2':mse,'image_angle_gain':gain,'image_angle_bias':bias,'image_angle_drift_dps':drift,
    'alternate_offset_fits':aliases,'scope':'motion-only approximate alignment; not LED/exposure/contact synchronization'}
(OUT/'video_analysis.json').write_text(json.dumps(summary,indent=2),encoding='utf8')
print(json.dumps(summary))
