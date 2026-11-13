"""Approximate white-body-marker alignment and per-input audit; not LED sync."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_v12_trial import rows, num, stats


def compare(directory, video):
    samples = rows(directory, 'timeseries')
    samples = [r for r in samples if 0 < num(r['t_test_ms']) <= 30000]
    t = np.array([num(r['t_test_ms']) / 1000 for r in samples])
    angle = np.array([num(r['pitch_mekf_detector_relative_deg']) for r in samples])
    metadata = json.loads((directory/'metadata.json').read_text(encoding='utf8'))
    assists = metadata['foot_support_assist']['records']
    events = rows(directory, 'energy_control_autonomous_zero_cross_events')
    peaks = rows(directory, 'energy_control_autonomous_peak_events')
    feet = rows(directory, 'foot_angles')
    audit = []
    nz = np.flatnonzero(angle != 0)
    transitions = [(a,b) for a,b in zip(nz[:-1],nz[1:]) if angle[a]*angle[b] < 0]
    for support in assists:
        ms = support['time_ms']
        matches = [r for r in events if r['work_control_branch']=='17'
                   and abs(num(r['pulse_start_ms'])-ms) <= 10 and r['output_executed']=='True']
        matched = matches[0] if len(matches)==1 else None
        next_peaks = [r for r in peaks if num(r['peak_time_ms'])>ms]
        same_peak_others = [r for r in events if r['output_executed']=='True'
                           and num(r['previous_peak_time_ms'])==support['peak_ms'] and r is not matched]
        crossing = next(((a,b) for a,b in transitions if t[b]*1000>ms), None)
        upcoming = next_peaks[0] if next_peaks else None
        within = (t>=ms/1000) & (t<=min(30,ms/1000+.5))
        measurement = [r for r in samples if ms<=num(r['t_test_ms'])<=ms+100]
        audit.append(dict(support,
            matched_branch17=matched is not None,
            matched_pulse_start_ms=num(matched['pulse_start_ms']) if matched else None,
            matched_width_ms=num(matched['pulse_width_ms']) if matched else None,
            next_sampled_sign_cross_bracket_s=[float(t[crossing[0]]),float(t[crossing[1]])] if crossing else None,
            next_peak={k:upcoming[k] for k in ['peak_time_ms','physical_peak_side','peak_amplitude_deg']} if upcoming else None,
            other_executed_same_peak=[{'event_kind':r['event_kind'],'pulse_start_ms':r['pulse_start_ms']} for r in same_peak_others],
            observed_current_100ms=stats([num(r['roller_actual_current_mA']) for r in measurement]),
            following_500ms_angle_deg=stats(angle[within])))

    cap=cv2.VideoCapture(str(video)); fps=cap.get(cv2.CAP_PROP_FPS)
    optical=[]; frame_id=0; previous=np.array([645.,275.])
    while True:
        ok, im=cap.read()
        if not ok: break
        mask=cv2.inRange(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),(0,0,180),(179,100,255))
        mask[:195]=0;mask[540:]=0;mask[:,:430]=0;mask[:,1000:]=0
        count, labels, boxes, centers=cv2.connectedComponentsWithStats(mask)
        points=[]
        for j in range(1,count):
            x,y,w,h,area=boxes[j]
            if 90<=area<=1600 and .65<=w/h<=1.5 and area/(w*h)>.53:
                points.append(centers[j])
        choices=[]
        for a in points:
            for b in points:
                dx,dy=b-a
                if 155<dx<350 and abs(dy)<120:
                    mid=(a+b)/2
                    score=np.linalg.norm(mid-previous)+.04*abs(dx-220)
                    choices.append((score,a,b,mid))
        if choices:
            score,a,b,mid=min(choices,key=lambda x:x[0])
            if score<75:
                previous=mid
                optical.append([frame_id/fps,math.degrees(math.atan2(b[1]-a[1],b[0]-a[0])),*mid,b[0]-a[0]])
        frame_id+=1
    cap.release(); optical=np.array(optical)
    np.savetxt(directory/'video_body_markers.csv',optical,delimiter=',',
               header='video_s,image_marker_line_deg,mid_x_px,mid_y_px,horizontal_marker_spacing_px',comments='')
    fits=[]
    for offset in np.arange(2,13,.005):
        vt=optical[:,0]-offset
        mask=(vt>=.15)&(vt<=27.5)
        x=np.interp(vt[mask],t,angle); y=optical[mask,1]
        coef=np.linalg.lstsq(np.c_[x,np.ones(len(x)),vt[mask]],y,rcond=None)[0]
        if not .4<abs(coef[0])<1.8:continue
        err=y-coef[0]*x-coef[1]-coef[2]*vt[mask]
        fits.append((float(np.mean(err**2)),float(offset),*map(float,coef)))
    mse,offset,gain,bias,drift=min(fits)
    alignment={'method':'white-body-marker waveform fit; affine gain/bias/slow drift; not LED/exposure/contact sync',
        'measurement_to_video_offset_s':offset,'fit_mse_deg2':mse,'gain':gain,'bias':bias,'drift_dps':drift,
        'alternative_fits_separated_300ms':[x for x in sorted(fits) if abs(x[1]-offset)>.3][:3]}
    fig,axes=plt.subplots(3,1,figsize=(13,9),constrained_layout=True)
    for ax,(lo,hi) in zip(axes,[(0,30),(22.7,24.5),(27.8,30)]):
        vt=optical[:,0]-offset
        ax.plot(t,angle,label='IMU detector',lw=1)
        ax.plot(vt,(optical[:,1]-bias-drift*vt)/gain,label='white body markers, affine/drift fit',lw=1)
        for r in assists:
            if r['output_executed']:ax.axvline(r['time_ms']/1000,color='red',alpha=.5)
        ax.set_xlim(lo,hi);ax.set_ylabel('Body [deg]');ax.grid(alpha=.3);ax.legend()
    axes[-1].set_xlabel('Approximate measurement time [s]')
    fig.savefig(directory/'motion_alignment.png',dpi=140);plt.close(fig)
    cap=cv2.VideoCapture(str(video))
    for name,start,step,count in [('assist_9561',9.3,.1,12),('assist_23661',23.3,.1,16),
                                   ('assist_28858',28.45,.1,20),('end',29.3,.15,12)]:
        fig,axs=plt.subplots((count+3)//4,4,figsize=(12,2.5*((count+3)//4)))
        for i,ax in enumerate(axs.flat):
            ax.axis('off')
            if i>=count:continue
            tm=start+i*step;vtime=tm+offset
            cap.set(cv2.CAP_PROP_POS_MSEC,vtime*1000);ok,im=cap.read()
            if ok:ax.imshow(cv2.cvtColor(im[170:720,450:1030],cv2.COLOR_BGR2RGB))
            ax.set_title(f'video {vtime:.2f}s / test ~{tm:.2f}s',fontsize=9)
        fig.suptitle('Approximate body-waveform alignment; no synchronized contact measurement')
        fig.tight_layout(rect=(0,0,1,.97));fig.savefig(directory/('video_'+name+'.png'),dpi=125);plt.close(fig)
    cap.release()
    result={'video_sha256':hashlib.sha256(video.read_bytes()).hexdigest(),
            'fps':fps,'frames':frame_id,'tracked_marker_frames':len(optical),
            'alignment':alignment,'support_audits':audit}
    (directory/'video_audit.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('converted',type=Path);p.add_argument('video',type=Path)
    a=p.parse_args();compare(a.converted,a.video)
