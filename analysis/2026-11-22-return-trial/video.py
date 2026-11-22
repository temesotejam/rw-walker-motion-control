"""Measure the two front-view clips against their visible mat and MEKF time series."""
import argparse
from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path

import cv2
import numpy as np

here = Path(__file__).resolve().parent
spec = spec_from_file_location('optical_marks', here.parent / '2026-11-20-turn1-trial/video.py')
optical = module_from_spec(spec)
spec.loader.exec_module(optical)
clips = {70067320: 'WIN_20261122_14_40_33_Pro.mp4', 195727101: 'WIN_20261122_14_42_40_Pro.mp4'}
seeds = ((564, 235), (779, 232), (646, 375), (690, 375))


def floor(xy):
    # Horizontal gridline centres at x=465 and x=800 in the first frame.
    x, y = xy
    left = np.array([374, 383, 393, 403, 414, 425, 437, 449])
    right = np.array([373, 383, 392, 403, 413, 425, 436, 448])
    lines = left + (right - left) * (x - 465) / 335
    depth = optical.interp(y, lines, np.arange(len(lines)))
    # Fourteen full small cells between two lines, calibrated at three depths.
    lo = optical.interp(y, [376, 409, 444], [510, 503, 494])
    hi = optical.interp(y, [376, 409, 444], [803, 816, 829])
    return [14 * (x-lo)/(hi-lo), depth]


def registration(videos):
    images = []
    for clip in clips.values():
        capture = cv2.VideoCapture(str(videos / clip))
        ok, frame = capture.read()
        capture.release()
        if not ok:
            raise OSError(clip)
        images.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    offsets, scores = [], []
    for x,y in [(470,390), (845,390), (475,450), (880,450), (500,516), (830,520), (540,565), (790,565)]:
        template = images[0][y-15:y+16, x-15:x+16]
        search = images[1][y-21:y+22, x-21:x+22]
        _, score, _, offset = cv2.minMaxLoc(cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED))
        offsets.append(np.array(offset)-6)
        scores.append(score)
    shift = np.median(offsets, axis=0)
    if np.abs(offsets-shift).max() > 1 or min(scores) < .8:
        raise ValueError('Mat does not align between clips')
    return shift, min(scores)


def aligned_observations(video, folder, shift, markers=seeds, floor_map=floor, through_s=30.):
    stamps, marks = optical.tracks(video, markers, end_s=45)
    marker_axis = marks[:,1] - marks[:,0]
    angle = np.degrees(np.arctan2(marker_axis[:,1], marker_axis[:,0]))
    columns = ('t_test_ms','state_id','pitch_mekf_detector_relative_deg')
    imu = np.genfromtxt(folder / 'timeseries.csv', delimiter=',', names=True, usecols=columns)
    imu = imu[imu['state_id']==3]
    fits = []
    for start, end in [(0., 4.), (.5,4.), (0.,6.)]:
        times = imu['t_test_ms'] * .001
        keep = (start <= times) & (times < end)
        origins = np.arange(3., 12., .001)
        corr = [np.corrcoef(np.interp(times[keep]+o, stamps, angle), imu[columns[2]][keep])[0,1] for o in origins]
        best = np.nanargmin(corr)
        fits.append([round(origins[best],3), round(corr[best],4)])
    t = stamps - fits[0][0]
    selected = (t>=-.8) & (t<through_s+.1)
    if not np.isfinite(marks[selected]).all() or t[-1]<through_s:
        raise ValueError('Incomplete marker coverage')
    feet = np.array([[floor_map(mark-shift) for mark in pair] for pair in marks[:,2:]])
    return t, feet, marks, fits


def measure_positions(folder, t, feet, marks, fits, axis=None, through_s=30.):
    def pos(at, window):
        return feet[np.abs(t-at)<window].mean(axis=0)

    base = feet[(t>=-.8) & (t<0)].mean(axis=0)
    final = np.median(feet[(t>=12) & (t<=through_s)], axis=0)
    intervals = []
    for a,b in [(0,4),(4,8),(8,12),(0,12),(12,through_s-.5)]:
        d = pos(b,.4) - (base if a==0 else pos(a,.4))
        intervals.append(dict(start_s=a,end_s=b,right_forward=d[0,1],left_forward=d[1,1],
                              mean_forward=d[:,1].mean(),mean_lateral=d[:,0].mean()))
    settle = {}
    for tol in [.05,.10,.20]:
        outside = np.flatnonzero((t>=0) & (t<=through_s) & (np.max(np.abs(feet[:,:,1]-final[:,1]),axis=1)>tol))
        next_frame = outside[-1]+1 if len(outside) else None
        settle[tol] = float(t[next_frame]) if next_frame is not None and next_frame<len(t) and t[next_frame]<=through_s else None
    meta = json.loads((folder/'metadata.json').read_bytes())
    peaks = meta['energy_control_autonomous_peak_events']
    swings = []
    for e in meta['energy_control_autonomous_zero_cross_events']:
        if e['event_kind'] != 'energy_control_zero_cross':
            continue
        time = e['zero_cross_time_ms']
        p = next((p for p in peaks if p['peak_time_ms']>time), None)
        side = e['physical_next_peak_side']
        if not p or p['physical_peak_side']!=side:
            continue
        row = dict(zero_ms=time, peak_ms=p['peak_time_ms'], expected='left' if side==1 else 'right')
        for width in [.05,.10]:
            d = pos(p['peak_time_ms']*.001+.025,width)-pos(time*.001,width)
            row[str(width)] = dict(right=float(d[0,1]),left=float(d[1,1]))
        swings.append(row)
    between_peaks = []
    for previous, current in zip(peaks, peaks[1:]):
        row = dict(from_ms=previous['peak_time_ms'], to_ms=current['peak_time_ms'],
                   expected='left' if current['physical_peak_side']==1 else 'right')
        for width in [.05,.10]:
            d = pos(row['to_ms']*.001+.025,width)-pos(row['from_ms']*.001+.025,width)
            row[str(width)] = dict(right=float(d[0,1]),left=float(d[1,1]))
        between_peaks.append(row)
    if axis is not None:
        visible = (t >= -.1) & (t <= 12.05)
        for i,(label,color) in enumerate([('機体の右足','#17698a'),('機体の左足','#bc612a')]):
            axis.plot(t[visible], feet[visible,i,1]-base[i,1], label=label, color=color, lw=1.7)
        axis.axvline(settle[.10], color='#535963', linestyle='--', lw=1)
        axis.set(xlim=(0,12),ylim=(-.2,3), ylabel='手前への変位（小区画）')
        axis.set_title(f"{folder.name}    {meta.get('walking_profile', '')}", loc='left',fontsize=11)
        axis.grid(axis='y', alpha=.2)
    at_rest = marks[(12<=t)&(t<=through_s),2:]
    return dict(video_zero_and_correlation=fits, intervals=intervals,settled_forward_s=settle,
                stationary_frames=len(at_rest),stationary_xy_span_px=np.ptp(at_rest,axis=0).tolist(),
                swings=swings,between_peaks=between_peaks)


def measure(video, folder, shift, axis=None):
    return measure_positions(folder, *aligned_observations(video, folder, shift), axis=axis)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_root',type=Path)
    parser.add_argument('--videos',type=Path,default=Path.home()/'Pictures/Camera Roll')
    parser.add_argument('--plot',type=Path,help='Save both feet trajectories as a PNG')
    args=parser.parse_args()
    shift,score=registration(args.videos)
    results=dict(registration_px=shift.tolist(), minimum_grid_correlation=score)
    axes=[None,None]
    if args.plot:
        import matplotlib.pyplot as plt
        plt.rcParams['font.family']=['Meiryo','DejaVu Sans']
        fig,axes=plt.subplots(2,1,figsize=(8.5,5.9),sharex=True,layout='constrained')
        fig.suptitle('動画の足先白印を床格子に投影した前進量', fontsize=15)
    for i,(run,clip) in enumerate(clips.items()):
        results[run]=measure(args.videos/clip,args.converted_root/str(run),shift if i else np.zeros(2),axes[i])
    if args.plot:
        axes[0].legend(loc='upper left',ncol=2)
        axes[1].set_xlabel('測定開始後の時間（秒）　　破線は両足が終盤の位置の±0.1小区画に収まった時刻')
        fig.savefig(args.plot,dpi=140)
    print(json.dumps(results, ensure_ascii=True, allow_nan=False,indent=2))
