"""Compare the 15:39 run against the mat and its own MEKF clock."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from video import aligned_observations, measure_positions, optical


MARKERS = ((541, 272), (752, 267), (625, 409), (668, 407))
ROW_LEFT = np.array([406, 415, 425, 435, 446, 457, 469, 481, 493])
ROW_RIGHT = np.array([405, 415, 424, 434, 445, 456, 467, 479, 492])


def mat_position(point):
    # Full bright grid lines surrounding the feet in this clip's first frame.
    # Their row centres were read at x=475 and x=800. Two vertical lines
    # spanning fourteen small cells were read at each of three image depths.
    x, y = point
    rows = ROW_LEFT + (ROW_RIGHT - ROW_LEFT) * (x-475) / 325
    left = optical.interp(y, [420, 443, 480], [508, 503, 494])
    right = optical.interp(y, [420, 443, 480], [805, 814, 829])
    return [14*(x-left)/(right-left), optical.interp(y, rows, np.arange(len(rows)))]


def check_mat(video):
    cap = cv2.VideoCapture(str(video))
    ok, frame = cap.read()
    if not ok:
        raise OSError(video)
    ref = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    offsets, scores = [], []
    for second in range(2, 40, 2):
        cap.set(cv2.CAP_PROP_POS_MSEC, 1000*second)
        ok, frame = cap.read()
        if not ok:
            raise OSError(f'No frame at {second} s')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for x,y in [(475,445), (827,445), (475,495), (827,495), (520,565), (790,565)]:
            patch = ref[y-15:y+16, x-15:x+16]
            search = gray[y-19:y+20, x-19:x+20]
            _, score, _, delta = cv2.minMaxLoc(cv2.matchTemplate(search, patch, cv2.TM_CCOEFF_NORMED))
            offsets.append(np.array(delta)-4)
            scores.append(score)
    cap.release()
    drift = int(np.max(np.abs(offsets)))
    if drift > 1 or min(scores) < .8:
        raise ValueError('Check camera and mat position before using the first-frame calibration')
    return dict(regions_compared=len(scores), max_shift_px=drift, minimum_correlation=min(scores))


def plot(path, time, travel, aids, last_peak, settled):
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Meiryo', 'DejaVu Sans']
    fig, axes = plt.subplots(2, 1, figsize=(8.3, 6.4), layout='constrained')
    fig.suptitle('11月22日 15時39分　足先はどこまで前進したか')
    for ax,limits in zip(axes, [(0,12), (4.2,6.3)]):
        visible = (time >= limits[0]-.05) & (time <= limits[1]+.05)
        for i,(label,color) in enumerate([('右足先','#17698a'),('左足先','#bc612a')]):
            ax.plot(time[visible], travel[visible,i], color=color, label=label, lw=1.8)
        for i,aid in enumerate(aids):
            ax.axvspan(aid['output_start_ms']/1000, aid['output_end_ms']/1000, color='#e9ba47',
                       alpha=.6, label='左への補助通電' if i==0 else None)
        ax.axvline(last_peak, color='#5c485a', ls=':', lw=1.5, label='最後の右ピーク')
        ax.axvline(settled, color='#656765', ls='--', lw=1, label='終盤位置の±0.1区画に収束')
        ax.set(xlim=limits, ylabel='床格子の手前方向（小区画）')
        ax.grid(axis='y',alpha=.2)
    axes[0].legend(loc='lower right',ncol=2,fontsize=8)
    axes[1].set(xlabel='測定開始後の時間（秒）', ylim=(1.7,2.8), title='最後の右ピークと左への補助通電の前後')
    fig.savefig(path,dpi=145)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_root',type=Path)
    parser.add_argument('--video',type=Path, default=Path.home()/'Pictures/Camera Roll/WIN_20261122_15_39_17_Pro.mp4')
    parser.add_argument('--plot',type=Path)
    args = parser.parse_args()
    folder = args.converted_root/'135284989'
    data = json.loads((folder/'metadata.json').read_bytes())
    alignment = check_mat(args.video)
    time, feet, marks, fits = aligned_observations(args.video, folder, np.zeros(2), MARKERS, mat_position)
    result = measure_positions(folder, time, feet, marks, fits)
    forward = feet[:,:,1]
    travel = forward - np.mean(forward[(time>=-.8) & (time<0)],axis=0)
    rest = np.median(travel[(time>=12) & (time<=30)], axis=0)
    aids = [a for a in data['walking_energy']['records'] if a['return_event'] and a['executed'] and a['side']==-1]
    last_peak = data['energy_control_autonomous_peak_events'][-1]['peak_time_ms']/1000
    times = [a['output_start_ms']/1000 for a in aids] + [last_peak]
    result['forward_remaining_to_rest'] = [
        dict(time_s=t, right_left_cells=(rest-travel[np.abs(time-t)<.05].mean(axis=0)).tolist())
        for t in sorted(times)]
    result['mat'] = alignment
    if args.plot:
        plot(args.plot, time, travel, aids, last_peak, result['settled_forward_s'][.10])
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__=='__main__':
    main()
