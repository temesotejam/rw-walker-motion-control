"""Plot the final leftward return from run 135284989 using acquisition and availability times."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
from receiving_carry import CarryReplay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted', type=Path)
    parser.add_argument('output_png', type=Path)
    args = parser.parse_args()
    meta = json.loads((args.converted / 'metadata.json').read_bytes())
    header = json.loads((args.converted / 'header.json').read_bytes())
    if header['run_start_us'] != 135284989:
        raise ValueError('Expected the 15:39 trial 135284989')
    aid = [r for r in meta['walking_energy']['records'] if r['side'] == -1 and r['return_event'] and r['executed']][-1]
    peak_ms = aid['peak_ms']
    replay = CarryReplay(args.converted)
    motion = replay.follow(dict(peak_ms=peak_ms, decision_ms=aid['output_end_ms'], terminal_deg=0.))
    if motion['status'] != 'reversed':
        raise ValueError('Expected a measured reversal before the signed mechanical zero')
    reverse = motion['end_ms'] / 1000
    start, end = peak_ms / 1000 - .02, reverse + .035
    gyro_time, rate, body = (np.array(a) for a in replay.gyro)
    gyro_time = gyro_time / 1000
    visible = (start <= gyro_time) & (gyro_time <= end)
    plt.rcParams['font.family'] = ['Meiryo', 'DejaVu Sans']
    fig, axes = plt.subplots(4, 1, figsize=(8.8, 7.7), sharex=True, layout='constrained')
    fig.suptitle('右ピークから左へ戻る途中で何が起きたか', weight='bold', fontsize=15)
    for ax in axes:
        ax.axvspan(aid['output_start_ms']/1000, aid['output_end_ms']/1000, color='#edbd62', alpha=.45)
        ax.axvline(reverse, color='#a94a40', ls='--', lw=1.1)
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.2)
        ax.set_xlim(start, end)
    axes[0].plot(gyro_time[visible], body[visible], color='#235c85')
    axes[0].set_ylabel('胴体の左右角（°）')
    axes[0].axhline(0, color='#555555', lw=.8)
    axes[0].text(aid['output_start_ms']/1000-.004, 7.4, '左補助 15 ms', ha='right', color='#785216')
    axes[0].annotate('右側 +0.57°で折返し', (reverse,.57), xytext=(reverse-.105,2.7),
                     arrowprops=dict(arrowstyle='->', color='#a94a40'), color='#8a352d')
    axes[1].plot(gyro_time[visible], -rate[visible], color='#235c85')
    axes[1].set_ylabel('左向き速度（°/s）')
    times, diffs = [], []
    for f in meta['foot_frames']:
        if f['state_id'] != 3 or not f['control_observed_us']:
            continue
        t = (f['control_observed_us'] - replay.clock) / 1.e6
        if not start-.06 <= t <= end:
            continue
        valid = all(f[k] for k in ('frame_valid', 'timestamp_valid', 'zero_ready', 'left_valid',
                    'right_valid', 'left_control_range_ok', 'right_control_range_ok'))
        times.append(t)
        diffs.append(f['left_deg'] - f['right_deg'] if valid else np.nan)
    axes[2].step(times, diffs, where='post', color='#367759')
    axes[2].axhline(3, color='#805049', linestyle=':', lw=1.1)
    axes[2].set_ylabel('左足角 − 右足角（°）')
    axes[2].text(start+.006, 3.02, '判定境界 3°', color='#805049')
    anchors = [(ready/1000, 1000*float(row['coast_margin_j'])) for _,ready,row in replay.by_peak[peak_ms]
               if start<=ready/1000<=end and int(row['anchor']) and row['coast_margin_j']]
    t, margin = zip(*anchors)
    axes[3].bar(t, margin, width=.006, color=['#397859' if v>=0 else '#985446' for v in margin])
    axes[3].axhline(0, color='#555555', lw=.8)
    axes[3].set_ylabel('惰行の予測余裕（mJ）')
    axes[3].set_xlabel('測定開始からの時間（秒）')
    axes[3].set_title('左の円弧端が定まった後の機上予測（計算結果が使える時刻）', loc='left', fontsize=10)
    fig.savefig(args.output_png, dpi=145)
    plt.close(fig)


if __name__ == '__main__':
    main()
