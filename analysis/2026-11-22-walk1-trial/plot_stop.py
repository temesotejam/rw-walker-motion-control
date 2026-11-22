"""Plot body motion in the current and preceding trials, and actual final aid in walk1."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

ANALYSIS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ANALYSIS / '2026-11-22-walking-simulator'))
sys.path.insert(0, str(ANALYSIS / '2026-11-22-return-trial'))
from aid_current_delivery import evaluate_trial
from pulse_release import trace
from receiving_carry import CarryReplay


def return_at_angles(gyro, start_ms, end_ms, targets):
    """Interpolate leftward speed at the first inward crossing of each body angle."""
    times, rates, angles = gyro
    speeds = {}
    for i in range(1, len(times)):
        if times[i] > end_ms:
            break
        if times[i-1] < start_ms or times[i] - times[i-1] > 10:
            continue
        hi, lo = angles[i-1:i+1]
        if hi is None or lo is None:
            continue
        for angle in targets:
            if angle not in speeds and hi > angle >= lo:
                fraction = (hi - angle) / (hi - lo)
                speeds[angle] = -(rates[i-1] + fraction * (rates[i] - rates[i-1]))
    return speeds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('current', type=Path)
    parser.add_argument('previous', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    for folder, expected in ((args.current, 68907120), (args.previous, 135284989)):
        if json.loads((folder / 'header.json').read_bytes())['run_start_us'] != expected:
            parser.error(f'Expected converted recording {expected} in {folder}')
    current = CarryReplay(args.current)
    previous = CarryReplay(args.previous)
    last_aid = evaluate_trial(args.current)[-1]
    assert last_aid['status'] == 'reversed'
    on, off, reverse = (last_aid[key] / 1000 for key in ('driver_on_ms', 'driver_off_ms', 'end_ms'))
    receiving = last_aid['receiving_state_ms'] / 1000
    _, traces = trace(args.current)
    read_time, actual_current = np.array(traces['current']).T

    plt.rcParams['font.family'] = ['Meiryo', 'DejaVu Sans']
    fig, axes = plt.subplots(4, 1, figsize=(8.9, 10.2), layout='constrained',
                             gridspec_kw={'height_ratios': [1.3, 1, .9, 1.1]})
    blue, grey, red = '#20598a', '#73777c', '#a14438'
    fig.suptitle('左へ戻る途中で中心を越えられなくなった実走', fontsize=16, weight='bold')
    for run, color, label, style in ((previous, grey, '前回 15:39', '--'), (current, blue, '今回 16:56', '-')):
        run_times, _, run_angles = run.gyro
        axes[0].plot(np.asarray(run_times) / 1000, run_angles, color=color, lw=1.55, ls=style, label=label)
    axes[0].set(xlim=(0, 8), ylabel='胴体の左右角 (°)', xlabel='測定開始から (秒)')
    axes[0].axhline(0, color='black', lw=.8)
    axes[0].legend(loc='lower right', frameon=False, ncol=2)
    axes[0].axvspan(on-.19, reverse+.06, color='#e3ebf1', zorder=-1)
    times, rates, angles = (np.array(s, dtype=float) for s in current.gyro)
    reversal_angle = np.interp(reverse, times / 1000, angles)
    axes[0].scatter(reverse, reversal_angle, color=red, s=28, zorder=5)
    axes[0].annotate(f'今回の折返し\n右側 +{reversal_angle:.2f}°', (reverse, reversal_angle),
                     xytext=(4.35, -4.1), color=red, arrowprops={'arrowstyle': '->', 'color': red},
                     bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .92, 'pad': 1})
    axes[1].plot(times / 1000, -rates, color=blue)
    axes[1].set_ylabel('左向きの胴体速度 (°/s)')
    axes[1].axhline(0, color='#444444', lw=.8)
    axes[2].plot(read_time / 1000, -actual_current, '.-', color='#614885', markersize=4,
                 label='ドライバの実電流')
    axes[2].set(xlabel='測定開始から (秒、今回の最後の左補助を拡大)', ylabel='左指令方向の実電流 (mA)')
    for ax in axes[1:3]:
        ax.axvspan(on, off, color='#e2ae54', alpha=.45, label='通電指令')
        ax.axvline(receiving, color='#408679', ls=':', lw=1.7, label='左足裏が低くなった最初のCAD標本')
        ax.axvline(reverse, color=red, ls='--', lw=1.4, label='中心手前の折返し')
        ax.set_xlim(2.865, 3.345)
    visible = (times > 2865) & (times < 3345)
    axes[1].set_ylim(np.nanmin(-rates[visible]) - 3, np.nanmax(-rates[visible]) + 6)
    visible = (read_time > 2865) & (read_time < 3345)
    axes[2].set_ylim(np.min(-actual_current[visible]) - 9, np.max(-actual_current[visible]) + 15)
    axes[1].legend(facecolor='white', edgecolor='none', framealpha=.95, loc='upper right', fontsize=8)
    targets = (4.76, 4., 3.5, 3., 2.5, 2.)
    right_peaks = [peak for _, peak, side in current.confirmations if side == 1]
    for peak, color, label in ((right_peaks[-2], grey, 'ひとつ前・中心を通過'),
                               (right_peaks[-1], blue, '最終・中心手前で折返し')):
        # Both returns have crossed the selected angles within 0.5 s of the peak.
        speeds = return_at_angles(current.gyro, peak, peak + 500, targets)
        axes[3].plot(targets, [speeds[x] for x in targets], 'o-', color=color, label=label)
        print(f'Right peak {peak} ms: ' + ', '.join(f'{x:g} deg = {speeds[x]:.1f} deg/s' for x in targets))
    axes[3].set(xlim=(4.9, 1.85), ylim=(0, 30), ylabel='左向きの胴体速度 (°/s)',
                xlabel='胴体の左右角 (°、右側から機械的中心へ)')
    axes[3].set_title('同じ試行の2回の戻りを、同じ胴体角で比較', loc='left', fontsize=10)
    axes[3].legend(frameon=False, loc='lower left', fontsize=8)
    for ax in axes:
        ax.grid(axis='y', alpha=.18)
        ax.spines[['right', 'top']].set_visible(False)
    fig.savefig(args.output, dpi=145)
    plt.close(fig)


if __name__ == '__main__':
    main()
