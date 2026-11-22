"""Plot measured body motion, available camera observations and output in the last left return."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
from receiving_carry import CarryReplay


def plot_trial(ax, folder, label):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    peak = meta['energy_control_autonomous_peak_events'][-1]
    origin = peak['peak_time_ms']
    assert peak['physical_peak_side'] == 1
    replay = CarryReplay(folder)
    t, rates, angles = (np.array(series, dtype=float) for series in replay.gyro)
    interval = (t >= origin - 25) & (t <= origin + 385)
    elapsed = t[interval] - origin
    ax[0].plot(elapsed, angles[interval], color='#145b89')
    ax[1].plot(elapsed, -rates[interval], color='#145b89')
    ax[1].axhline(3, color='#707070', linestyle=':', lw=1)
    ax[1].text(376, 3.8, '3°/s', ha='right', color='#555555', fontsize=8)
    frames = [f for f in meta['foot_frames'] if f['state_id'] == 3 and f['control_observed_us']]
    for side, color, name in [('left', '#29764e', '左足'), ('right', '#985484', '右足')]:
        known = [(f['control_observed_us'] - replay.clock) / 1000 - origin for f in frames]
        values = [f[side+'_deg'] if f['timestamp_valid'] and f['frame_valid'] and f['zero_ready']
                  and f[side+'_valid'] and f[side+'_control_range_ok'] else float('nan') for f in frames]
        ax[2].plot(known, values, '.-', color=color, lw=1.3, ms=4, label=name)
    outcome = replay.follow({'peak_ms': origin, 'decision_ms': origin + 70, 'terminal_deg': 0.})
    if outcome['status'] != 'reversed':
        raise ValueError(f"{folder.name} has no observed reversal before ZERO")
    reverse = outcome['end_ms'] - origin
    ax[0].scatter([reverse], [np.interp(reverse, elapsed, angles[interval])], color='#aa3e32', zorder=3)
    ax[0].text(reverse - 4, .4, f"{outcome['end_ms']/1000:.3f} s", color='#90382b', ha='right', fontsize=9)
    aids = [r for r in meta['walking_energy']['records'] if r['peak_ms']==origin and r['return_event'] and r['executed']]
    for a in ax:
        for r in aids:
            a.axvspan(r['output_start_ms']-origin, r['output_end_ms']-origin, color='#ecb95a', alpha=.35)
        a.axvline(reverse, color='#aa3e32', linestyle='--', lw=1)
        a.set_xlim(-25, 385)
        a.spines[['top','right']].set_visible(False)
        a.grid(axis='y', alpha=.20)
    ax[0].set_title(f"{label}   ·   {origin/1000:.3f} s", loc='left', weight='bold')
    if aids:
        r = aids[-1]
        ax[0].text((r['output_start_ms']+r['output_end_ms'])/2-origin, 8.7,
                   f"{r['actual_elapsed_ms']} ms", ha='center', color='#845500', fontsize=9)
    if folder.name == '195727101':
        # Inspect the gyro samples strictly between the two fresh receiving-sole snapshots.
        window = np.flatnonzero((t > 4001.050) & (t < 4027.877))
        i = window[np.argmin(-rates[window])]
        point = (t[i] - origin, -rates[i])
        ax[1].plot(*point, 'o', ms=4, color='#8c4b22')
        ax[1].annotate(f'{-rates[i]:.2f}°/s', point, xytext=(228, -8), fontsize=8,
                       arrowprops={'arrowstyle': '->', 'color': '#8c4b22'}, color='#8c4b22')
    return ax


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_root', type=Path)
    parser.add_argument('output_png', type=Path)
    args = parser.parse_args()
    plt.rcParams['font.family'] = ['Meiryo', 'DejaVu Sans']
    fig, axes = plt.subplots(3, 2, figsize=(10, 7.4), sharex='col', sharey='row')
    for i, (run, label) in enumerate([('70067320','1走目　比較条件'), ('195727101','2走目　履歴補正条件')]):
        plot_trial(axes[:,i], args.converted_root / run, label)
        axes[2,i].set_xlabel('最後の右ピークからの時間（ms）')
    axes[0,0].set(ylabel='胴体の左右角（°）', ylim=(-.3,9.5))
    axes[1,0].set_ylabel('左へ戻る角速度（°/s）')
    axes[2,0].set(ylabel='カメラで測った足角（°）', ylim=(-1,6.2))
    axes[2,1].legend(loc='upper right', frameon=False, ncol=2)
    fig.suptitle('最後の右ピークから左に戻るまで', fontsize=16, weight='bold', x=.09, ha='left')
    fig.subplots_adjust(left=.09,right=.98,top=.89,bottom=.16,hspace=.13,wspace=.12)
    fig.text(.09,.045,'黄色は左向きの補助通電、赤い破線は機械的中心の手前で折り返した時刻。\n'
             '足角は制御側が結果を受け取った時刻に表示。検出できなかった画像では線が途切れる。',
             color='#444444', fontsize=9)
    fig.savefig(args.output_png, dpi=155, facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    main()
