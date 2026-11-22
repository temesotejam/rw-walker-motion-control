"""Plot two held-out measured returns, the three historical analogues and the local coast forecast."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def angles(folder):
    with (folder / 'timeseries.csv').open(encoding='utf-8', newline='') as stream:
        return [(int(r['t_test_ms']), float(r['pitch_mekf_detector_relative_deg'])) for r in csv.DictReader(stream)
                if r['state_id'] == '3' and r['pitch_mekf_detector_relative_deg']]


def path(ax, samples, start, end, **style):
    points = [(t - start, x) for t, x in samples if start <= t <= end]
    if points:
        t, x = zip(*points)
        ax.plot(t, x, **style)


def plot(report, recordings, output):
    warnings = report['warnings']
    if not warnings:
        raise ValueError('No warnings available in this replay')
    returns = report['returns']
    lookup = {(r['run'], r['peak_ms']): r for s in ('prior_leave_one_run_out', 'held_out')
              for r in returns[s]['sequential']['episodes']}
    signals = {}
    def signal(run):
        if run not in signals:
            signals[run] = angles(recordings / str(run))
        return signals[run]

    plt.rcParams.update({'font.family': ['Yu Gothic', 'DejaVu Sans'], 'font.size': 10})
    fig, axes = plt.subplots(1, len(warnings), figsize=(10.8, 4.5), squeeze=False, sharey=True,
                             layout='constrained')
    for ax, review in zip(axes[0], warnings):
        run, when = review['run'], review['warning_ms']
        ep = lookup[run, review['peak_ms']]
        state = next(o for o in ep['observations'] if o['time_ms'] == when)
        for i, (donor, peak) in enumerate(state['analogs']):
            ref = lookup[donor, peak]
            start = next(o['time_ms'] for o in ref['observations'] if o['section_deg'] == state['section_deg'])
            path(ax, signal(donor), start, min(start + 200, ref['end_ms']), color='#999eab',
                 lw=1.5, ls='--', label='似た過去3走（中心通過か折返しまで）' if i == 0 else None)
        path(ax, signal(run), when, when + 200, lw=2.5, color='#15657b', label='今回の実測')
        ax.scatter(ep['end_ms'] - when, ep['end_angle_deg'], s=55, color='#15657b',
                   edgecolors='white', linewidths=.8, zorder=4, label='実測の折返し')
        for name, marker, color, label in (
                ('previous', 'x', '#ac592f', '従来の惰行予測'),
                ('selected', 'D', '#246e43', '右ピークからの戻りを加えた予測')):
            point = review['coast_to_zero'][name]
            ax.scatter(point['time_ms'], point['angle_deg'], marker=marker, s=65, color=color,
                       zorder=5, label=label)
        ax.axhline(0, color='#6e737e', lw=.9)
        ax.set(xlim=(0, 200), ylim=(-.15, 3.3), xlabel='警報からの時間 (ms)',
               title=f"試行 {run}　警報 {when / 1000:.3f} 秒")
        ax.grid(axis='y', color='#e3e6eb', lw=.7)
        ax.spines[['right', 'top']].set_visible(False)
    axes[0, 0].set_ylabel('胴体の左右角 (°)　正は右側')
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=2, frameon=False, fontsize=9)
    fig.suptitle('停止直前の惰行　実測の軌道と最長150 ms先の予測終点', fontsize=15)
    fig.savefig(output, dpi=145)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('recordings', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    plot(json.loads(args.report.read_text(encoding='utf-8')), args.recordings, args.output)
