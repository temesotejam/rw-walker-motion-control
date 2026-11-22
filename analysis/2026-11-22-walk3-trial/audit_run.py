"""Report the last assisted left return in a converted walking RWLOG."""
import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

ANALYSIS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ANALYSIS / '2026-11-22-walking-simulator'))
from aid_current_delivery import evaluate_trial
from receiving_carry import CarryReplay


def summarize(folder):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    motion = CarryReplay(folder)
    t, v, x = motion.gyro
    left_crossings = [i for i in range(1, len(t)) if x[i] <= 0 < x[i-1] and v[i] < 0]
    peaks = meta['energy_control_autonomous_peak_events']
    assert all(a['physical_peak_side'] == -b['physical_peak_side'] for a, b in zip(peaks, peaks[1:]))
    walking = meta['walking_energy']['records']
    aid = next(row for row in reversed(walking) if row['return_event'] and row['executed'] and row['side'] == -1)
    earlier = [row for row in walking if row['time_ms'] < aid['time_ms'] and row['executed'] and not row['return_event']]
    prior_main = next(row for row in reversed(earlier) if row['side'] == -1)
    right_main = next(row for row in reversed(earlier) if row['side'] == 1)
    right_prediction = right_main.get('right_arrival_elapsed_j')
    with (folder / 'walking_energy.csv').open() as file:
        rows = list(csv.DictReader(file))
    with (folder / 'foot_periodic.csv').open() as file:
        last_feet = [row for row in csv.DictReader(file) if int(row['slot_us']) >= 25_000_000]
    shape = next(row for row in rows if int(row['time_ms']) == aid['time_ms'])
    delivery = evaluate_trial(folder, motion)[-1]
    turn_index = min(range(len(t)), key=lambda i: abs(t[i] - delivery['end_ms']))
    last = [i for i in range(len(t)) if t[i] >= 25000]
    return dict(
        build=meta['diagnostic_build_id'], terminal=meta['terminal_state']['state'],
        left_crossings=len(left_crossings), peaks=len(peaks), last_peak=peaks[-1]['peak_amplitude_deg'],
        right_main_ms=right_main['actual_elapsed_ms'], right_arrival_uJ=1e6*right_prediction if right_prediction is not None else None,
        incoming_peak_uJ=1e6*aid['incoming_peak_energy_j'],
        previous_left_main_elapsed_ms=prior_main['actual_elapsed_ms'], reserve_ms=int(shape['arc_main_reserve_ms']),
        aid_decision_ms=aid['time_ms'], aid_elapsed_ms=aid['actual_elapsed_ms'],
        selected_request_uJ=1e6*aid['request_j'], modeled_on_time_work_uJ=1e6*aid['elapsed_work_pred_j'],
        envelope_after_ms=aid['envelope_after_ms'], reversal_ms=t[turn_index], reversal_right_deg=x[turn_index],
        final_right_deg=statistics.median(x[i] for i in last),
        final_abs_rate_dps=statistics.median(abs(v[i]) for i in last),
        final_feet_deg=[statistics.median(float(row[k]) for row in last_feet if row[k])
                        for k in ('left_est_deg', 'right_est_deg')], driver=delivery)


def plot_last_return(folder, output):
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    families = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams['font.family'] = next((name for name in ('Yu Gothic', 'Meiryo', 'Noto Sans CJK JP')
                                        if name in families), 'sans-serif')
    meta = json.loads((folder / 'metadata.json').read_bytes())
    m = CarryReplay(folder)
    t, _, angle = m.gyro
    report = summarize(folder)
    last = next(r for r in reversed(meta['walking_energy']['records']) if r['return_event'] and r['executed'])
    right = next(r for r in reversed(meta['walking_energy']['records']) if r['side'] == 1 and r['executed'])
    feet = [f for f in meta['foot_frames'] if f['state_id'] == 3 and f['timestamp_valid'] and
            f['frame_valid'] and f['zero_ready'] and f['left_valid'] and f['right_valid']]
    fig, (overview, detail) = plt.subplots(2, 1, figsize=(8.1, 5.8), layout='constrained')
    fig.suptitle(f"歩行実測　{meta['walking_profile']}　／　{folder.name}", fontsize=13)
    for ax in (overview, detail):
        ax.plot([ms/1000 for ms in t], angle, label='胴体の傾き（右が正）', color='#205379', lw=1.7)
        ax.axhline(0, color='#555555', lw=.8, ls='--')
        ax.set_ylabel('角度（°）')
        ax.spines[['top','right']].set_visible(False)
        ax.grid(axis='y', alpha=.16)
    overview.set(xlim=(0, 30), title='測定中の胴体の傾き')
    overview.set_xlabel('測定開始からの時間（秒）')
    for side, color, label in [('left','#a44b21','左足角（前が正）'), ('right','#42825c','右足角（前が正）')]:
        detail.plot([(f['frame_us']-m.clock)*1e-6 for f in feet], [f[side+'_deg'] for f in feet],
                    '.-', ms=3, lw=.85, color=color, label=label)
    detail.set_xlim(right['time_ms']*.001-.08, report['reversal_ms']*.001+.23)
    detail.set_ylim(-1.6, max(7., report['last_peak']+1.2))
    detail.set_title('最後に左へ戻ろうとした区間　　灰色はホイールの通電指令')
    detail.set_xlabel('測定開始からの時間（秒）')
    for row, y, align in ((right, 5.2,'left'), (last, 4.1,'center')):
        on, off = row['output_start_ms']*.001, row['output_end_ms']*.001
        detail.axvspan(on, off, color='#555555', alpha=.18)
        detail.annotate(f"{row['actual_elapsed_ms']} ms", (on,y), (on,y+.6), ha=align, fontsize=9,
                        arrowprops={'arrowstyle':'-','lw':.8})
    turn = report['reversal_ms']*.001, report['reversal_right_deg']
    detail.annotate(f"右{turn[1]:.2f}°で折返し", turn, (turn[0]+.015, 2.75), fontsize=9,
                    arrowprops={'arrowstyle':'->','lw':.8})
    detail.legend(loc='upper right', fontsize=8, frameon=False)
    fig.savefig(output, dpi=165)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path, nargs='+')
    parser.add_argument('--figure', type=Path, help='plot the first trial')
    args = parser.parse_args()
    if args.figure:
        plot_last_return(args.trial[0], args.figure)
    for trial in args.trial:
        print(json.dumps(summarize(trial), indent=2, allow_nan=False))
