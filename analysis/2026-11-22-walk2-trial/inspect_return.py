"""Compare the last left return in 41749536 against observed returns and frozen dynamics."""
import argparse
from bisect import bisect_right
import json
from pathlib import Path
import statistics
import sys

ANALYSIS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ANALYSIS / '2026-11-22-walking-simulator'), str(ANALYSIS / '2026-11-22-return-trial')]
import action_model
from aid_current_delivery import evaluate_trial
import phase_dynamics
from receiving_carry import CarryReplay


def what_if(folder, reference, late, widths):
    trial = phase_dynamics.PhaseRecording(folder)
    runs = [phase_dynamics.PhaseRecording(reference / str(run)) for run in action_model.PRIOR]
    windows = [phase_dynamics.training_windows(run) for run in runs]
    models = [phase_dynamics.PhaseFit(windows, phase_dynamics.SELECTED)]
    for omitted in range(len(runs)):
        subset = windows[:omitted] + windows[omitted + 1:]
        models.append(phase_dynamics.PhaseFit(subset, phase_dynamics.SELECTED))
    examples = action_model.action_examples(runs)
    aid = trial.aids[-1]
    delay = aid.start - aid.decision
    measured = CarryReplay(folder)
    events = []
    for at, target_widths in ((aid.decision, sorted({aid.width, *widths})), (late, sorted({0, *widths}))):
        for section in (None, 4., 0.):
            fits = [action_model.compare(trial, at, model, examples, target_widths, (delay,), section) for model in models]
            if fits[0]['status'] != 'ok':
                events.append(dict(decision_ms=at, requested_section=section, status=fits[0]['status']))
                continue
            primary = fits[0]
            target = primary['target_section_deg']
            observed = measured.follow(dict(peak_ms=aid.peak, decision_ms=at, terminal_deg=target))
            for i, width in enumerate(target_widths):
                scenario = primary['scenarios'][i]
                sensitivity = [f['scenarios'][i] for f in fits[1:]]
                events.append(dict(decision_ms=at, command_delay_ms=delay, section_deg=target, width_ms=width,
                                   initial_deg=primary['angle_deg'], initial_speed_dps=primary['speed_dps'],
                                   observed=observed, modeled=scenario,
                                   leave_one_run_out_reached=sum(s['outcome'] == 'section' for s in sensitivity)))
    return events


def figure(motion, metadata, deliveries, output):
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Meiryo', 'DejaVu Sans']
    fig, ax = plt.subplots(3, 1, figsize=(8.6, 7.5), layout='constrained', height_ratios=[1.2, 1, 1])
    t, rate, angle = motion.gyro
    seconds = [x / 1000 for x in t]
    on, off, reversal = (deliveries[-1][k] / 1000 for k in ('driver_on_ms', 'driver_off_ms', 'end_ms'))
    red, blue, green = '#9c412e', '#195c87', '#19705f'
    ax[0].plot(seconds, angle, color=blue, lw=1.5)
    ax[0].axhline(0, color='grey', lw=.7)
    for event in deliveries:
        pos = bisect_right(t, event['driver_on_ms'])-1
        ax[0].scatter(t[pos]/1000, angle[pos], s=35, color=red, zorder=3)
    ax[0].scatter([], [], s=35, color=red, label='左への補助開始')
    ax[0].legend(loc='upper right')
    ax[0].set(xlim=(0, 14), xlabel='測定開始からの時間 (s)', ylabel='胴体の左右角 (°)')
    ax[1].plot(seconds, rate, color=blue, label='胴体の左右角速度')
    ax[1].axhline(0, color='grey', lw=.7)
    visible_rates = [v for sec, v in zip(seconds, rate) if 5.5 <= sec <= 6.1]
    ax[1].set(ylabel='角速度 (°/s)', ylim=(min(visible_rates) - 4, max(visible_rates) + 4))
    periodic = metadata['foot_periodic']
    for j, label, color in [(4, '左足の前後角', green), (5, '右足の前後角', blue)]:
        ax[2].plot([p[1]/1e6 for p in periodic['samples']], [p[j] for p in periodic['samples']],
                   '.-', markersize=3, color=color, label=label)
    ax[2].set(ylabel='足角 (°)', xlabel='測定開始からの時間 (s)')
    for box in ax[1:]:
        box.axvspan(on, off, color=red, alpha=.18, label='最後の電流指令')
        box.axvline(reversal, color='black', ls='--', lw=1, label='中心手前の折返し')
        box.set(xlim=(5.5, 6.1))
        box.legend(loc='best', ncol=2, fontsize=8)
    for box in ax:
        box.spines[['top','right']].set_visible(False)
        box.grid(axis='y', lw=.5, alpha=.35)
    fig.suptitle('最後の左補助を出した後、胴体は右4.01°で折り返した')
    fig.savefig(output, dpi=155)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--late-ms', type=int, default=5779)
    parser.add_argument('--widths', nargs='+', type=int, default=[10, 20, 30, 34, 40])
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    if json.loads((args.trial/'header.json').read_bytes())['run_start_us'] != 41749536:
        parser.error('Expected converted recording 41749536')
    if any(w < 0 or w > 80 for w in args.widths):
        parser.error('Use 0–80 ms for pulse widths')
    motion = CarryReplay(args.trial)
    metadata = json.loads((args.trial/'metadata.json').read_bytes())
    deliveries = evaluate_trial(args.trial, motion)
    times, speeds, angles = motion.gyro
    tail = [(angle, speed) for t, angle, speed in zip(times, angles, speeds) if t >= 25000 and angle is not None]
    feet = metadata['foot_periodic']['samples']
    result = dict(profile=metadata['walking_profile'], deliveries=deliveries,
                  last_five_seconds=dict(body_deg=statistics.median(a for a, v in tail),
                                         abs_rate_dps=statistics.median(abs(v) for a, v in tail),
                                         foot_deg=[statistics.median(p[j] for p in feet if p[1]>=25e6 and p[j] is not None) for j in (4,5)]))
    if args.reference:
        result['scenarios'] = what_if(args.trial, args.reference, args.late_ms, args.widths)
    if args.plot:
        figure(motion, metadata, deliveries, args.plot)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
