"""Compare spending the same added command time at the 68907120 aid or at the receiving event."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
import action_model as action
import phase_dynamics as phase
from receiving_carry import CarryReplay, first_receiving_outcomes


def evaluate(reference, trial, candidate_ms):
    rec = phase.PhaseRecording(trial)
    if rec.run != 68907120 or len(rec.aids) != 1:
        raise ValueError('Expected walk1 trial 68907120 with one executed left aid')
    aid, = rec.aids
    if not aid.width < candidate_ms <= 80:
        raise ValueError('The comparison needs an extended aid no longer than the half-cycle budget')
    # Reuse the previously selected dynamics and its fixed eight training runs.
    training = [phase.PhaseRecording(reference / str(i)) for i in action.PRIOR]
    windows = [phase.training_windows(r) for r in training]
    models = [phase.PhaseFit(windows, phase.SELECTED)]
    for i in range(len(windows)):
        models.append(phase.PhaseFit(windows[:i] + windows[i+1:], phase.SELECTED))
    examples = action.action_examples(training)
    measured = CarryReplay(trial)
    receiving, = (o for o in first_receiving_outcomes(measured) if o['peak_ms'] == aid.peak)
    # The image is eligible only when its CAD calculation was complete. At this
    # second origin the original aid has already been sent in every scenario.
    late_ms = math.ceil(receiving['decision_ms'])
    if late_ms <= aid.end:
        raise ValueError('The first aid had not ended when the receiving calculation arrived')
    delay = aid.start - aid.decision
    rows = []
    for event, at, widths in [('original_aid', aid.decision, (aid.width, candidate_ms)),
                               ('receiving', late_ms, (0, candidate_ms - aid.width))]:
        for target in (None, 2., 0.):
            fits = [action.compare(rec, at, model, examples, widths, (delay,), target) for model in models]
            if any(f['status'] != 'ok' for f in fits):
                raise ValueError(f'Unavailable state at {at} ms')
            full = fits[0]
            for i, width in enumerate(widths):
                scenario = full['scenarios'][i]
                # The matched counts describe the full training set. The omitted
                # run changes just the dynamics fit for this sensitivity check.
                outcomes = [f['scenarios'][i] for f in fits[1:]]
                rows.append(dict(event=event, origin_ms=at, section_deg=full['target_section_deg'], width_ms=width,
                                 electrical_goal_ma=full['goal_ma'], rise_ms=full['rise_tau_ms'],
                                 matched_states=full['matched_prior_actions'],
                                 matched_actions=scenario['matched_action_count'],
                                 modeled=scenario, recorded=full['observed_followup'],
                                 observed_gyro=measured.follow(dict(peak_ms=aid.peak, decision_ms=at,
                                                                   terminal_deg=full['target_section_deg'])),
                                 leave_one_run_out_reached=sum(s['outcome'] == 'section' for s in outcomes),
                                 leave_one_run_out_angle_range=[min(s['angle_deg'] for s in outcomes),
                                                               max(s['angle_deg'] for s in outcomes)]))
    peak = next(p for p in rec.peaks if p['peak_time_ms'] == aid.peak)
    state = rec.observation(aid.decision, peak)
    comparison_aids = []
    for run_id, earlier, pulse in examples:
        if not (abs(earlier.angle-state.angle) <= 1 and abs(earlier.speed-state.speed) <= 5 and
                abs(earlier.foot_difference-state.foot_difference) <= 2 and abs(pulse.width-candidate_ms) <= 5
                and abs((pulse.start-pulse.decision)-delay) <= 5):
            continue
        past = CarryReplay(reference / str(run_id))
        comparison_aids.append(dict(run=run_id, decision_ms=pulse.decision, width_ms=pulse.width,
                                    center=past.follow(dict(peak_ms=pulse.peak, decision_ms=pulse.decision,
                                                            terminal_deg=0.)),
                                    receiving=next((o for o in first_receiving_outcomes(past)
                                                    if o['peak_ms'] == pulse.peak), None)))
    return dict(training_ids=action.PRIOR, nominal_lookahead_ms=action.LIMIT,
                recorded_driver_delay_ms=delay, candidate_ms=candidate_ms, rows=rows, matched=comparison_aids)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference_recordings', type=Path)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--candidate-ms', type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.reference_recordings, args.trial, args.candidate_ms),
                     indent=2, allow_nan=False))
