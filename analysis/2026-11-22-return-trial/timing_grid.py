"""Replay alternatives for the two final returns with the frozen eight-run dynamics."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
import action_model as action
import phase_dynamics as phase
from receiving_carry import CarryReplay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference_recordings', type=Path)
    parser.add_argument('trial_recordings', type=Path)
    parser.add_argument('--carry-trial', action='store_true',
                        help='Compare 15 and 34 ms at the 15:39 trial decision using its measured 2 ms dispatch delay')
    args = parser.parse_args()
    training = [phase.PhaseRecording(args.reference_recordings / str(run)) for run in action.PRIOR]
    model = phase.PhaseFit([phase.training_windows(rec) for rec in training], phase.SELECTED)
    examples = action.action_examples(training)
    rows = []
    # Run 1's actual last aid started at 4762 ms, 62 ms after this common origin.
    trials = ([(135284989, 4869, (0,15,34), (2,))] if args.carry_trial else
              [(70067320, 4700, (0,28,33), (0,25,50,62,75)), (195727101, 3901, (0,21), (0,25,50,75))])
    for run, origin, widths, delays in trials:
        rec = phase.PhaseRecording(args.trial_recordings / str(run))
        observed = CarryReplay(args.trial_recordings / str(run))
        for endpoint in (None, 0.):
            out = action.compare(rec, origin, model, examples, widths, delays, endpoint)
            if out['status'] == 'ok':
                motion = observed.follow(dict(
                    peak_ms=out['peak_ms'], decision_ms=origin, terminal_deg=out['target_section_deg']))
                if motion['at_least_5_dps'] is not None:
                    motion['at_least_5_dps'] = bool(motion['at_least_5_dps'])
                out['recorded_motion_with_actual_inputs'] = motion
                if endpoint == 0.:
                    out['observed_followup'] = phase.zero_followup(rec, origin, out['peak_ms'], out['observed_followup'])
            rows.append(out)
    print(json.dumps(dict(training_runs=action.PRIOR, model=phase.SELECTED, comparisons=rows),
                     allow_nan=False, indent=2))


if __name__ == '__main__':
    main()
