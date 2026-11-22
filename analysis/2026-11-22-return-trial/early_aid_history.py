"""Audit a +4.5 deg early trigger with and without same-half-cycle intervention matching."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-22-walking-simulator'))
import return_model


def observations(folder):
    meta = json.loads((folder / 'metadata.json').read_bytes())
    energy = meta['walking_energy']
    if energy['truncated'] or energy['total'] != len(energy['records']):
        raise ValueError(f'Incomplete pulse archive in {folder}')
    states, _ = return_model.load_sections(folder, (4.5,))
    rows = []
    for obs in states:
        state = obs.state
        issued = [a for a in energy['records'] if a['peak_ms'] == state.peak_ms and
                  a['time_ms'] <= state.time_ms and a['executed']]
        # Do not borrow an eventual command acceptance from after this observation.
        if any(a['output_start_ms'] > state.time_ms for a in issued):
            continue
        main_used = any(a['peak_ms'] == state.peak_ms and a['time_ms'] <= state.time_ms and
                        not a['return_event'] and (a['executed'] or
                        (a['valid'] and a['coast'] and a['energy_admission_allowed']))
                        for a in energy['records'])
        if main_used:
            continue
        rows.append((obs, any(a['return_event'] for a in issued)))
    return rows


def compare(reference_root, trial_root):
    groups = {'earlier_8': return_model.PRIOR, 'nov20_held_out': return_model.RECENT,
              'nov22_held_out': (70067320, 195727101, 135284989)}
    trials = {n: observations((trial_root if group == 'nov22_held_out' else reference_root) / str(n))
              for group, ids in groups.items() for n in ids}
    prior = [record for n in return_model.PRIOR for record in trials[n]]
    previous_aid = {(o.state.run, o.state.peak_ms): spent for o, spent in prior}
    results = {}
    for group, ids in groups.items():
        counts, alerts = Counter(), []
        for run in ids:
            for obs, spent in trials[run]:
                state = obs.state
                scored = not obs.future_pulse and obs.outcome in ('crossed', 'reversed')
                # Only the absence of a previous commitment could allow a new aid.
                # Succeeding pulses are used for scoring eligibility only.
                if not spent and scored:
                    counts[f'unused_{obs.outcome}'] += 1
                for key, training in [('pooled', [o for o, _ in prior]),
                                      ('same_history', [o for o, s in prior if s == spent])]:
                    p = return_model.predict(training, state)  # excludes query run for all earlier rows
                    supported = p is not None and p['distance'] <= 1
                    if not spent and scored and supported:
                        counts[f'{key}_supported_{obs.outcome}'] += 1
                    alarm = supported and p['cross_fraction'] < .5
                    if scored and alarm:
                        counts[f'{key}_alarm_{"spent" if spent else "unused"}_{obs.outcome}'] += 1
                    if run not in return_model.PRIOR and not spent and obs.outcome == 'reversed':
                        alerts.append(dict(run=run, peak_ms=state.peak_ms, at_ms=state.time_ms,
                            future_pulse=obs.future_pulse, model=key, alarm=bool(alarm),
                            analogs=[] if p is None else [
                                dict(run=r, peak_ms=t, aided=previous_aid[r, t]) for r, t in p['analogs']]))
        results[group] = dict(counts=dict(counts), unused_reversals=alerts)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference_root', type=Path)
    parser.add_argument('trial_root', type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.reference_root, args.trial_root), indent=2, allow_nan=False))
