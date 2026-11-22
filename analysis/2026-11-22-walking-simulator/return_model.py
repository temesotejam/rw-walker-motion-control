"""Empirical return-to-centre model for the first inward swing from a confirmed positive peak.

Learn from comparable approaches with no further commanded pulse until physical ZERO
or reversal. An already completed pulse may still affect the current state.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np


PRIOR = (69501075, 224021399, 69728952, 205922296, 50784949, 52400264, 45820669, 239670034)
RECENT = (33125639, 216664125)
SECTIONS = (3., 2.)
CARRY_SPEED_DPS = 5.


@dataclass(frozen=True)
class State:
    run: int
    peak_ms: int
    section: float
    time_ms: int
    angle: float
    speed: float  # deg/s, positive toward centre
    acceleration: float  # backward difference of measured inward speed, deg/s²
    left: float
    right: float
    foot_age_ms: float
    foot_publish_ms: float


@dataclass(frozen=True)
class Observation:
    state: State
    outcome: str
    end_ms: float
    zero_speed: float  # zero by convention for an actual reversal; nan for censoring
    future_pulse: bool
    interfering_starts_ms: tuple
    previous_pulse_end_ms: float | None


def read_rows(folder, name):
    with (folder / (name + '.csv')).open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def foot_at(packets, published, t):
    # Only the latest packet already published when the IMU sample was recorded.
    i = bisect.bisect_right(published, t * 1000) - 1
    if i >= 0:
        p = packets[i]
        if all(p[k] for k in ('left_age_us', 'right_age_us', 'left_observed_deg', 'right_observed_deg')):
            publish_ms = published[i] / 1000
            ages_us = [float(p[k]) for k in ('left_age_us', 'right_age_us')]
            age = max(ages_us) / 1000 + t - publish_ms
            if min(ages_us) >= 0 and age <= 75 and int(p['left_sequence']) and int(p['right_sequence']):
                return float(p['left_observed_deg']), float(p['right_observed_deg']), age, publish_ms
    return math.nan, math.nan, math.nan, math.nan


def first_outcome(t, x, v, start, end, ordinary_zero):
    """Stop at the first physical crossing, confirmed reversal, or a gap/timeout."""
    lowest = start
    for i in range(start + 1, end):
        if t[i] - t[i-1] > 25:
            return 'censored', i-1, t[i-1], math.nan, t[i-1]
        if x[i] <= 0 < x[i-1]:
            alpha = x[i-1] / (x[i-1] - x[i])
            tm = t[i-1] + alpha * (t[i] - t[i-1])
            speed = v[i-1] + alpha * (v[i] - v[i-1])
            # Where the real controller observed this crossing, use its synchronized
            # detector event so a pulse issued *at* ZERO does not influence arrival.
            if ordinary_zero and abs(ordinary_zero[0] - tm) <= 10:
                tm, speed = ordinary_zero
            return 'crossed', i, tm, max(0., speed), tm
        if x[i] <= x[lowest]:
            lowest = i
        elif x[start] - x[lowest] >= .2 and x[i] - x[lowest] >= .2 and v[i] < 0:
            return 'reversed', lowest, t[lowest], 0., t[i]
    if end <= start:
        return 'censored', start, t[start], math.nan, t[start]
    return 'censored', end - 1, t[end-1], math.nan, t[end-1]


def load_sections(folder, sections=SECTIONS, episodes=None, *, foot_packets=None, executed_intervals=None):
    folder = Path(folder)
    meta = json.loads((folder/'metadata.json').read_text(encoding='utf8'))
    run = json.loads((folder/'header.json').read_text(encoding='utf8'))['run_start_us']
    if meta.get('metadata_event_detail_truncated') or meta.get('energy_control_autonomous_event_overflow'):
        raise ValueError(f'Run {run} has incomplete event history. Coasting exposure cannot be verified.')
    samples = [r for r in read_rows(folder, 'timeseries') if r['state_id']=='3'
               and r['pitch_mekf_detector_relative_deg'] and r['physical_roll_rate_dps']]
    t = [int(r['t_test_ms']) for r in samples]
    x = [float(r['pitch_mekf_detector_relative_deg']) for r in samples]
    v = [-float(r['physical_roll_rate_dps']) for r in samples]
    if any(a > b for a, b in zip(t, t[1:])):
        raise ValueError(f'Run {run} has nonmonotone IMU timestamps.')
    packets = sorted(read_rows(folder, 'foot_periodic') if foot_packets is None else foot_packets,
                     key=lambda p: int(p['publish_us']))
    published = [int(p['publish_us']) for p in packets]
    events = meta['energy_control_autonomous_zero_cross_events']
    zero_by_peak = {e['previous_peak_time_ms']: (e['zero_cross_time_ms'], abs(e['zero_cross_rate_dps']))
                    for e in events if e['event_kind']=='energy_control_zero_cross' and e['physical_next_peak_side']==-1}
    if executed_intervals is None:
        ends = {a['output_start_ms']: a['output_end_ms'] for a in meta['walking_energy']['records'] if a['executed']}
        pulses_by_start = {e['pulse_start_ms']: e['pulse_end_ms'] for e in events if e['output_executed']}
        pulses_by_start.update(ends)
        pulses = sorted(pulses_by_start.items())
    else:
        pulses = sorted(executed_intervals)
    rows, counts = [], Counter()
    peaks = meta['energy_control_autonomous_peak_events']
    for n, peak in enumerate(peaks):
        if peak['physical_peak_side'] != 1:
            continue
        peak_ms = peak['peak_time_ms']
        start = bisect.bisect_left(t, peak_ms)
        next_peak = peaks[n+1]['peak_time_ms'] if n+1 < len(peaks) else (t[-1] if t else peak_ms)
        end = bisect.bisect_right(t, min(next_peak, peak_ms + 1000))
        if start < end and t[start] - peak_ms <= 25:
            outcome, last, end_ms, zero_speed, confirmed_ms = first_outcome(t, x, v, start, end, zero_by_peak.get(peak_ms))
        else:
            outcome, last, end_ms, zero_speed, confirmed_ms = 'censored', start-1, peak_ms, math.nan, peak_ms
        if episodes is not None:
            episodes.append({'run':run, 'peak_ms':peak_ms, 'peak_available_ms':peak['peak_telemetry_capture_time_ms'],
                             'outcome':outcome, 'end_ms':end_ms, 'end_angle_deg':x[last] if start<=last else None,
                             'actual_zero_speed_dps':zero_speed if math.isfinite(zero_speed) else None})
        for section in sections:
            counts[(section, 'positive_peak')] += 1
            # One observation per peak and section. Peaks that never reach a section
            # cannot be rescued by a controller waiting for that observation.
            i = next((j for j in range(start+1, last+1) if x[j-1] > section >= x[j]
                      and v[j] > 0 and t[j] >= peak['peak_telemetry_capture_time_ms']), None)
            if i is None:
                counts[(section, 'no_section')] += 1
                counts[(section, 'no_section_' + outcome)] += 1
                continue
            j = bisect.bisect_right(t, t[i] - 25, 0, i) - 1
            if j < 0 or t[i] - t[j] > 50 or max(np.diff(t[j:i+1])) > 25:
                counts[(section, 'missing_imu_history')] += 1
                continue
            acceleration = (v[i] - v[j]) * 1000 / (t[i] - t[j])
            left, right, age, publish_ms = foot_at(packets, published, t[i])
            state = State(run, peak_ms, section, t[i], x[i], v[i], acceleration, left, right, age, publish_ms)
            intervening = tuple(a for a,b in pulses if a < confirmed_ms and b > t[i])
            previous = max((b for a,b in pulses if b <= t[i]), default=None)
            intervened = bool(intervening)
            rows.append(Observation(state, outcome, end_ms, zero_speed, intervened, intervening, previous))
            counts[(section, outcome)] += 1
            if intervened:
                counts[(section, 'overlapping_or_future_pulse')] += 1
            if not math.isfinite(left):
                counts[(section, 'missing_fresh_foot')] += 1
    return rows, counts


def features(state, use_foot=True):
    values = [state.speed, state.acceleration]
    if use_foot:
        values.append(state.left - state.right)
    return np.array(values)


def coast_reference(rows):
    return [r for r in rows if not r.future_pulse and r.outcome in ('crossed','reversed')]


def predict(train, state, use_foot=True):
    """Estimate ZERO speed and empirical crossing frequency. One nearest case per run.

    Returns None if the current observation is invalid or fewer than three
    historical runs are available. Distances are measured in training IQR units.
    """
    train = [r for r in coast_reference(train) if r.state.run != state.run and r.state.section == state.section
             and np.all(np.isfinite(features(r.state, use_foot)))]
    query = features(state, use_foot)
    if not train or not np.all(np.isfinite(query)):
        return None
    values = np.array([features(r.state, use_foot) for r in train])
    scale = np.maximum(np.percentile(values,75,axis=0)-np.percentile(values,25,axis=0),
                       ([1.,10.,.25] if use_foot else [1.,10.]))
    distance = np.sqrt(np.mean(((values-query)/scale)**2, axis=1))
    examples = []
    seen = set()
    for i in np.argsort(distance):
        run = train[i].state.run
        if run not in seen:
            examples.append(i)
            seen.add(run)
        if len(examples)==3:
            break
    if len(examples)<3:
        return None
    speeds = [train[i].zero_speed for i in examples]
    speed = float(np.mean(speeds))
    frequency = float(np.mean([train[i].outcome=='crossed' for i in examples]))
    # K/I = omega²/2. Average energies of individual analogs before reporting
    # aggregated values; intermittent complete reversals have zero arrival energy.
    energies = [0.5 * math.radians(v)**2 for v in speeds]
    target_energy = 0.5 * math.radians(CARRY_SPEED_DPS)**2
    return {'zero_speed':speed,'cross_fraction':frequency,'distance':float(distance[examples[-1]]),
            'energy_per_inertia':{
                'unit':'J/(kg m^2)', 'carry_speed_dps':CARRY_SPEED_DPS,
                'mean':float(np.mean(energies)), 'median':float(np.median(energies)),
                'mean_positive_deficit':float(np.mean([max(0., target_energy - k) for k in energies])),
                'below_carry_count':sum(v < CARRY_SPEED_DPS for v in speeds), 'analog_count':len(speeds)},
            'analogs':[(train[i].state.run,train[i].state.peak_ms) for i in examples]}


def constant_acceleration(state):
    squared = state.speed**2 + 2 * state.acceleration * state.angle
    return math.sqrt(max(0., squared)), squared>0


def evaluate(train, test):
    """Preserve identical rows across comparators; report coasting support separately."""
    scores = []
    for obs in coast_reference(test):
        s = obs.state
        with_foot = predict(train, s)
        without_foot = predict(train, s, False)
        if not with_foot or not without_foot:
            continue
        local_speed, local_cross = constant_acceleration(s)
        scores.append((obs, {'constant_speed':(s.speed,1.), 'constant_acceleration':(local_speed,float(local_cross)),
            'body_analogs':(without_foot['zero_speed'],without_foot['cross_fraction']),
            'body_foot_analogs':(with_foot['zero_speed'],with_foot['cross_fraction'])}, with_foot['distance']))
    return scores


def summary(scores):
    if not scores:
        return {}
    out = {'n':len(scores),'reversed':sum(r.outcome=='reversed' for r,_,_ in scores),
           'within_one_iqr':sum(dist<=1 for _,_,dist in scores)}
    for name in scores[0][1]:
        values=[(r.zero_speed,r.outcome=='crossed',pred[name][0],pred[name][1]) for r,pred,_ in scores]
        mae=np.mean([abs(v-guess) for v,_,guess,_ in values])
        brier=np.mean([(ok-prob)**2 for _,ok,_,prob in values])
        out[name]={'speed_mae_dps':round(float(mae),3),'brier':round(float(brier),4),
                   'missed_reversals':sum(not ok and prob>=.5 for _,ok,_,prob in values),
                   'false_reversal_warnings':sum(ok and prob<.5 for _,ok,_,prob in values)}
    return out


def evaluate_episodes(training_rows, observed_rows, peak_records=()):
    """Replay both decision sections sequentially, with at most one warning per peak.

    Pass the optional third argument from load_sections(..., episodes=peak_records)
    to retain peaks where neither section was observed. A new command between the
    first section and the endpoint invalidates the no-additional-command comparison.
    All states and predictions are computed before looking at that eligibility.
    """
    episodes = {(p['run'], p['peak_ms']): dict(p, observations=[]) for p in peak_records}
    for obs in sorted(observed_rows, key=lambda r: (r.state.run, r.state.time_ms)):
        s = obs.state
        key = (s.run, s.peak_ms)
        if key not in episodes:
            episodes[key] = {'run':s.run, 'peak_ms':s.peak_ms, 'end_ms':obs.end_ms,
                             'outcome':obs.outcome, 'observations':[]}
        p = predict(training_rows, s)
        _, local_cross = constant_acceleration(s)
        row = {
            'section_deg':s.section, 'time_ms':s.time_ms, 'body_angle_deg':s.angle,
            'inward_speed_dps':s.speed, 'inward_acceleration_dps2':round(s.acceleration, 3),
            'left_deg':s.left if math.isfinite(s.left) else None,
            'right_deg':s.right if math.isfinite(s.right) else None,
            'foot_age_ms':round(s.foot_age_ms, 3) if math.isfinite(s.foot_age_ms) else None,
            'foot_publish_ms':s.foot_publish_ms if math.isfinite(s.foot_publish_ms) else None,
            'previous_pulse_end_ms':obs.previous_pulse_end_ms,
            'interfering_pulse_starts_ms':list(obs.interfering_starts_ms),
            'baseline_warning':not local_cross,
            'prediction_available':p is not None,
            'supported':p is not None and p['distance'] <= 1,
            'warning':None if p is None or p['distance'] > 1 else p['cross_fraction'] < .5
        }
        if p is not None:
            row.update({'predicted_zero_speed_dps':round(p['zero_speed'], 3),
                        'cross_fraction':p['cross_fraction'], 'third_distance_iqr':round(p['distance'], 3),
                        'energy_per_inertia':p['energy_per_inertia'], 'analogs':p['analogs']})
        episodes[key]['observations'].append(row)

    counts = Counter()
    model_names = ('warning', 'baseline_warning')
    metrics = {name:{'detected_reversals':0, 'missed_reversals':0, 'false_warnings':0, 'correct_crossings':0,
                     'detected_reversal_leads_ms':[]} for name in model_names}
    for ep in episodes.values():
        rows = ep['observations']
        valid = [r for r in rows if r['supported']]
        if not rows:
            reason = 'no_usable_section'
        elif ep['outcome'] == 'censored':
            reason = 'censored'
        elif rows[0]['interfering_pulse_starts_ms']:
            reason = 'intervening_pulse'
        elif not any(r['prediction_available'] for r in rows):
            reason = 'missing_prediction'
        elif not valid:
            reason = 'outside_reference'
        else:
            reason = 'scored'
        counts[reason] += 1
        ep['score_status'] = reason
        for name in model_names:
            first = next((r['time_ms'] for r in valid if r[name]), None)
            ep[name + '_ms'] = first
            if reason != 'scored':
                continue
            m = metrics[name]
            if ep['outcome'] == 'reversed':
                if first is None:
                    m['missed_reversals'] += 1
                else:
                    m['detected_reversals'] += 1
                    m['detected_reversal_leads_ms'].append(round(ep['end_ms'] - first, 2))
            elif first is not None:
                m['false_warnings'] += 1
            else:
                m['correct_crossings'] += 1
    return {'counts':dict(counts), 'metrics':metrics, 'episodes':list(episodes.values())}


def run(recordings_root: Path, test_runs=RECENT):
    """Fit only the fixed prior runs; predict any disjoint converted run IDs."""
    root = Path(recordings_root)
    test_runs = tuple(int(r) for r in test_runs)
    if set(test_runs).intersection(PRIOR) or len(set(test_runs)) != len(test_runs):
        raise ValueError('Test run IDs must be unique and disjoint from the fixed training runs.')
    observations, all_peaks, coverage = {}, {}, {}
    for run_id in PRIOR + test_runs:
        all_peaks[run_id] = []
        folder = root / str(run_id)
        if json.loads((folder / 'header.json').read_text(encoding='utf8'))['run_start_us'] != run_id:
            raise ValueError(f'Run identity in the recording does not match folder {run_id}.')
        observations[run_id], coverage[run_id] = load_sections(folder, episodes=all_peaks[run_id])
    train = [r for run_id in PRIOR for r in observations[run_id]]
    result = {'training_run_ids':list(PRIOR), 'test_run_ids':list(test_runs), 'sections_deg':list(SECTIONS)}
    for split, ids in [('prior_leave_one_run_out', PRIOR), ('held_out', test_runs)]:
        rows = [r for run_id in ids for r in observations[run_id]]
        peaks = [p for run_id in ids for p in all_peaks[run_id]]
        counts = Counter()
        for run_id in ids:
            counts.update(coverage[run_id])
        sections = {}
        for angle in SECTIONS:
            section_rows = [r for r in rows if r.state.section == angle]
            sections[str(int(angle))] = {
                'coverage':{name:n for (a,name),n in counts.items() if a == angle},
                'scores':summary(evaluate(train, section_rows))}
        result[split] = {'sections':sections, 'sequential':evaluate_episodes(train, rows, peaks)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('converted_root', type=Path)
    parser.add_argument('--test-runs', type=int, nargs='+', default=RECENT)
    args = parser.parse_args()
    print(json.dumps(run(args.converted_root, args.test_runs), ensure_ascii=False, allow_nan=False, indent=2))


if __name__=='__main__':
    main()
