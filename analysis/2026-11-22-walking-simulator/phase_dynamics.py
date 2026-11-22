"""Compare the recorded leftward return with causal local rollouts, one whole run at a time."""
from collections import Counter
from dataclasses import dataclass, replace
from pathlib import Path
import math

import numpy as np

import action_model as action
from loss_observer import coast_forecast
from foot_kinematics import FootHistory
from return_model import load_sections


@dataclass(frozen=True)
class PhaseState(action.State):
    elapsed_ms: float
    fastest_inward_dps: float
    left_angle: float
    left_rate: float
    right_rate: float
    detector_peak_deg: float

    def after_slot(self, angle, speed, acceleration):
        # Only the IMU state advances. Do not borrow subsequent camera measurements.
        return replace(self, angle=angle, speed=speed, acceleration=acceleration,
                       fastest_inward_dps=max(self.fastest_inward_dps,speed),
                       elapsed_ms=self.elapsed_ms+action.STEP)

    def has_rates(self):
        return math.isfinite(self.left_rate) and math.isfinite(self.right_rate)


class PhaseRecording(action.Recording):
    def __init__(self, folder):
        super().__init__(folder)
        self.folder = folder
        self.history = FootHistory.from_folder(folder)

    def observation(self, t, peak):
        state = super().observation(t, peak)
        if state is None:
            return None
        feet = self.history.at(t)
        start, end = np.searchsorted(self.t, [peak['peak_time_ms'], t], side='right')
        fastest = max(state.speed, max(self.v[start:end],default=state.speed))
        def rate(foot):
            return foot.rate_dps if foot and foot.rate_dps is not None else math.nan
        detector_peak = peak.get('detector_peak_angle_deg')
        if detector_peak is None:
            detector_peak = self.x[start-1] if start > 0 else math.nan
        if not math.isfinite(detector_peak):
            return None
        return PhaseState(state.angle, state.speed, state.acceleration, state.foot_difference,
                          state.peak_amplitude, t - peak['peak_time_ms'], fastest,
                          feet.left.angle_deg if feet.left else math.nan, rate(feet.left), rate(feet.right),
                          detector_peak)


def body_features(s, previous, future):
    # Mean inward speed since the previous right peak distinguishes a late, slow
    # return from one that has just accelerated away from a similar peak.
    elapsed = max(action.STEP, s.elapsed_ms)
    mean_speed = (s.detector_peak_deg - s.angle) * 1000 / elapsed
    return [*action.features(s, previous, future)[:-2], mean_speed, elapsed / (200 + elapsed),
            previous, future]


def energy_features(s, previous, future):
    # Reduction in lateral kinetic energy per unit inertia since peak inward speed.
    loss = .5*(math.radians(s.fastest_inward_dps)**2-math.radians(s.speed)**2)
    return [*body_features(s, previous, future)[:-2], max(0.,loss), previous,future]


def foot_features(s, previous, future):
    return [*energy_features(s, previous, future)[:-2], s.left_angle, s.left_rate, s.right_rate,
            previous, future]


def training_windows(rec):
    """Include inward labels up to the terminal event, never a post-crossing sample."""
    windows = []
    for peak, start, end, outcome in rec.segments:
        terminal_time = float(rec.t[end])
        if outcome == 'crossed' and end > start:
            xa, xb = rec.x[end-1:end+1]
            if xb <= 0 < xa:
                terminal_time = rec.t[end-1] + xa / (xa-xb) * (rec.t[end]-rec.t[end-1])
        t = max(rec.t[start] + action.STEP, peak['peak_telemetry_capture_time_ms'])
        while t < terminal_time:
            until = min(t + action.STEP, terminal_time)
            duration = until - t
            if duration < 8:
                break
            state = rec.observation(t, peak)
            lo = int(np.searchsorted(rec.t, t, side='right')-1)
            hi = int(np.searchsorted(rec.t, until))
            if (state and state.angle > 0 and state.speed > 0 and hi < len(rec.t)
                    and hi > lo and np.max(np.diff(rec.t[lo:hi+1])) <= 15):
                speed = float(np.interp(until, rec.t[hi-1:hi+1], rec.v[hi-1:hi+1]))
                # Use the measured gyro value at the turning point. ZERO on a
                # failure is the later scoring convention, not a gyro measurement.
                dt = until - rec.t[lo]
                previous = action.mean_current(rec.current, t-action.STEP)
                current = action.mean_current(rec.current, t, duration)
                if math.isfinite(previous) and math.isfinite(current):
                    windows.append((state, previous, current, (speed-state.speed)*1000/dt,
                                    bool(duration < action.STEP)))
            t += action.STEP
    return windows


class PhaseFit:
    def __init__(self, groups, mode):
        self.mode = mode
        self.coast_observer = coast_forecast if mode == 'body_coast' else None
        def fit(fn, complete_rates=False):
            rows = [[(fn(state, old, new), dv) for state, old, new, dv, _ in group
                     if not complete_rates or state.has_rates()] for group in groups]
            return action.Fit(rows, feature_fn=fn)
        self.body = fit({'terminal':action.features, 'body':body_features, 'body_coast':body_features,
                         'energy':energy_features, 'feet':energy_features}[mode])
        self.foot = fit(foot_features, True) if mode == 'feet' else None
        self.last_state = None

    def acceleration(self, s, old_ma, new_ma):
        # The ridge model is affine in new current within the 25 ms state update.
        if s is not self.last_state or self.last_ma != old_ma:
            use = self.foot if self.foot is not None and s.has_rates() else self.body
            self.offset = use.acceleration(s, old_ma, 0.)
            self.slope = use.beta[-1] / use.scale[-1]
            self.last_state, self.last_ma = s, old_ma
        return self.offset + self.slope * new_ma


def origins(rec):
    """Pre-specified comparisons: actual aid decisions and one +3 deg coast per return."""
    for pulse in rec.aids:
        yield 'aid', pulse.decision, pulse.width, pulse.start - pulse.decision, None
    for obs in load_sections(rec.folder, sections=(3.,))[0]:
        yield 'coast', obs.state.time_ms, 0, 0, obs


def crossing_speed(rec, origin, end, peak_ms, *, factual_input_at_origin=False):
    """Match the model's signed gyro series using only pre-command samples.

    Firmware: detector=(gy-MEKF_bias)*scale; physical=gy-static_bias.
    Stored biases are quantized, so use the last available pair before the
    crossing. If unavailable, require a raw angle/gyro bracket before actuation.
    """
    cutoff = min((p.start for p in rec.pulses if p.decision>origin
                  or (p.decision==origin and not factual_input_at_origin)),default=math.inf)
    e = rec.left_zeros.get(peak_ms)
    if e and e['zero_cross_time_ms']==end and rec.gyro_scale and math.isfinite(rec.gyro_scale):
        stamp = e.get('interpolated_crossing_time_ms')
        rate = e.get('zero_cross_rate_dps')
        if stamp is not None and math.isfinite(stamp) and abs(stamp-end)<=10 and rate is not None and math.isfinite(rate):
            i = min(np.searchsorted(rec.t,min(stamp,end),side='right'),np.searchsorted(rec.t,cutoff))-1
            if i>=0 and 0<=stamp-rec.t[i]<=15 and math.isfinite(rec.bias_difference[i]):
                inward = -(rate/rec.gyro_scale+rec.bias_difference[i])
                return dict(speed_dps=round(float(inward),2),speed_source='synchronized_gyro_conversion',
                            bias_age_ms=round(float(stamp-rec.t[i]),3))
    first = max(1,np.searchsorted(rec.t,end-10))
    last = min(len(rec.t),np.searchsorted(rec.t,end+10,side='right')+1)
    for i in range(first,last):
        if rec.t[i]>=cutoff:
            break
        if (rec.t[i-1]>=origin and 0<rec.t[i]-rec.t[i-1]<=15 and rec.x[i]<=0<rec.x[i-1]):
            fraction = rec.x[i-1]/(rec.x[i-1]-rec.x[i])
            crossing = rec.t[i-1]+fraction*(rec.t[i]-rec.t[i-1])
            if abs(crossing-end)<=10:
                speed = rec.v[i-1]+fraction*(rec.v[i]-rec.v[i-1])
                return dict(speed_dps=round(float(speed),2),speed_source='pre_command_gyro_bracket')
    return None


def coast_endpoint(rec, observation, local_observation):
    """Do not censor a ZERO state because its main pulse starts at that ZERO.

    The return labels resolve synchronized detector/IMU timestamps and check
    for another pulse until reversal is confirmed. Endpoints are scored only
    for the actual 150 ms local horizon.
    """
    elapsed = observation.end_ms - observation.state.time_ms
    if not observation.future_pulse and 0 <= elapsed <= action.LIMIT:
        if observation.outcome == 'crossed':
            measurement = crossing_speed(rec,observation.state.time_ms,observation.end_ms,observation.state.peak_ms)
            if measurement is None:
                return dict(outcome='censored',reason='no_pre_command_gyro_at_zero')
            return dict(outcome='section',angle_deg=0.,time_ms=round(elapsed,1),**measurement)
        if observation.outcome == 'reversed':
            return dict(outcome='reversed',angle_deg=round(float(np.interp(observation.end_ms,rec.t,rec.x)),3),
                        speed_dps=0.,time_ms=round(elapsed,1),speed_source='reversal_convention')
    return dict(local_observation,speed_source='local_imu') if local_observation['outcome']!='censored' else local_observation


def metrics(rows, method):
    comparable = [r for r in rows if r['actual']['outcome'] != 'censored']
    if not comparable:
        return {'cases': 0}
    errs, matched, timed, advantages, paired = [], [], [], [], []
    status = Counter()
    for r in comparable:
        actual, pred = r['actual'], r[method]
        status[actual['outcome'] + '_as_' + pred['outcome']] += 1
        err = abs(pred['speed_dps'] - actual['speed_dps'])
        errs.append(err)
        advantages.append(abs(r['previous']['speed_dps']-actual['speed_dps']) - err)
        if actual['outcome'] == pred['outcome']:
            matched.append(err)
            if actual['outcome'] == r['previous']['outcome']:
                paired.append((err,abs(r['previous']['speed_dps']-actual['speed_dps'])))
            if actual['outcome'] != 'horizon':
                timed.append(abs(pred['time_ms']-actual['time_ms']))
    return {
        'cases':len(comparable), 'speed_mae_dps':round(float(np.mean(errs)),3),
        'same_endpoint_cases':len(matched),
        'same_endpoint_speed_mae_dps':round(float(np.mean(matched)),3) if matched else None,
        'common_correct_endpoint_with_previous':len(paired),
        'speed_mae_model_and_previous_on_common_dps':[
            round(float(np.mean(vals)),3) for vals in zip(*paired)] if paired else None,
        'matched_event_time_mae_ms':round(float(np.mean(timed)),3) if timed else None,
        'improved_vs_previous':int(sum(x > 0 for x in advantages)),
        'worse_vs_previous':int(sum(x < 0 for x in advantages)), 'outcomes':dict(status)}


METHODS = ('previous', 'terminal', 'body', 'energy', 'feet', 'body_coast')
SELECTED = 'body_coast'


def evaluate_split(ids, recs, prior, windows):
    rows, excluded = [], Counter()
    for test in ids:
        rec = recs[test]
        train_ids = [i for i in action.PRIOR if i != test]
        groups = [windows[i] for i in train_ids]
        models = {'previous':action.Fit([prior[i] for i in train_ids])}
        models.update({key:PhaseFit(groups,key) for key in METHODS[1:]})
        examples = action.action_examples([recs[i] for i in train_ids])
        for group,t,width,delay,coast in origins(rec):
            if group == 'coast' and any(p.decision == t for p in rec.pulses):
                excluded['coast_at_command'] += 1
                continue
            row = dict(run=test, group=group, time_ms=t)
            for name, model in models.items():
                report = action.compare(rec,t,model,examples,(width,),(delay,),0. if coast else None)
                if report['status'] != 'ok':
                    excluded[group+'_'+report['status']] += 1
                    break
                row[name] = report.get('coast_forecast', report['scenarios'][0])
                if name == SELECTED and 'coast_observation' in report:
                    row['coast_observation'] = report['coast_observation']
            else:
                row['actual'] = coast_endpoint(rec,coast,report['observed_followup']) if coast else report['observed_followup']
                if coast:
                    row['physical_return_outcome'] = coast.outcome
                    row['intervened_before_return'] = coast.future_pulse
                rows.append(row)
    sources = Counter(r['actual'].get('speed_source',r['actual'].get('reason','censored'))
                      for r in rows if r['group']=='coast')
    coast_cases = [r for r in rows if r['group']=='coast' and r['actual']['outcome']!='censored']
    audit = {'scored':len(coast_cases),
             'with_correction':sum(r['coast_observation']['correction_dps2']<0 for r in coast_cases),
             'recorded_reversals':sum(r['actual']['outcome']=='reversed' for r in coast_cases),
             'censored_after_valid_origin':sum(r['group']=='coast' and r['actual']['outcome']=='censored' for r in rows)}
    for method in ('body', SELECTED):
        audit[method] = {
            'correct_endpoints':sum(r[method]['outcome']==r['actual']['outcome'] for r in coast_cases),
            'detected_reversals':sum(r[method]['outcome']==r['actual']['outcome']=='reversed' for r in coast_cases),
            'false_reversals_within_local_horizon':sum(r[method]['outcome']=='reversed' and r['actual']['outcome']!='reversed'
                                                       for r in coast_cases)}
    return {'rows':rows, 'excluded_origins':dict(excluded), 'coast_speed_sources':dict(sources), 'coast_observer':audit, 'metrics':{
        group:{method:metrics([r for r in rows if r['group']==group],method) for method in METHODS}
        for group in ('aid','coast')}}


def reference_rows(root):
    recs = {i:PhaseRecording(Path(root)/str(i)) for i in action.PRIOR}
    return recs, {i:action.samples(rec) for i,rec in recs.items()}, {i:training_windows(rec) for i,rec in recs.items()}


def evaluate_trial(root, folder):
    """Evaluate an independent raw-converted trial against the frozen eight reference runs."""
    trial = PhaseRecording(Path(folder))
    if trial.run in action.PRIOR:
        raise ValueError('Evaluation trial belongs to the fitted reference set')
    recs, prior, windows = reference_rows(root)
    recs[trial.run] = trial
    return dict(run=trial.run, reference_runs=list(action.PRIOR), selected_model=SELECTED,
                **evaluate_split((trial.run,),recs,prior,windows))


def simulate_at(recordings_root, run_id, time_ms, widths=(0,10,20), delays=(0,25), section_deg=None):
    """Same command and guard semantics as action_model; never propagate past the fitted positive side."""
    run_id = int(run_id)
    recs = {i:PhaseRecording(Path(recordings_root)/str(i)) for i in dict.fromkeys((*action.PRIOR,run_id))}
    train = [r for i,r in recs.items() if i != run_id]
    result = action.compare(recs[run_id],time_ms,PhaseFit([training_windows(r) for r in train],SELECTED),
                            action.action_examples(train),widths,delays,section_deg)
    result['model'] = SELECTED
    result['training_runs'] = [r.run for r in train]
    if result.get('target_section_deg') == 0. and result['status'] == 'ok':
        result['observed_followup'] = zero_followup(recs[run_id],time_ms,result['peak_ms'],result['observed_followup'])
    return result


def zero_followup(rec, origin, peak_ms, local_observation):
    """Apply the same first-return labels to a user's arbitrary query time."""
    e = rec.left_zeros.get(peak_ms)
    ordinary = (e['zero_cross_time_ms'],0.) if e else None  # speed comes from the matching series below
    start = np.searchsorted(rec.t,peak_ms)
    next_peak = next((p['peak_time_ms'] for p in rec.peaks if p['peak_time_ms']>peak_ms),peak_ms+1000)
    end = np.searchsorted(rec.t,min(next_peak,peak_ms+1000),side='right')
    kind,_,stop,_,confirmed = action.first_outcome(rec.t,rec.x,rec.v,start,end,ordinary)
    if (kind not in ('crossed','reversed') or not 0<stop-origin<=action.LIMIT
            or any(p.decision>origin and p.start<confirmed for p in rec.pulses)):
        return local_observation
    if kind=='reversed':
        return dict(outcome='reversed',angle_deg=round(float(np.interp(stop,rec.t,rec.x)),3),
                    speed_dps=0.,time_ms=round(float(stop-origin),1),speed_source='reversal_convention')
    measurement = crossing_speed(rec,origin,stop,peak_ms,factual_input_at_origin=True)
    if measurement is None:
        return dict(outcome='censored',reason='no_pre_command_gyro_at_zero')
    return dict(outcome='section',angle_deg=0.,time_ms=round(float(stop-origin),1),**measurement)


def run(recordings_root, test_runs=action.RECENT):
    test_runs = tuple(test_runs)
    if set(test_runs).intersection(action.PRIOR):
        raise ValueError('Evaluation runs must be separate from reference runs')
    recs, prior, windows = reference_rows(recordings_root)
    recs.update({i:PhaseRecording(Path(recordings_root)/str(i)) for i in test_runs})
    result = {'reference_runs':list(action.PRIOR), 'test_runs':list(test_runs),
              'selected_model':SELECTED,
              'training_windows':{i:{'all':len(rows), 'short_terminal':sum(w[-1] for w in rows),
                                     'with_foot_rates':sum(w[0].has_rates() for w in rows)}
                                  for i, rows in windows.items()}}
    for split, ids in [('prior',action.PRIOR), ('held',test_runs)]:
        result[split] = evaluate_split(ids,recs,prior,windows)
    return result
