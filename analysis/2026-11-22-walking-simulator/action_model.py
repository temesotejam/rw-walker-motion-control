"""Local simulator of a positive-peak return towards mechanical ZERO.

Fit 25 ms changes in gyro speed to body/foot state and the signed current
predicted from recorded pulses. A scenario starts from a real observation and
replaces subsequent commands. Positive speed/current point towards the left.
"""
import argparse
import bisect
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from return_model import PRIOR, RECENT, first_outcome, foot_at, read_rows

STEP = 25
LIMIT = 150  # maximum local foot-state forecast, matching support_transfer_plan.h
FALL = .070


@dataclass(frozen=True)
class Pulse:
    decision: int
    start: int
    end: int
    side: int
    i0: float  # signed mA towards left
    goal: float
    tau: float
    aid: bool
    peak: int
    width: int

    def current(self, time):
        dt = max(0., min(time, self.end) - self.start) / 1000
        on = self.goal + (self.i0 - self.goal) * math.exp(-dt / self.tau)
        return on * math.exp(-max(0., time - self.end) / (1000 * FALL))


class Recording:
    def __init__(self, folder, expected_run_id=None):
        run_id = json.loads((folder / 'header.json').read_text(encoding='utf8'))['run_start_us']
        if expected_run_id is None and folder.name.isdecimal():
            expected_run_id = int(folder.name)
        if expected_run_id is not None and run_id != expected_run_id:
            raise ValueError(f'Wrong recording in folder {folder}')
        self.run = run_id
        meta = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
        if (meta.get('metadata_event_detail_truncated') or meta.get('energy_control_autonomous_event_overflow')
                or meta['walking_energy']['truncated']):
            raise ValueError(f'Incomplete command history: {self.run}')
        self.energy = next(c for c in meta['walking_energy']['passive_coefficients'] if c['side']==-1)
        rows = [r for r in read_rows(folder, 'timeseries') if r['state_id'] == '3' and
                r['pitch_mekf_detector_relative_deg'] and r['physical_roll_rate_dps']]
        self.t = np.array([int(r['t_test_ms']) for r in rows])
        self.x = np.array([float(r['pitch_mekf_detector_relative_deg']) for r in rows])
        self.v = np.array([-float(r['physical_roll_rate_dps']) for r in rows])
        # The synchronized ZERO event reports the controller's scaled, MEKF-bias-
        # corrected rate. Keep the correction needed to score against self.v.
        self.gyro_scale = meta.get('mekf_gyro_y_scale')
        self.bias_difference = np.array([
            float(r['mekf_bias_y_dps'])-float(r['gyro_bias_y_dps'])
            if r.get('mekf_bias_y_dps') and r.get('gyro_bias_y_dps') else math.nan for r in rows])
        self.left_zeros = {e['previous_peak_time_ms']:e for e in meta['energy_control_autonomous_zero_cross_events']
                           if e['event_kind']=='energy_control_zero_cross' and e['physical_next_peak_side']==-1}
        if np.any(np.diff(self.t)<0):
            raise ValueError(f'Nonmonotone IMU samples: {self.run}')
        self.feet = sorted(read_rows(folder, 'foot_periodic'),key=lambda f:int(f['publish_us']))
        self.published = [int(f['publish_us']) for f in self.feet]
        self.peaks = meta['energy_control_autonomous_peak_events']
        self.records = [r for r in meta['walking_energy']['records'] if r['executed']]
        ends = {r['output_start_ms']: r['output_end_ms'] for r in self.records}
        self.pulses = []
        for e in meta['energy_control_autonomous_zero_cross_events']:
            if not e['output_executed']:
                continue
            sign = -e['q_command_direction']  # + means assistance towards left
            start = e['pulse_start_ms']
            self.pulses.append(Pulse(e['zero_cross_time_ms'], start, ends.get(start, e['pulse_end_ms']),
                                     -sign, sign * (e['aligned_initial_current_mA'] or 0.),
                                     sign * e['predicted_current_goal_mA'] if e['predicted_current_goal_mA'] else math.nan,
                                     e['predicted_current_rise_tau_s'] or math.nan,
                                     e['event_kind'] == 'inward_stall_rescue', e['previous_peak_time_ms'],
                                     e['pulse_width_ms']))
        self.pulses.sort(key=lambda p: p.start)
        self.starts = [p.start for p in self.pulses]
        self.aids = [p for p in self.pulses if p.aid and p.side == -1]
        self.segments = []
        for n, p in enumerate(self.peaks):
            if p['physical_peak_side'] != 1:
                continue
            start = int(np.searchsorted(self.t, p['peak_time_ms']))
            next_t = self.peaks[n+1]['peak_time_ms'] if n+1 < len(self.peaks) else self.t[-1]
            end = int(np.searchsorted(self.t, min(next_t, p['peak_time_ms'] + 1000), side='right'))
            if end <= start + 1:
                continue
            outcome, last, *_ = first_outcome(self.t,self.x,self.v,start,end,None)
            self.segments.append((p,start,last,outcome))

    def current(self, t):
        i = bisect.bisect_right(self.starts, t) - 1
        if i < 0 or t - self.pulses[i].end > 500:
            return 0.
        return self.pulses[i].current(t)

    def observation(self, t, p):
        i = bisect.bisect_right(self.t,t) - 1
        j = bisect.bisect_right(self.t,t-STEP) - 1
        if j < 0 or self.t[i] - self.t[j] <= 0 or self.t[j] < t-STEP-15 or t-self.t[i]>12:
            return None
        left, right, *_ = foot_at(self.feet,self.published,t)
        if not math.isfinite(left-right):
            return None
        a = 1000*(self.v[i]-self.v[j])/(self.t[i]-self.t[j])
        return State(self.x[i],self.v[i],a,left-right,p['peak_amplitude_deg'])

    def reserve(self, t, peak):
        past = [r for r in self.records if r['output_end_ms'] <= t]
        normal = [r for r in past if r['side']==-1 and not r['return_event'] and
                  r['completion_reason']==1 and r['completion_valid']]
        assisted = [r['actual_elapsed_ms'] for r in normal if r['assisted_cycle']]
        reserve = min(80,max(assisted or [r['actual_elapsed_ms'] for r in normal] or [80]))
        # Same-key history only; output duration (including jitter) is charged.
        spent = sum(r['actual_elapsed_ms'] for r in past if r['peak_ms']==peak and r['side']==-1)
        return reserve,max(0,80-spent)


@dataclass(frozen=True)
class State:
    angle: float
    speed: float
    acceleration: float
    foot_difference: float
    peak_amplitude: float

    def after_slot(self, angle, speed, acceleration):
        return replace(self, angle=angle, speed=speed, acceleration=acceleration)


def mean_current(fn, t, duration=STEP):
    # Midpoints give a stable integral when a short pulse straddles a 25 ms slot.
    return sum(fn(t + (i+.5)*duration/10) for i in range(10)) / 10


def features(s, previous_ma, next_ma):
    return [s.angle, min(2.,s.angle), s.speed, s.acceleration,
            s.foot_difference, s.peak_amplitude, previous_ma, next_ma]


def samples(rec):
    result = []
    for p, start, end, _ in rec.segments:
        t = max(rec.t[start]+STEP,p['peak_telemetry_capture_time_ms'])
        while t + STEP <= rec.t[end]:
            s = rec.observation(t,p)
            ids = np.searchsorted(rec.t,[t,t+STEP],side='right')-1
            dt = rec.t[ids[1]]-rec.t[ids[0]]
            if (s and s.angle>0 and s.speed>0 and abs(dt-STEP)<=6 and
                    np.max(np.diff(rec.t[ids[0]:ids[1]+1]))<=15):
                row = features(s,mean_current(rec.current,t-STEP),mean_current(rec.current,t))
                if np.isfinite(row).all():
                    result.append((row,1000*(rec.v[ids[1]]-s.speed)/dt))
            t += STEP
    return result


class Fit:
    def __init__(self, groups, with_current=True, feature_fn=features):
        # Every recording has equal aggregate weight. Fixed regularization and
        # scales fitted on training runs only. Never fit on the two recent runs.
        self.feature_fn = feature_fn
        xs,ys,ws = [],[],[]
        for g in groups:
            if g:
                xs.extend(x for x,_ in g)
                ys.extend(y for _,y in g)
                ws.extend([1/(len(groups)*len(g))]*len(g))
        x,y,w = np.array(xs),np.array(ys),np.array(ws)
        if not with_current:
            x = x[:,:-2]
        self.mu = np.average(x,axis=0,weights=w)
        self.scale = np.maximum(.001,np.sqrt(np.average((x-self.mu)**2,axis=0,weights=w)))
        z = (x-self.mu)/self.scale
        self.intercept = np.average(y,weights=w)
        self.beta = np.linalg.solve(z.T@(z*w[:,None])+.02*np.eye(z.shape[1]),z.T@(w*(y-self.intercept)))

    def acceleration(self,s,previous_ma,next_ma):
        row = np.array(self.feature_fn(s,previous_ma,next_ma))[:len(self.beta)]
        return self.intercept + (row-self.mu)/self.scale @ self.beta


def advance(model,state,t,current,old_current,terminal):
    """Roll to next requested angular section, reversal, or local validity limit."""
    when, charge, s = 0., 0., state
    x, v = s.angle, s.speed
    status = 'horizon'
    for slot in range(0,LIMIT,STEP):
        # The regression is affine in its last current feature. Integrate the
        # varying current in 1 ms bins so a later command cannot push an earlier
        # section crossing. Refresh body state and acceleration every fitted slot.
        v0, q0 = v, charge
        for offset in range(STEP):
            ma = current(t+slot+offset+.5)
            a = model.acceleration(s,old_current,ma)
            dt = min(1.,-1000*v/a) if a<0 else 1.
            ds = dt*.001
            if x-v*ds-.5*a*ds**2 <= terminal:
                terminal_v = math.sqrt(max(0,v*v+2*a*(x-terminal)))
                dt = 2000*(x-terminal)/max(1e-12,v+terminal_v)
                status = 'section'
            elif dt<1. or v+a*.001<=0:
                status = 'reversed'
            ds = dt*.001
            x -= v*ds+.5*a*ds**2
            v = max(0.,v+a*ds)
            charge += ma*ds
            when += dt
            if status!='horizon':
                break
        if status!='horizon':
            break
        s = s.after_slot(x,v,(v-v0)*1000/STEP)
        old_current = (charge-q0)*1000/STEP
    return dict(outcome=status,time_ms=round(when,1),angle_deg=round(x,3),
                speed_dps=round(v,2),signed_charge_mas=round(charge,3))


def load(root,run_ids=(*PRIOR,*RECENT)):
    return {i:Recording(Path(root)/str(i)) for i in dict.fromkeys(run_ids)}


def action_examples(recs):
    examples = []
    for r in recs:
        pmap = {p['peak_time_ms']:p for p in r.peaks}
        for a in r.aids:
            s = r.observation(a.decision,pmap[a.peak])
            if s and np.isfinite([a.goal,a.tau]).all() and a.tau>0:
                examples.append((r.run,s,a))
    return examples


def first_return_status(rec,peak_time,t):
    """Check only measurements already available, including reversal confirmation."""
    start = bisect.bisect_left(rec.t,peak_time)
    end = bisect.bisect_right(rec.t,t)
    if start>=end or rec.t[start]-peak_time>15:
        return 'unknown_return_history'
    outcome,_,_,_,confirmed = first_outcome(rec.t,rec.x,rec.v,start,end,None)
    if outcome in ('crossed','reversed'):
        return 'outside_first_inward_return'
    if confirmed<rec.t[end-1]:
        return 'unknown_return_history'
    return 'ok'


def compare(rec,t,model,examples,widths,delays,section_deg=None):
    """Replace future commands at t. A repeat aid and unobserved guards are flagged."""
    out = {'run':rec.run,'decision_ms':int(t)}
    # Replay the commands already issued and their completions already observed.
    # Between a decision and its completion, future start/end times and elapsed
    # budget are unknown. Do not stage an alternative across the commitment.
    if any(a.decision<t<a.end for a in rec.pulses):
        return dict(out,status='previous_command_pending_or_active')
    peaks = [p for p in rec.peaks if p['peak_telemetry_capture_time_ms']<=t]
    if (not peaks or peaks[-1]['physical_peak_side']!=1 or not rec.t[0]<=t<=rec.t[-1] or
            t-peaks[-1]['peak_time_ms']>1000):
        return dict(out,status='outside_positive_return')
    p = peaks[-1]
    phase = first_return_status(rec,p['peak_time_ms'],t)
    if phase!='ok':
        return dict(out,status=phase)
    s = rec.observation(t,p)
    if not s or s.angle<=0 or s.speed<=0:
        return dict(out,status='no_fresh_inward_state')
    requested = max(0.,s.angle-1.91) if section_deg is None else section_deg
    if not math.isfinite(requested) or not -15.<=requested<s.angle:
        return dict(out,status='section_out_of_range')
    # The first-return model is fitted on the positive side only. The next
    # support's CAD edge may lie on the negative side; do not extrapolate there.
    terminal = max(0.,requested)
    if not examples:
        return dict(out,status='no_training_aid')
    logged = next((a for a in rec.aids if a.decision==t),None)
    past = [a for a in rec.pulses if a.decision<t]
    starts = [a.start for a in past]
    last = past[-1] if past else None
    # All previous decisions have a completed pulse at this point.
    def base(at):
        i = bisect.bisect_right(starts,at)-1
        if i<0 or at-past[i].end>500:
            return 0.
        return past[i].current(at)
    old_ma = mean_current(base,t-STEP)
    if not np.isfinite([base(t),old_ma]).all():
        return dict(out,status='unknown_preceding_current')
    # At a logged decision use its pre-input electrical telemetry. Else use the
    # training median, as the motor command has not been solved at this state.
    goal = logged.goal if logged else float(np.median([a.goal for _,_,a in examples]))
    tau = logged.tau if logged else float(np.median([a.tau for _,_,a in examples]))
    reserve,remaining = rec.reserve(t,p['peak_time_ms'])
    prior_aid = [a for a in past if a.peak==p['peak_time_ms'] and a.aid]
    # Support ranges are descriptive calipers, not causal propensity matching.
    near = [(r,a) for r,state,a in examples if abs(state.angle-s.angle)<=1. and
            abs(state.speed-s.speed)<=5. and abs(state.foot_difference-s.foot_difference)<=2.]
    out.update(status='ok',angle_deg=round(s.angle,3),speed_dps=round(s.speed,2),
               left_minus_right_deg=round(s.foot_difference,2),target_section_deg=round(terminal,3),
               peak_ms=p['peak_time_ms'],reserve_ms=reserve,remaining_ms=remaining,
               prior_aid_start_ms=prior_aid[-1].start if prior_aid else None,
               since_last_command_end_ms=int(t-last.end) if last else None,
               goal_ma=round(goal,2),rise_tau_ms=round(1000*tau,2),matched_prior_actions=len(near),
               matched_prior_runs=len({r for r,_ in near}),
               logged_width_ms=logged.width if logged else None,scenarios=[],
               observed_followup=observed(rec,t,terminal))
    if requested < 0.:
        out.update(requested_section_deg=requested, prediction_limit='mechanical_zero',
                   receiving_side_prediction_available=False)
    for delay in delays:
        for width in widths:
            if width<0 or delay<0 or width>80 or delay>LIMIT:
                raise ValueError('width must be 0..80 ms; delay must be 0..150 ms')
            if width==0 and delay!=delays[0]:
                continue
            at = t+delay
            i0 = logged.i0 if logged and delay==(logged.start-t) else base(at)
            pulse = Pulse(t,at,at+width,-1,i0,goal,tau,True,p['peak_time_ms'],width)
            def current(time):
                return pulse.current(time) if width>0 and time>=at else base(time)
            prediction = advance(model,s,t,current,old_ma,terminal)
            if requested < 0.:
                prediction['requested_section_outcome'] = 'beyond_trained_domain'
            rate = prediction['speed_dps']
            prediction['effective_rate_energy_mj'] = round(1000*rec.energy['omega2_j']*math.radians(rate)**2,5)
            prediction['within_rate_energy_calibration'] = bool(rec.energy['rate_min_dps']<=rate<=rec.energy['rate_max_dps'])
            issues = []
            if width:
                if prior_aid:
                    issues.append('repeat_aid_same_half_cycle')
                if width>remaining-reserve:
                    issues.append('insufficient_main_reserve')
                if not logged or width>logged.width or delay!=logged.start-t:
                    issues.append('wheel_guard_at_new_action_unverified')
            coverage = sum(abs(a.width-width)<=5 and abs(a.start-a.decision-delay)<=5 for _,a in near) if width else None
            out['scenarios'].append(dict(delay_ms=int(delay),width_ms=int(width),matched_action_count=coverage,
                                         constraints=issues,**prediction))
    return out


def simulate_at(recordings_root,run_id,time_ms,widths=(0,10,20),delays=(0,25),section_deg=None):
    """Compare leftward pulse/coast from causal state; section_deg=0 targets ZERO."""
    run_id = int(run_id)
    recs = load(recordings_root,(*PRIOR,run_id))
    train = [recs[i] for i in PRIOR if i!=run_id]
    return compare(recs[run_id],int(time_ms),Fit([samples(r) for r in train]),
                   action_examples(train),tuple(widths),tuple(delays),section_deg)


def observed(rec,t,terminal):
    """Score a future endpoint from t; never bridge a >15 ms gap or another input."""
    i = bisect.bisect_right(rec.t,t)-1
    if i<0 or i+1>=len(rec.t):
        return {'outcome':'censored','reason':'outside_recording'}
    other_start = min((p.start for p in rec.pulses if p.decision>t),default=math.inf)
    until = min(t+LIMIT,other_start)
    time, x, v = t,rec.x[i],rec.v[i]
    if rec.t[i]<t:
        # Retrospective label only. The rollout itself sees only samples at or before t.
        gap = rec.t[i+1]-rec.t[i]
        if gap>15 or rec.t[i+1]>=other_start:
            return {'outcome':'censored','reason':'missing_start_bracket'}
        f = (t-rec.t[i])/gap
        x += f*(rec.x[i+1]-x)
        v += f*(rec.v[i+1]-v)
    if x<=terminal:
        return {'outcome':'censored','reason':'section_already_passed'}
    lowest_time,lowest_x = time,x
    for j in range(i+1,len(rec.t)):
        sample_time = rec.t[j]
        if sample_time==time:
            continue
        if sample_time>=other_start:
            return {'outcome':'censored','reason':'next_command'}
        if sample_time-rec.t[j-1]>15:
            return {'outcome':'censored','reason':'imu_gap'}
        next_time = min(sample_time,until)
        f=(next_time-time)/(sample_time-time)
        next_x,next_v = x+f*(rec.x[j]-x),v+f*(rec.v[j]-v)
        if next_x<=terminal<x:
            f=(x-terminal)/(x-next_x)
            return dict(outcome='section',angle_deg=round(terminal,3),speed_dps=round(v+f*(next_v-v),2),
                        time_ms=round(time+f*(next_time-time)-t,1))
        if next_x<lowest_x:
            lowest_time,lowest_x = next_time,next_x
        elif next_x-lowest_x>.2 and next_v<0:
            return dict(outcome='reversed',angle_deg=round(lowest_x,3),speed_dps=0.,time_ms=int(lowest_time-t))
        if next_time==until:
            return dict(outcome='horizon',angle_deg=round(next_x,3),speed_dps=round(next_v,2),time_ms=LIMIT)
        time,x,v = next_time,next_x,next_v
    return {'outcome':'censored','reason':'end_of_recording'}


def metrics(rows):
    scored = [r for r in rows if r['observation']['outcome']!='censored']
    same = [r for r in scored if r['prediction']['outcome']==r['observation']['outcome']]
    error = [abs(r['prediction']['speed_dps']-r['observation']['speed_dps']) for r in same]
    time_error = [abs(r['prediction']['time_ms']-r['observation']['time_ms']) for r in same
                  if r['prediction']['outcome']!='horizon']
    return dict(scored=len(scored),same_endpoint=len(same),
                matched_speed_mae_dps=round(float(np.mean(error)),2) if error else None,
                matched_event_time_mae_ms=round(float(np.mean(time_error)),2) if time_error else None)


def run(recordings_root):
    recs = load(recordings_root)
    training = {i:samples(recs[i]) for i in PRIOR}
    checks = {k:[] for k in ('cross_validation','holdout','cv_no_current','holdout_no_current')}
    examples, recent = action_examples([recs[i] for i in PRIOR]), []
    for run_id,rec in recs.items():
        train = [g for i,g in training.items() if i!=run_id]
        key = 'cross_validation' if run_id in PRIOR else 'holdout'
        support = [a for a in examples if a[0]!=run_id]
        for use_current in (True,False):
            model = Fit(train,use_current)
            for a in rec.aids:
                delay = a.start-a.decision
                result = compare(rec,a.decision,model,support,(0,a.width),tuple(dict.fromkeys((delay,delay+25))))
                if result['status']!='ok':
                    continue
                actual = result['observed_followup']
                factual = next(s for s in result['scenarios'] if s['width_ms']==a.width and s['delay_ms']==delay)
                name = key if use_current else ('cv_no_current' if run_id in PRIOR else 'holdout_no_current')
                checks[name].append(dict(run=run_id,decision_ms=a.decision,observation=actual,prediction=factual))
                if use_current and run_id in RECENT:
                    recent.append(dict(result,observation=actual))
    return {'step_ms':STEP,'local_limit_ms':LIMIT,'action_direction':'left, from a positive body peak',
            'prior_25ms_rows':{str(i):len(g) for i,g in training.items()},
            'prior_logged_aids':sum(len(recs[i].aids) for i in PRIOR),
            'prior_fresh_aids':len(examples),'prior_width_ms':[min(a.width for _,_,a in examples),max(a.width for _,_,a in examples)],
            'prior_trigger_to_start_ms':[min(a.start-a.decision for _,_,a in examples),max(a.start-a.decision for _,_,a in examples)],
            'heldout_logged_aids':{str(i):len(recs[i].aids) for i in RECENT},
            'metrics':{k:metrics(v) for k,v in checks.items()},'recent_cases':recent,
            'counterfactual_gain_identified':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings_root',type=Path)
    parser.add_argument('--run',type=int,help='any compatible unpacked run ID alongside the eight training recordings')
    parser.add_argument('--time-ms',type=int)
    parser.add_argument('--widths',nargs='+',type=int,default=[0,10,20])
    parser.add_argument('--delays',nargs='+',type=int,default=[0,25])
    parser.add_argument('--section-deg',type=float,help='terminal body angle; defaults to 1.91 degrees travel or ZERO')
    args=parser.parse_args()
    if (args.run is None)!=(args.time_ms is None):
        parser.error('--run and --time-ms must be supplied together')
    if args.run is None:
        output=run(args.recordings_root)
    else:
        output=simulate_at(args.recordings_root,args.run,args.time_ms,args.widths,args.delays,args.section_deg)
    print(json.dumps(output,indent=2,allow_nan=False))
