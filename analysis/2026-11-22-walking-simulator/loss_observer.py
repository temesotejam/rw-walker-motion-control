"""Replay an IMU-based innovation correction on completed left aids and coasting returns."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

import action_model as action
from return_model import load_sections


def innovation(rec, peak, decision, model, prior_current):
    """Compare a one-slot velocity forecast with the last available gyro reading.

    The comparison uses the modeled current for the interval that has already
    elapsed. The new command, if any, therefore cannot create its own residual.
    """
    end = np.searchsorted(rec.t, decision, side='right') - 1
    if end < 1 or decision - rec.t[end] > 12:
        return None
    target = rec.t[end] - action.STEP
    right = np.searchsorted(rec.t, target)
    start = min((i for i in (right-1, right) if i >= 0), key=lambda i: abs(rec.t[i]-target))
    start_ms, end_ms = int(rec.t[start]), int(rec.t[end])
    dt = end_ms - start_ms
    if (abs(dt - action.STEP) > 6 or start_ms < peak['peak_telemetry_capture_time_ms']
            or np.max(np.diff(rec.t[start:end+1])) > 15):
        return None
    s = rec.observation(start_ms, peak)
    if not s or s.angle <= 0 or s.speed <= 0 or rec.v[end] <= 0:
        return None
    prev = action.mean_current(prior_current, start_ms-action.STEP)
    cur = action.mean_current(prior_current, start_ms, dt)
    a = float(model.acceleration(s, prev, cur))
    predicted = float(s.speed + dt*.001*a)
    measured = float(rec.v[end])
    if predicted <= 0 or not math.isfinite(predicted):
        return None
    return {'from_ms':start_ms, 'to_ms':end_ms, 'measured_speed_change_dps':float(measured-s.speed),
            'speed_error_dps':measured-predicted,
            'acceleration_error_dps2':(measured-predicted)*1000/dt,
            'energy_error_per_inertia_s2':.5*(math.radians(measured)**2-math.radians(predicted)**2)}


class Corrected:
    """Persist an observed error in modeled acceleration, halving its weight every slot."""
    def __init__(self, model, initial_elapsed, error):
        self.model = model
        self.elapsed = initial_elapsed
        self.error = error

    def acceleration(self, s, previous, future):
        bias = self.error * 2**(-(s.elapsed_ms - self.elapsed)/action.STEP)
        return self.model.acceleration(s, previous, future) + bias


def persistent_loss(previous, current):
    """Require two deficient slots and actual slowing in the latest one."""
    if not previous or not current or current['measured_speed_change_dps'] >= 0:
        return 0.
    return min(0., max(previous['acceleration_error_dps2'], current['acceleration_error_dps2']))


def loss_history(model, rec, peak, decision, prior_current):
    latest = innovation(rec, peak, decision, model, prior_current)
    previous = innovation(rec, peak, latest['from_ms'], model, prior_current) if latest else None
    loss = persistent_loss(previous, latest)
    return dict(previous=previous, latest=latest, correction_dps2=loss)


def coast_forecast(model, rec, peak, decision, prior_current):
    # Extending the correction to the earlier +4.5 degree section introduced
    # a false reversal in reference data. Evaluate the normal return model there.
    i = np.searchsorted(rec.t, decision, side='right')-1
    if i < 0 or not 0 < rec.x[i] <= 3.:
        return model, dict(previous=None,latest=None,correction_dps2=0.,in_validated_angle_range=False)
    details = loss_history(model, rec, peak, decision, prior_current)
    details['in_validated_angle_range'] = True
    loss = details['correction_dps2']
    if loss == 0:
        return model, details
    return Corrected(model, decision-peak['peak_time_ms'], loss), details


def assess(rec, group, t, width, delay, coast, model, examples):
    import phase_dynamics as phase
    target = 0. if coast else None
    factual = action.compare(rec,t,model,examples,(width,),(delay,),target)
    if factual['status'] != 'ok':
        return None
    peak = next(p for p in rec.peaks if p['peak_time_ms']==factual['peak_ms'])
    # All intervals used here have ended at or before the origin. A concurrently
    # issued command cannot contribute to any of the preceding midpoint samples.
    audit = loss_history(model,rec,peak,t,rec.current)
    delta, prev = audit['latest'], audit['previous']
    def apply(error):
        if not error:
            return factual['scenarios'][0]
        fit = Corrected(model,t-peak['peak_time_ms'],error)
        return action.compare(rec,t,fit,examples,(width,),(delay,),target)['scenarios'][0]
    loss = audit['correction_dps2']
    actual = phase.coast_endpoint(rec,coast,factual['observed_followup']) if coast else factual['observed_followup']
    if actual['outcome']=='censored':
        return None
    return dict(run=rec.run, group=group, time_ms=t, peak_ms=peak['peak_time_ms'],
                preceding_aid_ms=factual['prior_aid_start_ms'], innovation=delta,
                previous_innovation=prev, persistent_loss_dps2=loss, actual=actual,
                base=factual['scenarios'][0], corrected=apply(delta['acceleration_error_dps2'] if delta else 0.),
                persistent=apply(loss))


def summarize(rows):
    result = {'cases':len(rows), 'with_innovation':sum(r['innovation'] is not None for r in rows),
              'with_persistent_loss':sum(r['persistent_loss_dps2']<0 for r in rows)}
    for method in ('base','corrected','persistent'):
        pairs = [(r[method],r['actual']) for r in rows]
        result[method] = dict(
            speed_mae_dps=round(float(np.mean([abs(a['speed_dps']-b['speed_dps']) for a,b in pairs])),3)
            if pairs else None,
            endpoint_counts=dict(Counter(b['outcome']+'_as_'+a['outcome'] for a,b in pairs)))
    return result


def run(root, section=3.):
    import phase_dynamics as phase
    recs, _, windows = phase.reference_rows(root)
    recs.update({i:phase.PhaseRecording(Path(root)/str(i)) for i in action.RECENT})
    result = {}
    for split, ids in [('reference',action.PRIOR),('later',action.RECENT)]:
        rows = []
        for i in ids:
            train = [j for j in action.PRIOR if j != i]
            model = phase.PhaseFit([windows[j] for j in train],'body')
            examples = action.action_examples([recs[j] for j in train])
            rec = recs[i]
            aids = [('aid',p.decision,p.width,p.start-p.decision,None) for p in rec.aids]
            coasts = [('coast',obs.state.time_ms,0,0,obs)
                      for obs in load_sections(rec.folder,sections=(section,))[0]]
            for group,t,width,delay,coast in aids + coasts:
                if coast and any(p.decision==t for p in recs[i].pulses):
                    continue
                r = assess(recs[i],group,t,width,delay,coast,model,examples)
                if r:
                    rows.append(r)
        result[split] = {'coast_section_deg':section, 'rows':rows, 'groups':{g:summarize([r for r in rows if r['group']==g])
                                                for g in ('aid','coast')}}
    return result


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings_root',type=Path)
    parser.add_argument('--section-deg', type=float, default=3., help='Inward crossing for the coasting score')
    parser.add_argument('--json-out',type=Path)
    args = parser.parse_args()
    results = run(args.recordings_root, args.section_deg)
    if args.json_out:
        args.json_out.write_text(json.dumps(results,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v['groups'] for k,v in results.items()},indent=2))
