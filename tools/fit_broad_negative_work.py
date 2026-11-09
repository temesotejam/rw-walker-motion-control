"""Reproduce the 19-run negative work fit and run-separated validation.

Offline only; requires numpy. No future work or measured peak enters the
features used at runtime. The current no-input run is held out of fitting.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent


def potential(deg):
    a = abs(deg) * math.pi / 180
    r, h, x = .15, .12, .005
    c = math.sqrt(r*r-x*x)
    rise = (-2*h*math.sin(a/2)**2+x*math.sin(a)) if a <= math.asin(x/r) else (
        r-c+2*(c-h)*math.sin(a/2)**2)
    return .1997*9.80665*rise*1000


def angle(energy):
    lo, hi = 0., math.degrees(math.asin(.045/.15))
    if energy < 0 or energy > potential(hi):
        raise ValueError('Predicted energy outside geometry')
    for _ in range(40):
        mid = (lo+hi)/2
        if potential(mid) < energy:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2


def features(r):
    rate = (float(r['rate_dps'])-72.5)/3
    rpm = (float(r['wheel_rpm'])-80)/50
    prev = (float(r['previous_peak_deg'])-8)/.5
    voltage = (float(r['battery_mV'])-7000)/1000
    t = float(r['assigned_width_ms'])/25
    return np.array([t,t*t,t*rate,t*rpm]), np.array([1,rate,rpm,t,t*t]), np.array([1,rate,rpm,prev,voltage])


def work_parts(r, model):
    on, off, _ = features(r)
    after_zero = off.copy()
    after_zero[-2:] = 0
    on_work = float(on @ model['on'])
    off_work = float(off @ model['off'])
    return on_work, off_work, on_work+off_work-float(after_zero @ model['off'])


def fit(rows):
    model = {}
    for name, index, key in [('on',0,'observed_work_on_mj'),('off',1,'observed_work_post_mj')]:
        model[name] = np.linalg.lstsq(np.array([features(r)[index] for r in rows]),
                                      np.array([float(r[key]) for r in rows]),rcond=None)[0]
    x = np.array([list(features(r)[2])+[work_parts(r,model)[2]] for r in rows])
    model['arrival'] = np.linalg.lstsq(x,np.array([potential(float(r['observed_next_peak_deg'])) for r in rows]),rcond=None)[0]
    return model


def predicted_energy(r, model):
    return float(features(r)[2] @ model['arrival'][:5])+model['arrival'][5]*work_parts(r,model)[2]


def metrics(errors):
    a = np.asarray(errors)
    return dict(n=len(a),rmse_deg=float(np.sqrt(np.mean(a*a))),bias_deg=float(np.mean(a)))


def main():
    with (ROOT/'tools/fixtures/broad_negative_19runs.csv').open(encoding='utf-8',newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 412 and len({r['run'] for r in rows}) == 19
    cv, by_run = [], {}
    for run in sorted({r['run'] for r in rows},key=int):
        model = fit([r for r in rows if r['run'] != run])
        errors = [angle(predicted_energy(r,model))-float(r['observed_next_peak_deg']) for r in rows if r['run']==run]
        by_run[run] = metrics(errors)
        cv.extend(errors)
    model = fit(rows)
    hold = json.loads((ROOT/'tools/fixtures/broad_negative_no_input_holdout.json').read_text())
    hold_errors = [angle(predicted_energy(r,model))-float(r['observed_next_peak_deg']) for r in hold]
    report = dict(runs=19,events=len(rows),leave_one_run_out=metrics(cv),
                  no_input_42884626_last5s=metrics(hold_errors),
                  coefficients={k:v.tolist() for k,v in model.items()},by_run=by_run,
                  note='No-input holdout validates only the baseline at the current state; brake effect there is not newly measured.')
    assert report['leave_one_run_out']['rmse_deg'] < .30
    assert report['no_input_42884626_last5s']['rmse_deg'] < .20
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
