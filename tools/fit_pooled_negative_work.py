"""Fit pulse + post-pulse work and arrival energy, with whole-run validation.

Requires numpy. The new 77054774 run is first held out in its entirety; the
final coefficients then include it. Runtime features use only pre-input state.
"""
import csv
import json
import math
from pathlib import Path
import numpy as np
from fit_broad_negative_work import potential, angle

ROOT = Path(__file__).resolve().parent.parent


def features(row):
    t = float(row['ms'])/25
    r = (float(row['rate'])-70)/10
    n = (float(row['rpm'])-200)/200
    v = (float(row['v'])-7300)/500
    p = float(row['prev'])-8
    return (np.array([t,t*t,t*r,t*n,t*v]),
            np.array([1,r,n,v,t,t*t,t*r,t*n]),
            np.array([1,(float(row['rate'])*math.pi/180)**2,n,p,v]))


def parts(row, model):
    on, post, state = features(row)
    zero = post.copy()
    zero[4:] = 0
    won, wpost = float(on @ model['on']), float(post @ model['post'])
    delta = won+wpost-float(zero @ model['post'])
    free = float(state @ model['arrival'][:5])
    energy = free+model['arrival'][5]*delta
    return won,wpost,delta,free,float(energy)


def fit(rows):
    model = {}
    for key, index in [('on',0),('post',1)]:
        x = np.array([features(r)[index] for r in rows])
        coef, _, rank, _ = np.linalg.lstsq(x,np.array([float(r[key]) for r in rows]),rcond=None)
        assert rank == x.shape[1]
        model[key] = coef
    x = []
    for row in rows:
        on, post, state = features(row)
        zero = post.copy(); zero[4:] = 0
        delta = float(on @ model['on']+(post-zero) @ model['post'])
        x.append(list(state)+[delta])
    model['arrival'], _, rank, _ = np.linalg.lstsq(np.array(x),
        np.array([potential(float(r['peak'])) for r in rows]),rcond=None)
    assert rank == 6
    return model


def metrics(rows, model):
    errors = [angle(parts(r,model)[4])-float(r['peak']) for r in rows]
    return dict(n=len(errors),rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
                bias_deg=float(np.mean(errors)))


def main():
    with (ROOT/'tools/fixtures/pooled_negative_31runs.csv').open(encoding='utf-8',newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows)==670 and len({r['run'] for r in rows})==31
    before = fit([r for r in rows if r['run']!='77054774'])
    report = {'runs':31,'events':len(rows),
        'new_run_entirely_heldout':metrics([r for r in rows if r['run']=='77054774'],before)}
    cv, by_run = [], {}
    for run in sorted({r['run'] for r in rows}):
        model = fit([r for r in rows if r['run']!=run])
        test = [r for r in rows if r['run']==run]
        by_run[run] = metrics(test,model)
        cv.extend(angle(parts(r,model)[4])-float(r['peak']) for r in test)
    report['leave_one_run_out'] = dict(n=len(cv),rmse_deg=float(np.sqrt(np.mean(np.square(cv)))))
    report['by_run'] = by_run
    model = fit(rows)
    report['final_coefficients'] = {k:v.tolist() for k,v in model.items()}
    report['new_run_training_error'] = metrics([r for r in rows if r['run']=='77054774'],model)
    report['support'] = {key:[min(float(r[key]) for r in rows),max(float(r[key]) for r in rows)]
                         for key in ['rate','rpm','prev','v','ms']}
    # A physical braking candidate must reduce predicted arrival energy over
    # this command envelope; verify every measured state's entire duration grid.
    for row in rows:
        energies = [parts(dict(row,ms=width),model)[4] for width in [0]+list(range(10,41))]
        assert all(a >= b for a,b in zip(energies,energies[1:]))
    assert report['new_run_entirely_heldout']['rmse_deg'] < .15
    assert report['leave_one_run_out']['rmse_deg'] < .33
    report['limitations'] = ('Strong pulses at the new low-voltage state are cross-condition predictions; '
        'not directly validated. Firmware advances a completed-peak-gated 25/30/35/40ms cap. '
        'Work coefficient is an empirical residual correction, not mechanical efficiency.')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
