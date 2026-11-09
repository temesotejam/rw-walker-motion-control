"""Reproduce the high-voltage coast calibration without changing pulse-effect fits.

The original 31 runs remain frozen. Fit a single energy offset above 8.00 V.
Temporal validation fits 10-20 s and tests the untouched 20-30 s block first.
The final offset then includes all 22 coast events. Requires numpy via the
original fitting module; never supplies measured future peaks to runtime.
"""
import csv
import json
import math
from pathlib import Path
import numpy as np
from fit_broad_negative_work import potential, angle
from fit_pooled_negative_work import parts, metrics

ROOT = Path(__file__).resolve().parent.parent

def weight(row):
    return max(0., min(1., (float(row['v'])-8000.)/100.))

def main():
    baseline = json.loads((ROOT/'docs/pooled_negative_31runs_validation.json').read_text())
    model = {key:np.array(value) for key,value in baseline['final_coefficients'].items()}
    with (ROOT/'tools/fixtures/full_charge_negative_coast_69890231.csv').open(newline='') as f:
        rows = list(csv.DictReader(f))
    assert len(rows)==22 and all(float(r['ms'])==0 for r in rows)
    train=[r for r in rows if float(r['time_ms'])<20000]
    hold=[r for r in rows if float(r['time_ms'])>=20000]
    assert len(train)==11 and len(hold)==11
    def fit(data):
        return sum(weight(r)*(potential(float(r['peak']))-parts(r,model)[4]) for r in data)/sum(weight(r)**2 for r in data)
    def score(data,bias):
        errors=[angle(parts(r,model)[4]+weight(r)*bias)-float(r['peak']) for r in data]
        return {'n':len(errors),'rmse_deg':math.sqrt(sum(x*x for x in errors)/len(errors)),
                'bias_deg':sum(errors)/len(errors)}
    causal_coef=fit(train)
    coefficient=fit(rows)
    report={'original_fit_runs':31,'new_calibration_runs':1,'new_coast_events':22,
        'formula':'E_free_new = E_free_old + coefficient_mJ * clamp((battery_mV-8000)/100,0,1)',
        'coefficient_mj':coefficient,
        'temporal_validation':{'train_ms':[10000,20000],'test_ms':[20000,30000],
            'coefficient_from_train_only_mj':causal_coef,'before':score(hold,0),
            'after':score(hold,causal_coef)},
        'final_training_error':score(rows,coefficient),'before_all':score(rows,0),
        'runtime_voltage_range_mv':[6210,8120],
        'scope':'One-run no-input calibration only. All pulse predictions above 8000 mV are cross-condition; 8120 mV is a 10 mV margin above observed whole-run maximum 8110 mV. No high-voltage braking validation or claim of reaching 8 degrees.'}
    with (ROOT/'tools/fixtures/pooled_negative_31runs.csv').open(newline='') as f:
        old=list(csv.DictReader(f))
    assert all(weight(r)==0 for r in old)
    report['old_fit_predictions_unchanged']=True
    # All finite, supported coast states now admit every candidate, and the
    # calibration cannot change the original duration-response monotonicity.
    for row in rows:
        es=[parts(dict(row,ms=t),model)[4]+weight(row)*coefficient for t in [0]+list(range(10,41))]
        assert all(a>=b for a,b in zip(es,es[1:]))
    assert report['temporal_validation']['after']['rmse_deg']<.20
    assert report['final_training_error']['rmse_deg']<.26
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
