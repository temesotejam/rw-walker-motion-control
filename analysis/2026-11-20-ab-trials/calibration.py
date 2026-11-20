"""Fit prior right-going mechanical ZERO arrivals and replay the unchanged firmware fit on two held-out runs."""
import argparse
import csv
import json
import math
import subprocess
import tempfile
from collections import Counter
from itertools import combinations
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
TRAIN = {69501075, 224021399, 69728952, 205922296, 50784949, 52400264, 45820669, 239670034}
HOLD = {33125639, 216664125}
FIELDS = 'run zero_ms rate_dps q_mas previous_peak_deg next_peak_deg original_ms old_predicted_deg target_deg i0_ma goal_ma tau_s'.split()
Q_LIMIT = 13.27138  # positive-side training range of the inherited Q model


def extract(directories):
    rows = []
    seen = set()
    for folder in directories:
        header = json.loads((folder / 'header.json').read_text(encoding='utf8'))
        run = int(header['run_start_us'])
        if run in seen or run not in TRAIN | HOLD:
            raise ValueError(f'Unexpected or duplicate run {run}')
        seen.add(run)
        data = json.loads((folder / 'metadata.json').read_text(encoding='utf8'))
        with (folder / 'walking_energy.csv').open(encoding='utf8', newline='') as f:
            audit = list(csv.DictReader(f))
        events = data['energy_control_autonomous_zero_cross_events']
        peaks = data['energy_control_autonomous_peak_events']
        for event in events:
            if event['event_kind'] != 'energy_control_zero_cross' or event['physical_next_peak_side'] != 1:
                continue
            t, previous = event['zero_cross_time_ms'], event['previous_peak_time_ms']
            peak = next((p for p in peaks if p['peak_time_ms'] > t), None)
            if not peak or peak['physical_peak_side'] != 1 or peak['peak_time_ms'] - t > 750:
                continue
            if any(e is not event and e['output_executed'] and previous < e['zero_cross_time_ms'] < peak['peak_time_ms'] for e in events):
                continue
            matches = [a for a in audit if a['side']=='1' and a['optional']=='False' and a['rescue']=='False'
                       and int(a['peak_ms'])==previous and abs(int(a['time_ms'])-t)<=8]
            if len(matches)!=1:
                raise ValueError(f'Ambiguous energy audit for run {run} time {t}')
            a = matches[0]
            powered = bool(event['output_executed'])
            if powered and (a['planned_duration_completed']!='True' or a['completion_valid']!='True'):
                continue
            rows.append(dict(zip(FIELDS, [run, t, event['zero_cross_abs_rate_dps'], event['q_effective_pred_mA_s'] if powered else 0,
                event['previous_peak_amplitude_deg'], peak['peak_amplitude_deg'], event['pulse_width_ms'] if powered else 0,
                event['predicted_next_peak_amplitude_deg'], event['startup_target_deg'], event['aligned_initial_current_mA'],
                event['predicted_current_goal_mA'],event['predicted_current_rise_tau_s']])))
    if seen != TRAIN | HOLD:
        raise ValueError(f'Missing runs {(TRAIN | HOLD) - seen}')
    return sorted(rows, key=lambda r: (r['run'],r['zero_ms']))


def potential(deg):
    a=abs(math.radians(deg))
    r,h,x=.150,.120,.005
    center=math.sqrt(r*r-x*x)
    s=math.sin(a/2)**2
    dh=-2*h*s+x*math.sin(a) if a<=math.asin(x/r) else r-center+2*(center-h)*s
    return .1997*9.80665*dh


def angle(energy):
    lo,hi=0.,math.degrees(math.asin(.045/.15))
    for _ in range(40):
        mid=(lo+hi)/2
        if potential(mid)<energy: lo=mid
        else: hi=mid
    return (lo+hi)/2


def features(r,with_previous=False):
    w=math.radians(r['rate_dps'])
    f=[1.,w*w,w*r['q_mas']]
    return f+[potential(r['previous_peak_deg'])] if with_previous else f


def fit(rows,with_previous=False):
    # Nonnegative least squares on the small set of active coefficient subsets.
    x=np.array([features(r,with_previous) for r in rows])
    y=np.array([potential(r['next_peak_deg']) for r in rows])
    best=np.zeros(x.shape[1]); best_error=y@y
    for size in range(1,x.shape[1]+1):
        for subset in combinations(range(x.shape[1]),size):
            coefficients=np.linalg.lstsq(x[:,subset],y,rcond=None)[0]
            if min(coefficients)<0: continue
            residual=x[:,subset]@coefficients-y
            if residual@residual<best_error:
                best_error=residual@residual; best=np.zeros(x.shape[1]); best[list(subset)]=coefficients
    return best


def mae(values):
    return sum(abs(v) for v in values)/len(values)


def validate(rows):
    train=[r for r in rows if r['run'] in TRAIN and 0<=r['q_mas']<=Q_LIMIT]
    held=[r for r in rows if r['run'] in HOLD]
    print('Prior runs, usable arrivals, coast',len(TRAIN),len(train),sum(r['q_mas']==0 for r in train))
    print('Prior sample count by run', sorted(Counter(r['run'] for r in train).items()))
    print('Coefficients in joules',fit(train).tolist())
    for extra in [False,True]:
        errors=[]
        for run in sorted(TRAIN):
            c=fit([r for r in train if r['run']!=run],extra)
            errors.extend(angle(c@features(r,extra))-r['next_peak_deg'] for r in train if r['run']==run)
        print('Leave-one-run-out angle MAE, using previous peak as regressor',extra,mae(errors))
    with tempfile.TemporaryDirectory() as directory:
        exe=Path(directory)/'right_arrival_replay.exe'
        subprocess.run(['g++','-std=c++17','-O2','-I',HERE.parents[1]/'src',HERE/'calibration.cpp','-o',exe],check=True)
        fields='rate_dps previous_peak_deg target_deg i0_ma goal_ma tau_s original_ms'.split()
        payload=''.join(' '.join(str(r[k]) for k in fields)+'\n' for r in held)
        result=subprocess.run([exe],input=payload,text=True,capture_output=True,check=True).stdout.splitlines()
    errors=[]; old=[]; powered=[]
    if len(result)!=len(held): raise ValueError('Incomplete firmware replay')
    for r,line in zip(held,result):
        eligible,width,pred,selected=map(float,line.split())
        if not eligible: raise ValueError(f'Held-out row outside firmware support {r}')
        fitted=angle(fit(train)@features(r))
        if abs(fitted-pred)>.003: raise ValueError('Firmware coefficients or charge reconstruction diverged from extracted data')
        errors.append(pred-r['next_peak_deg']);old.append(r['old_predicted_deg']-r['next_peak_deg']);powered.append(r['original_ms']>0)
        d=width/1000; q=r['goal_ma']*d+(r['i0_ma']-r['goal_ma'])*r['tau_s']*(-math.expm1(-d/r['tau_s']))
        neighbors=sum(x['q_mas']>0 and abs(x['rate_dps']-r['rate_dps'])<=3 and abs(x['q_mas']-q)<=1 for x in train)
        print(int(r['run']),int(r['zero_ms']),'ms',int(r['original_ms']),int(width),'measured deg',r['next_peak_deg'],
              'original models legacy/fit deg',round(r['old_predicted_deg'],3),round(pred,3),'candidate model deg',round(selected,3),
              'candidate Q / powered neighbors',round(q,3),neighbors)
    print('Held-out original-command angle MAE, legacy / fitted',mae(old),mae(errors))
    for is_powered in [True,False]:
        print('Original command powered',is_powered,'angle MAE, legacy / fitted',
              mae([v for v,p in zip(old,powered) if p==is_powered]),mae([v for v,p in zip(errors,powered) if p==is_powered]))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract',type=Path,nargs='+',help='Regenerate the event table from all ten converted RWLOG directories')
    args=parser.parse_args()
    table=HERE/'right_arrivals.csv'
    if args.extract:
        rows=extract(args.extract)
        with table.open('w',encoding='utf8',newline='') as f:
            writer=csv.DictWriter(f,FIELDS,lineterminator='\n');writer.writeheader();writer.writerows(rows)
    else:
        with table.open(encoding='utf8',newline='') as f:
            rows=[{k:(int(v) if k in ('run','zero_ms','original_ms') else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]
    validate(rows)
