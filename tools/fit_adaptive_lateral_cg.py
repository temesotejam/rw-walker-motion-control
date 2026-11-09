#!/usr/bin/env python3
"""Model-relative odd potential fit; never identifies unique physical CG."""
import csv,json,math
from pathlib import Path

def potential(angle):
    a=abs(math.radians(angle));h=.120;r=.150;x=.005
    c=math.sqrt(r*r-x*x)
    dh=-2*h*math.sin(a/2)**2+x*math.sin(a) if a<=math.asin(x/r) else r-c+2*(c-h)*math.sin(a/2)**2
    return .1997*9.80665*dh*1000

rows=list(csv.DictReader(Path('tools/fixtures/adaptive_cg_124463989.csv').open()))
points=[]
for row in rows:
    if float(row['decision_ms'])<10000: continue
    a=float(row['peak_deg']);side=int(row['side'])
    k=.1997*9.80665*math.sin(side*math.radians(a))
    residual=potential(a)-float(row['raw_prediction_mj'])
    points.append((k,residual))
n=len(points);sk=sum(k for k,r in points);sr=sum(r for k,r in points)
skk=sum(k*k for k,r in points);skr=sum(k*r for k,r in points)
x=-(skr-sk*sr/n)/(skk-sk*sk/n);b=(sr+x*sk)/n
assert n==41 and abs(x-(-.20587803))<.0001 and abs(b-.02262225)<.0001
print(json.dumps({'source':'124463989','n':n,'fit_window_decision_s':[10,30],
    'equivalent_shift_mm':x,'common_bias_mj':b,
    'energy_rmse_mj':math.sqrt(sum((r-(b-k*x))**2 for k,r in points)/n),
    'physical_cg_identified':False,
    'assumption':'shared residual bias plus odd potential; asymmetric loss/reference error remains confounded',
    'use':'prior for next run; fitting and replay on same log are not independent validation',
    'initial_target_plus_mj':potential(8)+.1997*9.80665*math.sin(math.radians(8))*x,
    'initial_target_minus_mj':potential(8)-.1997*9.80665*math.sin(math.radians(8))*x},indent=2))
