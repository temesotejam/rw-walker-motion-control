#!/usr/bin/env python3
"""Fit provisional energy-domain seed coefficients from inherited fixed-foot logs.

These closed-loop logs do not independently identify causal actuator response.
Free-energy seed uses the inherited rate baseline; response uses observed peaks.
No target label is a predictor. All current integrals are MODEL predictions.
Standard-library only; no hardware validation is claimed.
"""
import json, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'tools/fixtures/fixed_foot_20260929.json'
M, G, H, R, X = .1997, 9.80665, .120, .150, .005
def potential(a):
    a = math.radians(abs(a)); center = math.sqrt(R*R-X*X)
    height = H*math.cos(a)+X*math.sin(a) if a <= math.asin(X/R) else R+(H-center)*math.cos(a)
    return M*G*(height-H)
def solve(a,b):
    z=[list(row)+[v] for row,v in zip(a,b)]; n=len(b)
    for k in range(n):
        p=max(range(k,n),key=lambda i:abs(z[i][k]));z[k],z[p]=z[p],z[k]
        if abs(z[k][k])<1e-15: raise ValueError('singular fit')
        q=z[k][k];z[k]=[v/q for v in z[k]]
        for j in range(n):
            if j!=k:
                q=z[j][k];z[j]=[u-q*v for u,v in zip(z[j],z[k])]
    return [row[-1] for row in z]
def least_squares(x,y):
    return solve([[sum(r[i]*r[j] for r in x) for j in range(len(x[0]))] for i in range(len(x[0]))],
                 [sum(r[i]*v for r,v in zip(x,y)) for i in range(len(x[0]))])
def stable(value):
    # Different libm implementations differ by a few ulps. The firmware seed
    # stores ten significant digits, so publish the same precision here.
    return float(format(value,'.10g'))
def fit(rows,side):
    selected=[r for group in rows for r in group if r[0]==side]
    aa,bb=(6.730502640,.187441879) if side>0 else (7.691200794,.194970900)
    omega=[math.radians(r[1]) for r in selected]
    free=[potential(max(0,aa+bb*(r[1]-65))) for r in selected]
    offset,kinetic=least_squares([[1,w*w] for w in omega],free)
    if kinetic<=0: raise ValueError('invalid energy slope')
    y=[potential(r[3])-max(0,offset+kinetic*w*w) for r,w in zip(selected,omega)]
    x=[[w*r[2]/1000,(r[2]/1000)**2] for r,w in zip(selected,omega)]
    # Nonnegative response coefficients: enumerate both active sets.
    possibilities=[]
    for active in ([0],[1],[0,1]):
        try: c=least_squares([[r[i] for i in active] for r in x],y)
        except ValueError: continue
        if min(c)<0: continue
        full=[0.,0.]
        for i,v in zip(active,c):full[i]=v
        error=sum((sum(a*b for a,b in zip(r,full))-v)**2 for r,v in zip(x,y))
        possibilities.append((error,full))
    error,response=min(possibilities)
    return dict(side=side,count=len(selected),free_offset_j=stable(offset),
        free_omega2_j_per_rad2_s2=stable(kinetic),
        work_omega_q_j_per_rad_s_A_s=stable(response[0]),work_q2_j_per_A2_s2=stable(response[1]),
        endpoint_energy_rmse_j=stable(math.sqrt(error/len(selected))),
        rate_support_min_dps=min(r[1] for r in selected),rate_support_max_dps=max(r[1] for r in selected),
        charge_support_min_mA_s=min(r[2] for r in selected),
        charge_support_max_mA_s=max(r[2] for r in selected))
def main():
    data=json.loads(DATA.read_text());groups=data['rows']
    if isinstance(groups[0],dict): groups=[g['events'] if 'events' in g else g['rows'] for g in groups]
    result=dict(revision='fixed_foot_energy_seed_20261013',status='provisional_closed_loop_seed_not_causal_identification',
        source='inherited_fixed_foot_20260929_204_events;predicted_Q_not_measured_current_integral',
        geometry=dict(mass_kg=M,gravity_m_s2=G,cg_height_m=H,radius_m=R,inner_edge_m=X),
        sides=[fit(groups,s) for s in (1,-1)])
    output=ROOT/'docs/lateral_energy_seed.json';output.write_text(
        json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
