"""Fit positive pulse/coast work and effective arrival energy on 8 recorded runs.
Offline labels contain observed work and peaks; runtime features use only
pre-input state and the candidate's predicted current integral.
"""
import csv,json,math
from pathlib import Path
import numpy as np
from fit_broad_negative_work import potential,angle
ROOT=Path(__file__).resolve().parents[1]
def load():
    with (ROOT/'tools/fixtures/positive_work_8runs.csv').open(newline='') as f:
        return [{k:(v if k=='run' else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]
def features(r):
    return np.array([1,r['q']*r['rate']*math.pi/180,r['rpm']/500,(r['v']-7500)/1000])
def fit(rows):
    x=np.array([features(r) for r in rows])
    work=np.linalg.lstsq(x,np.array([r['work'] for r in rows]),rcond=None)[0]
    body=np.linalg.lstsq(np.array([[1,(r['rate']*math.pi/180)**2,float(features(r)@work)] for r in rows]),np.array([potential(r['peak']) for r in rows]),rcond=None)[0]
    on=sum(features(r)[1]*r['on'] for r in rows)/sum(features(r)[1]**2 for r in rows)
    assert work[1]>0 and body[1]>0 and body[2]>0
    return work,body,on
def predict(r,model):
    work,body,_=model
    return body[0]+body[1]*(r['rate']*math.pi/180)**2+body[2]*float(features(r)@work)
def score(rows,model,adaptive=False):
    errors=[];history=[]
    for r in sorted(rows,key=lambda x:x['time']):
        bias=max(-.25,min(.25,sum(history)/len(history))) if adaptive and len(history)>=2 else 0
        pred=predict(r,model)
        errors.append(angle(max(0,pred+bias))-r['peak'])
        history=(history+[potential(r['peak'])-pred])[-3:]
    return {'n':len(errors),'rmse_deg':float(np.sqrt(np.mean(np.square(errors))))}
def main():
    rows=load();assert len(rows)==180 and len({r['run'] for r in rows})==8
    report={'runs':8,'events':180,'leave_one_run_out':{},'adaptive_leave_one_run_out':{}}
    for run in sorted({r['run'] for r in rows}):
        tr=[r for r in rows if r['run']!=run];te=[r for r in rows if r['run']==run];model=fit(tr)
        report['leave_one_run_out'][run]=score(te,model)
        report['adaptive_leave_one_run_out'][run]=score(te,model,True)
    model=fit(rows);report['coefficients']={'work':model[0].tolist(),'arrival':model[1].tolist(),'on_qomega':model[2]}
    report['latest_run_training']=score([r for r in rows if r['run']=='130765421'],model)
    report['support']={k:[min(r[k] for r in rows),max(r[k] for r in rows)] for k in ['rate','rpm','v','q','prev','ma']}
    report['scope']='Effective arrival-energy model under fixed-leg conditions. U_ref is the existing symmetric geometry reference, not a newly measured side-specific physical potential. Work multiplier is empirical, not efficiency. Replaying unchanged inputs is not closed-loop validation.'
    assert report['latest_run_training']['rmse_deg']<.20
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
