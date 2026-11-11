"""Replay the firmware predictor at recorded delivery+processing times.

The next-frame error is a diagnostic using interpolation of future measured
camera angles, not independent angular ground truth or intervention efficacy.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('converted',type=Path);parser.add_argument('--out',type=Path,required=True)
args=parser.parse_args()
with (args.converted/'foot_angles.csv').open(newline='',encoding='utf8') as f:frames=list(csv.DictReader(f))
def number(x):
    try:return float(x)
    except (TypeError,ValueError):return math.nan
def quality(f):
    return all(f[k]=='True' for k in ['timestamp_valid','frame_valid','zero_ready','left_valid','right_valid','left_in_range','right_in_range']) and all(math.isfinite(number(f[k])) for k in ['left_deg','right_deg'])
source=r'''
#include "foot_pose_prediction.h"
#include "cad_support.h"
#include <iostream>
int main(){foot_pose_prediction::Predictor p;unsigned run,t,now,v;float l,r;
 while(std::cin>>run>>t>>now>>v>>l>>r){auto q=p.observe(run,t,now,v,l,r);float a=NAN,b=NAN;
 if(q.ready()){cad_support::switchAngle(q.left,q.right,0,a);cad_support::switchAngle(-q.left,-q.right,0,b);}
 std::cout<<int(q.reason)<<' '<<q.left<<' '<<q.right<<' '<<q.horizon_ms<<' '<<a<<' '<<b<<'\n';}}
'''
with tempfile.TemporaryDirectory(prefix='cad_replay_') as td:
    cpp=Path(td)/'replay.cpp';exe=Path(td)/'replay.exe';cpp.write_text(source)
    subprocess.run(['g++','-std=c++11','-O2','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    lines=[]
    epoch=int(frames[0]['frame_us'])-int(frames[0]['log_time_us'])
    # Recording-relative times: differences match measurement-relative clocks.
    times=[]
    for f in frames:
        t=max(0,(int(f['frame_us'])-epoch)//1000);now=max(t,(int(f['delivered_us'])+int(f['processing_us'])-epoch)//1000)
        valid=quality(f) and int(f['log_time_us'])>=0 and f['state_id']=='3'
        l,r=[number(f[k]) for k in ['left_deg','right_deg']]
        lines.append(f"{f['run_id']} {t} {now} {int(valid)} {l if math.isfinite(l) else 0} {r if math.isfinite(r) else 0}")
        times.append(t)
    proc=subprocess.run([str(exe)],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    outputs=proc.stdout.splitlines();assert len(outputs)==len(frames)
    records=[];errors=[]
    for i,(f,line) in enumerate(zip(frames,outputs)):
        reason,l,r,horizon,a,b=line.split();reason=int(reason);target=times[i]+int(horizon)
        record={'sequence':int(f['sequence']),'reason':reason,'predicted_left_deg':number(l),'predicted_right_deg':number(r),'target_log_ms':target,'switch_sign_plus_deg':number(a),'switch_sign_minus_deg':number(b)}
        j=bisect.bisect_left(times,target)
        if reason==0 and 0<j<len(frames) and quality(frames[j-1]) and quality(frames[j]) and times[j]>times[j-1] and times[j]-times[j-1]<=200:
            alpha=(target-times[j-1])/(times[j]-times[j-1])
            err=[]
            for side,pred in [('left',float(l)),('right',float(r))]:
                actual=(1-alpha)*number(frames[j-1][side+'_deg'])+alpha*number(frames[j][side+'_deg'])
                err.append(abs(pred-actual))
            record['interpolated_camera_abs_error_deg']=err;errors.extend(err)
        records.append({k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in record.items()})
    summary={'frames':len(frames),'ready':sum(r['reason']==0 for r in records),'camera_error_count':len(errors),
             'camera_error_mae_deg':statistics.mean(errors) if errors else None,
             'camera_error_max_deg':max(errors) if errors else None,
             'scope':'shadow; approximate processing time; interpolated camera comparison not angle ground truth; no output simulation',
             'records':records}
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')
    print({k:v for k,v in summary.items() if k!='records'})
