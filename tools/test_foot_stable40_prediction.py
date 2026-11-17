"""New camera cadence must remain usable without accepting stale/jumping poses."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
CODE=r'''
#include <cassert>
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"
#include "foot_pose_prediction.h"
#define CHECK(NS) {NS::Predictor p; NS::Reason reason=NS::Reason::Warmup; \
 for(unsigned i=0;i<4;++i)reason=p.observe(1,100+i*dt,120+i*dt,true,.1f*i,.1f*i); \
 assert(reason==(dt>=foot_timing_policy::kMinFrameIntervalMs?NS::Reason::Ready:NS::Reason::Gap)); \
 if(dt>=foot_timing_policy::kMinFrameIntervalMs){ \
  assert(p.observe(1,100+3*dt,120+3*dt,true,.3f,.3f)==NS::Reason::Timestamp); \
  p.reset();p.observe(1,100,120,true,0,0); \
  assert(p.observe(1,100+dt,120+dt,true,9,9)==NS::Reason::Jump); \
  p.reset();assert(p.observe(1,100,400,true,0,0)==NS::Reason::Stale);}}
int main(){
 for(unsigned dt:{14u,15u,20u,23u,25u,40u}){
  CHECK(foot_phase_prediction);CHECK(foot_local_prediction);
  foot_pose_prediction::Predictor p;p.observe(1,100,120,true,0,0);
  assert(p.observe(1,100+dt,120+dt,true,.1f,.1f).reason==
    (dt>=foot_timing_policy::kMinFrameIntervalMs?foot_pose_prediction::Reason::Ready:foot_pose_prediction::Reason::Gap));
 }
}
'''
with tempfile.TemporaryDirectory(prefix='foot_prediction40_') as folder:
    p=Path(folder);(p/'test.cpp').write_text('#include <initializer_list>\n'+CODE)
    for stable in [False,True]:
        flags=['-DFOOT_SPEED_V22=1']+(['-DFOOT_STABLE40_V36=1'] if stable else [])
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',*flags,'-I'+str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
        subprocess.run([str(p/'test.exe')],check=True)
print('PASS three real predictors at14/15/20/23/25/40ms; old25ms gate retained, duplicates/jumps/stale rejected')
