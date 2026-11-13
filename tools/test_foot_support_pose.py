"""Shared source selection must not borrow confidence or cross image identities."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
source=r'''
#include "foot_support_pose.h"
#include <cassert>
int main(){
 foot_phase_prediction::Projection h;h.run_id=2;h.frame_ms=300;h.received_ms=360;
 h.reason=foot_phase_prediction::Reason::Ready;h.confidence=.9f;
 for(unsigned j=0;j<4;++j){h.left[j]=0;h.right[j]=3;h.left_uncertainty[j]=h.right_uncertainty[j]=.4f;}
 foot_local_prediction::Predictor local;
 local.observe(2,100,160,true,0,5.8f);local.observe(2,200,260,true,0,5.4f);local.observe(2,300,360,true,0,4);
 auto l=local.projection();auto s=foot_support_pose::select(h,l,370);
 assert(s.usable && s.source==foot_support_pose::Source::History && !s.local_returning_confirmed);
 h.confidence=0;s=foot_support_pose::select(h,l,370);
 assert(s.usable && s.source==foot_support_pose::Source::Local && s.local_returning_confirmed && s.pose.confidence==0);
 l.run_id=3;assert(!foot_support_pose::select(h,l,370).usable);l.run_id=2;
 l.frame_ms=299;assert(!foot_support_pose::select(h,l,370).usable);l.frame_ms=300;
 assert(!foot_support_pose::select(h,l,359).usable);
 assert(!foot_support_pose::select(h,l,461).usable);
}
'''
with tempfile.TemporaryDirectory(prefix='foot_pose_sources_') as td:
    td=Path(td);cpp=td/'test.cpp';exe=td/'test.exe';cpp.write_text(source)
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
print('History/local source priority, independent score, identity and freshness PASS')
