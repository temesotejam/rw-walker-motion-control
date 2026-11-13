"""Portable v12 selector checks, including an exact reduced-envelope v11 comparison."""
from pathlib import Path
import shutil,subprocess,tempfile

ROOT=Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "foot_support_assist_v12.h"
#include <cassert>
#include <limits>
namespace old=foot_support_assist;
namespace next=foot_support_assist_v12;
next::Input good() {
 next::Input i;i.run_id=i.foot_run_id=3;i.peak_id=800;i.now_ms=1000;i.frame_ms=925;
 i.peak_side=1;i.active=i.pose_ready=i.pose_confident=i.normal_drive_selected=true;
 i.left_deg=2.5f;i.right_deg=0;i.left_uncertainty_deg=i.right_uncertainty_deg=.2f;
 i.body_angle_deg=2;i.body_rate_dps=-20;i.normal_width_ms=55;
 i.prediction_source=next::PredictionSource::Local;return i;
}
int main() {
 auto i=good();auto d=next::decide(i);
 assert(d.selected && d.width_ms==55 && !d.stall_route && !d.coast_assist);
 assert(d.mode==next::Mode::OrdinaryRelocation && d.prediction_source==next::PredictionSource::Local);
 assert(next::widthPreserved(d,55) && !next::widthPreserved(d,54));
 i.normal_width_ms=70;assert(next::decide(i).selected && next::decide(i).width_ms==70);
 assert(old::decide(i).reason==old::Reason::Width);
 i.normal_width_ms=80;assert(next::decide(i).selected && next::decide(i).width_ms==80);
 i.normal_width_ms=81;assert(next::decide(i).reason==next::Reason::Width);
 next::Limits unsafe;unsafe.max_ordinary_width_ms=150;
 assert(next::decide(i,unsafe).reason==next::Reason::Width);
 // A reduced normal-width limit cannot truncate the command to gain entry.
 i.normal_width_ms=70;next::Limits width60;width60.max_ordinary_width_ms=60;
 assert(next::decide(i,width60).reason==next::Reason::Width);
 // TTC above 250ms is allowed only as an explicitly tagged approaching-stall
 // trial with measured deceleration AND current shape/local-return evidence.
 i=good();i.normal_width_ms=70;i.body_angle_deg=2.5f;i.body_rate_dps=-8;
 i.inward_decelerating=i.anticipated_shape_need=i.local_returning_confirmed=true;
 d=next::decide(i);assert(d.selected && d.stall_route && d.width_ms==70);
 assert(d.mode==next::Mode::StallOrdinaryRelocation && d.estimated_zero_ms==312.5f);
 assert(!next::widthPreserved(d,15) && next::widthPreserved(d,70));
 i.peak_side=-1;i.body_angle_deg=-2.5f;i.body_rate_dps=8;
 assert(next::decide(i).selected && next::decide(i).stall_route);
 i=good();i.normal_width_ms=70;i.body_angle_deg=2.5f;i.body_rate_dps=-8;
 i.inward_decelerating=i.anticipated_shape_need=i.local_returning_confirmed=true;
 i.inward_decelerating=false;assert(next::decide(i).reason==next::Reason::StallNeedsEvidence);
 i.inward_decelerating=true;i.anticipated_shape_need=false;assert(next::decide(i).reason==next::Reason::StallNeedsEvidence);
 i.anticipated_shape_need=true;i.local_returning_confirmed=false;assert(next::decide(i).reason==next::Reason::StallNeedsEvidence);
 i.local_returning_confirmed=true;next::Limits no_stall;no_stall.allow_stall_route=false;
 assert(next::decide(i,no_stall).reason==next::Reason::TooEarly);
 i.right_deg=i.left_deg;assert(next::decide(i).reason==next::Reason::StallNeedsEvidence);
 i.right_deg=0;i.body_angle_deg=4.1f;i.body_rate_dps=-6.5f;
 assert(next::decide(i).reason==next::Reason::StallTooEarly);
 i.body_angle_deg=4;i.body_rate_dps=-8;assert(next::decide(i).selected); // exactly 500ms
 i.body_angle_deg=4.01f;assert(next::decide(i).reason==next::Reason::StallTooEarly);
 // Previously latched evidence must never make zero/stopped/outward motion
 // eligible. Min motion, age, clock, uncertainty and width guards remain.
 i=good();i.inward_decelerating=i.anticipated_shape_need=i.local_returning_confirmed=true;
 i.body_rate_dps=0;assert(next::decide(i).reason==next::Reason::WrongMotion);
 i.body_rate_dps=8;assert(next::decide(i).reason==next::Reason::WrongMotion);
 i.body_rate_dps=-5.99f;assert(next::decide(i).reason==next::Reason::Speed);
 i=good();i.body_angle_deg=1;i.body_rate_dps=-20;i.normal_width_ms=70;
 assert(next::decide(i).reason==next::Reason::InsufficientLead);
 i=good();i.frame_ms=839;assert(next::decide(i).reason==next::Reason::StaleFrame);
 i=good();i.frame_ms=1001;assert(next::decide(i).reason==next::Reason::Time);
 i=good();i.pose_confident=false;assert(next::decide(i).reason==next::Reason::PoseUncertain);
 // The v11 15ms coast branch remains separate from ordinary relocation.
 i=good();i.normal_drive_selected=false;i.normal_coast_selected=true;i.normal_width_ms=0;
 assert(next::decide(i).reason==next::Reason::CoastNeedsEvidence);
 i.anticipated_shape_need=true;d=next::decide(i);
 assert(d.selected && d.coast_assist && d.width_ms==15 && !d.stall_route);
 i.body_angle_deg=2.5f;i.body_rate_dps=-8;
 assert(next::decide(i).reason==next::Reason::StallNeedsEvidence);
 i.inward_decelerating=i.local_returning_confirmed=true;d=next::decide(i);
 assert(d.selected && d.coast_assist && d.stall_route && d.width_ms==15);
 assert(d.mode==next::Mode::StallCoastAssist && !next::widthPreserved(d,14));
 i.normal_width_ms=15;assert(next::decide(i).reason==next::Reason::Width);
 // Stable indices let metadata distinguish v12 additions without relabeling
 // any v11 rejection. This selector itself never owns a peak or sends output.
 static_assert(static_cast<unsigned>(next::Reason::CoastNeedsEvidence)==16,"v11 index");
 static_assert(static_cast<unsigned>(next::Reason::StallNeedsEvidence)==17,"v12 evidence index");
 static_assert(static_cast<unsigned>(next::Reason::StallTooEarly)==18,"v12 far limit index");
 // With 60ms/no-stall comparison limits, all preexisting policy gates match
 // v11 across independent body/foot/width/evidence combinations.
 next::Limits baseline;baseline.max_ordinary_width_ms=60;baseline.allow_stall_route=false;
 for(float l:{-1.f,.9f,1.5f,3.f})for(float r:{-1.f,0.f,2.f})
 for(float a:{-.1f,.79f,.8f,2.f,4.5f,5.01f})for(float v:{-1.f,0.f,5.9f,6.f,20.f,45.f,45.1f})
 for(unsigned w:{0u,15u,55u,60u,70u,80u,81u})for(unsigned state=0;state<4;++state)
 for(unsigned evidence=0;evidence<4;++evidence){
   auto z=good();z.left_deg=l;z.right_deg=r;z.body_angle_deg=a;z.body_rate_dps=-v;
   z.normal_width_ms=static_cast<uint16_t>(w);z.normal_drive_selected=state&1u;z.normal_coast_selected=state&2u;
   z.inward_decelerating=evidence&1u;z.anticipated_shape_need=evidence&2u;
   const auto was=old::decide(z);const auto now=next::decide(z,baseline);
   assert(was.selected==now.selected && was.coast_assist==now.coast_assist && was.width_ms==now.width_ms);
   assert(static_cast<unsigned>(was.reason)==static_cast<unsigned>(now.reason));
 }
}
'''

def main():
    compiler=shutil.which('g++')
    if not compiler:raise SystemExit('g++ is required')
    with tempfile.TemporaryDirectory(prefix='foot_support_assist_v12_') as tmp:
        p=Path(tmp);(p/'test.cpp').write_text('#include <initializer_list>\n'+SOURCE,encoding='utf8')
        subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
        subprocess.run([str(p/'test.exe')],check=True)
    print('v12 ordinary-width/stall envelope and reduced-envelope v11 equivalence PASS')

if __name__=='__main__':main()
