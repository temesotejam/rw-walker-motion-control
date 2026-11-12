"""Compile the portable early-input selector and verify physical/control gates."""
from pathlib import Path
import shutil,subprocess,tempfile

ROOT=Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "foot_support_assist.h"
#include <cassert>
#include <limits>
using namespace foot_support_assist;
Input good() {
 Input i;i.run_id=i.foot_run_id=3;i.peak_id=800;i.now_ms=1000;i.frame_ms=925;
 i.peak_side=1;i.active=i.pose_ready=i.pose_confident=i.normal_drive_selected=true;
 i.left_deg=2.5f;i.right_deg=0.f;i.left_uncertainty_deg=i.right_uncertainty_deg=.2f;
 i.body_angle_deg=2.f;i.body_rate_dps=-20.f;i.normal_width_ms=55;return i;
}
int main() {
 auto i=good();auto d=decide(i);assert(d.selected && d.width_ms==55 && d.frame_age_ms==75);
 assert(d.estimated_zero_ms==100.f && d.threshold_deg>2.f && d.threshold_deg<=5.f);
 assert(widthPreserved(d,55));assert(!widthPreserved(d,54));assert(!widthPreserved(d,60));
 assert(!widthPreserved(Decision{},55));
 // Both body sides and either foot use the same policy; no inferred stance map.
 i.peak_side=-1;i.body_angle_deg=-2;i.body_rate_dps=20;assert(decide(i).selected);
 std::swap(i.left_deg,i.right_deg);assert(decide(i).selected);
 i=good();i.active=false;assert(decide(i).reason==Reason::Inactive);
 i=good();i.foot_run_id=2;assert(decide(i).reason==Reason::Identity);
 i=good();i.peak_id=0;assert(!decide(i).selected);
 i=good();i.peak_side=0;assert(!decide(i).selected);
 i=good();i.frame_ms=1001;assert(decide(i).reason==Reason::Time);
 i=good();i.frame_ms=0;assert(decide(i).reason==Reason::Time);
 i=good();i.peak_id=1000;assert(decide(i).reason==Reason::Time);
 i=good();i.peak_id=1;i.now_ms=1502;assert(decide(i).reason==Reason::Time);
 i=good();i.frame_ms=i.now_ms-kMaxFrameAgeMs;assert(decide(i).selected);
 --i.frame_ms;assert(decide(i).reason==Reason::StaleFrame);
 i=good();i.pose_ready=false;assert(decide(i).reason==Reason::PoseUnavailable);
 i=good();i.pose_confident=false;assert(decide(i).reason==Reason::PoseUncertain);
 i=good();i.left_deg=std::numeric_limits<float>::quiet_NaN();assert(decide(i).reason==Reason::Nonfinite);
 i=good();i.body_rate_dps=std::numeric_limits<float>::infinity();assert(decide(i).reason==Reason::Nonfinite);
 i=good();i.right_uncertainty_deg=-.01f;assert(decide(i).reason==Reason::PoseUncertain);
 i=good();i.left_uncertainty_deg=.751f;assert(decide(i).reason==Reason::PoseUncertain);
 i=good();i.left_deg=.9f;assert(decide(i).reason==Reason::SmallForward);
 i=good();i.left_deg=-2;i.right_deg=-1;assert(decide(i).reason==Reason::SmallForward);
 i=good();i.body_rate_dps=20;assert(decide(i).reason==Reason::WrongMotion);
 i=good();i.body_angle_deg=-2;assert(decide(i).reason==Reason::WrongMotion);
 i=good();i.body_angle_deg=0;assert(decide(i).reason==Reason::WrongMotion);
 i=good();i.body_angle_deg=.79f;assert(decide(i).reason==Reason::OutsideWindow);
 i=good();i.body_angle_deg=5.01f;assert(decide(i).reason==Reason::OutsideWindow);
 i=good();i.body_rate_dps=-5.9f;assert(decide(i).reason==Reason::Speed);
 i=good();i.body_rate_dps=-45.1f;assert(decide(i).reason==Reason::Speed);
 i=good();i.normal_drive_selected=false;assert(decide(i).reason==Reason::NoOrdinaryDrive);
 i=good();i.normal_width_ms=0;assert(decide(i).reason==Reason::Width);
 i=good();i.normal_width_ms=61;assert(decide(i).reason==Reason::Width);
 i=good();i.normal_width_ms=60;assert(decide(i).selected);
 i=good();i.body_rate_dps=-30;assert(decide(i).reason==Reason::InsufficientLead);
 i=good();i.body_rate_dps=-6;assert(decide(i).reason==Reason::TooEarly);
 // Conservative uncertainty decreases the shape-change proxy and threshold.
 i=good();auto a=decide(i);i.left_uncertainty_deg=i.right_uncertainty_deg=.7f;
 auto b=decide(i);assert(b.threshold_deg<=a.threshold_deg && b.forward_deg<a.forward_deg);
 // A selected normal coast alone cannot authorize input. It requires either
 // measured sustained inward deceleration or caller-validated shape margins,
 // the dominant forward foot's returning phase and an incoming peak cap.
 i=good();i.normal_drive_selected=false;i.normal_coast_selected=true;i.normal_width_ms=0;
 assert(decide(i).reason==Reason::CoastNeedsEvidence);
 static_assert(static_cast<unsigned>(Reason::CoastNeedsEvidence)==16,"reason indices must remain stable");
 static_assert(Reason::CoastNeedsDeceleration==Reason::CoastNeedsEvidence,"compatibility reason alias");
 i.inward_decelerating=true;auto coast=decide(i);
 assert(coast.selected && coast.coast_assist && coast.width_ms==15);
 assert(widthPreserved(coast,15) && !widthPreserved(coast,14));
 i.peak_side=-1;i.body_angle_deg=-2;i.body_rate_dps=20;
 auto mirror=decide(i);assert(mirror.selected && mirror.coast_assist && mirror.width_ms==15);
 i.normal_drive_selected=true;assert(decide(i).reason==Reason::NoOrdinaryDrive);
 i.normal_drive_selected=false;i.normal_width_ms=20;assert(decide(i).reason==Reason::Width);
 i.normal_width_ms=0;i.pose_confident=false;assert(decide(i).reason==Reason::PoseUncertain);
 i=good();i.normal_drive_selected=false;i.normal_coast_selected=true;i.normal_width_ms=0;
 i.anticipated_shape_need=true;coast=decide(i);
 assert(coast.selected && coast.coast_assist && coast.width_ms==15 && !i.inward_decelerating);
 i.anticipated_shape_need=false;assert(decide(i).reason==Reason::CoastNeedsEvidence);
 i.anticipated_shape_need=true;i.frame_ms=800;assert(decide(i).reason==Reason::StaleFrame);
 i=good();i.anticipated_shape_need=true;
 assert(decide(i).selected && !decide(i).coast_assist && decide(i).width_ms==55);
 i=good();assert(!decide(i).coast_assist); // drive does not require deceleration
 // The selector has no foot-rate sign gate: a forward foot returning toward
 // neutral can still change support geometry. Either side remains symmetric.
 i.left_deg=1.6f;i.right_deg=-.3f;i.body_angle_deg=1.7f;i.body_rate_dps=-15;
 assert(decide(i).selected);
 // Trial threshold and pulse ceilings remain bounded over large foot values.
 for(float f=1;f<20;f+=.25f){i=good();i.left_deg=f;auto c=decide(i);
   if(std::isfinite(c.threshold_deg))assert(c.threshold_deg>=1.5f && c.threshold_deg<=5.f);
   if(c.selected)assert(c.width_ms<=60);
 }
}
'''

def main():
    compiler=shutil.which('g++')
    if not compiler:
        raise SystemExit('g++ is required')
    with tempfile.TemporaryDirectory(prefix='foot_support_assist_') as tmp:
        p=Path(tmp);(p/'test.cpp').write_text('#include <utility>\n'+SOURCE,encoding='utf8')
        subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I',str(ROOT/'src'),str(p/'test.cpp'),'-o',str(p/'test.exe')],check=True)
        subprocess.run([str(p/'test.exe')],check=True)
    print('Foot support assist identity, delay, direction, ordinary-width preservation and boundaries PASS')

if __name__=='__main__':main()
