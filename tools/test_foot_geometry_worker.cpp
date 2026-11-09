#include "foot_geometry_worker.h"
#include <cassert>
#include <cmath>
using namespace foot_geometry_worker;
static Input input() {
  Input x; x.run_id=1;x.peak_id=3;x.generation=7;x.frame_ms=100;x.observed_ms=160;x.expiry_ms=260;
  x.left_deg=5;x.right_deg=0;x.pitch_lo_deg=0;x.pitch_hi_deg=0;
  x.origin_uncertainty_deg=0;x.pitch_origin_uncertainty_deg=0;x.max_spread_deg=1.5f;x.side=1;
  x.exposure=Input::Exposure::Verified;x.exposure_earliest_ms=95;x.exposure_latest_ms=105;
  x.pose_interval_bracketed=true;return x;
}
int main() {
  Pose p[3];p[0].ms=90;p[0].fore_aft_deg=1;p[1].ms=100;p[1].fore_aft_deg=2;
  p[2].ms=110;p[2].fore_aft_deg=4;float v=NAN;
  assert(interpolate(p,3,105,12,v) && std::fabs(v-3)<.001f);
  assert(!interpolate(p,3,105,9,v));assert(!interpolate(p,3,120,12,v));
  PoseHistory<3> hist;assert(hist.push(p[0]));assert(hist.push(p[1]));assert(hist.push(p[2]));
  assert(!hist.push(p[2]));assert(hist.at(105,12,v) && std::fabs(v-3)<.001f);
  assert(!hist.at(105,9,v));float lo,hi;
  assert(hist.range(95,105,12,lo,hi) && std::fabs(lo-1.5f)<.001f && std::fabs(hi-3.f)<.001f);
  assert(!hist.range(95,105,9,lo,hi));hist.reset();assert(!hist.at(105,12,v));
  Worker w;Input x=input();x.exposure=Input::Exposure::Unknown;assert(!w.submit(x,160));
  assert(w.diagnostic().reason==Reason::UnknownExposure);
  x=input();x.exposure=Input::Exposure::Provisional;
#if FOOT_GEOMETRY_PROVISIONAL_EXPOSURE
  assert(w.submit(x,160));
#else
  assert(!w.submit(x,160));assert(w.diagnostic().reason==Reason::ProvisionalDisabled);
#endif
  x=input();x.exposure_earliest_ms=101;assert(!w.submit(x,160));assert(w.diagnostic().reason==Reason::BadExposureInterval);
  x=input();x.pose_interval_bracketed=false;assert(!w.submit(x,160));assert(w.diagnostic().reason==Reason::PoseUnavailable);
  x=input();assert(w.submit(x,160));
  for(int i=0;i<95;++i)assert(w.step(161,1,3,7)==Status::Working);
  assert(w.step(161,1,3,7)==Status::Ready);
  Result o;assert(w.result(162,1,3,7,o));
  assert(o.run_id==1 && o.peak_id==3 && o.frame_ms==100 && o.ready_ms==162);
  assert(o.threshold_deg>0 && o.root_max_deg-o.root_min_deg<1.5f);
  // Same data used to pass with a known pitch origin. Adding the trial +/-2
  // degree initial-pose uncertainty makes the six geometric hypotheses
  // disagree; it must reject rather than publish a spurious early trigger.
  x=input();x.pitch_origin_uncertainty_deg=2.f;assert(w.submit(x,160));
  for(int i=0;i<96 && w.status()==Status::Working;++i)w.step(161,1,3,7);
  assert(w.status()==Status::Rejected && w.diagnostic().reason==Reason::Disagree);
  assert(w.diagnostic().pitch_origin_uncertainty_deg==2.f);
  x=input();assert(w.submit(x,160));assert(w.step(161,2,3,7)==Status::Rejected);
  assert(w.submit(x,160));assert(w.step(161,1,4,7)==Status::Rejected);
  assert(w.submit(x,160));assert(w.step(161,1,3,8)==Status::Rejected);
  assert(w.submit(x,160));assert(w.step(260,1,3,7)==Status::Rejected);
  assert(w.submit(x,160));for(int i=0;i<96;++i)w.step(161,1,3,7);
  assert(!w.result(260,1,3,7,o));
  assert(w.submit(x,160));for(int i=0;i<96;++i)w.step(161,1,3,7);
  assert(!w.result(162,1,4,7,o));
  x=input();x.origin_uncertainty_deg=10;assert(w.submit(x,160));
  for(int i=0;i<96 && w.status()==Status::Working;++i)w.step(161,1,3,7);
  assert(w.status()==Status::Rejected);
  x=input();assert(w.submit(x,160));w.step(161,1,3,7);x.peak_id=4;assert(w.submit(x,162));
  assert(w.step(163,1,3,7)==Status::Rejected); // replaced job cannot escape generation guard
}


