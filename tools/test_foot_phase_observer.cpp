#include "foot_phase_observer.h"
#include <cassert>
#include <cmath>
#include <iostream>

using foot_phase_observer::Observer;
static bool close(float a,float b) { return std::fabs(a-b)<1e-5f; }

// Each delayed confirmed peak sees an earlier frame timestamp, then a closer
// later frame timestamp. The invalid watermark completes the candidate window.
static void capture(Observer& o, uint16_t run, int side, unsigned ms, float l, float r) {
  assert(o.frame(run,ms-40,true,true,l+10,r+10));
  assert(o.peak(run,side,ms));
  assert(o.frame(run,ms+10,true,true,l,r));
  assert(o.frame(run,ms+110,false,false,0,0));
}

int main() {
  Observer o;o.reset(7);
  capture(o,7,-1,1000,1,2);
  capture(o,7,1,1400,4,1);
  capture(o,7,-1,1800,1,2);
  assert(o.count()==1 && !o.valid(1810));
  capture(o,7,1,2200,4,1);
  capture(o,7,-1,2600,1,2);
  assert(o.count()==2 && o.valid(2710));
  assert(o.cycleStart()==1800 && o.cycleTime()==2600);
  assert(close(o.leftMean(),3) && close(o.rightMean(),1));
  assert(!o.valid(4601));
  assert(!o.frame(8,2800,true,true,8,8));
  assert(!o.peak(8,1,3000));
  assert(o.count()==2);

  // A run-relative peak paired with a sync-relative timestamp must NOT match.
  o.reset(9);assert(o.frame(9,5000,true,true,2,3));assert(o.peak(9,-1,1000));
  assert(o.count()==0 && !o.valid(5000));

  // Missing right detection invalidates the entire matched peak, rather than
  // accidentally mixing two different frames or retaining old cycles.
  o.reset(10);
  capture(o,10,-1,1000,1,2);capture(o,10,1,1400,4,1);capture(o,10,-1,1800,1,2);
  capture(o,10,1,2200,4,1);capture(o,10,-1,2600,1,2);
  assert(o.valid(2710));
  assert(o.peak(10,1,3000));
  assert(o.frame(10,3010,true,false,4,1));
  assert(o.frame(10,3110,false,false,0,0));
  assert(o.count()==0 && !o.valid(3110));

  // Reverse motion is signed and must not be clipped to zero.
  o.reset(11);
  capture(o,11,-1,1000,3,2);capture(o,11,1,1400,1,4);capture(o,11,-1,1800,3,2);
  capture(o,11,1,2200,1,4);capture(o,11,-1,2600,3,2);
  assert(o.valid(2710) && close(o.leftMean(),-2) && close(o.rightMean(),-2));

  // A change in cycles is visible and can be rejected with a tighter spread.
  capture(o,11,1,3000,2,4);capture(o,11,-1,3400,3,2);
  assert(o.count()==3 && o.valid(3510));
  assert(o.cycleStart()==2600 && o.cycleTime()==3400);
  assert(close(o.leftLatest(),-1) && close(o.rightLatest(),-2));
  assert(!close(o.leftMean(),o.leftLatest())); // no older cycles in latest pair
  assert(!o.valid(3510,2000,.5f));
  assert(!o.frame(11,3400,true,true,0,0));
  assert(!o.peak(11,1,3300));

  // A missing same-side counterpart breaks the consecutive cycle history.
  assert(o.peak(11,-1,4000));
  assert(o.frame(11,4010,true,true,3,2));
  assert(o.frame(11,4110,false,false,0,0));
  assert(o.count()==0 && !o.valid(4110));

  // Camera stall during a pending peak cannot leave old feedback enabled.
  o.reset(12);
  capture(o,12,-1,1000,1,2);capture(o,12,1,1400,4,1);capture(o,12,-1,1800,1,2);
  capture(o,12,1,2200,4,1);capture(o,12,-1,2600,1,2);
  assert(o.valid(2710));assert(o.peak(12,1,3000));
  assert(!o.valid(3501));

  // Both feet use the same average of negative endpoints, preserving signed
  // values under zero drift instead of treating the right foot asymmetrically.
  o.reset(13);
  capture(o,13,-1,1000,0,2);capture(o,13,1,1400,4,1);capture(o,13,-1,1800,2,4);
  assert(close(o.leftLatest(),3) && close(o.rightLatest(),2));

  // Overlapping +/-100 ms peak windows must not reuse one frame for both.
  o.reset(14);
  capture(o,14,-1,1000,1,2);capture(o,14,1,1400,4,1);capture(o,14,-1,1800,1,2);
  capture(o,14,1,2200,4,1);
  assert(o.peak(14,-1,2600));
  assert(o.frame(14,2670,true,true,1,2));
  assert(o.frame(14,2710,false,false,0,0));
  assert(o.count()==2);
  assert(o.peak(14,1,2740));
  assert(o.frame(14,2850,false,false,0,0));
  assert(o.count()==0 && !o.valid(2850));
  
  // A detected marker may have a finite extrapolated angle while outside the
  // measured calibration support. Such frames must never qualify a peak.
  o.reset(15);
  assert(o.peak(15,-1,1000));
  assert(o.calibratedFrame(15,1010,true,true,false,true,1,2));
  assert(o.calibratedFrame(15,1110,false,false,true,true,0,0));
  assert(o.peak(15,1,1400));
  assert(o.calibratedFrame(15,1410,true,true,true,false,4,1));
  assert(o.calibratedFrame(15,1510,false,false,true,true,0,0));
  assert(o.peak(15,-1,1800));
  assert(o.calibratedFrame(15,1810,true,true,true,true,1,2));
  assert(o.calibratedFrame(15,1910,false,false,true,true,0,0));
  assert(o.count()==0 && !o.valid(1910));
  // Detection validity remains required even when both coordinates are in range.
  assert(o.peak(15,1,2200));
  assert(o.calibratedFrame(15,2210,false,true,true,true,4,1));
  assert(o.calibratedFrame(15,2310,false,false,true,true,0,0));
  assert(o.count()==0 && !o.valid(2310));
  // Camera coordinates do not have a calibration-support cutoff; missing
  // detections and nonfinite values still cannot form observations.
  const float lmin=appcfg::kFootAngleBMinCalXPx, lmax=appcfg::kFootAngleBMaxCalXPx;
  const float rmin=appcfg::kFootAngleAMinCalXPx, rmax=appcfg::kFootAngleAMaxCalXPx;
  auto ok=Observer::boundedMarker;
  assert(ok(true,lmin-3.f,lmin,lmax));
  assert(ok(true,lmax+3.f,lmin,lmax));
  assert(ok(true,lmin-3.01f,lmin,lmax));
  assert(ok(true,lmax+3.01f,lmin,lmax));
  assert(ok(true,lmax+20.f,lmin,lmax));
  assert(!ok(true,NAN,lmin,lmax));
  assert(!ok(false,lmax,lmin,lmax));
  assert(ok(true,rmax+3.f,rmin,rmax));
  assert(ok(true,rmax+3.01f,rmin,rmax));
  o.reset(16);
  auto rawCapture=[&](int side,unsigned ms,float lx,float rx,bool detected=true) {
    assert(o.peak(16,side,ms));
    assert(o.rawFrame(16,ms+10,detected,detected,lx,rx));
    assert(o.rawFrame(16,ms+110,false,false,lx,rx));
  };
  rawCapture(-1,1000,195,198);rawCapture(1,1400,170,202);
  rawCapture(-1,1800,195,198);rawCapture(1,2200,170,202);
  rawCapture(-1,2600,195,198);
  assert(o.valid(2710));
  const float l_expected=foot_calibration::angleAt(foot_calibration::left,170)-foot_calibration::angleAt(foot_calibration::left,195);
  const float r_expected=foot_calibration::angleAt(foot_calibration::right,198)-foot_calibration::angleAt(foot_calibration::right,202);
  assert(close(o.leftLatest(),l_expected) && close(o.rightLatest(),r_expected));
  // Crossing calibration table segments is nonlinear. Any fixed reference
  // cancels exactly in the phase differences, including if the UI zero locks.
  for(float ref: {150.f,201.f}) {
    const float calc=Observer::fixedAngle(foot_calibration::left,170,ref)-Observer::fixedAngle(foot_calibration::left,195,ref);
    assert(close(calc,l_expected));
  }
  assert(!close(l_expected,25.f*appcfg::kFootAngleBDegPerPx));
  rawCapture(1,3000,lmax+1.f,rmax+3.01f);
  assert(o.count()==2);
  rawCapture(-1,3400,lmax,rmax,false);
  assert(!o.valid(3510));
  std::cout << "foot phase observer: nearest frame, run timebase, signed motion and incomplete-cycle guards PASS\n";
}
