#include "tilt_priority.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using tilt_priority::Controller;
int main(){
  Controller c;
  c.peak(1,220,2.46524f);c.peak(-1,598,5.0033f);
  c.peak(1,991,6.16416f);
  assert(c.negativeEarlyThreshold(1200,1)==0.f);
  c.peak(-1,6880,4.19616f);c.peak(1,7238,7.2264f);
  c.footCycle(6053,6880,3.58f,.56f,true);
  assert(std::fabs(c.negativeEarlyThreshold(7300,1)-1.81814f)<.001f);
  c.footCycle(6880,7724,3.87f,.48f,true);
  c.peak(-1,7724,4.05603f);c.peak(1,8091,7.03083f);
  assert(std::fabs(c.negativeEarlyThreshold(8250,1)-1.78488f)<.001f);
  assert(c.negativeEarlyThreshold(8250,-1)==0.f);
  assert(c.negativeEarlyThreshold(12000,1)==0.f); // stale peaks never trigger
  c.footCycle(7724,8534,2.8f,.77f,false);
  c.peak(-1,8534,5.f);c.peak(1,8908,7.f);
  assert(c.negativeEarlyThreshold(9000,1)==0.f); // invalid camera and gap <2.5
  c.footCycle(8534,9368,3.f,.4f,true);
  assert(c.negativeEarlyThreshold(9400,1)==0.f); // one cycle is not enough
  c.footCycle(9368,10235,4.f,.5f,true);
  c.peak(-1,10235,4.f);c.peak(1,10598,6.f);
  assert(std::fabs(c.negativeEarlyThreshold(10650,1)-1.2f)<.001f);
  c.footCycle(10235,11213,3.f,2.5f,true);
  assert(c.negativeEarlyThreshold(11300,1)==0.f); // balance restored
  c.peak(-1,13729,3.86261f);c.peak(1,14106,8.38403f);
  assert(std::fabs(c.negativeEarlyThreshold(14403,1)-2.713f)<.001f);
  // Run 305's final observed return passed +2.71 -> +2.63 with negative
  // velocity, but never reached the old +1 degree trigger. The inputs are
  // replayed observations, not the modified controller's future trajectory.
  c.peak(-1,19952,4.34886f);c.peak(1,20339,8.85291f);
  const float threshold=c.negativeEarlyThreshold(20656,1);
  assert(std::fabs(threshold-2.70243f)<.001f);
  const bool inward=-13.77f<0.f;
  const bool would_cross=inward && 2.71f>threshold && 2.63f<=threshold;
  const bool old_one_degree=inward && 2.71f>1.f && 2.63f<=1.f;
  assert(would_cross && !old_one_degree);
  c.peak(-1,21000,1.f);c.peak(1,21300,9.f);
  assert(c.negativeEarlyThreshold(21400,1)==3.f); // 3 degree ceiling
  c.reset();assert(c.negativeEarlyThreshold(14403,1)==0.f);
  Controller cap;cap.peak(-1,500,2.f);cap.crossing(1,1000);
  tilt_priority::Choice drive;drive.selected=true;drive.direction=1;
  drive.target_deg=4.5f;drive.current_ma=1200;drive.width_ms=10;
  cap.confirm(1,1000,drive,true);cap.peak(1,1050,2.f);
  assert(cap.timingCap(1,1100,150)==20); // observed 50ms to peak, minus 30ms
  assert(cap.timingCap(1,1100,10)==10); // absolute hardware cap
  assert(cap.timingCap(1,5000,150)==150); // stale timing never clamps a future trial
  assert(cap.timingCap(0,1100,150)==0);
  std::puts("body asymmetry and camera-based early timing: PASS");
}
