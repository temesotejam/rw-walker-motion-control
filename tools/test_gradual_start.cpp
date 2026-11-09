#include "gradual_start.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  gradual_start::Controller c;
  assert(c.target()==4.5f && c.positiveCapMs()==80);
  // Positive pitch is the provisional physical-left hypothesis; the 8-degree
  // preset and negative side stay unchanged. The positive bias starts at
  // zero and increases continuously across the existing startup ramp.
  assert(gradual_start::sideTargetDeg(4.5f,1)==4.5f);
  assert(gradual_start::sideTargetDeg(8.f,1)==10.f);
  assert(gradual_start::sideTargetDeg(8.f,-1)==8.f);
  assert(std::fabs(gradual_start::sideTargetDeg(6.25f,1)-7.25f)<1e-6f);
  assert(!c.positivePeakAtLeast(8.f,1000));
  c.peak(1,7.9f,200,true,1000);assert(!c.positivePeakAtLeast(8.f,1001));
  c.peak(1,8.1f,200,true,1100);assert(c.positivePeakAtLeast(8.f,1500));
  assert(!c.positivePeakAtLeast(8.f,2601));assert(!c.positivePeakAtLeast(8.f,1099));
  c.peak(1,8.1f,200,false,1500);assert(!c.positivePeakAtLeast(8.f,1500));
  c.reset();
  auto pair=[&](unsigned t,float a,float b,float rpm){c.peak(1,a,rpm,true,t);c.peak(-1,b,rpm,true,t+400);};
  pair(1000,4.3f,4.2f,450);assert(c.target()==4.5f);
  pair(2000,4.3f,4.2f,450);assert(c.target()==4.5f);
  pair(3000,4.3f,4.2f,450);assert(c.target()==5.f);
  pair(4000,6,6,501);pair(5000,6,6,450);assert(c.target()==5.f);
  c.peak(1,6,450,false,6000);c.peak(-1,6,450,true,6400);assert(c.target()==5.f);
  c.reset();unsigned t=1000;
  for(int i=0;i<15;++i){pair(t,8,8,450);t+=1000;}
  assert(c.target()==8 && c.positiveCapMs()==100);
  for(int i=0;i<4;++i){pair(t,8,8,450);t+=1000;}assert(c.target()==8);
  c.reset();c.peak(1,8,450,true,1000);c.peak(-1,8,450,true,3001);assert(c.target()==4.5f);
  // Real 30460274 was trapped at stage 5.5 deg with a normal 85 ms cap:
  // even though safe alternating peaks continued, one side did not meet the
  // current target. Two measured pairs now advance the requested target and
  // pulse cap without requiring the under-target side to first improve. The
  // first detected pair is startup, so it is excluded from advancement.
  c.reset();t=1000;
  for(int i=0;i<15;++i) {
    const float positive=(i<5?4.3f:5.6f),negative=(i<5?4.5f:2.4f);
    pair(t,positive,negative,200.f);t+=800;
    const float expected=std::fmin(8.f,4.5f+.5f*static_cast<float>(i/2));
    assert(std::fabs(c.target()-expected)<1e-6f);
    assert(c.positiveCapMs()<=100);
  }
  assert(c.target()==8.f && c.positiveCapMs()==100);
  // A phase can be small or zero without being reclassified as an invalid
  // sensor: target error is not a range check. Actual sensor freshness,
  // timestamp order, alternating side, and the existing speed guard remain.
  c.reset();pair(1000,0.f,0.f,0.f);pair(2000,0.f,0.f,0.f);pair(3000,0.f,0.f,0.f);
  assert(c.target()==5.f);
  c.reset();pair(1000,2.f,2.f,200.f);
  c.peak(1,NAN,200.f,true,2000);c.peak(-1,2.f,200.f,true,2400);
  assert(c.target()==4.5f);
  c.reset();pair(1000,2.f,2.f,200.f);pair(2000,2.f,2.f,501.f);
  assert(c.target()==4.5f);
  c.reset();pair(1000,2.f,2.f,-200.f);pair(2000,2.f,2.f,-501.f);
  assert(c.target()==4.5f);
  c.reset();pair(1000,2.f,2.f,200.f);
  c.peak(1,2.f,NAN,true,2000);c.peak(-1,2.f,200.f,true,2400);
  assert(c.target()==4.5f);
  c.reset();pair(1000,2.f,2.f,200.f);
  c.peak(1,2.f,200.f,true,3000);c.peak(-1,2.f,200.f,true,4600);
  assert(c.target()==4.5f);
  c.reset();pair(1000,2.f,2.f,200.f);
  c.peak(1,2.f,200.f,false,2000);c.peak(-1,2.f,200.f,true,2400);
  assert(c.target()==4.5f);
  std::cout<<"Gradual startup: paired peaks, speed and freshness gates, staged cap and reset PASS\n";
}
