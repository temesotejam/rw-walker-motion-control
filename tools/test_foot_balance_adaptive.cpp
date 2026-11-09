#include "foot_balance_adaptive.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  foot_balance_adaptive::Controller c;c.reset(7);
  assert(!c.observe(1,2,true,8,1000,1000));
  assert(!c.observe(1,2,true,7,1000,800));
  assert(!c.observe(1,NAN,true,7,1000,1000));
  for(unsigned i=0;i<3;++i) {
    assert(c.observe(1,2,true,7,1000+i*800,1000+i*800));
    assert(c.observe(-1,.1f,true,7,1400+i*800,1400+i*800));
  }
  assert(std::fabs(c.negativeOffsetDeg()-.1f)<1e-5f);
  assert(!c.observe(-1,.1f,true,7,3000,3000));
  const float before=c.negativeOffsetDeg();
  assert(!c.observe(-1,0,false,7,4000,4000));assert(c.negativeOffsetDeg()==before);
  c.reset(8);assert(c.negativeOffsetDeg()==0);
  for(unsigned i=0;i<30;++i) {
    assert(c.observe(1,.1f,true,8,1000+i*800,1000+i*800));
    assert(c.observe(-1,2.f,true,8,1400+i*800,1400+i*800));
  }
  assert(c.positiveOffsetDeg()>.9f && c.positiveOffsetDeg()<=1.f);
  assert(c.negativeOffsetDeg()==0);
  c.reset(9);
  for(unsigned i=0;i<3;++i)assert(c.observe(1,2,true,9,1000+i*800,1000+i*800));
  for(unsigned i=0;i<3;++i)assert(c.observe(-1,0,true,9,10000+i*800,10000+i*800));
  assert(c.negativeOffsetDeg()==0); // no correction using stale other-side frames
  // A late camera delivery may contain an exposure closer to the peak than
  // the last already-delivered frame. Compare the nearest on both sides.
  c.reset(10);
  for(unsigned i=0;i<3;++i) {
    const unsigned t=1000+i*800;
    c.frame(10,t-80,true,true,1.f,.1f);c.peak(1,t);
    c.frame(10,t+20,true,true,2.f,.1f);
    c.frame(10,t+320,true,true,1.f,.2f);c.peak(-1,t+400);
    c.frame(10,t+420,true,true,1.f,.2f);
  }
  assert(std::fabs(c.leftMean()-2.f)<1e-5f);
  assert(std::fabs(c.rightMean()-.2f)<1e-5f);
  assert(c.negativeOffsetDeg()>.09f);
  c.reset(11);c.frame(10,1000,true,true,9.f,9.f);c.peak(1,1000);
  c.frame(11,1200,true,true,9.f,9.f);assert(std::isnan(c.leftMean()));
  std::cout<<"Foot-balance adaptive: run, time, valid-frame and bounded adaptation checks PASS\n";
}
