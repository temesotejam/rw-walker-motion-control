#include "gyro_steering.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
  using namespace steering;
  // True ZYX trajectories -> body rates. Heading remains independent of tilt.
  for (float turn : {0.0f,84.57f,-16.38f,370.0f}) {
    GyroHeading h; assert(h.reset(0,0,1,0xffff0000U));
    for (unsigned i=1;i<=12000;++i) {
      const float t=i*.0025f, f=2*3.14159265f*1.1f;
      const float r=2.5f*kRad*sinf(f*t), p=12*kRad*sinf(f*t);
      const float rd=2.5f*kRad*f*cosf(f*t), pd=12*kRad*f*cosf(f*t), yd=turn/30*kRad;
      h.update((rd-yd*sinf(p))*kDeg,
               (pd*cosf(r)+yd*sinf(r)*cosf(p))*kDeg,
               (-pd*sinf(r)+yd*cosf(r)*cosf(p))*kDeg,0xffff0000U+i*2500);
    }
    assert(h.valid()); assert(fabsf(h.yaw()-turn)<.04f);
    const float previous=h.yaw();
    h.update(NAN,0,0,0xffff0000U+12000*2500); // duplicate isn't a new sample
    assert(h.yaw()==previous);
    h.update(0,0,0,0xffff0000U+12000*2500+10001);
    assert(!h.valid() && !std::isfinite(h.yaw()));
  }
  GyroHeading bad; assert(!bad.reset(0,0,0,1)); assert(!bad.reset(NAN,0,1,1));
  assert(bad.reset(0,0,1,1)); bad.update(INFINITY,0,0,2501); assert(!bad.valid());
  // Deterministic illustrative plant. Not a claim of physical closed-loop stability.
  for (float disturbance : {-2.f,0.f,2.f}) {
    Controller c; float yaw=0,rate=disturbance,delta=0;
    for (unsigned cycle=0;cycle<40;++cycle) {
      const uint32_t ms=500+1000*cycle;
      // Geometric asymmetry can remain at zero yaw.
      const float difference=-1.2f+2*c.state().delta_deg;
      rate=disturbance-1.5f*(difference+1.2f);
      yaw+=rate;
      c.peak(1,10+difference/2,yaw-rate/2,ms,false,false);
      c.peak(-1,10-difference/2,yaw,ms+500,false,false);
      const auto& s=c.state();
      assert(fabsf(s.delta_deg)<=kLimitDeg && fabsf(s.delta_deg-delta)<=kStepDeg+1e-6f);
      assert(fabsf((c.target(10,1)+c.target(10,-1))/2-10)<1e-6f);
      if(ms+500<kSettleMs) assert(s.delta_deg==0);
      delta=s.delta_deg;
    }
    assert(fabsf(rate)<.45f);
    if(disturbance==0) { assert(fabsf(delta)<1e-6f); assert(fabsf(c.state().actual_difference_deg+1.2f)<1e-5f); }
  }
  // Both required actuator directions blocked: no windup. Reverse rotation releases.
  Controller c;float yaw=0;
  for(unsigned i=0;i<40;++i) {
    yaw+=3;
    c.peak(1,9,yaw-1.5f,i*1000+500,true,false);
    c.peak(-1,11,yaw,i*1000+1000,false,true);
  }
  assert(c.state().delta_deg==0 && c.state().reason==Reason::Saturated);
  for(unsigned i=40;i<50;++i) {
    yaw-=3;c.peak(1,9,yaw+1.5f,i*1000+500,true,false);
    c.peak(-1,11,yaw,i*1000+1000,false,true);
  }
  assert(c.state().delta_deg<0);
  const float held=c.state().delta_deg;
  c.peak(-1,11,NAN,51000,false,false);
  assert(c.state().delta_deg==held && c.state().reason==Reason::Invalid);
  c.reset(); assert(c.state().delta_deg==0 && c.state().cycles==0);
  c.peak(-1,10,0,11000,false,false);
  c.peak(1,10,1,11500,false,false);
  c.peak(1,10,1,11600,false,false); // duplicate positive peak invalidates this pair
  c.peak(-1,10,2,12000,false,false);
  assert(c.state().cycles==0 && c.state().delta_deg==0);
  c.peak(1,10,2,12500,false,false);
  c.peak(-1,10,2,13000,false,false);
  assert(c.state().cycles==1 && c.state().delta_deg==0);
  c.peak(1,10,2,13500,false,false);
  c.peak(-1,10,2,18000,false,false); // missing cycle: reacquire, hold correction
  assert(c.state().cycles==1 && c.state().delta_deg==0);
  // The ordinary-run response check is a prescribed input, independent of
  // measured yaw / asymmetry. Reverse its order across consecutive runs.
  for (uint16_t run : {uint16_t(1),uint16_t(2),uint16_t(65535)}) {
    Controller a,b;
    a.reset(Mode::ResponseCheck,run);b.reset(Mode::ResponseCheck,run);
    a.peak(-1,10,0,0,false,false);b.peak(-1,11,0,0,false,false);
    float previous=0;
    const float sign=responseFirstSign(run);
    for(unsigned cycle=1;cycle<=30;++cycle) {
      const uint32_t ms=cycle*1000;
      a.peak(1,10,3*cycle-1.5f,ms-500,true,true);
      b.peak(1,9,-4.f*cycle+2.f,ms-500,false,false);
      a.peak(-1,10,3*cycle,ms,true,true);
      b.peak(-1,11,-4.f*cycle,ms,false,false);
      const float d=a.state().delta_deg;
      assert(d==b.state().delta_deg);
      assert(fabsf(d)<=.2f && fabsf(d-previous)<=.080001f);
      assert(std::isnan(a.state().desired_difference_deg));
      for(float mean : {8.f,10.f,12.f})
        assert(fabsf((a.target(mean,1)+a.target(mean,-1))/2-mean)<1e-6f);
      if(ms<10000) assert(d==0 && a.state().reason==Reason::Settling);
      if(ms==10000) assert(fabsf(d-.08f*sign)<1e-6f);
      if(ms>=12000 && ms<18000) assert(fabsf(d-.2f*sign)<1e-6f);
      if(ms==18000) assert(fabsf(d-.12f*sign)<1e-6f);
      if(ms>=22000 && ms<26000) assert(fabsf(d+.2f*sign)<1e-6f);
      if(ms>=28000) assert(d==0 && a.state().reason==Reason::ResponseReturn);
      if(ms>=10000 && ms<26000) {
        const auto positive=ms<18000 ? sign>0 : sign<0;
        assert(a.state().reason==(positive?Reason::ResponsePositive:Reason::ResponseNegative));
      }
      previous=d;
    }
  }
  Controller probe;probe.reset(Mode::ResponseCheck,1);
  probe.peak(-1,10,0,11000,false,false);
  probe.peak(1,10,1,11500,false,false);probe.peak(-1,10,2,12000,false,false);
  const float before_fault=probe.state().delta_deg;
  probe.peak(1,10,NAN,12500,false,false);probe.peak(-1,10,NAN,13000,false,false);
  assert(probe.state().delta_deg==before_fault && probe.state().reason==Reason::Invalid);
  probe.reset(Mode::ResponseCheck,2);
  assert(probe.state().delta_deg==0 && probe.state().cycles==0);
  puts("Prescribed response schedule, reversed order, yaw independence, preserved mean/slew/bounds, return and invalid-data hold PASS");
  puts("Independent 3D gyro, zero-yaw sway, wrap/gap faults, both steering signs, mean/step bounds, nonzero straight asymmetry and saturation/reversal PASS");
}
