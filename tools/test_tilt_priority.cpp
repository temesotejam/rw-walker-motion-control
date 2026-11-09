#include "tilt_priority.h"
#include <cassert>
#include <cmath>
#include <iostream>
using tilt_priority::Choice;
using tilt_priority::Controller;
using tilt_priority::Input;

static Input input(int side,uint32_t now) {
  Input in;in.now_ms=now;in.preset_deg=8;in.target_deg=8;
  in.crossing_rate_dps=10.f;in.next_side=side;in.hardware_cap_ms=150;in.hardware_healthy=true;return in;
}
static Choice step(Controller& c,int side,uint32_t cross,float peak,bool accepted=true,uint32_t elapsed=180) {
  const auto choice=c.propose(input(side,cross));assert(choice.selected);
  c.crossing(side,cross);c.confirm(side,cross,choice,accepted);
  c.peak(side,cross+elapsed,peak);return choice;
}
int main() {
  // Run 464: one coast is retained after a large incoming peak, but a
  // second coast is replaced by a small startup trial once the return peak
  // has fallen. The replay does not predict the physical result.
  Controller early;early.peak(1,235,2.30266f);
  auto drive1=early.propose(input(-1,417));
  early.crossing(-1,417);early.confirm(-1,417,drive1,true);early.peak(-1,608,5.42757f);
  Input in2=input(1,814);in2.crossing_rate_dps=33.2032f;
  auto drive2=early.propose(in2);
  early.crossing(1,814);early.confirm(1,814,drive2,true);early.peak(1,995,6.32329f);
  Input in3=input(-1,1235);in3.crossing_rate_dps=34.2564f;
  auto first_coast=early.propose(in3);assert(first_coast.coast);
  early.crossing(-1,1235);early.confirm(-1,1235,first_coast,true);early.peak(-1,1421,3.86672f);
  Input in4=input(1,1640);in4.crossing_rate_dps=25.1239f;
  auto recovery=early.propose(in4);
  assert(recovery.selected && !recovery.coast && recovery.width_ms==15);
  in4.hardware_healthy=false;assert(!early.propose(in4).selected);
  // Regression for run 30224541: the kickoff peak is an isolated half-step,
  // and the big opposite peak forbids a second 1200mA pulse on the way back.
  Controller c;assert(c.target()==4.5f);
  c.peak(1,220,2.21185f);
  Input observed302=input(-1,414);observed302.crossing_rate_dps=18.645f;
  auto a=c.propose(observed302);
  assert(a.selected && !a.coast && a.current_ma==1200 && a.width_ms==59 && a.target_deg==4.5f);
  c.crossing(-1,414);c.confirm(-1,414,a,true);c.peak(-1,641,6.57367f);
  assert(c.target()==4.5f); // no complete post-start pair yet
  observed302=input(1,865);observed302.crossing_rate_dps=36.6997f;
  a=c.propose(observed302);
  assert(a.selected && a.coast && a.width_ms==0 && a.current_ma==0 && a.direction==1);
  // Actual zero-cross rates from runs 302 and 803. This uses only the
  // crossing sample; it cannot guarantee subsequent acceleration or foot speed.
  Controller rates;rates.peak(1,220,2.21185f);
  Input rate=input(-1,414);rate.crossing_rate_dps=18.645f;
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms==59);
  rate.crossing_rate_dps=14.3847f; // run 803, early positive crossing
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms==60);
  rate.crossing_rate_dps=32.496f; // run 803, next negative crossing
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms>20 && a.width_ms<60);
  rate.crossing_rate_dps=26.63f;
  auto b=rates.propose(rate);assert(b.selected && !b.coast && b.width_ms>a.width_ms && b.width_ms<60);
  rate.crossing_rate_dps=36.6997f; // run 302, return crossing
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms==39);
  rate.crossing_rate_dps=80.f;
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms==20);
  rate.hardware_cap_ms=15;
  a=rates.propose(rate);assert(a.selected && !a.coast && a.width_ms==15);
  rate.hardware_cap_ms=150;
  rate.crossing_rate_dps=NAN;assert(!rates.propose(rate).selected);
  rate.crossing_rate_dps=0;assert(!rates.propose(rate).selected);
  rate.crossing_rate_dps=INFINITY;assert(!rates.propose(rate).selected);
  // The recorded 803 crossing rate remains eligible for drive even when it
  // exceeds the previous 36 dps rate-only coast threshold. This does not
  // predict whether the real machine continues oscillating.
  Controller sustained;sustained.peak(1,1698,4.67317f);
  Input recorded803=input(-1,1858);recorded803.crossing_rate_dps=41.0152f;
  a=sustained.propose(recorded803);
  assert(a.selected && !a.coast && a.width_ms==34);
  Controller boundary;boundary.peak(1,100,6.f);
  assert(!boundary.propose(input(-1,300)).coast); // equality is permitted
  Controller excessive;excessive.peak(1,100,6.001f);
  a=excessive.propose(input(-1,300));assert(a.selected && a.coast);
  // Failed shortened commands cannot train the width or progress the stage.
  Controller rate_failed;rate_failed.peak(1,100,2.f);
  for(int i=0;i<4;++i) {
    int side=i%2?-1:1;uint32_t cross=300u+400u*i;
    Input ri=input(side,cross);ri.crossing_rate_dps=32.496f;
    auto shortened=rate_failed.propose(ri);
    // First side may have no preceding opposite measurement: fallback is not a stop.
    if(shortened.selected){rate_failed.crossing(side,cross);rate_failed.confirm(side,cross,shortened,false);}
    rate_failed.peak(side,cross+180,3.f);
  }
  assert(rate_failed.target()==4.5f && rate_failed.widthLimit(1)==60 && rate_failed.widthLimit(-1)==60);
  // Accepted, reduced-width pulses and their measured peaks can advance the
  // target after complete cycles, but cannot claim the 60 ms cap was tried.
  Controller reduced;reduced.peak(1,100,2.f);
  for(int i=0;i<4;++i) {
    int side=i%2?1:-1;uint32_t cross=300u+400u*i;
    Input ri=input(side,cross);ri.crossing_rate_dps=32.496f;
    auto shortened=reduced.propose(ri);
    assert(shortened.selected && !shortened.coast && shortened.width_ms<60);
    reduced.crossing(side,cross);reduced.confirm(side,cross,shortened,true);
    reduced.peak(side,cross+180,3.f);
  }
  assert(reduced.target()==5.f && reduced.widthLimit(1)==60 && reduced.widthLimit(-1)==60);
  // Even an accepted startup drive and a large first peak do not teach a
  // width adjustment; the initial half-step is not a completed cycle.
  Controller first;auto seed=first.propose(input(1,100));
  first.crossing(1,100);first.confirm(1,100,seed,true);first.peak(1,280,8.f);
  assert(first.widthLimit(1)==60 && first.target()==4.5f);
  // An omitted/failed write or an incomplete pair cannot train duration or
  // advance the angle stage; even an observed physical peak is not a send.
  Controller failed;failed.peak(1,100,2.f);
  a=failed.propose(input(-1,300));failed.crossing(-1,300);failed.confirm(-1,300,a,false);
  failed.peak(-1,480,3.f);assert(failed.widthLimit(-1)==60 && failed.target()==4.5f);
  failed.crossing(1,700);b=failed.propose(input(1,700));
  failed.confirm(1,700,b,false);failed.peak(1,880,4.f);
  assert(failed.widthLimit(1)==60 && failed.target()==4.5f);

  // Two actual, fully observed low-amplitude cycles increase each side's
  // independent width only one 10ms step, not immediately to 150ms.
  Controller learning;learning.peak(1,100,2.f);
  assert(step(learning,-1,300,3.f).width_ms==60);
  assert(step(learning,1,700,3.f).width_ms==60);
  assert(learning.target()==4.5f && learning.widthLimit(-1)==60);
  assert(step(learning,-1,1100,3.f).width_ms==60);
  assert(step(learning,1,1500,3.f).width_ms==60);
  assert(learning.widthLimit(-1)==70 && learning.widthLimit(1)==70);
  assert(learning.target()==5.f); // valid low peaks do not trap the ramp
  // Two more complete alternating pairs advance only another 0.5°.
  step(learning,-1,1900,4.3f);step(learning,1,2300,4.4f);
  assert(learning.target()==5.f);
  step(learning,-1,2700,4.3f);step(learning,1,3100,4.4f);
  assert(learning.target()==5.5f);
  // A continuously under-target observed walk can explore to the user's
  // ceiling without creating an "outside model range" stopping state.
  Controller persistent;persistent.peak(1,100,2.f);
  uint32_t t=300;
  for(int i=0;i<20;++i){
    auto minus=step(persistent,-1,t,3.f);assert(minus.width_ms<=150);t+=400;
    auto plus=step(persistent,1,t,3.f);assert(plus.width_ms<=150);t+=400;
  }
  assert(persistent.widthLimit(-1)==150 && persistent.widthLimit(1)==150);
  assert(persistent.target()==8.f);
  assert(persistent.propose(input(-1,t)).selected);

  // An over-target result shrinks the next proposal; both halves must be
  // genuinely measured before any outcome is accepted.
  Controller backoff;backoff.peak(1,100,2.f);
  step(backoff,-1,300,6.1f);assert(backoff.widthLimit(-1)==60);
  auto coast=step(backoff,1,700,3.f);assert(coast.coast);
  assert(backoff.widthLimit(-1)==50 && backoff.target()==4.5f);
  Controller hold;hold.peak(1,100,2.f);
  step(hold,-1,300,6.01f);step(hold,1,700,3.f);
  step(hold,-1,1100,3.f);step(hold,1,1500,3.f);
  assert(hold.target()==4.5f); // an excessive peak breaks the two-cycle sequence

  // An early peak limits later pulse duration, not the machine's operation.
  Controller timing;timing.peak(1,100,2.f);
  step(timing,-1,300,3.f,true,80);step(timing,1,700,3.f,true,80);
  a=timing.propose(input(-1,1100));assert(a.selected && a.width_ms==50);
  Input x=input(1,1000);x.hardware_healthy=false;assert(!timing.propose(x).selected);
  x=input(0,1100);assert(!timing.propose(x).selected);
  x=input(1,1100);x.hardware_cap_ms=0;assert(!timing.propose(x).selected);
  x=input(1,1100);x.preset_deg=10;assert(!timing.propose(x).selected);
  x=input(1,200);assert(!timing.propose(x).selected); // future measurement
  Controller stale;stale.peak(1,100,2.f);
  assert(!stale.propose(input(-1,1801)).selected); // ordinary control, not a stop
  Controller absent;assert(absent.propose(input(-1,100)).selected); // initial seed
  timing.reset();assert(timing.target()==4.5f && timing.widthLimit(-1)==60);
  std::cout<<"tilt ramp: startup half-step, paired evidence, coast and rollback PASS\n";
}
