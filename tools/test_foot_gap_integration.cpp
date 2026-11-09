#include "foot_span_control.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace foot_span_control;
static void period(Controller& x,unsigned start,unsigned end,float left,float right,
                   bool quality,bool strict=true,int ma=1200,float target=1.f,float rate=30.f,bool healthy=true) {
  const unsigned now=start+300;
  const auto w=x.limit(now,30,ma,1,80,true,target,rate,0);
  x.confirm(now,true,ma,1,w);
  Cycle c;c.start_ms=start;c.end_ms=end;c.positive_ms=now;
  c.left=left;c.right=right;c.negative_peak=5;c.target_mj=target;
  c.negative_key=0x80000000u|(300u<<16)|0x8000u|50u;
  c.positive_width=w;c.positive_ma=ma;c.positive_sign=1;
  c.negative_rate_dps=30;c.positive_rate_dps=rate;
  c.negative_known=c.positive_executed=true;c.gap_allow=healthy;
  c.gap_quality=quality;c.complete=quality && strict && healthy;x.cycle(c);
}
static void seed(Controller& x) {
  // Foot spans/boundaries resemble run 343; command times and input contexts
  // below are synthetic. This is not a replay or a causal outcome estimate.
  x.preferHighCurrentFirst();
  period(x,17127,17982,3.10016f,2.72944f,true);
  period(x,17982,18834,3.84595f,.49921f,true);
  period(x,18834,19614,0,0,false);
  period(x,19614,20511,0,0,false);
  period(x,20511,21291,0,0,false);
  period(x,21291,22160,3.93578f,1.12996f,true);
  period(x,22160,22939,2.53153f,.21827f,true);
}
int main() {
  Controller x;seed(x);
  assert(x.state()==State::ProbeOnce && x.offset()==5);
  assert(x.limit(23140,30,1200,1,80,true,1,30,0)==35);
  x.confirm(23140,true,1200,1,35);
  assert(x.acceptedProbeCycles()==1);
  assert(x.limit(23950,30,1200,1,80,true,1,30,0)==30);
  x.confirm(23950,true,1200,1,30);
  assert(x.offset()==0 && x.acceptedProbeCycles()==2);
  for(unsigned t=24000;t<28950;t+=800) {
    const auto w=x.limit(t,30,1200,1,80,true,1,30,0);assert(w==30);x.confirm(t,true,1200,1,w);
  }
  Controller stale;seed(stale);
  assert(stale.limit(24500,30,1200,1,80,true,1,30,0)==30 && stale.offset()==0);
  Controller changed;seed(changed);
  assert(changed.limit(23140,30,400,1,80,true,1,30,0)==30 && changed.offset()==0);
  Controller interrupted;interrupted.preferHighCurrentFirst();
  period(interrupted,17127,17982,3,1,true);
  period(interrupted,17982,18834,3,1,true);
  period(interrupted,18834,19614,0,0,false);
  period(interrupted,19614,20511,0,0,false);
  period(interrupted,20511,21291,0,0,false);
  period(interrupted,21291,22160,3,1,true);
  period(interrupted,22160,22939,3,1,true,true,1200,1.2f);
  assert(interrupted.offset()==0); // target change breaks history
  Controller context;context.preferHighCurrentFirst();
  period(context,17127,17982,3,1,true);
  period(context,17982,18834,3,1,true);
  period(context,18834,19614,0,0,false);
  period(context,19614,20511,0,0,false);
  period(context,20511,21291,0,0,false);
  period(context,21291,22160,3,1,true,true,1200,1.f,30.f);
  period(context,22160,22939,3,1,true,true,1200,1.f,44.f);
  assert(context.offset()==5);
  // 58 is close to the latest 44 but far from the preceding 30.
  assert(context.limit(23140,30,1200,1,80,true,1,58,0)==30 && context.offset()==0);
  Controller failed;seed(failed);
  assert(failed.limit(23140,30,1200,1,80,true,1,30,0)==35);
  failed.confirm(23140,false,1200,1,35);assert(failed.fault());
  Controller before;seed(before);
  Cycle missing;missing.start_ms=22939;missing.end_ms=23781;
  before.cycle(missing);assert(before.offset()==0);
  assert(before.limit(23900,30,1200,1,80,true,1,30,0)==30);
  Controller after;seed(after);
  assert(after.limit(23140,30,1200,1,80,true,1,30,0)==35);
  after.confirm(23140,true,1200,1,35);after.cycle(missing);
  assert(after.limit(23900,30,1200,1,80,true,1,30,0)==30);
  after.confirm(23900,true,1200,1,30);
  assert(after.offset()==0 && after.acceptedProbeCycles()==2);
  Controller unhealthy;unhealthy.preferHighCurrentFirst();
  period(unhealthy,17127,17982,3,1,true);
  period(unhealthy,17982,18834,3,1,true);
  period(unhealthy,18834,19614,0,0,false,true,1200,1.f,30.f,false);
  period(unhealthy,19614,20511,0,0,false);
  period(unhealthy,20511,21291,0,0,false);
  period(unhealthy,21291,22160,3,1,true);
  period(unhealthy,22160,22939,3,1,true);
  assert(unhealthy.offset()==0); // unhealthy period clears old evidence
  std::cout << "gap trend one-shot, confirmed restore, expiry and context PASS\n";
}
