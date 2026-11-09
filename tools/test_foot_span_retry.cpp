#define FOOT_TREND_CYCLES 2
#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static uint16_t step(Controller& x,uint32_t start,bool valid=true) {
  const auto width=x.limit(start+50,30,1200,1,60,true,1);
  x.confirm(start+50,true,1200,1,width);
  Cycle c;c.start_ms=start;c.end_ms=start+100;c.positive_ms=start+50;
  c.left=3;c.right=1;c.negative_peak=5;c.target_mj=1;
  c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
  c.positive_width=width;c.positive_ma=1200;c.positive_sign=1;
  c.complete=valid;c.negative_known=c.positive_executed=true;x.cycle(c);return width;
}
int main(){
  for(int mode=0;mode<2;++mode){
    Controller x;x.preferHighCurrentFirst();
    assert(step(x,100)==30);assert(step(x,200)==30);assert(x.offset()==5);
    assert(step(x,300)==35);assert(x.acceptedProbeCycles()==1);
    if(mode==0)assert(step(x,400,false)==35);
    else {assert(x.cancelUnsent(30));}
    // Return to baseline is not delayed; it starts the five-second wait.
    assert(step(x,500)==30);assert(x.offset()==0);
    assert(step(x,1000)==30);assert(step(x,2000)==30);
    assert(step(x,4000)==30);assert(step(x,5500)==30);assert(x.offset()==0);
    // A cycle begun at 5 seconds after the confirmed restore is the first
    // eligible one; a second continuous full cycle is still required.
    assert(step(x,5550)==30);assert(x.offset()==0);
    assert(step(x,5650)==30);assert(x.offset()==5);
    assert(step(x,5750)==35);assert(x.acceptedProbeCycles()==3);
    x.reset();x.preferHighCurrentFirst();assert(step(x,100)==30);assert(step(x,200)==30);assert(x.offset()==5);
  }
  // In run309 the target changed from 0.53986 to 0.59995 mJ while a
  // 1200 mA correction was active. A changed target must restore the normal
  // width at the very next command, rather than repeat the old correction.
  Controller ramp;ramp.preferHighCurrentFirst();
  for(uint32_t start=100;start<=200;start+=100) {
    auto w=ramp.limit(start+50,20,1200,1,60,true,.53986f);ramp.confirm(start+50,true,1200,1,w);
    Cycle c;c.start_ms=start;c.end_ms=start+100;c.positive_ms=start+50;c.left=3;c.right=1;
    c.negative_peak=5;c.target_mj=.53986f;c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
    c.positive_width=w;c.positive_ma=1200;c.positive_sign=1;c.complete=c.negative_known=c.positive_executed=true;ramp.cycle(c);
  }
  assert(ramp.limit(350,20,1200,1,60,true,.53986f)==25);ramp.confirm(350,true,1200,1,25);
  assert(ramp.limit(450,20,1200,1,60,true,.59995f)==20);ramp.confirm(450,true,1200,1,20);
  assert(ramp.offset()==0 && !ramp.fault());
  // An unsent first proposal never starts a cooldown.
  Controller unsent;unsent.preferHighCurrentFirst();step(unsent,100);step(unsent,200);
  assert(unsent.cancelUnsent(30));step(unsent,300);step(unsent,400);assert(unsent.offset()==5);
  std::cout << "interrupted foot trial: confirmed restore, fresh-cycle cooldown and reset PASS\n";
}

