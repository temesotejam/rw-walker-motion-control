#define FOOT_TREND_CYCLES 2
#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static Cycle cycle(unsigned t,float l,float r) {
  Cycle c;c.start_ms=t;c.end_ms=t+100;c.positive_ms=t+50;
  c.left=l;c.right=r;c.negative_peak=5;c.target_mj=1;
  c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
  c.positive_width=30;c.positive_ma=1200;c.positive_sign=1;
  c.complete=c.negative_known=c.positive_executed=true;return c;
}
int main(){
  for (int kind=0;kind<3;++kind) {
    Controller x;x.preferHighCurrentFirst();
    // Three ordinary commands arrive before delayed camera cycles are processed.
    for(unsigned t=150;t<=350;t+=100){assert(x.limit(t,30,1200,1,60,true,1)==30);x.confirm(t,true,1200,1,30);}
    x.cycle(cycle(100,3,1));x.cycle(cycle(200,3,1));assert(x.offset()==5);
    if(kind==0)x.cycle(cycle(300,2,2));
    if(kind==1)x.cycle(cycle(300,1,3));
    if(kind==2)x.cycle(cycle(300,3,1));
    const auto requested=x.limit(450,30,1200,1,60,true,1);
    assert(requested==(kind==2?35:30));
    assert(x.acceptedProbeCycles()==0 && !x.fault());
    x.confirm(450,true,1200,1,requested);
    assert(x.acceptedProbeCycles()==(kind==2?1u:0u));
  }
  // A queued exploration must preserve a confirmed offset when canceled;
  // a pending safety rollback must never be canceled by new balance.
  for(int rollback=0;rollback<2;++rollback) {
    Controller x;x.preferHighCurrentFirst();
    for(unsigned t=100;t<=200;t+=100) {
      auto w=x.limit(t+50,30,1200,1,60,true,1);x.confirm(t+50,true,1200,1,w);x.cycle(cycle(t,3,1));
    }
    for(unsigned t=300;t<=600;t+=100) {
      auto w=x.limit(t+50,30,1200,1,60,true,1);assert(w==35);x.confirm(t+50,true,1200,1,w);
      if(t<500){auto c=cycle(t,rollback?1.5f:2.5f,rollback?.5f:1.1f);c.positive_width=w;x.cycle(c);}
    }
    auto c5=cycle(500,rollback?1.5f:2.5f,rollback?.5f:1.1f);c5.positive_width=35;x.cycle(c5);
    assert(x.offset()==(rollback?0:10));
    auto c6=cycle(600,1,3);c6.positive_width=35;x.cycle(c6);
    assert(x.offset()==(rollback?0:5));
    assert(x.limit(750,30,1200,1,60,true,1)==(rollback?30:35));
  }
  std::cout << "queued foot correction: balanced or reversed latest cycle cancels proposal PASS\n";
}
