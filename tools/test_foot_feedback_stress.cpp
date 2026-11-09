#include "foot_span_control.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

static uint32_t seed=0x6345a12fu;
static uint32_t next(){seed=1664525u*seed+1013904223u;return seed;}
int main(){
  foot_span_control::Controller controller;controller.preferHighCurrentFirst();unsigned changed=0,invalid=0;
  for(uint32_t i=1;i<=50000;++i){
    const uint32_t start=100+i*100,mid=start+50,end=start+100;
    controller.preferHighCurrentFirst(); // must not reset a learned reversal
    const uint16_t base=static_cast<uint16_t>(29+next()%3),cap=60;
    const uint16_t width=controller.limit(mid,base,1200,1,cap,true,1.f);
    assert(width>=10 && width<=cap);
    if(width!=base)++changed;
    controller.confirm(mid,true,1200,1,width);
    foot_span_control::Cycle c;
    c.start_ms=start;c.end_ms=end;c.positive_ms=mid;
    c.positive_width=width;c.positive_ma=1200;c.positive_sign=1;
    c.complete=c.negative_known=c.positive_executed=true;
    c.negative_key=0x80000000u|(1200u<<16)|0x8000u|80u;
    c.target_mj=1.f;c.negative_peak=4.f+static_cast<float>(next()%30)*.1f;
    c.left=2.f+static_cast<float>(next()%40)*.05f;
    c.right=.3f+static_cast<float>(next()%35)*.05f;
    c.negative_trigger_deg=static_cast<float>(next()%7)*.1f;
    c.negative_rate_dps=30.f+static_cast<float>(next()%9);
    c.negative_rpm=-50.f+static_cast<float>(next()%50);
    if(next()%17==0){c.complete=false;++invalid;}
    if(next()%41==0){c.negative_trigger_deg=std::numeric_limits<float>::quiet_NaN();++invalid;}
    if(next()%71==0)c.negative_rate_dps+=20.f; // incomparable, not a fault
    controller.cycle(c);
    assert(!controller.fault());
    assert(controller.offset()>=-30 && controller.offset()<=30);
  }
  assert(changed>0 && invalid>0);
  std::printf("50000 variable 1200mA cycles: bounded proposals, missing data and recovery PASS (%u changes)\n",changed);
}
