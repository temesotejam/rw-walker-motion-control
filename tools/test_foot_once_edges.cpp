#include "foot_span_control.h"
#include <cassert>
using namespace foot_span_control;
static void make(Controller& x,unsigned t,float target,int ma=1200){
 auto w=x.limit(t+50,30,ma,1,30,true,target);x.confirm(t+50,true,ma,1,w);
 Cycle c;c.start_ms=t;c.end_ms=t+100;c.positive_ms=t+50;c.left=3;c.right=1;c.negative_peak=5;c.target_mj=target;
 c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 50u;c.positive_width=w;c.positive_ma=ma;c.positive_sign=1;
 c.complete=c.negative_known=c.positive_executed=true;x.cycle(c);
}
int main(){
 Controller x;x.preferHighCurrentFirst();make(x,100,1);make(x,200,1);make(x,300,1);make(x,400,1.06f);
 assert(x.state()==State::ProbeOnce && x.offset()==5);
 assert(x.limit(550,30,1200,1,30,true,1.06f)==25 && x.state()==State::ProbeOnce);x.confirm(550,true,1200,1,25);
 assert(x.limit(650,30,1200,1,30,true,1.06f)==30);x.confirm(650,true,1200,1,30);assert(!x.fault());
 Controller stale;stale.preferHighCurrentFirst();make(stale,100,1);make(stale,200,1);make(stale,300,1);make(stale,400,1.06f);
 assert(stale.limit(4000,30,1200,1,80,true,1.06f)==30 && stale.offset()==0);
 Controller fail;fail.preferHighCurrentFirst();make(fail,100,1);make(fail,200,1);make(fail,300,1);make(fail,400,1.06f);
 assert(fail.limit(550,30,1200,1,80,true,1.06f)==35);fail.confirm(550,true,1200,1,35);
 assert(fail.limit(650,30,1200,1,80,true,1.06f)==30);fail.confirm(650,false,1200,1,30);assert(fail.fault());
}
