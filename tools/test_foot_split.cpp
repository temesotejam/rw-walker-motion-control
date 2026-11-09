#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static uint16_t tick(Controller& x,uint32_t t,float l,float r,float target,float negtrigger=0, int base=30, bool valid=true){
 auto w=x.limit(t+50,base,1200,1,60,true,target);x.confirm(t+50,true,1200,1,w);
 Cycle c;c.start_ms=t;c.end_ms=t+100;c.positive_ms=t+50;c.left=l;c.right=r;c.negative_peak=5;c.target_mj=target;
 c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;c.negative_trigger_deg=negtrigger;
 c.positive_width=w;c.positive_ma=1200;c.positive_sign=1;c.complete=valid;c.negative_known=c.positive_executed=true;
 x.cycle(c);return w;
}
int main(){
 Controller x;x.preferHighCurrentFirst();
 // Mixed older targets and timings remain useful as trend, while only the
 // last two mutually comparable cycles form the before-intervention baseline.
 tick(x,100,3,1,.60f,0,29);tick(x,200,3,1,.66f,.2f,30);
 tick(x,300,3,1,.73f,1,31);tick(x,400,3,1,.73f,1.1f,31);
 assert(x.offset()==5);
 assert(x.limit(550,40,1200,1,60,true,.73f)==40); // new 9ms normal change: no probe
 x.confirm(550,true,1200,1,40);assert(x.offset()==0);
 // Feed the already-confirmed ordinary command as its complete foot cycle.
 // The new observation can support a single-shot proposal; the four-cycle
 // history must survive rejection of the unsent candidate.
 Cycle c;c.start_ms=500;c.end_ms=600;c.positive_ms=550;c.left=3;c.right=1;
 c.negative_peak=5;c.target_mj=.73f;c.negative_key=0x80000000u | (300u<<16) | 0x8000u | 80u;
 c.negative_trigger_deg=1.1f;c.positive_width=40;c.positive_ma=1200;c.positive_sign=1;
 c.complete=c.negative_known=c.positive_executed=true;x.cycle(c);
 assert(x.offset()==5 && x.state()==State::ProbeOnce);
 assert(tick(x,600,3,1,.73f,1.1f,40)==45);
 assert(x.limit(750,40,1200,1,60,true,.73f)==40);

 Controller disagree;disagree.preferHighCurrentFirst();
 tick(disagree,100,6,1,.6f);tick(disagree,200,6,1,.66f);
 tick(disagree,300,1,2,.73f);tick(disagree,400,1,2,.73f);
 assert(disagree.offset()==0); // long positive, recent negative: do not chase
 // Even if the latest cycle agrees with the long trend, a complete
 // two-cycle reference that disagrees must not be bypassed by a one-shot.
 Controller conflicted;conflicted.preferHighCurrentFirst();
 tick(conflicted,100,6,1,1);tick(conflicted,200,6,1,1);
 tick(conflicted,300,1,6,1);tick(conflicted,400,2,1,1);
 assert(conflicted.offset()==0);
 Controller mismatch;mismatch.preferHighCurrentFirst();
 tick(mismatch,100,3,1,.6f);tick(mismatch,200,3,1,.66f);
 tick(mismatch,300,3,1,.73f,0);tick(mismatch,400,3,1,.73f,1);
 assert(mismatch.offset()==5 && mismatch.state()==State::ProbeOnce);
 assert(mismatch.limit(550,30,1200,1,60,true,.73f)==35);
 mismatch.confirm(550,true,1200,1,35);
 assert(mismatch.limit(650,30,1200,1,60,true,.73f)==30);
 mismatch.confirm(650,true,1200,1,30);
 assert(mismatch.offset()==0 && !mismatch.fault());
 // Waiting for fresh cycles after the confirmed restoration does not
 // prevent ordinary pulses and does not immediately repeat the test.
 assert(tick(mismatch,700,3,1,.73f,1)==30);assert(mismatch.offset()==0);
 Controller missing;missing.preferHighCurrentFirst();
 tick(missing,100,3,1,.6f);tick(missing,200,3,1,.66f);
 tick(missing,300,3,1,.73f,0,30,false);tick(missing,400,3,1,.73f);
 tick(missing,500,3,1,.73f);tick(missing,600,3,1,.73f);
 assert(missing.offset()==0);tick(missing,700,3,1,.73f);assert(missing.offset()==5);
 std::cout<<"foot trend and matched reference separation PASS\n";
}
