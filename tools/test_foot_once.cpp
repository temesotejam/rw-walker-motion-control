#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static void cycle(Controller& x,unsigned t,float target,unsigned base=30,int ma=1200,float l=3,float r=1){
 auto w=x.limit(t+50,base,ma,1,80,true,target);x.confirm(t+50,true,ma,1,w);
 Cycle c;c.start_ms=t;c.end_ms=t+100;c.positive_ms=t+50;c.left=l;c.right=r;c.negative_peak=5;c.target_mj=target;
 c.negative_key=0x80000000u|(300u<<16)|0x8000u|50u;c.positive_width=w;c.positive_ma=ma;c.positive_sign=1;
 c.complete=c.negative_known=c.positive_executed=true;x.cycle(c);
}
static void arm(Controller& x){x.preferHighCurrentFirst();cycle(x,100,1);cycle(x,200,1);cycle(x,300,1);cycle(x,400,1.06f);}
int main(){
 Controller x;arm(x);assert(x.offset()==5 && x.state()==State::ProbeOnce);
 assert(x.limit(550,30,1200,1,80,true,1.06f)==35);x.confirm(550,true,1200,1,35);
 assert(x.acceptedProbeCycles()==1 && x.state()==State::ProbeOnce);
 assert(x.limit(650,30,1200,1,80,true,1.06f)==30);x.confirm(650,true,1200,1,30);
 assert(x.offset()==0 && x.acceptedProbeCycles()==2 && x.state()==State::Collect);
 for(unsigned t=700;t<=5500;t+=100){cycle(x,t,1.06f);assert(x.offset()==0);}
 cycle(x,5700,1.06f);cycle(x,5800,1.06f);cycle(x,5900,1.06f);assert(x.offset()==0);
 cycle(x,6000,1.06f);assert(x.offset()==5 && x.state()==State::ProbePlus);
 Controller strict;strict.preferHighCurrentFirst();for(unsigned t=100;t<=400;t+=100)cycle(strict,t,1);
 assert(strict.offset()==5 && strict.state()==State::ProbePlus);
 assert(strict.limit(550,30,1200,1,80,true,1)==35);strict.confirm(550,true,1200,1,35);
 assert(strict.limit(650,30,1200,1,80,true,1)==35);
 Controller bad;arm(bad);assert(bad.limit(550,50,1200,1,80,true,1.06f)==50);assert(bad.offset()==0&&bad.acceptedProbeCycles()==0);
 Controller other;other.preferHighCurrentFirst();cycle(other,100,1,30,400);cycle(other,200,1,30,400);cycle(other,300,1,30,400);cycle(other,400,1.06f,30,400);assert(other.offset()==0);
 Controller failed;arm(failed);assert(failed.limit(550,30,1200,1,80,true,1.06f)==35);failed.confirm(550,false,1200,1,35);assert(failed.fault());
 std::cout<<"one-shot correction, confirmed restore, cooldown and strict path PASS\n";
}
