#include "positive_sway_probe.h"
#include <cassert>
using positive_sway_probe::Controller;
static void c(Controller& x,unsigned start,float l=2,float r=2,float p=4,float m=6,bool valid=true,bool neutral=true,bool effective=false) {
  x.cycle(start,start+700,l,r,p,m,valid,true,neutral,effective);
}
int main(){
  Controller x;for(unsigned i=0;i<4;++i)c(x,100+i*700);
  assert(x.candidate(3000,true,true));assert(!x.candidate(3000,true,false));
  x.confirm(3010,20,20,true,true);assert(!x.attempted());
  x.confirm(3020,20,24,false,true);assert(!x.attempted());
  x.confirm(3030,20,24,true,true);assert(x.active()&&x.attempted()&&x.effectiveCommand(3030));
  c(x,2900,2,2,4,6,true,true,false);assert(x.active()); // overlaps send
  c(x,3100,2,2,4.1,6.1,true,true,true);assert(x.active());
  c(x,3800,2,2,4.1,6.1,true,true,true);assert(!x.active()&&x.attempted());
  assert(!x.candidate(4600,true,true)); // at most one accepted trial
  Controller loss;for(unsigned i=0;i<4;++i)c(loss,100+i*700);
  loss.confirm(3030,20,24,true,true);c(loss,3100,.1f,2,4,6,true,true,true);assert(!loss.active());
  Controller invalid;for(unsigned i=0;i<4;++i)c(invalid,100+i*700);
  invalid.confirm(3030,20,24,true,true);c(invalid,3100,2,2,4,6,false,true,true);assert(!invalid.active());
  Controller ineffective;for(unsigned i=0;i<4;++i)c(ineffective,100+i*700);
  ineffective.confirm(3030,20,24,true,true);c(ineffective,3100,2,2,4,6,true,true,false);assert(!ineffective.active());
  Controller gap;c(gap,100);c(gap,800);c(gap,1600);c(gap,2300);assert(!gap.candidate(3100,true,true));
  Controller busy;for(unsigned i=0;i<4;++i)c(busy,100+i*700);
  busy.confirm(3030,20,24,true,true);assert(!busy.request(3100,true,false));
  Controller timeout;for(unsigned i=0;i<4;++i)c(timeout,100+i*700);
  timeout.confirm(3030,20,24,true,true);assert(!timeout.request(6031,true,true));
  Controller duplicate;for(unsigned i=0;i<4;++i)c(duplicate,100+i*700);
  duplicate.confirm(3030,20,24,true,true);c(duplicate,3100,2,2,4,6,true,true,true);
  c(duplicate,3100,2,2,4,6,true,true,true);assert(!duplicate.active());
  Controller response_gap;for(unsigned i=0;i<4;++i)c(response_gap,100+i*700);
  response_gap.confirm(3030,20,24,true,true);c(response_gap,3100,2,2,4,6,true,true,true);
  c(response_gap,3900,2,2,4,6,true,true,true);assert(!response_gap.active());
  // Observed 58304881 camera cycles and corresponding body peaks. This is
  // an observational eligibility check, not a physical counterfactual;
  // the live observer also requires its command/context and idle checks.
  Controller loglike;
  loglike.cycle(24152,24877,.445f,2.57419f,3.75986f,6.16996f,true,true,true,false);
  loglike.cycle(24877,25581,1.25419f,1.58805f,4.24965f,6.19736f,true,true,true,false);
  loglike.cycle(25581,26291,1.48794f,2.0269f,4.3995f,6.56935f,true,true,true,false);
  loglike.cycle(26291,26991,1.40561f,1.48861f,4.05615f,6.00941f,true,true,true,false);
  assert(loglike.candidate(27904,true,true));
}
