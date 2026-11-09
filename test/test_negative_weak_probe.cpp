#include "../src/negative_weak_probe.h"
#include <cassert>
using negative_weak_probe::Controller;
static void pair(Controller& p,unsigned t,unsigned width=57,float plus=7.f,float minus=4.f) {
  p.command(1,t,57,60,true);p.peak(1,t+100,plus,8,true,true);
  p.command(-1,t+200,width,60,true);p.peak(-1,t+300,minus,8,true,true);
}
int main() {
  Controller p;
  for(unsigned i=0;i<3;++i)pair(p,1000+i*800);
  assert(!p.candidate(4000,8,60,true));
  pair(p,3400);assert(p.candidate(3800,8,60,true));
  assert(!p.candidate(6000,8,60,true));
  assert(!p.candidate(3800,8,60,false));
  assert(!p.candidate(3800,8,70,true));
  p.attempt(3800,false,57,60);assert(!p.attempted());
  p.attempt(3800,true,57,57);assert(!p.attempted());
  p.attempt(3800,true,57,63);assert(!p.attempted());
  p.attempt(3800,true,57,60);assert(p.applied() && p.attempted());
  assert(!p.candidate(3800,8,60,true));
  p.reset();for(unsigned i=0;i<4;++i)pair(p,1000+i*800,54);
  assert(!p.candidate(3800,8,60,true));
  p.reset();for(unsigned i=0;i<3;++i)pair(p,1000+i*800);
  p.command(1,3400,57,60,true);p.peak(1,3500,7,8,true,true);
  p.command(-1,3600,57,60,false);p.peak(-1,3700,4,8,true,true);
  assert(!p.candidate(3800,8,60,true));
}
