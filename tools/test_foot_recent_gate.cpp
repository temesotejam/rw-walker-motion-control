#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;

static void cycle(Controller& x, unsigned t, float target, float left, float right) {
  const auto w=x.limit(t+50,30,1200,1,80,true,target);
  x.confirm(t+50,true,1200,1,w);
  Cycle c;
  c.start_ms=t; c.end_ms=t+100; c.positive_ms=t+50;
  c.left=left; c.right=right; c.negative_peak=5; c.target_mj=target;
  c.negative_key=0x80000000u|(300u<<16)|0x8000u|50u;
  c.positive_width=w; c.positive_ma=1200; c.positive_sign=1;
  c.complete=c.negative_known=c.positive_executed=true;
  x.cycle(c);
}
int main() {
  // The long trend and two-cycle average remain imbalanced, but the newest
  // completed cycle is balanced. Preserve ordinary motion.
  Controller strict; strict.preferHighCurrentFirst();
  for(unsigned t=100;t<=300;t+=100) cycle(strict,t,1,3,1);
  cycle(strict,400,1,2.01f,2);
  assert(strict.offset()==0 && strict.acceptedProbeCycles()==0);
  // A renewed same-sign mismatch reopens the gate without discarding trend.
  cycle(strict,500,1,3,1);
  assert(strict.offset()==5 && strict.state()==State::ProbePlus);

  // A latest opposite-side mismatch also blocks a stale correction even
  // when older samples keep the four-cycle average positive.
  Controller reversed; reversed.preferHighCurrentFirst();
  for(unsigned t=100;t<=300;t+=100) cycle(reversed,t,1,3,1);
  cycle(reversed,400,1,1,1.5f);
  assert(reversed.offset()==0 && reversed.acceptedProbeCycles()==0);

  // Mixed contexts must not bypass the same recent-cycle gate.
  Controller once; once.preferHighCurrentFirst();
  for(unsigned t=100;t<=300;t+=100) cycle(once,t,1,3,1);
  cycle(once,400,1.06f,2.01f,2);
  assert(once.offset()==0);
  cycle(once,500,1.12f,3,1);
  assert(once.offset()==5 && once.state()==State::ProbeOnce);
  std::cout << "fresh-cycle gate protects balanced and reversed motion PASS\n";
}
