#include "foot_span_control.h"
#include <cassert>
#include <cstdio>
using namespace foot_span_control;

static Cycle makeCycle(unsigned start,unsigned end,unsigned positive,bool valid=true) {
  Cycle c;c.start_ms=start;c.end_ms=end;c.positive_ms=positive;
  c.left=3;c.right=1;c.negative_peak=5;c.target_mj=1;
  c.negative_key=0x80000000u|(300u<<16)|0x8000u|50u;
  c.positive_width=30;c.positive_ma=1200;c.positive_sign=1;
  c.complete=valid;c.negative_known=c.positive_executed=true;return c;
}
static void ordinary(Controller& x,unsigned ms) {
  const auto w=x.limit(ms,30,1200,1,80,true,1);assert(w==30);
  x.confirm(ms,true,1200,1,w);
}
static void setup(Controller& x) {
  x.preferHighCurrentFirst();
  for(unsigned start=100;start<=300;start+=100) {
    ordinary(x,start+50);x.cycle(makeCycle(start,start+100,start+50));
  }
  ordinary(x,450);
  // The command for the next camera cycle is already in flight when the
  // delayed fourth cycle arrives and arms the intervention.
  ordinary(x,550);
  x.cycle(makeCycle(400,500,450));
  assert(x.offset()==5 && x.state()==State::ProbePlus);
  assert(x.limit(650,30,1200,1,80,true,1)==35);
  x.confirm(650,true,1200,1,35);
  assert(x.offset()==5 && x.acceptedProbeCycles()==1);
}
int main() {
  Controller stale;setup(stale);
  // Reproduce the log's delayed pre-intervention invalid foot cycle. It
  // cannot identify a response, but invalidates the trial reference and
  // therefore requests a return to the ordinary selector.
  stale.cycle(makeCycle(500,600,550,false));
  assert(stale.offset()==0 && stale.state()==State::Restore);
  assert(stale.acceptedProbeCycles()==1);
  assert(stale.limit(750,30,1200,1,80,true,1)==30);

  Controller equality;setup(equality);
  equality.cycle(makeCycle(600,650,625,false));
  assert(equality.offset()==0 && equality.state()==State::Restore);

  // Crossing the intervention timestamp is not stale, even when its start
  // was earlier. Invalid observations must still trigger immediate restore.
  Controller overlap;setup(overlap);
  overlap.cycle(makeCycle(600,700,625,false));
  assert(overlap.offset()==0 && overlap.state()==State::Restore);
  assert(overlap.limit(750,30,1200,1,80,true,1)==30);

  Controller post;setup(post);
  post.cycle(makeCycle(650,750,700,false));
  assert(post.offset()==0 && post.state()==State::Restore);

  // A malformed interval cannot bypass the invalid-observation protection.
  Controller malformed;setup(malformed);
  malformed.cycle(makeCycle(600,600,550,false));
  assert(malformed.offset()==0 && malformed.state()==State::Restore);

  // A proposal that has not actually been sent remains subject to normal
  // invalid-cycle cancellation.
  Controller unsent;unsent.preferHighCurrentFirst();
  for(unsigned start=100;start<=400;start+=100) {
    ordinary(unsent,start+50);unsent.cycle(makeCycle(start,start+100,start+50));
  }
  assert(unsent.offset()==5 && unsent.acceptedProbeCycles()==0);
  unsent.cycle(makeCycle(500,600,550,false));
  assert(unsent.offset()==0 && unsent.state()==State::Collect);
  std::puts("delayed pre-intervention invalid cycle: conservative restore PASS");
}
