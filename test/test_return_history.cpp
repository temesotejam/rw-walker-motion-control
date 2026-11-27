#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define WALKING_RETURN_HISTORY 1
#include "walking/required_work.h"
#include <cassert>

using receiving_arc_aid::ReturnHistory;
using receiving_arc_aid::ReturnState;

static const ReturnState first{1936,66,5.349f,20.643f,5.007f,-.110f,-.444f,77.21f,-1.904f,.000343854f};
static const ReturnState second{3690,46,5.324f,19.311f,4.258f,-.084f,-.283f,129.92f,-1.906f,.000343854f};
static const ReturnState third{5438,44,4.813f,19.581f,3.666f,-.102f,-.248f,59.754f,-1.907f,.000343854f};

static void addFirst(ReturnHistory& h) {
  h.completedAid(first,2121,28,true);
  h.peak(-1,2519,3.1f,first.departure_ms,2336,true);
}
static void addSecond(ReturnHistory& h) {
  h.completedAid(second,3878,33,true);
  h.peak(-1,4242,2.7f,second.departure_ms,4051,true);
}
int main() {
  ReturnHistory h;
  addFirst(h);
  assert(!h.compare(third,5592).first_peak_ms);
  addSecond(h);
  const auto hit=h.compare(third,5592);
  assert(hit.first_peak_ms==2519 && hit.second_peak_ms==4242 && hit.limit_ms==28);
  assert(hit.deceleration==first.deceleration && hit.request_j>0 && hit.request_j<=.5f*third.target_j);
  auto state=third;state.deceleration=80;assert(!h.compare(state,5592).request_j);
  state=third;state.image_age_ms=40;assert(!h.compare(state,5592).limit_ms);
  state=third;state.left_deg+=4;assert(!h.compare(state,5592).limit_ms);
  state=third;state.right_deg-=2.1f;assert(!h.compare(state,5592).limit_ms);
  state=third;state.body_deg+=2;assert(!h.compare(state,5592).limit_ms);
  state=third;state.fore_aft_deg+=2;assert(!h.compare(state,5592).limit_ms);
  state=third;state.speed_dps-=3;assert(!h.compare(state,5592).limit_ms);
  state=third;state.target_j*=2;assert(!h.compare(state,5592).limit_ms);
  state=third;state.terminal_deg=NAN;assert(!h.compare(state,5592).limit_ms);
  state=third;state.image_age_ms=76;assert(!h.compare(state,5592).limit_ms);
  state=third;state.departure_ms=6500;assert(!h.compare(state,6520).limit_ms);

  for (unsigned failure=0;failure<7;++failure) {
    ReturnHistory broken;addFirst(broken);
    broken.completedAid(second,3878,failure==0?81:33,failure!=1);
    assert(!broken.compare(third,5592).limit_ms);
    broken.peak(failure==2?1:-1,4242,failure==3?1.89f:2.7f,
        failure==4?1936:3690,failure==5?3800:4051,failure!=6);
    assert(!broken.compare(third,5592).limit_ms);
  }
  // Replayed evidence from the same departure cannot make an independent second observation.
  ReturnHistory repeated;addFirst(repeated);addFirst(repeated);
  assert(!repeated.compare(third,5592).limit_ms);
}
