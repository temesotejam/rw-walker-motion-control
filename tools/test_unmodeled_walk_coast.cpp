#include "unmodeled_walk_coast.h"
#include <cassert>
#include <cmath>
#include <iostream>

using wheel_work_energy_control::State;
static State state(float rate,float rpm,float previous) {
  State s{rate,rpm,8060.f,true,previous};
  s.target_deg=8.f;s.absolute_reference=true;return s;
}
int main() {
  // Real unsupported negative decisions in 39993895.
  assert(unmodeled_walk_coast::eligible(state(33.7186f,287.04f,4.18894f),.38619f,true));
  assert(unmodeled_walk_coast::eligible(state(38.6994f,431.63f,4.46858f),.38619f,true));
  assert(unmodeled_walk_coast::eligible(state(29.2212f,171.7f,4.82969f),.38619f,true));
  // A supported state belongs to the measured selector, not this fallback.
  assert(!unmodeled_walk_coast::eligible(state(36.4829f,441.63f,4.45538f),.38619f,true));
  assert(!unmodeled_walk_coast::eligible(state(33.7186f,287.04f,4.18894f),.38619f,false));
  auto s=state(33.7186f,287.04f,4.18894f);
  s.wheel_fresh=false;assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  s=state(NAN,287.04f,4.18894f);assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  s=state(33.7186f,NAN,4.18894f);assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  s=state(33.7186f,287.04f,NAN);assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  s=state(33.7186f,287.04f,4.18894f);s.battery_mv=0;
  assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  s=state(33.7186f,287.04f,4.18894f);
  assert(!unmodeled_walk_coast::eligible(s,NAN,true));
  s.target_deg=10.f;assert(!unmodeled_walk_coast::eligible(s,.38619f,true));
  std::cout << "unmodeled walking coast: coverage and physical safety separation PASS\n";
}
