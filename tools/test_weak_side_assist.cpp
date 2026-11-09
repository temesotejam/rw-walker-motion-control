#include "weak_side_assist.h"
#include <cassert>
#include <cmath>
#include <iostream>
using weak_side_assist::Input;
using weak_side_assist::choose;
int main() {
  Input x;x.target_preset_deg=8.f;x.active_target_deg=5.5f;x.physical_next_side=-1;
  x.last_negative_peak_deg=3.44f;x.last_negative_peak_ms=21098;x.now_ms=21726;
  x.negative_peak_count=29;x.crossing_rate_dps=22.6566f;x.wheel_rpm=264.3f;
  x.battery_mv=7870.f;x.last_positive_peak_deg=5.57611f;
  x.hardware_healthy=true;x.speed_fresh=true;x.selector_valid=true;x.selector_width_ms=0;x.hardware_max_width_ms=70;
  auto a=choose(x);assert(a.selected && a.width_ms==43 && a.current_ma==300 && a.motion_sign==1);
  // Reproduce low-rpm measured coast with NO learned-model range restriction.
  x.now_ms=24787;x.last_negative_peak_ms=24184;x.last_negative_peak_deg=3.64441f;x.wheel_rpm=171.72f;x.selector_valid=false;
  a=choose(x);assert(a.selected && a.width_ms==40);
  x.wheel_rpm=-10.f;assert(choose(x).selected); // negative wheel direction is NOT a support cutoff
  // A known powered normal choice is retained, even if it is opposing/braking.
  x.selector_valid=true;x.selector_width_ms=45;assert(!choose(x).selected);
  x.selector_width_ms=0;x.last_negative_peak_deg=5.1f;assert(!choose(x).selected);
  x.last_negative_peak_deg=5.0f;assert(!choose(x).selected); // equality at deadband
  x.last_negative_peak_deg=1.0f;a=choose(x);assert(a.selected && a.width_ms==70);
  x.hardware_max_width_ms=35;a=choose(x);assert(a.selected && a.width_ms==35);
  x.hardware_max_width_ms=19;assert(choose(x).selected && choose(x).width_ms==19);
  x.hardware_max_width_ms=0;assert(!choose(x).selected);
  x.hardware_max_width_ms=70;x.hardware_healthy=false;assert(!choose(x).selected);
  x.hardware_healthy=true;x.speed_fresh=false;assert(!choose(x).selected);
  x.speed_fresh=true;x.wheel_rpm=NAN;assert(!choose(x).selected);
  x.wheel_rpm=170.f;x.battery_mv=0;assert(!choose(x).selected);
  x.battery_mv=7870.f;x.crossing_rate_dps=NAN;assert(!choose(x).selected);
  x.crossing_rate_dps=25.f;x.physical_next_side=1;assert(!choose(x).selected);
  x.physical_next_side=-1;x.target_preset_deg=10.f;assert(!choose(x).selected);
  x.target_preset_deg=8.f;x.negative_peak_count=1;assert(!choose(x).selected);
  x.negative_peak_count=2;x.last_negative_peak_ms=x.now_ms+1;assert(!choose(x).selected);
  x.last_negative_peak_ms=x.now_ms-1501;assert(!choose(x).selected);
  x.last_negative_peak_ms=x.now_ms-1500;assert(choose(x).selected);
  x.last_positive_peak_deg=16.f;assert(!choose(x).selected);
  x.last_positive_peak_deg=5.f;x.last_negative_peak_deg=NAN;assert(!choose(x).selected);
  // Feedback advances only from confirmed commands and a fresh, time-matched
  // subsequent negative peak. These are synthetic observations, not proof
  // of physical peak or foot response.
  weak_side_assist::Controller ctrl;
  Input y;y.target_preset_deg=8;y.active_target_deg=8;y.physical_next_side=-1;
  y.last_negative_peak_deg=4.5f;y.last_negative_peak_ms=900;y.now_ms=1000;
  y.negative_peak_count=2;y.crossing_rate_dps=30;y.wheel_rpm=300;y.battery_mv=8000;
  y.last_positive_peak_deg=5.3f;y.hardware_healthy=true;y.speed_fresh=true;
  y.selector_valid=false;y.selector_width_ms=0;y.hardware_max_width_ms=150;
  // At full target, a 2-degree deficit has a 43ms proportional width but must
  // seed the 50ms experiment so that observations can eventually advance it.
  y.last_negative_peak_deg=6.f;
  auto seed=ctrl.propose(y);assert(seed.selected && seed.width_ms==50);
  y.hardware_max_width_ms=42;seed=ctrl.propose(y);assert(seed.selected && seed.width_ms==42);
  y.hardware_max_width_ms=150;y.last_negative_peak_deg=4.5f;
  auto c=ctrl.propose(y);assert(c.selected && c.width_ms==50);
  // No confirmed command means no trial, even after repeated observations.
  for(int i=0;i<4;i++) {y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==50);
  // An active ramp target below 8 degrees never triggers exploration.
  y.active_target_deg=7.5f;
  for(int i=0;i<4;i++) {c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==50);y.active_target_deg=8;
  for(int i=0;i<2;i++) {c=ctrl.propose(y);assert(c.width_ms==50);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;c=ctrl.propose(y);}
  assert(ctrl.limitMs()==60 && c.width_ms==60);
  // A trial that is followed by two comparable weak peaks rolls back.
  for(int i=0;i<2;i++) {c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;y.last_negative_peak_deg=4.0f;ctrl.propose(y);}
  assert(ctrl.limitMs()==50);
  // Repeated weakness no longer raises the width during the four-cycle pause.
  for(int i=0;i<3;i++) {c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==50);
  ctrl.reset();y.last_negative_peak_deg=4.5f;
  // Continue the plateau experiment up to (never above) the hardware cap.
  y.hardware_max_width_ms=75;
  for(int i=0;i<16;i++) {c=ctrl.propose(y);assert(c.width_ms<=75);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==75);
  ctrl.reset();y.hardware_max_width_ms=150;
  for(int i=0;i<40;i++) {c=ctrl.propose(y);assert(c.width_ms<=150);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==150);
  ctrl.reset();y.hardware_max_width_ms=150;
  for(int i=0;i<2;i++) {c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);}
  assert(ctrl.limitMs()==60);
  c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);ctrl.interrupt();assert(ctrl.limitMs()==50);
  // After interruption, forced exploration must not survive the abandoned
  // trial; ordinary deficit-based output is used even with a high old cap.
  y.last_negative_peak_deg=7.25f;
  c=ctrl.propose(y);assert(c.selected && c.width_ms==24);
  y.last_negative_peak_deg=4.5f;
  // An old or missing peak, a changing target, an unhealthy sensor, or a
  // normal powered selector never become evidence and never latch a stop.
  c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);y.now_ms+=800;
  ctrl.propose(y);assert(ctrl.limitMs()==50);
  y.last_negative_peak_ms=y.now_ms-100;
  c=ctrl.propose(y);ctrl.confirm(y,c.width_ms);y.now_ms+=800;
  y.last_negative_peak_ms=y.now_ms-100;y.active_target_deg=7.5;
  ctrl.propose(y);assert(ctrl.limitMs()==50);
  y.active_target_deg=8;y.hardware_healthy=false;assert(!ctrl.propose(y).selected);
  y.hardware_healthy=true;y.selector_valid=true;y.selector_width_ms=40;
  assert(!ctrl.propose(y).selected);ctrl.interrupt();
  y.selector_valid=false;y.selector_width_ms=0;assert(ctrl.propose(y).selected);
  // A mismatched send width is rejected instead of counted as an experiment.
  ctrl.reset();c=ctrl.propose(y);ctrl.confirm(y,c.width_ms+1);
  y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;ctrl.propose(y);
  assert(ctrl.limitMs()==50);
  // The 50ms seed cannot override an existing downward backoff.
  ctrl.reset();y.last_negative_peak_deg=6.f;
  for(int i=0;i<2;i++) {c=ctrl.propose(y);assert(c.width_ms==50);ctrl.confirm(y,c.width_ms);
    y.now_ms+=800;y.last_negative_peak_ms=y.now_ms-100;y.last_negative_peak_deg-=.2f;ctrl.propose(y);}
  assert(ctrl.limitMs()==40);c=ctrl.propose(y);assert(c.selected && c.width_ms==40);
  std::cout << "weak side assist: plateau exploration, rollback and guards PASS\n";
}
