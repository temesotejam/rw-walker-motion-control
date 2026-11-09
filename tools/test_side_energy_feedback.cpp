#include "side_energy_feedback.h"
#include "wheel_work_energy_control.h"
#include "work_start_policy.h"
#include <cassert>
#include <iostream>
int main() {
  using namespace wheel_work_energy_control;
  assert(work_start_policy::referenceAllowed(true,false));
  assert(!work_start_policy::referenceAllowed(false,false));
  assert(work_start_policy::referenceAllowed(false,true));
  assert(work_start_policy::ready(true,true,true,true,true));
  assert(!work_start_policy::ready(true,false,true,true,true));
  assert(!work_start_policy::ready(true,true,false,true,true));
  assert(!work_start_policy::ready(true,true,true,true,false));
  const lateral_energy::Geometry g;
  SideEnergyFeedback positive,negative;
  const float peak=9.f;
  const float raw=1000.f*lateral_energy::potential(g,peak)-.22f;
  for(int i=0;i<3;++i) assert(positive.observe(g,1,peak,raw));
  assert(negative.count()==0); // A missing opposite side does not freeze this side.
  assert(std::fabs(positive.biasMj(0,0)-.22f)<1e-5f);
  const float coefficient=g.mass_g*std::sin(peak*lateral_energy::kRadPerDeg);
  assert(std::fabs(positive.biasMj(.3f,.1f)-(.22f+coefficient*.3f))<1e-5f);
  // Raw prediction + bias equals the measured CG-corrected energy exactly.
  assert(std::fabs(raw+positive.biasMj(.3f,0)-1000.f*lateral_cg::potential(g,peak,.3f))<1e-5f);
  for(int i=0;i<3;++i) assert(negative.observe(g,-1,peak,raw));
  assert(std::fabs(negative.biasMj(.3f,0)-(.22f-coefficient*.3f))<1e-5f);
  assert(!positive.observe(g,1,9,100));
  assert(!positive.observe(g,1,NAN,raw));
  State context{68,0,8000,true,9.3f};
  assert(negativeBoundaryWeight(context)==0); // Historical contract retained.
  context.absolute_reference=true;
  assert(negativeBoundaryWeight(context)==1.f);
  const float target=1000.f*lateral_energy::potential(g,8.f);
  const auto choice=chooseNegativeBoundary(context,25,target,0);
  assert(choice.valid && choice.width_ms>0 && choice.width_ms<=25);
  assert(choice.cross_condition);
  assert(choice.energy_mj<negativeBoundary(context,0).energy_mj);
  context.wheel_fresh=false;assert(!chooseNegativeBoundary(context,25,target,0).valid);
  context.wheel_fresh=true;context.battery_mv=8200;
  assert(!chooseNegativeBoundary(context,25,target,0).valid);
  context.battery_mv=8000;context.previous_peak_deg=10.4f;
  assert(!chooseNegativeBoundary(context,25,target,0).valid);
  std::cout<<"reference-free work start, independent energy correction, CG re-expression and absolute-context brake PASS\n";
}
