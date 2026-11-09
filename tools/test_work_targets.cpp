#include "wheel_work_energy_control.h"
#include <cassert>
#include <iostream>
using namespace wheel_work_energy_control;
int main() {
  const lateral_energy::Geometry g;
  float previous_energy=0;
  for(float target : {8.f,10.f,12.f}) {
    assert(work_target_policy::packageAllowed(target,40,300,false));
    assert(!work_target_policy::packageAllowed(target,40,350,false));
    assert(!work_target_policy::packageAllowed(target,40,300,true));
    assert(work_target_policy::packageAllowed(target,25,300,false)==(target==8.f));
    const float energy=1000.f*lateral_cg::potential(g,target,0);
    assert(energy>previous_energy);previous_energy=energy;
    for(int side : {1,-1}) {
      const float signed_energy=lateral_cg::potential(g,side*target,.2f);
      assert(std::fabs(lateral_cg::angle(g,signed_energy,side,.2f)-target)<.001f);
    }
    State state{work_target_policy::rateCeiling(target,-1)-1.f,200,7800,true,target};
    state.absolute_reference=true;state.target_deg=target;
    assert(negativeBoundaryWeight(state)==1.f);
    const auto candidate=chooseNegativeBoundary(state,25,energy,0);
    assert(candidate.valid && candidate.width_ms<=25);
    if(target!=8.f) assert(candidate.cross_condition);
    state.rate_dps=work_target_policy::rateCeiling(target,-1)+2.1f;
    assert(!chooseNegativeBoundary(state,25,energy,0).valid);
    state.rate_dps=work_target_policy::rateCeiling(target,1)-1.f;
    assert(positiveWorkWeight(state)==1.f);
    if(target!=8.f) assert(positiveWorkRaw(state,12).cross_condition);
    NegativeFeedback guard;
    for(unsigned cap=25;cap<45;cap+=5) {
      guard.observe(energy+.1f,energy,energy,target,cap,true,target);
      guard.observe(energy+.1f,energy,energy,target,cap,true,target);
    }
    assert(guard.balancedCapMs(g,target,0,target)==45);
    assert(guard.balancedCapMs(g,target-1.01f,0,target)==0);
    const auto partial=guard.balancedCapMs(g,target-.6f,0,target);
    assert(partial>25 && partial<45);
  }
  assert(!work_target_policy::packageAllowed(9,40,300,false));
  State invalid{70,200,7800,true,8};invalid.absolute_reference=true;invalid.target_deg=9;
  assert(negativeBoundaryWeight(invalid)==0 && positiveWorkWeight(invalid)==0);
  std::cout<<"8/10/12 package validation, target potentials, bounded rate envelopes, target-relative reserve and extrapolation flags PASS\n";
}
