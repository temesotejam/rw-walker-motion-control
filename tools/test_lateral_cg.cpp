#include "lateral_cg_energy.h"
#include "wheel_work_energy_control.h"
#include <Preferences.h>
#include <cassert>
#include <iostream>
int main() {
  const lateral_energy::Geometry g;
  for(float a : {0.f,.1f,1.f,2.f,8.f,10.f,12.f}) {
    assert(lateral_cg::potential(g,a,0)==lateral_energy::potential(g,a));
    assert(lateral_cg::angle(g,lateral_energy::potential(g,a),1,0)==
        lateral_energy::angle(g,lateral_energy::potential(g,a)));
  }
  for(float x : {-2.f,-1.f,-.1f,.1f,1.f,2.f}) for(int side : {-1,1}) {
    for(float a : {8.f,10.f,12.f}) {
      const float e=lateral_cg::potential(g,side*a,x);
      assert(e>0 && std::fabs(lateral_cg::angle(g,e,side,x)-a)<.001f);
      assert(std::fabs(e-lateral_cg::potential(g,-side*a,-x))<1e-9f);
    }
  }
  const float ep=lateral_cg::potential(g,8,1)*1000;
  const float en=lateral_cg::potential(g,-8,1)*1000;
  assert(ep>en && std::fabs(ep-en-2*g.mass_g*std::sin(8*lateral_energy::kRadPerDeg))<1e-4f);
  float shift=99;
  assert(!lateral_cg::fromEquilibrium(g,0,shift) && shift==99);
  assert(!lateral_cg::fromEquilibrium(g,1,shift));
  assert(!lateral_cg::fromEquilibrium(g,10,shift));
  assert(lateral_cg::fromEquilibrium(g,2.5f,shift) && shift<0);
  const float eps=.001f;
  const float derivative=(lateral_cg::potential(g,2.5f+eps,shift)-
      lateral_cg::potential(g,2.5f-eps,shift))/(2*eps);
  assert(std::fabs(derivative)<1e-7f);
  assert(lateral_cg::potential(g,2.5f+eps*100,shift)>lateral_cg::potential(g,2.5f,shift));
  assert(std::isnan(lateral_cg::potential(g,8,3)));
  assert(std::isnan(lateral_cg::angle(g,.001f,0,1)));
  lateral_cg::Record r,loaded;
  assert(!lateral_cg::load(loaded));
  r.magic=lateral_cg::kMagic;r.revision=1;r.upright_revision=3;
  r.source=1;r.shift_mm=-.5f;r.upright_abs_deg=.7f;
  assert(lateral_cg::save(r) && lateral_cg::load(loaded));
  assert(lateral_cg::matches(loaded,3,.7f));
  assert(!lateral_cg::matches(loaded,4,.7f));
  assert(!lateral_cg::matches(loaded,3,.8f));
  Preferences::fail_write=true;r.shift_mm=.5f;
  assert(!lateral_cg::save(r) && lateral_cg::load(loaded) && loaded.shift_mm==-.5f);
  Preferences::fail_write=false;
  // Same measured pre-input state: changing the physical energy target changes
  // the selected input; energy is never replaced by an angle-error command.
  using namespace wheel_work_energy_control;
  const State state{65,100,7800,true,8};
  Choice low,high;
  for(uint16_t ms=0;ms<=70;ms=(ms==0 ? 10 : ms+1)) {
    const auto prediction=negative(state,ms,0);
    consider(low,300,ms,NAN,prediction,en);
    consider(high,300,ms,NAN,prediction,ep);
  }
  finish(low,en);finish(high,ep);
  assert(low.valid && high.valid && low.width_ms>=high.width_ms);
  std::cout << "lateral CG: zero equivalence, signed targets, inverse, static identifiability, persistence, input direction PASS\n"
      << "1 mm targets: positive=" << ep << " negative=" << en << " mJ\n";
}
