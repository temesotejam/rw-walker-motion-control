#include "control_angle_reference.h"
#include "adaptive_lateral_cg.h"
#include <cassert>
#include <iostream>
int main() {
  for (float saved : {-3.f, 0.f, .57f, 2.2f}) {
    const float origin=control_angle_reference::origin(true,saved);
    assert(origin==0.f);
    assert(8.f-origin==8.f && -8.f-origin==-8.f);
    assert(control_angle_reference::origin(false,saved)==saved);
  }
  lateral_cg::Record old;
  old.magic=lateral_cg::kMagic;old.revision=1;old.upright_revision=1;old.source=1;
  old.shift_mm=.3f;old.upright_abs_deg=0.f;
  assert(lateral_cg::valid(old) && !lateral_cg::matchesAbsolute(old));
  old.version=2;
  assert(lateral_cg::matchesAbsolute(old));
  old.upright_abs_deg=.57f;
  assert(!lateral_cg::matchesAbsolute(old));
  lateral_cg::Adaptive model; model.reset(0.f,0.f);
  assert(model.snapshot().shift_mm==0.f && model.snapshot().common_bias_mj==0.f);
  const lateral_energy::Geometry g;
  const float e=lateral_cg::potential(g,8.f,0.f);
  assert(e==lateral_cg::potential(g,-8.f,0.f));
  for (int side : {1,-1}) {
    const float target=lateral_cg::potential(g,side*8.f,.3f);
    assert(std::fabs(lateral_cg::angle(g,target,side,.3f)-8.f)<.001f);
  }
  assert(lateral_cg::potential(g,8.f,.3f)>lateral_cg::potential(g,-8.f,.3f));
  std::cout << "absolute origin invariance, old-prior rejection and signed energy inversion PASS\n";
}
