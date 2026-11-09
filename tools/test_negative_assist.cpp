#include "wheel_work_energy_control.h"
#include "side_energy_feedback.h"
#include <cassert>
#include <iostream>
using namespace wheel_work_energy_control;
int main() {
  State s{66,125,7520,true,10};s.absolute_reference=true;s.target_deg=10;
  lateral_energy::CurrentTable table;assert(table.prepare(.02f));
  NegativeAssistFeedback f;
  const float bias=-.4f,target=1.2f;
  auto choose=[&](){return chooseNegativeBidirectional(s,25,f,table,0,300,target,bias,bias);};
  auto c=choose();assert(c.valid && c.width_ms==0 && c.target_above_reachable);
  f.observe(8,10,false,0);assert(!f.allowed());
  f.observe(8,10,false,0);assert(f.allowed());
  c=choose();assert(c.valid && c.branch==9 && c.command_sign==1 && c.width_ms<=25);
  assert(c.q_mas>0 && c.q_mas<=25 && c.cross_condition);
  assert(c.energy_mj>negativeBoundary(s,0,bias).energy_mj);
  assert(-1*c.command_sign==-1); // Negative body motion needs negative raw direction.
  assert(std::fabs(c.work_mj-c.on_mj-c.after_mj)<1e-5f);
  const auto coast=negativeAssistRaw(s,0,bias);
  assert(std::fabs(coast.energy_mj-negativeBoundary(s,0,bias).energy_mj)<1e-6f);
  const auto different_bias_zero=negativeAssistRaw(s,0,bias,bias-.2f);
  assert(std::fabs(different_bias_zero.energy_mj-coast.energy_mj)<1e-6f);
  const auto near_zero=negativeAssistRaw(s,.00001f,bias,bias-.2f);
  assert(std::fabs(near_zero.energy_mj-coast.energy_mj)<1e-5f);
  assert(!negativeAssistRaw(s,26,bias).valid);
  for(int i=0;i<40;++i)f.observe(8,10,true,f.capMs());
  assert(f.capMs()==100);
  c=choose();assert(c.width_ms<=100 && c.q_mas<=25 && c.current_ma==300);
  const auto brake=chooseNegativeBidirectional(s,25,f,table,0,300,.6f,bias,bias);
  assert(brake.valid && brake.command_sign==-1 && brake.branch!=9);
  assert(-1*brake.command_sign==1); // Braking reverses the body-motion command.
  s.target_deg=8;assert(choose().branch!=9); // Existing 8-degree mode retained.
  s.target_deg=10;s.wheel_fresh=false;assert(!choose().valid);
  s.wheel_fresh=true;s.battery_mv=8200;assert(!choose().valid);
  s.battery_mv=7520;s.rate_dps=93;assert(!choose().valid);
  f.observe(10,10,false,0);assert(f.allowed()); // Meeting target must not disable necessary sustaining input.
  lateral_energy::Geometry g;SideEnergyFeedback corrected,old;
  const float raw=1000*lateral_energy::potential(g,8)+.6f;
  assert(!old.observe(g,-1,8,raw,8));
  for(int i=0;i<3;++i)assert(corrected.observe(g,-1,8,raw,10));
  assert(std::fabs(corrected.biasMj(0,0)+.6f)<1e-5f);
  assert(!corrected.observe(g,-1,8,100,10));
  std::cout<<"negative shortage assistance, excess braking, signed output, whole work, staged caps, stale/range gates and target-scaled residuals PASS\n";
}
