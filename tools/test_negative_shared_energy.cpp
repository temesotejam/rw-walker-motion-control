#include "wheel_work_energy_control.h"
#include "side_energy_feedback.h"
#include <cassert>
#include <iostream>
using namespace wheel_work_energy_control;
int main() {
  State s{77,4,7570,true,12.2f};s.absolute_reference=true;s.target_deg=12;
  const float bias=-.18f;
  const auto coast=negativeBoundary(s,0,bias);assert(coast.valid);
  float last=coast.energy_mj;
  for(unsigned i=0;i<=250;++i) {
    const float q=i*.1f;
    const auto p=negativeAssistRaw(s,q,bias,-.6f);
    assert(p.valid && p.energy_mj>=last);
    assert(std::fabs(p.energy_mj-(coast.energy_mj+.22502117220f*.06189960458f*q*s.rate_dps*kRadPerDeg))<1e-5f);
    assert(p.applied_bias_mj==bias);
    assert(std::fabs(p.work_mj-p.on_mj-p.after_mj)<1e-5f);
    last=p.energy_mj;
  }
  assert(work_target_policy::negativeFeedbackIndex(12,false)==2);
  assert(work_target_policy::negativeFeedbackIndex(12,true)==2);
  assert(work_target_policy::negativeFeedbackIndex(10,true)==3);
  lateral_energy::CurrentTable table;assert(table.prepare(.06f));
  NegativeAssistFeedback f;
  f.observe(11,12,false,0);f.observe(11,12,false,0);
  f.observe(11,12,true,25);f.observe(12,12,false,0);f.observe(11,12,true,25);
  assert(f.capMs()==30); // An unassisted period does not erase a measured limited assist.
  auto c=chooseNegativeBidirectional(s,25,f,table,0,250,coast.energy_mj+.5f,bias,-.6f);
  assert(c.valid && c.branch==9 && c.width_ms==30 && c.command_sign==1 && c.target_above_reachable);
  assert(c.current_ma==300 && c.q_mas<=25);
  const auto brake=chooseNegativeBidirectional(s,25,f,table,0,250,coast.energy_mj-.1f,bias,-.6f);
  assert(brake.valid && brake.branch!=9 && brake.command_sign==-1);
  f.observe(13,12,true,30);assert(f.capMs()==25);
  NegativeAssistFeedback larger;
  for(unsigned i=0;i<14;++i)larger.observe(11,12,true,larger.capMs());
  assert(larger.capMs()==60);
  const auto reachable=chooseNegativeBidirectional(s,25,larger,table,0,250,coast.energy_mj+.06f,bias,-.6f);
  assert(reachable.valid && reachable.branch==9 && reachable.width_ms>0 && reachable.width_ms<larger.capMs());
  NegativeAssistFeedback historical;
  historical.observe(8,10,true,25);historical.observe(10,10,false,0);historical.observe(8,10,true,25);
  assert(historical.capMs()==25);
  s.wheel_fresh=false;assert(!chooseNegativeBidirectional(s,25,f,table,0,250,2,bias,bias).valid);
  s.wheel_fresh=true;s.battery_mv=8200;assert(!chooseNegativeBidirectional(s,25,f,table,0,250,2,bias,bias).valid);
  const lateral_energy::Geometry g;SideEnergyFeedback shared;
  const float u=1000*lateral_energy::potential(g,11);
  for(unsigned i=0;i<3;++i)assert(shared.observe(g,-1,11,u+.2f,12));
  assert(std::fabs(shared.biasMj(0,0)+.2f)<1e-5f);
  std::cout<<"target12 shared negative energy offset, increasing predicted assist work, endpoint selection, interleaved cap evidence, excess braking, historical behavior and hard gates PASS\n";
}
