#include "side_energy_feedback.h"
#include "wheel_work_energy_control.h"
#include <cassert>
#include <iostream>
using namespace wheel_work_energy_control;
int main() {
  const lateral_energy::Geometry g;
  const auto u=[&](float angle){return 1000.f*lateral_energy::potential(g,angle);};
  assert(work_target_policy::observationCeiling(8)==13);
  assert(work_target_policy::observationCeiling(10)==13);
  assert(work_target_policy::observationCeiling(12)==15);
  SideEnergyFeedback f;
  // Synthetic large underprediction below target: protection starts at the
  // first completed peak, before the ordinary two-observation mean is ready.
  const float raw=u(10.8f)-.45f;
  assert(f.observe(g,1,10.8f,raw,12,0,0));
  assert(f.count()==1 && f.biasMj(0,0)==0);
  assert(std::fabs(f.positiveBiasMj(0,0,12)-.45f)<1e-5f);
  assert(f.positiveBiasMj(0,0,8)==0 && f.positiveBiasMj(0,0,10)==0);
  assert(f.observe(g,1,13.6f,u(13.6f)-.7f,12,0,.45f));
  assert(f.count()==2);
  assert(f.observe(g,1,14.2f,u(14.2f)-.75f,12,0,.6f));
  assert(f.count()==3 && !f.coastOnly());
  for(int i=0;i<3;++i)assert(f.observe(g,1,13.2f,u(13.2f)-.6f,12,0,0));
  assert(std::fabs(f.positiveBiasMj(0,0,12)-.6f)<1e-5f); // MAX, never double the residual.
  const float cg=.2f;
  const float expected=.6f+g.mass_g*std::sin(13.2f*lateral_energy::kRadPerDeg)*cg;
  assert(std::fabs(f.positiveBiasMj(cg,0,12)-expected)<1e-5f);
  SideEnergyFeedback historical;
  assert(!historical.observe(g,1,13.6f,u(13.6f)-.2f,8));
  assert(!historical.observe(g,1,13.6f,u(13.6f)-.2f,10));
  assert(!historical.coastOnly());
  assert(!f.observe(g,1,15.2f,u(15.2f)-.4f,12));
  assert(f.coastOnly());
  lateral_energy::CurrentTable storage[3];
  const lateral_energy::CurrentTable* tables[]={&storage[0],&storage[1],&storage[2]};
  float goals[]={250,270,290};
  for(auto& t:storage)assert(t.prepare(.05f));
  State s{78,170,7750,true,13.8f};s.absolute_reference=true;s.target_deg=12;
  SideEnergyFeedback single;
  assert(single.observe(g,1,10.8f,u(10.8f)-.45f,12));
  const auto unguarded=choosePositive(g,s,0,tables,goals,u(12),0,0);
  const auto guarded=choosePositive(g,s,0,tables,goals,u(12),single.positiveBiasMj(0,0,12),0);
  assert(unguarded.valid && guarded.valid && guarded.q_mas<unguarded.q_mas);
  const auto stopped=choosePositive(g,s,0,tables,goals,u(12),0,0,f.coastOnly());
  assert(stopped.valid && stopped.width_ms==0 && stopped.q_mas==0);
  // A finite but unmodelled energy residual also blocks further excitation.
  assert(!f.observe(g,1,14,0,12));assert(f.coastOnly());
  assert(f.observe(g,1,12,u(12)-.2f,12,0,.2f));assert(!f.coastOnly());
  assert(!f.observe(g,1,NAN,0,12));assert(!f.coastOnly());
  NegativeAssistFeedback assist;
  for(int i=0;i<4;++i)assist.observe(11,12,true,assist.capMs());
  const auto cap=assist.capMs();assert(cap>25);
  assist.observe(13.6f,12,true,cap);assert(assist.capMs()==cap-5);
  std::cout<<"target12 completed peaks to15deg, first-error energy guard, no double counting, CG re-expression, coast/recovery, unchanged old ranges and assist overshoot reduction PASS\n";
}
