#include "foot_span_candidate.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace wheel_work_energy_control;
static bool close(float a,float b){return std::fabs(a-b)<1e-5f;}
int main(){
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable tables[3];
  assert(tables[0].prepare(.05f)&&tables[1].prepare(.07f)&&tables[2].prepare(.09f));
  const float goals[]={270.f,320.f,360.f};
  const float work_bias=.023f,bridge_bias=-.017f,initial=25.f,target=1.0f;
  const State states[]={State(25.f,100.f,7500.f,true,5.f),
                        State(55.f,100.f,7500.f,true,5.f),
                        State(46.3821f,100.f,7500.f,true,5.f)};
  const uint8_t expected[]={5,6,7};
  for(int i=0;i<3;++i) {
    State s=states[i];s.absolute_reference=true;s.target_deg=8.f;
    for(int ci=0;ci<3;++ci) {
      const int16_t ma=static_cast<int16_t>(300+50*ci);
      const auto& table=tables[ci];const float goal=goals[ci];
      const float q0=lateral_energy::charge(table,70,initial,goal);
      const auto normal=positiveBoundary(geometry,s,q0,work_bias,bridge_bias);
      assert(normal.valid&&normal.branch==expected[i]);
      Choice baseline;consider(baseline,ma,70,q0,normal,target);baseline.allowed_cap_ms=90;
      const auto q=lateral_energy::charge(table,75,initial,goal);
      const auto p=positiveBoundary(geometry,s,q,work_bias,bridge_bias);
      const auto candidate=foot_span_candidate::choose(geometry,s,baseline,table,75,initial,goal,target,work_bias,bridge_bias);
      assert(candidate.valid&&candidate.branch==expected[i]);
      assert(candidate.current_ma==ma&&candidate.width_ms==75&&candidate.allowed_cap_ms==75);
      assert(close(candidate.q_mas,q)&&close(candidate.energy_mj,p.energy_mj));
      assert(close(candidate.applied_bias_mj,p.applied_bias_mj));
      assert(!foot_span_candidate::choose(geometry,s,baseline,table,91,initial,goal,target,work_bias,bridge_bias).valid);
      assert(!foot_span_candidate::choose(geometry,s,baseline,table,9,initial,goal,target,work_bias,bridge_bias).valid);
      Choice wrong=baseline;wrong.branch=expected[(i+1)%3];
      assert(!foot_span_candidate::choose(geometry,s,wrong,table,75,initial,goal,target,work_bias,bridge_bias).valid);
      wrong=baseline;wrong.command_sign=-1;
      assert(!foot_span_candidate::choose(geometry,s,wrong,table,75,initial,goal,target,work_bias,bridge_bias).valid);
      wrong=baseline;wrong.current_ma=450;
      assert(!foot_span_candidate::choose(geometry,s,wrong,table,75,initial,goal,target,work_bias,bridge_bias).valid);
    }
  }
  std::cout << "positive span candidate: branches 5/6/7, biases, current, caps and mismatch PASS\n";
}
