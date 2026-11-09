#include <cassert>
#include <cmath>
#include <initializer_list>
#include "wheel_work_energy_control.h"
#include "lateral_energy_control.h"

int main() {
  using namespace wheel_work_energy_control;
  const State positive_state{54.f,300.f,7350.f,true};
  assert(supported(positive_state,1));
  assert(!positive(positive_state,kPositiveMinQ-.01f).valid);
  const auto p_low=positive(positive_state,9.f);
  const auto p_high=positive(positive_state,15.f);
  assert(p_low.valid && p_high.valid && p_high.energy_mj>p_low.energy_mj);
  Choice positive_choice;
  consider(positive_choice,300,70,9.f,p_low,p_high.energy_mj);
  consider(positive_choice,400,90,15.f,p_high,p_high.energy_mj);
  finish(positive_choice,p_high.energy_mj);
  assert(positive_choice.valid && positive_choice.current_ma==400 &&
      positive_choice.width_ms==90 && !positive_choice.target_above_reachable);

  const State state{69.2351f,218.97f,7150.f,true,7.88428f};
  assert(supported(state,-1));
  const auto no_input=negative(state,0);
  assert(no_input.valid && no_input.branch==4 && std::fabs(no_input.change_mj)<1e-6f);
  float last=no_input.energy_mj;
  Choice all;
  for (uint16_t width=0;width<=40;width=(width==0 ? 10 : width+1)) {
    const auto p=negative(state,width);
    assert(p.valid && p.energy_mj<=last);
    assert(std::fabs(p.work_mj-p.on_mj-p.after_mj)<1e-6f);
    assert(std::fabs(p.change_mj-(p.work_mj-no_input.work_mj))<1e-6f);
    assert(std::fabs(p.energy_mj-(p.free_mj+.44113787836f*p.change_mj))<1e-6f);
    assert(p.cross_condition==(width>25));
    consider(all,300,width,NAN,p,.73342f);
    last=p.energy_mj;
  }
  finish(all,.73342f);
  assert(all.valid && all.width_ms==40 && all.target_below_reachable);
  assert(!negative(state,71).valid && !negative(state,1).valid);
  assert(!negative(state,25,NAN).valid);
  assert(negative(state,25).on_mj<0 && negative(state,25).after_mj>0);
  assert(std::fabs(negative(state,25,.05f).energy_mj-negative(state,25).energy_mj-.05f)<1e-6f);
  const State measured_strong{68.f,350.f,7400.f,true,8.f};
  assert(!negative(measured_strong,40).cross_condition);
  assert(!supported(State{74.f,40.f,7100.f,false,8.f},-1));
  assert(!supported(State{74.f,40.f,7100.f,true},-1));
  assert(!supported(State{74.f,40.f,6200.f,true,8.f},-1));
  assert(supported(State{74.f,40.f,6210.f,true,8.f},-1));
  assert(supported(State{74.f,40.f,8000.f,true,8.f},-1));
  assert(supported(State{74.f,40.f,8110.f,true,8.f},-1));
  assert(supported(State{74.f,40.f,8120.f,true,8.f},-1));
  assert(!supported(State{74.f,40.f,8121.f,true,8.f},-1));
  assert(fullChargeCorrectionMj(7990.f)==0.f);
  assert(std::fabs(fullChargeCorrectionMj(8050.f)+.0298612584f)<1e-6f);
  assert(std::fabs(fullChargeCorrectionMj(8110.f)+.05972251687f)<1e-6f);
  const State full_charge{72.f,60.f,8100.f,true,8.f};
  assert(negative(full_charge,0).valid && !negative(full_charge,0).cross_condition);
  for (uint16_t ms : {10,25,30,35,40}) {
    assert(negative(full_charge,ms).valid && negative(full_charge,ms).cross_condition);
    assert(negative(full_charge,ms).energy_mj<negative(full_charge,0).energy_mj);
  }
  assert(!supported(State{77.f,40.f,7100.f,true,8.f},-1));
  assert(!supported(State{74.f,3.f,7100.f,true,8.f},-1));

  PositiveFeedback positive_feedback;
  assert(positive_feedback.biasMj()==0.f);
  for (unsigned i=0;i<12;++i) {
    const float raw=.8f;
    const float corrected=raw+positive_feedback.biasMj();
    // Runtime strips previously applied bias before observing again.
    positive_feedback.observe(.74f,corrected-positive_feedback.biasMj());
  }
  assert(std::fabs(positive_feedback.biasMj()+.06f)<1e-6f);
  const auto valid_count=positive_feedback.count();
  positive_feedback.observe(NAN,.8f);
  positive_feedback.observe(.7f,NAN);
  assert(positive_feedback.count()==valid_count);
  PositiveFeedback separate_work_branch;
  assert(separate_work_branch.biasMj()==0.f);
  for (unsigned i=0;i<3;++i) positive_feedback.observe(.2f,.8f);
  assert(std::fabs(positive_feedback.biasMj()+.25f)<1e-6f);
  positive_feedback=PositiveFeedback{};
  assert(positive_feedback.count()==0 && positive_feedback.biasMj()==0.f);
  // Underprediction and overprediction change the energy inverse in opposite directions.
  lateral_energy::CurrentTable table;
  assert(table.prepare(.07f));
  const lateral_energy::Geometry g;
  const float target=lateral_energy::potential(g,8.f);
  const auto nominal=lateral_energy::decideBridge(g,table,1,54.f,target,0.f,0.f,300.f);
  const auto more=lateral_energy::decideBridge(g,table,1,54.f,target,0.f,0.f,300.f,-.00006f);
  const auto less=lateral_energy::decideBridge(g,table,1,54.f,target,0.f,0.f,300.f,.00006f);
  assert(nominal.valid && more.valid && less.valid);
  assert(more.requested_q_mAs>nominal.requested_q_mAs);
  assert(less.requested_q_mAs<nominal.requested_q_mAs);

  NegativeFeedback feedback;
  assert(feedback.capMs(8.f)==25 && feedback.biasMj()==0.f);
  feedback.observe(1.14f,1.08f,.733f,8.f,25);
  assert(feedback.capMs(8.f)==25 && feedback.biasMj()==0.f);
  feedback.observe(1.14f,1.08f,.733f,8.f,25);
  assert(feedback.capMs(8.f)==30 && std::fabs(feedback.biasMj()-.06f)<1e-6f);
  feedback.observe(1.14f,1.08f,.733f,8.f,25);
  assert(feedback.capMs(8.f)==30); // No growth unless the current cap was used.
  for (uint16_t cap : {30,35,40,45,50,55,60,65}) {
    feedback.observe(1.1f,1.05f,.733f,8.f,cap);
    assert(feedback.capMs(8.f)==cap);
    feedback.observe(1.1f,1.05f,.733f,8.f,cap);
    assert(feedback.capMs(8.f)==cap+5);
  }
  for (unsigned i=0;i<20;++i) feedback.observe(1.1f,1.05f,.733f,8.f,70);
  assert(feedback.capMs(8.f)==70 && feedback.capMs(7.6f)==40 && feedback.capMs(7.4f)==25);
  feedback.observe(.65f,.65f,.733f,8.f,70);
  assert(feedback.capMs(8.f)==65);
  feedback.observe(1.1f,1.05f,.733f,7.4f,25);
  assert(feedback.capMs(8.f)==25);
  const auto samples=feedback.count();
  feedback.observe(NAN,1.0f,.733f,8.f,25);
  assert(feedback.count()==samples && feedback.capMs(8.f)==25);
  for (unsigned i=0;i<4;++i) feedback.observe(1.5f,1.f,.733f,8.f,25);
  assert(std::fabs(feedback.biasMj()-.12f)<1e-6f);

  NegativeFeedback consistent;
  for (unsigned i=0;i<12;++i) {
    const float applied_bias=consistent.biasMj();
    const float corrected_prediction=1.f+applied_bias;
    consistent.observe(1.05f,corrected_prediction-applied_bias,.733f,8.f,25);
  }
  assert(std::fabs(consistent.biasMj()-.05f)<1e-6f);

  // Runtime float regression against the latest five measured 25-ms peaks.
  // Whole-run held-out accuracy is checked separately by the fitting script.
  const float actual[][5] = {
      {69.2351f,218.97f,7150.f,7.88428f,10.34158f},
      {69.9325f,224.45f,7150.f,7.92085f,10.43093f},
      {69.0678f,251.69f,7160.f,7.98930f,10.55294f},
      {69.3050f,245.62f,7150.f,7.97437f,10.48583f},
      {70.7654f,236.37f,7150.f,7.95699f,10.51178f}};
  float squared_error=0.f;
  const lateral_energy::Geometry geometry;
  for (const auto& row : actual) {
    const State x{row[0],row[1],row[2],true,row[3]};
    const auto p=negative(x,25);
    const float predicted=lateral_energy::angle(geometry,p.energy_mj*.001f);
    squared_error+=(predicted-row[4])*(predicted-row[4]);
    assert(negative(x,40).energy_mj<p.energy_mj);
  }
  assert(std::sqrt(squared_error/5.f)<.15f);

  // Causal replay of recorded positive peaks, run 107786469 (1.0.26).
  // Evaluates prediction correction on the original inputs, not a closed-loop simulation.
  const float recorded[][3]={
      {864.f,0.288500000f,0.201410000f},
      {1663.f,0.288520000f,0.227970000f},
      {2426.f,0.290290000f,0.257420000f},
      {3155.f,0.350790000f,0.290060000f},
      {3876.f,0.451170000f,0.353810000f},
      {4646.f,0.537680000f,0.411600000f},
      {5484.f,0.631520000f,0.517240000f},
      {6415.f,0.734680000f,0.649670000f},
      {7338.f,0.735190000f,0.654170000f},
      {8261.f,0.741650000f,0.671700000f},
      {9178.f,0.740310000f,0.661750000f},
      {10097.f,0.744960000f,0.670820000f},
      {11014.f,0.747410000f,0.692510000f},
      {11928.f,0.747070000f,0.697520000f},
      {12841.f,0.742950000f,0.695930000f},
      {13760.f,0.751190000f,0.700480000f},
      {14688.f,0.749390000f,0.672960000f},
  };
  PositiveFeedback replay;
  float before=0.f,after=0.f;
  unsigned evaluated=0;
  for (const auto& row : recorded) {
    const float bias=replay.biasMj(); // only earlier completed peaks
    if (row[0]>=10000.f) {
      before+=(row[2]-row[1])*(row[2]-row[1]);
      after+=(row[2]-row[1]-bias)*(row[2]-row[1]-bias);
      ++evaluated;
    }
    replay.observe(row[2],row[1]);
  }
  assert(evaluated>=5 && after<before*.5f);

  // All 22 full-charge coast states from run 69890231; future peak is a label only.
  const float coast[][5]={
      {69.7648f,126.53f,8100.0f,7.87608f,11.01608f},
      {68.0071f,5.08f,8100.0f,8.16258f,10.22045f},
      {69.1148f,114.1f,8100.0f,7.81279f,10.95255f},
      {69.9403f,41.38f,8100.0f,8.13755f,10.80819f},
      {69.5775f,69.07f,8100.0f,7.81705f,11.21942f},
      {72.7185f,52.49f,8100.0f,7.86459f,11.54426f},
      {71.7123f,74.96f,8100.0f,7.63654f,11.78907f},
      {73.1078f,74.74f,8100.0f,7.75522f,12.03167f},
      {75.1802f,71.39f,8100.0f,7.94507f,12.18258f},
      {75.9844f,66.78f,8100.0f,8.27243f,12.0617f},
      {74.2108f,48.72f,8100.0f,8.45512f,11.75084f},
      {71.7286f,31.78f,8100.0f,8.25699f,11.37612f},
      {72.0848f,37.46f,8100.0f,7.88122f,11.22865f},
      {71.1919f,70.82f,8100.0f,7.70232f,11.40698f},
      {71.2793f,68.29f,8090.0f,7.72208f,11.64485f},
      {73.0069f,71.26f,8090.0f,7.68752f,11.9069f},
      {75.3913f,83.76f,8090.0f,7.90867f,12.15519f},
      {75.4882f,61.63f,8090.0f,8.2761f,11.94572f},
      {74.8551f,66.08f,8090.0f,8.28882f,11.9518f},
      {73.373f,40.68f,8090.0f,8.33391f,11.6414f},
      {72.498f,39.13f,8090.0f,8.00752f,11.45356f},
      {72.4501f,56.57f,8090.0f,7.78304f,11.5787f},
  };
  float coast_squared=0.f;
  for (const auto& row : coast) {
    const State x{row[0],row[1],row[2],true,row[3]};
    const auto free=negative(x,0);
    assert(free.valid && !free.cross_condition);
    const float a=lateral_energy::angle(geometry,free.energy_mj*.001f);
    coast_squared+=(a-row[4])*(a-row[4]);
    float previous=free.energy_mj;
    for (uint16_t ms=10;ms<=40;++ms) {
      const auto braking=negative(x,ms);
      assert(braking.valid && braking.cross_condition && braking.energy_mj<previous);
      previous=braking.energy_mj;
    }
  }
  assert(std::sqrt(coast_squared/22.f)<.26f);
  assert(negative(State{72.f,60.f,8120.f,true,8.f},0).cross_condition);
}
