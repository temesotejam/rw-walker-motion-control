#include "weak_side_assist.h"
#include "wheel_work_energy_control.h"
#include "peak_error_adaptation.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <cstdint>
namespace Config { constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS=100;
constexpr uint16_t WALKING_TARGET8_MAX_PULSE_MS=150;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED=12,ENERGY_CONTROL_AUTONOMOUS_REASON_NONE=0; }
enum class ExperimentState {RUNNING_BATCH_SWEEP, STOPPED};
uint32_t millis(){return 123456;}
namespace PsramLogger {
struct EnergyControlAutonomousZeroCrossEvent {
 int8_t physical_next_peak_side=-1,previous_peak_side=1,q_command_direction=0;
 float target_peak_deg=8.f,previous_peak_amplitude_deg=5.57611f,zero_cross_abs_rate_dps=22.6566f,
 pre_input_wheel_speed_rpm=264.3f,target_energy_j=.00038619f,i0_estimated_mA=0.f;
 uint16_t vbat_mV=7870;bool pre_input_wheel_speed_valid=true;uint32_t pre_input_wheel_speed_age_us=50,previous_peak_time_ms=21450;
 bool side_mismatch_diagnostic=false;uint32_t zero_cross_time_ms=21723;
 uint16_t solver_selected_integer_width_ms=0,work_brake_cap_ms=0;
 uint8_t work_control_branch=0;bool energy_model_extrapolated=false,work_prediction_cross_condition=false;
 float passive_energy_j=1,free_next_peak_amplitude_deg=1,predicted_energy_j=1,predicted_observed_energy_j=1,
 predicted_next_peak_amplitude_deg=1,predicted_observed_next_peak_deg=1,predicted_wheel_work_j=1,
 predicted_input_work_j=1,predicted_post_input_work_j=1,predicted_work_change_j=1,
 q_command_mA_s=NAN,q_effective_pred_mA_s=NAN,predicted_current_goal_mA=NAN,predicted_current_rise_tau_s=NAN,
 aligned_initial_current_mA=NAN;
 bool command_matches_zero_cross_motion=false,valid=false,output_executed=false;int16_t command_current_mA=0;
 uint16_t pulse_width_ms=0;uint32_t pulse_start_ms=0,pulse_end_ms=0;uint8_t reason=0;
};}
struct FakeImu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct FakeRoller {bool good=true;bool ok()const{return good;}};
struct FakeFeedback {bool broken=false;void interrupt(){broken=true;}void breakPair(){broken=true;}};
struct FakeGradual {float value=5.5f;float target()const{return value;}};
struct Status {bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;float current_audit_q_target_mA_s=0,current_audit_q_pred_mA_s=0;};
class ExperimentRunner {
public:
 struct Peak {uint32_t ms=0;float amplitude=NAN;};
 struct Decision {uint32_t ms=0;};
 Peak phase_negative_peaks_[8];Status status_;FakeRoller roller_obj;FakeImu imu_obj;FakeRoller* roller_=&roller_obj;FakeImu* imu_=&imu_obj;
 Decision phase_negative_decisions_[8];
 FakeFeedback walking_loss_feedback_,walking_low_speed_guard_,adaptive_cg_;FakeGradual gradual_start_;
 weak_side_assist::Controller weak_side_assist_;
 lateral_energy::CurrentTable lateral_current_table_;
 float run_cg_shift_mm_=0,pending_cg_shift_mm_=0,pending_target_energy_j_=NAN;
 bool energy_control_autonomous_pending_peak_=false,energy_control_autonomous_pending_probe_=false;
 int8_t energy_control_autonomous_pending_next_side_=0,energy_control_autonomous_pending_mode_=0;
 float energy_control_autonomous_pending_target_deg_=NAN,energy_control_autonomous_pending_q_command_mA_s_=NAN,
 energy_control_autonomous_pending_model_energy_j_=NAN,energy_control_autonomous_pending_wheel_rpm_=NAN;
 uint32_t energy_control_autonomous_pending_wheel_age_us_=0,energy_control_autonomous_pending_zero_ms_=0,run_start_ms_=100;
 bool energy_control_autonomous_pending_wheel_valid_=false,energy_control_autonomous_pending_saturated_upper_=false,
 energy_control_autonomous_pending_saturated_lower_=false,work_positive_pending_=false,work_negative_pending_=false,
 work_walking_pending_=false,work_walking_low_pending_=false;
 bool begin_ok=true;int call_count=0,last_sign=0,last_ma=0;uint16_t last_width=0;uint32_t last_now=0,last_test=0;
 bool wheelWorkMode()const{return true;}
 float predictCurrentGoalMa(int16_t ma,float)const{return static_cast<float>(ma);}
 bool beginEnergyControlAutonomousPulse(uint32_t now,uint32_t test,int8_t sign,uint16_t width,int16_t ma){
  ++call_count;last_now=now;last_test=test;last_sign=sign;last_width=width;last_ma=ma;
  if(begin_ok)status_.pulse_active=true;
  return begin_ok;
 }
 bool tryWeakSideAssist(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,const wheel_work_energy_control::Choice&,bool,uint32_t,float);
};
// METHOD_SOURCE
static void configure(ExperimentRunner& r,uint32_t now=21726,uint32_t prior_duration=200) {
 r.phase_negative_peaks_[0].ms=now-1300;r.phase_negative_peaks_[0].amplitude=3.44f;
 r.phase_negative_peaks_[1].ms=now-500;r.phase_negative_peaks_[1].amplitude=3.44f;
 r.phase_negative_decisions_[0].ms=now-500-prior_duration;
 assert(r.lateral_current_table_.prepare(.07f));
}
int main(){
 using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
 wheel_work_energy_control::Choice coast;coast.valid=true;coast.width_ms=0;coast.branch=11;
 {ExperimentRunner r;configure(r);Event e;assert(r.tryWeakSideAssist(e,coast,false,21726,7.87f));
  assert(r.call_count==1&&r.last_sign==-1&&r.last_ma==300&&r.last_width==43&&r.last_test==21726);
  assert(e.valid&&e.output_executed&&e.work_control_branch==13&&e.pulse_width_ms==43);
  assert(e.command_current_mA==300&&e.q_command_direction==-1&&e.q_command_mA_s>0);
  assert(e.energy_model_extrapolated&&e.work_prediction_cross_condition&&std::isnan(e.predicted_energy_j));
  assert(r.energy_control_autonomous_pending_peak_&&r.energy_control_autonomous_pending_probe_);
  assert(r.energy_control_autonomous_pending_mode_==peak_error_adaptation::Excite);
  assert(r.walking_loss_feedback_.broken&&r.walking_low_speed_guard_.broken&&r.adaptive_cg_.broken);}
 {ExperimentRunner r;configure(r);Event e;auto powered=coast;powered.width_ms=30;assert(!r.tryWeakSideAssist(e,powered,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;e.pre_input_wheel_speed_valid=false;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;r.status_.emergency_stop=true;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;e.previous_peak_time_ms=20000;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;e.side_mismatch_diagnostic=true;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;r.imu_obj.good=false;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;r.roller_obj.good=false;assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;wheel_work_energy_control::Choice missing;assert(!r.tryWeakSideAssist(e,missing,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;wheel_work_energy_control::Choice missing;assert(r.tryWeakSideAssist(e,missing,true,21726,7.87f)&&e.output_executed);}
 {ExperimentRunner r;configure(r);Event e;r.begin_ok=false;assert(r.tryWeakSideAssist(e,coast,false,21726,7.87f));
  assert(!e.valid&&!e.output_executed&&e.reason==12&&e.pulse_width_ms==0&&e.command_current_mA==0&&e.solver_selected_integer_width_ms==43);
  assert(!r.energy_control_autonomous_pending_peak_&&!r.status_.pulse_active);}
 {ExperimentRunner r;configure(r);Event e;r.lateral_current_table_=lateral_energy::CurrentTable{};
  assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 // Missing, stale, invalid, or wrapping timing evidence yields ordinary coast.
 {ExperimentRunner r;configure(r);r.phase_negative_decisions_[0].ms=0;Event e;
  assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r,21726,700);Event e;
  assert(!r.tryWeakSideAssist(e,coast,false,21726,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r);Event e;e.previous_peak_time_ms=1;
  assert(!r.tryWeakSideAssist(e,coast,false,30,7.87f)&&r.call_count==0);}
 {ExperimentRunner r;configure(r,21726,60);r.gradual_start_.value=8.f;Event e;
  assert(r.tryWeakSideAssist(e,coast,false,21726,7.87f));assert(e.pulse_width_ms==30&&e.work_brake_cap_ms==30);}
 // Exercise actual runner + controller repeatedly under comparable fresh peaks.
 // Prior 200ms zero-to-peak history caps the pulse at 150ms. Values beyond
 // 100ms are explicitly unmodeled and never reported as fitted q or current.
 {ExperimentRunner r;configure(r);r.gradual_start_.value=8.f;bool saw_unmodeled=false,saw_150=false;
  for(unsigned i=0;i<40;++i) {
   const uint32_t now=21726+i*800;configure(r,now,200);r.status_.pulse_active=false;
   Event e;e.previous_peak_time_ms=now-150;e.zero_cross_time_ms=now-3;
   assert(r.tryWeakSideAssist(e,coast,false,now,7.87f)&&e.output_executed);
   assert(e.pulse_width_ms<=150&&e.pulse_width_ms<=e.work_brake_cap_ms);
   if(e.pulse_width_ms>100) {
    saw_unmodeled=true;assert(std::isnan(e.q_command_mA_s)&&std::isnan(e.q_effective_pred_mA_s));
    assert(std::isnan(e.predicted_current_goal_mA)&&std::isnan(e.predicted_current_rise_tau_s));
    assert(std::isnan(r.energy_control_autonomous_pending_q_command_mA_s_));
    assert(r.energy_control_autonomous_pending_mode_==peak_error_adaptation::Excite);
    assert(r.energy_control_autonomous_pending_probe_&&std::isnan(r.energy_control_autonomous_pending_model_energy_j_));
   }
   if(e.pulse_width_ms==150)saw_150=true;
  }
  assert(saw_unmodeled&&saw_150);}
 // Long-pulse failed sends do not become observations or pending pulses.
 {ExperimentRunner r;configure(r);r.gradual_start_.value=8.f;bool tested=false;
  for(unsigned i=0;i<40;++i) {
   const uint32_t now=21726+i*800;configure(r,now,200);r.status_.pulse_active=false;
   Event e;e.previous_peak_time_ms=now-150;e.zero_cross_time_ms=now-3;
   if(r.weak_side_assist_.limitMs()>100) {
    r.begin_ok=false;r.energy_control_autonomous_pending_peak_=false;
    assert(r.tryWeakSideAssist(e,coast,false,now,7.87f));
    assert(!e.output_executed&&!e.valid&&e.pulse_width_ms==0&&e.command_current_mA==0);
    assert(e.solver_selected_integer_width_ms>100&&std::isnan(e.q_command_mA_s));
    assert(!r.energy_control_autonomous_pending_peak_&&!r.status_.pulse_active);
    tested=true;break;
   }
   assert(r.tryWeakSideAssist(e,coast,false,now,7.87f)&&e.output_executed);
  }
  assert(tested);}
 std::cout<<"actual runner weak-assist timing and unmodeled extension PASS\n";
}
