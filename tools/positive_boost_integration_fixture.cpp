#include "positive_left_boost.h"
#include "tilt_priority.h"
#include "negative_weak_probe.h"
#include "negative_cap_recovery.h"
#include "wheel_cycle_guard.h"
#include "wheel_work_energy_control.h"
#include "peak_error_adaptation.h"
#include "gradual_start.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
namespace Config {
uint16_t WALKING_TARGET8_MAX_PULSE_MS=150;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED=12;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONE=0;
}
enum class ExperimentState { RUNNING_BATCH_SWEEP, STOPPED };
uint32_t millis(){return 123456;}
namespace PsramLogger {
struct EnergyControlAutonomousPeakEvent {bool peak_speed_valid;uint32_t peak_speed_age_us;bool pending_command_matched;};
struct EnergyControlAutonomousZeroCrossEvent {
  uint32_t previous_peak_time_ms=800,zero_cross_time_ms=997;
  int8_t physical_next_peak_side=1,previous_peak_side=-1,q_command_direction=0;
  float target_peak_deg=8,foot_balance_target_deg=10,previous_peak_amplitude_deg=5;
  float pre_input_wheel_speed_rpm=120,target_energy_j=.00073342f;
  uint16_t vbat_mV=8000; bool pre_input_wheel_speed_valid=true;
  uint32_t pre_input_wheel_speed_age_us=1000;
  bool side_mismatch_diagnostic=false;
  uint8_t work_control_branch=0,reason=0;
  bool energy_model_extrapolated=false,work_prediction_cross_condition=false;
  uint16_t work_brake_cap_ms=0,solver_selected_integer_width_ms=0;
  float solver_required_width_ms=NAN,passive_energy_j=1,free_next_peak_amplitude_deg=1;
  float predicted_energy_j=1,predicted_observed_energy_j=1,predicted_next_peak_amplitude_deg=1;
  float predicted_observed_next_peak_deg=1,predicted_wheel_work_j=1,predicted_input_work_j=1;
  float predicted_post_input_work_j=1,predicted_work_change_j=1,q_command_mA_s=1,q_effective_pred_mA_s=1;
  bool command_matches_zero_cross_motion=false,valid=false,output_executed=false;
  int16_t command_current_mA=0;uint16_t pulse_width_ms=0;
  uint32_t pulse_start_ms=0,pulse_end_ms=0;
};
}
struct FakeImu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct FakeRoller {bool good=true;bool ok()const{return good;}};
struct FakeFoot {bool good=true;unsigned calls=0;bool selectorUnavailable(uint32_t){++calls;return good;}};
struct FakeAdaptive {bool broken=false;void breakPair(){broken=true;}};
struct Status {bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;float current_audit_q_target_mA_s=0,current_audit_q_pred_mA_s=0;};
struct FakeGradual {float target()const{return 8.f;}};
class ExperimentRunner {
 public:
  Status status_;FakeImu imu_obj;FakeRoller roller_obj;FakeFoot foot_span_control_;FakeAdaptive adaptive_cg_;
  FakeImu* imu_=&imu_obj;FakeRoller* roller_=&roller_obj;
  positive_left_boost::Input positive_left_input_;positive_left_boost::Controller positive_left_boost_;
  tilt_priority::Controller tilt_priority_;
  negative_weak_probe::Controller negative_weak_probe_;
  negative_cap_recovery::Controller negative_cap_recovery_;
  wheel_cycle_guard::Controller wheel_cycle_guard_;
  peak_error_adaptation::Mode energy_control_autonomous_pending_mode_=peak_error_adaptation::Coast;
  FakeGradual gradual_start_;float energy_control_autonomous_target_peak_deg_=8.f;
  float run_cg_shift_mm_=0,pending_cg_shift_mm_=0,pending_target_energy_j_=NAN;
  bool energy_control_autonomous_pending_peak_=false,energy_control_autonomous_pending_probe_=false;
  int8_t energy_control_autonomous_pending_next_side_=0;
  float energy_control_autonomous_pending_target_deg_=NAN,energy_control_autonomous_pending_q_command_mA_s_=1;
  float energy_control_autonomous_pending_model_energy_j_=1,energy_control_autonomous_pending_wheel_rpm_=NAN;
  uint32_t energy_control_autonomous_pending_zero_ms_=0,energy_control_autonomous_pending_wheel_age_us_=0,run_start_ms_=100;
  bool energy_control_autonomous_pending_wheel_valid_=false,energy_control_autonomous_pending_saturated_upper_=false,
       energy_control_autonomous_pending_saturated_lower_=false,work_positive_pending_=true,work_negative_pending_=true,
       work_walking_pending_=true,work_walking_low_pending_=true;
  bool begin_ok=true,emergency=false;unsigned calls=0;int last_sign=0,last_ma=0;uint16_t last_width=0;
  uint32_t last_now=0,last_test=0;
  bool wheelWorkMode()const{return true;}
  void requestEmergencyStop(const char*){emergency=true;}
  bool beginEnergyControlAutonomousPulse(uint32_t now,uint32_t test,int8_t sign,uint16_t width,int16_t ma){
    ++calls;last_now=now;last_test=test;last_sign=sign;last_width=width;last_ma=ma;
    if(begin_ok)status_.pulse_active=true;
    return begin_ok;
  }
  bool tryPositiveLeftBoost(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,
                            const wheel_work_energy_control::Choice&,uint32_t);
  float energyControlPeakTarget(int8_t physical_side)const;
  void notifyPositiveLeftPeak(int8_t physical_side,uint32_t peak_ms,float amplitude_deg,
                             const PsramLogger::EnergyControlAutonomousPeakEvent& event);
  void normalConfirmation(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_ms) {
    // NORMAL_SOURCE
  }
  void feedPeak(int8_t physical_side,uint32_t peak_ms,float amplitude_deg,bool valid) {
    PsramLogger::EnergyControlAutonomousPeakEvent event{valid,valid?100u:999999u,false};
    notifyPositiveLeftPeak(physical_side,peak_ms,amplitude_deg,event);
  }
};
// METHOD_SOURCE
// HELPER_SOURCE
using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
static wheel_work_energy_control::Choice normal() {
  wheel_work_energy_control::Choice n;n.valid=true;n.command_sign=1;n.current_ma=400;
  n.width_ms=100;n.allowed_cap_ms=100;return n;
}
static Event sample(uint32_t now) {Event e;e.previous_peak_time_ms=now-200;e.zero_cross_time_ms=now-3;return e;}
static void normalCycle(ExperimentRunner& r,uint32_t now,float peak) {
  r.status_.pulse_active=false;Event e=sample(now);const auto n=normal();
  assert(!r.tryPositiveLeftBoost(e,n,now));
  // Drive the actual production confirmation statement using the event
  // that the ordinary motor path would have recorded on a successful send.
  e.valid=true;e.output_executed=true;e.command_current_mA=400;e.pulse_width_ms=100;e.q_command_direction=1;
  r.normalConfirmation(e,now);r.feedPeak(1,now+200,peak,true);r.feedPeak(-1,now+600,5.f,true);
}
static void seed(ExperimentRunner& r) {normalCycle(r,1000,5.0f);normalCycle(r,1800,5.2f);}
static Event trial(ExperimentRunner& r,uint32_t now,int expected_ma,uint16_t expected_width,uint16_t cap=150) {
  r.status_.pulse_active=false;Event e=sample(now);auto n=normal();
  assert(r.tryPositiveLeftBoost(e,n,now));
  assert(e.valid&&e.output_executed&&e.work_control_branch==14&&e.command_current_mA==expected_ma);
  assert(e.solver_required_width_ms==100&&e.solver_selected_integer_width_ms==expected_width);
  assert(e.pulse_width_ms==expected_width&&e.q_command_direction==1&&e.work_brake_cap_ms==cap);
  assert(e.pulse_start_ms==now&&e.pulse_end_ms==now+expected_width);
  assert(r.last_now==now+100&&r.last_test==now&&r.last_width==expected_width&&r.last_ma==expected_ma&&r.last_sign==1);
  assert(e.energy_model_extrapolated&&e.work_prediction_cross_condition);
  assert(std::isnan(e.predicted_energy_j)&&std::isnan(e.predicted_next_peak_amplitude_deg)&&
         std::isnan(e.predicted_wheel_work_j)&&std::isnan(e.q_command_mA_s));
  assert(r.energy_control_autonomous_pending_peak_&&r.energy_control_autonomous_pending_probe_&&
         r.energy_control_autonomous_pending_next_side_==1&&r.energy_control_autonomous_pending_target_deg_==10);
  assert(std::isnan(r.energy_control_autonomous_pending_model_energy_j_)&&
         std::isnan(r.energy_control_autonomous_pending_q_command_mA_s_));
  assert(!r.work_positive_pending_&&!r.work_negative_pending_&&!r.work_walking_pending_&&r.adaptive_cg_.broken);
  return e;
}
int main() {
  {ExperimentRunner r;assert(r.energyControlPeakTarget(1)==4.5f);
   assert(r.energyControlPeakTarget(-1)==4.5f);r.energy_control_autonomous_pending_peak_=true;
   r.energy_control_autonomous_pending_next_side_=1;r.energy_control_autonomous_pending_target_deg_=9.f;
   assert(r.energyControlPeakTarget(1)==9.f&&r.energyControlPeakTarget(-1)==4.5f);
   r.energy_control_autonomous_pending_target_deg_=NAN;
   assert(r.energyControlPeakTarget(1)==4.5f);
   r.energy_control_autonomous_pending_peak_=false;r.energy_control_autonomous_target_peak_deg_=6.f;
   assert(r.energyControlPeakTarget(1)==6.f&&r.energyControlPeakTarget(-1)==6.f);}
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);
   r.status_.pulse_active=false;Event no=sample(2600);assert(!r.tryPositiveLeftBoost(no,normal(),2600));
   r.feedPeak(1,2800,5.5f,true);r.feedPeak(-1,3200,4.8f,true);
   trial(r,3400,700,100);r.feedPeak(1,3600,5.7f,true);r.feedPeak(-1,4000,4.8f,true);
   trial(r,4200,900,100);r.feedPeak(1,4420,5.8f,true);r.feedPeak(-1,4800,4.9f,true);
   trial(r,5000,1200,100);r.feedPeak(1,5200,5.9f,true);r.feedPeak(-1,5600,4.9f,true);
   trial(r,5800,1200,125);r.feedPeak(1,6000,6.f,true);r.feedPeak(-1,6400,4.9f,true);
   trial(r,6600,1200,150);}
  // Equal increases on both sides must not be reported as asymmetric progress;
  // the next current level may still be tried rather than stopping walking.
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,6.f,true);
   r.feedPeak(-1,3200,5.9f,true);assert(!r.positive_left_boost_.accepted());
   trial(r,3400,700,100);}
  // When both peaks increase together up to the target, do not escalate
  // further; resume the ordinary walking path without an emergency stop.
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,9.6f,true);
   r.feedPeak(-1,3200,9.5f,true);assert(!r.positive_left_boost_.accepted());
   r.status_.pulse_active=false;Event e=sample(3400);
   assert(!r.tryPositiveLeftBoost(e,normal(),3400)&&!r.status_.emergency_stop);}
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,9.f,true);
   r.feedPeak(-1,3200,8.1f,true);r.status_.pulse_active=false;Event e=sample(3400);
   assert(!r.tryPositiveLeftBoost(e,normal(),3400)&&!r.status_.emergency_stop);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.previous_peak_amplitude_deg=8.f;
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls&&!r.status_.emergency_stop);}
  {ExperimentRunner r;seed(r);r.begin_ok=false;Event e=sample(2600);
   assert(r.tryPositiveLeftBoost(e,normal(),2600));
   assert(e.work_control_branch==14&&!e.valid&&!e.output_executed&&e.reason==12&&e.pulse_width_ms==0);
   assert(e.solver_selected_integer_width_ms==100&&!r.energy_control_autonomous_pending_peak_);
   r.feedPeak(1,2800,8,true);r.feedPeak(-1,3200,5,true);r.begin_ok=true;
   Event after=sample(3400);assert(!r.tryPositiveLeftBoost(after,normal(),3400));}
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,4.4f,true);
   r.feedPeak(-1,3200,5.4f,true);r.status_.pulse_active=false;Event e=sample(3400);
   assert(!r.tryPositiveLeftBoost(e,normal(),3400));}
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,5.5f,true);
   r.feedPeak(-1,3200,3.0f,true);r.status_.pulse_active=false;Event e=sample(3400);
   assert(!r.tryPositiveLeftBoost(e,normal(),3400));}
  {ExperimentRunner r;seed(r);trial(r,2600,500,100);r.feedPeak(1,2800,5.5f,false);
   r.status_.pulse_active=false;Event e=sample(3400);assert(!r.tryPositiveLeftBoost(e,normal(),3400));}
  {Config::WALKING_TARGET8_MAX_PULSE_MS=120;ExperimentRunner r;seed(r);trial(r,2600,500,100,120);
   r.feedPeak(1,2800,5.5f,true);r.feedPeak(-1,3200,4.8f,true);
   trial(r,3400,700,100,120);r.feedPeak(1,3600,5.7f,true);r.feedPeak(-1,4000,4.8f,true);
   trial(r,4200,900,100,120);r.feedPeak(1,4400,5.8f,true);r.feedPeak(-1,4800,4.8f,true);
   trial(r,5000,1200,100,120);r.feedPeak(1,5200,5.9f,true);r.feedPeak(-1,5600,4.8f,true);
   trial(r,5800,1200,120,120);Config::WALKING_TARGET8_MAX_PULSE_MS=150;}
  {ExperimentRunner r;seed(r);r.foot_span_control_.good=false;Event e=sample(2600);
   assert(r.tryPositiveLeftBoost(e,normal(),2600)&&r.emergency&&!r.calls);}
  {ExperimentRunner r;seed(r);r.status_.emergency_stop=true;Event e=sample(2600);
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);r.imu_obj.good=false;Event e=sample(2600);
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);r.roller_obj.good=false;Event e=sample(2600);
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.pre_input_wheel_speed_valid=false;
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.pre_input_wheel_speed_age_us=peak_error_adaptation::kMaxSpeedAgeUs+1;
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.side_mismatch_diagnostic=true;
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.previous_peak_time_ms=1000;
   assert(!r.tryPositiveLeftBoost(e,normal(),2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);auto n=normal();n.allowed_cap_ms=90;
   assert(!r.tryPositiveLeftBoost(e,n,2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);auto n=normal();n.current_ma=350;
   assert(r.tryPositiveLeftBoost(e,n,2600)&&r.last_ma==500);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);auto n=normal();n.command_sign=-1;
   assert(!r.tryPositiveLeftBoost(e,n,2600)&&!r.calls);}
  {ExperimentRunner r;seed(r);Event e=sample(2600);e.foot_balance_target_deg=9.5;
   assert(r.tryPositiveLeftBoost(e,normal(),2600)&&r.last_ma==500);}
  std::cout<<"disconnected runner positive boost helper with hardware doubles PASS\n";
}
