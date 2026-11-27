#include "walking/support/walking_return_energy.h"
#include "walking/support/return_observation_policy.h"
#include <cstring>
#include "walking/pulse.h"
#include "walking/walking_energy_policy.h"
#include "control_setup/run_settings.h"
#include "foot_motion_prediction.h"
#include "walking/support/return_motion.h"
#define FOOT_TREND_CYCLES 2
#define ANGLE_EARLY_TRIAL 1
#define WALK_SMOOTH_TRIAL 1
#include "control_trials/tilt_priority.h"
#include "control_trials/foot_balance_adaptive.h"
#include "control_trials/walking_side_assist.h"
#include "control_trials/negative_cap_recovery.h"
#include "control_trials/foot_span_control.h"
#include "models/peak_error_adaptation.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
using std::isfinite;
namespace Config {
constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS=100;
constexpr int16_t WALKING_TARGET8_MAX_CURRENT_MA=1200,ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN=1,MOTOR_DRIVER_POLARITY=1;
constexpr float MODEL_VBAT_REFERENCE_V=7.5f;

constexpr uint16_t WALKING_TARGET8_MAX_PULSE_MS=150;
constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS=250;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED=12;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONE=0;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT=1;
}
enum class ExperimentState { RUNNING_BATCH_SWEEP, STOPPED };
uint32_t mock_us=2000000,mock_ms=1300;
uint32_t micros(){return mock_us;}
uint32_t millis(){return mock_ms;}
namespace PsramLogger {
struct EnergyControlAutonomousZeroCrossEvent {
 uint32_t previous_peak_time_ms=900;float previous_peak_amplitude_deg=8;
 float detector_angle_after_deg=0,zero_cross_rate_dps=35,i0_estimated_mA=0,cg_shift_mm=0;
 float kinetic_energy_j=0,delta_energy_required_j=0,commanded_energy_deficit_j=0;
 float corrected_target_energy_j=0,integral_energy_j=0,predicted_loss_j=0;
 float maximum_input_work_j=0,unapplied_energy_request_j=0;
 uint32_t zero_cross_time_ms=999;int8_t physical_next_peak_side=1,q_command_direction=0;
 float target_peak_deg=8,pre_input_wheel_speed_rpm=20,target_energy_j=.001f,zero_cross_abs_rate_dps=10;
 uint16_t vbat_mV=7500;bool pre_input_wheel_speed_valid=true;uint32_t pre_input_wheel_speed_age_us=100;
 bool side_mismatch_diagnostic=false,energy_model_extrapolated=false,work_prediction_cross_condition=false;
 uint8_t work_control_branch=0,reason=0;uint16_t work_brake_cap_ms=0,solver_selected_integer_width_ms=0;
 uint16_t wheel_guard_legacy_ms=0,wheel_guard_extra_ms=0;bool wheel_guard_trial_eligible=false;
 uint16_t sway_trim_before_ms=0,sway_trim_cap_ms=0;
 uint16_t fast_trim_before_ms=0,fast_trim_cap_ms=0;
 uint16_t weak_probe_baseline_ms=0,weak_probe_requested_ms=0;bool weak_probe_executed=false;
 float passive_energy_j=1,free_next_peak_amplitude_deg=1,predicted_energy_j=1,predicted_observed_energy_j=1;
 float predicted_next_peak_amplitude_deg=1,predicted_observed_next_peak_deg=1,predicted_wheel_work_j=1;
 float predicted_input_work_j=1,predicted_post_input_work_j=1,predicted_work_change_j=1;
 float q_command_mA_s=1,q_effective_pred_mA_s=1,predicted_current_goal_mA=1;
 float predicted_current_rise_tau_s=1,aligned_initial_current_mA=1;
 bool command_matches_zero_cross_motion=false,valid=false,output_executed=false;
 int16_t command_current_mA=0;uint16_t pulse_width_ms=0;uint32_t pulse_start_ms=0,pulse_end_ms=0;
 enum FootSpanFlag : uint16_t { Span_selected=2, NegativeRecoveryStateBit0=1u<<12, NegativeRecoveryStateBit1=1u<<13, PositiveSwayProbe=1u<<14 };
 uint16_t foot_span_baseline_ms=0,foot_span_requested_ms=0,foot_span_flags=0;
 void setFootSpanFlag(FootSpanFlag f,bool v){if(v)foot_span_flags|=f;else foot_span_flags&=~f;}
};
}
struct Status {uint16_t input_interval_ms=1000;int16_t roller_actual_current_mA=0;int16_t current_mA_setting=0,motor_cmd_mA=0;uint16_t pulse_width_ms_setting=0;int8_t pulse_direction=0;float beta_model_vbat_mV=7500;uint16_t run_id=3;uint32_t pulse_id=0;bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;
 float current_audit_q_target_mA_s=1,current_audit_q_pred_mA_s=1;};
struct Imu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct Telemetry {int16_t actual_current_mA=0;float speed_rpm=20;uint32_t speed_sample_time_us=1999900;bool speed_valid=true;};
struct Roller {
 bool good=true,write_ok=true;Telemetry telemetry;unsigned writes=0;uint32_t write_delay_ms=0;int16_t last_command=0;
 unsigned stop_requests=0;uint32_t stop_delay_ms=0;bool stop(){++stop_requests;mock_ms+=stop_delay_ms;mock_us+=stop_delay_ms*1000;if(last_command!=0){++writes;last_command=0;}return true;}
 bool ok()const{return good;}Telemetry telemetrySnapshot()const{return telemetry;}
 bool setCurrentMa(int16_t command){++writes;last_command=command;mock_ms+=write_delay_ms;mock_us+=write_delay_ms*1000;return write_ok;}
};
struct Resettable {int resets=0;void reset(){++resets;}void interrupt(){++resets;}};
struct Adaptive {bool broken=false;void breakPair(){broken=true;}};
enum class EnergyControlAutonomousHalfCycleState {WAIT_PEAK,WAIT_ZERO_CROSS,PULSE_ACTIVE};
struct Logger {unsigned events=0;PsramLogger::EnergyControlAutonomousZeroCrossEvent last;void addEnergyControlAutonomousZeroCrossEvent(const PsramLogger::EnergyControlAutonomousZeroCrossEvent& e){++events;last=e;}};
enum class EnergyControlAutonomousPhase {ENERGY_CONTROL,HOLD,STOP,STRONG_START_KICK,WAIT_FIRST_PEAK};
class ExperimentRunner {public:
 uint32_t energy_control_autonomous_previous_detector_test_ms_=1198;
 float energy_control_autonomous_previous_detector_rate_dps_=-15;
 void processCrossingSample(uint32_t,uint32_t,float,float,float);
 void updateEnergyControlAutonomousPeakTracker(uint32_t,float,float){}
 bool observeInwardStallRescue(uint32_t,bool,bool,float,float){walking_return_event_=false;return false;}

 bool q_ident_pulse_authorized_=true,energy_control_v0_pulse_authorized_=true,zero_cross_mode_=false;
 uint32_t last_pulse_end_ms_=0,next_pulse_start_test_ms_=0;
 void updateEnergyControlAutonomousPulse(uint32_t);void stopActivePulse(uint32_t);

 bool energy_control_autonomous_mode_=true;EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;
 uint32_t energy_control_autonomous_last_motion_sample_us_=2000000;
 uint32_t active_pulse_start_ms_=0,active_pulse_start_test_ms_=0;bool energy_control_autonomous_pulse_authorized_=false;
 float predictRiseTauS(float)const{return .0316f;}void updatePulseModelPrediction(){}
 template<typename... T>void startTimingProbe(T...){}
 void stopMotor();

/* PRODUCTION: ENERGY_MEMBERS */
  lateral_energy::Geometry lateral_energy_geometry_;
  uint32_t predicted_current_end_ms_=0;float predicted_signed_current_end_mA_=0;
  float predictCurrentGoalMa(int,float)const{return 329.547119f;}

/* PRODUCTION: FOOT_MEMBERS */

 bool stall_rescue_event_=false,optional_early_event_=false,optional_early_write_uncertain_=false;
 uint32_t optional_early_attempted_peak_ms_=0,energy_control_autonomous_last_peak_ms_=900;
 bool energy_control_autonomous_last_peak_valid_=true;
 int8_t energy_control_autonomous_last_peak_side_=-1;
 float energy_control_autonomous_target_peak_deg_=8;
 int energy_control_autonomous_input_peak_percent_=0;
 Logger log_obj;Logger* logger_=&log_obj;
 EnergyControlAutonomousHalfCycleState energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
 bool energy_control_autonomous_last_accepted_zero_cross_valid_=true,energy_control_autonomous_zero_cross_consumed_for_peak_=true;
 uint32_t energy_control_autonomous_last_accepted_zero_cross_ms_=0;
 unsigned peak_tracker_resets=0,finished=0;
 void resetEnergyControlAutonomousPeakTracker(bool){++peak_tracker_resets;}
 void finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent& e,uint32_t){++finished;last_finished_event=e;}
 void restoreUnsentStallRescue(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,bool,uint32_t);
 bool dispatchTiltPriority(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,bool,uint32_t);
 void acceptOptionalEarlyMotion(uint32_t,float,float);
 void acceptMotionCrossing(bool,uint32_t t,float rate,float,float after,float,float) {
/* PRODUCTION: CROSSING_GUARD */
   auto prev_valid=energy_control_autonomous_last_accepted_zero_cross_valid_;
   auto prev_ms=energy_control_autonomous_last_accepted_zero_cross_ms_;
   energy_control_autonomous_zero_cross_consumed_for_peak_=true;
   energy_control_autonomous_last_accepted_zero_cross_ms_=t;
   PsramLogger::EnergyControlAutonomousZeroCrossEvent e;e.physical_next_peak_side=rate>0?1:-1;e.zero_cross_time_ms=t;
   e.zero_cross_abs_rate_dps=std::fabs(rate);e.pre_input_wheel_speed_rpm=simulated_rpm;
   e.side_mismatch_diagnostic=simulated_side_mismatch;
   e.zero_cross_rate_dps=rate;e.detector_angle_after_deg=after;
/* PRODUCTION: RESIDUAL_CURRENT */
   e.previous_peak_time_ms=energy_control_autonomous_last_peak_ms_;
   e.previous_peak_amplitude_deg=energy_control_autonomous_last_peak_amplitude_deg_;

   if(change_before_dispatch==1)++status_.run_id;
   if(change_before_dispatch==2)mock_ms+=6;
   if(change_before_dispatch==3)mock_us+=10001;
   if(change_before_dispatch==4){foot_pose_.confidence=1;for(unsigned i=0;i<4;++i){foot_pose_.left_uncertainty[i]=.35f;foot_pose_.right_uncertainty[i]=.35f;}}
   if(change_before_dispatch==5)--foot_local_pose_.frame_ms;
   if(change_before_dispatch==6)mock_ms+=4;
   dispatchTiltPriority(e,t,prev_valid,prev_ms);
 }
 float energy_control_autonomous_last_peak_amplitude_deg_=3.f;
 uint32_t case_now=0;int change_before_dispatch=0;float simulated_rpm=20;bool simulated_side_mismatch=false;
 PsramLogger::EnergyControlAutonomousZeroCrossEvent last_finished_event;
 Status status_;Imu imu_obj;Roller roller_obj;Imu* imu_=&imu_obj;Roller* roller_=&roller_obj;
 tilt_priority::Controller tilt_priority_;foot_span_control::Controller foot_span_control_;
 wheel_cycle_guard::Controller wheel_cycle_guard_;
 long_foot_balance::Controller long_foot_balance_;
 positive_sway_probe::Controller positive_sway_probe_;
 negative_cap_recovery::Controller negative_cap_recovery_;
 uint32_t negative_recovery_last_applied_ms_=0;
 uint32_t foot_trial_pending_ms_=0,foot_trial_pending_generation_=0;
 bool foot_trial_pending_effective_=false;
 Resettable weak_side_assist_,walking_loss_feedback_,walking_low_speed_guard_;
 Adaptive adaptive_cg_;bool mode=true,begin_ok=true,simulate_write_uncertain=false;unsigned calls=0;int sign=0,ma=0;uint16_t width=0;
 float run_cg_shift_mm_=0,pending_cg_shift_mm_=0,pending_target_energy_j_=NAN;
 bool energy_control_autonomous_pending_peak_=false,energy_control_autonomous_pending_probe_=false;
 int8_t energy_control_autonomous_pending_next_side_=0;
 float energy_control_autonomous_pending_target_deg_=0,energy_control_autonomous_pending_q_command_mA_s_=0;
 float energy_control_autonomous_pending_model_energy_j_=0,energy_control_autonomous_pending_wheel_rpm_=0;
 uint32_t energy_control_autonomous_pending_wheel_age_us_=0,energy_control_autonomous_pending_zero_ms_=0,run_start_ms_=100;
 bool energy_control_autonomous_pending_wheel_valid_=false,energy_control_autonomous_pending_saturated_upper_=true;
 bool energy_control_autonomous_pending_saturated_lower_=true,work_positive_pending_=true,work_negative_pending_=true;
 bool work_walking_pending_=true,work_walking_low_pending_=true;
 peak_error_adaptation::Mode energy_control_autonomous_pending_mode_=peak_error_adaptation::Coast;
 bool wheelWorkMode()const{return mode;}
 void requestEmergencyStop(const char*){status_.emergency_stop=true;}
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t,uint16_t,int16_t);
 bool tryTiltPriority(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
 bool tryTiltPriorityOptionalEarly(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
 bool tryTiltPriorityImpl(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,bool*);
 bool tryTiltPriorityRescue(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
};
/* PRODUCTION: SELECTION */
/* PRODUCTION: restoreUnsentStallRescue */
/* PRODUCTION: dispatchTiltPriority */
/* PRODUCTION: acceptOptionalEarlyMotion */
/* PRODUCTION: tryFootSupportCandidate */
using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
#include <cstdlib>
#undef assert
#define assert(test) do {if(!(test)){std::cerr<<"assertion failed: "<<#test<<" line "<<__LINE__<<"\n";std::exit(2);}}while(false)
static void local_pose(ExperimentRunner& r,uint32_t now){
  foot_local_prediction::Predictor local;
  local.observe(r.status_.run_id,now-263,now-200,true,-.56573f,5.75716f);
  local.observe(r.status_.run_id,now-163,now-100,true,-.47464f,5.36924f);
  local.observe(r.status_.run_id,now-63,now,true,-.58475f,3.95754f);
  r.foot_local_pose_=local.projection();
  r.foot_pose_=foot_phase_prediction::Projection{};
  r.foot_pose_.run_id=r.status_.run_id;r.foot_pose_.frame_ms=now-63;r.foot_pose_.received_ms=now;
  r.foot_pose_.reason=foot_phase_prediction::Reason::Ready;r.foot_pose_.confidence=0;
  for(unsigned i=0;i<4;++i){r.foot_pose_.left[i]=-.58475f;r.foot_pose_.right[i]=3.95754f;}
  r.foot_pose_sequence_=1001;
}

/* PRODUCTION: ENERGY_METHODS */
static bool near(float a,float b,float tolerance=3e-9f){return std::fabs(a-b)<tolerance;}
static void ready(ExperimentRunner& r,float same=6,float previous=8){
  r.case_now=1200;mock_ms=r.run_start_ms_+r.case_now;mock_us=2000000;
  r.energy_control_autonomous_last_peak_ms_=900;
  r.energy_control_autonomous_last_peak_amplitude_deg_=previous;
  r.energy_control_autonomous_last_peak_side_=-1;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=600;
  r.walking_energy_peaks_[1]=same;r.walking_energy_peak_ms_[1]=700;
  assert(r.walking_energy_current_.prepare(.0316f));
  // Ramp the real stage to 8deg using only accepted peak history.
  r.tilt_priority_.peak(-1,1,4.5f);uint32_t t=10;
  for(unsigned n=0;n<32;++n){const int side=n%2?-1:1;
    tilt_priority::Choice c;c.selected=true;c.direction=side;c.current_ma=1200;
    c.width_ms=1;c.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(side,t);r.tilt_priority_.confirm(side,t,c,true,true,false);
    r.tilt_priority_.peak(side,t+10,c.target_deg);t+=25;}
  // Fresh unpaired observations clear the artificial short time-to-peak cap.
  r.tilt_priority_.peak(-1,900,previous);r.tilt_priority_.peak(1,925,same);
  assert(r.tilt_priority_.target()==8.f);
  local_pose(r,r.case_now);
}

/* PRODUCTION: beginEnergyControlAutonomousPulse */

/* PRODUCTION: updateEnergyControlAutonomousPulse */

/* PRODUCTION: stopActivePulse */

/* PRODUCTION: stopMotor */

void ExperimentRunner::processCrossingSample(uint32_t now_ms,uint32_t sample_us,
    float before_deg,float detector_relative_angle_deg,float rate_dps){
  const uint32_t t_test_ms=now_ms-run_start_ms_;
  const float peak_relative_angle_deg=detector_relative_angle_deg;
/* PRODUCTION: CROSSING_ROUTER */
}
static void clock_at(ExperimentRunner& r,unsigned t){
 mock_ms=r.run_start_ms_+t;mock_us=2000000u+(t-1200u)*1000u;
 r.energy_control_autonomous_last_motion_sample_us_=mock_us;
}
