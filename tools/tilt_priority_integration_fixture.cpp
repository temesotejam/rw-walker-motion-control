#define FOOT_TREND_CYCLES 2
#define ANGLE_EARLY_TRIAL 1
#define WALK_SMOOTH_TRIAL 1
#include "tilt_priority.h"
#include "stall_rescue.h"
#include "long_foot_balance.h"
#include "positive_sway_probe.h"
#include "negative_cap_recovery.h"
#include "wheel_cycle_guard.h"
#include "foot_span_control.h"
#include "peak_error_adaptation.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
using std::isfinite;
namespace Config {
constexpr uint16_t WALKING_TARGET8_MAX_PULSE_MS=150;
constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS=250;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED=12;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONE=0;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT=1;
}
enum class ExperimentState { RUNNING_BATCH_SWEEP, STOPPED };
uint32_t millis(){return 10000;}
namespace PsramLogger {
struct EnergyControlAutonomousZeroCrossEvent {
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
struct Status {bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;
 float current_audit_q_target_mA_s=1,current_audit_q_pred_mA_s=1;};
struct Imu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct Roller {bool good=true;bool ok()const{return good;}};
struct Resettable {int resets=0;void reset(){++resets;}void interrupt(){++resets;}};
struct Adaptive {bool broken=false;void breakPair(){broken=true;}};
enum class EnergyControlAutonomousHalfCycleState {WAIT_PEAK,WAIT_ZERO_CROSS,PULSE_ACTIVE};
struct Logger {unsigned events=0;void addEnergyControlAutonomousZeroCrossEvent(const PsramLogger::EnergyControlAutonomousZeroCrossEvent&){++events;}};
class ExperimentRunner {public:
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
 void finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t){++finished;}
 void restoreUnsentStallRescue(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,bool,uint32_t);
 bool dispatchTiltPriority(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,bool,uint32_t);
 void acceptOptionalEarlyMotion(uint32_t,float,float);
 void acceptMotionCrossing(bool,uint32_t t,float rate,float,float,float,float) {
   auto prev_valid=energy_control_autonomous_last_accepted_zero_cross_valid_;
   auto prev_ms=energy_control_autonomous_last_accepted_zero_cross_ms_;
   energy_control_autonomous_zero_cross_consumed_for_peak_=true;
   energy_control_autonomous_last_accepted_zero_cross_ms_=t;
   PsramLogger::EnergyControlAutonomousZeroCrossEvent e;e.physical_next_peak_side=rate>0?1:-1;e.zero_cross_time_ms=t;
   dispatchTiltPriority(e,t,prev_valid,prev_ms);
 }
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
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t d,uint16_t w,int16_t m){
   ++calls;sign=d;width=w;ma=m;
   if(!begin_ok && optional_early_event_ && simulate_write_uncertain)optional_early_write_uncertain_=true;
   return begin_ok;
 }
 bool tryTiltPriority(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
 bool tryTiltPriorityOptionalEarly(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
 bool tryTiltPriorityImpl(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,bool*);
 bool tryTiltPriorityRescue(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
};
// METHOD_SOURCE
// DISPATCH_SOURCE
// EARLY_WRAPPER_SOURCE
using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
int main(){
 // The real selector consumes the same cap in all downstream proposals,
 // records post-guard command only after a successful ordinary write, and
 // does not let early/rescue writes pollute that independent history.
 {ExperimentRunner r;bool healthy=false;r.tilt_priority_.ordinaryResult(1,100,20,true);
  Event first;first.physical_next_peak_side=1;
  assert(r.tryTiltPriorityImpl(first,600,&healthy) && healthy && first.output_executed);
  assert(first.pulse_width_ms==30 && r.tilt_priority_.ordinaryRiseCap(1,601,150)==40);
  Event other;other.physical_next_peak_side=-1;
  assert(r.tryTiltPriorityImpl(other,700,&healthy) && other.output_executed);
  assert(other.pulse_width_ms>30);
  r.optional_early_event_=true;Event early;early.physical_next_peak_side=1;
  assert(r.tryTiltPriorityImpl(early,800,&healthy) && early.output_executed && early.pulse_width_ms<=15);
  assert(r.tilt_priority_.ordinaryRiseCap(1,801,150)==40);
  r.optional_early_event_=false;r.begin_ok=false;Event failed;failed.physical_next_peak_side=1;
  assert(r.tryTiltPriorityImpl(failed,900,&healthy) && !failed.output_executed);
  assert(r.tilt_priority_.ordinaryRiseCap(1,901,150)==150);
 }
 // A paired weak result defers the limited early pulse. Exercise the real
 // dispatch rollback and then let the ordinary ZERO send its own proposal.
 {ExperimentRunner r;r.tilt_priority_.peak(-1,100,3.f);
  tilt_priority::Input in;in.now_ms=300;in.preset_deg=8;in.target_deg=8;
  in.crossing_rate_dps=10;in.next_side=1;in.hardware_cap_ms=150;in.hardware_healthy=true;
  auto c=r.tilt_priority_.propose(in);assert(c.selected && !c.coast);
  r.tilt_priority_.crossing(1,300);r.tilt_priority_.confirm(1,300,c,true);
  r.tilt_priority_.peak(1,480,2.f);
  r.optional_early_event_=true;Event early;early.physical_next_peak_side=1;
  assert(r.dispatchTiltPriority(early,600,true,400));
  assert(!early.output_executed && r.calls==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==400);
  r.optional_early_event_=false;r.energy_control_autonomous_zero_cross_consumed_for_peak_=true;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=620;
  Event zero;zero.physical_next_peak_side=1;zero.zero_cross_time_ms=620;
  assert(r.dispatchTiltPriority(zero,620,true,400) && zero.output_executed);
  assert(r.calls==1 && r.finished==1 && r.width>15);}
 // The explicitly invoked future adapter refuses past-centre and repeated
 // attempts and returns to the ordinary zero path after failed output.
 {ExperimentRunner r;r.tilt_priority_.peak(-1,900,3.f);r.begin_ok=false;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=800;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.acceptOptionalEarlyMotion(1200,10.f,.2f);assert(r.calls==0);
  r.acceptOptionalEarlyMotion(1200,10.f,-.2f);assert(r.calls==1);
  assert(!r.optional_early_event_ && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
  r.acceptOptionalEarlyMotion(1202,10.f,-.15f);assert(r.calls==1);}
 // A candidate before the minimum half-cycle is not an attempted decision.
 {ExperimentRunner r;r.tilt_priority_.peak(-1,900,3.f);
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=1000;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.acceptOptionalEarlyMotion(1200,10.f,-.2f);
  assert(r.calls==0 && r.optional_early_attempted_peak_ms_==0);
  r.acceptOptionalEarlyMotion(1250,10.f,-.15f);
  assert(r.calls==1 && r.energy_control_autonomous_zero_cross_consumed_for_peak_);}
 // Failed transport may still have delivered a command; stop and never
 // reopen this half-cycle, even if the ordinary centre follows immediately.
 {ExperimentRunner r;r.tilt_priority_.peak(-1,900,3.f);r.begin_ok=false;
  r.simulate_write_uncertain=true;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=800;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  r.acceptOptionalEarlyMotion(1200,10.f,-.2f);
  assert(r.calls==1 && r.status_.emergency_stop);
  assert(r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==1200);
  r.begin_ok=true;r.acceptOptionalEarlyMotion(1202,10.f,-.15f);
  assert(r.calls==1 && r.log_obj.events==1);}
 // Future optional earlier no-output attempt preserves the real centre
 // opportunity and restores the previous crossing timestamp.
 {ExperimentRunner r;r.optional_early_event_=true;r.roller_obj.good=false;
  Event early;assert(r.dispatchTiltPriority(early,1200,true,800));
  assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
  assert(r.log_obj.events==1 && r.finished==0);}
 // Early coast must also leave centre armed; the normal selector is allowed
 // to reconsider at the real zero crossing.
 {ExperimentRunner r;r.optional_early_event_=true;r.tilt_priority_.peak(-1,900,8.f);
  Event early;early.physical_next_peak_side=1;assert(r.dispatchTiltPriority(early,1200,true,800));
  assert(!early.output_executed && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);}
 // Rejected early motor command leaves the next centre attempt eligible.
 {ExperimentRunner r;r.optional_early_event_=true;r.begin_ok=false;r.tilt_priority_.peak(-1,900,3.f);
  r.foot_trial_pending_ms_=777;r.foot_trial_pending_generation_=12;r.foot_trial_pending_effective_=true;
  r.status_.current_audit_q_target_mA_s=9;r.status_.current_audit_q_pred_mA_s=8;
  const auto old_tilt=r.tilt_priority_;const auto old_guard=r.wheel_cycle_guard_;
  Event early;early.physical_next_peak_side=1;assert(r.dispatchTiltPriority(early,1200,true,800));
  assert(!early.output_executed && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==800);
  assert(r.foot_trial_pending_ms_==777 && r.foot_trial_pending_generation_==12 && r.foot_trial_pending_effective_);
  assert(r.status_.current_audit_q_target_mA_s==9 && r.status_.current_audit_q_pred_mA_s==8);
  tilt_priority::Input in;in.now_ms=1400;in.preset_deg=8;in.target_deg=8;in.crossing_rate_dps=10;
  in.next_side=1;in.hardware_cap_ms=150;in.hardware_healthy=true;
  const auto before=old_tilt.propose(in),after=r.tilt_priority_.propose(in);
  assert(before.selected==after.selected && before.coast==after.coast && before.width_ms==after.width_ms);
  auto guard_a=old_guard,guard_b=r.wheel_cycle_guard_;
  assert(guard_a.limit(1,20,true,1400,60)==guard_b.limit(1,20,true,1400,60));
  // Mirror the centre entry's accepted-timestamp bookkeeping, then execute
  // the real dispatch after the failed early motor attempt.
  r.optional_early_event_=false;r.begin_ok=true;
  r.energy_control_autonomous_zero_cross_consumed_for_peak_=true;
  r.energy_control_autonomous_last_accepted_zero_cross_ms_=1400;
  Event centre;centre.physical_next_peak_side=1;centre.zero_cross_time_ms=1400;
  assert(r.dispatchTiltPriority(centre,1400,true,800) && centre.output_executed);
  assert(r.energy_control_autonomous_pending_peak_ && r.energy_control_autonomous_pending_next_side_==1 &&
         r.energy_control_autonomous_pending_zero_ms_==1400);
  assert(r.finished==1 && r.energy_control_autonomous_last_accepted_zero_cross_ms_==1400);}
 // A selected early command that actually reaches the actuator owns its
 // half-cycle; do not re-arm a duplicate centre pulse.
 {ExperimentRunner r;r.optional_early_event_=true;r.tilt_priority_.peak(-1,900,3.f);
  Event early;early.physical_next_peak_side=1;assert(r.dispatchTiltPriority(early,1200,true,800));
  assert(early.output_executed && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(early.pulse_width_ms>0 && early.pulse_width_ms<=15 && r.width<=15);
  assert(r.finished==1 && r.log_obj.events==0);}
 // Current ordinary no-output semantics are untouched unless the future
 // early path is explicitly selected.
 {ExperimentRunner r;r.tilt_priority_.peak(-1,900,8.f);
  Event normal;normal.physical_next_peak_side=1;assert(r.dispatchTiltPriority(normal,1200,true,800));
  assert(!normal.output_executed && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);}
 // Seed the real tilt controller with an overshoot-induced negative cap of
 // 30ms and a completed 8-degree startup; no actuator shortcuts are used.
 auto prepare_negative=[](ExperimentRunner& r) {
  uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  for(int n=0;n<3;++n)for(int side:{-1,1}) {
   tilt_priority::Choice c;c.selected=true;c.direction=static_cast<int8_t>(side);
   c.width_ms=r.tilt_priority_.widthLimit(side);c.current_ma=1200;c.target_deg=r.tilt_priority_.target();
   r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
   r.tilt_priority_.confirm(static_cast<int8_t>(side),t,c,true);
   r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,side<0?8.f:3.f);t+=300;
  }
  assert(r.tilt_priority_.widthLimit(-1)==30);
  for(int n=0;n<14;++n)for(int side:{-1,1}) {
   tilt_priority::Choice c;c.selected=true;c.coast=true;c.direction=static_cast<int8_t>(side);c.target_deg=r.tilt_priority_.target();
   r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
   r.tilt_priority_.confirm(static_cast<int8_t>(side),t,c,true);
   r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,3.f);t+=300;
  }
  assert(r.tilt_priority_.target()==8.f);
  for(int i=0;i<6;++i) {
   const uint32_t start=t-4300+i*700,end=start+700;
   r.negative_cap_recovery_.peak(1,start+300,6.f,true,true);
   r.negative_cap_recovery_.peak(-1,end,5.f,true,true);
   r.negative_cap_recovery_.footCycle(start,end,3.f,.5f,true,true,true);
  }
  assert(r.negative_cap_recovery_.candidate(t,8.f,30,true,true));
  return t;
 };
#if defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL
 // Seed paired peaks from the v7 pre-turn context, and exercise the actual
 // ordinary runner. The cap must remain effective through downstream guards.
 {ExperimentRunner r;uint32_t t=prepare_negative(r);
  for(int side:{1,-1}) {
   tilt_priority::Choice c;c.selected=true;c.direction=static_cast<int8_t>(side);
   c.width_ms=30;c.current_ma=1200;c.target_deg=8;
   r.tilt_priority_.crossing(c.direction,t);r.tilt_priority_.confirm(c.direction,t,c,true);
   r.tilt_priority_.peak(c.direction,t+100,side>0?5.76f:6.14f);t+=300;
  }
  Event e;e.physical_next_peak_side=1;e.zero_cross_abs_rate_dps=46.7354f;bool healthy=false;
  assert(r.tryTiltPriorityImpl(e,t,&healthy) && healthy && e.output_executed);
  assert(e.fast_trim_before_ms-e.fast_trim_cap_ms==5 && e.pulse_width_ms==e.fast_trim_cap_ms);
 }
#endif
#if defined(SWAY_BALANCE_TRIAL) && SWAY_BALANCE_TRIAL
 // Actual runner path: a paired asymmetric observation reduces only the
 // ordinary positive proposal, and downstream guards cannot raise the cap.
 {ExperimentRunner r;uint32_t t=prepare_negative(r);
  for(int side:{1,-1}) {
   tilt_priority::Choice c;c.selected=true;c.direction=static_cast<int8_t>(side);
   c.width_ms=30;c.current_ma=1200;c.target_deg=8;
   r.tilt_priority_.crossing(c.direction,t);r.tilt_priority_.confirm(c.direction,t,c,true);
   r.tilt_priority_.peak(c.direction,t+100,side>0?7.25f:4.5f);t+=300;
  }
  Event e;e.physical_next_peak_side=1;e.zero_cross_abs_rate_dps=46.7354f;bool healthy=false;
  assert(r.tryTiltPriorityImpl(e,t,&healthy) && healthy && e.output_executed);
  assert(e.sway_trim_before_ms>e.sway_trim_cap_ms && e.sway_trim_before_ms-e.sway_trim_cap_ms<=5);
  assert(e.pulse_width_ms==e.sway_trim_cap_ms);
#if defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL
  assert(e.fast_trim_cap_ms==0); // overlapping thresholds never stack
#endif
 }
#endif
 // The production runner compares candidate and baseline through identical
 // guards, claims a trial only after a real accepted +1..5ms command, and
 // records a distinct audit marker. Fixture readings are constructed, not
 // an assertion about physical robot response.
 auto reduce_plus=[](ExperimentRunner& r,uint32_t t){
  for(int n=0;n<4;++n)for(int side:{1,-1}) {tilt_priority::Choice c;c.selected=true;c.current_ma=1200;
   c.direction=static_cast<int8_t>(side);c.width_ms=r.tilt_priority_.widthLimit(side);c.target_deg=8;
   r.tilt_priority_.crossing(c.direction,t);r.tilt_priority_.confirm(c.direction,t,c,true);
   r.tilt_priority_.peak(c.direction,t+180,side>0?10.f:6.f);t+=300;}
  for(int side:{1,-1}) {tilt_priority::Choice c;c.selected=true;c.current_ma=1200;
   c.direction=static_cast<int8_t>(side);c.width_ms=r.tilt_priority_.widthLimit(side);c.target_deg=8;
   r.tilt_priority_.crossing(c.direction,t);r.tilt_priority_.confirm(c.direction,t,c,true);
   r.tilt_priority_.peak(c.direction,t+180,side>0?4.f:6.f);t+=300;}
  return t;};
 {ExperimentRunner r;auto t=reduce_plus(r,prepare_negative(r));
  for(unsigned i=0;i<4;++i){const auto start=t-2800+i*700;
    r.positive_sway_probe_.cycle(start,start+700,2,2,4,6,true,true,true,false);}
  Event e;assert(r.tryTiltPriority(e,t));
  assert(e.output_executed && (e.foot_span_flags & Event::PositiveSwayProbe));
  assert(e.weak_probe_executed && e.weak_probe_requested_ms>e.weak_probe_baseline_ms);
  assert(e.weak_probe_requested_ms-e.weak_probe_baseline_ms<=5 && r.positive_sway_probe_.active());
  assert(!r.long_foot_balance_.trialActive());}
 {ExperimentRunner r;auto t=reduce_plus(r,prepare_negative(r));
  for(unsigned i=0;i<4;++i){const auto start=t-2800+i*700;
    r.positive_sway_probe_.cycle(start,start+700,2,2,4,6,true,true,true,false);}
  r.begin_ok=false;Event e;assert(r.tryTiltPriority(e,t));
  assert(!e.output_executed && !r.positive_sway_probe_.active() && !r.positive_sway_probe_.attempted());}
 // Even when all old negative-recovery evidence is present, the ordinary
 // protected command is sent and the trial cannot start.
 {ExperimentRunner r;auto t=prepare_negative(r);Event e;e.physical_next_peak_side=-1;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==30);
  assert(!e.weak_probe_executed && !r.negative_cap_recovery_.active());
  Event held;held.physical_next_peak_side=-1;assert(r.tryTiltPriority(held,t+300));
  assert(held.pulse_width_ms==30 && !held.weak_probe_executed);}
 // A rejected send cannot start the trial or advance its output history.
 {ExperimentRunner r;auto t=prepare_negative(r);r.begin_ok=false;Event e;e.physical_next_peak_side=-1;
  const auto guard_before=r.wheel_cycle_guard_;
  assert(r.tryTiltPriority(e,t));assert(!e.output_executed && !e.weak_probe_executed);
  assert(!r.negative_cap_recovery_.active() && !r.negative_cap_recovery_.attempted());
  auto reference=guard_before;assert(r.wheel_cycle_guard_.limit(1,300.f,true,t+100,60)==reference.limit(1,300.f,true,t+100,60));}
 // Guard-reduced baseline and candidate still use the same prior history.
 {ExperimentRunner r;auto t=prepare_negative(r);
  r.wheel_cycle_guard_.limit(1,-400.f,true,t-100,60);auto reference=r.wheel_cycle_guard_;
  assert(reference.limit(-1,-400.f,true,t,35)==27);
  Event e;e.physical_next_peak_side=-1;e.pre_input_wheel_speed_rpm=-400.f;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==23);
  assert(!e.weak_probe_executed && !r.negative_cap_recovery_.active());}
 // A timing cap that makes both candidates equal is not a successful probe.
 {ExperimentRunner r;auto t=prepare_negative(r);
  tilt_priority::Choice c;c.selected=true;c.direction=-1;c.width_ms=30;c.current_ma=1200;c.target_deg=8;
  r.tilt_priority_.crossing(-1,t-400);r.tilt_priority_.confirm(-1,t-400,c,true);
  r.tilt_priority_.peak(-1,t-340,3.f);
  Event e;e.physical_next_peak_side=-1;assert(r.tryTiltPriority(e,t));
  assert(e.output_executed && e.pulse_width_ms==30);
  assert(!e.weak_probe_executed);
  assert(!r.negative_cap_recovery_.active() && !r.negative_cap_recovery_.attempted());}
 // A legacy active probe is cancelled and cannot block the positive path.
 {ExperimentRunner r;auto t=prepare_negative(r);
  r.negative_cap_recovery_.command(t-100,30,35,true,true);
  assert(r.negative_cap_recovery_.active());
  Event positive;positive.physical_next_peak_side=1;assert(r.tryTiltPriority(positive,t));
  assert(positive.output_executed && !r.negative_cap_recovery_.active());}
 // The real dispatch records the old limit and bounded additional change.
 {ExperimentRunner r;
  r.wheel_cycle_guard_.peak(-1,5.3f,100,true);r.wheel_cycle_guard_.peak(1,5.4f,450,true);
  r.wheel_cycle_guard_.peak(-1,5.51f,800,true);r.wheel_cycle_guard_.peak(1,5.23f,1150,true);
  Event a;a.physical_next_peak_side=1;a.pre_input_wheel_speed_rpm=-270.f;
  assert(r.tryTiltPriority(a,1200));
  Event b;b.physical_next_peak_side=-1;b.pre_input_wheel_speed_rpm=14.f;
  assert(r.tryTiltPriority(b,1500));
  assert(b.output_executed && b.wheel_guard_trial_eligible && b.wheel_guard_extra_ms>0);
  assert(b.wheel_guard_extra_ms<=5 && b.pulse_width_ms+b.wheel_guard_extra_ms==b.wheel_guard_legacy_ms);
 }
 // No 1200mA foot-width trial interrupts the 4.5 degree startup stage.
 {ExperimentRunner r;Event e;assert(r.tryTiltPriority(e,500));
  assert(e.output_executed && e.pulse_width_ms==60 && e.foot_span_baseline_ms==0);
  assert(!(e.foot_span_flags & Event::Span_selected));}
 // Five valid observations qualify a proposal. These inputs verify dispatch,
 // not physical foot response. Guard comparisons use identical wheel history.
 auto prepare_foot=[](ExperimentRunner& r) {
  uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  for(int n=0;n<6;++n)for(int side: {-1,1}) {
    tilt_priority::Choice coast;coast.selected=true;coast.coast=true;
    coast.direction=static_cast<int8_t>(side);coast.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
    r.tilt_priority_.confirm(static_cast<int8_t>(side),t,coast,true);
    r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,3.f);t+=300;
  }
  assert(r.tilt_priority_.target()==6.f);
  for(int i=0;i<5;++i)r.long_foot_balance_.observe(t-600+i*100,t-500+i*100,3.f,1.f,6.f,5.f,true,true);
  return t;
 };
 // Production camera path groups six adjacent complete cycles into three
 // paired observations before the actual protected output is selected.
 {ExperimentRunner r;auto t=prepare_foot(r);r.long_foot_balance_.reset();
  for(int i=0;i<6;++i) {
   const uint32_t start=t-1800+i*300,end=start+300;
   r.long_foot_balance_.observeCycle(start,end,3.f,1.f,6.f,5.f,true,true,true);
  }
  Event e;assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==65);
  assert(r.long_foot_balance_.offset()==5 && r.foot_trial_pending_effective_);}
 {ExperimentRunner r;auto t=prepare_foot(r);Event e;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==65);
  assert(e.foot_span_baseline_ms==60 && e.foot_span_requested_ms==65);
  assert(e.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_ms_==t);
  assert(r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());
  Event held;assert(r.tryTiltPriority(held,t+200));
  assert(held.pulse_width_ms==65 && held.foot_span_baseline_ms==60 && held.foot_span_requested_ms==65);
  assert(held.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());}
 {ExperimentRunner r;auto t=prepare_foot(r);
  r.wheel_cycle_guard_.limit(-1,400.f,true,t-100,60);
  auto expected=r.wheel_cycle_guard_;assert(expected.limit(1,400.f,true,t,65)==49);
  Event e;e.pre_input_wheel_speed_rpm=400.f;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==49);
  assert(e.foot_span_baseline_ms==45 && e.foot_span_requested_ms==49);
  assert(e.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());
  assert(r.wheel_cycle_guard_.limit(-1,200.f,true,t+300,40)==expected.limit(-1,200.f,true,t+300,40));}
 // A later observed cycle may have no intervention left after the timing
 // cap, even though the controller still holds its previously sent offset.
 // Such a cycle is recorded, but cannot serve as post-trial evidence.
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));assert(r.foot_trial_pending_effective_);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t+200);r.tilt_priority_.confirm(1,t+200,coast,true);
  r.tilt_priority_.peak(1,t+285,3.f);
  Event clipped;assert(r.tryTiltPriority(clipped,t+300));assert(clipped.output_executed);
  assert(r.long_foot_balance_.trialActive());
  assert(clipped.weak_probe_baseline_ms==clipped.weak_probe_requested_ms);
  assert(!r.foot_trial_pending_effective_ && !clipped.weak_probe_executed);}
 // A second trial compares +10 against the already accepted +5, not against
 // ordinary 0. No post-confirm generation may reuse the older reference.
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));const auto first_generation=r.long_foot_balance_.trialGeneration();
  for(int i=0;i<5;++i){const uint32_t ms=t+500+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  assert(r.long_foot_balance_.state()==long_foot_balance::Controller::State::Hold);
  for(int i=0;i<5;++i){const uint32_t ms=t+2200+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  const uint32_t t2=t+3700;r.tilt_priority_.peak(-1,t2-50,3.f);
  Event second;assert(r.tryTiltPriority(second,t2));assert(second.output_executed);
  assert(r.long_foot_balance_.trialGeneration()!=first_generation);
  assert(r.long_foot_balance_.trialReferenceOffset()==5 && r.long_foot_balance_.offset()==10);
  assert(second.weak_probe_baseline_ms==65 && second.weak_probe_requested_ms==70);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());}
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));const auto gen=r.long_foot_balance_.trialGeneration();
  for(int i=0;i<5;++i){const uint32_t ms=t+500+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  for(int i=0;i<5;++i){const uint32_t ms=t+2200+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  const uint32_t t2=t+3700;r.tilt_priority_.peak(-1,t2-140,3.f);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t2-100);r.tilt_priority_.confirm(1,t2-100,coast,true);
  r.tilt_priority_.peak(1,t2-15,3.f);
  Event blocked;assert(r.tryTiltPriority(blocked,t2));assert(blocked.output_executed);
  assert(r.long_foot_balance_.trialGeneration()==gen && r.long_foot_balance_.offset()==5);
  assert(blocked.weak_probe_baseline_ms==blocked.weak_probe_requested_ms);
  assert(!r.foot_trial_pending_effective_ && !blocked.weak_probe_executed);}
 {ExperimentRunner r;auto t=prepare_foot(r);r.begin_ok=false;Event e;
  r.wheel_cycle_guard_.limit(-1,300.f,true,t-100,60);auto untouched=r.wheel_cycle_guard_;
  assert(r.tryTiltPriority(e,t));assert(!e.output_executed && !(e.foot_span_flags & Event::Span_selected));
  assert(r.long_foot_balance_.offset()==0);
  assert(!r.foot_trial_pending_effective_ && r.foot_trial_pending_ms_==0);
  assert(r.wheel_cycle_guard_.limit(1,300.f,true,t+100,60)==untouched.limit(1,300.f,true,t+100,60));}
 {ExperimentRunner r;auto t=prepare_foot(r);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t-100);r.tilt_priority_.confirm(1,t-100,coast,true);
  r.tilt_priority_.peak(1,t-10,3.f);
  Event e;assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==60);
  assert(e.foot_span_baseline_ms==60 && e.foot_span_requested_ms==60);
  assert(!(e.foot_span_flags & Event::Span_selected));assert(r.long_foot_balance_.offset()==0);}
 // The optional camera trial must never silently cap an ordinary 1200 mA
 // positive pulse at 100 ms. Grow the existing ramp through actual paired
 // peak/command confirmations, then test the real runner dispatch.
 {ExperimentRunner r;uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  auto action=[&](int side){
    tilt_priority::Input in;in.now_ms=t;in.preset_deg=8;in.target_deg=r.tilt_priority_.target();
    in.crossing_rate_dps=10;in.next_side=static_cast<int8_t>(side);in.hardware_cap_ms=150;in.hardware_healthy=true;
    auto choice=r.tilt_priority_.propose(in);assert(choice.selected&&!choice.coast);
    r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
    r.tilt_priority_.confirm(static_cast<int8_t>(side),t,choice,true);
    r.tilt_priority_.peak(static_cast<int8_t>(side),t+200,3.f);t+=400;
  };
  for(int n=0;n<12;++n){action(-1);action(1);}
  action(-1);assert(r.tilt_priority_.widthLimit(1)>100);
  Event e;e.physical_next_peak_side=1;e.zero_cross_abs_rate_dps=10;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms>100);
  assert(e.foot_span_baseline_ms==0 && !(e.foot_span_flags & Event::Span_selected));
 }
 // Replay the measured startup peaks and crossings from run 302. The new
 // output widths are not a prediction of the resulting physical peaks.
 {ExperimentRunner r;r.tilt_priority_.peak(1,220,2.21185f);
  Event e;e.physical_next_peak_side=-1;e.zero_cross_time_ms=414;e.zero_cross_abs_rate_dps=18.645f;
  assert(r.tryTiltPriority(e,416));
  assert(e.valid&&e.output_executed&&e.work_control_branch==15);
  assert(r.sign==-1&&r.ma==1200&&r.width==59&&e.pulse_end_ms==475);
  assert(r.energy_control_autonomous_pending_probe_&&r.energy_control_autonomous_pending_next_side_==-1);
  assert(r.energy_control_autonomous_pending_target_deg_==4.5f);
  assert(std::isnan(r.energy_control_autonomous_pending_model_energy_j_));
  assert(!r.work_positive_pending_&&!r.work_negative_pending_&&r.adaptive_cg_.broken);
  r.tilt_priority_.peak(-1,641,6.57367f);
  Event e2;e2.zero_cross_time_ms=865;e2.zero_cross_abs_rate_dps=36.6997f;assert(r.tryTiltPriority(e2,867));
  assert(e2.valid&&!e2.output_executed&&e2.reason==1&&r.calls==1);
  assert(e2.command_current_mA==0&&e2.pulse_width_ms==0&&e2.pulse_start_ms==0);
  assert(e2.q_command_direction==0&&e2.q_command_mA_s==0);
  assert(r.energy_control_autonomous_pending_peak_&&r.energy_control_autonomous_pending_probe_);
  assert(r.energy_control_autonomous_pending_mode_==peak_error_adaptation::Coast);
  // Synthetic return peak after one coast permits a short recovery pulse.
  r.tilt_priority_.peak(1,1098,3.8f);
  Event e3;e3.physical_next_peak_side=-1;assert(r.tryTiltPriority(e3,1300));
  assert(e3.valid&&e3.output_executed&&e3.pulse_width_ms==15&&r.calls==2);
  r.tilt_priority_.peak(-1,1500,3.5f);
  Event e4;assert(r.tryTiltPriority(e4,1700));
  assert(e4.output_executed&&r.calls==3&&e4.command_current_mA==1200);
 }
 {ExperimentRunner r;Event e;r.begin_ok=false;assert(r.tryTiltPriority(e,1000));
  assert(!e.valid&&!e.output_executed&&e.pulse_width_ms==0&&e.reason==12);
  assert(!r.energy_control_autonomous_pending_peak_);
  r.tilt_priority_.peak(1,1190,1.f);
  assert(r.tilt_priority_.widthLimit(1)==60);}
 // A failed rescue preserves the measured-controller history and must not
 // interrupt the independent weak-side adaptation. The next ordinary
 // decision matches an otherwise identical untouched runner.
 {ExperimentRunner r,control; r.stall_rescue_event_=true;r.begin_ok=false;
  Event trial;trial.physical_next_peak_side=-1;trial.zero_cross_abs_rate_dps=2;
  assert(r.tryTiltPriority(trial,1000));
  assert(!trial.output_executed && trial.reason==12 && r.weak_side_assist_.resets==0);
  r.stall_rescue_event_=false;r.begin_ok=true;
  Event a,b;a.physical_next_peak_side=b.physical_next_peak_side=-1;
  assert(r.tryTiltPriority(a,1300) && control.tryTiltPriority(b,1300));
  assert(a.output_executed && b.output_executed && a.pulse_width_ms==b.pulse_width_ms);
  assert(r.tilt_priority_.widthLimit(-1)==control.tilt_priority_.widthLimit(-1));
  assert(r.wheel_cycle_guard_.limit(1,300.f,true,1600,40)==
         control.wheel_cycle_guard_.limit(1,300.f,true,1600,40));
 }
 // Run 803's recorded 1858 ms crossing and immediately prior peak,
 // evaluated as an isolated hypothetical decision at the initial stage.
 {ExperimentRunner r;r.tilt_priority_.peak(1,1698,4.67317f);
  Event e;e.zero_cross_time_ms=1858;e.physical_next_peak_side=-1;e.zero_cross_abs_rate_dps=41.0152f;
  assert(r.tryTiltPriority(e,1860));
  assert(e.valid&&e.output_executed&&e.pulse_width_ms==34&&r.sign==-1);}
 // Signed direction remains determined by the crossing side; the rate is
 // the positive interpolated magnitude, for both signs.
 {ExperimentRunner slow,medium,fast,negative;Event a,b,c,d;
  a.zero_cross_abs_rate_dps=15;b.zero_cross_abs_rate_dps=30;c.zero_cross_abs_rate_dps=60;
  d.zero_cross_abs_rate_dps=30;d.physical_next_peak_side=-1;
  assert(slow.tryTiltPriority(a,1000)&&medium.tryTiltPriority(b,1000));
  assert(fast.tryTiltPriority(c,1000)&&negative.tryTiltPriority(d,1000));
  assert(a.pulse_width_ms==60&&b.pulse_width_ms<60&&b.pulse_width_ms>20);
  assert(c.valid&&c.output_executed&&c.pulse_width_ms==20&&fast.calls==1);
  assert(d.pulse_width_ms==b.pulse_width_ms&&negative.sign==-1);
 }
 {ExperimentRunner r;Event e;e.zero_cross_abs_rate_dps=NAN;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.zero_cross_abs_rate_dps=0;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;r.status_.emergency_stop=true;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;r.imu_obj.good=false;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.pre_input_wheel_speed_age_us=999999;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.pre_input_wheel_speed_rpm=1200.f;assert(r.tryTiltPriority(e,1000));
  assert(e.valid&&e.output_executed&&r.ma==1200);}
 {ExperimentRunner r;Event e;e.target_peak_deg=10;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 std::cout<<"actual runner staged output, coast recovery and sensor guards PASS\n";
}
