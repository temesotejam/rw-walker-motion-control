"""Exercise real v19 optional support admission and retained ZERO execution.

Uses the v18 fixture's real selector/dispatch/start/timer/stop/finalizer. The
transport and crossing-event producer remain doubles; the fore/aft guard is
given real identity-quaternion reference observations for this energy test.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
admission_source_text = (ROOT/'test/fixtures/energy/test_energy_transfer_runner_v18.py').read_text(encoding='utf8')
admission_fixture_prefix, admission_fixture_tail = admission_source_text.split("\nfixture += r'''\n", 1)
exec(compile(admission_fixture_prefix, str(ROOT/'test/fixtures/energy/test_energy_transfer_runner_v18.py'), 'exec'), globals())
fixture = '#include "energy_admission_v19.h"\n#include "fore_aft_motion_guard.h"\n#if WALKING_RIGHT_ARRIVAL\n#include "right_arrival_energy.h"\n#endif\n' + fixture
fixture += admission_fixture_tail.split('static void ready18(', 1)[0]
# Preserve the production emergency-stop pulse path while omitting unrelated
# LEDs, capture and logger status I/O from this host double.
fixture = re.sub(r'(enum class ExperimentState\s*\{)([^}]*)(\};)',
    lambda m: m[1] + m[2].rstrip().rstrip(',') + ', ESTOP ' + m[3], fixture, count=1)
emergency_stub = 'void requestEmergencyStop(const char*){status_.emergency_stop=true;}'
assert fixture.count(emergency_stub) == 1
fixture = fixture.replace(emergency_stub,
    'void requestEmergencyStop(const char*){walking_energy_stop_reason_=3;stopMotor();'
    'status_.state=ExperimentState::ESTOP;status_.emergency_stop=true;}')
# Match the last recorded v18 positive input's Vbat-derived current goal.
fixture = fixture.replace('float predictCurrentGoalMa(int,float)const{return 329.547119f;}',
                          'float simulated_goal_ma=345.005126953f;'
                          ' float predictCurrentGoalMa(int,float)const{return simulated_goal_ma;}')
fixture = r'''
#if WALKING_TURN_OBSERVER
#include "gait_turn_energy.h"
struct GaitStateFixture {
 gait_turn::Status value;
 const gait_turn::Status& status() const {return value;}
};
#endif
#if SUPPORT_TRANSFER_CARRY2
#include "receiving_arc_aid.h"
#include "periodic_foot_state.h"
#define esp_timer_get_time micros
struct StreamDouble {
 periodic_foot::Estimator estimator;
 bool rescuePair(unsigned run,uint64_t now,uint64_t epoch,uint32_t seq,uint64_t image,uint64_t received,
                 periodic_foot::Side& left,periodic_foot::Side& right)const{
  return run==3 && estimator.rescuePair(now,epoch,seq,image,received,left,right);
 }
} periodicFeet;
#endif
''' + fixture
fixture = fixture.replace('struct Status {', 'struct Status {\n#if WALKING_TURN_OBSERVER\n'
    ' struct { bool gyro_valid = false; } steering;\n#endif\n', 1)
fixture = fixture.replace('struct EnergyControlAutonomousZeroCrossEvent {',
    'struct EnergyControlAutonomousZeroCrossEvent {\n#if WALKING_TURN_OBSERVER\n'
    ' uint8_t event_kind = 0;\n#endif\n', 1)
fixture = fixture.replace('class ExperimentRunner {public:', '''class ExperimentRunner {public:
#if WALKING_SMALL_STEP_AB
 bool small_step=false;
 bool smallStepBaselineSelected()const{return small_step;}
 uint32_t angle_late_blocked_peak_ms_=0,angle_last_check_ms_=0;
 void tryAngleEarlyCandidate(uint32_t,uint32_t,float,float);
 void decideAngleEarlyCandidate(uint32_t,uint32_t,float,float);
#endif
#if WALKING_TURN_OBSERVER
 GaitStateFixture gait_turn_run_;
#endif
#if SUPPORT_TRANSFER_CARRY2
 periodic_foot::Packet periodic_foot_packet_;
 void observeSupportTransfer(uint32_t,uint32_t,float,float){}  // Detached shadow observer.
#endif
''')
angle_begin = runner.index('void ExperimentRunner::tryAngleEarlyCandidate(')
angle_end = runner.index('\n}\n#endif\n#endif', runner.index('void ExperimentRunner::decideAngleEarlyCandidate(',angle_begin)) + 2
fixture += '\n#if WALKING_SMALL_STEP_AB\n' + runner[angle_begin:angle_end] + '\n#endif\n'
fixture += r'''
static void refresh19(ExperimentRunner& r){
 fore_aft_motion_guard::Input in;in.valid=true;in.now_us=mock_us;in.sample_us=mock_us;
 in.quaternion.w=1;in.quaternion.x=in.quaternion.y=in.quaternion.z=0;
 assert(!r.observeWalkingForeAft(in));
 assert(r.walking_fore_aft_last_.reference_ready && r.walking_fore_aft_last_.allow_output);
}
static void ready19(ExperimentRunner& r,float same=6,float previous=8){
 ready(r,same,previous);assert(r.walking_half_cycle_budget_.sourcePeak(r.energy_control_autonomous_last_peak_ms_));
 r.energy_control_autonomous_pending_peak_=false;refresh19(r);
}
static void clock19(ExperimentRunner& r,unsigned t){clock_at(r,t);refresh19(r);}
static void positive_bias(ExperimentRunner& r,float bias){
 const float target=lateral_energy::potential(r.lateral_energy_geometry_,r.tilt_priority_.target());
 r.walking_arrival_feedback_.observe(1,1000,.0007f+bias,.0007f,0,target,true);
 r.walking_arrival_feedback_.observe(1,1100,.0007f+bias,.0007f,0,target,true);
}
static void main19(ExperimentRunner& r,unsigned t,float rate,float angle=0){
 clock19(r,t);r.acceptMotionCrossing(false,t,rate,rate>0?-.01f:.01f,angle,1,float(t));
}
static void preserved(const ExperimentRunner& r){
 assert(!r.status_.pulse_active && !r.walking_energy_completion_pending_);
 assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
 assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==600);
 assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
 assert(r.walking_half_cycle_budget_.remaining()==80 && !r.walking_half_cycle_budget_.mainUsed());
 assert(!r.energy_control_autonomous_pending_peak_);
}
static void rejected_command(const Event& e){
 assert(!e.valid && !e.output_executed && e.work_control_branch==24);
 assert(e.command_current_mA==0 && e.pulse_width_ms==0 && e.pulse_start_ms==0 && e.pulse_end_ms==0);
 assert(e.q_command_mA_s==0 && e.predicted_input_work_j==0 && e.solver_selected_integer_width_ms==0);
 assert(near(e.unapplied_energy_request_j,e.commanded_energy_deficit_j));
}
static bool fore_aft(ExperimentRunner& r,unsigned t,float angle,float rate){
 clock_at(r,t);fore_aft_motion_guard::Input in;in.valid=true;in.sample_us=in.now_us=mock_us;
 in.quaternion.w=std::cos(angle*lateral_energy::kRadPerDeg*.5f);
 in.quaternion.x=std::sin(angle*lateral_energy::kRadPerDeg*.5f);
 in.quaternion.y=in.quaternion.z=0;in.gyro_rad_s.x=rate*lateral_energy::kRadPerDeg;
 return r.observeWalkingForeAft(in);
}
#if WALKING_TURN_OBSERVER
static void confirmed_right_weak(ExperimentRunner& r){
 auto& state=r.gait_turn_run_.value;
 state.enabled=true;
 auto& cycle=state.latest;
 cycle.reason=gait_turn::Reason::Confirmed;cycle.weak_side=1;cycle.turn_sign=-1;cycle.streak=2;
 cycle.start_ms=100;cycle.mid_ms=480;cycle.end_ms=900;
 cycle.left_deg=7;cycle.right_deg=1;cycle.left_swing_deg=8;cycle.right_swing_deg=1;
 cycle.yaw_delta_deg=-4;cycle.pair_yaw_delta_deg=-7;
 state.confirmed_ms=cycle.end_ms;state.confirmed_side=cycle.weak_side;
 r.status_.steering.gyro_valid=true;
 auto& raw=r.walking_return_frame_;
 raw.run_id=r.status_.run_id;raw.sequence=11;raw.image_us=mock_us-35000;raw.received_us=mock_us-10000;
 raw.timestamp_valid=raw.frame_valid=raw.zero_ready=true;
 raw.left_detected=raw.right_detected=raw.left_range_ok=raw.right_range_ok=true;
 raw.left_deg=7.f;raw.right_deg=.4f;
}
static void turn_main_setup(ExperimentRunner& r,int side){
 ready19(r,10,side>0?7:8);confirmed_right_weak(r);
 r.energy_control_autonomous_last_peak_side_=-side;
 unsigned index=side>0?1:0;
 r.walking_energy_peaks_[index]=10;r.walking_energy_peak_ms_[index]=700;
 if(side>0)positive_bias(r,.00018f);
}
static void turn_main_response(){
 assert(gait_turn_energy::widthStep(9)==0 && gait_turn_energy::widthStep(10)==1 &&
        gait_turn_energy::widthStep(39)==3 && gait_turn_energy::widthStep(80)==5);
 for(int side:{1,-1}){
  ExperimentRunner reference;turn_main_setup(reference,side);
  reference.gait_turn_run_.value.enabled=false;
  main19(reference,1200,side*36.75f);
  const auto before=reference.walking_energy_active_pulse_;
  assert(before.executed && before.guarded_ms==(side>0?66:61));
  ExperimentRunner r;turn_main_setup(r,side);
  main19(r,1200,side*36.75f);
  const auto a=r.walking_energy_active_pulse_;
  assert(a.executed && a.turn_cycle_end_ms==900 && a.turn_base_ms==before.guarded_ms);
  assert(a.turn_candidate_ms==(side>0?63:64) && a.guarded_ms==a.turn_candidate_ms);
  assert(side*a.turn_adjustment_j<0 && near(before.decision.request_j+a.turn_adjustment_j,a.decision.request_j));
  const float work_change=a.decision.predicted_work_j-before.decision.predicted_work_j;
  const int step=gait_turn_energy::widthStep(before.guarded_ms);
  assert(work_change*a.turn_adjustment_j>0 && std::abs(int(a.guarded_ms)-int(before.guarded_ms))<=step);
  const auto& geometry=r.lateral_energy_geometry_;
  const float step_j=std::fabs(lateral_energy::potential(geometry,8.f-side*.20f)-lateral_energy::potential(geometry,8.f));
  const auto edge_ms=static_cast<uint16_t>(before.guarded_ms-side*step);
  const float edge_q=lateral_energy::charge(r.walking_energy_current_,edge_ms,a.start_i0_ma,a.start_goal_ma);
  const float edge_work=lateral_energy::work(lateral_energy::coefficients(side),
      36.75f*lateral_energy::kRadPerDeg,edge_q);
  const float expected_request=before.decision.predicted_work_j-
      side*(7.f/9.f)*std::min(step_j,std::fabs(edge_work-before.decision.predicted_work_j));
  assert(near(a.decision.request_j,expected_request));
  assert(r.status_.pulse_active && r.roller_obj.last_command==side*1200);
  assert(r.last_finished_event.pulse_width_ms==a.guarded_ms);
  assert(near(r.last_finished_event.predicted_input_work_j,a.decision.predicted_work_j));
  assert(r.walking_half_cycle_budget_.mainUsed());
  clock19(r,a.output_start_ms+a.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(!r.status_.pulse_active && r.walking_half_cycle_budget_.spentMs()==a.guarded_ms);
  assert(r.walking_energy_active_pulse_.completion.valid &&
         r.walking_energy_active_pulse_.completion.actual_elapsed_ms==a.guarded_ms);
 }
 // The longer negative candidate can exceed the fitted Q range even when
 // the original negative command was supported. Retain the ordinary event.
 {ExperimentRunner r;turn_main_setup(r,-1);
  r.energy_control_autonomous_last_peak_amplitude_deg_=7;
  main19(r,1200,-36.75f);const auto& a=r.walking_energy_active_pulse_;
  assert(a.executed && a.turn_candidate_ms>a.turn_base_ms && a.guarded_ms==a.turn_base_ms);
  assert(lateral_energy::charge(r.walking_energy_current_,a.turn_candidate_ms,a.start_i0_ma,a.start_goal_ma)>
         lateral_energy::coefficients(-1).charge_max_mAs);
  assert(a.guarded_ms==62 && a.turn_adjustment_j==0);}
 // Confirmations from the wrong sign, a prior or expired cycle, or a missing
 // camera/gyro observation must reproduce the ordinary guarded pulse exactly.
 for(unsigned mode=0;mode<8;++mode){
  ExperimentRunner r;turn_main_setup(r,1);
  auto& cycle=r.gait_turn_run_.value.latest;
  switch(mode){
   case 0:cycle.weak_side=-1;break;
   case 1:cycle.turn_sign=1;break;
   case 2:cycle.start_ms=500;cycle.end_ms=750;break;
   case 3:cycle.reason=gait_turn::Reason::Pending;break;
   case 4:r.walking_return_frame_.image_us=mock_us-75001;break;
   case 5:r.walking_return_frame_.run_id++;break;
   case 6:r.walking_return_frame_.right_detected=false;break;
   case 7:r.status_.steering.gyro_valid=false;break;
  }
  main19(r,1200,36.75f);
  const auto& a=r.walking_energy_active_pulse_;
  assert(a.executed && a.guarded_ms==66 && a.turn_adjustment_j==0);
 }
 // A transport failure cannot become an executed correction or debit any on-time.
 {ExperimentRunner r;turn_main_setup(r,1);r.roller_obj.write_ok=false;main19(r,1200,36.75f);
  const auto& a=r.walking_energy_audit_[0];
  assert(!a.executed && !r.status_.pulse_active && a.turn_adjustment_j<0);
  assert(r.walking_half_cycle_budget_.spentMs()==0 && !r.walking_half_cycle_budget_.mainUsed());}
 std::cout<<"turn feedback: signed ZERO work and duration, freshness, scope, failed write and completion PASS\n";
}
#endif
int main(){
#if WALKING_RIGHT_ARRIVAL
 // Measured 33125639, 1.705 s ZERO. The body reached +7.95 deg after
 // the original 54 ms request. Feed its pre-command values to live dispatch.
 {ExperimentRunner r;ready19(r,4.56917f,3.86125f);r.tilt_priority_.reset();
  r.energy_control_autonomous_last_peak_ms_=1505;
  r.walking_energy_peak_ms_[1]=1070;
  assert(r.walking_half_cycle_budget_.sourcePeak(1505));
  assert(r.walking_energy_current_.prepare(.031652f));
  r.simulated_goal_ma=353.6989f;clock19(r,1708);
  Event e;e.zero_cross_rate_dps=e.zero_cross_abs_rate_dps=25.5256f;
  e.previous_peak_time_ms=1505;e.previous_peak_amplitude_deg=3.86125f;
  e.physical_next_peak_side=1;e.zero_cross_time_ms=1705;e.target_peak_deg=8;
  e.pre_input_wheel_speed_valid=true;e.pre_input_wheel_speed_age_us=2000;
  e.detector_angle_after_deg=.06943f;e.i0_estimated_mA=-2.332f;
  e.vbat_mV=8040;e.pre_input_wheel_speed_rpm=-390.27f;
  bool healthy=false;
  assert(r.tryWalkingEnergy(e,1708,&healthy) && healthy && e.output_executed);
  const auto& a=r.walking_energy_active_pulse_;
  assert(a.right_arrival.eligible && a.requested_ms==54 && a.right_base_ms==54);
  assert(e.pulse_width_ms==23 && a.guarded_ms==23 && r.walking_half_cycle_budget_.reservedMs()==23);
  assert(a.right_arrival.original_j>a.right_final_j && a.right_base_q>a.decision.q_mas);
 }
 // Same fresh real-ZERO request, routed through normal transport and timer.
 for(unsigned mode=0;mode<7;++mode){
   ExperimentRunner r;ready19(r,4.56f,3.86f);
   if(mode!=3)r.tilt_priority_.reset();
   if(mode==1)r.small_step=true;
   if(mode==4)r.roller_obj.write_ok=false;
   if(mode==5)r.walking_fore_aft_last_.allow_output=false;
   if(mode==6)confirmed_right_weak(r);
   if(mode==5)r.acceptMotionCrossing(false,1200,26,-.01f,.01f,1,1200);
   else main19(r,1200,mode==2?18.f:26.f,.01f);
   const auto a=r.walking_energy_audit_[0];
   if(mode==1 || mode==3){assert(!a.right_arrival_checked && a.executed);continue;}
   if(mode==5){assert(!a.right_arrival_checked && !a.executed);continue;}
   assert(a.right_arrival_checked);
   if(mode==2){assert(!a.right_arrival.eligible && a.right_arrival.reason==right_arrival_energy::Reason::OutsideSupport);continue;}
   assert(a.right_arrival.eligible && a.right_arrival.reason==right_arrival_energy::Reason::Reduced);
   assert(a.right_base_ms>a.guarded_ms && a.guarded_ms>0 && a.guarded_ms<=a.right_arrival.width_ms);
   assert(a.right_arrival.original_j>a.right_final_j && a.right_base_q>a.decision.q_mas);
   if(mode==4){assert(!a.executed && !r.status_.pulse_active && !r.walking_half_cycle_budget_.mainUsed());continue;}
   assert(a.executed && r.status_.pulse_active && a.guarded_ms==r.last_finished_event.pulse_width_ms);
   clock19(r,a.output_start_ms+a.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
   assert(!r.status_.pulse_active && r.walking_half_cycle_budget_.spentMs()==a.guarded_ms);
 }
 {ExperimentRunner r;ready19(r,2.6f,3.f);r.tilt_priority_.reset();main19(r,1200,39,.01f);
  const auto a=r.walking_energy_audit_[0];
  assert(a.right_arrival.eligible && a.right_arrival.reason==right_arrival_energy::Reason::Coast);
  assert(!a.executed && a.decision.coast && a.decision.unmet_j>0 && a.decision.predicted_work_j==0);
  assert(r.walking_half_cycle_budget_.mainUsed() && r.walking_half_cycle_budget_.spentMs()==0 && !r.status_.pulse_active);
 }
#endif
#if WALKING_SMALL_STEP_AB
 // A v14 partial contribution and a turn3 defer on the same state.
 {ExperimentRunner r;ready19(r,6.47749f,5.21523f);r.small_step=true;
  positive_bias(r,.0001457f);confirmed_right_weak(r);
  r.tryAngleEarlyCandidate(1200,mock_us,-1.35f,39.2354f); // ~34 ms before centre.
  assert(!r.status_.pulse_active && !r.walking_energy_count_);
  clock19(r,1206);
  r.predicted_current_end_ms_=mock_ms;r.predicted_signed_current_end_mA_=-3.9053f;
  r.tryAngleEarlyCandidate(1206,mock_us,-.99409f,39.2354f); // ~25 ms before centre.
  const auto a=r.walking_energy_audit_[0];
  assert(a.executed && a.optional && r.status_.pulse_active && r.last_finished_event.work_control_branch==19);
  assert(a.guarded_ms==15 && a.early_budget_admitted && a.decision.unmet_j>0 && a.decision.q_mas>0);
  assert(a.arrival_bias_j==0 && a.turn_adjustment_j==0 && r.walking_half_cycle_budget_.mainUsed());
  clock19(r,a.output_start_ms+16);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(!r.status_.pulse_active && r.walking_half_cycle_budget_.spentMs()==16);
  const auto sent=r.roller_obj.writes;main19(r,1240,35);assert(r.roller_obj.writes==sent);
 }
 {ExperimentRunner r;ready19(r);r.small_step=true;
  r.tryAngleEarlyCandidate(1200,mock_us,-.65f,39.2354f); // Already within 20 ms.
  assert(!r.walking_energy_count_ && !r.status_.pulse_active);
  main19(r,1220,31);assert(r.status_.pulse_active);
 }
 // Zero work at the early opportunity keeps the real crossing available.
 {ExperimentRunner r;ready19(r);r.small_step=true;
  r.acceptOptionalEarlyMotion(1200,90,-1.f);
  assert(!r.walking_energy_audit_[0].executed && !r.walking_energy_audit_[0].early_budget_admitted);
  preserved(r);
  main19(r,1240,25,.15f);const auto& a=r.walking_energy_active_pulse_;
  assert(a.executed && !a.optional && r.last_finished_event.work_control_branch==19);
  assert(a.decision.potential_j==0 && a.guarded_ms>15);
 }
 // The opt-in comparison retains machine protection, duplicate suppression,
 // transport-failure handling and causal freshness.
 for(unsigned mode=0;mode<4;++mode){
  ExperimentRunner r;ready19(r);r.small_step=true;
  if(mode==0)r.walking_fore_aft_last_.allow_output=false;
  if(mode==1)r.energy_control_autonomous_last_motion_sample_us_=mock_us-11000;
  if(mode==2)r.roller_obj.write_ok=false;
  if(mode==3)r.walking_return_event_=r.stall_rescue_event_=true;
  if(mode==3)r.acceptMotionCrossing(true,1200,-15,4.05f,4.f,1,1200);
  else r.acceptOptionalEarlyMotion(1200,39.2354f,-.99409f);
  assert(!r.status_.pulse_active && !r.walking_energy_audit_[0].executed);
  assert(!r.walking_half_cycle_budget_.mainUsed() && r.walking_half_cycle_budget_.spentMs()==0);
  if(mode==2)assert(r.status_.emergency_stop);
 }
#endif
#if SUPPORT_TRANSFER_STALL_V3
 // A measured shortfall can spend only the part of this half-cycle left after
 // reserving a completed main pulse on the same side. Exercise real dispatch.
 for(unsigned mode=0;mode<4;++mode){
  ExperimentRunner r;ready19(r);r.energy_control_autonomous_last_peak_side_=1;
  if(mode!=2)r.walking_main_reserve_.observe(-1,true,mode==1?80:62);
  if(mode==3)r.roller_obj.write_ok=false;
  r.walking_return_event_=r.stall_rescue_event_=true;
  r.walking_return_choice_.request_j=.00009f;r.walking_return_choice_.deceleration_dps2=75;
  r.walking_return_observation_.evidence=return_observation_policy::Evidence::MeasuredArcShortfall;
  r.acceptMotionCrossing(true,1200,-15,1.7f,1.65f,1,1200);
  auto a=r.walking_energy_audit_[0];
  assert(a.return_event && a.return_evidence==5 && a.return_work_j>0);
  if(mode!=0){
   assert(!a.executed && !r.status_.pulse_active && r.walking_half_cycle_budget_.remaining()==80);
   if(mode!=3)assert(a.main_reserve_ms==80 && a.arc_aid.reason==receiving_arc_aid::Reason::NoMainRoom);
   continue;
  }
  assert(a.executed && a.side==-1 && a.main_reserve_ms==62 && a.guarded_ms>0 && a.guarded_ms<=18);
  r.walking_return_event_=r.stall_rescue_event_=false;
  clock19(r,a.output_start_ms+a.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(!r.status_.pulse_active && r.walking_half_cycle_budget_.remaining()>=62 && !r.walking_half_cycle_budget_.mainUsed());
  main19(r,1400,-25);
  assert(r.status_.pulse_active && r.walking_half_cycle_budget_.mainUsed());
 }
#endif
#if WALKING_TURN_OBSERVER
 turn_main_response();
#endif
#if SUPPORT_TRANSFER_CARRY2
 {
  receiving_arc_aid::Input shape;shape.side=-1;shape.observed_shape=true;shape.body_deg=4;
  shape.speed_dps=15;shape.decel_dps2=180;shape.fore_aft_deg=0;
  shape.left_deg=shape.left_est=16;shape.right_deg=shape.right_est=1;
  shape.left_rate=shape.right_rate=0;shape.base_j=.000061f;shape.target_j=.001f;
  auto result=receiving_arc_aid::evaluate(shape);
  assert(result.reason==receiving_arc_aid::Reason::Eligible && result.terminal_deg<0 && near(result.horizon_s,.15f,1e-6f));
  shape.left_rate=-80;shape.right_rate=10;
  assert(receiving_arc_aid::evaluate(shape).reason==receiving_arc_aid::Reason::ShapeReturning);
  shape.left_rate=shape.right_rate=0;shape.left_est=3;shape.right_est=0;shape.body_deg=8;
  assert(receiving_arc_aid::evaluate(shape).reason==receiving_arc_aid::Reason::UncertainPivot);
  shape.left_est=16;shape.fore_aft_deg=11;
  assert(receiving_arc_aid::evaluate(shape).reason==receiving_arc_aid::Reason::OutsideModel);
 }
 // Same mechanical event with a recent slot, an expired slot with a newer
 // causal paired image, missing identity, no main history, or no shape.
 unsigned arc_cases=9;
#if SUPPORT_TRANSFER_ARC_FEEDBACK
 arc_cases=15;
 {
  receiving_arc_aid::ArrivalFeedback feedback;
  lateral_energy::Geometry geometry;
  for (int invalid=0;invalid<4;++invalid) {
   feedback.completedAid(200,290,-1.89f,32,.000058f);
   feedback.peak(geometry,invalid==0?1:-1,450,invalid==1?2.f:1.59f,invalid==2?201:200,310,invalid!=3);
   assert(!feedback.recent(500));
  }
  feedback.completedAid(200,290,-1.89f,32,.000058f);
  feedback.peak(geometry,-1,450,1.59f,200,310,true);
  assert(!feedback.recent(449) && feedback.recent(3450) && !feedback.recent(3451));
  feedback.completedAid(500,590,-1.89f,32,0);
  assert(!feedback.recent(591));
 }
#endif
 for(unsigned mode=0;mode<arc_cases;++mode){
  ExperimentRunner r;ready19(r);r.energy_control_autonomous_last_peak_side_=1;
#if WALKING_TURN_OBSERVER
  confirmed_right_weak(r);
#endif
  r.walking_return_event_=r.stall_rescue_event_=true;
  r.walking_return_choice_.request_j=.000061f;r.walking_return_choice_.deceleration_dps2=180;
  r.walking_return_observation_.evidence=return_observation_policy::Evidence::ObservedShape;
  r.walking_return_observation_.foot_age_ms=36;
  auto& est=periodicFeet.estimator;est.start(mock_us-200000);
  for(int i=0;i<4;++i){
   const auto image=mock_us-96000+i*20000;
   est.observe(100+i,image,image+18000,true,true,true,16,true,true,1);
   if(i==2)assert(est.publish(mock_us-(mode>=7?24000:26000)));
  }
  auto& raw=r.walking_return_frame_;raw.run_id=3;raw.left_deg=16;raw.right_deg=1;
  raw.image_us=mock_us-36000;raw.received_us=mock_us-18000;raw.sequence=103;
  if(mode==0 || mode==3 || mode==4 || mode==6)assert(est.publish(mock_us-900));
  r.periodic_foot_packet_=est.packet;
  if(mode!=3)r.walking_main_reserve_.observe(-1,true,mode>=9?10:40);
#if SUPPORT_TRANSFER_ARC_FEEDBACK
  if(mode>=9) {
   r.walking_arc_feedback_.completedAid(350,500,-1.89f,mode==12?12:72,mode==11?.000012f:.0003f);
   r.walking_arc_feedback_.peak(r.lateral_energy_geometry_,-1,600,1.45f,350,560,mode!=13);
   if(mode==14)r.walking_main_reserve_.observe(-1,true,78);
  }
#endif
  if(mode==2)++raw.sequence;
  if(mode==4)r.walking_return_observation_.evidence=return_observation_policy::Evidence::RecognitionDropout;
  if(mode==5)r.periodic_foot_packet_.epoch_us++;
  if(mode==8)raw.run_id=4;
  if(mode==6 || mode==10)r.roller_obj.write_ok=false;
  r.acceptMotionCrossing(true,1200,-15,4.05f,4.f,1,1200);
  const auto& a=r.walking_energy_audit_[0];
#if WALKING_TURN_OBSERVER
  assert(a.turn_cycle_end_ms==0 && a.turn_adjustment_j==0);
#endif
  if(mode==6 || mode==10){
   assert(!a.executed && a.arc_aid.reason==receiving_arc_aid::Reason::CommandVeto);
#if SUPPORT_TRANSFER_ARC_FEEDBACK
   if(mode==10)assert(a.arc_feedback_added_j>0 && r.walking_arc_feedback_.recent(1200));
#endif
   continue;
  }
  assert(a.executed && r.status_.pulse_active);
#if SUPPORT_TRANSFER_ARC_FEEDBACK
  if(mode>=9) {
   if(mode==9) {
    assert(a.arc_feedback_peak_ms==600 && a.arc_feedback_added_j>0);
    assert(a.guarded_ms>a.arc_feedback_base_ms && a.guarded_ms<=70);
    assert(near(a.decision.request_j,a.return_work_j+a.arc_aid.extra_j+a.arc_feedback_added_j));
    assert(a.decision.predicted_work_j<=.0003f);
   } else {
    assert(a.arc_feedback_added_j==0);
    assert((a.arc_feedback_peak_ms==600)==(mode==11 || mode==12));
   }
   const unsigned end=a.output_start_ms+a.guarded_ms;
   clock19(r,end);r.updateEnergyControlAutonomousPulse(mock_ms);
   assert(!r.walking_arc_feedback_.recent(end));
   assert(r.walking_half_cycle_budget_.spentMs()==a.guarded_ms);
   continue;
  }
#endif
  if(mode<2 || mode==7){
   assert(a.arc_foot_source==(mode==0?1:2));
   assert(a.arc_aid.reason==receiving_arc_aid::Reason::Applied && a.arc_aid.extra_j>0);
   assert(a.guarded_ms>a.base_aid_ms && a.guarded_ms<=40 && a.guarded_ms<=a.arc_candidate_ms);
   assert(near(a.decision.request_j,a.return_work_j+a.arc_aid.extra_j));
  }else{
   assert(a.guarded_ms==a.base_aid_ms && near(a.decision.request_j,a.return_work_j));
   assert(a.arc_aid.reason==(mode==3?receiving_arc_aid::Reason::NoMainRoom:receiving_arc_aid::Reason::NoFreshShape));
  }
 }
#endif
 // The recorded 15954ms state still selects64ms and covers its model J. The
 // v19 gate rejects only its relocation because passive39.2354>37.5379dps.
 {ExperimentRunner r;ready19(r,6.47749f,5.21523f);positive_bias(r,.0001457f);
  r.predicted_current_end_ms_=mock_ms;r.predicted_signed_current_end_mA_=-3.9053f;
  r.acceptOptionalEarlyMotion(1200,39.2354f,-.99409f);
  assert(r.roller_obj.writes==0 && r.walking_energy_count_==1);preserved(r);
  const auto rejected=r.walking_energy_audit_[0];
  assert(rejected.requested_ms==64 && rejected.guarded_ms==64 && rejected.decision.width_ms==64);
  assert(walking_return_energy::earlyAdmissible(rejected.decision));
  assert(rejected.energy_admission_applies && !rejected.energy_admission_allowed && rejected.energy_unsupported_mask==1);
  assert(rejected.energy_admission_reason==unsigned(energy_admission_v19::Reason::PassiveRateUnsupported));
  assert(rejected.early_budget_checked && !rejected.early_budget_admitted && !rejected.executed);
  rejected_command(r.log_obj.last);
  // No retry of the same optional candidate, but a genuine ZERO still owns
  // the full inherited allowance and keeps its normal J-based cap result.
  r.acceptOptionalEarlyMotion(1202,35,-.8f);assert(r.roller_obj.writes==0);
  main19(r,1240,35);assert(r.status_.pulse_active && r.last_finished_event.work_control_branch==24);
  const auto& main=r.walking_energy_active_pulse_;
  assert(!main.energy_admission_applies && main.energy_admission_allowed);
  assert(main.energy_admission_reason==unsigned(energy_admission_v19::Reason::OrdinaryZeroExempt));
  assert(main.envelope_before_ms==80 && main.guarded_ms<=80 && r.walking_half_cycle_budget_.mainUsed());
 }
 // A supported positive relocation is still exactly the energy result; the
 // gate adds neither an arbitrary reduction nor a second input at ZERO.
 {ExperimentRunner r;ready19(r);positive_bias(r,.00018f);r.roller_obj.write_delay_ms=7;
#if WALKING_TURN_OBSERVER
  confirmed_right_weak(r);
#endif
  r.acceptOptionalEarlyMotion(1200,32,-1.2f);assert(r.status_.pulse_active);
  const auto a=r.walking_energy_active_pulse_;
#if WALKING_TURN_OBSERVER
  assert(a.turn_cycle_end_ms==0 && a.turn_adjustment_j==0);
#endif
  assert(a.energy_admission_applies && a.energy_admission_allowed && a.energy_unsupported_mask==0);
  assert(a.early_budget_admitted && a.guarded_ms==a.decision.width_ms && a.guarded_ms==a.requested_ms);
  assert(a.output_start_ms==1207 && a.guarded_ms>0 && r.walking_half_cycle_budget_.mainUsed());
#if SUPPORT_TRANSFER_CARRY2
  assert(r.walking_main_reserve_.forSide(1)==80);
#endif
  r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);assert(r.status_.pulse_active);
  clock19(r,a.output_start_ms+a.guarded_ms+1);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_half_cycle_budget_.spentMs()==a.guarded_ms+1u);
#if SUPPORT_TRANSFER_CARRY2
  assert(r.walking_main_reserve_.forSide(1)==a.guarded_ms+1u && r.walking_main_reserve_.forSide(-1)==80);
#endif
  const auto writes=r.roller_obj.writes;main19(r,1400,32);assert(r.roller_obj.writes==writes);
 }
 // A valid covered candidate whose Q exceeds the fitted positive range must
 // also preserve ZERO. No substitute narrower command is manufactured.
 {ExperimentRunner r;ready19(r,8,8);r.acceptOptionalEarlyMotion(1200,32,-1.5f);
  assert(r.roller_obj.writes==0);preserved(r);const auto& a=r.walking_energy_audit_[0];
  assert(walking_return_energy::earlyAdmissible(a.decision) && a.decision.q_mas>13.27138f);
  assert(a.energy_unsupported_mask==4 && a.energy_admission_reason==unsigned(energy_admission_v19::Reason::ChargeUnsupported));
  assert(a.requested_ms==a.guarded_ms && a.guarded_ms==a.decision.width_ms);
  rejected_command(r.log_obj.last);}
 // Negative rate domains have no overlap. Both kinds of rejection are
 // explicit, while the later ordinary negative input remains operational.
 for(float rate:{27.3215f,36.1672f}){ExperimentRunner r;ready19(r);
  r.energy_control_autonomous_last_peak_side_=1;r.acceptOptionalEarlyMotion(1200,-rate,.8f);
  assert(r.roller_obj.writes==0);preserved(r);const auto& a=r.walking_energy_audit_[0];
  assert(!a.energy_admission_allowed && a.energy_unsupported_mask!=0);
  main19(r,1240,-rate);assert(r.status_.pulse_active && !r.walking_energy_active_pulse_.energy_admission_applies);}
 // A downstream wheel reduction that leaves uncovered J still rejects the
 // relocation under the existing budget contract, preserving the guard too.
 {ExperimentRunner r;ready19(r);positive_bias(r,.00018f);r.simulated_rpm=600;
  r.wheel_cycle_guard_.limit(-1,600,true,1100,80);
  unsigned char before[sizeof(r.wheel_cycle_guard_)];std::memcpy(before,&r.wheel_cycle_guard_,sizeof(r.wheel_cycle_guard_));
  r.acceptOptionalEarlyMotion(1200,32,-1.2f);assert(r.roller_obj.writes==0);preserved(r);
  const auto& a=r.walking_energy_audit_[0];assert(a.guarded_ms<a.requested_ms && a.decision.unmet_j>0);
  assert(a.energy_admission_reason==unsigned(energy_admission_v19::Reason::NoPositiveCoveredBudget));
  assert(std::memcmp(before,&r.wheel_cycle_guard_,sizeof(r.wheel_cycle_guard_))==0);}
 // Low-speed explicit-J aid is exempt from this relocation gate. Its actual
 // completion and following normal ZERO still share the same80ms envelope,
 // including with the diagnostic archive already full.
 for(bool full:{false,true}){ExperimentRunner r;ready19(r);r.energy_control_autonomous_last_peak_side_=1;
  if(full){r.walking_energy_count_=192;r.walking_energy_audit_[191].completion_reason=77;}
  r.walking_return_event_=true;r.stall_rescue_event_=true;r.walking_return_choice_.request_j=.0001f;
  r.acceptMotionCrossing(true,1200,-15,9.5f,9.4f,1,1200);
  r.walking_return_event_=false;r.stall_rescue_event_=false;assert(r.status_.pulse_active);
  const auto aid=r.walking_energy_active_pulse_;
  assert(aid.return_event && !aid.energy_admission_applies && aid.energy_admission_allowed);
  assert(aid.energy_admission_reason==unsigned(energy_admission_v19::Reason::ExplicitReturnAidExempt));
  assert(!aid.decision.work_rate_supported && !r.energy_control_autonomous_pending_peak_);
  clock19(r,aid.output_start_ms+aid.guarded_ms+1);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
  const auto remaining=r.walking_half_cycle_budget_.remaining();
  assert(remaining==79-aid.guarded_ms);main19(r,1400,-25);
  assert(r.status_.pulse_active && r.walking_energy_active_pulse_.guarded_ms<=remaining);
  assert(!r.walking_energy_active_pulse_.energy_admission_applies && r.walking_return_pending_);
  if(full)assert(r.walking_energy_active_audit_==-1 && r.walking_energy_audit_[191].completion_reason==77);
#if SUPPORT_TRANSFER_CARRY2
  const auto next=r.walking_energy_active_pulse_;
  assert(r.walking_main_reserve_.forSide(-1)==80);
  clock19(r,next.output_start_ms+next.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_main_reserve_.forSide(-1)==next.guarded_ms);
  r.walking_main_reserve_.observe(-1,false,80);  // More relevant completed post-aid experience wins.
  assert(r.walking_main_reserve_.forSide(-1)==next.guarded_ms);
#endif
 }
 // The real fore/aft observer vetoes new optional, ordinary and return-aid
 // outputs during its outward candidate without inventing corrective J.
 for(unsigned mode=0;mode<3;++mode){ExperimentRunner r;ready19(r);positive_bias(r,.00018f);
#if WALKING_TURN_OBSERVER
  confirmed_right_weak(r);
#endif
  assert(!fore_aft(r,1202,8,60));assert(!r.walkingForeAftAllowsOutput());
  assert(r.walking_fore_aft_last_.reason==fore_aft_motion_guard::Reason::Candidate);
  if(mode==0)r.acceptOptionalEarlyMotion(1202,32,-1.2f);
  else {r.walking_return_event_=r.stall_rescue_event_=mode==2;
   r.walking_return_choice_.request_j=.0001f;r.acceptMotionCrossing(mode==2,1202,-25,.2f,0,1,1202);}
  assert(r.roller_obj.writes==0 && !r.status_.pulse_active && !r.walking_half_cycle_budget_.active());
  assert(r.walking_half_cycle_budget_.remaining()==80 && !r.walking_half_cycle_budget_.mainUsed());}
 // A stale guard cannot be bypassed by direct entry to the real pulse
 // starter, even if no preceding healthy selector was run.
 {ExperimentRunner r;ready19(r);mock_us+=10001;
  assert(!r.walkingForeAftAllowsOutput());
  assert(!r.beginEnergyControlAutonomousPulse(mock_ms,1200,1,10,1200));
  assert(r.roller_obj.writes==0 && !r.walking_half_cycle_budget_.active());}
 // Sustained outward divergence during a real accepted main runs the real
 // observer -> emergency stop -> completion finalizer -> elapsed ledger.
 for(bool full:{false,true}){ExperimentRunner r;ready19(r,2,8);
  if(full){r.walking_energy_count_=192;r.walking_energy_audit_[191].completion_reason=77;}
  main19(r,1200,25);assert(r.status_.pulse_active);const auto start=r.walking_energy_active_pulse_.output_start_ms;
  assert(r.walking_energy_active_pulse_.guarded_ms>42);
  for(unsigned elapsed:{2u,12u,22u,32u}){
   assert(!fore_aft(r,start+elapsed,8.f+(elapsed-2)*.06f,60));assert(r.status_.pulse_active);}
  assert(fore_aft(r,start+42,10.4f,60));
  assert(r.status_.emergency_stop && r.status_.state==ExperimentState::ESTOP && !r.status_.pulse_active);
  assert(r.walking_fore_aft_last_.reason==fore_aft_motion_guard::Reason::Diverging);
  const auto& a=r.walking_energy_active_pulse_;
  assert(a.completion_reason==3 && a.completion.actual_elapsed_ms==42 && !a.completion.duration_completed);
#if SUPPORT_TRANSFER_CARRY2
  assert(r.walking_main_reserve_.forSide(1)==80);
#endif
  assert(r.walking_half_cycle_budget_.spentMs()==42 && r.walking_half_cycle_budget_.remaining()==38);
  assert(!r.walking_energy_completion_pending_ && r.walking_energy_active_audit_==-1);
  if(full)assert(r.walking_energy_audit_[191].completion_reason==77 && r.walking_energy_count_==192);
  else assert(r.walking_energy_audit_[0].completion_reason==3 && r.walking_energy_audit_[0].completion.actual_elapsed_ms==42);
  const auto spent=r.walking_half_cycle_budget_.spentMs();r.stopMotor();assert(r.walking_half_cycle_budget_.spentMs()==spent);}
 std::cout<<"v19 real runner: unsupported64/Q/negative defer, ZERO retained, supportedJ unchanged, aid/actual80ms/archive-full, candidate/stale veto and guardstop completion PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='energy_admission_runner_v19_') as folder:
    temp = Path(folder);(temp/'fixture.cpp').write_text(fixture, encoding='utf8')
    for carry, turn, feedback, stall in ((0,0,0,0),(1,0,0,0),(1,1,0,0),(1,1,1,0),(1,1,1,1)):
        subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
        '-DSUPPORT_TRANSFER_CARRY2='+str(carry),'-DSUPPORT_TRANSFER_V39='+str(carry),'-DWALKING_TURN_OBSERVER='+str(turn),
        '-DSUPPORT_TRANSFER_ARC_FEEDBACK='+str(feedback),
        '-DSUPPORT_TRANSFER_STALL_V3='+str(stall),
        '-DWALKING_SMALL_STEP_AB='+str(stall),
        '-DWALKING_RIGHT_ARRIVAL='+str(stall),
        '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1',
        '-DFOOT_SUPPORT_ENERGY=1','-DEARLY_ENERGY_MAIN=1','-DPULSE_LIFECYCLE_FIX=1',
        '-DSWAY_BALANCE_TRIAL=1','-DWALKING_RETURN_ENERGY=1','-DWALKING_TRANSFER_V18=1',
        '-DWALKING_ADMISSION_V19=1','-I'+str(ROOT/'src'),str(temp/'fixture.cpp'),'-o',str(temp/'fixture.exe')],check=True)
        subprocess.run([str(temp/'fixture.exe')],check=True)
