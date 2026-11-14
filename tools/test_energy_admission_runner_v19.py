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
admission_source_text = (ROOT/'tools/test_energy_transfer_runner_v18.py').read_text(encoding='utf8')
admission_fixture_prefix, admission_fixture_tail = admission_source_text.split("\nfixture += r'''\n", 1)
exec(compile(admission_fixture_prefix, str(ROOT/'tools/test_energy_transfer_runner_v18.py'), 'exec'), globals())
fixture = '#include "energy_admission_v19.h"\n#include "fore_aft_motion_guard.h"\n' + fixture
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
fixture = fixture.replace('return 329.547119f;', 'return 345.005126953f;')
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
int main(){
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
  r.acceptOptionalEarlyMotion(1200,32,-1.2f);assert(r.status_.pulse_active);
  const auto a=r.walking_energy_active_pulse_;
  assert(a.energy_admission_applies && a.energy_admission_allowed && a.energy_unsupported_mask==0);
  assert(a.early_budget_admitted && a.guarded_ms==a.decision.width_ms && a.guarded_ms==a.requested_ms);
  assert(a.output_start_ms==1207 && a.guarded_ms>0 && r.walking_half_cycle_budget_.mainUsed());
  r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);assert(r.status_.pulse_active);
  clock19(r,a.output_start_ms+a.guarded_ms+1);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_half_cycle_budget_.spentMs()==a.guarded_ms+1u);
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
  if(full)assert(r.walking_energy_active_audit_==-1 && r.walking_energy_audit_[191].completion_reason==77);}
 // The real fore/aft observer vetoes new optional, ordinary and return-aid
 // outputs during its outward candidate without inventing corrective J.
 for(unsigned mode=0;mode<3;++mode){ExperimentRunner r;ready19(r);positive_bias(r,.00018f);
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
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
        '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1',
        '-DFOOT_SUPPORT_ENERGY=1','-DEARLY_ENERGY_MAIN=1','-DPULSE_LIFECYCLE_FIX=1',
        '-DSWAY_BALANCE_TRIAL=1','-DWALKING_RETURN_ENERGY=1','-DWALKING_TRANSFER_V18=1',
        '-DWALKING_ADMISSION_V19=1','-I'+str(ROOT/'src'),str(temp/'fixture.cpp'),'-o',str(temp/'fixture.exe')],check=True)
    subprocess.run([str(temp/'fixture.exe')],check=True)
