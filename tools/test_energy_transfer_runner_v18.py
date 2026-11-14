"""Exercise v18's real aid/main selection, cutoff, timer and completion.

Transport and sensor/event construction are host doubles. The crossing router
is extracted from the real motion method, and executes the real cutoff and
dispatch. These are software contracts, not physical walking validation.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'tools/test_energy_main_lifecycle_v16.py').read_text(encoding='utf8')
prefix = source.split("\nfixture+=r'''\n", 1)[0]
exec(compile(prefix, str(ROOT / 'tools/test_energy_main_lifecycle_v16.py'), 'exec'), globals())
fixture = ('#include "half_cycle_energy_budget.h"\n#include "return_observation_policy.h"\n'
           '#include <cstring>\n' + fixture)
fixture = fixture.replace('class ExperimentRunner {public:', 'class ExperimentRunner {public:\n'
    ' uint32_t energy_control_autonomous_previous_detector_test_ms_=1198;\n'
    ' float energy_control_autonomous_previous_detector_rate_dps_=-15;\n'
    ' void processCrossingSample(uint32_t,uint32_t,float,float,float);\n'
    ' void updateEnergyControlAutonomousPeakTracker(uint32_t,float,float){}\n'
    ' bool observeInwardStallRescue(uint32_t,bool,bool,float,float){walking_return_event_=false;return false;}\n')
# Use the real residual-current producer as well. At a cutoff the fresh stop
# clock may be newer than the motion loop timestamp; unsigned subtraction of
# that older timestamp must not erase the signed current via exp(-UINT_MAX).
producer_begin = runner.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(')
residual_begin = runner.index('  const uint32_t now_ms =', producer_begin)
residual_begin = runner.rfind('#if defined(WALKING_TRANSFER_V18)', producer_begin, residual_begin)
assert residual_begin >= producer_begin
residual_end = runner.index('\n\n  // Capture the latest', residual_begin)
residual = runner[residual_begin:residual_end].replace('event.', 'e.').replace('t_test_ms', 't')
fixture = fixture.replace('e.zero_cross_rate_dps=rate;e.detector_angle_after_deg=after;',
    'e.zero_cross_rate_dps=rate;e.detector_angle_after_deg=after;\n' + residual)
fixture = fixture.replace('unsigned stop_requests=0;bool stop(){++stop_requests;',
    'unsigned stop_requests=0;uint32_t stop_delay_ms=0;bool stop(){++stop_requests;'
    'mock_ms+=stop_delay_ms;mock_us+=stop_delay_ms*1000;')

motion_begin = runner.index('void ExperimentRunner::updateEnergyControlAutonomousMotion(')
route_begin = runner.index('#if defined(WALKING_TRANSFER_V18)',
                           runner.index('  const float before_deg =', motion_begin))
route_end = runner.index('\n#if FOOT_EARLY_TRIAL', route_begin)
route = runner[route_begin:route_end]
assert 'cutReturnAidAtZero' in route and 'if(cut_return_at_zero)interpolated_rate_dps=rate_dps;' in route
fixture += '\nvoid ExperimentRunner::processCrossingSample(uint32_t now_ms,uint32_t sample_us,\n'
fixture += '    float before_deg,float detector_relative_angle_deg,float rate_dps){\n'
fixture += '  const uint32_t t_test_ms=now_ms-run_start_ms_;\n'
fixture += '  const float peak_relative_angle_deg=detector_relative_angle_deg;\n'
fixture += route + '\n}\n'
fixture += r'''
static void clock_at(ExperimentRunner& r,unsigned t){
 mock_ms=r.run_start_ms_+t;mock_us=2000000u+(t-1200u)*1000u;
 r.energy_control_autonomous_last_motion_sample_us_=mock_us;
}
static void ready18(ExperimentRunner& r){
 ready(r);assert(r.walking_half_cycle_budget_.sourcePeak(r.energy_control_autonomous_last_peak_ms_));
 r.energy_control_autonomous_pending_peak_=false;
}
static void start_aid(ExperimentRunner& r,bool full=false,int source_side=1){
 ready18(r);r.energy_control_autonomous_last_peak_side_=source_side;
 r.energy_control_autonomous_last_peak_amplitude_deg_=10.6f;
 if(full){r.walking_energy_count_=192;r.walking_energy_audit_[191].completion_reason=77;}
 r.walking_return_event_=true;r.stall_rescue_event_=true;r.walking_return_choice_.request_j=.0001f;
 r.walking_return_observation_.evidence=return_observation_policy::Evidence::RecognitionDropout;
 r.acceptMotionCrossing(true,1200,-source_side*15,source_side*9.5f,source_side*9.4f,1,1200);
 r.walking_return_event_=false;r.stall_rescue_event_=false;
 assert(r.status_.pulse_active && r.walking_energy_completion_pending_);
 const auto& a=r.walking_energy_active_pulse_;
 assert(a.return_event && a.executed && a.rescue && a.assisted_cycle);
 assert(a.return_evidence==2 && a.decision.available_j==0 && near(a.decision.request_j,.0001f));
 assert(a.guarded_ms>15 && a.guarded_ms<=80 && a.envelope_before_ms==80);
 assert(r.walking_half_cycle_budget_.aidUsed() && !r.walking_half_cycle_budget_.mainUsed());
 assert(r.walking_half_cycle_budget_.reservedMs()==a.guarded_ms);
 assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
 assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==600);
 assert(r.energy_control_autonomous_last_accepted_zero_cross_valid_);
 assert(!r.energy_control_autonomous_pending_peak_);
 assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE);
}
static void main_at(ExperimentRunner& r,unsigned t,float rate=-25,float angle=0){
 clock_at(r,t);r.acceptMotionCrossing(false,t,rate,.1f,angle,1,float(t));
}
int main(){
 // A completed aid restores WAIT_ZERO and preserves accepted true-ZERO history.
 // All elapsed time, including a late scheduler tick, debits the shared80ms.
 for(bool full:{false,true})for(unsigned late:{0u,1u}){
  ExperimentRunner r;start_aid(r,full);const auto aid=r.walking_energy_active_pulse_;
  const unsigned elapsed=aid.guarded_ms+late;clock_at(r,aid.output_start_ms+elapsed);
  r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200); // stale loop argument
  assert(!r.status_.pulse_active && !r.walking_energy_completion_pending_);
  assert(r.walking_energy_active_pulse_.completion_reason==1);
  assert(r.walking_energy_active_pulse_.completion.actual_elapsed_ms==elapsed);
  assert(r.walking_half_cycle_budget_.spentMs()==elapsed);
  assert(r.walking_half_cycle_budget_.remaining()==80-elapsed);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
  assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==600);
  assert(!r.energy_control_autonomous_pending_peak_);
  const auto stops=r.roller_obj.stop_requests;
  r.updateEnergyControlAutonomousPulse(mock_ms);assert(r.roller_obj.stop_requests==stops);
  const auto attempts=r.roller_obj.writes;r.acceptOptionalEarlyMotion(aid.output_start_ms+elapsed+5,-25,.1f);
  assert(r.roller_obj.writes==attempts); // No relocated main after aid.
  main_at(r,1400);assert(r.status_.pulse_active && r.last_finished_event.work_control_branch==23);
  const auto main=r.walking_energy_active_pulse_;
  assert(!main.return_event && main.assisted_cycle && main.envelope_before_ms==80-elapsed);
  assert(main.guarded_ms>0 && main.guarded_ms<=80-elapsed && main.start_i0_ma>0);
  assert(r.walking_half_cycle_budget_.mainUsed() && r.walking_return_pending_);
  assert(r.energy_control_autonomous_pending_peak_ && r.energy_control_autonomous_pending_next_side_==-1);
  clock_at(r,main.output_start_ms+main.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_half_cycle_budget_.spentMs()==elapsed+main.guarded_ms);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);
  const auto writes=r.roller_obj.writes;main_at(r,1500);assert(r.roller_obj.writes==writes);
  if(full){assert(r.walking_energy_count_==192 && r.walking_energy_active_audit_==-1);
   assert(r.walking_energy_audit_[191].completion_reason==77 && r.walking_energy_total_==2);}
 }
 // An aid never alters the ordinary guard/tilt or pending-arrival ownership.
 {ExperimentRunner r;ready18(r);r.energy_control_autonomous_last_peak_side_=1;
  r.wheel_cycle_guard_.limit(1,-600,true,1100,80);r.simulated_rpm=-600;
  unsigned char tilt_before[sizeof(r.tilt_priority_)],guard_before[sizeof(r.wheel_cycle_guard_)];
  std::memcpy(tilt_before,&r.tilt_priority_,sizeof(r.tilt_priority_));
  std::memcpy(guard_before,&r.wheel_cycle_guard_,sizeof(r.wheel_cycle_guard_));
  r.energy_control_autonomous_pending_zero_ms_=100;r.energy_control_autonomous_pending_model_energy_j_=.0123f;
  r.walking_arrival_pending_bias_j_=.000012f;
  r.walking_return_event_=true;r.stall_rescue_event_=true;r.walking_return_choice_.request_j=.0001f;
  r.acceptMotionCrossing(true,1200,-15,9.5f,9.4f,1,1200);
  assert(r.status_.pulse_active && !r.energy_control_autonomous_pending_peak_);
  assert(std::memcmp(tilt_before,&r.tilt_priority_,sizeof(r.tilt_priority_))==0);
  assert(std::memcmp(guard_before,&r.wheel_cycle_guard_,sizeof(r.wheel_cycle_guard_))==0);
  assert(r.energy_control_autonomous_pending_zero_ms_==100 && near(r.energy_control_autonomous_pending_model_energy_j_,.0123f));
  assert(near(r.walking_arrival_pending_bias_j_,.000012f));}
 // Real fresh crossing routing stops an active aid (reason5), integrates its
 // actual duration, and sends the normal decision using that same rate sample.
 for(bool full:{false,true})for(int sign:{-1,1}){ExperimentRunner r;start_aid(r,full,sign);
  const auto start=r.walking_energy_active_pulse_.output_start_ms;
  const unsigned cut=start+17;clock_at(r,cut);
  r.energy_control_autonomous_previous_detector_test_ms_=cut-2;
  r.energy_control_autonomous_previous_detector_rate_dps_=-sign*60;
  r.processCrossingSample(mock_ms,mock_us,sign*.03f,-sign*.02f,-sign*25);
  assert(r.status_.pulse_active && r.walking_energy_active_pulse_.start_rate_dps==25);
  assert(!r.walking_energy_active_pulse_.return_event && r.walking_energy_active_pulse_.guarded_ms<=63);
  assert(r.walking_half_cycle_budget_.spentMs()==17 && r.walking_half_cycle_budget_.mainUsed());
  assert(r.walking_return_pending_ && r.roller_obj.stop_requests==1);
  if(!full){const auto& aid=r.walking_energy_audit_[0];
   assert(aid.completion_reason==5 && aid.completion.actual_elapsed_ms==17 && !aid.completion.duration_completed);
   assert(aid.envelope_after_ms==63 && r.walking_energy_audit_[1].envelope_before_ms==63);}
 }
 // The loop timestamp can precede the fresh cutoff clock by one millisecond.
 // The selector must see the same signed residual as the actual starter.
 {ExperimentRunner r;start_aid(r);clock_at(r,1217);
  const auto loop_now=mock_ms;++mock_ms;mock_us+=1000;
  r.energy_control_autonomous_previous_detector_test_ms_=1215;
  r.processCrossingSample(loop_now,mock_us,.03f,-.02f,-25);
  const auto& a=r.walking_energy_active_pulse_;
  assert(r.status_.pulse_active && !a.return_event && a.start_i0_ma>0);
  assert(std::fabs(a.decision.coast_residual_q_mas-a.start_i0_ma*.070f)<1e-5f);
 }
 // Wrong direction, missing crossing, old/future sample and old previous
 // angle may not cut an aid or open an overlapping main opportunity.
 for(unsigned bad=0;bad<12;++bad){ExperimentRunner r;start_aid(r);clock_at(r,1217);
  r.energy_control_autonomous_previous_detector_test_ms_=1215;
  auto sample=mock_us;float before=.03f,after=-.02f,rate=-25;
  if(bad==0)sample=0;
  if(bad==1)sample-=10001;
  if(bad==2)++sample;
  if(bad==3)rate=25;
  if(bad==4)before=-.03f;
  if(bad==5)after=.02f;
  if(bad==6)r.energy_control_autonomous_previous_detector_test_ms_=1206;
  if(bad==7)before=NAN;
  if(bad==8)after=NAN;
  if(bad==9)rate=NAN;
  if(bad==10)r.energy_control_autonomous_last_peak_side_=0;
  if(bad==11)r.status_.emergency_stop=true;
  assert(!r.cutReturnAidAtZero(mock_ms,sample,before,after,rate));
  assert(r.status_.pulse_active && r.walking_half_cycle_budget_.spentMs()==0);
  assert(!r.walking_half_cycle_budget_.mainUsed() && r.roller_obj.stop_requests==0);}
 // A geometrically passed ZERO with unusable proof never creates a late
 // main. After the aid timer it advances to peak observation rather than
 // waiting indefinitely for the passage that detector history has consumed.
 for(unsigned bad=0;bad<5;++bad){ExperimentRunner r;start_aid(r);clock_at(r,1217);
  r.energy_control_autonomous_previous_detector_test_ms_=1215;auto sample=mock_us;
  if(bad==0)sample=0;
  if(bad==1)sample-=10001;
  if(bad==2)++sample;
  if(bad==3)r.energy_control_autonomous_previous_detector_test_ms_=1206;
  if(bad==4)r.energy_control_autonomous_previous_detector_test_ms_=1218;
  assert(!r.cutReturnAidAtZero(mock_ms,sample,.03f,-.02f,-25));
  assert(r.walking_aid_zero_seen_ && r.status_.pulse_active);
  const auto a=r.walking_energy_active_pulse_;clock_at(r,a.output_start_ms+a.guarded_ms+1);
  r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(!r.status_.pulse_active && !r.walking_half_cycle_budget_.mainUsed());
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);
  const auto writes=r.roller_obj.writes;main_at(r,1400);assert(r.roller_obj.writes==writes);
 }
 // A delayed stop or dispatch cannot launch a main from a stale ZERO sample.
 {ExperimentRunner r;start_aid(r);clock_at(r,1217);r.roller_obj.stop_delay_ms=6;
  r.energy_control_autonomous_previous_detector_test_ms_=1215;
  r.processCrossingSample(mock_ms,mock_us,.03f,-.02f,-25);
  assert(!r.status_.pulse_active && !r.walking_half_cycle_budget_.mainUsed());
  assert(r.walking_half_cycle_budget_.spentMs()==17 && r.roller_obj.stop_requests==1);}
 // Exhaustion still records the ordinary energy shortage and consumes main
 // as a zero-output observation without claiming an energy-surplus coast.
 {ExperimentRunner r;start_aid(r);const auto start=r.walking_energy_active_pulse_.output_start_ms;
  clock_at(r,start+81);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_half_cycle_budget_.remaining()==0);main_at(r,1400);
  assert(!r.status_.pulse_active && r.walking_half_cycle_budget_.mainUsed());
  const auto& a=r.walking_energy_audit_[r.walking_energy_count_-1];
  assert(a.decision.request_j>0 && a.decision.unmet_j>0 && a.guarded_ms==0);
  assert(r.last_finished_event.work_brake_cap_ms==0 && r.last_finished_event.unapplied_energy_request_j>0);}
 // Uncertain aid transport fails closed; it cannot restore ZERO and send an
 // extra pulse. A failed command reserves no confirmed software allowance.
 {ExperimentRunner r;ready18(r);r.energy_control_autonomous_last_peak_side_=1;
  r.roller_obj.write_ok=false;r.walking_return_event_=true;r.stall_rescue_event_=true;
  r.walking_return_choice_.request_j=.0001f;r.acceptMotionCrossing(true,1200,-15,9.5f,9.4f,1,1200);
  assert(r.status_.emergency_stop && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(!r.walking_half_cycle_budget_.aidUsed() && r.walking_half_cycle_budget_.remaining()==80);
  const auto writes=r.roller_obj.writes;main_at(r,1400);r.acceptOptionalEarlyMotion(1401,-25,.1f);
  assert(r.roller_obj.writes==writes);}
 // No-aid ordinary/optional main remains fully energy selected, with fresh
 // post-transport clock, one completion and no assisted-feedback flag.
 {ExperimentRunner r;ready18(r);r.roller_obj.write_delay_ms=7;
  r.acceptOptionalEarlyMotion(1200,55,-.5f);assert(r.status_.pulse_active);
  const auto a=r.walking_energy_active_pulse_;
  assert(a.output_start_ms==1207 && a.guarded_ms>15 && a.guarded_ms<=80);
  assert(!a.assisted_cycle && !r.walking_return_pending_ && !r.walking_half_cycle_budget_.aidUsed());
  r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);assert(r.status_.pulse_active);
  clock_at(r,a.output_start_ms+a.guarded_ms);r.updateEnergyControlAutonomousPulse(mock_ms);
  assert(r.walking_energy_active_pulse_.completion.actual_elapsed_ms==a.guarded_ms);
  assert(r.walking_half_cycle_budget_.spentMs()==a.guarded_ms);}
 for(unsigned bad=0;bad<2;++bad){ExperimentRunner r;ready18(r);
  r.change_before_dispatch=bad?3:2;main_at(r,1200,25);
  assert(!r.status_.pulse_active && r.roller_obj.writes==0 && !r.walking_half_cycle_budget_.mainUsed());}
 std::cout<<"v18 real runner: aid/main80ms, timer/zero cutoff, same fresh sample, signed residual, archive-full, no aid learning, stale/uncertain rejection PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='energy_transfer_runner_v18_') as folder:
    temp = Path(folder)
    (temp / 'fixture.cpp').write_text(fixture, encoding='utf8')
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1', '-DFOOT_SUPPORT_LOCAL=1', '-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1', '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1',
                    '-DSWAY_BALANCE_TRIAL=1', '-DWALKING_RETURN_ENERGY=1', '-DWALKING_TRANSFER_V18=1',
                    '-I' + str(ROOT / 'src'), str(temp / 'fixture.cpp'), '-o', str(temp / 'fixture.exe')], check=True)
    subprocess.run([str(temp / 'fixture.exe')], check=True)
