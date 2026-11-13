"""Bridge real admission/save/start to real timer/stop/completion in v16.

Transport, sensor/event construction and log storage are host doubles. The
actual main-input decision, optional dispatch, audit registration, pulse start,
timer update, stop methods and completion finalizer run as one contract.
This checks software elapsed time and model accounting, not physical RS485
application edges or delivered mechanical energy.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
v15=(ROOT/'tools/test_early_energy_v15_integration.py').read_text(encoding='utf8')
prefix=v15.split("\nfixture+=r'''\n",1)[0]
exec(compile(prefix,str(ROOT/'tools/test_early_energy_v15_integration.py'),'exec'),globals())
fixture='#include "energy_pulse_completion.h"\n'+fixture
for unused in ['static Event event(', 'static walking_energy_policy_v15::Input model(',
               'static void energy_fields(', 'static void candidate(']:
    begin=fixture.index(unused);end=fixture.index('{',begin)+1;depth=1
    while depth:
        depth+=(fixture[end]=='{')-(fixture[end]=='}');end+=1
    fixture=fixture[:begin]+fixture[end:]
fixture=fixture.replace('EnergyControlAutonomousPhase {ENERGY_CONTROL,HOLD,STOP}',
                        'EnergyControlAutonomousPhase {ENERGY_CONTROL,HOLD,STOP,STRONG_START_KICK,WAIT_FIRST_PEAK}')
fixture=fixture.replace('class ExperimentRunner {public:','class ExperimentRunner {public:\n'
    ' bool q_ident_pulse_authorized_=true,energy_control_v0_pulse_authorized_=true,zero_cross_mode_=false;\n'
    ' uint32_t last_pulse_end_ms_=0,next_pulse_start_test_ms_=0;\n'
    ' void updateEnergyControlAutonomousPulse(uint32_t);void stopActivePulse(uint32_t);\n')
fixture=fixture.replace('struct Status {','struct Status {uint16_t input_interval_ms=1000;int16_t roller_actual_current_mA=0;')
fixture=fixture.replace('struct Telemetry {','struct Telemetry {int16_t actual_current_mA=0;')
fixture=fixture.replace(' bool ok()const{return good;}Telemetry telemetrySnapshot()const{return telemetry;}',
    ' unsigned stop_requests=0;bool stop(){++stop_requests;if(last_command!=0){++writes;last_command=0;}return true;}\n'
    ' bool ok()const{return good;}Telemetry telemetrySnapshot()const{return telemetry;}')
stop_stub='void stopMotor(){status_.pulse_active=false;status_.motor_cmd_mA=0;}'
assert fixture.count(stop_stub)==1
fixture=fixture.replace(stop_stub,'void stopMotor();')
for name in ['updateEnergyControlAutonomousPulse','stopActivePulse','stopMotor']:
    fixture+='\n'+method(name,'void')+'\n'

fixture+=r'''
static bool close(float a,float b,float tolerance=2e-6f){return std::fabs(a-b)<=tolerance;}
static energy_pulse_completion::Input completion_input(const ExperimentRunner::WalkingEnergyAudit& a,unsigned elapsed){
 energy_pulse_completion::Input in;in.side=a.side;in.rate_dps=a.start_rate_dps;
 in.initial_current_ma=a.start_i0_ma;in.goal_current_ma=a.start_goal_ma;in.tau_s=a.start_tau_s;
 in.available_j=a.decision.available_j;in.request_j=a.decision.request_j;
 in.planned_ms=a.guarded_ms;in.actual_elapsed_ms=elapsed;return in;
}
static void start_main(ExperimentRunner& r,bool capacity_full=false){
 ready(r);r.roller_obj.write_delay_ms=7;
 if(capacity_full){r.walking_energy_count_=192;r.walking_energy_audit_[191].completion_reason=77;}
 r.acceptOptionalEarlyMotion(1200,55,-.5f);
 assert(r.roller_obj.writes==1 && r.status_.pulse_active);
 assert(r.last_finished_event.work_control_branch==21 && r.last_finished_event.output_executed);
 assert(r.status_.pulse_width_ms_setting>15 && r.status_.pulse_width_ms_setting<=80);
 assert(r.walking_energy_completion_pending_);
 assert(r.walking_energy_active_audit_==(capacity_full?-1:0));
 const auto& a=r.walking_energy_active_pulse_;
 assert(a.executed && a.early_budget_checked && a.early_budget_admitted && !a.completion_reason);
 assert(a.output_start_ms==1207 && a.trigger_ms==1200 && a.guarded_ms==r.status_.pulse_width_ms_setting);
 assert(std::isfinite(a.start_i0_ma) && a.start_goal_ma>0 && a.start_tau_s>0 && a.start_rate_dps==55);
 assert(r.energy_control_autonomous_pending_peak_ && r.energy_control_autonomous_pending_zero_ms_==a.trigger_ms);
 assert(r.energy_control_autonomous_pending_next_side_==a.side);
}
static void verify_completion(ExperimentRunner& r,unsigned elapsed,unsigned reason,bool overflow=false){
 const auto& a=r.walking_energy_active_pulse_;const auto expected=energy_pulse_completion::evaluate(completion_input(a,elapsed));
 assert(!r.status_.pulse_active && !r.walking_energy_completion_pending_ && r.walking_energy_active_audit_==-1);
 assert(a.completion_reason==reason && a.output_end_ms==a.output_start_ms+elapsed);
 assert(a.completion.valid && a.completion.work_valid && a.completion.actual_elapsed_ms==elapsed);
 assert(a.completion.planned_ms==a.guarded_ms && a.completion.duration_completed==(elapsed>=a.guarded_ms));
 assert(close(a.completion.q_mas,expected.q_mas) && near(a.completion.work_j,expected.work_j));
 assert(near(a.completion.arrival_j,expected.arrival_j) && near(a.completion.unmet_j,expected.unmet_j));
 assert(close(r.predicted_signed_current_end_mA_,a.side*expected.current_end_ma));
 assert(r.predicted_current_end_ms_==r.run_start_ms_+a.output_end_ms);
 assert(close(r.energy_control_autonomous_pending_q_command_mA_s_,expected.q_mas));
 assert(near(r.energy_control_autonomous_pending_model_energy_j_,expected.arrival_j));
 if(overflow){assert(r.walking_energy_count_==192 && r.walking_energy_audit_[191].completion_reason==77);}
 else{const auto& stored=r.walking_energy_audit_[0];
  assert(r.walking_energy_count_==1 && stored.completion_reason==reason);
  assert(stored.output_end_ms==a.output_end_ms && close(stored.completion.q_mas,a.completion.q_mas));}
 assert(r.walking_energy_total_==1);
}
int main(){
 // The very same stale now_ms captured before a7ms enqueue/preemption must
 // flow through real admission/start/save and real timer without expiring.
 for(bool overflow:{false,true}){ExperimentRunner r;start_main(r,overflow);
  const auto start=r.active_pulse_start_ms_;const unsigned width=r.status_.pulse_width_ms_setting;
  const uint32_t old_loop_now=r.run_start_ms_+1200;
  r.updateEnergyControlAutonomousPulse(old_loop_now);
  assert(r.status_.pulse_active && r.roller_obj.stop_requests==0 && r.walking_energy_completion_pending_);
  mock_ms=start+width-1;r.updateEnergyControlAutonomousPulse(old_loop_now);
  assert(r.status_.pulse_active && r.roller_obj.stop_requests==0);
  assert(r.walking_energy_active_pulse_.completion_reason==0);
  mock_ms=start+width;r.updateEnergyControlAutonomousPulse(old_loop_now);
  verify_completion(r,width,1,overflow);
  assert(r.roller_obj.stop_requests==1 && r.roller_obj.last_command==0);
  assert(r.status_.current_mA_setting==0 && r.status_.pulse_width_ms_setting==0);
  assert(!r.q_ident_pulse_authorized_ && !r.energy_control_v0_pulse_authorized_ && !r.energy_control_autonomous_pulse_authorized_);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_PEAK);
  const auto end=r.predicted_current_end_ms_;const auto q=r.energy_control_autonomous_pending_q_command_mA_s_;
  mock_ms+=20;r.updateEnergyControlAutonomousPulse(old_loop_now);
  assert(r.roller_obj.stop_requests==1 && r.predicted_current_end_ms_==end && close(r.energy_control_autonomous_pending_q_command_mA_s_,q));
 }
 // A late scheduling pass integrates all actual software elapsed time,
 // including overrun, rather than silently clipping back to planned width.
 {ExperimentRunner r;start_main(r);const unsigned width=r.status_.pulse_width_ms_setting;
  const float planned_q=r.walking_energy_active_pulse_.decision.q_mas;
  mock_ms=r.active_pulse_start_ms_+width+7;r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
  verify_completion(r,width+7,1);
  assert(r.walking_energy_active_pulse_.completion.q_mas>planned_q);}
 // Real stopMotor finalizes an early interruption. No synthetic completion
 // record is seeded: the same actual accepted main input registered it.
 {ExperimentRunner r;start_main(r);const unsigned width=r.status_.pulse_width_ms_setting;
  const auto planned_end=r.predicted_current_end_ms_;const float planned_q=r.walking_energy_active_pulse_.decision.q_mas;
  assert(width>17);mock_ms=r.active_pulse_start_ms_+17;r.stopMotor();
  verify_completion(r,17,2);
  assert(r.predicted_current_end_ms_<planned_end && r.walking_energy_active_pulse_.completion.q_mas<planned_q);
  const auto end=r.walking_energy_active_pulse_.output_end_ms;
  const auto residual=r.predicted_signed_current_end_mA_;mock_ms+=10;r.stopMotor();
  assert(r.walking_energy_active_pulse_.completion_reason==2 && r.walking_energy_active_pulse_.output_end_ms==end);
  assert(close(r.predicted_signed_current_end_mA_,residual));}
 // A rejected early opportunity cannot register a phantom active completion.
 {ExperimentRunner r;ready(r,2);r.acceptOptionalEarlyMotion(1200,25,-.4f);
  assert(r.roller_obj.writes==0 && !r.status_.pulse_active && !r.walking_energy_completion_pending_);
  assert(r.walking_energy_active_audit_==-1 && r.walking_energy_count_==1);
  r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
  assert(r.walking_energy_audit_[0].completion_reason==0 && r.roller_obj.stop_requests==0);}
 std::cout<<"v16 end-to-end real admission/save/start/update/stop/finalize: stale old clock, full duration, overrun, interruption, pending Q/arrival/residual, idempotence and192-row overflow PASS\n";
}
'''

with tempfile.TemporaryDirectory(prefix='energy_main_lifecycle_v16_') as temp:
    p=Path(temp);(p/'fixture.cpp').write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1','-DEARLY_ENERGY_MAIN=1','-DPULSE_LIFECYCLE_FIX=1',
                    '-DSWAY_BALANCE_TRIAL=1','-I'+str(ROOT/'src'),str(p/'fixture.cpp'),'-o',str(p/'fixture.exe')],check=True)
    subprocess.run([str(p/'fixture.exe')],check=True)
