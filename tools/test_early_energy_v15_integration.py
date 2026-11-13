"""Run actual v15 energy/optional/dispatch/foot code and the v14 regression.

Uses the established host driver doubles plus the actual normal-ZERO entry
guard. Crossing event construction and transport are doubles. This checks
command/state contracts, not ESP deadlines or a changed physical trajectory.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
v14=(ROOT/'tools/test_walking_energy_integration.py').read_text(encoding='utf8')
marker="\nfixture += r'''\n"
prefix,tail=v14.split(marker,1)
exec(compile(prefix,str(ROOT/'tools/test_walking_energy_integration.py'),'exec'),globals())
fixture='#include "walking_energy_policy_v15.h"\n#include "autonomous_input_current.h"\n'+fixture
fixture+=tail.split('int main(){',1)[0].replace('walking_energy_policy::','walking_energy_policy_v15::')

# Retain the real outer entry guard, so a successful early main input cannot
# be sent twice at ZERO merely because this fixture builds crossing events.
guard_start=runner.index('  if (!energy_control_autonomous_mode_',runner.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross('))
guard_end=runner.index('  const uint32_t accepted_cross_ms',guard_start)
normal_guard=runner[guard_start:guard_end]
class_marker='class ExperimentRunner {public:'
fixture=fixture.replace(class_marker,'enum class EnergyControlAutonomousPhase {ENERGY_CONTROL,HOLD,STOP};\n'+class_marker+
    '\n bool energy_control_autonomous_mode_=true;EnergyControlAutonomousPhase energy_control_autonomous_phase_=EnergyControlAutonomousPhase::ENERGY_CONTROL;\n'
    ' uint32_t energy_control_autonomous_last_motion_sample_us_=2000000;\n'
    ' uint32_t active_pulse_start_ms_=0,active_pulse_start_test_ms_=0;bool energy_control_autonomous_pulse_authorized_=false;\n'
    ' float predictRiseTauS(float)const{return .0316f;}void updatePulseModelPrediction(){}\n'
    ' template<typename... T>void startTimingProbe(T...){}\n'
    ' void stopMotor(){status_.pulse_active=false;status_.motor_cmd_mA=0;}\n')
cross_marker='void acceptMotionCrossing(bool,uint32_t t,float rate,float,float after,float,float) {'
assert fixture.count(cross_marker)==1
fixture=fixture.replace(cross_marker,cross_marker+'\n'+normal_guard)
logger_marker='struct Logger {unsigned events=0;'
assert fixture.count(logger_marker)==1
fixture=fixture.replace(logger_marker,logger_marker+'PsramLogger::EnergyControlAutonomousZeroCrossEvent last;')
logger_method='void addEnergyControlAutonomousZeroCrossEvent(const PsramLogger::EnergyControlAutonomousZeroCrossEvent&){++events;}'
assert fixture.count(logger_method)==1
fixture=fixture.replace(logger_method,'void addEnergyControlAutonomousZeroCrossEvent(const PsramLogger::EnergyControlAutonomousZeroCrossEvent& e){++events;last=e;}')

# Execute the actual pulse starter too. The only transport double advances
# the host clock during a write, making a backdated duration observable.
begin_marker='bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t d,uint16_t w,int16_t m)'
begin=fixture.index(begin_marker);end=fixture.index('{',begin)+1;depth=1
while depth:
    depth+=(fixture[end]=='{')-(fixture[end]=='}');end+=1
fixture=fixture[:begin]+'bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t,uint16_t,int16_t);'+fixture[end:]
fixture+='\n'+method('beginEnergyControlAutonomousPulse','bool')+'\n'
fixture=fixture.replace('namespace Config {','namespace Config {\n'
    'constexpr uint16_t ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS=100;\n'
    'constexpr int16_t WALKING_TARGET8_MAX_CURRENT_MA=1200,ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN=1,MOTOR_DRIVER_POLARITY=1;\n'
    'constexpr float MODEL_VBAT_REFERENCE_V=7.5f;\n')
fixture=fixture.replace('struct Status {','struct Status {int16_t current_mA_setting=0,motor_cmd_mA=0;uint16_t pulse_width_ms_setting=0;int8_t pulse_direction=0;')
roller_begin=fixture.index('struct Roller {');roller_end=fixture.index('};',roller_begin)+2
fixture=fixture[:roller_begin]+'''struct Roller {
 bool good=true,write_ok=true;Telemetry telemetry;unsigned writes=0;uint32_t write_delay_ms=0;int16_t last_command=0;
 bool ok()const{return good;}Telemetry telemetrySnapshot()const{return telemetry;}
 bool setCurrentMa(int16_t command){++writes;last_command=command;mock_ms+=write_delay_ms;mock_us+=write_delay_ms*1000;return write_ok;}
};'''+fixture[roller_end:]
fixture+=r'''
static void rejected_labels(const Event& e){
 assert(!e.output_executed && !e.valid && e.work_control_branch==20);
 assert(e.command_current_mA==0 && e.pulse_width_ms==0 && e.solver_selected_integer_width_ms==0);
 assert(e.pulse_start_ms==0 && e.pulse_end_ms==0 && e.q_command_direction==0);
 assert(e.q_command_mA_s==0 && e.q_effective_pred_mA_s==0 && e.predicted_input_work_j==0);
 assert(near(e.predicted_energy_j,e.passive_energy_j));
 assert(near(e.unapplied_energy_request_j,e.commanded_energy_deficit_j));
}
static void ordinary(ExperimentRunner& r,uint32_t t=1240,float rate=55){
 mock_ms=r.run_start_ms_+t;r.acceptMotionCrossing(false,t,rate,0,0,1,float(t));
}
int main(){
 namespace e=walking_energy_policy_v15;
 // The same normal80ms envelope moves earlier; integer selection covers J
 // rather than selecting a closer, slightly under-budget millisecond.
 {ExperimentRunner r;ready(r);r.tilt_priority_.ordinaryResult(1,1100,1,true);
  r.acceptOptionalEarlyMotion(1200,55,-.5f);
  assert(r.calls==1 && r.width>15 && r.width<=80 && r.ma==1200);
  const auto& event=r.last_finished_event;const auto& a=r.walking_energy_audit_[0];
  assert(event.work_control_branch==20 && event.valid && event.output_executed);
  assert(a.early_budget_checked && a.early_budget_admitted && a.executed);
  assert(e::earlyAdmissible(a.decision) && a.decision.predicted_work_j+1e-9f>=a.decision.request_j);
  energy_fields(event,a.decision);assert(event.work_brake_cap_ms==80);
  assert(r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  // Simulate pulse completion, then exercise the real ZERO entry guard.
  r.status_.pulse_active=false;ordinary(r);assert(r.calls==1);
  r.acceptOptionalEarlyMotion(1246,55,-.3f);assert(r.calls==1);}

 // Infeasible early main input is not transmitted. All command-Q/work labels
 // say no actuation while the separate audit retains the rejected candidate.
 {ExperimentRunner r;ready(r,2);const auto before=r.wheel_cycle_guard_;
  r.acceptOptionalEarlyMotion(1200,25,-.4f);
  assert(r.calls==0 && r.log_obj.events==1 && r.walking_energy_count_==1);
  const auto& a=r.walking_energy_audit_[0];
  assert(a.early_budget_checked && !a.early_budget_admitted && !a.executed);
  assert(a.requested_ms==80 && a.decision.unmet_j>0);rejected_labels(r.log_obj.last);
  assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_last_accepted_zero_cross_ms_==600);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);
  auto old=before,after=r.wheel_cycle_guard_;assert(old.limit(1,20,true,1240,80)==after.limit(1,20,true,1240,80));
  r.acceptOptionalEarlyMotion(1206,25,-.3f);assert(r.calls==0); // one attempt per peak
  ordinary(r,1240,25);assert(r.calls==1 && r.last_finished_event.work_control_branch==20);
  assert(r.last_finished_event.output_executed && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  r.status_.pulse_active=false;ordinary(r,1246,25);assert(r.calls==1);}

 // Guard-only reduction is recomputed and cannot turn an under-budget early
 // input into an accepted main input. It leaves ordinary ZERO available.
 {ExperimentRunner r;ready(r);r.simulated_rpm=600;
  r.wheel_cycle_guard_.limit(-1,600,true,1000,80);const auto before=r.wheel_cycle_guard_;
  r.acceptOptionalEarlyMotion(1200,55,-.5f);assert(r.calls==0);
  const auto& a=r.walking_energy_audit_[0];
  assert(a.early_budget_checked && !a.early_budget_admitted);
  assert(a.guarded_ms<a.requested_ms && a.decision.unmet_j>0);
  auto in=model(r,r.log_obj.last);in.body_angle_deg=-.5f;
  auto recomputed=e::evaluateWidth(r.lateral_energy_geometry_,r.walking_energy_current_,in,a.guarded_ms);
  assert(recomputed.valid && near(recomputed.predicted_work_j,a.decision.predicted_work_j));
  rejected_labels(r.log_obj.last);assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  auto old=before,after=r.wheel_cycle_guard_;assert(old.limit(1,600,true,1240,80)==after.limit(1,600,true,1240,80));
  ordinary(r);assert(r.calls==1 && r.last_finished_event.output_executed);}

 // The retained observed time-to-peak cap can defer a full main input too.
 {ExperimentRunner r;ready(r);tilt_priority::Choice c;c.selected=true;c.direction=1;
  c.current_ma=1200;c.width_ms=10;c.target_deg=8;
  r.tilt_priority_.crossing(1,1000);r.tilt_priority_.confirm(1,1000,c,true,true,false);
  r.tilt_priority_.peak(1,1060,6);
  assert(r.tilt_priority_.timingCap(1,1200,80)==30);
  r.acceptOptionalEarlyMotion(1200,55,-.5f);assert(r.calls==0);
  assert(r.log_obj.last.work_brake_cap_ms==30 && r.walking_energy_audit_[0].decision.saturated);
  rejected_labels(r.log_obj.last);assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);}

 // Surplus early energy never manufactures a pulse or consumes normal ZERO.
 {ExperimentRunner r;ready(r,11,11);r.acceptOptionalEarlyMotion(1200,100,-.5f);
  assert(r.calls==0 && r.walking_energy_audit_[0].decision.coast);
  assert(r.walking_energy_audit_[0].early_budget_checked && !r.walking_energy_audit_[0].early_budget_admitted);
  rejected_labels(r.log_obj.last);assert(!r.energy_control_autonomous_zero_cross_consumed_for_peak_);}

 // The separate low-speed rescue keeps15ms and may report an unmet budget.
 {ExperimentRunner r;ready(r);r.stall_rescue_event_=true;Event event=::event(r,5.5f);
  event.detector_angle_after_deg=-.4f;assert(r.dispatchTiltPriority(event,1200,true,600));
  assert(r.calls==1 && event.output_executed && event.work_brake_cap_ms==15 && r.width<=15);
  const auto& a=r.walking_energy_audit_[0];assert(a.rescue && !a.early_budget_checked && a.decision.unmet_j>0);}

 // A genuine foot timing relocation also carries exactly the covered main
 // amount. An infeasible preview leaves ZERO armed without a foot command.
 {ExperimentRunner r;ready(r);candidate(r,5.f,30.f);
  assert(r.calls==1 && r.width>15 && r.width<=80 && r.foot_support_audit_count_==1);
  const auto& a=r.walking_energy_audit_[0];const auto& foot=r.foot_support_audit_[0];
  assert(a.early_budget_admitted && e::earlyAdmissible(a.decision));
  assert(foot.executed && !foot.coast_assist && foot.raw_boost_ms==0);
  assert(r.width==foot.requested_width && r.width==a.guarded_ms);}
 {ExperimentRunner r;ready(r);candidate(r,3.f,25.f);
  assert(r.calls==0 && r.foot_support_audit_count_==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);}
 // All generic early paths enforce fresh IMU samples and the5ms decision
 // deadline before transport, preserving the ordinary opportunity on reject.
 for(unsigned bad=0;bad<6;++bad){ExperimentRunner r;ready(r);
  if(bad==0)r.energy_control_autonomous_last_motion_sample_us_=mock_us-10001;
  if(bad==1)r.energy_control_autonomous_last_motion_sample_us_=mock_us+1;
  if(bad==2)r.energy_control_autonomous_last_motion_sample_us_=0;
  if(bad==3)r.change_before_dispatch=2;
  if(bad==4)r.change_before_dispatch=3;
  if(bad==5)--mock_ms;
  r.acceptOptionalEarlyMotion(1200,55,-.5f);
  assert(r.calls==0 && !r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  assert(r.energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS);}
 // A7ms write consumes no part of the selected post-write command duration.
 {ExperimentRunner r;ready(r);r.roller_obj.write_delay_ms=7;
  r.acceptOptionalEarlyMotion(1200,55,-.5f);assert(r.calls==1);
  const auto& event=r.last_finished_event;const auto& a=r.walking_energy_audit_[0];
  assert(event.pulse_start_ms==1207 && event.pulse_end_ms-event.pulse_start_ms==r.width);
  assert(a.output_start_ms==1207 && r.active_pulse_start_test_ms_==1207);
  assert(r.active_pulse_start_ms_==r.run_start_ms_+1207);
  assert(r.predicted_current_end_ms_==r.active_pulse_start_ms_+r.width);}
 // The real starter marks a failed write as possibly delivered; it cannot
 // reopen ZERO and invite another main input in the same half-cycle.
 {ExperimentRunner r;ready(r);r.roller_obj.write_ok=false;r.roller_obj.write_delay_ms=7;
  r.acceptOptionalEarlyMotion(1200,55,-.5f);
  assert(r.calls==1 && r.status_.emergency_stop && r.energy_control_autonomous_zero_cross_consumed_for_peak_);
  ordinary(r);r.acceptOptionalEarlyMotion(1246,55,-.3f);assert(r.calls==1);}
 std::cout<<"v15 real runner: covered early main>15ms, infeasible/guard defer preserves ZERO, no double input, timing cap, rescue15, foot timing-only PASS\n";
}
'''
fixture=fixture.replace('r.calls','r.roller_obj.writes').replace('r.width','r.status_.pulse_width_ms_setting').replace('r.ma','r.status_.current_mA_setting')

with tempfile.TemporaryDirectory(prefix='early_energy_v15_runner_') as temp:
    p=Path(temp);(p/'fixture.cpp').write_text(fixture,encoding='utf8')
    subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1','-DFOOT_SUPPORT_LOCAL=1','-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1','-DEARLY_ENERGY_MAIN=1','-DSWAY_BALANCE_TRIAL=1',
                    '-I'+str(ROOT/'src'),str(p/'fixture.cpp'),'-o',str(p/'fixture.exe')],check=True)
    subprocess.run([str(p/'fixture.exe')],check=True)

# Compile the same current runner without EARLY_ENERGY_MAIN and execute all
# established v14 contracts; source separation must preserve its old behavior.
subprocess.run(['python',str(ROOT/'tools/test_walking_energy_integration.py')],check=True)
