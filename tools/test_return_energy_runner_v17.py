"""Run v17's real selection/dispatch/start/timer/stop/completion with host I/O.

Reuses the established fixture, not the old version's expected decisions.
This exercises software contracts; it cannot establish physical walking.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'tools/test_energy_main_lifecycle_v16.py').read_text(encoding='utf8')
prefix = source.split("\nfixture+=r'''\n", 1)[0]
exec(compile(prefix, str(ROOT / 'tools/test_energy_main_lifecycle_v16.py'), 'exec'), globals())
fixture = '#include "walking_return_energy.h"\n' + fixture
fixture += r'''
int main(){
 // An energy-feasible main input still completes with the fresh timer clock.
 {ExperimentRunner r;ready(r);r.roller_obj.write_delay_ms=7;
  r.walking_arrival_feedback_.observe(1,1000,.0009f,.0007f,0,.0007f,true);
  r.walking_arrival_feedback_.observe(1,1100,.0009f,.0007f,0,.0007f,true);
  r.acceptOptionalEarlyMotion(1200,55,-.5f);
  assert(r.last_finished_event.work_control_branch==22 && r.status_.pulse_active);
  auto& a=r.walking_energy_active_pulse_;
  assert(a.arrival_bias_j>0 && a.executed && !a.return_event);
  auto start=r.active_pulse_start_ms_;auto width=a.guarded_ms;
  mock_ms=start+width-1;r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
  assert(r.status_.pulse_active);
  mock_ms=start+width;r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
  assert(!r.status_.pulse_active && a.completion_reason==1 && a.completion.actual_elapsed_ms==width);
  assert(r.energy_control_autonomous_pending_q_command_mA_s_==a.completion.q_mas);
  assert(r.walking_arrival_pending_bias_j_==a.arrival_bias_j);
 }
 // Nominal U at +9.4deg would coast. Return-work demand must reach the real
 // dispatch and actuator width, using the main envelope rather than rescue15.
 {ExperimentRunner r;ready(r);r.energy_control_autonomous_last_peak_side_=1;
  r.walking_return_event_=true;r.walking_return_choice_.request_j=.0001f;r.stall_rescue_event_=true;
  Event e;e.zero_cross_time_ms=1200;e.zero_cross_abs_rate_dps=15;e.zero_cross_rate_dps=-15;
  e.detector_angle_after_deg=9.4f;e.physical_next_peak_side=-1;
  e.previous_peak_time_ms=900;e.previous_peak_amplitude_deg=10.6f;
  assert(r.dispatchTiltPriority(e,1200,true,600));
  assert(e.output_executed && e.work_control_branch==22 && e.work_brake_cap_ms==80);
  auto& a=r.walking_energy_active_pulse_;
  assert(a.return_event && near(a.return_work_j,.0001f) && a.guarded_ms>15);
  assert(a.decision.request_j>=.00009999f && r.walking_return_pending_);
  mock_ms=r.active_pulse_start_ms_+a.guarded_ms;r.updateEnergyControlAutonomousPulse(r.run_start_ms_+1200);
  assert(a.completion_reason==1 && a.completion.actual_elapsed_ms==a.guarded_ms);
 }
 // Missing health must not turn the new budget into an unconditional pulse.
 {ExperimentRunner r;ready(r);r.imu_obj.good=false;r.walking_return_event_=true;
  r.walking_return_choice_.request_j=.0001f;r.stall_rescue_event_=true;
  Event e;e.zero_cross_time_ms=1200;
  assert(r.dispatchTiltPriority(e,1200,true,600));
  assert(!e.output_executed && !r.status_.pulse_active && r.roller_obj.writes==0);
 }
 std::cout<<"v17 real runner: bias, return energy beyond centre gate, rescue80, health rejection, fresh-clock completion PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='return_energy_runner_v17_') as folder:
    temp = Path(folder)
    (temp / 'fixture.cpp').write_text(fixture, encoding='utf8')
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DFOOT_SUPPORT_ASSIST=1', '-DFOOT_SUPPORT_LOCAL=1', '-DFOOT_SUPPORT_TRANSFER=1',
                    '-DFOOT_SUPPORT_ENERGY=1', '-DEARLY_ENERGY_MAIN=1', '-DPULSE_LIFECYCLE_FIX=1',
                    '-DSWAY_BALANCE_TRIAL=1', '-DWALKING_RETURN_ENERGY=1',
                    '-I' + str(ROOT / 'src'), str(temp / 'fixture.cpp'), '-o', str(temp / 'fixture.exe')], check=True)
    subprocess.run([str(temp / 'fixture.exe')], check=True)
