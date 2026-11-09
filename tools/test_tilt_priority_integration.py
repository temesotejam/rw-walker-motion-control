"""Execute the real tilt-priority runner with hardware doubles.

This checks dispatch and telemetry, not ESP timing or physical response.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
runner=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
fixture=(root/'tools/tilt_priority_integration_fixture.cpp').read_text(encoding='utf-8')
start='bool ExperimentRunner::tryTiltPriority('
end='\nbool ExperimentRunner::tryWeakSideAssist('
assert runner.count(start)==1 and runner.count(end)==1
method=start+runner.split(start,1)[1].split(end,1)[0]
at_zero=runner.split('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(',1)[1]
assert at_zero.index('dispatchTiltPriority(event,t_test_ms,previous_accepted_valid,previous_accepted_ms)') < at_zero.index('work_choice=chooseWheelWork(')
pulse=runner.split('bool ExperimentRunner::beginEnergyControlAutonomousPulse(',1)[1].split('bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(',1)[0]
assert pulse.index('roller_->setCurrentMa(command_current_mA)') < pulse.index('if(optional_early_event_)optional_early_write_uncertain_=true')
assert 'requestEmergencyStop("optional_early_write_unresolved")' in runner
branch=runner.split('bool ExperimentRunner::dispatchTiltPriority(',1)[1].split('\n__attribute__((noinline)) void ExperimentRunner::finishSolverAudit(',1)[0]
assert 'tryTiltPriority(event,t_test_ms)' in branch
assert 'if(!event.output_executed)' in branch and 'resetEnergyControlAutonomousPeakTracker(true)' in branch
assert branch.count('restoreUnsentStallRescue(event,previous_accepted_valid,previous_accepted_ms)')==2
restore=runner.split('void ExperimentRunner::restoreUnsentStallRescue(',1)[1].split('\n__attribute__((noinline)) bool ExperimentRunner::dispatchTiltPriority(',1)[0]
assert 'energy_control_autonomous_zero_cross_consumed_for_peak_=false' in restore
assert 'energy_control_autonomous_last_accepted_zero_cross_ms_=previous_accepted_ms' in restore
assert 'EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS' in restore
assert 'stall_rescue_attempted_peak_ms_!=energy_control_autonomous_last_peak_ms_' in runner
audit=at_zero.split('auto finish_audit =',1)[1].split('solver_audit::ScopeExit',1)[0]
assert audit.index('const uint32_t end_us=micros()') < audit.index('finishSolverAudit(audit,event,end_us)')
assert 'audit.end_us = end_us' in runner and 'logger_->addSolverAuditEvent(audit)' in runner
assert 'energy_control_autonomous_pending_probe_=true' in method
assert 'energy_control_autonomous_pending_model_energy_j_=NAN' in method
assert 'event.work_control_branch=15' in method
assert 'tilt_priority_.crossing(in.next_side,t_test_ms)' in method
assert 'in.crossing_rate_dps=event.zero_cross_abs_rate_dps' in method
assert 'foot_span_control_.reset()' not in method
assert 'long_foot_balance_.propose(' in method
assert 'long_foot_balance_.confirm(' in method
assert 'tilt_priority_.timingCap(' in method
assert 'event.zero_cross_abs_rate_dps = fabsf(rate_dps)' in at_zero
assert 'imu_->acquisitionHealthy() && !imu_->stale(millis())' in method
assert 'tilt_priority_.peak(physical_side,peak_ms,amplitude_deg)' in runner
assert 'std::fabs(event.pre_input_wheel_speed_rpm)' not in method
assert fixture.count('// METHOD_SOURCE')==1
fixture=fixture.replace('// METHOD_SOURCE',method)
assert fixture.count('// DISPATCH_SOURCE')==1
restore_method='void ExperimentRunner::restoreUnsentStallRescue('+runner.split('void ExperimentRunner::restoreUnsentStallRescue(',1)[1].split('\n__attribute__((noinline)) bool ExperimentRunner::dispatchTiltPriority(',1)[0]
dispatch_method='bool ExperimentRunner::dispatchTiltPriority('+branch
fixture=fixture.replace('// DISPATCH_SOURCE',restore_method+'\n'+dispatch_method)
assert fixture.count('// EARLY_WRAPPER_SOURCE')==1
wrapper='void ExperimentRunner::acceptOptionalEarlyMotion('+runner.split('void ExperimentRunner::acceptOptionalEarlyMotion(',1)[1].split('\nvoid ExperimentRunner::updateEnergyControlAutonomousMotion(',1)[0]
fixture=fixture.replace('// EARLY_WRAPPER_SOURCE',wrapper)
with tempfile.TemporaryDirectory(prefix='tilt_priority_runner_') as td:
 source=Path(td)/'fixture.cpp';exe=Path(td)/'fixture'
 source.write_text(fixture,encoding='utf-8')
 subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                 '-I'+str(root/'src'),str(source),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
