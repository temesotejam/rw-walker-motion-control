"""Compile the real positive-boost runner method with host hardware doubles.

The helper is currently disconnected from the automatic runner and tested
only in isolation here. This exercises its command decisions and matching,
not motor physics, ESP timing, or automatic production activation. The peak
notification and normal-output confirmation statements come from the runner.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
runner = (root / 'src/experiment_runner.cpp').read_text(encoding='utf-8')
fixture = (root / 'tools/positive_boost_integration_fixture.cpp').read_text(encoding='utf-8')
start = 'bool ExperimentRunner::tryPositiveLeftBoost('
end = '\nvoid ExperimentRunner::applyFootEnergyTarget('
assert runner.count(start) == 1 and runner.count(end) == 1
method = start + runner.split(start, 1)[1].split(end, 1)[0]

normal_start = 'if(event.work_control_branch!=14 && positive_left_input_.now_ms==t_ms)'
normal_end = 'event.pulse_width_ms,event.q_command_direction,event.valid && event.output_executed);'
assert runner.count(normal_start) == 1 and runner.count(normal_end) == 1
normal = normal_start + runner.split(normal_start, 1)[1].split(normal_end, 1)[0] + normal_end
helper_start = 'float ExperimentRunner::energyControlPeakTarget('
helper_end = '\nbool ExperimentRunner::recordEnergyControlAutonomousPeak('
assert runner.count(helper_start) == 1 and runner.count(helper_end) == 1
helper = helper_start + runner.split(helper_start, 1)[1].split(helper_end, 1)[0]
assert 'positive_left_boost_.peak(' in helper
assert 'event.peak_speed_valid && event.peak_speed_age_us<=30000' in helper

# An exploratory pulse owns its half-cycle and must not seed the established
# model with a prediction at an untested current/duration.
assert 'energy_control_autonomous_pending_model_energy_j_=NAN;' in method
assert 'energy_control_autonomous_pending_probe_=true;' in method
assert 'work_positive_pending_=false;work_negative_pending_=false;' in method
# The old high-current helper remains available for isolated testing but is
# intentionally disconnected from automatic walking. Reintroduction requires
# an explicit design and integration review, not a silent call-site change.
assert runner.count('tryPositiveLeftBoost(') == 1
assert 'tryPositiveLeftBoost(event,work_choice,t_test_ms)' not in runner
record = runner.split('bool ExperimentRunner::recordEnergyControlAutonomousPeak(', 1)[1].split(
    '\nvoid ExperimentRunner::', 1)[0]
assert 'energyControlPeakTarget(physical_side)' in record
assert 'notifyPositiveLeftPeak(physical_side,peak_ms,amplitude_deg,event)' in record
assert '!energy_control_autonomous_pending_probe_' in record
assert 'std::isfinite(energy_control_autonomous_pending_model_energy_j_)' in record
begin = runner.split('bool ExperimentRunner::beginEnergyControlAutonomousPulse(', 1)[1].split(
    '\nbool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(', 1)[0]
assert 'Config::WALKING_TARGET8_MAX_PULSE_MS' in begin
assert 'Config::WALKING_TARGET8_MAX_CURRENT_MA' in begin
assert 'autonomous_input_current::valid(requested_current_mA)' in begin
assert 'pulse_width_ms < 1 || pulse_width_ms > permitted_ms' in begin
assert 'status_.pulse_active' in begin and 'status_.emergency_stop' in begin
assert 'roller_->setCurrentMa(command_current_mA)' in begin

for marker, replacement in [('// METHOD_SOURCE', method), ('// NORMAL_SOURCE', normal), ('// HELPER_SOURCE', helper)]:
    assert fixture.count(marker) == 1
    fixture = fixture.replace(marker, replacement)
with tempfile.TemporaryDirectory(prefix='positive_boost_runner_') as td:
    source = Path(td) / 'runner_fixture.cpp'
    exe = Path(td) / 'runner_fixture'
    source.write_text(fixture, encoding='utf-8')
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root / 'src'), str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
