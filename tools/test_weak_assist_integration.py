"""Compile the actual runner assist function with hardware doubles.

This is a host integration check: it verifies command selection, failure and
pending-state bookkeeping but does not validate ESP32 timing or motor response.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
runner = (root / 'src/experiment_runner.cpp').read_text(encoding='utf-8')
fixture = (root / 'tools/weak_assist_integration_fixture.cpp').read_text(encoding='utf-8')
start = 'bool ExperimentRunner::tryWeakSideAssist('
end = '\nbool ExperimentRunner::selectFootPhaseOutput('
assert runner.count(start) == 1 and runner.count(end) == 1
method = start + runner.split(start, 1)[1].split(end, 1)[0]
assert fixture.count('// METHOD_SOURCE') == 1

# The real caller must not arm a new peak tracker during an active pulse.
assert runner.count('if(work_mode && event.physical_next_peak_side<0 &&\n'
                    '      tryWeakSideAssist(event,work_choice,unmodeled_coast,t_test_ms,fast_v)) {') == 1
caller = runner.split('tryWeakSideAssist(event,work_choice,unmodeled_coast,t_test_ms,fast_v)) {', 1)[1]
assert 'finishFootEnergyDecision(event,t_test_ms);\n    // A successful output owns its half-cycle until updatePulse stops it.\n    if(!event.output_executed)rearm_for_next_peak();\n    return;' in caller

# A branch-13 response has no fitted energy prediction and cannot be fed into
# residual, adaptive-CG or normal negative-model training as though it had one.
assert 'energy_control_autonomous_pending_model_energy_j_=NAN;' in method
assert 'energy_control_autonomous_pending_probe_=true;' in method
assert 'work_positive_pending_=false;work_negative_pending_=false;' in method
assert 'walking_loss_feedback_.interrupt();walking_low_speed_guard_.interrupt();adaptive_cg_.breakPair();' in method
peak = runner.split('bool ExperimentRunner::recordEnergyControlAutonomousPeak(', 1)[1].split(
    '\nvoid ExperimentRunner::', 1)[0]
assert 'physical_side == energy_control_autonomous_pending_next_side_' in peak
assert '!energy_control_autonomous_pending_probe_' in peak
assert 'std::isfinite(energy_control_autonomous_pending_model_energy_j_)' in peak

with tempfile.TemporaryDirectory(prefix='weak_assist_runner_') as td:
    source = Path(td) / 'runner_fixture.cpp'
    exe = Path(td) / 'runner_fixture'
    source.write_text(fixture.replace('// METHOD_SOURCE', method), encoding='utf-8')
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root / 'src'), str(source), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
