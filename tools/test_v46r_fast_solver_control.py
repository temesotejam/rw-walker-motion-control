#!/usr/bin/env python3
from pathlib import Path
import json

R = Path(__file__).resolve().parents[1]
runner = (R / 'src/experiment_runner.cpp').read_text(encoding='utf-8')
config = (R / 'src/config.h').read_text(encoding='utf-8')
imu_h = (R / 'src/imu_manager.h').read_text(encoding='utf-8')
main = (R / 'src/main.cpp').read_text(encoding='utf-8')
manifest = json.loads((R / 'site/manifest.json').read_text(encoding='utf-8'))
site = (R / 'site/index.html').read_text(encoding='utf-8')

assert 'v46aj_fixed_3ms_compensation_20260920' in config
assert manifest['version'] == '0.46.42'
assert 'V46al-R2' in manifest['name']
assert 'V46al-R2 / 0.46.42' in site

# Physical output envelope and safety limits remain unchanged.
for token in (
    'ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000',
    'ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100',
    'ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 1000',
    'ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100',
    'CURRENT_AUDIT_FAST_READ_PERIOD_US = 1000UL',
    'CURRENT_AUDIT_LOG_PERIOD_US = 2000UL',
):
    assert token in config, token
assert 'kMaximumDeliveryAgeUs = 10000' in imu_h
assert 'imu_acquisition_overflow_backlog_or_stale' in main

start = runner.index('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross')
end = runner.index('void ExperimentRunner::runEnergyControlAutonomousSolverShadow', start)
control = runner[start:end]

# V46aj physical selector uses the bounded fast search, not the 0..100 exhaustive scans.
for token in ('struct FastCandidate', 'evaluate_width', 'fast_pick_width',
              'while (hi > lo', 'const FastCandidate ff', 'const FastCandidate selected_fast'):
    assert token in control, token
assert 'width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms' not in control
assert 'solver_shadow_pending_ = true' not in control

# Invalid solver state must fail closed: record/coast/rearm, never fall back to a long scan.
assert control.count('if (!ff.valid)') == 1
assert control.count('if (!selected_fast.valid)') == 1
assert control.count('ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE') >= 2
assert control.count('rearm_for_next_peak();') >= 2

# Preserve the legacy special zero-output candidate exactly: passive energy wins unless
# a fast-search candidate produces a strict improvement.
for token in (
    'uint16_t selected_width_ms = 0;',
    'float selected_q_mA_s = 0.0f;',
    'float selected_energy_j = event.passive_energy_j;',
    'zero_output_error_j',
    'if (selected_fast.error_j < zero_output_error_j)',
):
    assert token in control, token

# Current write remains behind the existing state/hardware/pulse-width guards.
begin = runner[runner.index('bool ExperimentRunner::beginEnergyControlAutonomousPulse'):
               runner.index('bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse')]
for token in (
    'status_.emergency_stop',
    'status_.state != ExperimentState::RUNNING_BATCH_SWEEP',
    '!roller_ || !roller_->ok()',
    'pulse_width_ms < 1 || pulse_width_ms > Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS',
    'roller_->setCurrentMa(command_current_mA)',
):
    assert token in begin, token

# No deferred solver work is scheduled after a physical pulse ends.
pulse = runner[runner.index('void ExperimentRunner::updateEnergyControlAutonomousPulse'):
               runner.index('bool ExperimentRunner::recordEnergyControlAutonomousPeak')]
assert 'stopActivePulse(now_ms);' in pulse
assert 'runEnergyControlAutonomousSolverShadow();' not in pulse

print('V46aj fast physical selector + legacy zero-output semantics + unchanged safety guards PASS')
