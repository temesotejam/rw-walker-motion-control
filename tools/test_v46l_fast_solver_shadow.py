from pathlib import Path

config = Path("src/config.h").read_text(encoding="utf-8")
runner_h = Path("src/experiment_runner.h").read_text(encoding="utf-8")
runner = Path("src/experiment_runner.cpp").read_text(encoding="utf-8")
logger_h = Path("src/psram_logger.h").read_text(encoding="utf-8")
logger = Path("src/psram_logger.cpp").read_text(encoding="utf-8")
log_types = Path("src/log_types.h").read_text(encoding="utf-8")
manifest = Path("site/manifest.json").read_text(encoding="utf-8")

assert "v46l_fast_solver_shadow_20260914" in config
assert '"version": "0.46.16"' in manifest

# Physical/control configuration remains frozen from V46k.
for token in (
    "CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL",
    "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL",
    "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000",
    "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 1000",
    "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
    "BMI270_GYRO_ODR_HZ = 400",
    "BMI270_ACCEL_ODR_HZ = 200",
):
    assert token in config, token
assert "sizeof(LogSample) == 226" in log_types
assert "RWLOG_FORMAT_VERSION = 46" in logger

# Solver shadow is metadata-only.
for token in (
    "SolverShadowEvent",
    "addSolverShadowEvent",
    "v46l_solver_shadow_events",
    "legacy_ff_width_ms",
    "legacy_selected_width_ms",
    "fast_ff_width_ms",
    "fast_selected_width_ms",
    "legacy_ff_scan_us",
    "legacy_selected_scan_us",
    "legacy_decision_us",
    "fast_shadow_us",
    "fast_eval_count",
    "ff_width_match",
    "selected_width_match",
):
    assert token in logger_h or token in logger, token

# Legacy exhaustive scans remain the physical selector.
assert runner.count(
    "width_ms <= Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS; ++width_ms"
) >= 2
assert (
    "beginEnergyControlAutonomousPulse(now_ms, t_test_ms, event.q_command_direction, selected_width_ms)"
    in runner
)

# Fast solver is deferred until the already-commanded normal pulse has ended.
pulse_region = runner[
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousPulse"):
    runner.index("bool ExperimentRunner::recordEnergyControlAutonomousPeak")
]
assert "stopActivePulse(now_ms);" in pulse_region
assert "runEnergyControlAutonomousSolverShadow();" in pulse_region
assert pulse_region.index("stopActivePulse(now_ms);") < pulse_region.index(
    "runEnergyControlAutonomousSolverShadow();"
)

shadow_start = runner.index("void ExperimentRunner::runEnergyControlAutonomousSolverShadow()")
shadow_end = runner.index("void ExperimentRunner::startTimingProbe", shadow_start)
shadow_region = runner[shadow_start:shadow_end]
for token in (
    "fast_pick_width",
    "while (hi > lo",
    "evaluate_width",
    "logger_->addSolverShadowEvent(result)",
):
    assert token in shadow_region, token

# Fast result must never feed motor output or legacy width selection.
control_before_shadow = runner[:shadow_start]
for forbidden in (
    "fast_selected_width_ms)",
    "fast_ff_width_ms)",
    "result.fast_selected_width_ms",
):
    assert forbidden not in control_before_shadow, forbidden

# Run reset clears pending shadow state.
reset_start = runner.index("void ExperimentRunner::resetEnergyControlAutonomous()")
reset_end = runner.index("void ExperimentRunner::resetEnergyControlAutonomousPeakTracker", reset_start)
reset_region = runner[reset_start:reset_end]
assert "solver_shadow_event_ = PsramLogger::SolverShadowEvent{}" in reset_region
assert "solver_shadow_pending_ = false" in reset_region

print("V46l fast-solver shadow guards passed")
