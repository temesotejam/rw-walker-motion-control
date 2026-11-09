from pathlib import Path

config = Path("src/config.h").read_text(encoding="utf-8")
main = Path("src/main.cpp").read_text(encoding="utf-8")
runner_h = Path("src/experiment_runner.h").read_text(encoding="utf-8")
runner = Path("src/experiment_runner.cpp").read_text(encoding="utf-8")
logger_h = Path("src/psram_logger.h").read_text(encoding="utf-8")
logger = Path("src/psram_logger.cpp").read_text(encoding="utf-8")
log_types = Path("src/log_types.h").read_text(encoding="utf-8")
manifest = Path("site/manifest.json").read_text(encoding="utf-8")

assert "v46l_fast_solver_shadow_20260914" in config
assert "V46l" in main
assert '"version": "0.46.16"' in manifest

# Physical/control behavior is deliberately frozen from V46j.
assert "CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL" in config
assert "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL" in config
assert "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 1000" in config
assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "sizeof(LogSample) == 226" in log_types
assert "RWLOG_FORMAT_VERSION = 46" in logger

# Timing probes are metadata-only and cannot be consulted by control selection.
for token in (
    "TimingProbeEvent", "addTimingProbeEvent", "v46k_timing_probe_events",
    "set_current_us", "state_update_us", "current_model_us",
    "update_pulse_model_us", "pulse_begin_total_us",
    "first_audit_log_us", "imu_update_call_us", "runner_update_call_us",
    "first_imu_dt_after_start_us", "first_imu_sample_offset_us",
):
    assert token in logger_h or token in logger, token

for token in (
    "recordTimingProbeLoop", "startTimingProbe", "maybeFinalizeTimingProbe",
    "set_current_t0_us", "pulse_model_t0_us", "probe_log",
    "r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start",
):
    assert token in runner_h or token in runner, token

assert "v46k_timing_probe_active" in main
assert "runner.recordTimingProbeLoop" in main

# Never print timing inside a Run: that would perturb the quantity being measured.
probe_region = runner[runner.index("void ExperimentRunner::startTimingProbe"):runner.index("void ExperimentRunner::captureAngleOffsets")]
assert "Serial." not in probe_region
assert "printf" not in probe_region

# The logger remains observational: controller code before logSampleNow may not
# read timing values for pulse selection or stopping.
controller_region = runner[:runner.index("void ExperimentRunner::logSampleNow()")]
for forbidden in (
    "timing_probe_event_.set_current_us >",
    "timing_probe_event_.first_imu_dt_after_start_us >",
    "timing_probe_event_.runner_update_call_us >",
):
    assert forbidden not in controller_region

# Run boundaries must not carry an unfinished probe into the next autonomous Run.
reset_region = runner[runner.index("void ExperimentRunner::resetEnergyControlAutonomous()"):runner.index("void ExperimentRunner::resetEnergyControlAutonomousPeakTracker") ]
for token in (
    "timing_probe_event_ = PsramLogger::TimingProbeEvent{}",
    "timing_probe_pending_ = false",
    "timing_probe_loop_captured_ = false",
    "timing_probe_log_captured_ = false",
    "timing_probe_imu_captured_ = false",
):
    assert token in reset_region, token

# The final pulse can end the 30 s Run before its first due 2 ms audit row.
# Keep one metadata event with an explicit capture mask rather than silently
# dropping it; this remains observational and cannot affect control.
assert "kMaxTimingProbeEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS" in logger_h
assert "uint8_t capture_mask" in logger_h
assert "bool complete" in logger_h
assert "terminal_partial" in runner
assert "timing_probe_event_.capture_mask" in runner
assert "capture_mask" in logger
assert "complete" in logger

print("V46l timing-probe guards passed")
