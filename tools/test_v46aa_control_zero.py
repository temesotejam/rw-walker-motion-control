#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
runner_h = (ROOT / "src/experiment_runner.h").read_text(encoding="utf-8")
log_types = (ROOT / "src/log_types.h").read_text(encoding="utf-8")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")
config = (ROOT / "src/config.h").read_text(encoding="utf-8")
converter = (ROOT / "tools/convert_rwlog_to_csv.py").read_text(encoding="utf-8")

assert "v46aj_fixed_3ms_compensation_20260920" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "BMI270_I2C_HZ = 1000000UL" in config

# V46aa prediction-reference fields remain in the append-only log layout so old
# v48 tooling and old logs are still interpretable. V46ac treats them as diagnostics.
for token in (
    "pitch_mekf_detector_relative_deg",
    "mekf_detector_zero_predicted_abs_deg",
    "mekf_detector_zero_sample_us",
):
    assert token in runner_h, token

measurement = runner[
    runner.index("void ExperimentRunner::beginMeasurementRun"):
    runner.index("void ExperimentRunner::beginTrial")
]
assert "status_.mekf_detector_zero_predicted_abs_deg = raw_mekf_predicted_abs_deg_" in measurement

display = runner[
    runner.index("void ExperimentRunner::updateDisplayedAngles"):
    runner.index("void ExperimentRunner::updateCurrentRollState")
]
assert "status_.pitch_mekf_deg = status_.pitch_mekf_detector_relative_deg;" in display
assert "status_.pitch_mekf_detector_relative_deg =" in display
assert "status_.pitch_mekf_measurement_relative_deg" in display

motion = runner[
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousMotion"):
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousPeakTracker")
]
assert "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;" in motion
assert "pitch_mekf_predicted_abs_deg" not in motion
assert "raw_mekf_predicted_abs_deg_" not in motion

# V46aj amplitude uses the unprojected measurement-relative posterior.
assert "energy_control_autonomous_gyro_relative_deg_" not in motion
assert "pitch_mekf_measurement_relative_deg" in motion

assert "RWLOG_FORMAT_VERSION = 51" in logger
assert "sizeof(LogSample) == 258" in log_types
assert 'SAMPLE_FORMAT_V49 = SAMPLE_FORMAT_V48' in converter
assert "autonomous_control_prediction_enabled" in logger
assert "false" in logger

print("V46ac control-angle guards PASS; V46aa prediction fields retained append-only")
