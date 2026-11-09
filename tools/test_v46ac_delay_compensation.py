#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
config = (ROOT / "src/config.h").read_text(encoding="utf-8")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")

assert "v46aj_fixed_3ms_compensation_20260920" in config
assert "ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US = 3000UL" in config

display = runner[
    runner.index("void ExperimentRunner::updateDisplayedAngles"):
    runner.index("void ExperimentRunner::updateCurrentRollState")
]
for token in (
    "(r.gy_dps - status_.mekf_bias_y_dps) * Config::MEKF_GYRO_Y_SCALE",
    "Config::ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US",
    "status_.pitch_mekf_measurement_relative_deg + mekf_pitch_rate_dps * compensation_s",
    "status_.pitch_mekf_deg = status_.pitch_mekf_detector_relative_deg",
):
    assert token in display, token

# Video comparison coordinate itself remains uncompensated.
assert "updateMekfComparisonRelativeAngles();" in display
assert display.index("updateMekfComparisonRelativeAngles();") < display.index("V46ac delay compensation begin")

# Heavy quaternion prediction is skipped in Autonomous.
filter_series = runner[
    runner.index("void ExperimentRunner::updateFilterSeries"):
    runner.index("void ExperimentRunner::updateStartupCalibration")
]
assert "if (energy_control_autonomous_mode_)" in filter_series
assert "raw_mekf_predicted_abs_deg_ = NAN;" in filter_series
assert "mekf_.predictEulerDeg" in filter_series

# Compensation only affects timing angle, not energy amplitude or output envelope.
motion = runner[
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousMotion"):
    runner.index("void ExperimentRunner::updateEnergyControlAutonomousPeakTracker")
]
assert "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;" in motion
assert "energy_control_autonomous_gyro_relative_deg_" not in motion
assert "pitch_mekf_measurement_relative_deg" in motion
for token in (
    "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000",
    "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
):
    assert token in config, token

assert "RWLOG_FORMAT_VERSION = 51" in logger
assert "lightweight_scalar_delay_compensation" in logger
assert "autonomous_timing_compensation_us" in logger
print("V46ac lightweight delay compensation guards PASS")
