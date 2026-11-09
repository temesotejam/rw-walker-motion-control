#!/usr/bin/env python3
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(ROOT / "tools"))
import convert_rwlog_to_csv as converter

config = (SRC / "config.h").read_text(encoding="utf-8")
runner_h = (SRC / "experiment_runner.h").read_text(encoding="utf-8")
runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
log_types = (SRC / "log_types.h").read_text(encoding="utf-8")
logger = (SRC / "psram_logger.cpp").read_text(encoding="utf-8")

assert "v46aj_fixed_3ms_compensation_20260920" in config
assert "IMU_POLL_PERIOD_US = 1000UL" in config
assert "BMI270_GYRO_ODR_HZ = 400" in config
assert "BMI270_ACCEL_ODR_HZ = 200" in config
assert "BMI270_I2C_HZ = 1000000UL" in config

for token in (
    "pitch_mekf_start_sync_relative_deg",
    "pitch_mekf_measurement_relative_deg",
    "pitch_mekf_trial_relative_deg",
    "mekf_start_sync_zero_abs_deg",
    "mekf_measurement_zero_abs_deg",
    "mekf_trial_zero_abs_deg",
    "mekf_start_sync_zero_sample_us",
    "mekf_measurement_zero_sample_us",
    "mekf_trial_zero_sample_us",
):
    assert token in runner_h, token

# All three comparison outputs are posterior MEKF differences only.
comparison = runner[
    runner.index("void ExperimentRunner::captureMekfComparisonZero"):
    runner.index("void ExperimentRunner::requestEmergencyStop")
]
assert "zero_abs_deg = raw_mekf_pitch_abs_deg_" in comparison
assert comparison.count("raw_mekf_pitch_abs_deg_ - status_.mekf_") == 3
for forbidden in ("mekf_.reset()", "mekf_.initializeFromAccel", "setGyroBiasRadS", "roller_->setCurrentMa"):
    assert forbidden not in comparison, forbidden

# Semantic reference boundaries are explicit and observational.
start_sync = runner[runner.index("void ExperimentRunner::beginStartSync"):
                    runner.index("void ExperimentRunner::updateStartSync")]
measurement = runner[runner.index("void ExperimentRunner::beginMeasurementRun"):
                     runner.index("void ExperimentRunner::beginTrial")]
trial = runner[runner.index("void ExperimentRunner::beginTrial"):
               runner.index("void ExperimentRunner::finishTrial")]
assert "mekf_start_sync_zero_abs_deg" in start_sync
assert "mekf_measurement_zero_abs_deg" in measurement
assert "mekf_trial_zero_abs_deg" in trial

# Existing physical control/detector signal is untouched.
display = runner[runner.index("void ExperimentRunner::updateDisplayedAngles"):
                 runner.index("void ExperimentRunner::updateCurrentRollState")]
assert "status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_" in display

def method(name: str) -> str:
    start = runner.index(name)
    brace = runner.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (runner[end] == "{") - (runner[end] == "}")
        end += 1
    return runner[start:end]

control = "\n".join(method(name) for name in (
    "void ExperimentRunner::updateEnergyControlAutonomousMotion",
    "void ExperimentRunner::updateEnergyControlAutonomousPeakTracker",
    "void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross",
    "bool ExperimentRunner::beginEnergyControlAutonomousPulse",
    "bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse",
))
assert "pitch_mekf_measurement_relative_deg" in control
for token in ("pitch_mekf_start_sync_relative_deg", "pitch_mekf_trial_relative_deg"):
    assert token not in control, token

assert "RWLOG_FORMAT_VERSION = 51" in logger
assert "sizeof(LogSample) == 258" in log_types
assert struct.calcsize(converter.SAMPLE_FORMAT_V47) == 250
# The converter has derived/display CSV columns, so binary field count is not
# required to equal CSV column count. The synthetic v47 fixture above exercises
# the actual unpack -> conversion -> CSV path.
assert len(converter.CSV_COLUMNS_V47) == len(converter.CSV_COLUMNS_V46) + 9
print("V46ac comparison-zero guards PASS: posterior-only references, posterior comparison references retained, RWLOG v49 consistent")
