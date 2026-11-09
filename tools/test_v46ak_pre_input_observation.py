from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
config = (ROOT / "src/config.h").read_text(encoding="utf-8")
roller_h = (ROOT / "src/roller485_manager.h").read_text(encoding="utf-8")
roller_cpp = (ROOT / "src/roller485_manager.cpp").read_text(encoding="utf-8")
runner = (ROOT / "src/experiment_runner.cpp").read_text(encoding="utf-8")
logger_h = (ROOT / "src/psram_logger.h").read_text(encoding="utf-8")
logger_cpp = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")
converter = (ROOT / "tools/convert_rwlog_to_csv.py").read_text(encoding="utf-8")
manifest = json.loads((ROOT / "site/manifest.json").read_text(encoding="utf-8"))
site = (ROOT / "site/index.html").read_text(encoding="utf-8")

# The adopted attitude/control baseline must remain V46aj.
assert 'ATTITUDE_VALIDATION_REVISION[] = "v46aj_fixed_3ms_compensation_20260920"' in config
assert "ENERGY_CONTROL_AUTONOMOUS_TIMING_COMPENSATION_US = 3000UL" in config
assert 'AMPLITUDE_CONTROL_OBSERVATION_REVISION[] = "v46ak_pre_input_state_observation_20260920"' in config

# V46ak observation remains frozen inside the stable-based successor build.
assert manifest["version"] == "0.46.42"
assert "V46al-R2" in manifest["name"]
assert "V46al-R2 / 0.46.42" in site

# Official Roller485 Speed Readback register and scale.
assert "REG_SPEED_READBACK = 0x60" in roller_cpp
assert "static_cast<float>(speed_raw_x100) / 100.0f" in roller_cpp
for token in ("speed_rpm", "speed_sample_time_us", "speed_sequence",
              "speed_read_failure_count", "speed_valid"):
    assert token in roller_h

# Speed sampling is observational and restricted to coast/no-pending-command state.
guard = "if (command_mA_ == 0 && requested_current_mA_ == 0)"
assert guard in roller_cpp
guard_pos = roller_cpp.index(guard)
speed_call_pos = roller_cpp.index("readSpeedFresh();", guard_pos)
assert speed_call_pos > guard_pos
active_audit = roller_cpp[roller_cpp.index("bool Roller485Manager::readCurrentFresh"):
                          roller_cpp.index("void Roller485Manager::recordFreshCurrent")]
assert "REG_SPEED_READBACK" not in active_audit

# A speed read failure must not itself feed the established recordIo safety state.
speed_method = roller_cpp[roller_cpp.index("bool Roller485Manager::readSpeedFresh"):
                          roller_cpp.index("void Roller485Manager::beginCurrentAuditPulse")]
assert "recordIo(" not in speed_method
assert "speed_valid = false" in speed_method

# Pre-input snapshot is captured before q_available / solver work and is not a controller input.
capture_pos = runner.index("const uint32_t pre_input_capture_us = micros();")
solver_pos = runner.index("event.q_available_mA_s = fabsf(predictedChargeMaS", capture_pos)
assert capture_pos < solver_pos
for token in (
    "pre_input_measured_current_mA",
    "pre_input_current_sample_time_us",
    "pre_input_current_age_us",
    "pre_input_current_valid",
    "pre_input_wheel_speed_rpm",
    "pre_input_wheel_speed_sample_time_us",
    "pre_input_wheel_speed_age_us",
    "pre_input_wheel_speed_valid",
):
    assert token in logger_h
    assert token in logger_cpp
    assert token in converter

# No pre-input observation field may influence the prediction/solver expression region.
solver_region = runner[solver_pos:runner.index("logger_->addEnergyControlAutonomousZeroCrossEvent(event);", solver_pos)]
assert "pre_input_" not in solver_region

# Time-series binary layout stays frozen at RWLOG v51; additions are metadata-event only.
assert "RWLOG_FORMAT_VERSION = 51" in logger_cpp
assert "sizeof(LogSample) == 258" in (ROOT / "src/log_types.h").read_text(encoding="utf-8")
assert "amplitude_control_observation_revision" in logger_cpp
assert "observation_only_never_read_by_control" in logger_cpp

print("V46ak pre-input observation guards PASS")
