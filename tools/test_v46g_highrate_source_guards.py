#!/usr/bin/env python3
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
config = (SRC / "config.h").read_text(encoding="utf-8")
imu_h = (SRC / "imu_manager.h").read_text(encoding="utf-8")
imu_cpp = (SRC / "imu_manager.cpp").read_text(encoding="utf-8")
runner_h = (SRC / "experiment_runner.h").read_text(encoding="utf-8")
runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
mekf_h = (SRC / "mekf6.hpp").read_text(encoding="utf-8")
mekf_cpp = (SRC / "mekf6.cpp").read_text(encoding="utf-8")
for token in (
    "IMU_POLL_PERIOD_US = 1000UL", "BMI270_GYRO_ODR_HZ = 400", "BMI270_ACCEL_ODR_HZ = 200",
    "BMI270_GYRO_ODR_CODE = 0x0A", "BMI270_ACCEL_ODR_CODE = 0x09",
    "MEKF_CONTROL_PREDICTION_FIXED_US = 2500UL", "MEKF_CONTROL_PREDICTION_MAX_US = 10000UL"):
    assert token in config, token
assert (
    "v46l_fast_solver_shadow_20260914" in config or
    "v46aj_fixed_3ms_compensation_20260920" in config
), "supported V46l/V46s controller identity"
for token in ("M5.Imu.getType() != m5::imu_bmi270", "getImuInstancePtr(0)", "sensor_mask_accel",
              "sensor_mask_gyro", "accel_sequence", "gyro_sequence"):
    assert token in imu_cpp or token in imu_h, token
assert "predictEulerDeg" in mekf_h
assert "EulerDeg Mekf6::predictEulerDeg" in mekf_cpp
assert "accel_is_new_for_filter" in runner
assert "last_mekf_accel_sequence_" in runner_h
assert "raw_mekf_predicted_abs_deg_" in runner
assert "status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_" in runner
assert "status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_" in runner
assert "r.accel_sequence != g_v46_mekf_run_reinit.last_accel_sequence" in runner
print("V46l/V46s high-rate / forward-prediction source guards passed")

web = Path("src/web_ui.cpp").read_text(encoding="utf-8")
assert ("if(refreshInFlight)return;" in web or "if(downloading||refreshInFlight)return;" in web)
# V46o+: a bounded, fixed-size heartbeat replaces the 41-second blind pause.
assert "setInterval(refresh,1000)" in web
status_region = web[web.index("void WebUi::handleStatus()"):web.index("void WebUi::handleStartEnergyControlAutonomous()")]
assert "char body[192]" in status_region
assert status_region.index("return;") < status_region.index("statusJson()")
assert "41000" not in web
