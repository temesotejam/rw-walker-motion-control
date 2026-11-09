from pathlib import Path


def one(path, old, new):
    p = Path(path)
    s = p.read_text(encoding='utf-8')
    if s.count(old) != 1:
        raise RuntimeError(f'{path}: expected one match, got {s.count(old)}')
    p.write_text(s.replace(old, new, 1), encoding='utf-8')

# Web status: posterior vs predicted are explicit.
p = Path('src/web_ui.cpp')
s = p.read_text(encoding='utf-8')
anchor = '  json += ",\\"pitch_mekf_abs_deg\\":" + String(st.pitch_mekf_abs_deg, 3);\n'
if s.count(anchor) != 1:
    raise RuntimeError('web status MEKF abs anchor missing')
s = s.replace(anchor,
              anchor +
              '  json += ",\\"pitch_mekf_predicted_abs_deg\\":" + String(st.pitch_mekf_predicted_abs_deg, 3);\n' +
              '  json += ",\\"mekf_prediction_horizon_us\\":" + String(st.mekf_prediction_horizon_us);\n', 1)
p.write_text(s, encoding='utf-8')

# RWLOG metadata/header reserved values; binary sample layout stays v46.
p = Path('src/psram_logger.cpp')
s = p.read_text(encoding='utf-8')
anchor = '  json += "\\"imu_period_ms\\":" + String(Config::IMU_PERIOD_MS) + ",";\n'
if s.count(anchor) != 1:
    raise RuntimeError('logger imu metadata anchor missing')
s = s.replace(anchor,
              anchor +
              '  json += "\\"imu_poll_period_us\\":" + String(Config::IMU_POLL_PERIOD_US) + ",";\n' +
              '  json += "\\"bmi270_gyro_odr_hz\\":" + String(Config::BMI270_GYRO_ODR_HZ) + ",";\n' +
              '  json += "\\"bmi270_accel_odr_hz\\":" + String(Config::BMI270_ACCEL_ODR_HZ) + ",";\n' +
              '  json += "\\"mekf_control_prediction_fixed_us\\":" + String(Config::MEKF_CONTROL_PREDICTION_FIXED_US) + ",";\n' +
              '  json += "\\"mekf_control_prediction_max_us\\":" + String(Config::MEKF_CONTROL_PREDICTION_MAX_US) + ",";\n' +
              '  json += "\\"mekf_control_coordinate\\":\\"posterior_plus_forward_prediction; pitch_mekf_abs_deg remains posterior\\",";\n', 1)
one('src/psram_logger.cpp',
    '  header.imu_period_ms = Config::IMU_PERIOD_MS;\n',
    '  header.imu_period_ms = Config::IMU_PERIOD_MS;\n'
    '  header.reserved[0] = Config::IMU_POLL_PERIOD_US;\n'
    '  header.reserved[1] = Config::BMI270_GYRO_ODR_HZ;\n'
    '  header.reserved[2] = Config::BMI270_ACCEL_ODR_HZ;\n'
    '  header.reserved[3] = Config::MEKF_CONTROL_PREDICTION_FIXED_US;\n')

# V46g guard.
Path('tools/test_v46g_highrate_source_guards.py').write_text(r'''#!/usr/bin/env python3
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
    "IMU_POLL_PERIOD_US = 2500UL", "BMI270_GYRO_ODR_HZ = 400", "BMI270_ACCEL_ODR_HZ = 200",
    "BMI270_GYRO_ODR_CODE = 0x0A", "BMI270_ACCEL_ODR_CODE = 0x09",
    "MEKF_CONTROL_PREDICTION_FIXED_US = 2500UL", "MEKF_CONTROL_PREDICTION_MAX_US = 10000UL",
    "v46g_mekf_400hz_predict_200hz_accel_20260913"):
    assert token in config, token
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
print("V46g high-rate / forward-prediction source guards passed")
''', encoding='utf-8')

# Update old source guard for the intentional V46g semantics.
p = Path('tools/test_v46_motor_validation_source_guards.py')
s = p.read_text(encoding='utf-8')
s = s.replace('"v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913" in config',
              '"v46g_mekf_400hz_predict_200hz_accel_20260913" in config')
s = s.replace('"V46f MEKF motor-driven dynamic validation" in main_cpp',
              '"V46g MEKF motor-driven dynamic validation" in main_cpp')
s = s.replace('assert "status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_" in runner',
              'assert "status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_" in runner')
p.write_text(s, encoding='utf-8')

# Identity/docs/page.
for path in ['src/main.cpp', 'site/index.html', 'docs/FIRST_V46_DYNAMIC_VALIDATION.md', 'docs/MEKF_DYNAMIC_COMPARE_V46.md']:
    p = Path(path)
    s = p.read_text(encoding='utf-8')
    s = s.replace('V46f', 'V46g')
    s = s.replace('v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913',
                  'v46g_mekf_400hz_predict_200hz_accel_20260913')
    p.write_text(s, encoding='utf-8')
p = Path('site/manifest.json')
s = p.read_text(encoding='utf-8').replace('V46f', 'V46g').replace('"version": "0.46.5"', '"version": "0.46.6"')
p.write_text(s, encoding='utf-8')
print('V46g metadata/UI/guard patch prepared')
