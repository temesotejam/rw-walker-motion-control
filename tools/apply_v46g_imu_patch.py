from pathlib import Path


def one(path, old, new):
    p = Path(path)
    s = p.read_text(encoding='utf-8')
    if s.count(old) != 1:
        raise RuntimeError(f'{path}: expected one match, got {s.count(old)}')
    p.write_text(s.replace(old, new, 1), encoding='utf-8')

# Exact 2.5 ms poll schedule + BMI270 ODR targets.
one('src/config.h',
    'static constexpr uint16_t IMU_PERIOD_MS = 5;\n',
    '// V46g high-rate BMI270 schedule. Exact timing is microsecond-based.\n'
    'static constexpr uint32_t IMU_POLL_PERIOD_US = 2500UL;\n'
    'static constexpr uint16_t IMU_PERIOD_MS = 2;  // legacy integer RWLOG header field only\n'
    'static constexpr uint16_t BMI270_GYRO_ODR_HZ = 400;\n'
    'static constexpr uint16_t BMI270_ACCEL_ODR_HZ = 200;\n'
    'static constexpr uint8_t BMI270_GYRO_ODR_CODE = 0x0A;\n'
    'static constexpr uint8_t BMI270_ACCEL_ODR_CODE = 0x09;\n'
    'static constexpr uint32_t MEKF_CONTROL_PREDICTION_FIXED_US = 2500UL;\n'
    'static constexpr uint32_t MEKF_CONTROL_PREDICTION_MAX_US = 10000UL;\n')
one('src/config.h',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46f_mekf_ry180_gyro_y_calibrated_upright_reinit_20260913";\n',
    'static constexpr char ATTITUDE_VALIDATION_REVISION[] = "v46g_mekf_400hz_predict_200hz_accel_20260913";\n')

# Freshness/status fields.
one('src/imu_manager.h',
    '  uint32_t update_dt_us = 0;\n  uint32_t last_update_us = 0;\n  uint32_t last_update_ms = 0;\n  uint32_t error_count = 0;\n',
    '  bool accel_fresh = false;\n  bool gyro_fresh = false;\n  uint8_t sensor_mask = 0;\n'
    '  bool rate_config_ok = false;\n  uint8_t bmi270_acc_conf = 0;\n  uint8_t bmi270_gyr_conf = 0;\n'
    '  uint32_t accel_sequence = 0;\n  uint32_t gyro_sequence = 0;\n'
    '  uint32_t accel_update_dt_us = 0;\n  uint32_t gyro_update_dt_us = 0;\n'
    '  uint32_t last_accel_update_us = 0;\n  uint32_t last_gyro_update_us = 0;\n'
    '  uint32_t update_dt_us = 0;\n  uint32_t last_update_us = 0;\n  uint32_t last_update_ms = 0;\n  uint32_t error_count = 0;\n')
one('src/imu_manager.h',
    '  uint32_t prev_update_us_ = 0;\n',
    '  uint32_t prev_gyro_update_us_ = 0;\n  uint32_t prev_accel_update_us_ = 0;\n')
one('src/imu_manager.h',
    '  bool ok() const { return imu_present_ && reading_.imu_ok; }\n',
    '  bool ok() const { return imu_present_ && reading_.imu_ok && reading_.rate_config_ok; }\n')

p = Path('src/imu_manager.cpp')
s = p.read_text(encoding='utf-8')
start = s.index('bool ImuManager::begin() {')
end = s.index('\nvoid ImuManager::zeroPitch()')
new = r'''bool ImuManager::begin() {
  imu_present_ = M5.Imu.begin();
  if (!imu_present_) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "imu_init_failed";
    return false;
  }
  if (M5.Imu.getType() != m5::imu_bmi270) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "unexpected_imu_type_not_bmi270";
    return false;
  }
  auto* dev = M5.Imu.getImuInstancePtr(0);
  if (!dev) {
    reading_.imu_ok = false;
    reading_.rate_config_ok = false;
    last_error_ = "bmi270_instance_missing";
    return false;
  }
  constexpr uint8_t kAccConf = 0x40;
  constexpr uint8_t kGyrConf = 0x42;
  const uint8_t acc0 = dev->readRegister8(kAccConf);
  const uint8_t gyr0 = dev->readRegister8(kGyrConf);
  const uint8_t acc_target = static_cast<uint8_t>((acc0 & 0xF0u) | Config::BMI270_ACCEL_ODR_CODE);
  const uint8_t gyr_target = static_cast<uint8_t>((gyr0 & 0xF0u) | Config::BMI270_GYRO_ODR_CODE);
  const bool write_ok = dev->writeRegister8(kAccConf, acc_target) && dev->writeRegister8(kGyrConf, gyr_target);
  delay(2);
  reading_.bmi270_acc_conf = dev->readRegister8(kAccConf);
  reading_.bmi270_gyr_conf = dev->readRegister8(kGyrConf);
  reading_.rate_config_ok = write_ok &&
      ((reading_.bmi270_acc_conf & 0x0Fu) == Config::BMI270_ACCEL_ODR_CODE) &&
      ((reading_.bmi270_gyr_conf & 0x0Fu) == Config::BMI270_GYRO_ODR_CODE);
  if (!reading_.rate_config_ok) {
    reading_.imu_ok = false;
    last_error_ = "bmi270_odr_config_failed";
    return false;
  }
  reading_.imu_ok = true;
  reading_.last_update_ms = millis();
  last_error_ = "";
  return true;
}

void ImuManager::update() {
  reading_.accel_fresh = false;
  reading_.gyro_fresh = false;
  reading_.sensor_mask = 0;
  const uint32_t now_us = micros();
  if (last_due_us_ != 0 && static_cast<uint32_t>(now_us - last_due_us_) < Config::IMU_POLL_PERIOD_US) return;
  last_due_us_ = now_us;
  if (!imu_present_ || !reading_.rate_config_ok) {
    reading_.imu_ok = false;
    reading_.error_count++;
    last_error_ = "imu_not_ready";
    return;
  }
  const auto mask = M5.Imu.update();
  const uint8_t bits = static_cast<uint8_t>(mask);
  if (bits == 0) return;
  const auto d = M5.Imu.getImuData();
  const uint32_t sample_us = d.usec ? d.usec : now_us;
  const bool accel_new = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_accel);
  const bool gyro_new  = bits & static_cast<uint8_t>(m5::IMU_Class::sensor_mask_gyro);
  reading_.sensor_mask = bits;
  reading_.accel_fresh = accel_new;
  reading_.gyro_fresh = gyro_new;
  if (accel_new) {
    reading_.ax_g = d.accel.x; reading_.ay_g = d.accel.y; reading_.az_g = d.accel.z;
    reading_.acc_norm_g = sqrtf(reading_.ax_g*reading_.ax_g + reading_.ay_g*reading_.ay_g + reading_.az_g*reading_.az_g);
    reading_.acc_norm_error_g = reading_.acc_norm_g - 1.0f;
    reading_.pitch_accel_only_deg = Config::PITCH_SIGN * atan2f(-reading_.ax_g, sqrtf(reading_.ay_g*reading_.ay_g + reading_.az_g*reading_.az_g)) * 57.2957795f;
    reading_.accel_update_dt_us = prev_accel_update_us_ ? static_cast<uint32_t>(sample_us - prev_accel_update_us_) : 1000000UL / Config::BMI270_ACCEL_ODR_HZ;
    prev_accel_update_us_ = sample_us;
    reading_.last_accel_update_us = sample_us;
    ++reading_.accel_sequence;
  }
  if (gyro_new) {
    reading_.gx_dps = d.gyro.x; reading_.gy_dps = d.gyro.y; reading_.gz_dps = d.gyro.z;
    reading_.pitch_rate_dps = Config::GYRO_PITCH_RATE_SIGN * reading_.gy_dps;
    reading_.gyro_update_dt_us = prev_gyro_update_us_ ? static_cast<uint32_t>(sample_us - prev_gyro_update_us_) : Config::IMU_POLL_PERIOD_US;
    prev_gyro_update_us_ = sample_us;
    reading_.last_gyro_update_us = sample_us;
    ++reading_.gyro_sequence;
    reading_.update_dt_us = reading_.gyro_update_dt_us;
    reading_.last_update_us = sample_us;
    reading_.last_update_ms = millis();
  }
  reading_.time_since_last_pulse_ms = static_cast<uint16_t>(min<uint32_t>(65535, beta_context_time_since_last_pulse_ms_));
  consecutive_errors_ = 0;
  reading_.imu_ok = true;
  last_error_ = "";
}
'''
s = s[:start] + new + s[end:]
s = s.replace('  if (!reading_.imu_ok) return true;\n', '  if (!reading_.imu_ok || !reading_.rate_config_ok || reading_.last_gyro_update_us == 0) return true;\n', 1)
p.write_text(s, encoding='utf-8')
print('V46g BMI270 400/200 Hz patch prepared')
