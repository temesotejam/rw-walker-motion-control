from pathlib import Path


def one(path, old, new):
    p = Path(path)
    s = p.read_text(encoding='utf-8')
    if s.count(old) != 1:
        raise RuntimeError(f'{path}: expected one match, got {s.count(old)}')
    p.write_text(s.replace(old, new, 1), encoding='utf-8')

# Non-mutating forward prediction API.
one('src/mekf6.hpp', '  EulerDeg eulerDeg() const;\n',
    '  EulerDeg eulerDeg() const;\n  EulerDeg predictEulerDeg(const Vec3& gyro_rad_s, float dt_s) const;\n')

p = Path('src/mekf6.cpp')
s = p.read_text(encoding='utf-8')
anchor = 'EulerDeg Mekf6::eulerDeg() const {\n'
if s.count(anchor) != 1:
    raise RuntimeError('mekf euler anchor missing')
impl = r'''EulerDeg Mekf6::predictEulerDeg(const Vec3& gyro_rad_s, float dt_s) const {
  if (!std::isfinite(dt_s) || dt_s <= 0.0f) return eulerDeg();
  const Vec3 omega{gyro_rad_s.x - bias_.x, gyro_rad_s.y - bias_.y, gyro_rad_s.z - bias_.z};
  const Quaternion q = quatNormalized(quatMultiply(q_, deltaQuat({omega.x * dt_s, omega.y * dt_s, omega.z * dt_s})));
  const float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
  const float roll = std::atan2(sinr_cosp, cosr_cosp);
  const float sinp = clampf(2.0f * (q.w*q.y - q.z*q.x), -1.0f, 1.0f);
  const float pitch = std::asin(sinp);
  const float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
  const float yaw = std::atan2(siny_cosp, cosy_cosp);
  return {radToDeg(roll), radToDeg(pitch), radToDeg(yaw)};
}

'''
s = s.replace(anchor, impl + anchor, 1)
p.write_text(s, encoding='utf-8')

# Status/private members.
p = Path('src/experiment_runner.h')
s = p.read_text(encoding='utf-8')
s = s.replace('  float pitch_mekf_abs_deg = 0.0f;          // continuous MEKF physical/video body-frame pitch coordinate\n',
              '  float pitch_mekf_abs_deg = 0.0f;          // posterior physical/video body-frame pitch\n'
              '  float pitch_mekf_predicted_abs_deg = 0.0f; // one-step-ahead control-time pitch\n'
              '  uint32_t mekf_prediction_horizon_us = 0;\n', 1)
s = s.replace('  float raw_mekf_pitch_abs_deg_ = 0.0f;\n',
              '  float raw_mekf_pitch_abs_deg_ = 0.0f;\n  float raw_mekf_predicted_abs_deg_ = 0.0f;\n', 1)
s = s.replace('  uint32_t last_imu_update_us_ = 0;\n',
              '  uint32_t last_imu_update_us_ = 0;  // V46g: last consumed gyro sequence\n  uint32_t last_mekf_accel_sequence_ = 0;\n', 1)
p.write_text(s, encoding='utf-8')

p = Path('src/experiment_runner.cpp')
s = p.read_text(encoding='utf-8')
s = s.replace('  uint32_t last_imu_update_us = 0;\n', '  uint32_t last_accel_sequence = 0;\n', 1)
s = s.replace('  constexpr float imu_hz = 1000.0f / Config::IMU_PERIOD_MS;\n',
              '  constexpr float madgwick_hz = static_cast<float>(Config::BMI270_ACCEL_ODR_HZ);\n', 1)
s = s.replace('filter_beta1_raw_.begin(imu_hz);', 'filter_beta1_raw_.begin(madgwick_hz);')
s = s.replace('filter_beta1_bias_.begin(imu_hz);', 'filter_beta1_bias_.begin(madgwick_hz);')
s = s.replace('filter_dynamic_raw_[i].begin(imu_hz);', 'filter_dynamic_raw_[i].begin(madgwick_hz);')
s = s.replace('filter_dynamic_bias_[i].begin(imu_hz);', 'filter_dynamic_bias_[i].begin(madgwick_hz);')

old = '''  const ImuReading& r = imu_->reading();
  if (r.last_update_us != 0 && r.last_update_us != last_imu_update_us_) {
    updateFilterSeries(r);
    updateDisplayedAngles(r);
    if (status_.state == ExperimentState::STARTUP_GYRO_CALIB) updateStartupCalibration(r);
    updateCurrentRollState(r, now_ms);
    if ((passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
      updateQ1ShadowAtZeroCross(now_ms);
      if (energy_control_autonomous_mode_) updateEnergyControlAutonomousMotion(now_ms);
    }
    last_imu_update_us_ = r.last_update_us;
  }
'''
new = '''  const ImuReading& r = imu_->reading();
  if (r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_) {
    updateFilterSeries(r);
    updateDisplayedAngles(r);
    if (status_.state == ExperimentState::STARTUP_GYRO_CALIB) updateStartupCalibration(r);
    updateCurrentRollState(r, now_ms);
    if ((passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
      updateQ1ShadowAtZeroCross(now_ms);
      if (energy_control_autonomous_mode_) updateEnergyControlAutonomousMotion(now_ms);
    }
    last_imu_update_us_ = r.gyro_sequence;
  }
'''
if s.count(old) != 1:
    raise RuntimeError('runner update gate missing')
s = s.replace(old, new, 1)

s = s.replace(
    '  const float dt_s = r.update_dt_us > 0 ? static_cast<float>(r.update_dt_us) / 1000000.0f\n'
    '                                        : static_cast<float>(Config::IMU_PERIOD_MS) / 1000.0f;\n',
    '  const float dt_s = r.gyro_update_dt_us > 0 ? static_cast<float>(r.gyro_update_dt_us) / 1000000.0f\n'
    '                                             : static_cast<float>(Config::IMU_POLL_PERIOD_US) / 1000000.0f;\n'
    '  const bool accel_is_new_for_filter = r.accel_sequence != 0 && r.accel_sequence != last_mekf_accel_sequence_;\n',
    1)

old = '''  if (mekf_initialized_ && mekf_.predict(mekf_gyro, dt_s)) {
    mekf_.updateAccel(mekf_accel);
  }
  if (mekf_initialized_) {
    const auto e = mekf_.eulerDeg();
    raw_mekf_pitch_abs_deg_ = e.pitch;
'''
new = '''  if (mekf_initialized_ && mekf_.predict(mekf_gyro, dt_s)) {
    if (accel_is_new_for_filter) {
      mekf_.updateAccel(mekf_accel);
      last_mekf_accel_sequence_ = r.accel_sequence;
    }
  }
  if (mekf_initialized_) {
    const auto e = mekf_.eulerDeg();
    raw_mekf_pitch_abs_deg_ = e.pitch;
    const uint32_t sample_age_us = r.last_gyro_update_us == 0 ? 0 : static_cast<uint32_t>(micros() - r.last_gyro_update_us);
    const uint32_t horizon_us = min<uint32_t>(Config::MEKF_CONTROL_PREDICTION_MAX_US,
        sample_age_us + Config::MEKF_CONTROL_PREDICTION_FIXED_US);
    status_.mekf_prediction_horizon_us = horizon_us;
    raw_mekf_predicted_abs_deg_ = mekf_.predictEulerDeg(mekf_gyro, static_cast<float>(horizon_us) * 1.0e-6f).pitch;
    status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
'''
if s.count(old) != 1:
    raise RuntimeError('MEKF block missing')
s = s.replace(old, new, 1)
s = s.replace('    status_.mekf_accel_used = d.accel_used;\n',
              '    status_.mekf_accel_used = accel_is_new_for_filter && d.accel_used;\n', 1)

old = '''  if (!v46_mekf_dynamic_compare) {
    filter_beta1_raw_.updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
    filter_beta1_bias_.updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g, r.az_g);
    raw_beta1_raw_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_raw_.getPitch();
    raw_beta1_bias_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_bias_.getPitch();
  } else {
'''
new = '''  if (!v46_mekf_dynamic_compare && accel_is_new_for_filter) {
    filter_beta1_raw_.updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
    filter_beta1_bias_.updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g, r.az_g);
    raw_beta1_raw_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_raw_.getPitch();
    raw_beta1_bias_pitch_deg_ = Config::PITCH_SIGN * filter_beta1_bias_.getPitch();
  } else if (v46_mekf_dynamic_compare) {
'''
if s.count(old) != 1:
    raise RuntimeError('beta1 block missing')
s = s.replace(old, new, 1)

old = '''    if (!v46_mekf_dynamic_compare) {
      filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
      filter_dynamic_raw_[i].updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
      raw_dynamic_raw_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_raw_[i].getPitch();
    } else {
      raw_dynamic_raw_pitch_deg_[i] = NAN;
    }
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g,
                                      r.az_g);
    raw_dynamic_bias_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_bias_[i].getPitch();
'''
new = '''    if (accel_is_new_for_filter) {
      if (!v46_mekf_dynamic_compare) {
        filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
        filter_dynamic_raw_[i].updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
        raw_dynamic_raw_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_raw_[i].getPitch();
      } else {
        raw_dynamic_raw_pitch_deg_[i] = NAN;
      }
      filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
      filter_dynamic_bias_[i].updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias,
                                        r.ax_g, r.ay_g, r.az_g);
      raw_dynamic_bias_pitch_deg_[i] = Config::PITCH_SIGN * filter_dynamic_bias_[i].getPitch();
    } else if (v46_mekf_dynamic_compare) {
      raw_dynamic_raw_pitch_deg_[i] = NAN;
    }
'''
if s.count(old) != 1:
    raise RuntimeError('dynamic beta block missing')
s = s.replace(old, new, 1)

s = s.replace('  status_.imu_last_update_us = r.last_update_us;\n  status_.imu_update_dt_us = r.update_dt_us;\n',
              '  status_.imu_last_update_us = r.last_gyro_update_us;\n  status_.imu_update_dt_us = r.gyro_update_dt_us;\n', 1)

# First occurrence is startup calibration reset.
s = s.replace('    raw_mekf_pitch_abs_deg_ = mekf_.eulerDeg().pitch;\n',
              '    raw_mekf_pitch_abs_deg_ = mekf_.eulerDeg().pitch;\n'
              '    raw_mekf_predicted_abs_deg_ = raw_mekf_pitch_abs_deg_;\n'
              '    status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;\n', 1)

# Both updateFilterSeries tail and updateDisplayedAngles keep posterior absolute; add predicted absolute.
needle = '  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;\n  status_.pitch_madgwick_dynamic_abs_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];\n'
if s.count(needle) != 2:
    raise RuntimeError(f'expected two absolute status assignments, got {s.count(needle)}')
s = s.replace(needle,
              '  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;\n'
              '  status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;\n'
              '  status_.pitch_madgwick_dynamic_abs_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];\n')
s = s.replace('    status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_ - offset_mekf_pitch_deg_;\n',
              '    status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_;\n', 1)

old = '''    const ImuReading& r = imu_->reading();
    if (r.last_update_us != 0 &&
        r.last_update_us != g_v46_mekf_run_reinit.last_imu_update_us) {
      g_v46_mekf_run_reinit.last_imu_update_us = r.last_update_us;
      if (UprightPoseGuide::isUprightStableSample(r)) {
'''
new = '''    const ImuReading& r = imu_->reading();
    if (r.accel_sequence != 0 &&
        r.accel_sequence != g_v46_mekf_run_reinit.last_accel_sequence) {
      g_v46_mekf_run_reinit.last_accel_sequence = r.accel_sequence;
      if (UprightPoseGuide::isUprightStableSample(r)) {
'''
if s.count(old) != 1:
    raise RuntimeError('reinit sample gate missing')
s = s.replace(old, new, 1)

p.write_text(s, encoding='utf-8')

# Host test for non-mutating prediction.
p = Path('tools/test_mekf_host.cpp')
s = p.read_text(encoding='utf-8')
anchor = '  // 4) Strong translational acceleration must still be rejected.\n'
if s.count(anchor) != 1:
    raise RuntimeError('host test anchor missing')
test = r'''  // 3b) V46g one-step prediction must not mutate the posterior.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  const float posterior_before = reportedPitch(f);
  const auto pred = f.predictEulerDeg(gyroFilter(0, 90, 0), 0.0025f);
  const float posterior_after = reportedPitch(f);
  std::printf("v46g_predicted_pitch_2p5ms=%.4f posterior_after=%.4f\n", pred.pitch, posterior_after);
  if (!(pred.pitch > 0.19f && pred.pitch < 0.22f)) return 15;
  if (std::fabs(posterior_before - posterior_after) > 1.0e-6f) return 16;

'''
s = s.replace(anchor, test + anchor, 1)
s = s.replace('V46f MEKF Ry180 calibrated-gyro physical-frame test passed',
              'V46g MEKF 400Hz prediction / 200Hz accel test passed', 1)
p.write_text(s, encoding='utf-8')
print('V46g MEKF high-rate/prediction patch prepared')
