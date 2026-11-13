#if FOOT_EARLY_TRIAL
#include "early_clock.h"
#include "esp_timer.h"
#endif
#include "control_angle_reference.h"
#include "experiment_runner.h"
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
#include "compact_json_table.h"
#endif
#include "log_quantization.h"
#include "log_sample_encoder.h"
#include "madgwick_pitch.h"
#include "control_work_profile.h"
#include "control_latency.h"
#include "rate_baseline_correction.h"
#include "previous_peak_control_correction.h"
#include "runtime_diagnostics.h"

#include <math.h>
#include <initializer_list>

#include "upright_pose_guide.h"
#include "upright_reference_store.h"
#include "direct_q_solver.h"
#include "negative_probe_batch.h"
#include "negative_probe_extended_batch.h"
#include "positive_current_batch.h"
#include "one_peak_energy_inverse.h"
#include "wheel_work_energy_control.h"
#include "unmodeled_walk_coast.h"
#include "foot_span_candidate.h"
#include "weak_side_assist.h"
#include "walking_positive_cap.h"
#include "peak_mpc.h"

namespace {
// Called only for an accepted ZERO cross. Keep diagnostic field assembly out
// of the periodic IRAM motion path and within the existing 24 KiB code gate.
__attribute__((noinline)) lateral_energy::Decision workDecision(
    const wheel_work_energy_control::Choice& choice, int8_t side,
    float passive_j, float target_j,
    const lateral_energy::CurrentTable& current_table,
    float initial_current_ma, float goal_current_ma) {
  lateral_energy::Decision d;
  d.valid=true;d.ms=choice.width_ms;d.evaluations=choice.evaluations;
  d.command_sign=choice.command_sign;
  d.free_j=std::isfinite(choice.free_mj) ? choice.free_mj*.001f : passive_j;
  d.target_j=target_j;
  d.predicted_j=choice.energy_mj*.001f;
  d.loss_j=choice.walking_loss_mj*.001f;
  d.deficit_j=d.target_j-d.free_j;d.commanded_deficit_j=d.deficit_j;
  d.corrected_target_j=d.target_j;d.energy_error_j=d.target_j-d.predicted_j;
  d.selected_q_mAs=(side>0 || choice.command_sign>0 || std::isfinite(choice.q_mas)) ? choice.q_mas :
      -lateral_energy::charge(current_table,d.ms,initial_current_ma,goal_current_ma);
  d.requested_q_mAs=d.selected_q_mAs;
  d.lower=choice.target_below_reachable;
  d.upper=choice.target_above_reachable;
  return d;
}
struct V46MekfRunReinitAccumulator {
  bool active = false;
  bool done = false;
  uint32_t last_accel_sequence = 0;
  uint32_t stable_sample_count = 0;
  double ax_sum_g = 0.0;
  double ay_sum_g = 0.0;
  double az_sum_g = 0.0;
};

V46MekfRunReinitAccumulator g_v46_mekf_run_reinit;

bool isSelectableZeroCrossCurrent(int16_t current_mA) {
  return current_mA >= Config::ZERO_CROSS_CURRENT_MIN_MA &&
         current_mA <= Config::ZERO_CROSS_CURRENT_MAX_MA;
}

bool isSelectableZeroCrossPulseMs(uint16_t pulse_width_ms) {
  return pulse_width_ms >= Config::ZERO_CROSS_TIME_SWEEP_MIN_PULSE_MS &&
         pulse_width_ms <= Config::ZERO_CROSS_TIME_SWEEP_MAX_PULSE_MS;
}

int8_t signedSide(float value) {
  return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

mekf6::Config makeMekfConfig() {
  mekf6::Config cfg;
  cfg.gyro_noise_std_rad_s = Config::MEKF_GYRO_NOISE_STD_RAD_S;
  cfg.gyro_bias_rw_std_rad_s_sqrt_s = Config::MEKF_GYRO_BIAS_RW_STD_RAD_S_SQRT_S;
  cfg.accel_direction_noise_std = Config::MEKF_ACCEL_DIRECTION_NOISE_STD;
  cfg.accel_mag_full_g = Config::MEKF_ACCEL_MAG_FULL_G;
  cfg.accel_mag_reject_g = Config::MEKF_ACCEL_MAG_REJECT_G;
  cfg.accel_angle_full_deg = Config::MEKF_ACCEL_ANGLE_FULL_DEG;
  cfg.accel_angle_reject_deg = Config::MEKF_ACCEL_ANGLE_REJECT_DEG;
  cfg.accel_min_confidence = Config::MEKF_ACCEL_MIN_CONFIDENCE;
  cfg.min_dt_s = Config::MEKF_MIN_DT_S;
  cfg.max_dt_s = Config::MEKF_MAX_DT_S;
  return cfg;
}

// V46f sensor-to-body adapter. Ry(pi) gives the physical/video body frame.
// The Y gyro sensitivity is additionally calibrated before MEKF prediction;
// this is an input-model correction, not a post-estimator angle scale.
mekf6::Vec3 mekfAccelFromRaw(const ImuReading& r) {
  return {-r.ax_g, r.ay_g, -r.az_g};
}

mekf6::Vec3 mekfGyroRadFromRaw(const ImuReading& r) {
  return {mekf6::degToRad(-r.gx_dps),
          mekf6::degToRad(r.gy_dps * Config::MEKF_GYRO_Y_SCALE),
          mekf6::degToRad(-r.gz_dps)};
}

mekf6::Vec3 mekfStartupBiasFromRaw(float bx_dps, float by_dps, float bz_dps) {
  return {mekf6::degToRad(-bx_dps),
          mekf6::degToRad(by_dps * Config::MEKF_GYRO_Y_SCALE),
          mekf6::degToRad(-bz_dps)};
}
}  // namespace

void ExperimentRunner::begin(PsramLogger& logger, ImuManager& imu, Roller485Manager& roller) {
  logger_ = &logger;
  imu_ = &imu;
  roller_ = &roller;
  upright_reference::load(upright_reference_);
  lateral_cg::load(lateral_cg_);
  boot_start_ms_ = millis();
  calib_start_ms_ = boot_start_ms_;
  beginFilters();
  pinMode(Config::SYNC_LED_PIN, OUTPUT);
  setSyncLed(false);
  stopMotor();
  status_.state = ExperimentState::STARTUP_GYRO_CALIB;
}

bool ExperimentRunner::beginUprightReferenceCapture(bool equilibrium) {
  // Equilibrium capture uses absolute pitch; saved upright is diagnostic only.
  if (running() || status_.pulse_active || status_.motor_cmd_mA != 0 ||
      upright_reference_capture_.pending() || !bias_ready_ ||
      (status_.state != ExperimentState::READY_TO_MEASURE &&
       status_.state != ExperimentState::FINISHED && status_.state != ExperimentState::ESTOP)) {
    if (status_.state == ExperimentState::READY_TO_MEASURE)
      status_.last_error = "upright_reference_requires_stopped";
    return false;
  }
  stopMotor();
  equilibrium_capture_ = equilibrium;
  upright_reference_capture_.begin(micros(), upright_reference_.revision);
  return true;
}

bool ExperimentRunner::pollUprightReferenceCapture(bool& ok, const char*& result) {
  using State = upright_reference::Capture::State;
  if (upright_reference_capture_.state() == State::Idle) return false;
  if (upright_reference_capture_.pending()) {
    const auto& r = imu_->reading();
    const auto& a = status_.mekf_attitude;
    const uint32_t now_us = micros();
    const bool healthy = imu_->acquisitionHealthy() && !imu_->stale(millis()) && a.valid &&
        static_cast<uint32_t>(now_us - a.sample_us) <= 15000 && r.last_accel_update_us &&
        static_cast<uint32_t>(now_us - r.last_accel_update_us) <= 20000;
    upright_reference_capture_.step(now_us, a.sample_us, r.gyro_sequence, healthy,
        !running() && !status_.pulse_active && status_.motor_cmd_mA == 0,
        raw_mekf_pitch_abs_deg_, r.ax_g, r.az_g, UprightPoseGuide::accelNormG(r),
        UprightPoseGuide::gyroNormDps(r));
  }
  if (upright_reference_capture_.pending()) return false;
  ok = upright_reference_capture_.state() == State::Ready;
  result = upright_reference_capture_.error();
  if (ok && equilibrium_capture_) {
    float shift = 0;
    if (!lateral_cg::fromEquilibrium(lateral_energy_geometry_,
        upright_reference_capture_.candidate().pitch_deg, shift)) {
      ok=false;result="equilibrium_contact_not_identifiable";
    } else {
      lateral_cg::Record record;
      record.magic=lateral_cg::kMagic;record.version=2;record.revision=lateral_cg_.revision+1;
      record.upright_revision=1;
      record.upright_abs_deg=0.f;record.shift_mm=shift;record.source=2;
      record.equilibrium_abs_deg=upright_reference_capture_.candidate().pitch_deg;
      ok=lateral_cg::save(record);
      if (ok) {
        lateral_cg_=record;
        if (status_.state==ExperimentState::READY_TO_MEASURE) status_.last_error="";
      }
      result=ok ? "lateral_cg_saved" : "lateral_cg_save_failed";
    }
  } else if (ok) {
    // The controller owner writes flash only while stopped. One blob is
    // committed and read back before the live reference is replaced.
    ok = upright_reference::save(upright_reference_capture_.candidate());
    if (ok) {
      upright_reference_ = upright_reference_capture_.candidate();
      result = "upright_reference_saved";
      if (status_.state == ExperimentState::READY_TO_MEASURE) status_.last_error = "";
    } else result = "upright_reference_save_failed";
  }
  // Capture feedback belongs to the Web command. Preserve a completed run's
  // terminal error, especially ESTOP, until its log has been downloaded.
  if (!ok && status_.state == ExperimentState::READY_TO_MEASURE) status_.last_error = result;
  upright_reference_capture_ = upright_reference::Capture{};
  equilibrium_capture_ = false;
  return true;
}

bool ExperimentRunner::setLateralCgMm(float shift_mm) {
  if (running() || status_.pulse_active || status_.motor_cmd_mA != 0 ||
      !std::isfinite(shift_mm) ||
      std::fabs(shift_mm)>lateral_cg::kMaxShiftMm ||
      (status_.state != ExperimentState::READY_TO_MEASURE &&
       status_.state != ExperimentState::FINISHED && status_.state != ExperimentState::ESTOP)) return false;
  lateral_cg::Record r;
  r.magic=lateral_cg::kMagic;r.version=2;r.revision=lateral_cg_.revision+1;
  r.upright_revision=1;r.upright_abs_deg=0.f;
  r.source=1;r.shift_mm=shift_mm;r.equilibrium_abs_deg=0.f;
  if (!lateral_cg::save(r)) return false;
  lateral_cg_=r;
  if (status_.state==ExperimentState::READY_TO_MEASURE) status_.last_error="";
  return true;
}
__attribute__((noinline)) float ExperimentRunner::controlPotential(float signed_deg) const {
  return lateral_cg::potential(lateral_energy_geometry_,signed_deg,wheelWorkMode() ? run_cg_shift_mm_ : 0.f);
}
__attribute__((noinline)) float ExperimentRunner::controlAngle(float energy,int8_t side) const {
  return lateral_cg::angle(lateral_energy_geometry_,energy,side,wheelWorkMode() ? run_cg_shift_mm_ : 0.f);
}

float ExperimentRunner::uprightPitchErrorDeg(const ImuReading& r) const {
  if (wheelWorkMode()) {
    const float norm=UprightPoseGuide::accelNormG(r);
    return norm>.2f ? fabsf(asinf(fmaxf(-1.f,fminf(1.f,r.ax_g/norm)))*57.295779513f) : 180.f;
  }
  return upright_reference::valid(upright_reference_)
      ? upright_reference::pitchError(r.ax_g, UprightPoseGuide::accelNormG(r), upright_reference_)
      : UprightPoseGuide::pitchErrorDeg(r);
}
bool ExperimentRunner::isUprightStableSample(const ImuReading& r) const {
  const float norm = UprightPoseGuide::accelNormG(r);
  return isfinite(norm) && norm >= .85f && norm <= 1.15f && r.az_g < 0 &&
      uprightPitchErrorDeg(r) <= appcfg::kAutoZeroMaxPitchErrorDeg &&
      UprightPoseGuide::gyroNormDps(r) <= UprightPoseGuide::UPRIGHT_MAX_GYRO_NORM_DPS;
}

void ExperimentRunner::beginFilters() {
  constexpr float madgwick_hz = static_cast<float>(Config::BMI270_ACCEL_ODR_HZ);
  mekf_.reset();
  mekf_.setConfig(makeMekfConfig());
  mekf_initialized_ = false;
  status_.mekf_attitude = MekfAttitudeSnapshot{};
  filter_beta1_raw_.begin(madgwick_hz);
  filter_beta1_bias_.begin(madgwick_hz);
  filter_beta1_raw_.setBeta(Config::MADGWICK_BETA_ONE);
  filter_beta1_bias_.setBeta(Config::MADGWICK_BETA_ONE);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    filter_dynamic_raw_[i].begin(madgwick_hz);
    filter_dynamic_bias_[i].begin(madgwick_hz);
    beta_smooth_[i] = betaCeilingForStrategy(i);
    filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
  }
  status_.beta_target = beta_smooth_[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = beta_smooth_[Config::FILTER_ADOPTED_INDEX];
}

void ExperimentRunner::serviceFast() {
  // Actual output is permitted only for one explicit Energy Control path.
  const bool energy_control_v0_pulse_live = energy_control_v0_mode_ &&
      energy_control_v0_pulse_authorized_ && status_.state == ExperimentState::RUNNING_BATCH_SWEEP &&
      !status_.emergency_stop && status_.pulse_active;
  const bool energy_control_autonomous_pulse_live = energy_control_autonomous_mode_ &&
      energy_control_autonomous_pulse_authorized_ && status_.state == ExperimentState::RUNNING_BATCH_SWEEP &&
      !status_.emergency_stop && status_.pulse_active;
  if (!energy_control_v0_pulse_live && !energy_control_autonomous_pulse_live) stopMotor();
}

void ExperimentRunner::updateImuDynamicBetaContext() {
  const uint32_t now_ms = millis();
  uint32_t since_last_pulse_ms = 65535;
  if (last_pulse_end_ms_ != 0) since_last_pulse_ms = now_ms - last_pulse_end_ms_;
  if (imu_) imu_->setDynamicBetaContext(status_.pulse_active, since_last_pulse_ms,
                                        status_.state != ExperimentState::RUNNING_BATCH_SWEEP);
}

void ExperimentRunner::update() {
  if (!logger_ || !imu_ || !roller_) return;
  const uint32_t now_ms = millis();
  status_.boot_elapsed_ms = now_ms - boot_start_ms_;
  const RollerTelemetry roller_snapshot = roller_->telemetrySnapshot();
  status_.roller_actual_current_mA = roller_snapshot.actual_current_mA;
  status_.roller_battery_mV = roller_snapshot.battery_mV;

  const ImuReading& r = imu_->reading();
  const bool new_gyro_sample = r.gyro_sequence != 0 && r.gyro_sequence != last_imu_update_us_;
  if (new_gyro_sample) {
    if (timing_probe_pending_ && !timing_probe_imu_captured_ &&
        r.gyro_sequence != timing_probe_event_.gyro_sequence_at_start &&
        static_cast<int32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us) >= 0) {
      timing_probe_event_.first_imu_dt_after_start_us = r.gyro_update_dt_us;
      timing_probe_event_.first_imu_sample_offset_us = r.last_gyro_update_us == 0 ? 0 :
          static_cast<uint32_t>(r.last_gyro_update_us - timing_probe_event_.pulse_start_us);
      timing_probe_imu_captured_ = true;
      maybeFinalizeTimingProbe();
    }
    updateFilterSeries(r);
    updateDisplayedAngles(r);
    if (status_.state == ExperimentState::STARTUP_GYRO_CALIB) updateStartupCalibration(r);
    updateCurrentRollState(r, now_ms);
    control_latency::mark(control_latency::Ready);
    if (running()) {
      const auto& attitude = status_.mekf_attitude;
      const mekf6::Vec3 upright{-UprightPoseGuide::REF_AX,
          UprightPoseGuide::REF_AY, -UprightPoseGuide::REF_AZ};
      const char* stop_reason = tilt_guard_.reason(attitude.quaternion, attitude.valid,
          static_cast<uint32_t>(micros() - attitude.sample_us), upright);
      if (stop_reason) { requestEmergencyStop(stop_reason); logSampleNow(); }
    }
    if ((passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) && status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
      control_work::Scope work(control_work::Stage::Motion, true, status_.pulse_active);
      updateQ1ShadowAtZeroCross(now_ms);
      if (energy_control_autonomous_mode_) updateEnergyControlAutonomousMotion(now_ms);
    }
    // Same sample and pre-decision beta; completed before state transitions,
    // logging and snapshot publication. Control cannot read this comparator.
    finishDeferredComparison();
    last_imu_update_us_ = r.gyro_sequence;
  }

  if (status_.state == ExperimentState::MADGWICK_SETTLING &&
      static_cast<uint32_t>(now_ms - settling_start_ms_) >= Config::MADGWICK_SETTLING_MS) {
    status_.state = ExperimentState::READY_TO_MEASURE;
  }

  if (status_.state == ExperimentState::START_SYNC) {
    status_.measure_elapsed_ms = 0;
    status_.remaining_ms = measurementTotalDurationMs();
    updateStartSync(now_ms);
    logSampleIfDue();
  } else if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP) {
    status_.measure_elapsed_ms = now_ms - run_start_ms_;
    status_.remaining_ms = status_.measure_elapsed_ms >= measurementTotalDurationMs()
                                ? 0
                                : measurementTotalDurationMs() - status_.measure_elapsed_ms;
    status_.trial_elapsed_ms = now_ms - trial_start_ms_;
    updateMidSyncLed(now_ms);
    if (passive_capture_mode_) {
      // The passive recording path has no pulse, Q, rebuild, or cooldown action.
      stopMotor();
    } else if (energy_control_autonomous_mode_) {
      updateEnergyControlAutonomousPulse(now_ms);
    } else if (energy_control_v0_mode_) {
      // Sole V0 actual-output route. It cannot enter the legacy scheduler.
      updateEnergyControlV0Pulse(now_ms);
    } else if (q_ident_mode_) {
      // Q_IDENT is frozen; retain only a fail-closed status path.
      updateQIdentPulse(now_ms);
    } else {
      updateInputPulse(now_ms);
    }
    logSampleIfDue();
    if (status_.trial_elapsed_ms >= status_.trial_duration_ms) {
      if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) finishRun();
      else finishTrial(now_ms);
    }
  } else if (status_.state == ExperimentState::TRIAL_REST) {
    status_.measure_elapsed_ms = now_ms - run_start_ms_;
    status_.remaining_ms = status_.measure_elapsed_ms >= measurementTotalDurationMs()
                                ? 0
                                : measurementTotalDurationMs() - status_.measure_elapsed_ms;
    status_.trial_elapsed_ms = 0;
    updateMidSyncLed(now_ms);
    logSampleIfDue();
    updateTrialRest(now_ms);
  } else if (status_.state == ExperimentState::END_SYNC) {
    status_.measure_elapsed_ms = measurementTotalDurationMs();
    status_.remaining_ms = 0;
    updateEndSync(now_ms);
    logSampleIfDue();
  } else {
    setSyncLed(false);
    stopMotor();
  }

  // Diagnostics run only after motion, actuator and logging work; no steering
  // or motor path reads the result. Log rows see the preceding sample.
  if (new_gyro_sample) updateGyroHeadingDiagnostic(r);
}

__attribute__((noinline)) void ExperimentRunner::updateGyroHeadingDiagnostic(const ImuReading& r) {
  if (energy_control_autonomous_mode_ && running()) {
    if (gyro_heading_start_pending_) {
      gyro_heading_start_pending_ = false;
      const bool fresh_accel = r.imu_ok && r.accel_sequence != 0 && r.last_accel_update_us != 0 &&
          static_cast<uint32_t>(micros() - r.last_accel_update_us) <= 50000;
      gyro_heading_diagnostic_.start(r.ax_g, r.ay_g, r.az_g,
          r.last_gyro_update_us, bias_ready_, fresh_accel);
    } else {
      gyro_heading_diagnostic_.sample(r.gx_dps, r.gy_dps, r.gz_dps,
          status_.gyro_bias_x_dps, status_.gyro_bias_y_dps, status_.gyro_bias_z_dps,
          Config::MEKF_GYRO_Y_SCALE, r.last_gyro_update_us);
    }
    status_.steering.gyro_valid = gyro_heading_diagnostic_.valid();
    status_.steering.yaw_deg = gyro_heading_diagnostic_.yaw();
  }
}

void ExperimentRunner::updateFilterSeries(const ImuReading& r) {
  control_work::Scope work(control_work::Stage::Filter,
      status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
  deferred_comparison_.pending = false;
  const float dt_s = r.gyro_update_dt_us > 0 ? static_cast<float>(r.gyro_update_dt_us) / 1000000.0f
                                             : static_cast<float>(Config::IMU_POLL_PERIOD_US) / 1000000.0f;
  const bool accel_is_new_for_filter = r.accel_sequence != 0 && r.accel_sequence != last_mekf_accel_sequence_;
  const float pitch_rate = Config::GYRO_PITCH_RATE_SIGN * r.gy_dps;
  const bool v46_mekf_dynamic_compare = energy_control_autonomous_mode_;
  if (!v46_mekf_dynamic_compare) {
    filter_beta1_raw_.setBeta(Config::MADGWICK_BETA_ONE);
    filter_beta1_bias_.setBeta(Config::MADGWICK_BETA_ONE);
  }

  const float gx_bias = bias_ready_ ? status_.gyro_bias_x_dps : 0.0f;
  const float gy_bias = bias_ready_ ? status_.gyro_bias_y_dps : 0.0f;
  const float gz_bias = bias_ready_ ? status_.gyro_bias_z_dps : 0.0f;

  const mekf6::Vec3 mekf_accel = mekfAccelFromRaw(r);
  const mekf6::Vec3 mekf_gyro = mekfGyroRadFromRaw(r);
  bool posterior_updated = false;
  if (!mekf_initialized_) {
    mekf_initialized_ = mekf_.initializeFromAccel(mekf_accel);
    posterior_updated = mekf_initialized_;
    if (mekf_initialized_ && bias_ready_) {
      mekf_.setGyroBiasRadS(mekfStartupBiasFromRaw(
          status_.gyro_bias_x_dps, status_.gyro_bias_y_dps, status_.gyro_bias_z_dps));
    }
  }
  bool predicted = false;
  if (mekf_initialized_) {
    control_work::Scope predict_work(control_work::Stage::MekfPredict,
        status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
    predicted = mekf_.predict(mekf_gyro, dt_s);
  }
  if (predicted) {
    posterior_updated = true;
    if (accel_is_new_for_filter) {
      control_work::Scope accel_work(control_work::Stage::MekfAccel,
          status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
      mekf_.updateAccel(mekf_accel);
      last_mekf_accel_sequence_ = r.accel_sequence;
    }
  }
  if (posterior_updated) {
    control_work::Scope attitude_work(control_work::Stage::MekfAttitude,
        status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
    status_.mekf_attitude = captureMekfAttitude(mekf_, mekf_initialized_,
        r.last_gyro_update_us, mekf_accel, mekf_gyro, r.last_accel_update_us);
  }
  if (mekf_initialized_) {
    raw_mekf_pitch_abs_deg_ = status_.mekf_attitude.pitch_deg;
    // V46ac autonomous diagnostic prediction begin
    // Avoid the quaternion forward-prediction cost in Autonomous. Legacy
    // non-Autonomous modes retain the previous prediction behavior.
    if (energy_control_autonomous_mode_) {
      status_.mekf_prediction_horizon_us = 0;
      raw_mekf_predicted_abs_deg_ = NAN;
      status_.pitch_mekf_predicted_abs_deg = NAN;
    } else {
      const uint32_t sample_age_us = r.last_gyro_update_us == 0 ? 0 :
          static_cast<uint32_t>(micros() - r.last_gyro_update_us);
      const uint32_t horizon_us = min<uint32_t>(Config::MEKF_CONTROL_PREDICTION_MAX_US,
          sample_age_us + Config::MEKF_CONTROL_PREDICTION_FIXED_US);
      status_.mekf_prediction_horizon_us = horizon_us;
      raw_mekf_predicted_abs_deg_ = mekf_.predictEulerDeg(
          mekf_gyro, static_cast<float>(horizon_us) * 1.0e-6f).pitch;
      status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
    }
    // V46ac autonomous diagnostic prediction end
    const auto q = status_.mekf_attitude.quaternion;
    status_.mekf_q_w = q.w; status_.mekf_q_x = q.x;
    status_.mekf_q_y = q.y; status_.mekf_q_z = q.z;
    const auto b = mekf_.gyroBiasRadS();
    // Convert the MEKF body-frame bias back to original raw IMU coordinates.
    status_.mekf_bias_x_dps = -mekf6::radToDeg(b.x);
    status_.mekf_bias_y_dps = mekf6::radToDeg(b.y) / Config::MEKF_GYRO_Y_SCALE;
    status_.mekf_bias_z_dps = -mekf6::radToDeg(b.z);
    const auto d = mekf_.diagnostics();
    status_.mekf_accel_confidence = d.accel_confidence;
    status_.mekf_accel_residual_deg = d.accel_direction_residual_deg;
    status_.mekf_accel_mag_error_g = d.accel_magnitude_error_g;
    status_.mekf_accel_used = accel_is_new_for_filter && d.accel_used;
  }
  control_latency::mark(control_latency::Mekf);
  if (!v46_mekf_dynamic_compare && accel_is_new_for_filter) {
    filter_beta1_raw_.updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
    filter_beta1_bias_.updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias, r.ax_g, r.ay_g, r.az_g);
    raw_beta1_raw_pitch_deg_ = Config::PITCH_SIGN * madgwick_pitch::readPitchDeg(filter_beta1_raw_);
    raw_beta1_bias_pitch_deg_ = Config::PITCH_SIGN * madgwick_pitch::readPitchDeg(filter_beta1_bias_);
  } else if (v46_mekf_dynamic_compare) {
    raw_beta1_raw_pitch_deg_ = NAN;
    raw_beta1_bias_pitch_deg_ = NAN;
  }
  const bool adopted_hold_after_input =
      !status_.pulse_active && last_pulse_end_ms_ != 0 &&
      static_cast<uint32_t>(millis() - last_pulse_end_ms_) <
          betaHoldAfterInputMsForStrategy(Config::FILTER_ADOPTED_INDEX);
  const bool input_protected = status_.pulse_active || adopted_hold_after_input;
  const float phase_rate_dps = pitch_rate - pitchBiasFromGyroBias();
  if (!v46_mekf_dynamic_compare) {
    if (Config::BETA_PHASE_RETURN_TEST_ENABLED) beta_phase_.update(phase_rate_dps, dt_s, input_protected);
    beta_turn_fast_.update(phase_rate_dps, dt_s, input_protected);
  }
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    const float beta_ceiling = betaCeilingForStrategy(i);
    if (v46_mekf_dynamic_compare && i != Config::FILTER_ADOPTED_INDEX) {
      raw_dynamic_raw_pitch_deg_[i] = NAN;
      raw_dynamic_bias_pitch_deg_[i] = NAN;
      status_.beta_target_series[i] = NAN;
      status_.beta_smooth_series[i] = NAN;
      continue;
    }
    float beta_target = beta_ceiling;
    if (i == Config::FILTER_FIXED_B100_INDEX || i == Config::FILTER_FIXED_B000_INDEX) {
      beta_smooth_[i] = betaCeilingForStrategy(i);
    } else if (i == Config::FILTER_DYNAMIC_TURN_FAST_INDEX) {
      beta_target = beta_turn_fast_.betaTarget(betaFloorForStrategy(i), beta_ceiling);
      beta_smooth_[i] = beta_target;
    } else {
      const uint16_t hold_after_input_ms = betaHoldAfterInputMsForStrategy(i);
      const bool hold_after_input =
          !status_.pulse_active && last_pulse_end_ms_ != 0 &&
          static_cast<uint32_t>(millis() - last_pulse_end_ms_) < hold_after_input_ms;
      if (status_.pulse_active || hold_after_input) {
        beta_target = betaFloorForStrategy(i);
        beta_smooth_[i] = beta_target;
      } else if (beta_smooth_[i] > beta_ceiling) {
        beta_smooth_[i] = beta_ceiling;
      } else if (beta_smooth_[i] < beta_ceiling) {
        // After the input floor, use the configured quadratic soft-start followed by
        // the configured linear ramp. Dynamic strategies are capped at beta_ceiling.
        const uint32_t elapsed_after_input_ms = static_cast<uint32_t>(millis() - last_pulse_end_ms_);
        const float recovery_ms = static_cast<float>(elapsed_after_input_ms - hold_after_input_ms);
        const float beta_floor = betaFloorForStrategy(i);
        // Scale the ramp so the reduced ceiling is reached at the same time that
        // the prior beta=0.100 profile would have reached its ceiling.
        const float reference_span = Config::BETA_TIME_MATCH_REFERENCE_MAX - beta_floor;
        const float target_span = beta_ceiling - beta_floor;
        const float recovery_rate_per_ms =
            (reference_span > 0.0f && target_span > 0.0f)
                ? Config::BETA_TIME_MATCH_REFERENCE_RECOVERY_PER_MS * target_span / reference_span
                : 0.0f;
        if (recovery_ms < static_cast<float>(Config::BETA_SOFT_START_MS)) {
          beta_target = beta_floor + recovery_rate_per_ms * recovery_ms * recovery_ms /
                                         (2.0f * static_cast<float>(Config::BETA_SOFT_START_MS));
        } else {
          beta_target = beta_floor + recovery_rate_per_ms *
                                         (recovery_ms - 0.5f * static_cast<float>(Config::BETA_SOFT_START_MS));
        }
        if (beta_target > beta_ceiling) beta_target = beta_ceiling;
        beta_smooth_[i] = beta_target;
      }
    }
    if (!v46_mekf_dynamic_compare && i == Config::FILTER_ADOPTED_INDEX && Config::BETA_PHASE_RETURN_TEST_ENABLED) {
      const float phase_ceiling = beta_phase_.betaCeiling(betaFloorForStrategy(i), beta_ceiling);
      if (beta_target > phase_ceiling) beta_target = phase_ceiling;
      beta_smooth_[i] = beta_target;
    }
    if (accel_is_new_for_filter) {
      const bool defer = v46_mekf_dynamic_compare &&
          status_.state == ExperimentState::RUNNING_BATCH_SWEEP;
      control_work::Scope madgwick_work(control_work::Stage::Madgwick,
          !defer && status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
      if (!v46_mekf_dynamic_compare) {
        filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
        filter_dynamic_raw_[i].updateIMU(r.gx_dps, r.gy_dps, r.gz_dps, r.ax_g, r.ay_g, r.az_g);
        raw_dynamic_raw_pitch_deg_[i] = Config::PITCH_SIGN * madgwick_pitch::readPitchDeg(filter_dynamic_raw_[i]);
      } else {
        raw_dynamic_raw_pitch_deg_[i] = NAN;
      }
      if (defer) {
        auto& input = deferred_comparison_;
        input.pending = true; input.pulse_at_entry = status_.pulse_active;
        input.gx = r.gx_dps - gx_bias; input.gy = r.gy_dps - gy_bias; input.gz = r.gz_dps - gz_bias;
        input.ax = r.ax_g; input.ay = r.ay_g; input.az = r.az_g; input.beta = beta_smooth_[i];
      } else {
        filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
        filter_dynamic_bias_[i].updateIMU(r.gx_dps - gx_bias, r.gy_dps - gy_bias, r.gz_dps - gz_bias,
                                          r.ax_g, r.ay_g, r.az_g);
        raw_dynamic_bias_pitch_deg_[i] = Config::PITCH_SIGN * madgwick_pitch::readPitchDeg(filter_dynamic_bias_[i]);
      }
    } else if (v46_mekf_dynamic_compare) {
      raw_dynamic_raw_pitch_deg_[i] = NAN;
    }
    status_.beta_target_series[i] = beta_target;
    status_.beta_smooth_series[i] = beta_smooth_[i];
  }
  // ImuManager already evaluates this identical expression once per new accel.
  raw_accel_pitch_deg_ = r.pitch_accel_only_deg;

  if (dt_s > 0.0f && dt_s < 1.0f) {
    gyro_raw_deg_ += pitch_rate * dt_s;
    gyro_bias_corrected_deg_ += (pitch_rate - pitchBiasFromGyroBias()) * dt_s;
  }

  status_.gyro_pitch_rate_dps = pitch_rate;
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  if (v46_mekf_dynamic_compare) {
    status_.beta_phase_state = 0;
    status_.beta_phase_progress = 0.0f;
    status_.beta_phase_peak_angle_deg = NAN;
    status_.beta_phase_angle_deg = NAN;
    status_.beta_phase_ceiling = NAN;
  } else {
    status_.beta_phase_state = static_cast<uint8_t>(beta_turn_fast_.state());
    status_.beta_phase_progress = beta_turn_fast_.recoveryProgress();
    status_.beta_phase_peak_angle_deg = beta_turn_fast_.peakAbsAngleDeg();
    status_.beta_phase_angle_deg = beta_turn_fast_.phaseAngleDeg();
    status_.beta_phase_ceiling = beta_turn_fast_.betaTarget(
        betaFloorForStrategy(Config::FILTER_DYNAMIC_TURN_FAST_INDEX),
        betaCeilingForStrategy(Config::FILTER_DYNAMIC_TURN_FAST_INDEX));
  }
  status_.ax_g = r.ax_g;
  status_.ay_g = r.ay_g;
  status_.az_g = r.az_g;
  status_.gx_dps = r.gx_dps;
  status_.gy_dps = r.gy_dps;
  status_.gz_dps = r.gz_dps;
  status_.acc_norm_g = r.acc_norm_g;
  status_.imu_last_update_us = r.last_gyro_update_us;
  status_.imu_update_dt_us = r.gyro_update_dt_us;
  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;
  status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
  status_.pitch_madgwick_dynamic_abs_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
}

void ExperimentRunner::updateStartupCalibration(const ImuReading& r) {
  bias_sum_x_ += r.gx_dps;
  bias_sum_y_ += r.gy_dps;
  bias_sum_z_ += r.gz_dps;
  bias_sample_count_++;
  status_.calibration_sample_count = bias_sample_count_;

  if (static_cast<uint32_t>(millis() - calib_start_ms_) < Config::STARTUP_GYRO_CALIB_MS) return;
  if (bias_sample_count_ > 0) {
    status_.gyro_bias_x_dps = static_cast<float>(bias_sum_x_ / bias_sample_count_);
    status_.gyro_bias_y_dps = static_cast<float>(bias_sum_y_ / bias_sample_count_);
    status_.gyro_bias_z_dps = static_cast<float>(bias_sum_z_ / bias_sample_count_);
  }
  status_.gyro_bias_pitch_dps = pitchBiasFromGyroBias();
  bias_ready_ = true;
  // Re-anchor the MEKF after the stationary startup-bias estimate so the
  // dynamic test begins from a clean attitude/covariance state.
  mekf_.reset();
  mekf_.setConfig(makeMekfConfig());
  mekf_initialized_ = mekf_.initializeFromAccel(mekfAccelFromRaw(r));
  status_.mekf_attitude = MekfAttitudeSnapshot{};
  if (mekf_initialized_) {
    mekf_.setGyroBiasRadS(mekfStartupBiasFromRaw(
        status_.gyro_bias_x_dps, status_.gyro_bias_y_dps, status_.gyro_bias_z_dps));
    status_.mekf_attitude = captureMekfAttitude(mekf_, true, r.last_gyro_update_us,
        mekfAccelFromRaw(r), mekfGyroRadFromRaw(r), r.last_accel_update_us);
    raw_mekf_pitch_abs_deg_ = status_.mekf_attitude.pitch_deg;
    raw_mekf_predicted_abs_deg_ = raw_mekf_pitch_abs_deg_;
    status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
  }
  settling_start_ms_ = millis();
  status_.state = ExperimentState::MADGWICK_SETTLING;
}

void ExperimentRunner::finishDeferredComparison() {
  if (!deferred_comparison_.pending) return;
  const auto input = deferred_comparison_;
  deferred_comparison_.pending = false;
  {
    control_work::Scope work(control_work::Stage::Madgwick, true, input.pulse_at_entry);
    constexpr uint8_t i = Config::FILTER_ADOPTED_INDEX;
    filter_dynamic_bias_[i].setBeta(input.beta);
    filter_dynamic_bias_[i].updateIMU(input.gx, input.gy, input.gz, input.ax, input.ay, input.az);
    raw_dynamic_bias_pitch_deg_[i] = Config::PITCH_SIGN * madgwick_pitch::readPitchDeg(filter_dynamic_bias_[i]);
  }
  updateComparisonDisplayAngles();
}

void ExperimentRunner::updateComparisonDisplayAngles() {
  status_.pitch_madgwick_dynamic_abs_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
  if (passive_capture_mode_) {
    // Preserve the continuous IMU gravity frame: no start-of-run subtraction.
    status_.pitch_madgwick_beta1_raw_deg = raw_beta1_raw_pitch_deg_;
    status_.pitch_madgwick_beta1_bias_deg = raw_beta1_bias_pitch_deg_;
    for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
      status_.pitch_dynamic_beta_deg[i] = raw_dynamic_bias_pitch_deg_[i];
    }
    status_.pitch_madgwick_dynamic_raw_deg = raw_dynamic_raw_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_madgwick_dynamic_bias_deg = raw_dynamic_bias_pitch_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_accel_only_deg = raw_accel_pitch_deg_;
  } else {
    status_.pitch_madgwick_beta1_raw_deg = raw_beta1_raw_pitch_deg_ - offset_beta1_raw_deg_;
    status_.pitch_madgwick_beta1_bias_deg = raw_beta1_bias_pitch_deg_ - offset_beta1_bias_deg_;
    for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
      status_.pitch_dynamic_beta_deg[i] = raw_dynamic_bias_pitch_deg_[i] - offset_dynamic_bias_deg_[i];
    }
    status_.pitch_madgwick_dynamic_raw_deg =
        raw_dynamic_raw_pitch_deg_[Config::FILTER_ADOPTED_INDEX] -
        offset_dynamic_raw_deg_[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_madgwick_dynamic_bias_deg = status_.pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX];
    status_.pitch_accel_only_deg = raw_accel_pitch_deg_ - offset_accel_deg_;
  }
}

void ExperimentRunner::updateDisplayedAngles(const ImuReading&) {
  control_work::Scope work(control_work::Stage::AngleDisplay,
      status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
  status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;
  status_.pitch_mekf_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
  status_.pitch_mekf_deg = passive_capture_mode_ ? raw_mekf_pitch_abs_deg_ :
      raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_;
  if (!deferred_comparison_.pending) updateComparisonDisplayAngles();
  status_.pitch_gyro_raw_deg = gyro_raw_deg_;
  status_.pitch_gyro_bias_corrected_deg = gyro_bias_corrected_deg_;
  // V46z comparison-zero begin
  updateMekfComparisonRelativeAngles();
  // V46z comparison-zero end
  // The adjustable input threshold uses the posterior angle directly.
  // It has no time-based advance or latency projection.
  status_.pitch_mekf_detector_relative_deg = status_.pitch_mekf_measurement_relative_deg;
  if (energy_control_autonomous_mode_) status_.pitch_mekf_deg = status_.pitch_mekf_detector_relative_deg;
  if (!running()) status_.pitch_mekf_deg = raw_mekf_pitch_abs_deg_;

}

void ExperimentRunner::updateCurrentRollState(const ImuReading& r, uint32_t now_ms) {
  control_work::Scope work(control_work::Stage::CurrentRoll,
      status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
  status_.physical_roll_candidate_deg = physicalRollCandidateDeg(r);
  status_.physical_roll_abs_deg =
      Config::PHYSICAL_ROLL_AFFINE_SLOPE * status_.physical_roll_candidate_deg +
      Config::PHYSICAL_ROLL_AFFINE_OFFSET_DEG;
  status_.physical_roll_rate_raw_dps = r.gy_dps;
  status_.body_rate_sample_time_us = bias_ready_ && r.imu_ok ? r.last_gyro_update_us : 0;
  status_.physical_roll_rate_dps =
      bias_ready_ ? r.gy_dps - status_.gyro_bias_y_dps : 0.0f;
  status_.display_zero_offset_deg = display_zero_offset_deg_;
  status_.current_roll_deg = status_.physical_roll_abs_deg - display_zero_offset_deg_;
  status_.target_roll_deg = target_roll_deg_;
  status_.target_error_deg = status_.current_roll_deg - target_roll_deg_;

  const bool rate_is_static = bias_ready_ &&
      fabsf(status_.physical_roll_rate_dps) <= Config::STATIC_RATE_THRESHOLD_DPS;
  if (!rate_is_static) {
    static_rate_since_ms_ = 0;
    status_.static_confirmed = false;
  } else {
    if (static_rate_since_ms_ == 0) static_rate_since_ms_ = now_ms;
    status_.static_confirmed =
        static_cast<uint32_t>(now_ms - static_rate_since_ms_) >= Config::STATIC_HOLD_TIME_MS;
  }
  status_.ready = status_.static_confirmed &&
      fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
}

bool ExperimentRunner::startBatchSweepTest() {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  zero_cross_mode_ = false;
  single_trial_mode_ = false;
  passive_capture_mode_ = false;
  selected_trial_index_ = 0;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.trial_index = 0;
  status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::BETA_SWEEP_TRIAL_DURATION_MS;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}

bool ExperimentRunner::startSingleTrialTest(uint8_t trial_number) {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  zero_cross_mode_ = false;
  passive_capture_mode_ = false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (trial_number < 1 || trial_number > Config::BETA_SWEEP_TRIAL_COUNT) {
    status_.last_error = "bad_trial";
    return false;
  }
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  single_trial_mode_ = true;
  selected_trial_index_ = trial_number - 1;
  const auto& trial = Config::BETA_SWEEP_TRIALS[selected_trial_index_];

  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.trial_index = trial_number;
  status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = trial.duration_ms;
  status_.current_mA_setting = trial.current_mA;
  status_.pulse_width_ms_setting = trial.pulse_width_ms;
  status_.input_interval_ms = trial.input_interval_ms;
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}


bool ExperimentRunner::startZeroCrossTest(int16_t fixed_current_mA, uint16_t fixed_pulse_width_ms,
                                           QRunMode q_run_mode, float control_target_deg,
                                           uint8_t q_probe_schedule_id) {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  if (q_run_mode == QRunMode::CONTROL &&
      (control_target_deg < Config::ZERO_CROSS_CONTROL_TARGET_MIN_DEG ||
       control_target_deg > Config::ZERO_CROSS_CONTROL_TARGET_MAX_DEG)) {
    status_.last_error = "invalid_control_target";
    return false;
  }
  if (q_run_mode == QRunMode::CONTROL &&
      q_probe_schedule_id >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT) {
    status_.last_error = "invalid_q_probe_schedule";
    return false;
  }
  if (!isSelectableZeroCrossCurrent(fixed_current_mA)) {
    status_.last_error = "invalid_zero_cross_current";
    return false;
  }
  if (!isSelectableZeroCrossPulseMs(fixed_pulse_width_ms)) {
    status_.last_error = "invalid_zero_cross_pulse_ms";
    return false;
  }
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  zero_cross_fixed_current_mA_ = fixed_current_mA;
  zero_cross_fixed_pulse_ms_ = fixed_pulse_width_ms;
  zero_cross_mode_ = true;
  passive_capture_mode_ = false;
  q_run_mode_ = q_run_mode;
  q_probe_schedule_id_ = q_run_mode_ == QRunMode::CONTROL ? q_probe_schedule_id :
      Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  identification_mode_ = q_run_mode_ != QRunMode::NONE;
  control_target_peak_deg_ = q_run_mode_ == QRunMode::CONTROL ? control_target_deg : 0.0f;
  identification_target_index_ = 0;
  identification_event_id_ = 0;
  identification_peak_waiting_ = false;
  identification_outbound_rate_sign_ = 0;
  identification_reverse_samples_ = 0;
  identification_peak_angle_deg_ = 0.0f;
  predicted_signed_current_end_mA_ = 0.0f;
  predicted_current_end_ms_ = 0;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  last_zero_cross_pulse_start_ms_ = 0;
  zero_cross_start_refractory_until_ms_ = 0;
  zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.measure_elapsed_ms = 0;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = measurementTotalDurationMs();
  status_.current_mA_setting = zero_cross_fixed_current_mA_;
  status_.pulse_width_ms_setting = zero_cross_fixed_pulse_ms_;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::startZeroCrossIdentificationTest() {
  return startZeroCrossTest(Config::ZERO_CROSS_OPERATING_CURRENT_MA, 20, QRunMode::VALIDATION);
}

bool ExperimentRunner::startZeroCrossControlTest(float target_peak_deg, uint8_t q_probe_schedule_id) {
  return startZeroCrossTest(Config::ZERO_CROSS_OPERATING_CURRENT_MA, 20,
                            QRunMode::CONTROL, target_peak_deg, q_probe_schedule_id);
}

bool ExperimentRunner::startPassiveCapture() {
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // Do not enter the zero-cross setup: this is a manual-release logger only.
  // The signed shadow target is snapshotted now, before the log is locked.
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  resetQIdentTracker();
  resetQ1ShadowZeroCrossTracker();
  passive_capture_mode_ = true;
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = 0.0f;
  zero_cross_fixed_current_mA_ = 0;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = Config::PASSIVE_CAPTURE_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::PASSIVE_CAPTURE_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}

bool ExperimentRunner::startEnergyControlV0Capture() {
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) { status_.last_error = "psram_not_ready"; return false; }
  if (!imu_->ok()) { status_.last_error = "imu_not_ready"; return false; }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // V0 is a manual one-release capture. No Q_IDENT arm, schedule, legacy
  // control, start-kick, rebuild, E2 or inverse-Q state can initialize here.
  passive_capture_mode_ = false;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = true;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  resetEnergyControlV0OutputGate();
  energy_control_v0_pulse_authorized_ = false;
  q_ident_run_schedule_id_ = 0;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;  // diagnostic only.
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG;
  // The frozen validated 300 mA rise/decay model is used only for this V0
  // pulse solver.  The stored UI settings remain zero until a pulse is issued.
  zero_cross_fixed_current_mA_ = Config::Q_IDENT_CURRENT_MA;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = Config::ENERGY_CONTROL_V0_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::ENERGY_CONTROL_V0_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::setEnergyControlAutonomousTarget(float target_deg) {
  if (running()) { status_.last_error = "energy_target_while_running"; return false; }
  bool selected = false;
  for (float allowed : Config::ENERGY_CONTROL_AUTONOMOUS_TARGET_CHOICES_DEG) {
    if (fabsf(target_deg - allowed) <= 0.001f) { selected = true; break; }
  }
  if (!selected) { status_.last_error = "energy_target_not_selectable"; return false; }
  if(target_deg!=energy_control_autonomous_target_peak_deg_)negative_cap_recovery_.interrupt();
  energy_control_autonomous_target_peak_deg_ = target_deg;
  status_.last_error = "";
  return true;
}

bool ExperimentRunner::setEnergyControlAutonomousInputPeakPercent(float peak_percent) {
  if (running() || !autonomous_input_percent::valid(peak_percent)) {
    status_.last_error = "input_percent_invalid_or_running"; return false;
  }
  energy_control_autonomous_input_peak_percent_ = peak_percent;
  status_.last_error = ""; return true;
}
bool ExperimentRunner::setEnergyControlAutonomousInputCurrentMa(int16_t current_mA) {
  if (running() || !autonomous_input_current::valid(current_mA)) {
    status_.last_error = "input_current_invalid_or_running"; return false;
  }
  energy_control_autonomous_input_current_mA_ = current_mA;
  status_.last_error = ""; return true;
}
bool ExperimentRunner::setEnergyControlAutonomousBrakeMaxMs(uint16_t brake_max_ms) {
  if (running() || !autonomous_brake_limit::valid(brake_max_ms)) {
    status_.last_error = "brake_max_invalid_or_running"; return false;
  }
  if(brake_max_ms!=energy_control_autonomous_brake_max_ms_)negative_cap_recovery_.interrupt();
  energy_control_autonomous_brake_max_ms_ = brake_max_ms;
  status_.last_error = ""; return true;
}

bool ExperimentRunner::setPeakAdaptationEnabled(bool enabled) {
  if (running()) { status_.last_error = "adaptation_setting_locked_while_running"; return false; }
  peak_adaptation_enabled_ = enabled;
  peak_error_model_.reset();
  status_.last_error = "";
  return true;
}
bool ExperimentRunner::setEnergyBridgeEnabled(bool enabled) {
  if (running()) { status_.last_error = "energy_model_setting_locked_while_running"; return false; }
  if(enabled!=energy_bridge_enabled_)negative_cap_recovery_.interrupt();
  energy_bridge_enabled_ = enabled;
  peak_error_model_.reset();
  status_.last_error = "";
  return true;
}
bool ExperimentRunner::setNegativeProbeMode(bool enabled) {
  if (running()) { status_.last_error = "negative_probe_locked_while_running"; return false; }
  if(enabled!=negative_probe_mode_)negative_cap_recovery_.interrupt();
  negative_probe_mode_ = enabled;
  status_.last_error = "";
  return true;
}
const char* ExperimentRunner::energyControlAutonomousPhaseName() const {
  switch (energy_control_autonomous_phase_) {
    case EnergyControlAutonomousPhase::STRONG_START_KICK: return "STRONG_START_KICK";
    case EnergyControlAutonomousPhase::WAIT_FIRST_PEAK: return "WAIT_FIRST_PEAK";
    case EnergyControlAutonomousPhase::ENERGY_CONTROL: return "ENERGY_CONTROL";
    case EnergyControlAutonomousPhase::HOLD: return "HOLD";
    case EnergyControlAutonomousPhase::STOP: return "STOP";
    default: return "IDLE";
  }
}

bool ExperimentRunner::startEnergyControlAutonomousCapture() {
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) { status_.last_error = "psram_not_ready"; return false; }
  if (!imu_->ok()) { status_.last_error = "imu_not_ready"; return false; }
  if (wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f &&
      energy_control_autonomous_input_current_mA_>Config::WALKING_TARGET8_MAX_CURRENT_MA) {
    status_.last_error = "walking_current_exceeds_1200mA"; return false;
  }
  // ESTOP must be cleared through the existing explicit reset path first.
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED) {
    status_.last_error = "not_ready";
    return false;
  }

  // New path: it does not inherit V0's manual-release gate, Q_IDENT schedule,
  // E2 shadow tracker, or legacy scheduler.
  passive_capture_mode_ = false;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  const ImuReading& autonomous_start_reading = imu_->reading();
  if ((!wheelWorkMode() && !upright_reference::valid(upright_reference_)) || upright_reference_capture_.pending()) {
    status_.last_error = "upright_reference_required"; return false;
  }
  if (!isUprightStableSample(autonomous_start_reading)) {
    status_.last_error = "upright_pose_required_before_start";
    return false;
  }
  if ((negative_probe_mode_ && (!energy_bridge_enabled_ ||
       energy_control_autonomous_target_peak_deg_ != 8.0f ||
       energy_control_autonomous_input_current_mA_ != 300 || peak_adaptation_enabled_ ||
       !(autonomous_brake_limit::probe(energy_control_autonomous_brake_max_ms_) ||
         autonomous_brake_limit::batch(energy_control_autonomous_brake_max_ms_) ||
         autonomous_brake_limit::extendedBatch(energy_control_autonomous_brake_max_ms_) ||
         autonomous_brake_limit::positiveCurrentBatch(energy_control_autonomous_brake_max_ms_)))) ||
      (!negative_probe_mode_ && energy_bridge_enabled_ && energy_control_autonomous_brake_max_ms_ != 0 &&
       !((energy_control_autonomous_brake_max_ms_ == 25 ||
          energy_control_autonomous_brake_max_ms_ == 40) &&
         work_target_policy::packageAllowed(energy_control_autonomous_target_peak_deg_,
             energy_control_autonomous_brake_max_ms_,energy_control_autonomous_input_current_mA_,peak_adaptation_enabled_)))) {
    status_.last_error = "negative_probe_settings_invalid"; return false;
  }
  g_v46_mekf_run_reinit = V46MekfRunReinitAccumulator{};
  g_v46_mekf_run_reinit.active = true;
  energy_control_autonomous_mode_ = true;
  energy_control_autonomous_pulse_authorized_ = false;
  // V46 records only the adopted hold-073 dynamic-beta Madgwick alongside MEKF.
  // Clear stale legacy comparison-series values before the very first START_SYNC
  // row so every logged sample is unambiguous from the beginning of the capture.
  status_.pitch_madgwick_beta1_raw_deg = NAN;
  status_.pitch_madgwick_beta1_bias_deg = NAN;
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    if (i == Config::FILTER_ADOPTED_INDEX) continue;
    status_.pitch_dynamic_beta_deg[i] = NAN;
    status_.beta_target_series[i] = NAN;
    status_.beta_smooth_series[i] = NAN;
  }
  resetEnergyControlAutonomous();
  lateral_target_energy_j_ = lateral_energy::potential(lateral_energy_geometry_, energy_control_autonomous_target_peak_deg_);
  if (!std::isfinite(lateral_target_energy_j_) ||
      !lateral_current_table_.prepare(predictRiseTauS(energy_control_autonomous_input_current_mA_)) ||
      ((positiveCurrentBatchMode() || wheelWorkMode()) &&
       (!positive_current_probe_tables_[0].prepare(predictRiseTauS(350)) ||
        !positive_current_probe_tables_[1].prepare(predictRiseTauS(400))))) {
    g_v46_mekf_run_reinit.active = false;
    energy_control_autonomous_mode_ = false; status_.last_error = "lateral_energy_model_invalid"; return false;
  }
  q_ident_run_schedule_id_ = 0;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = energy_control_autonomous_target_peak_deg_;
  // Autonomous current is intentionally independent of Q_IDENT scheduling.
  zero_cross_fixed_current_mA_ = energy_control_autonomous_input_current_mA_;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  negative_probe_batch_seed_ = anyProbeBatchMode()
      ? (static_cast<uint32_t>(micros()) ^
         (0x9e3779b9U * static_cast<uint32_t>(status_.run_id))) : 0;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  predicted_signed_current_end_mA_ = 0.0f;
  predicted_current_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = measurementTotalDurationMs();
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
bool ExperimentRunner::startQIdentCapture(uint8_t q_ident_run_schedule_id) {
  (void)q_ident_run_schedule_id;
  q_ident_mode_ = false;
  q_ident_pulse_authorized_ = false;
  energy_control_v0_mode_ = false;
  energy_control_autonomous_mode_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  stopMotor();
  status_.last_error = "q_ident_frozen_use_energy_control_v0";
  return false;
  if (!logger_ || !imu_ || !roller_) return false;
  if (!logger_->ready()) {
    status_.last_error = "psram_not_ready";
    return false;
  }
  if (!imu_->ok()) {
    status_.last_error = "imu_not_ready";
    return false;
  }
  if (q_ident_run_schedule_id >= Config::Q_IDENT_SCHEDULE_COUNT) {
    status_.last_error = "q_ident_schedule_invalid";
    return false;
  }
  if (status_.state != ExperimentState::READY_TO_MEASURE && status_.state != ExperimentState::FINISHED &&
      status_.state != ExperimentState::ESTOP) {
    status_.last_error = "not_ready";
    return false;
  }

  // This is the only start function allowed to enable actual fixed-Q output.
  passive_capture_mode_ = false;
  q_ident_mode_ = true;
  q_ident_pulse_authorized_ = false;
  q_ident_run_schedule_id_ = q_ident_run_schedule_id;
  resetQIdentTracker();
  q1_shadow_run_target_peak_abs_deg_ = q1_shadow_target_peak_abs_deg_;  // diagnostic only.
  resetQ1ShadowZeroCrossTracker();
  zero_cross_mode_ = false;
  single_trial_mode_ = true;
  selected_trial_index_ = 0;
  identification_mode_ = false;
  q_run_mode_ = QRunMode::NONE;
  q_probe_schedule_id_ = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A;
  control_target_peak_deg_ = 0.0f;
  // The validated 300 mA current/pulse model is used only by qIdentSolvePulse.
  zero_cross_fixed_current_mA_ = Config::Q_IDENT_CURRENT_MA;
  zero_cross_fixed_pulse_ms_ = 0;
  stopMotor();
  status_.run_id++;
  if (status_.run_id == 0) status_.run_id = 1;
  last_log_us_ = 0;
  last_pulse_end_ms_ = 0;
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  status_.motor_cmd_mA = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = Config::Q_IDENT_DURATION_MS;
  status_.trial_index = 1;
  status_.trial_count = 1;
  status_.trial_elapsed_ms = 0;
  status_.trial_duration_ms = Config::Q_IDENT_DURATION_MS;
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  status_.input_interval_ms = 0;
  status_.beta_hold_after_input_ms_setting = 0;
  status_.beta_recovery_tau_s_setting = 0.0f;
  updatePulseModelPrediction();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.sync_event_id = 0;
  beginStartSync(millis());
  return true;
}
const char* ExperimentRunner::qRunModeName() const {
  switch (q_run_mode_) {
    case QRunMode::VALIDATION: return "validation";
    case QRunMode::CONTROL: return "control";
    default: return "none";
  }
}

const char* ExperimentRunner::qProbeScheduleName() const {
  switch (q_probe_schedule_id_) {
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_A: return "A";
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_B: return "B";
    case Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_C: return "C";
    default: return "invalid";
  }
}

uint8_t ExperimentRunner::calibrationProbeQLevelIndex(uint8_t plan_index) const {
  if (q_probe_schedule_id_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SCHEDULE_COUNT ||
      plan_index >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
    return 0;
  }
  return Config::ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_INDEX_BY_SCHEDULE[
      q_probe_schedule_id_][plan_index];
}

uint8_t ExperimentRunner::classifyV59StateGate(float hprev_deg, float cprev_deg,
                                                 float abs_rate_dps, int8_t next_peak_side,
                                                 bool dynamic_h_in_support) const {
  if (!Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED) {
    return Config::ZERO_CROSS_V59_GATE_REASON_DISABLED;
  }
  const bool state_valid = calibration_last_half_range_valid_ &&
      calibration_last_center_dynamic_valid_ && isfinite(hprev_deg) && isfinite(cprev_deg) &&
      isfinite(abs_rate_dps) && (next_peak_side == -1 || next_peak_side == 1);
  if (!state_valid) return Config::ZERO_CROSS_V59_GATE_REASON_STATE_INVALID;
  if (!dynamic_h_in_support) return Config::ZERO_CROSS_V59_GATE_REASON_DYNAMIC_H_OUT_OF_SUPPORT;
  if (hprev_deg < Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_HPREV_BELOW;
  if (hprev_deg > Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_HPREV_ABOVE;
  if (cprev_deg < Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_CPREV_BELOW;
  if (cprev_deg > Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG) return Config::ZERO_CROSS_V59_GATE_REASON_CPREV_ABOVE;
  if (abs_rate_dps < Config::ZERO_CROSS_V59_GATE_ABS_RATE_MIN_DPS) return Config::ZERO_CROSS_V59_GATE_REASON_RATE_BELOW;
  if (abs_rate_dps > Config::ZERO_CROSS_V59_GATE_ABS_RATE_MAX_DPS) return Config::ZERO_CROSS_V59_GATE_REASON_RATE_ABOVE;
  if (next_peak_side != Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE) return Config::ZERO_CROSS_V59_GATE_REASON_DIRECTION_MISMATCH;
  return Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
}

void ExperimentRunner::recordV59StateGateEvent(uint32_t crossing_ms, float hprev_deg,
                                                 float cprev_deg, float abs_rate_dps,
                                                 int8_t next_peak_side, bool state_valid,
                                                 bool dynamic_h_in_support, uint8_t reason,
                                                 uint8_t action,
                                                 bool v60_cooldown_free_decay,
                                                 bool v60_gate_skipped_by_decay) {
  if (!logger_) return;
  PsramLogger::CalibrationStateGateEvent event;
  event.planned_probe_index = calibration_probe_plan_index_ + 1;
  event.q_level_index = calibrationProbeQLevelIndex(calibration_probe_plan_index_);
  event.reason = reason;
  event.action = action;
  event.wait_halfcycle_count = calibration_probe_wait_halfcycle_count_;
  event.rebuild_total_count = calibration_rebuild_total_count_;
  event.crossing_ms = crossing_ms;
  event.hprev_cdeg = state_valid ? centi(hprev_deg) : LOG_NAN_I16;
  event.cprev_cdeg = state_valid ? centi(cprev_deg) : LOG_NAN_I16;
  event.rate_abs_cdps = state_valid ? centi(abs_rate_dps) : LOG_NAN_I16;
  event.next_peak_side = next_peak_side;
  event.state_valid = state_valid;
  event.dynamic_h_in_support = dynamic_h_in_support;
  event.v60_cooldown_free_decay = v60_cooldown_free_decay;
  event.v60_gate_skipped_by_decay = v60_gate_skipped_by_decay;
  logger_->addCalibrationStateGateEvent(event);
}

void ExperimentRunner::beginStartSync(uint32_t now_ms) {
  stall_rescue_detector_.reset();stall_rescue_attempted_peak_ms_=0;stall_rescue_event_=false;
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
  approach_stall_rescue_detector_.reset();approach_stall_rescue_event_=false;
#endif
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
  early_stall_rescue_detector_.reset();early_stall_rescue_event_=false;
#endif
  optional_early_event_=false;optional_early_write_uncertain_=false;optional_early_attempted_peak_ms_=0;
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  foot_pose_=foot_phase_prediction::Projection{};foot_pose_sequence_=0;foot_support_last_check_ms_=0;
  foot_support_event_=false;foot_support_active_audit_=-1;
  foot_support_decision_=foot_support_policy::Decision{};
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  foot_local_pose_=foot_local_prediction::Projection{};foot_support_source_=foot_support_pose::Source::Unknown;
  foot_support_input_=foot_support_policy::Input{};
#endif
  foot_support_deceleration_.reset();
  foot_support_audit_count_=foot_support_audit_total_=0;
  for(auto& count:foot_support_reason_counts_)count=0;
#endif
#if FOOT_EARLY_TRIAL
  early_motion_gate_.clear();early_measurement_epoch_us_=0;
#if ANGLE_EARLY_TRIAL
  angle_late_blocked_peak_ms_=0;angle_last_check_ms_=0;
#endif
#endif
  // Amplitude only: relative heading is a diagnostic, never a target.
  energy_control_autonomous_pending_target_deg_ = NAN;
  status_.steering = steering::disabledSnapshot();
  gyro_heading_start_pending_ = energy_control_autonomous_mode_;
  tilt_guard_.reset();
  // V46z comparison-zero begin
  captureMekfComparisonZero(status_.mekf_start_sync_zero_abs_deg,
                            status_.mekf_start_sync_zero_sample_us);
  status_.mekf_measurement_zero_abs_deg = energy_control_autonomous_mode_ ? control_angle_reference::origin(wheelWorkMode(), upright_reference_.pitch_deg) : NAN;
  status_.mekf_trial_zero_abs_deg = energy_control_autonomous_mode_ ? control_angle_reference::origin(wheelWorkMode(), upright_reference_.pitch_deg) : NAN;
  status_.mekf_measurement_zero_sample_us = 0;
  status_.mekf_trial_zero_sample_us = 0;
  updateMekfComparisonRelativeAngles();
  // V46z comparison-zero end
  run_cg_shift_mm_ = wheelWorkMode() && lateral_cg::matchesAbsolute(lateral_cg_) ? lateral_cg_.shift_mm : 0.f;
  if (wheelWorkMode()) {
    adaptive_cg_.reset(run_cg_shift_mm_,0.f);
    run_cg_shift_mm_=adaptive_cg_.snapshot().shift_mm;
  }
  run_start_us_ = micros();
  logger_->startRun(status_.run_id, run_start_us_,
                    energy_control_autonomous_mode_ ? energy_control_autonomous_input_current_mA_ : status_.current_mA_setting,
                    status_.pulse_width_ms_setting,
                    status_.input_interval_ms, identification_mode_,
                    static_cast<uint8_t>(q_run_mode_), centi(control_target_peak_deg_), q_probe_schedule_id_,
                    passive_capture_mode_, q1_shadow_run_target_peak_abs_deg_, q_ident_mode_,
                    q_ident_mode_ ? static_cast<uint8_t>(q_ident_run_schedule_id_ + 1) : 0,
                     energy_control_v0_mode_, energy_control_autonomous_mode_,
                     energy_control_autonomous_input_peak_percent_,
                      energy_control_autonomous_brake_max_ms_, peak_adaptation_enabled_,
                      energy_bridge_enabled_, negative_probe_mode_,
                      energy_control_autonomous_mode_ ? upright_reference_ : upright_reference::Record{},
                      negative_probe_batch_seed_);
  logger_->recordLateralCg(wheelWorkMode() ? lateral_cg_ : lateral_cg::Record{});
  logger_->recordAdaptiveCg(wheelWorkMode() ? adaptive_cg_.snapshot() : lateral_cg::AdaptiveSnapshot{});
  last_log_us_ = 0;
  status_.state = ExperimentState::START_SYNC;
  status_.sync_event_id = 1;
  sync_step_ = 0;
  sync_step_start_ms_ = now_ms;
  sync_led_until_ms_ = 0;
  setSyncLed(Config::START_SYNC_PATTERN[0].led_on);
  logSampleNow();
}

void ExperimentRunner::beginMeasurementRun() {
  if (!passive_capture_mode_ && !q_ident_mode_ && !energy_control_v0_mode_ && !energy_control_autonomous_mode_) {
    captureAngleOffsets();
    updateDisplayedAngles(imu_->reading());
  }
  // V46z comparison-zero begin
  if (energy_control_autonomous_mode_) {
    // Work control keeps absolute zero. Saved pose and actual start attitude
    // are diagnostics and never redefine its control coordinate.
    logger_->recordMeasurementStartAngle(raw_mekf_pitch_abs_deg_);
  } else {
    captureMekfComparisonZero(status_.mekf_measurement_zero_abs_deg,
                             status_.mekf_measurement_zero_sample_us);
  }
  const bool dedicated_single_trial =
      passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_;
  if (dedicated_single_trial) {
    status_.mekf_trial_zero_abs_deg = status_.mekf_measurement_zero_abs_deg;
    status_.mekf_trial_zero_sample_us = status_.mekf_measurement_zero_sample_us;
  } else {
    status_.mekf_trial_zero_abs_deg = NAN;
    status_.mekf_trial_zero_sample_us = 0;
  }
  updateMekfComparisonRelativeAngles();
  // V46z comparison-zero end
  // V46aa control-zero capture begin
  if (energy_control_autonomous_mode_) {
    status_.mekf_detector_zero_predicted_abs_deg = control_angle_reference::origin(wheelWorkMode(), upright_reference_.pitch_deg);
    status_.mekf_detector_zero_sample_us = 0; // Saved calibration, not a new sample origin.
    status_.pitch_mekf_detector_relative_deg = status_.pitch_mekf_measurement_relative_deg;
    status_.pitch_mekf_deg = status_.pitch_mekf_detector_relative_deg;
  } else {
    status_.mekf_detector_zero_predicted_abs_deg = raw_mekf_predicted_abs_deg_;
    status_.mekf_detector_zero_sample_us = imu_ ? imu_->reading().last_gyro_update_us : 0;
    status_.pitch_mekf_detector_relative_deg =
        isfinite(status_.mekf_detector_zero_predicted_abs_deg) ? 0.0f : NAN;
  }
  // V46aa control-zero capture end
  run_start_ms_ = millis();
  if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) resetQ1ShadowZeroCrossTracker();
  if (energy_control_v0_mode_) resetEnergyControlV0OutputGate();
  if (energy_control_autonomous_mode_) resetEnergyControlAutonomous();
  if (q_ident_mode_) resetQIdentTracker();
  next_pulse_start_test_ms_ = 0;
  active_pulse_start_ms_ = 0;
  active_pulse_start_test_ms_ = 0;
  last_pulse_end_ms_ = 0;
  next_pulse_direction_ = 1;
  sync_led_until_ms_ = 0;
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.running = true;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.state = ExperimentState::RUNNING_BATCH_SWEEP;
  status_.sync_event_id = 2;
  setSyncLed(false);
  // Select the configured comparison beta before emitting the t_test=0 row.
  // Autonomous v46 keeps only the adopted hold-073 dynamic-beta Madgwick live;
  // other historical series are explicitly NaN so they cannot be mistaken for
  // synchronized comparison data. Legacy non-autonomous modes retain old setup.
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    if (energy_control_autonomous_mode_ && i != Config::FILTER_ADOPTED_INDEX) {
      status_.beta_target_series[i] = NAN;
      status_.beta_smooth_series[i] = NAN;
      status_.pitch_dynamic_beta_deg[i] = NAN;
      continue;
    }
    beta_smooth_[i] = betaCeilingForStrategy(i);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
    if (!energy_control_autonomous_mode_) filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
  }
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  updateDisplayedAngles(imu_->reading());
  logSampleNow();
  if (passive_capture_mode_ || q_ident_mode_ || energy_control_v0_mode_ || energy_control_autonomous_mode_) {
    // Dedicated capture modes skip beginTrial(), isolating legacy scheduling.
    status_.trial_index = 1;
    status_.trial_count = 1;
    status_.trial_elapsed_ms = 0;
    status_.trial_duration_ms = energy_control_autonomous_mode_ ? measurementTotalDurationMs() :
        (energy_control_v0_mode_ ? Config::ENERGY_CONTROL_V0_DURATION_MS :
        (q_ident_mode_ ? Config::Q_IDENT_DURATION_MS : Config::PASSIVE_CAPTURE_DURATION_MS));
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
    status_.input_interval_ms = 0;
    status_.motor_cmd_mA = 0;
    trial_start_ms_ = run_start_ms_;
    stopMotor();
    if (energy_control_autonomous_mode_) beginEnergyControlAutonomousStartKick(millis());
    return;
  }
  beginTrial(single_trial_mode_ ? selected_trial_index_ : 0);
}

void ExperimentRunner::beginTrial(uint8_t trial_index) {
  if (!zero_cross_mode_ && trial_index >= Config::BETA_SWEEP_TRIAL_COUNT) {
    finishRun();
    return;
  }
  stopMotor();
  status_.state = ExperimentState::RUNNING_BATCH_SWEEP;
  status_.trial_elapsed_ms = 0;
  if (zero_cross_mode_) {
    status_.trial_index = 1;
    status_.trial_count = 1;
    status_.trial_duration_ms = Config::ZERO_CROSS_TEST_DURATION_MS;
    status_.current_mA_setting = zero_cross_fixed_current_mA_;
    status_.pulse_width_ms_setting = zero_cross_fixed_pulse_ms_;
    status_.input_interval_ms = 0;  // Event-driven zero-cross mode.
  } else {
    const auto& trial = Config::BETA_SWEEP_TRIALS[trial_index];
    status_.trial_index = trial_index + 1;
    status_.trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
    status_.trial_duration_ms = trial.duration_ms;
    status_.current_mA_setting = trial.current_mA;
    status_.pulse_width_ms_setting = trial.pulse_width_ms;
    status_.input_interval_ms = trial.input_interval_ms;
  }
  status_.beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  status_.beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  updatePulseModelPrediction();
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    beta_smooth_[i] = betaCeilingForStrategy(i);
    status_.beta_target_series[i] = beta_smooth_[i];
    status_.beta_smooth_series[i] = beta_smooth_[i];
    filter_dynamic_raw_[i].setBeta(beta_smooth_[i]);
    filter_dynamic_bias_[i].setBeta(beta_smooth_[i]);
  }
  status_.beta_target = status_.beta_target_series[Config::FILTER_ADOPTED_INDEX];
  status_.beta_smooth = status_.beta_smooth_series[Config::FILTER_ADOPTED_INDEX];
  trial_start_ms_ = millis();
  active_pulse_start_ms_ = 0;
  active_pulse_start_test_ms_ = 0;
  last_pulse_end_ms_ = 0;
  last_zero_cross_pulse_start_ms_ = 0;
  zero_cross_start_refractory_until_ms_ = 0;
  zero_cross_half_cycle_peak_abs_deg_ = 0.0f;
  zero_cross_next_direction_ = -Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
  next_pulse_start_test_ms_ = 0;
  next_pulse_direction_ = 1;
  zero_cross_bootstrap_pending_ = zero_cross_mode_;
  zero_cross_armed_ = false;
  zero_cross_has_previous_angle_ = false;
  zero_cross_previous_angle_deg_ = 0.0f;
  resetCalibrationShadow();
  beta_phase_.reset();
  beta_turn_fast_.reset();
  status_.pulse_id = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  captureAngleOffsets();
  // V46z comparison-zero begin
  captureMekfComparisonZero(status_.mekf_trial_zero_abs_deg,
                            status_.mekf_trial_zero_sample_us);
  // V46z comparison-zero end
  updateDisplayedAngles(imu_->reading());
  status_.sync_event_id = 6;
  logSampleNow();
}

void ExperimentRunner::zeroAngleNow() {
  if (passive_capture_mode_) {
    status_.last_error = "zero_disabled_in_passive_mode";
    return;
  }
  captureAngleOffsets();
  if (imu_) updateDisplayedAngles(imu_->reading());
}

bool ExperimentRunner::zeroCurrentRollDisplay() {
  if (running()) {
    status_.last_error = "current_roll_zero_while_running";
    return false;
  }
  if (!status_.static_confirmed) {
    status_.last_error = "current_roll_zero_requires_static";
    return false;
  }
  display_zero_offset_deg_ = status_.physical_roll_abs_deg;
  status_.display_zero_offset_deg = display_zero_offset_deg_;
  status_.current_roll_deg = 0.0f;
  status_.target_error_deg = -target_roll_deg_;
  status_.ready = fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
  status_.last_error = "";
  return true;
}

bool ExperimentRunner::setCurrentRollTarget(float target_deg) {
  if (!isfinite(target_deg) || target_deg < Config::CURRENT_ROLL_TARGET_MIN_DEG ||
      target_deg > Config::CURRENT_ROLL_TARGET_MAX_DEG) {
    status_.last_error = "current_roll_target_out_of_range";
    return false;
  }
  target_roll_deg_ = target_deg;
  status_.target_roll_deg = target_roll_deg_;
  status_.target_error_deg = status_.current_roll_deg - target_roll_deg_;
  status_.ready = status_.static_confirmed &&
      fabsf(status_.target_error_deg) <= Config::TARGET_TOLERANCE_DEG;
  status_.last_error = "";
  return true;
}

bool ExperimentRunner::setQ1ShadowTargetPeakAbs(float target_deg) {
  if (running()) {
    status_.last_error = "q1_shadow_target_while_running";
    return false;
  }
  if (!isfinite(target_deg) || target_deg < Config::Q1_SHADOW_TARGET_MIN_DEG ||
      target_deg > Config::Q1_SHADOW_TARGET_MAX_DEG) {
    status_.last_error = "q1_shadow_target_out_of_range";
    return false;
  }
  q1_shadow_target_peak_abs_deg_ = target_deg;
  status_.last_error = "";
  return true;
}

void ExperimentRunner::resetE2ShadowPeakTracker() {
  e2_shadow_armed_ = false;
  e2_shadow_release_detected_ms_ = 0;
  e2_shadow_gyro_integrator_ready_ = false;
  e2_shadow_last_gyro_sample_us_ = 0;
  e2_shadow_last_gyro_rate_dps_ = 0.0f;
  e2_shadow_gyro_relative_deg_ = 0.0f;
  e2_shadow_pending_static_anchor_abs_deg_ = status_.static_confirmed &&
      isfinite(status_.physical_roll_abs_deg) ? status_.physical_roll_abs_deg : NAN;
  e2_shadow_static_anchor_abs_deg_ = NAN;
  e2_shadow_candidate_motion_sign_ = 0;
  e2_shadow_reverse_samples_ = 0;
  e2_shadow_candidate_gyro_relative_deg_ = 0.0f;
  e2_shadow_candidate_accel_abs_diag_deg_ = NAN;
  e2_shadow_candidate_peak_ms_ = 0;
  e2_shadow_last_peak_valid_ = false;
  e2_shadow_last_turn_side_ = 0;
  e2_shadow_last_peak_gyro_relative_deg_ = NAN;
  e2_shadow_turn_index_ = 0;
}

void ExperimentRunner::updateE2ShadowPeakTracker(uint32_t now_ms) {
  if (!logger_ || !imu_) return;
  const float accel_abs_diag_deg = status_.physical_roll_abs_deg;
  const float rate_dps = status_.physical_roll_rate_dps;
  if (!isfinite(rate_dps)) return;
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  const uint32_t sample_us = imu_->reading().last_update_us;

  // Retain only a static acceleration angle as the absolute-angle anchor. It
  // is a video-comparison diagnostic; it never contributes to H or Q validity.
  if (!e2_shadow_armed_ && status_.static_confirmed && isfinite(accel_abs_diag_deg)) {
    e2_shadow_pending_static_anchor_abs_deg_ = accel_abs_diag_deg;
    e2_shadow_gyro_relative_deg_ = 0.0f;
    e2_shadow_gyro_integrator_ready_ = true;
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  } else if (!e2_shadow_gyro_integrator_ready_) {
    e2_shadow_gyro_integrator_ready_ = true;
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  } else {
    const uint32_t dt_us = sample_us - e2_shadow_last_gyro_sample_us_;
    if (dt_us > 0 && dt_us <= Config::E2_SHADOW_GYRO_INTEGRATION_MAX_DT_US) {
      e2_shadow_gyro_relative_deg_ += 0.5f *
          (e2_shadow_last_gyro_rate_dps_ + rate_dps) * (static_cast<float>(dt_us) / 1000000.0f);
    }
    e2_shadow_last_gyro_sample_us_ = sample_us;
    e2_shadow_last_gyro_rate_dps_ = rate_dps;
  }

  if (!e2_shadow_armed_) {
    if (fabsf(rate_dps) < Config::E2_SHADOW_RELEASE_ARM_RATE_THRESHOLD_DPS) return;
    e2_shadow_armed_ = true;
    e2_shadow_release_detected_ms_ = t_test_ms;
    e2_shadow_static_anchor_abs_deg_ = e2_shadow_pending_static_anchor_abs_deg_;
    // Arm clears all pre-release peak history. The first accepted turn is a
    // baseline only; the first H is formed by the following opposite-side turn.
    e2_shadow_candidate_motion_sign_ = rate_dps > 0.0f ? 1 : -1;
    e2_shadow_reverse_samples_ = 0;
    e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
    e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
    e2_shadow_candidate_peak_ms_ = t_test_ms;
    e2_shadow_last_peak_valid_ = false;
    e2_shadow_last_turn_side_ = 0;
    e2_shadow_last_peak_gyro_relative_deg_ = NAN;
    e2_shadow_turn_index_ = 0;
    return;
  }

  const int8_t motion_sign = rate_dps >= Config::E2_SHADOW_PEAK_RATE_THRESHOLD_DPS ? 1 :
      (rate_dps <= -Config::E2_SHADOW_PEAK_RATE_THRESHOLD_DPS ? -1 : 0);
  if (motion_sign == 0) return;
  if (motion_sign == e2_shadow_candidate_motion_sign_) {
    e2_shadow_reverse_samples_ = 0;
    // Candidate time and gyro integral are frozen at the final sample before
    // the rate reversal; confirmation samples never enter the half-cycle H.
    e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
    e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
    e2_shadow_candidate_peak_ms_ = t_test_ms;
    return;
  }

  if (++e2_shadow_reverse_samples_ < Config::E2_SHADOW_PEAK_CONFIRM_SAMPLES) return;
  recordE2ShadowPeak(e2_shadow_candidate_peak_ms_, t_test_ms,
                     e2_shadow_candidate_motion_sign_,
                     e2_shadow_candidate_gyro_relative_deg_,
                     e2_shadow_candidate_accel_abs_diag_deg_);
  e2_shadow_candidate_motion_sign_ = motion_sign;
  e2_shadow_candidate_gyro_relative_deg_ = e2_shadow_gyro_relative_deg_;
  e2_shadow_candidate_accel_abs_diag_deg_ = accel_abs_diag_deg;
  e2_shadow_candidate_peak_ms_ = t_test_ms;
  e2_shadow_reverse_samples_ = 0;
}

void ExperimentRunner::recordE2ShadowPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                                            int8_t turn_side_from_rate,
                                            float candidate_gyro_relative_deg,
                                            float candidate_accel_abs_diag_deg) {
  if (!logger_) return;
  PsramLogger::E2ShadowPeakEvent event;
  event.e2_armed = e2_shadow_armed_;
  event.release_detected_ms = e2_shadow_release_detected_ms_;
  event.turn_index = ++e2_shadow_turn_index_;
  event.candidate_peak_ms = candidate_peak_ms;
  event.confirmed_ms = confirmed_ms;
  event.turn_side_from_rate = turn_side_from_rate;
  event.physical_next_peak_side = turn_side_from_rate == 0 ? 0 : -turn_side_from_rate;
  event.static_anchor_abs_deg = e2_shadow_static_anchor_abs_deg_;
  event.peak_accel_abs_diag_deg = candidate_accel_abs_diag_deg;
  event.peak_gyro_integrated_abs_deg = isfinite(e2_shadow_static_anchor_abs_deg_) &&
      isfinite(candidate_gyro_relative_deg)
      ? e2_shadow_static_anchor_abs_deg_ + candidate_gyro_relative_deg : NAN;
  event.previous_peak_deg = e2_shadow_last_peak_valid_ && isfinite(e2_shadow_static_anchor_abs_deg_)
      ? e2_shadow_static_anchor_abs_deg_ + e2_shadow_last_peak_gyro_relative_deg_ : NAN;
  event.current_peak_deg = event.peak_gyro_integrated_abs_deg;
  event.target_peak_deg = NAN;
  event.alternating_side_ok = e2_shadow_last_peak_valid_ &&
      turn_side_from_rate != 0 && turn_side_from_rate != e2_shadow_last_turn_side_;

  uint8_t reason = Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS;
  if (!e2_shadow_last_peak_valid_) {
    // Establish the first post-release turn. No half-cycle H exists yet.
    reason = Config::E2_SHADOW_INVALID_INSUFFICIENT_TURNS;
  } else if (!event.alternating_side_ok) {
    // Never bridge a chatter/nonalternating turn into a later H interval.
    reason = Config::E2_SHADOW_INVALID_NONALTERNATING_TURN;
  } else {
    event.halfcycle_gyro_integral_deg = candidate_gyro_relative_deg -
        e2_shadow_last_peak_gyro_relative_deg_;
    event.h_prev_gyro_deg = 0.5f * fabsf(event.halfcycle_gyro_integral_deg);
    event.h_prev_deg = event.h_prev_gyro_deg;  // legacy H_prev alias
    if (!isfinite(event.h_prev_gyro_deg) ||
        event.h_prev_gyro_deg < Config::E2_SHADOW_H_PREV_MIN_DEG ||
        event.h_prev_gyro_deg > Config::E2_SHADOW_H_PREV_MAX_DEG) {
      reason = Config::E2_SHADOW_INVALID_H_OUT_OF_CALIBRATION_RANGE;
    } else {
      event.h_next_e2_deg = Config::E2_SHADOW_E2_SLOPE * event.h_prev_gyro_deg +
          Config::E2_SHADOW_E2_OFFSET_DEG;
      event.h_next_e2_explicit_deg = event.h_next_e2_deg;
      if (!isfinite(event.h_next_e2_deg) || event.h_next_e2_deg <= 0.0f) {
        reason = Config::E2_SHADOW_INVALID_NONFINITE_STATE;
      } else {
        // This revision validates E2 only in H-space. The static-anchor gyro
        // absolute peak is logged for video comparison, but it is not yet a
        // validated coordinate for passive absolute peak or inverse-Q logic.
        reason = Config::E2_SHADOW_INVALID_ABSOLUTE_PEAK_ESTIMATOR_UNVALIDATED;
      }
    }
  }
  event.shadow_valid = false;
  event.shadow_invalid_reason = reason;
  logger_->addE2ShadowPeakEvent(event);

  if (e2_shadow_last_peak_valid_ && !event.alternating_side_ok) {
    e2_shadow_last_peak_valid_ = false;
    e2_shadow_last_turn_side_ = 0;
    e2_shadow_last_peak_gyro_relative_deg_ = NAN;
    return;
  }
  e2_shadow_last_peak_valid_ = turn_side_from_rate != 0 && isfinite(candidate_gyro_relative_deg);
  e2_shadow_last_turn_side_ = turn_side_from_rate;
  e2_shadow_last_peak_gyro_relative_deg_ = candidate_gyro_relative_deg;
}
void ExperimentRunner::resetQ1ShadowZeroCrossTracker() {
  q1_shadow_has_previous_angle_ = false;
  q1_shadow_zero_cross_armed_ = false;
  q1_shadow_angle_zero_deg_ = status_.pitch_mekf_deg;
  if (!isfinite(q1_shadow_angle_zero_deg_)) q1_shadow_angle_zero_deg_ = 0.0f;
  q1_shadow_previous_relative_angle_deg_ = 0.0f;
  q1_shadow_previous_time_ms_ = 0;
  q1_shadow_previous_rate_dps_ = 0.0f;
  q1_shadow_last_zero_cross_ms_ = 0;
}

void ExperimentRunner::updateQ1ShadowAtZeroCross(uint32_t now_ms) {
  if (!logger_) return;
  // The continuous filter angle is only a detector coordinate. Q1 itself uses
  // +gy physical_roll_rate_dps at the crossing, not an IMU absolute angle.
  const float detector_angle_deg =
      status_.pitch_mekf_deg - q1_shadow_angle_zero_deg_;
  const float rate_dps = status_.physical_roll_rate_dps;
  if (!isfinite(detector_angle_deg) || !isfinite(rate_dps)) return;
  if (energy_control_v0_mode_) updateEnergyControlV0OutputGate(detector_angle_deg);
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (!q1_shadow_has_previous_angle_) {
    q1_shadow_has_previous_angle_ = true;
    q1_shadow_previous_relative_angle_deg_ = detector_angle_deg;
    q1_shadow_previous_time_ms_ = t_test_ms;
    q1_shadow_previous_rate_dps_ = rate_dps;
    return;
  }

  if (fabsf(detector_angle_deg) >= Config::Q1_SHADOW_ZERO_CROSS_REARM_ANGLE_DEG) {
    q1_shadow_zero_cross_armed_ = true;
  }
  const bool detector_neg_to_pos = q1_shadow_previous_relative_angle_deg_ < 0.0f &&
      detector_angle_deg >= 0.0f;
  const bool detector_pos_to_neg = q1_shadow_previous_relative_angle_deg_ > 0.0f &&
      detector_angle_deg <= 0.0f;
  // The adopted detector coordinate is anti-correlated with official +gy.
  // Keep the detector, start reference, rearm, and interval unchanged; only
  // this sign-consistency condition is reversed from the previous build.
  const bool sign_gate_passed =
      (detector_neg_to_pos && rate_dps <= -Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS) ||
      (detector_pos_to_neg && rate_dps >= Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS);
  const bool interval_ok = q1_shadow_last_zero_cross_ms_ == 0 ||
      t_test_ms - q1_shadow_last_zero_cross_ms_ >= Config::Q1_SHADOW_ZERO_CROSS_MIN_INTERVAL_MS;
  if (q1_shadow_zero_cross_armed_ && interval_ok && sign_gate_passed) {
    // The sign of +gy after the crossing is the physical side of the next peak.
    Q1ZeroCrossDiagnostics diagnostics;
    diagnostics.detector_crossing_direction = detector_neg_to_pos ? 1 : -1;
    diagnostics.detector_angle_before_deg = q1_shadow_previous_relative_angle_deg_;
    diagnostics.detector_angle_after_deg = detector_angle_deg;
    const float denominator = q1_shadow_previous_relative_angle_deg_ - detector_angle_deg;
    if (isfinite(denominator) && fabsf(denominator) > 1.0e-6f) {
      diagnostics.crossing_interpolation_alpha =
          q1_shadow_previous_relative_angle_deg_ / denominator;
      if (diagnostics.crossing_interpolation_alpha < 0.0f) {
        diagnostics.crossing_interpolation_alpha = 0.0f;
      } else if (diagnostics.crossing_interpolation_alpha > 1.0f) {
        diagnostics.crossing_interpolation_alpha = 1.0f;
      }
      const uint32_t interval_ms = t_test_ms - q1_shadow_previous_time_ms_;
      diagnostics.interpolated_zero_cross_time_ms = q1_shadow_previous_time_ms_ +
          static_cast<uint32_t>(lroundf(diagnostics.crossing_interpolation_alpha * interval_ms));
      diagnostics.interpolated_physical_roll_rate_dps = q1_shadow_previous_rate_dps_ +
          diagnostics.crossing_interpolation_alpha * (rate_dps - q1_shadow_previous_rate_dps_);
    }
    diagnostics.physical_roll_rate_before_dps = q1_shadow_previous_rate_dps_;
    diagnostics.physical_roll_rate_after_dps = rate_dps;
    diagnostics.sign_gate_passed = true;
    // Do not substitute interpolation diagnostics into the Q1 model yet.
    recordQ1ShadowZeroCross(t_test_ms, rate_dps, diagnostics);
    if (energy_control_v0_mode_) updateEnergyControlV0AtZeroCross(t_test_ms, rate_dps, diagnostics);
    if (q_ident_mode_) updateQIdentAtZeroCross(t_test_ms, rate_dps, diagnostics);
    q1_shadow_last_zero_cross_ms_ = t_test_ms;
    q1_shadow_zero_cross_armed_ = false;
  }
  q1_shadow_previous_relative_angle_deg_ = detector_angle_deg;
  q1_shadow_previous_time_ms_ = t_test_ms;
  q1_shadow_previous_rate_dps_ = rate_dps;
}

void ExperimentRunner::recordQ1ShadowZeroCross(
    uint32_t t_test_ms, float rate_dps, const Q1ZeroCrossDiagnostics& diagnostics) {
  if (!logger_) return;
  PsramLogger::Q1ShadowEvent event;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.detector_crossing_direction = diagnostics.detector_crossing_direction;
  event.detector_angle_before_deg = diagnostics.detector_angle_before_deg;
  event.detector_angle_after_deg = diagnostics.detector_angle_after_deg;
  event.crossing_interpolation_alpha = diagnostics.crossing_interpolation_alpha;
  event.interpolated_zero_cross_time_ms = diagnostics.interpolated_zero_cross_time_ms;
  event.physical_roll_rate_before_dps = diagnostics.physical_roll_rate_before_dps;
  event.physical_roll_rate_after_dps = diagnostics.physical_roll_rate_after_dps;
  event.interpolated_physical_roll_rate_dps = diagnostics.interpolated_physical_roll_rate_dps;
  event.sign_gate_passed = diagnostics.sign_gate_passed;
  event.q1_intercept_deg = Config::Q1_SHADOW_INTERCEPT_DEG;
  event.q1_rate_term_deg = Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS *
      event.zero_cross_abs_rate_dps;
  event.q1_side_term_deg = Config::Q1_SHADOW_SIDE_TERM_DEG *
      static_cast<float>(event.physical_next_peak_side);
  event.q1_baseline_next_peak_abs_deg = event.q1_intercept_deg +
      event.q1_rate_term_deg + event.q1_side_term_deg;
  event.target_next_peak_abs_deg = q1_shadow_run_target_peak_abs_deg_;
  event.q1_gain_deg_per_mA_s = event.physical_next_peak_side > 0
      ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS
      : Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;

  uint8_t reason = Config::Q1_SHADOW_INVALID_NONE;
  if (!isfinite(event.zero_cross_abs_rate_dps) || !isfinite(event.q1_baseline_next_peak_abs_deg) ||
      !isfinite(event.target_next_peak_abs_deg) || !isfinite(event.q1_gain_deg_per_mA_s) ||
      event.q1_gain_deg_per_mA_s <= 0.0f) {
    reason = Config::Q1_SHADOW_INVALID_NONFINITE_STATE;
  } else if (event.zero_cross_abs_rate_dps < Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS) {
    reason = Config::Q1_SHADOW_INVALID_RATE_BELOW_SUPPORT;
  } else if (event.zero_cross_abs_rate_dps > Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS) {
    reason = Config::Q1_SHADOW_INVALID_RATE_ABOVE_SUPPORT;
  } else {
    event.delta_peak_required_deg = event.target_next_peak_abs_deg -
        event.q1_baseline_next_peak_abs_deg;
    if (event.delta_peak_required_deg <= 0.0f) {
      reason = Config::Q1_SHADOW_INVALID_BRAKING_NOT_IDENTIFIED;
    } else {
      event.q_req_shadow_mA_s = event.delta_peak_required_deg /
          event.q1_gain_deg_per_mA_s;
      // Q_req uses the exact canonical q_target_mA_s coordinate; the same
      // value is logged under its model-axis name for independent replay.
      event.q_model_axis_mA_s = event.q_req_shadow_mA_s;
      if (!isfinite(event.q_req_shadow_mA_s)) {
        reason = Config::Q1_SHADOW_INVALID_NONFINITE_STATE;
      } else if (event.q_req_shadow_mA_s < Config::Q1_SHADOW_Q_SUPPORT_MIN_MAS) {
        reason = Config::Q1_SHADOW_INVALID_Q_BELOW_SUPPORT;
      } else if (event.q_req_shadow_mA_s > Config::Q1_SHADOW_Q_SUPPORT_MAX_MAS) {
        reason = Config::Q1_SHADOW_INVALID_Q_ABOVE_SUPPORT;
      }
    }
  }
  event.q1_shadow_valid = reason == Config::Q1_SHADOW_INVALID_NONE;
  event.q1_shadow_invalid_reason = reason;
  logger_->addQ1ShadowEvent(event);
}
void ExperimentRunner::resetQIdentTracker() {
  q_ident_armed_ = false;
  q_ident_pulse_authorized_ = false;
  q_ident_arm_consecutive_count_ = 0;
  q_ident_last_arm_side_ = 0;
  for (uint8_t i = 0; i < Config::Q_IDENT_SIDE_COUNT; ++i) {
    q_ident_side_occurrence_count_[i] = 0;
  }
}

bool ExperimentRunner::qIdentRateInSupport(float abs_rate_dps) const {
  return isfinite(abs_rate_dps) && abs_rate_dps >= Config::Q_IDENT_RATE_SUPPORT_MIN_DPS &&
      abs_rate_dps <= Config::Q_IDENT_RATE_SUPPORT_MAX_DPS;
}

float ExperimentRunner::qIdentRequiredWidthMs(float q_target_mA_s, float signed_i0_mA,
                                               int8_t direction) const {
  if (!isfinite(q_target_mA_s) || !isfinite(signed_i0_mA) || q_target_mA_s <= 0.0f ||
      (direction != -1 && direction != 1)) return NAN;
  const float upper_ms = static_cast<float>(Config::Q_IDENT_SOLVER_DIAGNOSTIC_MAX_WIDTH_MS);
  const float q_at_upper = fabsf(identificationChargeMaS(signed_i0_mA, direction, upper_ms));
  if (!isfinite(q_at_upper) || q_at_upper + 0.0001f < q_target_mA_s) return NAN;
  float lo_ms = 0.0f;
  float hi_ms = upper_ms;
  for (uint8_t i = 0; i < 32; ++i) {
    const float mid_ms = 0.5f * (lo_ms + hi_ms);
    if (fabsf(identificationChargeMaS(signed_i0_mA, direction, mid_ms)) < q_target_mA_s) {
      lo_ms = mid_ms;
    } else {
      hi_ms = mid_ms;
    }
  }
  return hi_ms;
}

bool ExperimentRunner::qIdentSolvePulse(float q_target_mA_s, float signed_i0_mA, int8_t direction,
                                        float* required_width_ms,
                                        uint16_t* selected_integer_width_ms, uint16_t* width_ms,
                                        float* q_effective_pred_mA_s) const {
  if (!required_width_ms || !selected_integer_width_ms || !width_ms || !q_effective_pred_mA_s ||
      !isfinite(q_target_mA_s) || !isfinite(signed_i0_mA) || q_target_mA_s <= 0.0f ||
      (direction != -1 && direction != 1)) return false;
  *required_width_ms = qIdentRequiredWidthMs(q_target_mA_s, signed_i0_mA, direction);
  *selected_integer_width_ms = isfinite(*required_width_ms)
      ? static_cast<uint16_t>(ceilf(*required_width_ms - 0.0001f)) : 0;
  if (!isfinite(*required_width_ms) || *selected_integer_width_ms < Config::Q_IDENT_MIN_PULSE_MS ||
      *selected_integer_width_ms > Config::Q_IDENT_MAX_PULSE_MS) return false;
  const float q_effective = fabsf(identificationChargeMaS(
      signed_i0_mA, direction, static_cast<float>(*selected_integer_width_ms)));
  if (!isfinite(q_effective)) return false;
  *width_ms = *selected_integer_width_ms;
  *q_effective_pred_mA_s = q_effective;
  return true;
}

bool ExperimentRunner::beginQIdentPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                         uint16_t pulse_width_ms) {
  if (!q_ident_mode_ || !q_ident_armed_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < Config::Q_IDENT_MIN_PULSE_MS || pulse_width_ms > Config::Q_IDENT_MAX_PULSE_MS) {
    return false;
  }
  q_ident_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      Config::MOTOR_DRIVER_POLARITY * direction * Config::Q_IDENT_CURRENT_MA);
  if (!roller_->setCurrentMa(command_current_mA)) {
    q_ident_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::Q_IDENT_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(Config::Q_IDENT_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::Q_IDENT_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  updatePulseModelPrediction();
  return true;
}

void ExperimentRunner::updateQIdentPulse(uint32_t now_ms) {
  if (!q_ident_mode_) {
    stopMotor();
    return;
  }
  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
  }
}

void ExperimentRunner::updateQIdentAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                const Q1ZeroCrossDiagnostics& diagnostics) {
  if (!q_ident_mode_ || !logger_) return;
  PsramLogger::QIdentEvent event;
  event.q_ident_run_schedule_id = q_ident_run_schedule_id_ + 1;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.interpolated_zero_cross_time_ms = diagnostics.interpolated_zero_cross_time_ms;
  event.interpolated_physical_roll_rate_dps = diagnostics.interpolated_physical_roll_rate_dps;
  event.detector_crossing_direction = diagnostics.detector_crossing_direction;
  event.detector_angle_before_deg = diagnostics.detector_angle_before_deg;
  event.detector_angle_after_deg = diagnostics.detector_angle_after_deg;
  event.sign_gate_passed = diagnostics.sign_gate_passed;
  event.physical_roll_abs_diag_deg = status_.physical_roll_abs_deg;
  event.current_roll_deg = status_.current_roll_deg;
  event.physical_roll_rate_dps = rate_dps;
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.vbat_mV = status_.roller_battery_mV;
  event.pulse_width_guard_max_ms = Config::Q_IDENT_MAX_PULSE_MS;
  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);

  if (!q_ident_armed_) {
    const bool qualifies = event.sign_gate_passed &&
        event.zero_cross_abs_rate_dps >= Config::Q_IDENT_ARM_MIN_ABS_RATE_DPS &&
        (q_ident_last_arm_side_ == 0 || event.physical_next_peak_side != q_ident_last_arm_side_);
    if (qualifies) {
      q_ident_arm_consecutive_count_++;
      q_ident_last_arm_side_ = event.physical_next_peak_side;
      if (q_ident_arm_consecutive_count_ >= Config::Q_IDENT_ARM_CONSECUTIVE_CROSSES) {
        q_ident_armed_ = true;
        event.q_ident_armed = true;
        event.arm_consecutive_count = q_ident_arm_consecutive_count_;
        event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARMED_EVENT_NO_OUTPUT;
      } else {
        event.arm_consecutive_count = q_ident_arm_consecutive_count_;
        event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARM_WAITING;
      }
    } else {
      q_ident_arm_consecutive_count_ = 0;
      q_ident_last_arm_side_ = 0;
      event.arm_consecutive_count = 0;
      event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ARM_WAITING;
    }
    logger_->addQIdentEvent(event);
    return;
  }

  event.q_ident_armed = true;
  event.arm_consecutive_count = q_ident_arm_consecutive_count_;
  if (event.zero_cross_abs_rate_dps < Config::Q_IDENT_RATE_SUPPORT_MIN_DPS) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_RATE_BELOW_SUPPORT;
    logger_->addQIdentEvent(event);
    return;
  }
  if (event.zero_cross_abs_rate_dps > Config::Q_IDENT_RATE_SUPPORT_MAX_DPS) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_RATE_ABOVE_SUPPORT;
    logger_->addQIdentEvent(event);
    return;
  }

  const uint8_t side_index = event.physical_next_peak_side > 0 ? 0 : 1;
  if (q_ident_side_occurrence_count_[side_index] >= Config::Q_IDENT_OCCURRENCES_PER_SIDE) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_SCHEDULE_EXHAUSTED;
    logger_->addQIdentEvent(event);
    return;
  }
  const uint8_t occurrence = q_ident_side_occurrence_count_[side_index]++;
  const float q_target = Config::Q_IDENT_SCHEDULE_Q_MAS[q_ident_run_schedule_id_][side_index][occurrence];
  event.side_occurrence_index = occurrence + 1;
  event.q_schedule_target_mA_s = q_target;
  event.q_target_mA_s = q_target;
  event.q_command_direction = -event.physical_next_peak_side;

  bool q_level_valid = false;
  for (float level : Config::Q_IDENT_Q_LEVELS_MAS) {
    if (fabsf(q_target - level) <= 0.0001f) q_level_valid = true;
  }
  if (!q_level_valid || !isfinite(q_target)) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_SOLVER_NONFINITE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (q_target == 0.0f) {
    event.q_ident_valid = true;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_Q_ZERO_NO_PULSE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ESTOP_OR_STATE;
    logger_->addQIdentEvent(event);
    return;
  }
  if (!roller_ || !roller_->ok()) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_ROLLER_NOT_READY;
    logger_->addQIdentEvent(event);
    return;
  }
  const uint16_t battery_mV = status_.roller_battery_mV;
  if (battery_mV < Config::Q_IDENT_BATTERY_MIN_MV || battery_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_BATTERY_GUARD;
    logger_->addQIdentEvent(event);
    return;
  }
  const float signed_i0_mA = event.i0_estimated_mA;
  float solver_required_width_ms = NAN;
  uint16_t solver_selected_integer_width_ms = 0;
  uint16_t width_ms = 0;
  float q_effective_pred_mA_s = NAN;
  if (!qIdentSolvePulse(q_target, signed_i0_mA, event.q_command_direction,
                        &solver_required_width_ms, &solver_selected_integer_width_ms, &width_ms,
                        &q_effective_pred_mA_s)) {
    event.solver_required_width_ms = solver_required_width_ms;
    event.solver_selected_integer_width_ms = solver_selected_integer_width_ms;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_PULSE_WIDTH_GUARD;
    logger_->addQIdentEvent(event);
    return;
  }
  event.solver_required_width_ms = solver_required_width_ms;
  event.solver_selected_integer_width_ms = solver_selected_integer_width_ms;
  event.command_current_mA = Config::Q_IDENT_CURRENT_MA;
  event.pulse_width_ms = width_ms;
  event.q_effective_pred_mA_s = q_effective_pred_mA_s;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + width_ms;
  if (!beginQIdentPulse(now_ms, t_test_ms, event.q_command_direction, width_ms)) {
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_CURRENT_WRITE_FAILED;
    logger_->addQIdentEvent(event);
    return;
  }
  event.q_ident_valid = true;
  event.q_ident_invalid_reason = Config::Q_IDENT_INVALID_NONE;
  logger_->addQIdentEvent(event);
}
void ExperimentRunner::resetEnergyControlV0OutputGate() {
  energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION;
  energy_control_v0_previous_accepted_next_peak_side_ = 0;
  energy_control_v0_rearm_excursion_seen_ = false;
  energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
}

void ExperimentRunner::updateEnergyControlV0OutputGate(float detector_relative_angle_deg) {
  if (!energy_control_v0_mode_ || !isfinite(detector_relative_angle_deg)) return;
  const float abs_detector_angle_deg = fabsf(detector_relative_angle_deg);
  energy_control_v0_max_abs_detector_excursion_deg_ = fmaxf(
      energy_control_v0_max_abs_detector_excursion_deg_, abs_detector_angle_deg);
  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION) {
    if (abs_detector_angle_deg >= Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG) {
      energy_control_v0_rearm_excursion_seen_ = true;
      energy_control_v0_output_gate_state_ =
          EnergyControlV0OutputGateState::ARMED_FOR_ZERO_CROSS;
    }
    return;
  }
  if (energy_control_v0_output_gate_state_ !=
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION) return;

  // Detector is anti-correlated with +gy.  The return excursion after side s
  // has detector sign -s; V0.2 accidentally accepted the opposite sign.
  const int8_t previous_physical_side = energy_control_v0_previous_accepted_next_peak_side_;
  if (previous_physical_side != 0 &&
      detector_relative_angle_deg * static_cast<float>(previous_physical_side) <=
          -Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG) {
    energy_control_v0_rearm_excursion_seen_ = true;
    energy_control_v0_output_gate_state_ =
        EnergyControlV0OutputGateState::ARMED_FOR_ZERO_CROSS;
  }
}

void ExperimentRunner::disarmEnergyControlV0AfterAcceptedCross(
    int8_t physical_next_peak_side) {
  energy_control_v0_previous_accepted_next_peak_side_ = physical_next_peak_side;
  energy_control_v0_rearm_excursion_seen_ = false;
  energy_control_v0_max_abs_detector_excursion_deg_ = 0.0f;
  energy_control_v0_output_gate_state_ =
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION;
}
void ExperimentRunner::updateEnergyControlV0AtZeroCross(
    uint32_t t_test_ms, float rate_dps, const Q1ZeroCrossDiagnostics& diagnostics) {
  (void)diagnostics;  // Logged by the parallel Q1 shadow event.
  if (!energy_control_v0_mode_ || !logger_) return;

  PsramLogger::EnergyControlV0Event event;
  event.zero_cross_time_ms = t_test_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  event.candidate_physical_next_peak_side = event.physical_next_peak_side;
  event.previous_accepted_physical_next_peak_side =
      energy_control_v0_previous_accepted_next_peak_side_;
  event.output_gate_state = static_cast<uint8_t>(energy_control_v0_output_gate_state_);
  event.rearm_excursion_seen = energy_control_v0_rearm_excursion_seen_;
  event.maximum_excursion_since_previous_cross_deg =
      energy_control_v0_max_abs_detector_excursion_deg_;
  event.rearm_threshold_deg = Config::ENERGY_CONTROL_V0_REARM_EXCURSION_DEG;
  event.q_command_direction = -event.physical_next_peak_side;

  auto recordEnergyControlV0Event = [&]() {
    if (!event.output_executed && !event.valid &&
        event.output_blocked_reason == Config::ENERGY_CONTROL_V0_INVALID_NONE) {
      event.output_blocked_reason = event.reason;
    }
    logger_->addEnergyControlV0Event(event);
  };

  // If the corresponding audit record cannot be retained, no actual V0 pulse
  // may occur.  This keeps overflow fail-closed rather than silently losing
  // causality between a motor action and its RWLOG decision.
  if (logger_->energyControlV0EventCapacityReached() ||
      logger_->q1ShadowEventCapacityReached()) {
    logger_->markEnergyControlV0EventOverflow();
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_EVENT_LOG_OVERFLOW;
    event.output_blocked_reason = event.reason;
    if (!logger_->energyControlV0EventCapacityReached()) {
      recordEnergyControlV0Event();
    }
    return;
  }

  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_INITIAL_EXCURSION) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_WAIT_INITIAL_EXCURSION;
    recordEnergyControlV0Event();
    return;
  }
  if (energy_control_v0_output_gate_state_ ==
      EnergyControlV0OutputGateState::WAIT_OPPOSITE_EXCURSION) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_WAIT_REARM_EXCURSION;
    recordEnergyControlV0Event();
    return;
  }
  event.side_alternation_passed =
      event.previous_accepted_physical_next_peak_side == 0 ||
      event.candidate_physical_next_peak_side ==
          -event.previous_accepted_physical_next_peak_side;
  if (!event.side_alternation_passed) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_NONALTERNATING_SIDE;
    recordEnergyControlV0Event();
    return;
  }
  event.output_authorized = true;
  // A physically accepted crossing is consumed even when it later coasts or
  // fails a hardware guard.  The following candidate must re-establish the
  // opposite-side excursion before any new output can be authorized.
  disarmEnergyControlV0AfterAcceptedCross(event.physical_next_peak_side);
  event.target_peak_abs_deg = Config::ENERGY_CONTROL_V0_TARGET_PEAK_DEG;
  event.vbat_mV = status_.roller_battery_mV;
  event.rate_support_status = event.zero_cross_abs_rate_dps < Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS ? 0 :
      (event.zero_cross_abs_rate_dps <= Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS ? 1 : 2);
  event.q1_gain_deg_per_mA_s = event.physical_next_peak_side > 0
      ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS
      : Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;
  event.passive_next_peak_abs_deg = Config::Q1_SHADOW_INTERCEPT_DEG +
      Config::Q1_SHADOW_RATE_GAIN_DEG_PER_DPS * event.zero_cross_abs_rate_dps +
      Config::Q1_SHADOW_SIDE_TERM_DEG * static_cast<float>(event.physical_next_peak_side);

  const auto potentialJ = [](float amplitude_deg) -> float {
    const float amplitude_rad = fabsf(amplitude_deg) * 0.01745329251994329577f;
    const float theta_inner = asinf(Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M /
                                    Config::ENERGY_CONTROL_V0_RADIUS_M);
    const float theta_outer = asinf(Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M /
                                    Config::ENERGY_CONTROL_V0_RADIUS_M);
    if (!isfinite(amplitude_rad) || amplitude_rad > theta_outer) return NAN;
    const float center_height = sqrtf(Config::ENERGY_CONTROL_V0_RADIUS_M *
                                      Config::ENERGY_CONTROL_V0_RADIUS_M -
                                      Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M *
                                      Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M);
    const float height_m = amplitude_rad <= theta_inner
        ? Config::ENERGY_CONTROL_V0_CG_HEIGHT_M * cosf(amplitude_rad) +
              Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M * sinf(amplitude_rad)
        : Config::ENERGY_CONTROL_V0_RADIUS_M +
              (Config::ENERGY_CONTROL_V0_CG_HEIGHT_M - center_height) * cosf(amplitude_rad);
    return Config::ENERGY_CONTROL_V0_MASS_KG * Config::ENERGY_CONTROL_V0_GRAVITY_M_S2 *
        (height_m - Config::ENERGY_CONTROL_V0_CG_HEIGHT_M);
  };

  event.passive_energy_j = potentialJ(event.passive_next_peak_abs_deg);
  event.target_energy_j = potentialJ(event.target_peak_abs_deg);
  if (!isfinite(event.zero_cross_abs_rate_dps) || !isfinite(event.q1_gain_deg_per_mA_s) ||
      event.q1_gain_deg_per_mA_s <= 0.0f || !isfinite(event.passive_energy_j) ||
      !isfinite(event.target_energy_j)) {
    event.reason = !isfinite(event.passive_energy_j) || !isfinite(event.target_energy_j)
        ? Config::ENERGY_CONTROL_V0_INVALID_POTENTIAL_DOMAIN
        : Config::ENERGY_CONTROL_V0_INVALID_NONFINITE_STATE;
    recordEnergyControlV0Event();
    return;
  }

  event.delta_energy_required_j = event.target_energy_j - event.passive_energy_j;
  event.q_selected_mA_s = 0.0f;
  event.q_effective_pred_mA_s = 0.0f;
  event.predicted_next_peak_abs_deg = event.passive_next_peak_abs_deg;
  event.predicted_next_energy_j = event.passive_energy_j;
  event.q_support_status = 0;
  if (event.delta_energy_required_j <= 0.0f) {
    // Braking is not identified: coast safely and record a valid no-output event.
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT;
    recordEnergyControlV0Event();
    return;
  }

  float best_q = 0.0f;
  float best_peak = event.passive_next_peak_abs_deg;
  float best_energy = event.passive_energy_j;
  float best_error = fabsf(event.target_energy_j - best_energy);
  const uint16_t steps = static_cast<uint16_t>(lroundf(
      Config::ENERGY_CONTROL_V0_Q_MAX_MAS / Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS));
  for (uint16_t i = 1; i <= steps; ++i) {
    const float q = static_cast<float>(i) * Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS;
    const float candidate_peak = event.passive_next_peak_abs_deg + event.q1_gain_deg_per_mA_s * q;
    const float candidate_energy = potentialJ(candidate_peak);
    if (!isfinite(candidate_energy)) break;
    const float error = fabsf(event.target_energy_j - candidate_energy);
    if (error < best_error) {
      best_q = q;
      best_peak = candidate_peak;
      best_energy = candidate_energy;
      best_error = error;
    }
  }
  event.q_selected_mA_s = best_q;
  event.predicted_next_peak_abs_deg = best_peak;
  event.predicted_next_energy_j = best_energy;
  event.q_saturated_at_max = best_q >= Config::ENERGY_CONTROL_V0_Q_MAX_MAS -
      0.5f * Config::ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS &&
      best_energy + 1.0e-9f < event.target_energy_j;
  event.q_support_status = best_q <= 0.0f ? 0 :
      (best_q < Config::Q1_SHADOW_Q_SUPPORT_MIN_MAS ? 1 :
       (best_q <= Config::Q1_SHADOW_Q_SUPPORT_MAX_MAS ? 2 : 3));
  if (best_q <= 0.0f) {
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_V0_VALID_PASSIVE_NO_OUTPUT;
    recordEnergyControlV0Event();
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_ESTOP_OR_STATE;
    recordEnergyControlV0Event();
    return;
  }
  if (!roller_ || !roller_->ok()) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_ROLLER_NOT_READY;
    recordEnergyControlV0Event();
    return;
  }
  if (event.vbat_mV < Config::Q_IDENT_BATTERY_MIN_MV || event.vbat_mV > Config::Q_IDENT_BATTERY_MAX_MV) {
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_BATTERY_GUARD;
    recordEnergyControlV0Event();
    return;
  }

  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  float required_width_ms = NAN;
  uint16_t selected_integer_width_ms = 0;
  uint16_t width_ms = 0;
  float q_effective_pred_mA_s = NAN;
  if (!qIdentSolvePulse(best_q, event.i0_estimated_mA, event.q_command_direction,
                         &required_width_ms, &selected_integer_width_ms, &width_ms,
                         &q_effective_pred_mA_s)) {
    event.solver_required_width_ms = required_width_ms;
    event.solver_selected_integer_width_ms = selected_integer_width_ms;
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_PULSE_WIDTH_GUARD;
    recordEnergyControlV0Event();
    return;
  }
  event.solver_required_width_ms = required_width_ms;
  event.solver_selected_integer_width_ms = selected_integer_width_ms;
  event.q_effective_pred_mA_s = q_effective_pred_mA_s;
  event.predicted_next_peak_abs_deg = event.passive_next_peak_abs_deg +
      event.q1_gain_deg_per_mA_s * q_effective_pred_mA_s;
  event.predicted_next_energy_j = potentialJ(event.predicted_next_peak_abs_deg);
  event.command_current_mA = Config::Q_IDENT_CURRENT_MA;
  event.pulse_width_ms = width_ms;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + width_ms;
  if (!beginEnergyControlV0Pulse(now_ms, t_test_ms, event.q_command_direction, width_ms)) {
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.reason = Config::ENERGY_CONTROL_V0_INVALID_CURRENT_WRITE_FAILED;
    recordEnergyControlV0Event();
    return;
  }
  event.output_executed = true;
  event.valid = true;
  event.reason = Config::ENERGY_CONTROL_V0_INVALID_NONE;
  recordEnergyControlV0Event();
}

bool ExperimentRunner::beginEnergyControlV0Pulse(uint32_t now_ms, uint32_t t_test_ms,
                                                   int8_t direction, uint16_t pulse_width_ms) {
  if (!energy_control_v0_mode_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      pulse_width_ms < Config::Q_IDENT_MIN_PULSE_MS || pulse_width_ms > Config::Q_IDENT_MAX_PULSE_MS) {
    return false;
  }
  energy_control_v0_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      Config::MOTOR_DRIVER_POLARITY * direction * Config::Q_IDENT_CURRENT_MA);
  if (!roller_->setCurrentMa(command_current_mA)) {
    energy_control_v0_pulse_authorized_ = false;
    stopMotor();
    return false;
  }
  status_.current_mA_setting = Config::Q_IDENT_CURRENT_MA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(Config::Q_IDENT_CURRENT_MA, v);
  const float tau_s = predictRiseTauS(Config::Q_IDENT_CURRENT_MA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  updatePulseModelPrediction();
  return true;
}

void ExperimentRunner::updateEnergyControlV0Pulse(uint32_t now_ms) {
  if (!energy_control_v0_mode_) {
    stopMotor();
    return;
  }
  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
    status_.current_mA_setting = 0;
    status_.pulse_width_ms_setting = 0;
  }
}
void ExperimentRunner::resetEnergyControlAutonomous() {
  for (float &residual : inverse_negative_residual_j_) residual = 0.f;
  inverse_negative_residual_count_ = 0;
  inverse_negative_residual_next_ = 0;
  inverse_negative_extra_ms_ = 0;
  inverse_negative_pending_width_ms_ = 0;
  inverse_negative_pending_bias_j_ = 0.f;
  adaptive_cg_=lateral_cg::Adaptive{};
  if (wheelWorkMode()) {
    const float initial=lateral_cg::matchesAbsolute(lateral_cg_) ? lateral_cg_.shift_mm : 0.f;
    adaptive_cg_.reset(initial,0.f);
    run_cg_shift_mm_=adaptive_cg_.snapshot().shift_mm;
  } else run_cg_shift_mm_=0.f;
  pending_cg_shift_mm_=0;
  work_negative_feedback_ = wheel_work_energy_control::NegativeFeedback{};
  walking_loss_feedback_=walking_energy_loss::Feedback{};
  walking_low_speed_guard_=walking_energy_loss::LowSpeedGuard{};
  low_peak_feedback_.reset();work_positive_low_applied_bias_mj_=0.f;
  work_walking_pending_=false;work_walking_low_pending_=false;
  foot_balance_.reset(status_.run_id);foot_energy_.reset(status_.run_id);wheel_speed_balance_.reset();gradual_start_.reset();foot_frame_run_id_=0;
  foot_phase_observer_.reset(status_.run_id);foot_phase_cap_.reset();phase_observer_sequence_=0;
  foot_span_observer_.reset(status_.run_id);foot_span_control_.reset();weak_side_assist_.reset();
  long_foot_balance_.reset();
  negative_cap_recovery_.reset();
  negative_recovery_last_applied_ms_=0;
  foot_trial_pending_ms_=foot_trial_pending_generation_=0;foot_trial_pending_effective_=false;
  for(auto& g:foot_trial_generation_)g=0;
  for(auto& e:foot_trial_effective_)e=false;
  for(auto& peak:negative_preceding_positive_peaks_)peak=NAN;
  for(auto& ms:negative_preceding_positive_peak_ms_)ms=0;
  for(auto& branch:span_positive_branch_)branch=0;
  positive_left_boost_.reset();positive_left_input_=positive_left_boost::Input{};tilt_priority_.reset();wheel_cycle_guard_.reset();positive_sway_probe_.reset();span_observer_sequence_=0;
  span_positive_decision_next_=0;
  for (auto& x:span_positive_decisions_) x=SpanPositiveDecision{};
  for (auto& x:span_positive_context_) x=SpanPositiveContext{};
  phase_negative_peak_next_=phase_negative_decision_next_=0;
  for (auto& x:phase_negative_peaks_) x=PhaseNegativePeak{};
  for (auto& x:phase_negative_decisions_) x=PhaseNegativeDecision{};
  foot_frame_left_valid_=foot_frame_right_valid_=false;pending_target_energy_j_=NAN;
  for (auto& feedback : work_positive_feedback_) feedback=wheel_work_energy_control::SideEnergyFeedback{};
  work_negative_assist_feedback_=wheel_work_energy_control::NegativeAssistFeedback{};
  work_target12_controller_=target12_state_work::Controller{};
  work_pending_battery_mv_=NAN;work_pending_cap_ms_=0;
  work_pending_calibration_weight_=0.f;
  work_positive_pending_=false;
  work_positive_pending_raw_j_=NAN;
  work_negative_pending_ = false;
  work_negative_pending_supported_ = false;
  work_negative_pending_width_ms_ = 0;
  work_negative_pending_bias_j_ = 0.f;
  negative_probe_batch_crossing_index_ = 0;
  positive_current_batch_crossing_index_ = 0;
  solver_shadow_event_ = PsramLogger::SolverShadowEvent{};
  solver_shadow_pending_ = false;
  timing_probe_event_ = PsramLogger::TimingProbeEvent{};
  timing_probe_pending_ = false;
  timing_probe_loop_captured_ = false;
  timing_probe_log_captured_ = false;
  timing_probe_imu_captured_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::IDLE;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
  energy_control_autonomous_integral_plus_j_ = 0.0f;
  energy_control_autonomous_integral_minus_j_ = 0.0f;
  peak_error_model_.reset();
  energy_control_autonomous_last_motion_sample_us_ = 0;
  energy_control_autonomous_detector_has_previous_angle_ = false;
  // V46aa control-zero reset begin
  energy_control_autonomous_detector_zero_angle_deg_ = 0.0f;
  // V46aa control-zero reset end
  energy_control_autonomous_previous_detector_relative_angle_deg_ = 0.0f;
  energy_control_autonomous_previous_detector_rate_dps_ = 0.0f;
  energy_control_autonomous_previous_detector_test_ms_ = 0;
  energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  energy_control_autonomous_last_accepted_zero_cross_valid_ = false;
  energy_control_autonomous_last_accepted_zero_cross_ms_ = 0;
  resetEnergyControlAutonomousPeakTracker(false);
  energy_control_autonomous_last_peak_valid_ = false;
  energy_control_autonomous_last_peak_amplitude_deg_ = NAN;
  energy_control_autonomous_last_peak_side_ = 0;
  energy_control_autonomous_last_peak_ms_ = 0;
  energy_control_autonomous_pending_peak_ = false;
  energy_control_autonomous_pending_probe_ = false;
  energy_control_autonomous_pending_next_side_ = 0;
  energy_control_autonomous_pending_q_command_mA_s_ = 0.0f;
  energy_control_autonomous_pending_model_energy_j_ = NAN;
  energy_control_autonomous_pending_wheel_rpm_ = NAN;
  energy_control_autonomous_pending_wheel_age_us_ = UINT32_MAX;
  energy_control_autonomous_pending_zero_ms_ = 0;
  energy_control_autonomous_pending_mode_ = peak_error_adaptation::Coast;
  energy_control_autonomous_pending_wheel_valid_ = false;
  energy_control_autonomous_pending_saturated_upper_ = false;
  energy_control_autonomous_pending_saturated_lower_ = false;
  energy_control_autonomous_pending_zero_event_index_ = 0;
}

void ExperimentRunner::resetEnergyControlAutonomousPeakTracker(bool enable) {
  energy_control_autonomous_peak_tracker_enabled_ = enable;
  energy_control_autonomous_peak_tracker_started_ = false;
  energy_control_autonomous_candidate_detector_side_ = 0;
  energy_control_autonomous_return_samples_ = 0;
  energy_control_autonomous_candidate_detector_peak_abs_deg_ = 0.0f;
  energy_control_autonomous_candidate_peak_amplitude_deg_ = 0.0f;
  energy_control_autonomous_candidate_peak_ms_ = 0;
}
float ExperimentRunner::energyControlPotentialJ(float amplitude_deg) const {
  const float amplitude_rad = fabsf(amplitude_deg) * 0.01745329251994329577f;
  const float theta_inner = asinf(Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M /
                                  Config::ENERGY_CONTROL_V0_RADIUS_M);
  const float theta_outer = asinf(Config::ENERGY_CONTROL_V0_OUTER_EDGE_X_M /
                                  Config::ENERGY_CONTROL_V0_RADIUS_M);
  if (!isfinite(amplitude_rad) || amplitude_rad > theta_outer) return NAN;
  const float center_height = sqrtf(Config::ENERGY_CONTROL_V0_RADIUS_M *
                                    Config::ENERGY_CONTROL_V0_RADIUS_M -
                                    Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M *
                                    Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M);
  const float height_m = amplitude_rad <= theta_inner
      ? Config::ENERGY_CONTROL_V0_CG_HEIGHT_M * cosf(amplitude_rad) +
            Config::ENERGY_CONTROL_V0_INNER_EDGE_X_M * sinf(amplitude_rad)
      : Config::ENERGY_CONTROL_V0_RADIUS_M +
            (Config::ENERGY_CONTROL_V0_CG_HEIGHT_M - center_height) * cosf(amplitude_rad);
  return Config::ENERGY_CONTROL_V0_MASS_KG * Config::ENERGY_CONTROL_V0_GRAVITY_M_S2 *
      (height_m - Config::ENERGY_CONTROL_V0_CG_HEIGHT_M);
}

// V46ai: free-peak prediction is provided solely by rate_baseline::evaluate.
float ExperimentRunner::energyControlAutonomousGainForSide(int8_t physical_side) const {
  return physical_side > 0 ? Config::Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS :
      Config::Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS;
}

void ExperimentRunner::energyControlAutonomousCorrectionParameters(
    int8_t physical_side, float* c_side_used_deg, float* g_side_corrected_deg_per_mA_s) const {
  const float base_gain = energyControlAutonomousGainForSide(physical_side);
  float c_used_deg = 0.0f;
  float g_used_deg_per_mA_s = base_gain;
  if (Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_CORRECTION_ENABLED) {
    const float c_fit_deg = physical_side > 0
        ? Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_PLUS_DEG
        : Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_C_MINUS_DEG;
    const float g_fit_deg_per_mA_s = physical_side > 0
        ? Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_PLUS_DEG_PER_MAS
        : Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_FIT_G_MINUS_DEG_PER_MAS;
    c_used_deg = Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA * c_fit_deg;
    g_used_deg_per_mA_s = base_gain +
        Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_BLEND_LAMBDA *
            (g_fit_deg_per_mA_s - base_gain);
  }
  if (c_side_used_deg) *c_side_used_deg = c_used_deg;
  if (g_side_corrected_deg_per_mA_s) *g_side_corrected_deg_per_mA_s = g_used_deg_per_mA_s;
}

float ExperimentRunner::energyControlAutonomousCorrectedPrediction(
    float free_next_peak_deg, int8_t physical_side, float q_mA_s, float* correction_deg) const {
  const float base_gain = energyControlAutonomousGainForSide(physical_side);
  float c_used_deg = 0.0f;
  float g_used_deg_per_mA_s = base_gain;
  energyControlAutonomousCorrectionParameters(physical_side, &c_used_deg, &g_used_deg_per_mA_s);
  const float base_prediction_deg = free_next_peak_deg + base_gain * q_mA_s;
  const float raw_correction_deg = c_used_deg + (g_used_deg_per_mA_s - base_gain) * q_mA_s;
  const float bounded_correction_deg = fmaxf(-Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG,
      fminf(Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG, raw_correction_deg));
  if (correction_deg) *correction_deg = bounded_correction_deg;
  return fmaxf(0.0f, base_prediction_deg + bounded_correction_deg);
}

bool ExperimentRunner::beginEnergyControlAutonomousPulse(uint32_t now_ms, uint32_t t_test_ms,
    int8_t direction, uint16_t pulse_width_ms, int16_t requested_current_mA) {
  const bool walking_target8=wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f;
  const uint16_t permitted_ms=walking_target8 ? Config::WALKING_TARGET8_MAX_PULSE_MS
                                              : Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS;
  if (!energy_control_autonomous_mode_ || status_.emergency_stop ||
      status_.state != ExperimentState::RUNNING_BATCH_SWEEP || status_.pulse_active ||
      !roller_ || !roller_->ok() || (direction != -1 && direction != 1) ||
      !autonomous_input_current::valid(requested_current_mA) ||
      (walking_target8 && requested_current_mA>Config::WALKING_TARGET8_MAX_CURRENT_MA) ||
      pulse_width_ms < 1 || pulse_width_ms > permitted_ms ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      energy_control_autonomous_half_cycle_state_ !=
          EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS) {
    return false;
  }
  const uint32_t begin_t0_us = micros();
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      Config::ENERGY_CONTROL_AUTONOMOUS_NORMAL_OUTPUT_SIGN *
      Config::MOTOR_DRIVER_POLARITY * direction * requested_current_mA);
  const uint32_t set_current_t0_us = micros();
  const bool set_current_ok = roller_->setCurrentMa(command_current_mA);
  const uint32_t set_current_us = static_cast<uint32_t>(micros() - set_current_t0_us);
  if (!set_current_ok) {
    // A failed transport write does not prove that the actuator saw nothing.
    if(optional_early_event_)optional_early_write_uncertain_=true;
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }

  const uint32_t state_t0_us = micros();
  status_.current_mA_setting = requested_current_mA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = t_test_ms;
  const uint32_t pulse_start_us = micros();
  const uint32_t state_update_us = static_cast<uint32_t>(pulse_start_us - state_t0_us);

  const uint32_t model_t0_us = micros();
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      requested_current_mA, v);
  const float tau_s = predictRiseTauS(requested_current_mA);
  const float width_s = static_cast<float>(pulse_width_ms) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + pulse_width_ms;
  const uint32_t current_model_us = static_cast<uint32_t>(micros() - model_t0_us);

  const uint32_t pulse_model_t0_us = micros();
  updatePulseModelPrediction();
  const uint32_t update_pulse_model_us = static_cast<uint32_t>(micros() - pulse_model_t0_us);
  const uint32_t total_us = static_cast<uint32_t>(micros() - begin_t0_us);
  startTimingProbe(2, t_test_ms, command_current_mA, pulse_width_ms, pulse_start_us,
                   set_current_us, state_update_us, current_model_us,
                   update_pulse_model_us, total_us);
  return true;
}
bool ExperimentRunner::beginEnergyControlAutonomousStartKickPulse(uint32_t now_ms, int8_t direction) {
  if (!energy_control_autonomous_mode_ ||
      energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::STRONG_START_KICK ||
      status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP ||
      status_.pulse_active || !roller_ || !roller_->ok() ||
      direction != Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION) return false;
  const uint32_t begin_t0_us = micros();
  energy_control_autonomous_pulse_authorized_ = true;
  const int16_t command_current_mA = static_cast<int16_t>(
      Config::MOTOR_DRIVER_POLARITY * direction * energy_control_autonomous_input_current_mA_);
  const uint32_t set_current_t0_us = micros();
  const bool set_current_ok = roller_->setCurrentMa(command_current_mA);
  const uint32_t set_current_us = static_cast<uint32_t>(micros() - set_current_t0_us);
  if (!set_current_ok) {
    energy_control_autonomous_pulse_authorized_ = false;
    stopMotor();
    return false;
  }

  const uint32_t state_t0_us = micros();
  status_.current_mA_setting = energy_control_autonomous_input_current_mA_;
  status_.pulse_width_ms_setting = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  status_.motor_cmd_mA = command_current_mA;
  status_.pulse_direction = direction;
  status_.pulse_active = true;
  status_.pulse_id++;
  active_pulse_start_ms_ = now_ms;
  active_pulse_start_test_ms_ = 0;
  const uint32_t pulse_start_us = micros();
  const uint32_t state_update_us = static_cast<uint32_t>(pulse_start_us - state_t0_us);

  const uint32_t model_t0_us = micros();
  const float i0 = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target_current_mA = direction * predictCurrentGoalMa(
      energy_control_autonomous_input_current_mA_, v);
  const float tau_s = predictRiseTauS(energy_control_autonomous_input_current_mA_);
  const float width_s = static_cast<float>(Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS) / 1000.0f;
  predicted_signed_current_end_mA_ = target_current_mA +
      (i0 - target_current_mA) * expf(-width_s / tau_s);
  predicted_current_end_ms_ = now_ms + Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  const uint32_t current_model_us = static_cast<uint32_t>(micros() - model_t0_us);

  const uint32_t pulse_model_t0_us = micros();
  updatePulseModelPrediction();
  const uint32_t update_pulse_model_us = static_cast<uint32_t>(micros() - pulse_model_t0_us);
  const uint32_t total_us = static_cast<uint32_t>(micros() - begin_t0_us);
  startTimingProbe(1, 0, command_current_mA,
                   Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS, pulse_start_us,
                   set_current_us, state_update_us, current_model_us,
                   update_pulse_model_us, total_us);
  return true;
}
void ExperimentRunner::beginEnergyControlAutonomousStartKick(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_ || !logger_) return;
  // START_KICK is deliberately outside the V7 Q selector, so it has no
  // Q_target/Q_pred label in the current audit.
  status_.current_audit_q_target_mA_s = NAN;
  status_.current_audit_q_pred_mA_s = NAN;
  energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STRONG_START_KICK;
  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
  event.event_kind = 1;
  event.zero_cross_time_ms = 0;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.target_peak_deg = energy_control_autonomous_target_peak_deg_;
  event.physical_next_peak_side = 1;
  event.q_command_direction = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION;
  event.vbat_mV = status_.roller_battery_mV;
  event.command_current_mA = energy_control_autonomous_input_current_mA_;
  event.pulse_width_ms = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  event.pulse_start_ms = 0;
  event.pulse_end_ms = Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE;
  } else if (!roller_ || !roller_->ok()) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY;
  } else if (!beginEnergyControlAutonomousStartKickPulse(now_ms,
               Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION)) {
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
  } else {
    event.output_executed = true;
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_STRONG_START_KICK_EXECUTED;
  }
  logger_->addEnergyControlAutonomousZeroCrossEvent(event);
  if (!event.output_executed) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STOP;
    status_.last_error = "energy_strong_start_kick_failed";
  }
}
void ExperimentRunner::updateEnergyControlAutonomousPulse(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_) { stopMotor(); return; }
  if (!status_.pulse_active ||
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) < status_.pulse_width_ms_setting) return;
  const bool completed_start_kick =
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STRONG_START_KICK;
  const bool completed_normal_pulse =
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  stopActivePulse(now_ms);
  status_.current_mA_setting = 0;
  status_.pulse_width_ms_setting = 0;
  energy_control_autonomous_pulse_authorized_ = false;
  if (completed_start_kick) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::WAIT_FIRST_PEAK;
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  } else if (completed_normal_pulse) {
    // Do not carry a detector extremum formed by the pulse transient
    // into the next physical half-cycle.
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
    // V46r has no post-pulse solver workload on Core1. All controller decisions
    // were completed by the bounded fast selector at the accepted zero crossing.
  }
}
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
void ExperimentRunner::appendFootSupportMetadata(PsramString& json) const {
  auto number=[](float v){return std::isfinite(v)?String(v,5):String("null");};
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  json+=",\"foot_support_assist\":{\"revision\":\"local_returning_transfer_v12\",\"output_enabled\":true,\"cad_mapping_used_for_output\":false,";
  json+="\"policy\":\"history_or_confirmed_local_returning;ordinary_width_preserved_max80;coast15;common_v11_shape_motion_guards;stall_eta250_to500_requires_local_returning_shape_and_confirmed_deceleration;one_attempt_per_peak\",";
  json+="\"prediction_source_names\":[\"unknown\",\"history\",\"local\"],\"mode_names\":[\"none\",\"ordinary_relocation\",\"coast_assist\",\"stall_ordinary_relocation\",\"stall_coast_assist\"],\"confidence_semantics\":\"history_match_score_only;local_evidence_is_separate_not_probability\",";
  json+="\"normal_width_semantics\":\"normal_width_is_raw_proposal;normal_guarded_width_is_fresh_wheel_guard_preview;requested_preserves_guarded_ordinary_width;actual_guard_must_match\",";
#else
  json+=",\"foot_support_assist\":{\"revision\":\"pose_transfer_assist_v11\",\"output_enabled\":true,\"cad_mapping_used_for_output\":false,";
  json+="\"policy\":\"forward_lower_margin_ge1;body_distance_0.8_to_pose_threshold_max5;inward_6_to45dps;ordinary_width_preserved_max60;or_coast_with_deceleration_or_returning_shape_assist15;estimated_zero_lead_width_plus20_to250ms;one_attempt_per_peak;normal_zero_priority\",";
#endif
  json+="\"deceleration\":\"3sample_median;100ms_window;drop4dps;confirmed10ms_3samples;run_peak_gap_reset\",\"coast_override\":\"pose_and_either_deceleration_or_returning_dominant_foot_asymmetry_lower_ge1_incoming_peak_le8.5;unidentified_trial;wheel_and_hardware_guards_retained\",";
  json+="\"prediction_lead_ms\":0,\"diagnostic_prediction_lead_ms\":30,\"lead_is_measured_motor_latency\":false,\"max_frame_age_ms\":160,\"min_history_score\":0.5,";
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  json+="\"uncertainty_semantics\":\"history_disagreement_or_local_trend_margin;neither_is_error_bound\",\"stall_source\":\"selected_local_only;history_selection_keeps_250ms_ceiling\",\"mode_semantics\":\"candidate_mode_at_entry;mode_and_checked_ms_at_last_successful_recheck\",";
#else
  json+="\"uncertainty_semantics\":\"history_candidate_disagreement_plus_margin_not_error_bound\",";
#endif
  json+="\"phase_names\":[\"unknown\",\"stationary\",\"forward\",\"turning\",\"returning\"],";
  json+="\"reason_names\":[\"selected\",\"inactive\",\"identity\",\"time\",\"pose_unavailable\",\"pose_uncertain\",\"stale_frame\",\"nonfinite\",\"small_forward\",\"wrong_motion\",\"outside_window\",\"speed\",\"no_ordinary_drive\",\"width\",\"insufficient_lead\",\"too_early\",\"coast_needs_evidence\"";
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  json+=",\"stall_needs_evidence\",\"stall_too_early\"";
#endif
  json+="],\"candidate_check_counts\":[";
  for(unsigned i=0;i<sizeof(foot_support_reason_counts_)/sizeof(foot_support_reason_counts_[0]);++i){if(i)json+=",";json+=String(foot_support_reason_counts_[i]);}
  json+="],\"work_control_branch\":";
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  json+="17";
#else
  json+="16";
#endif
  json+=",\"event_kind\":\"angle_rate_early_trial\",\"records\":";
  CompactJsonTable<PsramString> table(json);uint32_t stored=0;
  for(uint32_t i=0;i<foot_support_audit_count_;++i){
    if(json.length()+4096U>=640U*1024U*8U/10U)break;
    const auto& a=foot_support_audit_[i];
    String row="{\"time_ms\":"+String(a.time_ms)+",\"peak_ms\":"+String(a.peak_ms)+",\"sequence\":"+String(a.sequence)+",\"frame_ms\":"+String(a.frame_ms);
    row+=",\"predicted_left_deg\":"+number(a.left)+",\"predicted_right_deg\":"+number(a.right)+",\"confidence\":"+number(a.confidence);
    row+=",\"left_uncertainty_deg\":"+number(a.left_uncertainty)+",\"right_uncertainty_deg\":"+number(a.right_uncertainty);
    row+=",\"body_angle_deg\":"+number(a.body_angle)+",\"body_rate_dps\":"+number(a.body_rate)+",\"threshold_deg\":"+number(a.threshold);
    row+=",\"estimated_zero_ms\":"+number(a.estimated_zero_ms)+",\"normal_width_ms\":"+String(a.normal_width)+",\"requested_width_ms\":"+String(a.requested_width)+",\"guarded_width_ms\":"+String(a.guarded_width);
    row+=",\"coast_assist\":"+String(a.coast_assist?"true":"false");
    row+=",\"anticipated_shape_need\":"+String(a.anticipated_shape_need?"true":"false");
    row+=",\"inward_decelerating\":"+String(a.inward_decelerating?"true":"false");
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
    row+=",\"prediction_source\":"+String(a.prediction_source)+",\"mode\":"+String(a.mode);
    row+=",\"candidate_mode\":"+String(a.candidate_mode)+",\"checked_ms\":"+String(a.checked_ms);
    row+=",\"stall_route\":"+String(a.stall_route?"true":"false")+",\"local_returning_confirmed\":"+String(a.local_returning?"true":"false");
    row+=",\"local_evidence\":"+number(a.local_evidence);
    row+=",\"normal_guarded_width_ms\":"+String(a.normal_guarded_width)+",\"preview_speed_rpm\":"+number(a.preview_speed_rpm)+",\"preview_speed_age_us\":"+String(a.preview_speed_age_us);
#endif
    row+=",\"next_side\":"+String(static_cast<int>(a.next_side))+",\"output_executed\":"+String(a.executed?"true":"false")+"}";
    if(!table.append(row)){json.fail();return;}++stored;
  }
  if(!table.finish()){json.fail();return;}
  json+=",\"stored\":"+String(stored)+",\"total\":"+String(foot_support_audit_total_)+",\"truncated\":"+String(stored<foot_support_audit_total_?"true":"false")+"}";
}
#endif
void ExperimentRunner::observeFootFrame(uint16_t run_id,uint32_t frame_ms,bool left_valid,bool right_valid,float left_deg,float right_deg,float left_px,float right_px,
                                        bool phase_left_detected,bool phase_right_detected,float phase_left_x,float phase_right_x) {
  foot_frame_run_id_=run_id;foot_frame_ms_=frame_ms;
  foot_frame_left_valid_=left_valid;foot_frame_right_valid_=right_valid;
  foot_frame_left_=left_deg;foot_frame_right_=right_deg;
  foot_balance_.frame(run_id,frame_ms,left_valid,right_valid,left_deg,right_deg);
  foot_energy_.frame(run_id,frame_ms,left_valid && std::isfinite(left_px),right_valid && std::isfinite(right_px),left_px,right_px);
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f) {
    foot_phase_observer_.rawFrame(run_id,frame_ms,phase_left_detected,phase_right_detected,phase_left_x,phase_right_x);
    // Fixed-reference calibrated camera-relative angles avoid the optional
    // boot zero; raw detection and finite pixels remain independently required.
    const bool lv=phase_left_detected && std::isfinite(phase_left_x);
    const bool rv=phase_right_detected && std::isfinite(phase_right_x);
    const float l=lv ? foot_phase_observer::Observer::fixedAngle(foot_calibration::left,phase_left_x,appcfg::kFootAngleBZeroXPx) : NAN;
    const float r=rv ? foot_phase_observer::Observer::fixedAngle(foot_calibration::right,phase_right_x,appcfg::kFootAngleAZeroXPx) : NAN;
    foot_span_observer_.frame(run_id,frame_ms,lv,rv,l,r);
    processFootPhaseCycle(frame_ms);
    processFootSpanCycle();
  }
}
void ExperimentRunner::processFootPhaseCycle(uint32_t now_ms) {
  (void)now_ms;
  const uint32_t seq=foot_phase_observer_.sequence();
  if(seq==phase_observer_sequence_)return;
  phase_observer_sequence_=seq;
  // The previous signed phase metric remains available as a diagnostic only;
  // it never concurrently adjusts the positive-side motor output.
}
void ExperimentRunner::processFootSpanCycle() {
  const auto& span=foot_span_observer_.latest();
  if(span.sequence==span_observer_sequence_)return;
  span_observer_sequence_=span.sequence;
  const bool timing_quality=span.valid &&
      (span.left.both_directions || span.left.low_signal) &&
      (span.right.both_directions || span.right.low_signal) &&
      !span.left.single_outlier && !span.right.single_outlier &&
      !span.left.edge_unverified && !span.right.edge_unverified;
  tilt_priority_.footCycle(span.start_ms,span.end_ms,
      span.left.trimmed_span,span.right.trimmed_span,timing_quality);
  // Preserve the established timing experiment. This stricter ambiguity
  // check gates only the foot-span width feedback, not the timing policy.
  const bool feedback_quality=timing_quality &&
      foot_span_control::spanReliable(span.left.span,span.left.trimmed_span) &&
      foot_span_control::spanReliable(span.right.span,span.right.trimmed_span);
  const PhaseNegativePeak* peak=nullptr;
  const PhaseNegativeDecision* negative=nullptr;
  const SpanPositiveDecision* positive=nullptr;
  const SpanPositiveContext* positive_context=nullptr;
  unsigned positive_index=0;
  float plus_peak=NAN;
  uint32_t plus_peak_ms=0;
  uint8_t positive_branch=0;
  unsigned negatives=0,positives=0;
  for(const auto& p:phase_negative_peaks_) if(p.ms && p.ms==span.end_ms)peak=&p;
  for(unsigned i=0;i<8;++i) {
    const auto& d=phase_negative_decisions_[i];
    if(d.ms>span.start_ms && d.ms<span.end_ms) {
      negative=&d;plus_peak=negative_preceding_positive_peaks_[i];
      plus_peak_ms=negative_preceding_positive_peak_ms_[i];++negatives;
    }
  }
  for(unsigned i=0;i<8;++i) {
    const auto& d=span_positive_decisions_[i];
    if(d.ms>span.start_ms && d.ms<span.end_ms) {
      positive=&d;positive_context=&span_positive_context_[i];positive_branch=span_positive_branch_[i];positive_index=i;++positives;
    }
  }
  foot_span_control::Cycle cycle;
  cycle.start_ms=span.start_ms;cycle.end_ms=span.end_ms;
  cycle.left=span.left.trimmed_span;cycle.right=span.right.trimmed_span;
  cycle.negative_peak=peak ? peak->amplitude : NAN;
  // Distinct actual negative drive pulses must never be compared as though
  // their observed response came from the same motor command. All explicit
  // valid coasts are equivalent zero command regardless of model coverage.
  if(negatives==1 && negative) {
    const auto& n=negative->value;
    cycle.negative_trigger_deg=negative->trigger_deg;
    cycle.negative_rate_dps=negative->rate_dps;cycle.negative_rpm=negative->rpm;
    const bool context_ok=std::isfinite(cycle.negative_trigger_deg) &&
        std::isfinite(cycle.negative_rate_dps) && std::isfinite(cycle.negative_rpm);
    if(context_ok && (n.mode==foot_phase_cap::NegativeMode::Coast || n.mode==foot_phase_cap::NegativeMode::UnmodeledCoast)) {
      cycle.negative_known=true;cycle.negative_key=1;
    } else if(context_ok && n.mode==foot_phase_cap::NegativeMode::Powered) {
      cycle.negative_known=true;
      cycle.negative_key=0x80000000u | (static_cast<uint32_t>(n.ma)<<16) |
          (n.sign<0 ? 0x8000u : 0u) | static_cast<uint32_t>(n.width);
    }
  }
  if(positives==1 && positive) {
    cycle.positive_ms=positive->ms;cycle.positive_width=positive->width;
    cycle.positive_ma=positive->ma;cycle.positive_sign=positive->sign;
    cycle.positive_executed=positive->executed;cycle.target_mj=positive->target_mj;
    cycle.positive_rate_dps=positive_context ? positive_context->rate_dps : NAN;
    cycle.positive_rpm=positive_context ? positive_context->rpm : NAN;
  }
  // A small, consistently detected foot is still a measurement, not an
  // absent foot. Require a reversal for larger motion so a monotonic drift
  // alone cannot masquerade as a complete full-cycle stroke.
  const bool left_shape=span.left.both_directions || span.left.low_signal;
  const bool right_shape=span.right.both_directions || span.right.low_signal;
  // A separately bounded trend can retain actual high-quality observations
  // across missed camera periods, while trials still require strict continuity.
  // Both actuator contexts and fresh healthy sensors remain mandatory even
  // for marking a period as skipped; a fault breaks the whole observation.
  cycle.gap_allow=positives==1 && negatives==1 && peak && imu_ && roller_ &&
      imu_->acquisitionHealthy() && !imu_->stale(millis()) && roller_->ok() &&
      !status_.emergency_stop && status_.state==ExperimentState::RUNNING_BATCH_SWEEP;
  cycle.gap_quality=feedback_quality && left_shape && right_shape &&
      !span.left.single_outlier && !span.right.single_outlier &&
      !span.left.edge_unverified && !span.right.edge_unverified;
  cycle.complete=feedback_quality && span.continuous && left_shape && right_shape &&
      !span.left.single_outlier && !span.right.single_outlier &&
      !span.left.edge_unverified && !span.right.edge_unverified &&
      positives==1 && negatives==1 && peak;
  const bool hardware_healthy=imu_ && roller_ && imu_->acquisitionHealthy() &&
      !imu_->stale(millis()) && roller_->ok() && !status_.emergency_stop &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP;
  const bool observable=cycle.gap_quality && cycle.gap_allow && cycle.negative_known &&
      cycle.positive_executed && cycle.positive_ma==1200 && cycle.positive_sign==1 &&
      cycle.positive_width>0 && positive_branch==15 && std::isfinite(cycle.positive_rate_dps) &&
      std::isfinite(cycle.positive_rpm) && std::isfinite(cycle.target_mj) &&
      plus_peak_ms>cycle.start_ms && negative && plus_peak_ms<negative->ms && plus_peak_ms<cycle.end_ms;
  // Reference samples for a negative intervention must be entirely after a
  // confirmed return of positive foot feedback to zero, not overlap it.
  const bool neutral_foot_context=long_foot_balance_.idleForNegativeTrial() &&
      span.start_ms>long_foot_balance_.lastChangeMs() && positives==1 && positive &&
      positive->executed && positive->sign==1 && positive_branch==15;
  const bool negative_was_active=negative_cap_recovery_.active();
  const bool sway_was_active=positive_sway_probe_.active();
  positive_sway_probe_.cycle(cycle.start_ms,cycle.end_ms,cycle.left,cycle.right,plus_peak,cycle.negative_peak,
      observable && cycle.complete,hardware_healthy,neutral_foot_context,
      positives==1 && positive && positive->executed && positive_sway_probe_.effectiveCommand(positive->ms));
  negative_cap_recovery_.footCycle(span.start_ms,span.end_ms,cycle.left,cycle.right,
      observable,hardware_healthy,neutral_foot_context);
  long_foot_balance_.observeCycle(cycle.start_ms,cycle.end_ms,cycle.left,cycle.right,
      plus_peak,cycle.negative_peak,observable && negative_cap_recovery::normalFootReferenceAllowed(
          negative_was_active,negative_cap_recovery_.active(),span.start_ms,negative_recovery_last_applied_ms_) &&
      !sway_was_active && !positive_sway_probe_.active() &&
      (!positive_sway_probe_.lastApplied() || cycle.start_ms>positive_sway_probe_.lastApplied()),hardware_healthy,
      positives==1 && positive && positive->executed &&
      foot_trial_effective_[positive_index] &&
      foot_trial_generation_[positive_index]==long_foot_balance_.trialGeneration());
}
void ExperimentRunner::recordFootPhasePeak(int8_t physical_side,uint32_t peak_ms,float amplitude_deg) {
  foot_balance_.peak(physical_side,peak_ms);
  foot_energy_.peak(physical_side,peak_ms);
  if(physical_side<0) {
    auto& p=phase_negative_peaks_[phase_negative_peak_next_];
    p.ms=peak_ms;p.amplitude=amplitude_deg;
    phase_negative_peak_next_=(phase_negative_peak_next_+1)%8;
  }
  foot_phase_observer_.peak(status_.run_id,physical_side,peak_ms);
  foot_span_observer_.peak(status_.run_id,physical_side,peak_ms);
  processFootPhaseCycle(millis()-run_start_ms_);
  processFootSpanCycle();
}
float ExperimentRunner::energyControlPeakTarget(int8_t physical_side) const {
  return energy_control_autonomous_pending_peak_ &&
      physical_side == energy_control_autonomous_pending_next_side_ && std::isfinite(energy_control_autonomous_pending_target_deg_)
      ? energy_control_autonomous_pending_target_deg_
      : (wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f
          ? tilt_priority_.target() : energy_control_autonomous_target_peak_deg_);
}
void ExperimentRunner::notifyPositiveLeftPeak(
    int8_t physical_side,uint32_t peak_ms,float amplitude_deg,
    const PsramLogger::EnergyControlAutonomousPeakEvent& event) {
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f) {
    const bool healthy=imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis());
    const bool paired_time=energy_control_autonomous_pending_zero_ms_ &&
        peak_ms>energy_control_autonomous_pending_zero_ms_ &&
        peak_ms-energy_control_autonomous_pending_zero_ms_<=1500;
    wheel_cycle_guard_.peak(physical_side,amplitude_deg,peak_ms,
        healthy && paired_time && event.pending_command_matched &&
        energy_control_autonomous_pending_mode_==peak_error_adaptation::Excite);
    if(healthy)
      tilt_priority_.peak(physical_side,peak_ms,amplitude_deg);
    negative_cap_recovery_.peak(physical_side,peak_ms,amplitude_deg,
        paired_time && event.pending_command_matched &&
        energy_control_autonomous_pending_mode_==peak_error_adaptation::Excite,healthy && roller_ && roller_->ok());
    positive_left_boost_.peak(physical_side,peak_ms,amplitude_deg,
        imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
        event.peak_speed_valid && event.peak_speed_age_us<=30000);
  }
}
bool ExperimentRunner::recordEnergyControlAutonomousPeak(uint32_t peak_ms, int8_t physical_side,
                                                            float amplitude_deg, float detector_peak_angle_deg) {
  if (!logger_ || physical_side == 0 || !isfinite(amplitude_deg) || !isfinite(detector_peak_angle_deg) ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::WAIT_FIRST_PEAK &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      status_.pulse_active ||
      energy_control_autonomous_half_cycle_state_ != EnergyControlAutonomousHalfCycleState::WAIT_PEAK) return false;
  const bool first_peak = energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::WAIT_FIRST_PEAK;
  if (!first_peak && energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      static_cast<uint32_t>(peak_ms - energy_control_autonomous_last_accepted_zero_cross_ms_) <
          Config::ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS) return false;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    // Logging capacity is diagnostic-only. The physical half-cycle controller
    // continues and the timeseries remains available.
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  PsramLogger::EnergyControlAutonomousPeakEvent event;
  event.peak_time_ms = peak_ms;
  event.physical_peak_side = physical_side;
  event.peak_amplitude_deg = amplitude_deg;
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f) wheel_speed_balance_.peak(physical_side,amplitude_deg);
  event.detector_peak_angle_deg = detector_peak_angle_deg;
  event.target_peak_deg = energyControlPeakTarget(physical_side);
  event.peak_error_deg = event.target_peak_deg - event.peak_amplitude_deg;
  event.first_peak = first_peak;
  event.pending_command_matched = energy_control_autonomous_pending_peak_ &&
      physical_side == energy_control_autonomous_pending_next_side_;
  event.pending_q_command_mA_s = event.pending_command_matched
      ? energy_control_autonomous_pending_q_command_mA_s_ : NAN;
  event.cg_shift_mm=wheelWorkMode() ? (event.pending_command_matched ? pending_cg_shift_mm_ : run_cg_shift_mm_) : 0.f;
  event.target_energy_j = lateral_cg::potential(lateral_energy_geometry_,physical_side * event.target_peak_deg,event.cg_shift_mm);
  if(event.pending_command_matched && isfinite(pending_target_energy_j_))
    event.target_energy_j=pending_target_energy_j_;
  event.measured_energy_j = lateral_cg::potential(lateral_energy_geometry_,physical_side * event.peak_amplitude_deg,event.cg_shift_mm);
  if (roller_) {
    const uint32_t capture_us=micros();
    const auto telemetry=roller_->telemetrySnapshot();
    event.peak_telemetry_capture_time_ms=millis()-run_start_ms_;
    event.peak_speed_age_us=telemetry.speed_sample_time_us ? capture_us-telemetry.speed_sample_time_us : UINT32_MAX;
    event.peak_current_age_us=telemetry.current_sample_time_us ? capture_us-telemetry.current_sample_time_us : UINT32_MAX;
    event.peak_speed_valid=telemetry.speed_valid && telemetry.speed_sample_time_us != 0;
    event.peak_current_valid=telemetry.current_valid && telemetry.current_sample_time_us != 0;
    event.peak_wheel_speed_rpm=event.peak_speed_valid ? telemetry.speed_rpm : NAN;
    event.peak_measured_current_mA=event.peak_current_valid ? telemetry.actual_current_mA : NAN;
  }
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f)
    gradual_start_.peak(physical_side,amplitude_deg,event.peak_wheel_speed_rpm,
        event.peak_speed_valid && event.peak_speed_age_us<=30000,peak_ms);
  notifyPositiveLeftPeak(physical_side,peak_ms,amplitude_deg,event);
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f && physical_side<0)
    low_peak_feedback_.negative(peak_ms,amplitude_deg,event.peak_speed_valid && event.peak_speed_age_us<=30000);
  event.energy_error_j = event.target_energy_j - event.measured_energy_j;
  if (wheelWorkMode() && physical_side<0 &&
      (!event.pending_command_matched || !std::isfinite(event.measured_energy_j) ||
       !std::isfinite(energy_control_autonomous_pending_model_energy_j_)))
    walking_low_speed_guard_.interrupt();
  if (event.pending_command_matched && !energy_control_autonomous_pending_probe_ &&
      std::isfinite(event.measured_energy_j) &&
      std::isfinite(energy_control_autonomous_pending_model_energy_j_)) {
    event.model_residual_j = event.measured_energy_j - energy_control_autonomous_pending_model_energy_j_;
    event.model_residual_observed = true;
    observeWheelWorkPeak(event);
    if (peak_adaptation_enabled_) peak_error_model_.observe(energy_control_autonomous_pending_zero_ms_, physical_side,
        energy_control_autonomous_pending_mode_, energy_control_autonomous_pending_wheel_rpm_,
        energy_control_autonomous_pending_wheel_valid_,
        energy_control_autonomous_pending_wheel_age_us_, event.model_residual_j);
    if (energyInverseMode() && physical_side < 0 &&
        inverse_negative_pending_width_ms_ <= 25) {
      // Store residual against the original fit, not the already corrected prediction.
      inverse_negative_residual_j_[inverse_negative_residual_next_] =
          event.model_residual_j + inverse_negative_pending_bias_j_;
      inverse_negative_residual_next_ = (inverse_negative_residual_next_ + 1) % 3;
      if (inverse_negative_residual_count_ < 3) ++inverse_negative_residual_count_;
    }
  }
  if (energyInverseMode() && physical_side < 0 && event.pending_command_matched) {
    // Extend beyond the measured 25 ms range only after a completed peak.
    // A weak preceding positive peak disables extension to protect the cycle.
    const bool opposite_healthy = energy_control_autonomous_last_peak_valid_ &&
        energy_control_autonomous_last_peak_side_ > 0 &&
        energy_control_autonomous_last_peak_amplitude_deg_ >= one_peak_energy_inverse::kLowOppositePeakDeg;
    inverse_negative_extra_ms_ = one_peak_energy_inverse::nextAdaptiveExtraMs(
        inverse_negative_extra_ms_, opposite_healthy, event.peak_amplitude_deg,
        event.peak_error_deg, inverse_negative_pending_width_ms_);
  }
  event.antiwindup_upper_hold = event.pending_command_matched &&
      energy_control_autonomous_pending_saturated_upper_ && event.energy_error_j > 0;
  event.antiwindup_lower_hold = event.pending_command_matched &&
      energy_control_autonomous_pending_saturated_lower_ && event.energy_error_j < 0;
  float* integral = physical_side > 0 ? &energy_control_autonomous_integral_plus_j_ :
      &energy_control_autonomous_integral_minus_j_;
  if (!wheelWorkMode() && !((negative_probe_mode_ || energyInverseMode()) && physical_side < 0)) *integral = energy_bridge_enabled_
      ? lateral_energy::updateBridgeIntegral(*integral,event.energy_error_j,event.target_energy_j,
            physical_side,energy_control_autonomous_pending_saturated_upper_,
            energy_control_autonomous_pending_saturated_lower_,event.pending_command_matched)
      : lateral_energy::updateIntegral(*integral,event.energy_error_j,event.target_energy_j,
            energy_control_autonomous_pending_saturated_upper_,energy_control_autonomous_pending_saturated_lower_,
            event.pending_command_matched);
  energy_control_autonomous_pending_peak_ = false;
  energy_control_autonomous_pending_probe_ = false;
  energy_control_autonomous_pending_model_energy_j_ = NAN;
  work_negative_pending_ = false;
  work_positive_pending_ = false;
  energy_control_autonomous_last_peak_valid_ = true;
  energy_control_autonomous_last_peak_amplitude_deg_ = amplitude_deg;
  energy_control_autonomous_last_peak_side_ = physical_side;
  energy_control_autonomous_last_peak_ms_ = peak_ms;
  energy_control_autonomous_zero_cross_consumed_for_peak_ = false;
  energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  resetEnergyControlAutonomousPeakTracker(false);
  energy_control_autonomous_phase_ = fabsf(event.peak_error_deg) <= Config::TARGET_TOLERANCE_DEG
      ? EnergyControlAutonomousPhase::HOLD : EnergyControlAutonomousPhase::ENERGY_CONTROL;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.integral_plus_mA_s = event.integral_minus_mA_s = NAN; // retired Q-integral columns
  event.integral_plus_j = energy_control_autonomous_integral_plus_j_;
  event.integral_minus_j = energy_control_autonomous_integral_minus_j_;
  logger_->addEnergyControlAutonomousPeakEvent(event);
  if(wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f)
    recordFootPhasePeak(physical_side,peak_ms,amplitude_deg);
  return true;
}
__attribute__((noinline)) bool ExperimentRunner::observeInwardStallRescue(
    uint32_t t_test_ms,bool physical_event_suppressed,
    bool peak_just_confirmed,float angle_deg,float rate_dps) {
  const int8_t peak_side=energy_control_autonomous_last_peak_side_;
  const bool scope=wheelWorkMode() && energy_control_autonomous_target_peak_deg_==8.f &&
      energy_control_autonomous_input_peak_percent_==0 &&
#if defined(SYMMETRIC_RESCUE_TRIAL) && SYMMETRIC_RESCUE_TRIAL
      (peak_side==1 || peak_side==-1) &&
#else
      peak_side==1 &&
#endif
      energy_control_autonomous_last_peak_valid_ && !physical_event_suppressed &&
      energy_control_autonomous_half_cycle_state_==EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS &&
      !energy_control_autonomous_zero_cross_consumed_for_peak_ && !peak_just_confirmed &&
      stall_rescue_attempted_peak_ms_!=energy_control_autonomous_last_peak_ms_;
  if(!scope) {
    stall_rescue_detector_.reset();
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
    approach_stall_rescue_detector_.reset();approach_stall_rescue_event_=false;
#endif
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
    early_stall_rescue_detector_.reset();early_stall_rescue_event_=false;
#endif
    return false;
  }
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
  approach_stall_rescue_event_=approach_stall_rescue_detector_.observe(t_test_ms,
      energy_control_autonomous_previous_detector_test_ms_,energy_control_autonomous_last_peak_ms_,
      energy_control_autonomous_last_peak_amplitude_deg_,
      energy_control_autonomous_previous_detector_relative_angle_deg_*peak_side,angle_deg*peak_side,
      energy_control_autonomous_previous_detector_rate_dps_*peak_side,rate_dps*peak_side,
      25.f,2.5f,4.f);
  if(approach_stall_rescue_event_)return true;
#endif
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
  early_stall_rescue_event_=early_stall_rescue_detector_.observe(t_test_ms,
      energy_control_autonomous_previous_detector_test_ms_,energy_control_autonomous_last_peak_ms_,
      energy_control_autonomous_last_peak_amplitude_deg_,
      energy_control_autonomous_previous_detector_relative_angle_deg_*peak_side,angle_deg*peak_side,
      energy_control_autonomous_previous_detector_rate_dps_*peak_side,rate_dps*peak_side,
      15.f,.8f,2.5f);
  if(early_stall_rescue_event_)return true;
#endif
  return stall_rescue_detector_.observe(t_test_ms,energy_control_autonomous_previous_detector_test_ms_,
      energy_control_autonomous_last_peak_ms_,energy_control_autonomous_last_peak_amplitude_deg_,
      energy_control_autonomous_previous_detector_relative_angle_deg_*peak_side,angle_deg*peak_side,
      energy_control_autonomous_previous_detector_rate_dps_*peak_side,rate_dps*peak_side);
}

__attribute__((noinline)) void ExperimentRunner::acceptMotionCrossing(bool rescue,uint32_t t_test_ms,
    float rate,float before,float after,float alpha,float interpolated_time_ms) {
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  foot_support_deceleration_.reset();
#endif
  stall_rescue_event_=rescue;
  if(rescue)stall_rescue_attempted_peak_ms_=energy_control_autonomous_last_peak_ms_;
  updateEnergyControlAutonomousAtZeroCross(t_test_ms,rate,before,after,alpha,interpolated_time_ms);
  stall_rescue_event_=false;
  stall_rescue_detector_.reset();
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
  approach_stall_rescue_detector_.reset();approach_stall_rescue_event_=false;
#endif
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
  early_stall_rescue_detector_.reset();early_stall_rescue_event_=false;
#endif
}

#if FOOT_EARLY_TRIAL
void ExperimentRunner::setFootEarlyEpoch(uint64_t epoch_us) {
  early_motion_gate_.clear();
  early_measurement_epoch_us_=early_clock::epochSupported(epoch_us)?epoch_us:0;
}
uint32_t ExperimentRunner::footEarlyPeakIdentity() const {
  return energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS &&
      energy_control_autonomous_last_peak_valid_ && !energy_control_autonomous_zero_cross_consumed_for_peak_
      ? energy_control_autonomous_last_peak_ms_ : 0;
}
int8_t ExperimentRunner::footEarlyPeakSide() const {
  return footEarlyPeakIdentity() ? energy_control_autonomous_last_peak_side_ : 0;
}
bool ExperimentRunner::offerFootEarlyCandidate(uint16_t run_id,uint32_t peak_id,uint32_t frame_ms,
    uint32_t ready_ms,float threshold_deg) {
  uint32_t now=0;
  if(!early_clock::relativeMs(esp_timer_get_time(),early_measurement_epoch_us_,now) ||
      run_id!=status_.run_id || peak_id!=footEarlyPeakIdentity() || status_.emergency_stop ||
      status_.state!=ExperimentState::RUNNING_BATCH_SWEEP ||
      threshold_deg*energy_control_autonomous_last_peak_side_<=0.f) {
    early_motion_gate_.clear();return false;
  }
  return early_motion_gate_.offer(run_id,peak_id,frame_ms,ready_ms,threshold_deg,now);
}
void ExperimentRunner::tryEarlyFootCandidate(uint32_t t_test_ms,uint32_t sample_us,
    float angle_deg,float rate_dps) {
  // Keep the ordinary fresh-sample path short. All actual authorization is
  // rechecked by decideEarlyFootCandidate after this threshold precheck.
  if(!early_motion_gate_.valid) return;
  const float threshold=early_motion_gate_.threshold;
  if(threshold>0.f ? angle_deg>threshold : angle_deg<threshold) return;
  if(energy_control_autonomous_last_accepted_zero_cross_valid_ &&
     (t_test_ms<=energy_control_autonomous_last_accepted_zero_cross_ms_ ||
      t_test_ms-energy_control_autonomous_last_accepted_zero_cross_ms_<
          Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS)) return;
  decideEarlyFootCandidate(t_test_ms,sample_us,angle_deg,rate_dps);
}
void ExperimentRunner::decideEarlyFootCandidate(uint32_t t_test_ms,uint32_t sample_us,
    float angle_deg,float rate_dps) {
  const uint32_t sample_now_us=micros();
  // Unsigned subtraction handles micros rollover for the bounded run. The
  // independent run clock rejects a wrapped or mismatched epoch.
  const uint32_t elapsed_us=sample_now_us-static_cast<uint32_t>(early_measurement_epoch_us_);
  if(!early_measurement_epoch_us_ || elapsed_us>600000000u) {early_motion_gate_.clear();return;}
  const uint32_t now=elapsed_us/1000u;
  if((now>t_test_ms ? now-t_test_ms : t_test_ms-now)>5 ||
      static_cast<uint32_t>(sample_now_us-sample_us)>10000 ||
      energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
      !early_motion_gate_.fire(status_.run_id,footEarlyPeakIdentity(),now,angle_deg,rate_dps)) {
    // A new observation may propose a new candidate; don't retry stale,
    // wrong-phase or insufficient-lead data at every later IMU sample.
    early_motion_gate_.clear();return;
  }
  acceptOptionalEarlyMotion(t_test_ms,rate_dps,angle_deg);
  // In this phase the minimum half-cycle has already passed; any no-send
  // or ambiguous write must not repeatedly retry this same camera result.
  early_motion_gate_.clear();
}

#if ANGLE_EARLY_TRIAL
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
void ExperimentRunner::tryFootSupportCandidate(uint32_t t_test_ms,uint32_t sample_us,
    float angle_deg,float rate_dps) {
  const int8_t side=energy_control_autonomous_last_peak_side_;
  if(status_.pulse_active || energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     (side!=1 && side!=-1) || !std::isfinite(angle_deg) || !std::isfinite(rate_dps) ||
     angle_deg*side<=0 || rate_dps*side>=0 || static_cast<uint32_t>(micros()-sample_us)>10000)
    foot_support_deceleration_.reset();
  if(!wheelWorkMode() || energy_control_autonomous_target_peak_deg_!=8.f ||
     energy_control_autonomous_input_peak_percent_!=0 || status_.emergency_stop || status_.pulse_active ||
     energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     !energy_control_autonomous_last_peak_valid_ || energy_control_autonomous_zero_cross_consumed_for_peak_ ||
     optional_early_attempted_peak_ms_==energy_control_autonomous_last_peak_ms_ ||
     (energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      (t_test_ms<=energy_control_autonomous_last_accepted_zero_cross_ms_ ||
       t_test_ms-energy_control_autonomous_last_accepted_zero_cross_ms_<Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS)) ||
     !std::isfinite(angle_deg) || std::fabs(angle_deg)>5.f ||
     !std::isfinite(rate_dps) || static_cast<uint32_t>(micros()-sample_us)>10000 ||
     (foot_support_last_check_ms_ && (t_test_ms<foot_support_last_check_ms_ ||
       t_test_ms-foot_support_last_check_ms_<5)))return;
  foot_support_last_check_ms_=t_test_ms;
  const bool decelerating=foot_support_deceleration_.update(status_.run_id,energy_control_autonomous_last_peak_ms_,t_test_ms,-rate_dps*side);
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  const auto selection=foot_support_pose::select(foot_pose_,foot_local_pose_,t_test_ms);
  const auto& pose=selection.pose;
#else
  const auto pose=foot_pose_.predict(t_test_ms,0);
#endif
  tilt_priority::Input normal;
  normal.now_ms=t_test_ms;normal.preset_deg=8.f;normal.target_deg=tilt_priority_.target();
  normal.next_side=-side;normal.crossing_rate_dps=std::fabs(rate_dps);
  normal.hardware_cap_ms=tilt_priority_.ordinaryRiseCap(normal.next_side,t_test_ms,Config::WALKING_TARGET8_MAX_PULSE_MS);
  normal.hardware_healthy=imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) && roller_ && roller_->ok();
  auto ordinary=tilt_priority_.propose(normal);
  if(ordinary.selected && !ordinary.coast)
    ordinary.width_ms=tilt_priority_.balanceTrimCap(t_test_ms,normal.next_side,ordinary.width_ms);
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  // Preserve the protected ordinary proposal, not an unguarded width. A copy
  // previews the same wheel rule without consuming its history or applying
  // the velocity taper twice. Actual telemetry/guard is checked again later.
  uint16_t normal_guarded_width=ordinary.width_ms;
  float preview_speed_rpm=NAN;uint32_t preview_speed_age_us=0;
  bool preview_speed_usable=false;
  if(ordinary.selected && !ordinary.coast && roller_){
    const auto telemetry=roller_->telemetrySnapshot();
    preview_speed_rpm=telemetry.speed_rpm;
    preview_speed_age_us=static_cast<uint32_t>(micros()-telemetry.speed_sample_time_us);
    preview_speed_usable=telemetry.speed_valid && telemetry.speed_sample_time_us &&
        preview_speed_age_us<=50000 && std::isfinite(preview_speed_rpm);
    if(preview_speed_usable){
      auto guard=wheel_cycle_guard_;
      normal_guarded_width=guard.limit(normal.next_side,preview_speed_rpm,true,t_test_ms,ordinary.width_ms);
    }
  }
#endif
  foot_support_policy::Input in;
  in.active=true;in.run_id=status_.run_id;in.foot_run_id=foot_pose_.run_id;
  in.peak_id=energy_control_autonomous_last_peak_ms_;in.peak_side=side;in.now_ms=t_test_ms;in.frame_ms=foot_pose_.frame_ms;
  in.pose_ready=pose.ready();in.pose_confident=pose.confidence>=.5f && pose.method==foot_phase_prediction::Method::History;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  in.pose_confident=selection.usable;
  in.local_returning_confirmed=selection.local_returning_confirmed;
  in.prediction_source=selection.source==foot_support_pose::Source::Local?foot_support_policy::PredictionSource::Local:
      selection.source==foot_support_pose::Source::History?foot_support_policy::PredictionSource::History:foot_support_policy::PredictionSource::Unknown;
#endif
  in.left_deg=pose.left;in.right_deg=pose.right;
  in.left_uncertainty_deg=pose.left_uncertainty;in.right_uncertainty_deg=pose.right_uncertainty;
  in.body_angle_deg=angle_deg;in.body_rate_dps=rate_dps;
  in.normal_drive_selected=ordinary.selected && !ordinary.coast;in.normal_width_ms=ordinary.width_ms;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  in.normal_drive_selected=in.normal_drive_selected && preview_speed_usable;
  if(in.normal_drive_selected)in.normal_width_ms=normal_guarded_width;
#endif
  in.normal_coast_selected=ordinary.selected && ordinary.coast;in.inward_decelerating=decelerating;
  const bool left_dominant=pose.left-pose.left_uncertainty>=pose.right-pose.right_uncertainty;
  const auto dominant_phase=left_dominant?pose.phase_left:pose.phase_right;
  in.anticipated_shape_need=dominant_phase==foot_phase_prediction::Phase::Returning &&
      std::fabs(pose.left-pose.right)-pose.left_uncertainty-pose.right_uncertainty>=1.f &&
      energy_control_autonomous_last_peak_amplitude_deg_<=8.5f;
  const auto decision=foot_support_policy::decide(in);
  const unsigned reason=static_cast<unsigned>(decision.reason);
  if(reason<sizeof(foot_support_reason_counts_)/sizeof(foot_support_reason_counts_[0]))++foot_support_reason_counts_[reason];
  if(!decision.selected)return;
  FootSupportAudit audit;
  audit.time_ms=t_test_ms;audit.peak_ms=in.peak_id;audit.sequence=foot_pose_sequence_;audit.frame_ms=in.frame_ms;
  audit.left=pose.left;audit.right=pose.right;audit.confidence=pose.confidence;
  audit.left_uncertainty=pose.left_uncertainty;audit.right_uncertainty=pose.right_uncertainty;
  audit.body_angle=angle_deg;audit.body_rate=rate_dps;audit.threshold=decision.threshold_deg;
  audit.estimated_zero_ms=decision.estimated_zero_ms;audit.normal_width=ordinary.width_ms;
  audit.requested_width=decision.width_ms;audit.coast_assist=decision.coast_assist;
  audit.anticipated_shape_need=in.anticipated_shape_need;audit.inward_decelerating=decelerating;audit.next_side=-side;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  audit.prediction_source=static_cast<uint8_t>(in.prediction_source);audit.mode=static_cast<uint8_t>(decision.mode);
  audit.candidate_mode=audit.mode;
  audit.stall_route=decision.stall_route;audit.local_returning=selection.local_returning_confirmed;
  audit.local_evidence=selection.local_evidence;
  audit.normal_guarded_width=normal_guarded_width;audit.preview_speed_rpm=preview_speed_rpm;audit.preview_speed_age_us=preview_speed_age_us;
  foot_support_source_=selection.source;foot_support_input_=in;
#endif
  foot_support_active_audit_=-1;
  ++foot_support_audit_total_;
  if(foot_support_audit_count_<96){foot_support_active_audit_=static_cast<int>(foot_support_audit_count_++);foot_support_audit_[foot_support_active_audit_]=audit;}
  foot_support_decision_=decision;foot_support_sample_us_=sample_us;foot_support_event_=true;
  const auto pulse_before=status_.pulse_id;
  acceptOptionalEarlyMotion(t_test_ms,rate_dps,angle_deg);
  if(foot_support_active_audit_>=0)foot_support_audit_[foot_support_active_audit_].executed=status_.pulse_id!=pulse_before && status_.pulse_active;
  foot_support_event_=false;foot_support_active_audit_=-1;
}
#endif
void ExperimentRunner::tryAngleEarlyCandidate(uint32_t t_test_ms,uint32_t sample_us,
    float angle_deg,float rate_dps) {
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  tryFootSupportCandidate(t_test_ms,sample_us,angle_deg,rate_dps);
#endif
  // Configured target 8 includes the staged 4.5-to-8-degree startup ramp.
  // Cold eligibility is checked at most every 5 ms near centre. Normal ZERO
  // detection remains live on every fresh sample and always takes priority.
  if(energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     optional_early_attempted_peak_ms_==energy_control_autonomous_last_peak_ms_ ||
     angle_late_blocked_peak_ms_==energy_control_autonomous_last_peak_ms_ ||
     angle_deg>1.5f || angle_deg< -1.5f ||
     (angle_last_check_ms_ && (t_test_ms<angle_last_check_ms_ ||
       t_test_ms-angle_last_check_ms_<5)))return;
  angle_last_check_ms_=t_test_ms;
  decideAngleEarlyCandidate(t_test_ms,sample_us,angle_deg,rate_dps);
}
void ExperimentRunner::decideAngleEarlyCandidate(uint32_t t_test_ms,uint32_t sample_us,
    float angle_deg,float rate_dps) {
  const int8_t side=energy_control_autonomous_last_peak_side_;
  const float distance=angle_deg*side, speed=-rate_dps*side;
  if((side!=1 && side!=-1) || !std::isfinite(angle_deg) || !std::isfinite(rate_dps) ||
     static_cast<uint32_t>(micros()-sample_us)>10000 ||
     energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     energy_control_autonomous_zero_cross_consumed_for_peak_ || distance<=.3f || distance>1.5f ||
     speed<=5.f || distance*100.f>speed*3.f)return;
  // At most 30 ms and strictly more than 20 ms estimated lead under a
  // constant-rate assumption. This is not physical ZERO/deadline proof.
  if(distance*50.f*.999999f<=speed) {
    angle_late_blocked_peak_ms_=energy_control_autonomous_last_peak_ms_;
    return;
  }
  acceptOptionalEarlyMotion(t_test_ms,rate_dps,angle_deg);
}
#endif
#endif

__attribute__((noinline)) void ExperimentRunner::acceptOptionalEarlyMotion(
    uint32_t t_test_ms,float rate_dps,float angle_deg) {
  // The caller must establish fresh input; this wrapper does not prove contact.
  const int8_t side=energy_control_autonomous_last_peak_side_;
  if(!wheelWorkMode() || energy_control_autonomous_target_peak_deg_!=8.f ||
     energy_control_autonomous_input_peak_percent_!=0 ||
     status_.emergency_stop || status_.pulse_active ||
     energy_control_autonomous_half_cycle_state_!=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
     !energy_control_autonomous_last_peak_valid_ ||
     energy_control_autonomous_zero_cross_consumed_for_peak_ ||
     !energy_control_autonomous_last_peak_ms_ ||
     t_test_ms<=energy_control_autonomous_last_peak_ms_ ||
     t_test_ms-energy_control_autonomous_last_peak_ms_>1500 ||
     (energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      (t_test_ms<=energy_control_autonomous_last_accepted_zero_cross_ms_ ||
       t_test_ms-energy_control_autonomous_last_accepted_zero_cross_ms_ <
           Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS)) ||
     optional_early_attempted_peak_ms_==energy_control_autonomous_last_peak_ms_ ||
     (side!=1 && side!=-1) || !isfinite(rate_dps) || !isfinite(angle_deg) ||
     rate_dps*side>=0.f || angle_deg*side<=0.f) return;
  optional_early_attempted_peak_ms_=energy_control_autonomous_last_peak_ms_;
  optional_early_write_uncertain_=false;
  optional_early_event_=true;
  acceptMotionCrossing(false,t_test_ms,rate_dps,angle_deg,angle_deg,1.f,
                       static_cast<float>(t_test_ms));
  optional_early_event_=false;
}

void ExperimentRunner::updateEnergyControlAutonomousMotion(uint32_t now_ms) {
  if (!energy_control_autonomous_mode_ ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::IDLE ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STOP || !imu_) return;
  // V46ae: read one fresh gyro/MEKF sample. The independent accumulated gyro
  // angle no longer participates in Autonomous control.
  const ImuReading& sample = imu_->reading();
  const uint32_t sample_us = sample.last_gyro_update_us;
  if (sample_us == 0 || sample_us == energy_control_autonomous_last_motion_sample_us_) return;
  energy_control_autonomous_last_motion_sample_us_ = sample_us;
  const float rate_dps =
      (sample.gy_dps - status_.mekf_bias_y_dps) * Config::MEKF_GYRO_Y_SCALE;
  const float peak_relative_angle_deg = status_.pitch_mekf_measurement_relative_deg;
  const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg;
  if (!isfinite(rate_dps) || !isfinite(peak_relative_angle_deg) ||
      !isfinite(detector_relative_angle_deg)) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STOP;
    requestEmergencyStop("energy_nonfinite_mekf_state");
    return;
  }
  if (fabsf(peak_relative_angle_deg) >=
      Config::ENERGY_CONTROL_AUTONOMOUS_FIXED_TEST_STOP_ABS_DEG) {
    energy_control_autonomous_phase_ = EnergyControlAutonomousPhase::STOP;
    requestEmergencyStop("lateral_energy_angle_limit");
    return;
  }
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (!energy_control_autonomous_detector_has_previous_angle_) {
    energy_control_autonomous_detector_has_previous_angle_ = true;
    energy_control_autonomous_previous_detector_relative_angle_deg_ = detector_relative_angle_deg;
    energy_control_autonomous_previous_detector_rate_dps_ = rate_dps;
    energy_control_autonomous_previous_detector_test_ms_ = t_test_ms;
    if (!status_.pulse_active &&
        energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_PEAK) {
      updateEnergyControlAutonomousPeakTracker(now_ms, peak_relative_angle_deg, rate_dps);
    }
    return;
  }
  const float before_deg = energy_control_autonomous_previous_detector_relative_angle_deg_;
  const bool was_waiting_for_peak = energy_control_autonomous_half_cycle_state_ ==
      EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
  const bool physical_event_suppressed = status_.pulse_active ||
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE;
  // Estimators, comparison diagnostics and detector history stay live during a
  // pulse. Only promotion to a physical control event is suppressed.
  if (!physical_event_suppressed &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_PEAK) {
    updateEnergyControlAutonomousPeakTracker(now_ms, peak_relative_angle_deg, rate_dps);
  }
  const int8_t peak_side = energy_control_autonomous_last_peak_side_;
  float threshold_deg = peak_side * energy_control_autonomous_last_peak_amplitude_deg_ *
      (energy_control_autonomous_input_peak_percent_ * 0.01f);
  // Normal 0% timing stays at centre even if asymmetry grows. The former
  // increasingly early input is not supported by a measured response.
  const bool peak_just_confirmed = was_waiting_for_peak &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
  // At 100% (or a threshold already passed during peak confirmation), fire
  // once return is confirmed. The peak cannot be known at its exact extremum.
  const bool confirmation_input = peak_just_confirmed &&
      energy_control_autonomous_input_peak_percent_ > 0 && rate_dps * peak_side < 0 &&
      detector_relative_angle_deg * peak_side > 0 &&
      fabsf(detector_relative_angle_deg) <= fabsf(threshold_deg);
  const bool crossing = rate_dps * peak_side < 0.0f &&
      ((peak_side < 0 && before_deg < threshold_deg && detector_relative_angle_deg >= threshold_deg) ||
       (peak_side > 0 && before_deg > threshold_deg && detector_relative_angle_deg <= threshold_deg));
  const bool rescue=observeInwardStallRescue(t_test_ms,physical_event_suppressed,
      peak_just_confirmed,detector_relative_angle_deg,rate_dps);
  if (!physical_event_suppressed && (crossing || confirmation_input || rescue) &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS &&
      (energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::ENERGY_CONTROL ||
       energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::HOLD) &&
      energy_control_autonomous_last_peak_valid_ &&
      !energy_control_autonomous_zero_cross_consumed_for_peak_) {
    const float before_distance = fabsf(before_deg - threshold_deg);
    const float denominator = before_distance + fabsf(detector_relative_angle_deg - threshold_deg);
    const float alpha = crossing && denominator > 0.0f ? before_distance / denominator : 1.0f;
    const float interpolated_time_ms = static_cast<float>(energy_control_autonomous_previous_detector_test_ms_) +
        alpha * static_cast<float>(t_test_ms - energy_control_autonomous_previous_detector_test_ms_);
    const float interpolated_rate_dps = energy_control_autonomous_previous_detector_rate_dps_ +
        alpha * (rate_dps - energy_control_autonomous_previous_detector_rate_dps_);
    acceptMotionCrossing(rescue,t_test_ms,interpolated_rate_dps,before_deg,
        detector_relative_angle_deg,alpha,interpolated_time_ms);
  }
#if FOOT_EARLY_TRIAL
  // Normal crossing takes precedence; the gate runs outside this IRAM body.
  if(!physical_event_suppressed && !crossing && !confirmation_input && !rescue) {
#if ANGLE_EARLY_TRIAL
    tryAngleEarlyCandidate(t_test_ms,sample_us,detector_relative_angle_deg,rate_dps);
#else
    tryEarlyFootCandidate(t_test_ms,sample_us,detector_relative_angle_deg,rate_dps);
#endif
  }
#endif
  const bool passed_centre_inward = rate_dps * peak_side < 0.0f &&
      ((peak_side < 0 && before_deg < 0 && detector_relative_angle_deg >= 0) ||
       (peak_side > 0 && before_deg > 0 && detector_relative_angle_deg <= 0));
  if (!physical_event_suppressed && passed_centre_inward &&
      energy_control_autonomous_half_cycle_state_ == EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS &&
      !energy_control_autonomous_zero_cross_consumed_for_peak_) {
    energy_control_autonomous_zero_cross_consumed_for_peak_ = true;
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  }
  energy_control_autonomous_previous_detector_relative_angle_deg_ = detector_relative_angle_deg;
  energy_control_autonomous_previous_detector_rate_dps_ = rate_dps;
  energy_control_autonomous_previous_detector_test_ms_ = t_test_ms;
}
void ExperimentRunner::updateEnergyControlAutonomousPeakTracker(uint32_t now_ms,
                                                                  float peak_relative_angle_deg,
                                                                  float rate_dps) {
  if (!energy_control_autonomous_mode_ || !energy_control_autonomous_peak_tracker_enabled_ ||
      energy_control_autonomous_half_cycle_state_ != EnergyControlAutonomousHalfCycleState::WAIT_PEAK ||
      status_.pulse_active ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::IDLE ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STRONG_START_KICK ||
      energy_control_autonomous_phase_ == EnergyControlAutonomousPhase::STOP ||
      !isfinite(peak_relative_angle_deg) || !isfinite(rate_dps)) return;
  const int8_t detector_side = peak_relative_angle_deg > 0.0f ? 1 :
      (peak_relative_angle_deg < 0.0f ? -1 : 0);
  if (detector_side == 0) return;
  // V46af: a compensated zero-cross can precede the posterior zero-cross.
  // With no output, tracking resumes immediately and may still see the old
  // side. Wait for the commanded half-cycle before seeding its extremum.
  if (energy_control_autonomous_pending_peak_ &&
      detector_side != energy_control_autonomous_pending_next_side_) return;
  const float detector_abs_deg = fabsf(peak_relative_angle_deg);
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (!energy_control_autonomous_peak_tracker_started_) {
    energy_control_autonomous_peak_tracker_started_ = true;
    energy_control_autonomous_candidate_detector_side_ = detector_side;
    energy_control_autonomous_candidate_detector_peak_abs_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_amplitude_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_ms_ = t_test_ms;
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  if (detector_side == energy_control_autonomous_candidate_detector_side_ &&
      detector_abs_deg >= energy_control_autonomous_candidate_detector_peak_abs_deg_) {
    energy_control_autonomous_candidate_detector_peak_abs_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_amplitude_deg_ = detector_abs_deg;
    energy_control_autonomous_candidate_peak_ms_ = t_test_ms;
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  const int8_t rate_sign = rate_dps > 0.0f ? 1 : (rate_dps < 0.0f ? -1 : 0);
  const bool returning_toward_centre = detector_abs_deg <
      energy_control_autonomous_candidate_detector_peak_abs_deg_;
  // V46e detector pitch uses the physical/video sign and therefore agrees with
  // +gy during outward motion. On return to centre, the rate sign is opposite
  // the detector/physical peak side.
  const bool rate_confirms_return = rate_sign == -energy_control_autonomous_candidate_detector_side_;
  if (!returning_toward_centre || !rate_confirms_return) {
    energy_control_autonomous_return_samples_ = 0;
    return;
  }
  if (++energy_control_autonomous_return_samples_ <
      Config::ENERGY_CONTROL_AUTONOMOUS_PEAK_CONFIRM_SAMPLES) return;
  const float detector_peak_angle_deg = static_cast<float>(
      energy_control_autonomous_candidate_detector_side_) *
      energy_control_autonomous_candidate_detector_peak_abs_deg_;
  const bool accepted = recordEnergyControlAutonomousPeak(
      energy_control_autonomous_candidate_peak_ms_,
      energy_control_autonomous_candidate_detector_side_,
      energy_control_autonomous_candidate_peak_amplitude_deg_, detector_peak_angle_deg);
  if (!accepted) resetEnergyControlAutonomousPeakTracker(true);
}
void ExperimentRunner::selectPositiveCurrentBatch(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    int16_t& selected_current_mA,
    const lateral_energy::CurrentTable*& selected_current_table) {
  if (!positiveCurrentBatchMode() || event.physical_next_peak_side <= 0) return;
  event.positive_current_batch_index = positive_current_batch_crossing_index_;
  event.positive_current_batch_planned_mA = positive_current_batch::current(
      negative_probe_batch_seed_, positive_current_batch_crossing_index_);
  ++positive_current_batch_crossing_index_;
  selected_current_mA = event.positive_current_batch_planned_mA;
  if (selected_current_mA == 350) selected_current_table = &positive_current_probe_tables_[0];
  else if (selected_current_mA == 400) selected_current_table = &positive_current_probe_tables_[1];
}

uint16_t ExperimentRunner::selectFixedNegativeProbeWidth(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event) const {
  const bool protected_batch = negativeProbeExtendedMode() || positiveCurrentBatchMode();
  const bool safe = !protected_batch ||
      (energy_control_autonomous_last_peak_side_ > 0 &&
       event.previous_peak_amplitude_deg >= 7.0f &&
       event.vbat_mV >= 7000 && event.vbat_mV <= 8500);
  event.negative_probe_batch_rate_skipped = anyNegativeProbeBatchMode() &&
      (!event.rate_support_diagnostic || !safe) &&
      event.negative_probe_batch_planned_width_ms != 0;
  if (!event.rate_support_diagnostic || !safe) return 0;
  if (positiveCurrentBatchMode()) return positive_current_batch::kNegativeWidthMs;
  if (anyNegativeProbeBatchMode()) return event.negative_probe_batch_planned_width_ms;
  return energy_control_autonomous_brake_max_ms_;
}

void ExperimentRunner::observeWheelWorkPeak(
    const PsramLogger::EnergyControlAutonomousPeakEvent& event) {
  if (!wheelWorkMode()) return;
  if (!event.pending_command_matched || !energy_control_autonomous_pending_wheel_valid_ ||
      energy_control_autonomous_pending_wheel_age_us_>30000) {
    if(event.physical_peak_side<0) {walking_loss_feedback_.interrupt();walking_low_speed_guard_.interrupt();}
    adaptive_cg_.breakPair();return;
  }
  float raw=NAN;
  bool learn=false;
  if(work_walking_pending_ && work_negative_pending_ && event.physical_peak_side<0) {
    raw=(energy_control_autonomous_pending_model_energy_j_-work_negative_pending_bias_j_)*1000.f;
    if(work_walking_low_pending_) {
      walking_low_speed_guard_.observe(event.measured_energy_j*1000.f,energy_control_autonomous_pending_model_energy_j_*1000.f,event.target_energy_j*1000.f,
          work_negative_pending_width_ms_,event.peak_speed_valid && event.peak_speed_age_us<=30000 &&
          event.peak_wheel_speed_rpm>=30.f,event.peak_time_ms);
    } else {
      walking_low_speed_guard_.bridge(event.measured_energy_j*1000.f,energy_control_autonomous_pending_model_energy_j_*1000.f,
          event.target_energy_j*1000.f,energy_control_autonomous_pending_wheel_rpm_,
          event.peak_speed_valid && event.peak_speed_age_us<=30000 && event.peak_wheel_speed_rpm>=30.f,event.peak_time_ms);
      walking_loss_feedback_.observe(event.measured_energy_j*1000.f,raw,event.target_energy_j*1000.f,work_negative_pending_width_ms_,
          event.peak_wheel_speed_rpm,event.peak_speed_valid && event.peak_speed_age_us<=30000);
    }
    adaptive_cg_.breakPair();
    return; // Separate measured walking branch: no historical bias/CG learning.
  }
  if(event.physical_peak_side<0) {walking_loss_feedback_.interrupt();walking_low_speed_guard_.interrupt();}
  // Peak-event target_peak_deg is the actual staged side target; the fitted
  // model/feedback family must still be selected by the operator preset.
  const float preset_deg=energy_control_autonomous_target_peak_deg_;
  if(preset_deg==12.f && (work_positive_pending_ || work_negative_pending_)) {
    const float raw_mj=work_positive_pending_ ? work_positive_pending_raw_j_*1000.f :
        (energy_control_autonomous_pending_model_energy_j_-work_negative_pending_bias_j_)*1000.f;
    const float bias=energy_control_autonomous_pending_model_energy_j_*1000.f-raw_mj;
    work_target12_controller_.observe(lateral_energy_geometry_,event.physical_peak_side,
        event.peak_amplitude_deg,raw_mj,energy_control_autonomous_pending_wheel_rpm_,
        work_pending_battery_mv_,bias,event.cg_shift_mm,work_negative_pending_assist_,
        work_negative_pending_width_ms_,work_pending_cap_ms_,work_pending_calibration_weight_>=.9f);
  }
  if (event.physical_peak_side>0 && work_positive_pending_) {
    raw=work_positive_pending_raw_j_*1000.f;
    learn=work_positive_pending_branch_==1;
    if (work_positive_pending_branch_<2)
      work_positive_feedback_[work_positive_pending_branch_].observe(lateral_energy_geometry_,1,
          event.peak_amplitude_deg,raw,preset_deg,event.cg_shift_mm,
          energy_control_autonomous_pending_model_energy_j_*1000.f-raw);
    if(work_positive_pending_branch_==0 && preset_deg==8.f)
      low_peak_feedback_.positive(event.peak_time_ms,event.measured_energy_j*1000.f,
          energy_control_autonomous_pending_model_energy_j_*1000.f,work_positive_low_applied_bias_mj_,
          event.target_energy_j*1000.f,event.peak_amplitude_deg,
          event.peak_speed_valid && event.peak_speed_age_us<=30000 && std::fabs(event.cg_shift_mm)<=1e-6f,
          foot_energy_.valid(),foot_energy_.leftMean(),foot_energy_.rightMean());
  } else if (work_negative_pending_ && event.physical_peak_side<0) {
    raw=(energy_control_autonomous_pending_model_energy_j_-work_negative_pending_bias_j_)*1000.f;
    work_negative_assist_feedback_.observe(event.peak_amplitude_deg,preset_deg,
        work_negative_pending_assist_,work_negative_pending_width_ms_);
    if (!work_negative_pending_assist_) work_negative_feedback_.observe(event.measured_energy_j*1000.f,raw,
        event.target_energy_j*1000.f,energy_control_autonomous_last_peak_amplitude_deg_,
        work_negative_pending_width_ms_,true,preset_deg);
    learn=work_negative_pending_supported_ && !work_negative_pending_assist_;
    if (work_negative_pending_supported_) work_positive_feedback_[work_target_policy::negativeFeedbackIndex(preset_deg,work_negative_pending_assist_)].observe(lateral_energy_geometry_,-1,
        event.peak_amplitude_deg,raw,preset_deg);
  }
  if (!learn) adaptive_cg_.breakPair();
  else {
    adaptive_cg_.observe(lateral_energy_geometry_,event.physical_peak_side,
        event.peak_amplitude_deg,raw,event.peak_time_ms);
    run_cg_shift_mm_=adaptive_cg_.snapshot().shift_mm;
    logger_->recordAdaptiveCg(adaptive_cg_.snapshot());
  }

}

void ExperimentRunner::captureWheelWorkDecision(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    const wheel_work_energy_control::Choice& choice) const {
  event.energy_model_extrapolated=!choice.valid;
  event.work_control_branch=choice.branch;
  event.low_peak_selected=choice.low_peak_selected;
  event.low_peak_base_width_ms=choice.low_peak_base_width_ms;
  event.low_peak_base_prediction_mj=choice.low_peak_base_prediction_mj;
  event.low_peak_applied_bias_mj=choice.low_peak_selected ? choice.low_peak_applied_bias_mj : 0.f;
  event.predicted_wheel_work_j=choice.work_mj*.001f;
  event.predicted_input_work_j=choice.on_mj*.001f;
  event.predicted_post_input_work_j=choice.after_mj*.001f;
  event.predicted_work_change_j=choice.change_mj*.001f;
  event.work_prediction_cross_condition=choice.cross_condition;
  event.work_model_weight=choice.support_weight;
  if (event.physical_next_peak_side<0) {
    event.work_brake_cap_ms=choice.allowed_cap_ms;
    event.work_bias_samples=static_cast<uint8_t>(work_positive_feedback_[work_target_policy::negativeFeedbackIndex(event.target_peak_deg,choice.branch==9)].count());
  }
  if (event.physical_next_peak_side>0) {
    event.work_brake_cap_ms=choice.allowed_cap_ms; // Positive excitation cap in 8-degree mode.
    event.work_bias_samples=static_cast<uint8_t>(work_positive_feedback_[choice.branch==6 ? 1 : 0].count());
  }
  if(event.target_peak_deg==12.f)
    event.work_bias_samples=static_cast<uint8_t>(work_target12_controller_.count(event.physical_next_peak_side));
  if(choice.branch==10)
    event.work_bias_samples=static_cast<uint8_t>(walking_loss_feedback_.count());
  if(choice.branch==11)
    event.work_bias_samples=static_cast<uint8_t>(walking_low_speed_guard_.coastCount(event.zero_cross_time_ms,event.target_energy_j*1000.f));
  if (choice.valid && std::isfinite(choice.free_mj)) {
    event.passive_energy_j=choice.free_mj*.001f;
    event.free_next_peak_amplitude_deg=controlAngle(event.passive_energy_j,event.physical_next_peak_side);
  }
}

wheel_work_energy_control::Choice ExperimentRunner::chooseWheelWork(
    const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    float fast_v, bool* state_supported, int16_t* selected_current_ma,
    const lateral_energy::CurrentTable** selected_current_table,uint16_t positive_or_walking_cap_ms) const {
  wheel_work_energy_control::Choice choice;
  const int8_t side=event.physical_next_peak_side;
  const float target_mj=event.target_energy_j*1000.f;
  wheel_work_energy_control::State state{
      event.zero_cross_abs_rate_dps,event.pre_input_wheel_speed_rpm,
      static_cast<float>(event.vbat_mV),
      peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
          event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us),
      event.previous_peak_amplitude_deg};
  state.absolute_reference=true;
  state.target_deg=event.target_peak_deg;
  const bool supported=wheel_work_energy_control::supported(state,side);
  if (state_supported) *state_supported=supported;
  if(side<0 && (walking_energy_loss::supported(state) || walking_energy_loss::lowSpeedSupported(state))) {
    return walking_energy_loss::choose(lateral_energy_geometry_,state,lateral_current_table_,
        static_cast<float>(side)*event.i0_estimated_mA,predictCurrentGoalMa(300,fast_v),
        target_mj,walking_loss_feedback_,run_cg_shift_mm_,positive_or_walking_cap_ms,
        &walking_low_speed_guard_,event.zero_cross_time_ms);
  }
  if ((side<0 && wheel_work_energy_control::negativeBoundaryWeight(state)<=0.f) ||
      !std::isfinite(target_mj) || target_mj<=0.f) return choice;
  const float initial_current_ma=static_cast<float>(side)*event.i0_estimated_mA;
  if(state.target_deg==12.f) {
    const lateral_energy::CurrentTable* tables[]={&lateral_current_table_,
        &positive_current_probe_tables_[0],&positive_current_probe_tables_[1]};
    const float goals[]={predictCurrentGoalMa(300,fast_v),predictCurrentGoalMa(350,fast_v),
        predictCurrentGoalMa(400,fast_v)};
    const uint16_t brake_cap=work_negative_feedback_.balancedCapMs(lateral_energy_geometry_,
        state.previous_peak_deg,run_cg_shift_mm_,12.f);
    choice=work_target12_controller_.choose(lateral_energy_geometry_,state,side,tables,goals,
        initial_current_ma,target_mj,run_cg_shift_mm_,brake_cap);
  } else if (side>0) {
    const float positive_bias_mj=work_positive_feedback_[1].positiveBiasMj(run_cg_shift_mm_,adaptive_cg_.snapshot().common_bias_mj,event.target_peak_deg);
    const float bridge_bias_mj=work_positive_feedback_[0].positiveBiasMj(run_cg_shift_mm_,0.f,event.target_peak_deg);
    const float weight=wheel_work_energy_control::positiveWorkWeight(state);
    const bool coast_only=event.target_peak_deg==12.f &&
        ((weight>0.f && work_positive_feedback_[1].coastOnly()) ||
         (weight<1.f && work_positive_feedback_[0].coastOnly()));
    const lateral_energy::CurrentTable* tables[] = {
        &lateral_current_table_,&positive_current_probe_tables_[0],
        &positive_current_probe_tables_[1]};
    const float goals[]={predictCurrentGoalMa(300,fast_v),predictCurrentGoalMa(350,fast_v),
        predictCurrentGoalMa(400,fast_v)};
    const uint16_t actual_cap=state.target_deg==8.f ?
        (wheel_speed_balance_.capMs()<positive_or_walking_cap_ms ? wheel_speed_balance_.capMs() : positive_or_walking_cap_ms) : 100;
    choice=wheel_work_energy_control::choosePositive(lateral_energy_geometry_,state,
        initial_current_ma,tables,goals,target_mj,positive_bias_mj,bridge_bias_mj,coast_only,actual_cap);

  } else {
    if(state.target_deg==8.f && state.previous_peak_deg<work_target_policy::reserveFloor(state.target_deg)) return choice;
    const uint16_t cap=work_negative_feedback_.balancedCapMs(lateral_energy_geometry_,state.previous_peak_deg,run_cg_shift_mm_,event.target_peak_deg);
    const float bias_mj=work_positive_feedback_[2].biasMj(run_cg_shift_mm_,adaptive_cg_.snapshot().common_bias_mj);
    const float assist_bias=work_positive_feedback_[work_target_policy::negativeFeedbackIndex(event.target_peak_deg,true)].biasMj(run_cg_shift_mm_,bias_mj);
    choice=wheel_work_energy_control::chooseNegativeBidirectional(state,cap,
        work_negative_assist_feedback_,lateral_current_table_,initial_current_ma,
        predictCurrentGoalMa(300,fast_v),target_mj,bias_mj,assist_bias);
  }
  wheel_work_energy_control::finish(choice,target_mj);
  if (choice.valid && side>0 && selected_current_ma && selected_current_table) {
    *selected_current_ma=choice.current_ma;
    *selected_current_table=choice.current_ma==350 ? &positive_current_probe_tables_[0] :
        choice.current_ma==400 ? &positive_current_probe_tables_[1] : &lateral_current_table_;
  }
  return choice;
}

wheel_work_energy_control::Choice ExperimentRunner::chooseFootPhaseWidth(
    const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    const wheel_work_energy_control::Choice& baseline,uint16_t width,float fast_v) const {
  using namespace wheel_work_energy_control;
  Choice c;
  const int16_t ma=baseline.current_ma;
  if(!baseline.valid || !foot_span_candidate::supportedBranch(baseline.branch) ||
     !foot_span_candidate::supportedCurrent(ma) || baseline.command_sign!=1 ||
     !std::isfinite(event.target_energy_j) || event.target_energy_j<=0.f)return c;
  State state{event.zero_cross_abs_rate_dps,event.pre_input_wheel_speed_rpm,
      static_cast<float>(event.vbat_mV),
      peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,event.pre_input_wheel_speed_valid,
          event.pre_input_wheel_speed_age_us),event.previous_peak_amplitude_deg};
  state.absolute_reference=true;state.target_deg=event.target_peak_deg;
  const lateral_energy::CurrentTable& table=ma==300 ? lateral_current_table_ :
      ma==350 ? positive_current_probe_tables_[0] : positive_current_probe_tables_[1];
  const float goal=predictCurrentGoalMa(ma,fast_v);
  // Match exactly the two biases used by chooseWheelWork. In the blended
  // branch a single already-blended bias cannot reconstruct both sources.
  const float work_bias=work_positive_feedback_[1].positiveBiasMj(run_cg_shift_mm_,
      adaptive_cg_.snapshot().common_bias_mj,event.target_peak_deg);
  const float bridge_bias=work_positive_feedback_[0].positiveBiasMj(run_cg_shift_mm_,0.f,event.target_peak_deg);
  return foot_span_candidate::choose(lateral_energy_geometry_,state,baseline,table,width,
      event.i0_estimated_mA,goal,event.target_energy_j*1000.f,work_bias,bridge_bias);
}

bool ExperimentRunner::prepareUnmodeledWalkCoast(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event) const {
  wheel_work_energy_control::State observed{event.zero_cross_abs_rate_dps,
      event.pre_input_wheel_speed_rpm,static_cast<float>(event.vbat_mV),
      peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
          event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us),
      event.previous_peak_amplitude_deg};
  observed.absolute_reference=true;observed.target_deg=event.target_peak_deg;
  const bool safe=!status_.emergency_stop && status_.state==ExperimentState::RUNNING_BATCH_SWEEP &&
      roller_ && roller_->ok() && imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      !event.side_mismatch_diagnostic;
  if(!unmodeled_walk_coast::eligible(observed,event.target_energy_j*1000.f,safe)) return false;
  event.work_control_branch=12;
  event.energy_model_extrapolated=true;
  event.passive_energy_j=NAN;event.free_next_peak_amplitude_deg=NAN;
  event.valid=true;event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT;
  return true;
}

bool ExperimentRunner::tryTiltPriority(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms) {
  if(optional_early_event_)return tryTiltPriorityOptionalEarly(event,t_test_ms);
  if(!stall_rescue_event_)return tryTiltPriorityImpl(event,t_test_ms,nullptr);
  return tryTiltPriorityRescue(event,t_test_ms);
}

bool ExperimentRunner::tryTiltPriorityOptionalEarly(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms) {
  // Do not overlap a speculative timing change with an independent foot or
  // sway experiment; the ordinary zero opportunity can still handle it.
  if(!long_foot_balance_.idleForNegativeTrial() || positive_sway_probe_.active() ||
     negative_cap_recovery_.active())return false;
  const auto old_tilt=tilt_priority_;
  const auto old_wheel=wheel_cycle_guard_;
  const auto old_negative=negative_cap_recovery_;
  const uint32_t old_foot_ms=foot_trial_pending_ms_,old_foot_generation=foot_trial_pending_generation_;
  const bool old_foot_effective=foot_trial_pending_effective_;
  const float old_q=status_.current_audit_q_target_mA_s,old_pred=status_.current_audit_q_pred_mA_s;
  bool healthy=false;
  const bool selected=tryTiltPriorityImpl(event,t_test_ms,&healthy);
  if(healthy && !event.output_executed) {
    tilt_priority_=old_tilt;wheel_cycle_guard_=old_wheel;negative_cap_recovery_=old_negative;
    foot_trial_pending_ms_=old_foot_ms;foot_trial_pending_generation_=old_foot_generation;
    foot_trial_pending_effective_=old_foot_effective;
    status_.current_audit_q_target_mA_s=old_q;status_.current_audit_q_pred_mA_s=old_pred;
  }
  return selected;
}

bool ExperimentRunner::tryTiltPriorityRescue(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms) {
  // Only speculative rescue attempts pay for snapshots. A failed ordinary
  // pulse keeps the established runner behavior and hot-path cost.
  const auto rescue_tilt_before=tilt_priority_;
  const auto rescue_wheel_before=wheel_cycle_guard_;
  bool healthy=false;
  const bool selected=tryTiltPriorityImpl(event,t_test_ms,&healthy);
  if(healthy && !event.output_executed) {
    tilt_priority_=rescue_tilt_before;
    wheel_cycle_guard_=rescue_wheel_before;
  }
  return selected;
}

bool ExperimentRunner::tryTiltPriorityImpl(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,bool* healthy_out) {
  foot_trial_pending_ms_=0;foot_trial_pending_effective_=false;
  tilt_priority::Input in;
  in.now_ms=t_test_ms;in.preset_deg=event.target_peak_deg;
  in.target_deg=tilt_priority_.target();in.next_side=event.physical_next_peak_side;
  in.crossing_rate_dps=event.zero_cross_abs_rate_dps;
  // Early geometry trial shares the rescue's 15 ms cap; normal ZERO is unchanged.
  in.hardware_cap_ms=(stall_rescue_event_ || optional_early_event_) ?
      stall_rescue::kMaxPulseMs : Config::WALKING_TARGET8_MAX_PULSE_MS;
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  // Relocate an ordinary proposal, never taper a 15 ms early proposal twice.
  if(foot_support_event_)in.hardware_cap_ms=Config::WALKING_TARGET8_MAX_PULSE_MS;
#endif
  in.hardware_healthy=wheelWorkMode() && !status_.emergency_stop &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP && !status_.pulse_active &&
      roller_ && roller_->ok() && imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      !event.side_mismatch_diagnostic && event.vbat_mV>0 &&
      std::isfinite(event.zero_cross_abs_rate_dps) && event.zero_cross_abs_rate_dps>0.f &&
      event.pre_input_wheel_speed_valid && event.pre_input_wheel_speed_age_us<=50000 &&
      std::isfinite(event.pre_input_wheel_speed_rpm);
  if(healthy_out)*healthy_out=in.hardware_healthy;
  if(!in.hardware_healthy) {wheel_cycle_guard_.reset();long_foot_balance_.reset();positive_sway_probe_.interrupt();}
#if defined(WALK_SMOOTH_TRIAL) && WALK_SMOOTH_TRIAL
  // All downstream proposals see this same optional upward ceiling.
  if(!stall_rescue_event_ && (!optional_early_event_
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
      || foot_support_event_
#endif
      )) {
    if(!in.hardware_healthy)tilt_priority_.ordinaryResult(in.next_side,t_test_ms,0,false);
    else in.hardware_cap_ms=tilt_priority_.ordinaryRiseCap(in.next_side,t_test_ms,in.hardware_cap_ms);
  }
#endif
  // The timeout and unhealthy hardware must cancel even on a positive input.
  // The v1.0.91 negative-side probe did not improve the observed run. Keep
  // its audit/observer code, but suppress new or continuing interventions
  // until its physical response can be identified independently.
  const bool negative_recovery_enabled=false;
  if(!negative_recovery_enabled)negative_cap_recovery_.interrupt();
  const uint16_t active_negative_extra=negative_recovery_enabled ?
      negative_cap_recovery_.extraCap(t_test_ms,in.hardware_healthy) : 0;
  auto choice=tilt_priority_.propose(in);
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  if(foot_support_event_ && foot_support_decision_.coast_assist) {
    // A foot-qualified, sustained inward slowdown can require force despite
    // a large preceding rocking peak. This explicitly overrides only a
    // normal coast proposal; it is a bounded, unidentified 15 ms trial.
    if(!choice.selected || !choice.coast)return false;
    choice.coast=false;choice.current_ma=1200;choice.width_ms=15;choice.proposed_width_ms=15;
    const uint16_t timing_cap=tilt_priority_.timingCap(in.next_side,t_test_ms,in.hardware_cap_ms);
    if(choice.width_ms>timing_cap)choice.width_ms=timing_cap;
  }
#endif
#if defined(SWAY_BALANCE_TRIAL) && SWAY_BALANCE_TRIAL
  if(!stall_rescue_event_ && (!optional_early_event_
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
      || foot_support_event_
#endif
      ) && choice.selected && !choice.coast) {
    const uint16_t trim_cap=tilt_priority_.balanceTrimCap(t_test_ms,in.next_side,choice.width_ms);
    if(trim_cap<choice.width_ms) {
      event.sway_trim_before_ms=choice.width_ms;
      event.sway_trim_cap_ms=trim_cap;
      if(in.hardware_cap_ms>trim_cap)in.hardware_cap_ms=trim_cap;
      // The proposal already includes the rate taper. Re-proposing with a
      // smaller hardware cap would taper a second time and remove >5 ms.
      choice.width_ms=trim_cap;
    }
  }
#endif
#if (defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL) || (defined(FAST_CROSSING_SHADOW) && FAST_CROSSING_SHADOW)
  if(!stall_rescue_event_ && !optional_early_event_ && choice.selected && !choice.coast &&
     !event.sway_trim_cap_ms) {
    const uint16_t trim_cap=tilt_priority_.fastCrossingTrimCap(t_test_ms,in.crossing_rate_dps,choice.width_ms);
    if(trim_cap<choice.width_ms) {
      event.fast_trim_before_ms=choice.width_ms;event.fast_trim_cap_ms=trim_cap;
#if defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL
      if(in.hardware_cap_ms>trim_cap)in.hardware_cap_ms=trim_cap;
      choice.width_ms=trim_cap;
#endif
    }
  }
#endif
  if(stall_rescue_event_)negative_cap_recovery_.interrupt();
  if((stall_rescue_event_ || optional_early_event_) && (!choice.selected || choice.coast))return false;
#if ANGLE_EARLY_TRIAL
  if(optional_early_event_
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
      && !foot_support_event_
#endif
      && tilt_priority_.deferEarlyWhenWeak(
      in,choice,Config::WALKING_TARGET8_MAX_PULSE_MS))return false;
#endif
  // Record every accepted healthy crossing, even when ordinary control is
  // selected. It will be paired with the corresponding measured peak.
  if(in.hardware_healthy)tilt_priority_.crossing(in.next_side,t_test_ms);
  if(!choice.selected) {
#if defined(WALK_SMOOTH_TRIAL) && WALK_SMOOTH_TRIAL
    if(!stall_rescue_event_ && !optional_early_event_)tilt_priority_.ordinaryResult(in.next_side,t_test_ms,0,false);
#endif
    negative_cap_recovery_.interrupt();if(in.next_side>0)positive_sway_probe_.interrupt();return false;}
  if(choice.coast && negative_cap_recovery_.active())negative_cap_recovery_.interrupt();
  // Preserve one owner for positive-width feedback. Evaluate its currently
  // applied setting and new proposal against the identical guard history.
  if(!stall_rescue_event_ && !optional_early_event_)weak_side_assist_.interrupt();
  bool foot_adjusted=false;
  bool foot_candidate_eligible=false;
  bool foot_path_active=false;
  uint16_t foot_guarded_current=0,foot_guarded_candidate=0;
  uint16_t foot_reference_baseline=0,foot_reference_cap=0;
  const auto wheel_guard_before=wheel_cycle_guard_;
  bool negative_probe_path=false,negative_probe_start=false,negative_probe_selected=false;
  uint16_t negative_guarded_baseline=0,negative_guarded_candidate=0;
  bool sway_probe_path=false,sway_probe_start=false,sway_probe_selected=false;
  uint16_t sway_guarded_baseline=0,sway_guarded_candidate=0;
  const bool sway_eligible=!stall_rescue_event_ && !optional_early_event_ && !choice.coast && in.next_side>0 &&
      in.target_deg==8.f && !negative_cap_recovery_.active() && long_foot_balance_.idleForNegativeTrial();
  if(in.next_side>0 && !optional_early_event_) {
    sway_probe_start=!positive_sway_probe_.active() && positive_sway_probe_.candidate(t_test_ms,in.hardware_healthy,sway_eligible);
    sway_probe_path=positive_sway_probe_.request(t_test_ms,in.hardware_healthy,sway_eligible) && sway_eligible;
  }
  if(sway_probe_path) {
    auto baseline_guard=wheel_guard_before;
    sway_guarded_baseline=baseline_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,choice.width_ms);
    auto trial_in=in;trial_in.positive_cap_extra_ms=5;
    const auto trial=tilt_priority_.propose(trial_in);
    auto candidate_guard=wheel_guard_before;
    if(trial.selected && !trial.coast && trial.direction==choice.direction && trial.current_ma==choice.current_ma) {
      sway_guarded_candidate=candidate_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
          in.hardware_healthy,t_test_ms,trial.width_ms);
      const int delta=static_cast<int>(sway_guarded_candidate)-static_cast<int>(sway_guarded_baseline);
      if(delta>0 && delta<=5) {choice=trial;choice.width_ms=sway_guarded_candidate;wheel_cycle_guard_=candidate_guard;sway_probe_selected=true;}
    }
    if(!sway_probe_selected){choice.width_ms=sway_guarded_baseline;wheel_cycle_guard_=baseline_guard;}
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::PositiveSwayProbe,true);
    event.weak_probe_baseline_ms=static_cast<uint8_t>(sway_guarded_baseline);
    event.weak_probe_requested_ms=static_cast<uint8_t>(sway_guarded_candidate);
  } else if(!stall_rescue_event_ && !optional_early_event_ && !choice.coast && in.next_side>0 &&
      choice.width_ms<=100 && tilt_priority_.target()>=6.f && !negative_cap_recovery_.active()) {
    foot_path_active=true;
    const uint16_t baseline=choice.width_ms;
    uint16_t cap=tilt_priority_.timingCap(in.next_side,t_test_ms,in.hardware_cap_ms);
    const uint16_t exploratory_cap=static_cast<uint16_t>(baseline+20u);
    if(cap>exploratory_cap)cap=exploratory_cap;
    if(cap>100)cap=100;
    foot_reference_baseline=baseline;foot_reference_cap=cap;
    const uint16_t proposed=long_foot_balance_.propose(t_test_ms,baseline,cap,in.hardware_healthy);
    const uint16_t current=long_foot_balance_.current(baseline,cap);
    auto ordinary_guard=wheel_guard_before;
    const uint16_t ordinary_guarded=ordinary_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,baseline);
    auto current_guard=wheel_guard_before;
    foot_guarded_current=current_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,current);
    auto candidate_guard=wheel_guard_before;
    foot_guarded_candidate=candidate_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,proposed);
    event.foot_span_baseline_ms=ordinary_guarded;
    event.foot_span_requested_ms=foot_guarded_candidate;
    const int guarded_change=static_cast<int>(foot_guarded_candidate)-static_cast<int>(foot_guarded_current);
    foot_candidate_eligible=foot_guarded_candidate>0 &&
        (long_foot_balance_.state()==long_foot_balance::Controller::State::Restore ||
         (guarded_change>=-5 && guarded_change<=5));
    if(foot_candidate_eligible) {
      choice.width_ms=foot_guarded_candidate;wheel_cycle_guard_=candidate_guard;
      foot_adjusted=foot_guarded_candidate!=ordinary_guarded;
    } else {
      choice.width_ms=foot_guarded_current;wheel_cycle_guard_=current_guard;
      foot_adjusted=foot_guarded_current!=ordinary_guarded;
    }
  } else if(!stall_rescue_event_ && !optional_early_event_ && !choice.coast && in.next_side<0) {
    negative_probe_start=negative_recovery_enabled && negative_cap_recovery_.candidate(t_test_ms,in.target_deg,
        tilt_priority_.widthLimit(-1),long_foot_balance_.idleForNegativeTrial(),in.hardware_healthy);
    negative_probe_path=negative_recovery_enabled && (negative_probe_start || (negative_cap_recovery_.active() && active_negative_extra>0));
    auto ordinary_guard=wheel_guard_before;
    negative_guarded_baseline=ordinary_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,choice.width_ms);
    wheel_cycle_guard_=ordinary_guard;
    if(negative_probe_path) {
      auto trial_in=in;trial_in.negative_cap_extra_ms=5;
      auto candidate=tilt_priority_.propose(trial_in);
      if(candidate.selected && !candidate.coast && candidate.direction==choice.direction &&
          candidate.current_ma==choice.current_ma) {
        auto candidate_guard=wheel_guard_before;
        negative_guarded_candidate=candidate_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
            in.hardware_healthy,t_test_ms,candidate.width_ms);
        event.weak_probe_baseline_ms=static_cast<uint8_t>(negative_guarded_baseline);
        event.weak_probe_requested_ms=static_cast<uint8_t>(negative_guarded_candidate);
        const int delta=static_cast<int>(negative_guarded_candidate)-static_cast<int>(negative_guarded_baseline);
        if(delta>0 && delta<=5) {
          choice=candidate;choice.width_ms=negative_guarded_candidate;
          wheel_cycle_guard_=candidate_guard;negative_probe_selected=true;
        } else choice.width_ms=negative_guarded_baseline;
      } else choice.width_ms=negative_guarded_baseline;
    } else choice.width_ms=negative_guarded_baseline;
  } else {
    choice.width_ms=wheel_cycle_guard_.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,choice.width_ms);
  }
  event.wheel_guard_legacy_ms=wheel_cycle_guard_.legacyWidth();
  event.wheel_guard_extra_ms=wheel_cycle_guard_.extraReduction();
  event.wheel_guard_trial_eligible=wheel_cycle_guard_.trialEligible();
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  if(foot_support_event_) {
    if(foot_support_active_audit_>=0)foot_support_audit_[foot_support_active_audit_].guarded_width=choice.width_ms;
    const uint32_t fresh_now=millis()-run_start_ms_;
    if(fresh_now<t_test_ms || fresh_now-t_test_ms>5 || static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000)return false;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
    const auto current=foot_support_pose::select(foot_pose_,foot_local_pose_,fresh_now);
    if(foot_pose_.run_id!=status_.run_id || !current.usable || current.source!=foot_support_source_)return false;
    auto check=foot_support_input_;
    const auto& fresh=current.pose;
    check.now_ms=fresh_now;check.pose_ready=fresh.ready();check.pose_confident=current.usable;
    check.left_deg=fresh.left;check.right_deg=fresh.right;
    check.left_uncertainty_deg=fresh.left_uncertainty;check.right_uncertainty_deg=fresh.right_uncertainty;
    check.local_returning_confirmed=current.local_returning_confirmed;
    // Only a <=5ms, fresh-IMU decision survives; account for that elapsed time
    // without pretending to know a future zero crossing or contact instant.
    check.body_angle_deg+=check.body_rate_dps*(fresh_now-t_test_ms)*.001f;
    const bool ld=fresh.left-fresh.left_uncertainty>=fresh.right-fresh.right_uncertainty;
    check.anticipated_shape_need=(ld?fresh.phase_left:fresh.phase_right)==foot_phase_prediction::Phase::Returning &&
        std::fabs(fresh.left-fresh.right)-fresh.left_uncertainty-fresh.right_uncertainty>=1.f &&
        energy_control_autonomous_last_peak_amplitude_deg_<=8.5f;
    const auto checked=foot_support_policy::decide(check);
    if(checked.coast_assist!=foot_support_decision_.coast_assist ||
       !foot_support_policy::widthPreserved(checked,choice.width_ms))return false;
    if(foot_support_active_audit_>=0){
      auto& audit=foot_support_audit_[foot_support_active_audit_];
      audit.mode=static_cast<uint8_t>(checked.mode);audit.stall_route=checked.stall_route;audit.checked_ms=fresh_now;
    }
#else
    const auto fresh=foot_pose_.predict(fresh_now,0);
    if(foot_pose_.run_id!=status_.run_id || !fresh.ready() || fresh.confidence<.5f ||
       fresh_now<t_test_ms || fresh_now-t_test_ms>5 ||
       static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000 ||
       foot_support_decision_.estimated_zero_ms<choice.width_ms+20.f+(fresh_now-t_test_ms) ||
       fresh.age_ms>foot_support_policy::kMaxFrameAgeMs || fresh.left_uncertainty>.75f || fresh.right_uncertainty>.75f ||
       !foot_support_policy::widthPreserved(foot_support_decision_,choice.width_ms))return false;
#endif
  }
#endif
  event.work_control_branch=15;event.energy_model_extrapolated=true;
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  if(foot_support_event_)event.work_control_branch=16;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  if(foot_support_event_)event.work_control_branch=17;
#endif
#endif
  event.work_prediction_cross_condition=true;
  event.work_brake_cap_ms=choice.width_ms;
  event.solver_selected_integer_width_ms=choice.width_ms;
  event.passive_energy_j=NAN;event.free_next_peak_amplitude_deg=NAN;
  event.predicted_energy_j=NAN;event.predicted_observed_energy_j=NAN;
  event.predicted_next_peak_amplitude_deg=NAN;event.predicted_observed_next_peak_deg=NAN;
  event.predicted_wheel_work_j=NAN;event.predicted_input_work_j=NAN;
  event.predicted_post_input_work_j=NAN;event.predicted_work_change_j=NAN;
  event.q_command_mA_s=choice.coast ? 0.f : NAN;event.q_effective_pred_mA_s=NAN;
  event.predicted_current_goal_mA=NAN;event.predicted_current_rise_tau_s=NAN;
  event.aligned_initial_current_mA=NAN;
  status_.current_audit_q_target_mA_s=NAN;status_.current_audit_q_pred_mA_s=NAN;
  event.q_command_direction=choice.coast ? 0 : choice.direction;
  event.command_matches_zero_cross_motion=!choice.coast;
  event.command_current_mA=choice.current_ma;event.pulse_width_ms=choice.width_ms;
  event.pulse_start_ms=choice.coast ? 0 : t_test_ms;
  event.pulse_end_ms=choice.coast ? 0 : t_test_ms+choice.width_ms;
  event.valid=false;event.output_executed=false;
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  if(foot_support_event_) {
    // Check again immediately before the only actuator call. A preempted
    // speculative decision must leave normal ZERO available.
    const uint32_t fresh_now=millis()-run_start_ms_;
    if(fresh_now<t_test_ms || fresh_now-t_test_ms>5 ||
       static_cast<uint32_t>(micros()-foot_support_sample_us_)>10000)return false;
  }
#endif
  if(!choice.coast && !beginEnergyControlAutonomousPulse(run_start_ms_+t_test_ms,t_test_ms,
                                        choice.direction,choice.width_ms,choice.current_ma)) {
#if defined(WALK_SMOOTH_TRIAL) && WALK_SMOOTH_TRIAL
    if(!stall_rescue_event_ && !optional_early_event_)tilt_priority_.ordinaryResult(in.next_side,t_test_ms,0,false);
#endif
    if(foot_path_active || negative_probe_path || sway_probe_path)wheel_cycle_guard_=wheel_guard_before;
    if(sway_probe_path)positive_sway_probe_.interrupt();
    if(negative_probe_path)negative_cap_recovery_.command(t_test_ms,negative_guarded_baseline,
        negative_guarded_candidate,false,negative_probe_start);
    if(in.next_side<0) {
      const unsigned recovery_state=negative_cap_recovery_.stateCode();
      event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::NegativeRecoveryStateBit0,(recovery_state&1u)!=0);
      event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::NegativeRecoveryStateBit1,(recovery_state&2u)!=0);
    }
    tilt_priority_.confirm(in.next_side,t_test_ms,choice,false);
    event.command_current_mA=0;event.pulse_width_ms=0;
    event.q_command_direction=0;event.command_matches_zero_cross_motion=false;
    event.pulse_start_ms=0;event.pulse_end_ms=0;
    event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    return true;
  }
  if((stall_rescue_event_ || optional_early_event_) && !choice.coast)weak_side_assist_.interrupt();
#if defined(WALK_SMOOTH_TRIAL) && WALK_SMOOTH_TRIAL
  if(!stall_rescue_event_ && !optional_early_event_)
    tilt_priority_.ordinaryResult(in.next_side,t_test_ms,choice.width_ms,!choice.coast);
#endif
  tilt_priority_.confirm(in.next_side,t_test_ms,choice,true,negative_probe_selected || sway_probe_selected || optional_early_event_,
      !optional_early_event_ && !sway_probe_path && !negative_probe_path && !foot_adjusted &&
      !(foot_candidate_eligible && foot_guarded_candidate!=foot_guarded_current) && !long_foot_balance_.trialActive());
  if(sway_probe_path)positive_sway_probe_.confirm(t_test_ms,sway_guarded_baseline,choice.width_ms,
      !choice.coast && sway_probe_selected,sway_probe_start);
  if(sway_probe_path)event.weak_probe_executed=sway_probe_selected;
  // Clear pre-probe foot reference when (and only when) the changed command
  // was sent. Post-probe evidence must be collected entirely afresh.
  if(sway_probe_start && sway_probe_selected)long_foot_balance_.reset();
  if(negative_probe_path) {
    negative_cap_recovery_.command(t_test_ms,negative_guarded_baseline,choice.width_ms,
        negative_probe_selected,negative_probe_start);
    event.weak_probe_executed=negative_probe_selected;
    if(negative_probe_selected)negative_recovery_last_applied_ms_=t_test_ms;
    if(negative_probe_start && negative_probe_selected)long_foot_balance_.reset();
  }
  if(in.next_side<0) {
    const unsigned recovery_state=negative_cap_recovery_.stateCode();
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::NegativeRecoveryStateBit0,(recovery_state&1u)!=0);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::NegativeRecoveryStateBit1,(recovery_state&2u)!=0);
  }
  if(foot_candidate_eligible)long_foot_balance_.confirm(t_test_ms,foot_guarded_current,foot_guarded_candidate,!choice.coast);
  if(foot_path_active && !choice.coast && long_foot_balance_.trialActive()) {
    // confirm can begin a new trial. Resolve its *confirmed* reference only
    // afterward, using the exact guard history from before this command.
    int reference=static_cast<int>(foot_reference_baseline)+long_foot_balance_.trialReferenceOffset();
    if(reference<0)reference=0;
    if(reference>foot_reference_cap)reference=foot_reference_cap;
    auto reference_guard=wheel_guard_before;
    const uint16_t foot_guarded_reference=reference_guard.limit(in.next_side,event.pre_input_wheel_speed_rpm,
        in.hardware_healthy,t_test_ms,static_cast<uint16_t>(reference));
    const int expected=long_foot_balance_.offset()-long_foot_balance_.trialReferenceOffset();
    const int delivered=static_cast<int>(choice.width_ms)-static_cast<int>(foot_guarded_reference);
    foot_trial_pending_ms_=t_test_ms;foot_trial_pending_generation_=long_foot_balance_.trialGeneration();
    foot_trial_pending_effective_=delivered>=-5 && delivered<=5 &&
        ((expected>0 && delivered>0) || (expected<0 && delivered<0));
    // Reuse the inactive 1.0.88 weak-probe fields: metadata identifies them
    // as the guarded trial-reference and accepted width in this version.
    event.weak_probe_baseline_ms=static_cast<uint8_t>(foot_guarded_reference);
    event.weak_probe_requested_ms=static_cast<uint8_t>(choice.width_ms);
    event.weak_probe_executed=foot_trial_pending_effective_;
  }
  event.valid=true;event.output_executed=!choice.coast;
  event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_selected,
      foot_adjusted && event.output_executed);
  event.reason=choice.coast ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT
                            : Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  pending_cg_shift_mm_=run_cg_shift_mm_;pending_target_energy_j_=event.target_energy_j;
  energy_control_autonomous_pending_peak_=true;energy_control_autonomous_pending_probe_=true;
  energy_control_autonomous_pending_next_side_=in.next_side;
  energy_control_autonomous_pending_target_deg_=choice.target_deg;
  energy_control_autonomous_pending_q_command_mA_s_=choice.coast ? 0.f : NAN;
  energy_control_autonomous_pending_model_energy_j_=NAN;
  energy_control_autonomous_pending_wheel_rpm_=event.pre_input_wheel_speed_rpm;
  energy_control_autonomous_pending_wheel_age_us_=event.pre_input_wheel_speed_age_us;
  energy_control_autonomous_pending_wheel_valid_=event.pre_input_wheel_speed_valid;
  energy_control_autonomous_pending_zero_ms_=event.zero_cross_time_ms;
  energy_control_autonomous_pending_mode_=choice.coast ? peak_error_adaptation::Coast : peak_error_adaptation::Excite;
  energy_control_autonomous_pending_saturated_upper_=false;
  energy_control_autonomous_pending_saturated_lower_=false;
  work_positive_pending_=false;work_negative_pending_=false;
  work_walking_pending_=false;work_walking_low_pending_=false;
  walking_loss_feedback_.interrupt();walking_low_speed_guard_.interrupt();adaptive_cg_.breakPair();
  return true;
}

bool ExperimentRunner::tryWeakSideAssist(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    const wheel_work_energy_control::Choice& normal,bool unmodeled_coast,
    uint32_t t_test_ms,float fast_v) {
  // Never turn an invalid selector into a pulse unless the existing explicit
  // unmodeled-coast classifier has already confirmed a healthy finite state.
  if(!wheelWorkMode() || event.physical_next_peak_side!=-1 ||
      (!normal.valid && !unmodeled_coast) ||
      (normal.valid && normal.width_ms!=0)) { weak_side_assist_.interrupt(); return false; }

  weak_side_assist::Input in;
  in.target_preset_deg=event.target_peak_deg;
  in.active_target_deg=gradual_start_.target();
  in.physical_next_side=event.physical_next_peak_side;
  in.now_ms=t_test_ms;
  for(const auto& p:phase_negative_peaks_) {
    if(!p.ms || p.ms>=t_test_ms || !std::isfinite(p.amplitude) || p.amplitude<=0.f)continue;
    ++in.negative_peak_count;
    if(p.ms>in.last_negative_peak_ms) {
      in.last_negative_peak_ms=p.ms;in.last_negative_peak_deg=p.amplitude;
    }
  }
  in.last_positive_peak_deg=event.previous_peak_side>0 ? event.previous_peak_amplitude_deg : NAN;
  in.crossing_rate_dps=event.zero_cross_abs_rate_dps;
  in.wheel_rpm=event.pre_input_wheel_speed_rpm;
  in.battery_mv=static_cast<float>(event.vbat_mV);
  in.hardware_healthy=!status_.emergency_stop && status_.state==ExperimentState::RUNNING_BATCH_SWEEP &&
      !status_.pulse_active && roller_ && roller_->ok() && imu_ && imu_->acquisitionHealthy() &&
      !imu_->stale(millis()) && !event.side_mismatch_diagnostic && std::isfinite(event.target_energy_j) &&
      event.target_energy_j>0.f && event.previous_peak_time_ms>0 &&
      t_test_ms>=event.previous_peak_time_ms && t_test_ms-event.previous_peak_time_ms<=1500;
  in.speed_fresh=peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
      event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us);
  in.selector_valid=normal.valid;in.selector_width_ms=normal.width_ms;
  // Infer available time from the most recently completed negative half-cycle.
  // Decisions are timestamped at actual scheduling; peaks carry measurement
  // timestamps. Never treat a missing, stale or implausible pair as evidence
  // that a long pulse fits. Ordinary coast/selector continues in that case.
  uint32_t preceding_negative_ms=0;
  for(const auto& d:phase_negative_decisions_) {
    if(d.ms && d.ms<in.last_negative_peak_ms && d.ms>preceding_negative_ms)
      preceding_negative_ms=d.ms;
  }
  if(!preceding_negative_ms || !in.last_negative_peak_ms ||
     t_test_ms-in.last_negative_peak_ms>1500) {weak_side_assist_.interrupt();return false;}
  const uint32_t measured_to_peak_ms=in.last_negative_peak_ms-preceding_negative_ms;
  if(measured_to_peak_ms<=30 || measured_to_peak_ms>600) {
    weak_side_assist_.interrupt();return false;
  }
  const uint32_t dynamic_cap_ms=measured_to_peak_ms-30;
  in.hardware_max_width_ms=static_cast<uint16_t>(
      dynamic_cap_ms<Config::WALKING_TARGET8_MAX_PULSE_MS ? dynamic_cap_ms : Config::WALKING_TARGET8_MAX_PULSE_MS);
  const auto assist=weak_side_assist_.propose(in);
  if(!assist.selected)return false;

  // Charge is an actuator-model estimate, not a measured current integral.
  // Match the ordinary negative excitatory command's motion-aligned signs.
  const bool modeled_width=assist.width_ms<=Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS;
  const float initial_ma=modeled_width ? static_cast<float>(event.physical_next_peak_side)*event.i0_estimated_mA : NAN;
  const float goal_ma=modeled_width ? predictCurrentGoalMa(assist.current_ma,fast_v) : NAN;
  const float q=modeled_width ? lateral_energy::charge(lateral_current_table_,assist.width_ms,initial_ma,goal_ma) : NAN;
  if((modeled_width && (!std::isfinite(q) || q<0.f || q>25.f)) || assist.width_ms==0 ||
      assist.width_ms>in.hardware_max_width_ms || assist.width_ms>Config::WALKING_TARGET8_MAX_PULSE_MS) {
    weak_side_assist_.interrupt(); return false;
  }

  // Branch 13 stores planned width separately from actual pulse_width_ms;
  // a failed send retains this proposal but records actual width/current 0.
  event.solver_selected_integer_width_ms=assist.width_ms;
  event.work_control_branch=13;
  event.energy_model_extrapolated=true;event.work_prediction_cross_condition=true;
  event.work_brake_cap_ms=weak_side_assist_.limitMs()<in.hardware_max_width_ms
      ? weak_side_assist_.limitMs() : in.hardware_max_width_ms;
  event.passive_energy_j=NAN;event.free_next_peak_amplitude_deg=NAN;
  event.predicted_energy_j=NAN;event.predicted_observed_energy_j=NAN;
  event.predicted_next_peak_amplitude_deg=NAN;event.predicted_observed_next_peak_deg=NAN;
  event.predicted_wheel_work_j=NAN;event.predicted_input_work_j=NAN;
  event.predicted_post_input_work_j=NAN;event.predicted_work_change_j=NAN;
  event.q_command_direction=event.physical_next_peak_side*assist.motion_sign;
  event.command_matches_zero_cross_motion=true;
  event.q_command_mA_s=q;event.q_effective_pred_mA_s=q;
  event.predicted_current_goal_mA=goal_ma;event.predicted_current_rise_tau_s=modeled_width ? lateral_current_table_.tau : NAN;
  event.aligned_initial_current_mA=initial_ma;
  event.command_current_mA=assist.current_ma;event.pulse_width_ms=assist.width_ms;
  event.pulse_start_ms=t_test_ms;event.pulse_end_ms=t_test_ms+assist.width_ms;
  event.valid=false;event.output_executed=false;
  status_.current_audit_q_target_mA_s=q;status_.current_audit_q_pred_mA_s=q;
  const uint32_t now_ms=run_start_ms_+t_test_ms;
  if(!beginEnergyControlAutonomousPulse(now_ms,t_test_ms,event.q_command_direction,
                                        assist.width_ms,assist.current_ma)) {
    weak_side_assist_.interrupt();
    event.command_current_mA=0;event.pulse_width_ms=0;
    event.pulse_start_ms=0;event.pulse_end_ms=0;
    event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    return true;
  }
  weak_side_assist_.confirm(in,assist.width_ms);
  event.valid=true;event.output_executed=true;event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  // Record the actual command for the matching next physical negative peak,
  // without fabricating an arrival prediction or feeding any fitted model.
  pending_cg_shift_mm_=run_cg_shift_mm_;
  pending_target_energy_j_=event.target_energy_j;
  energy_control_autonomous_pending_peak_=true;
  energy_control_autonomous_pending_probe_=true;
  energy_control_autonomous_pending_next_side_=event.physical_next_peak_side;
  // Pair with the active ramped target whose energy was actually requested,
  // not merely the operator's eventual 8-degree preset.
  energy_control_autonomous_pending_target_deg_=in.active_target_deg;
  energy_control_autonomous_pending_q_command_mA_s_=q;
  energy_control_autonomous_pending_model_energy_j_=NAN;
  energy_control_autonomous_pending_wheel_rpm_=event.pre_input_wheel_speed_rpm;
  energy_control_autonomous_pending_wheel_age_us_=event.pre_input_wheel_speed_age_us;
  energy_control_autonomous_pending_wheel_valid_=event.pre_input_wheel_speed_valid;
  energy_control_autonomous_pending_zero_ms_=event.zero_cross_time_ms;
  energy_control_autonomous_pending_mode_=peak_error_adaptation::Excite;
  energy_control_autonomous_pending_saturated_upper_=false;
  energy_control_autonomous_pending_saturated_lower_=false;
  work_positive_pending_=false;work_negative_pending_=false;
  work_walking_pending_=false;work_walking_low_pending_=false;
  walking_loss_feedback_.interrupt();walking_low_speed_guard_.interrupt();adaptive_cg_.breakPair();
  return true;
}

bool ExperimentRunner::selectFootPhaseOutput(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,float fast_v,
    wheel_work_energy_control::Choice& work_choice,int16_t& selected_current_mA,
    const lateral_energy::CurrentTable*& selected_current_table) {
  (void)t_test_ms;(void)fast_v;(void)selected_current_mA;(void)selected_current_table;
  // The long-term controller owns branch 15 only. Fallback remains ordinary
  // control while retaining the established stale sensor and hardware gate.
  if(wheelWorkMode() && event.target_peak_deg==8.f && event.physical_next_peak_side>0) {
    negative_cap_recovery_.interrupt();
    long_foot_balance_.reset();
    const bool healthy=peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
        event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us) &&
        event.vbat_mV>0 && !event.side_mismatch_diagnostic && !status_.emergency_stop &&
        status_.state==ExperimentState::RUNNING_BATCH_SWEEP && roller_ && roller_->ok() &&
        imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis());
    if(!healthy) {
      event.reason=status_.emergency_stop || status_.state!=ExperimentState::RUNNING_BATCH_SWEEP
          ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE
          : (!roller_ || !roller_->ok() ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY
                                       : Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE);
      return true;
    }
  }
  (void)work_choice;
  return false;
}

void ExperimentRunner::choosePeakMpc(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    float fast_v,wheel_work_energy_control::Choice& work_choice,int16_t& selected_current_mA,
    const lateral_energy::CurrentTable*& selected_current_table) const {
  if (event.target_peak_deg==8.f && work_choice.valid && !work_choice.low_peak_selected &&
      work_choice.command_sign==1 && work_choice.width_ms>=30 &&
      ((event.physical_next_peak_side>0 && work_choice.current_ma==400) ||
       (event.physical_next_peak_side<0 && work_choice.branch==10))) {
    bool alternate_supported=false;
    int16_t alternate_current=selected_current_mA;
    const lateral_energy::CurrentTable* alternate_table=selected_current_table;
    const auto alternative=chooseWheelWork(event,fast_v,&alternate_supported,
        &alternate_current,&alternate_table,work_choice.width_ms-5);
    if(alternative.valid && alternative.command_sign==work_choice.command_sign &&
       alternative.current_ma==work_choice.current_ma) {
      const peak_mpc::State state{event.zero_cross_abs_rate_dps,event.pre_input_wheel_speed_rpm,
          event.previous_peak_amplitude_deg,static_cast<float>(event.vbat_mV),event.physical_next_peak_side};
      if(peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us)) {
        const int next_side=-event.physical_next_peak_side;
        const float next_energy=controlPotential(next_side*gradual_start_.target())*foot_energy_.factor(next_side);
        const float next_target=controlAngle(next_energy,next_side);
        const auto result=peak_mpc::choose(state,work_choice.width_ms,alternative.width_ms,event.foot_balance_target_deg,next_target);
        if(result.valid) {
          event.mpc_valid=true;event.mpc_alternate=result.alternate;
          event.mpc_second_peak_deg=result.second_peak;event.mpc_speed_proxy_rpm=result.mean_rpm;
          if(result.alternate) {work_choice=alternative;selected_current_mA=alternate_current;selected_current_table=alternate_table;}
        }
      }
    }
  }
}

bool ExperimentRunner::tryPositiveLeftBoost(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    const wheel_work_energy_control::Choice& normal,uint32_t t_test_ms) {
  positive_left_input_=positive_left_boost::Input{};
  auto& in=positive_left_input_;
  in.now_ms=t_test_ms;in.preset_deg=event.target_peak_deg;
  in.side_target_deg=event.foot_balance_target_deg;in.next_side=event.physical_next_peak_side;
  in.opposite_peak_ms=event.previous_peak_side<0 ? event.previous_peak_time_ms : 0;
  in.opposite_peak_deg=event.previous_peak_side<0 ? event.previous_peak_amplitude_deg : NAN;
  in.wheel_rpm=event.pre_input_wheel_speed_rpm;in.battery_mv=static_cast<float>(event.vbat_mV);
  in.normal_valid=normal.valid;in.normal_motion_aligned=normal.command_sign==1;
  in.normal_ma=normal.current_ma;in.normal_width_ms=normal.width_ms;in.normal_cap_ms=normal.allowed_cap_ms;
  in.hardware_cap_ms=Config::WALKING_TARGET8_MAX_PULSE_MS;
  in.speed_fresh=peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
      event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us);
  in.hardware_healthy=wheelWorkMode() && !status_.emergency_stop &&
      status_.state==ExperimentState::RUNNING_BATCH_SWEEP && !status_.pulse_active &&
      roller_ && roller_->ok() && imu_ && imu_->acquisitionHealthy() && !imu_->stale(millis()) &&
      !event.side_mismatch_diagnostic && std::isfinite(event.target_energy_j) && event.target_energy_j>0.f;
  const auto trial=positive_left_boost_.propose(in);
  if(!trial.selected)return false;
  // Clear a queued foot-span proposal before an independently controlled
  // 500mA trial. Do not feed this input to the 400mA prediction learners.
  if(!foot_span_control_.selectorUnavailable(t_test_ms)) {
    positive_left_boost_.interrupt();requestEmergencyStop("foot_span_command_unresolved");return true;
  }
  event.work_control_branch=14;event.energy_model_extrapolated=true;
  event.work_prediction_cross_condition=true;event.work_brake_cap_ms=in.hardware_cap_ms;
  event.solver_required_width_ms=static_cast<float>(normal.width_ms);
  event.solver_selected_integer_width_ms=trial.width_ms;
  event.passive_energy_j=NAN;event.free_next_peak_amplitude_deg=NAN;
  event.predicted_energy_j=NAN;event.predicted_observed_energy_j=NAN;
  event.predicted_next_peak_amplitude_deg=NAN;event.predicted_observed_next_peak_deg=NAN;
  event.predicted_wheel_work_j=NAN;event.predicted_input_work_j=NAN;
  event.predicted_post_input_work_j=NAN;event.predicted_work_change_j=NAN;
  event.q_command_mA_s=NAN;event.q_effective_pred_mA_s=NAN;
  status_.current_audit_q_target_mA_s=NAN;status_.current_audit_q_pred_mA_s=NAN;
  event.q_command_direction=1;event.command_matches_zero_cross_motion=true;
  event.command_current_mA=trial.current_ma;event.pulse_width_ms=trial.width_ms;
  event.pulse_start_ms=t_test_ms;event.pulse_end_ms=t_test_ms+trial.width_ms;
  event.valid=false;event.output_executed=false;
  if(!beginEnergyControlAutonomousPulse(run_start_ms_+t_test_ms,t_test_ms,1,
                                         trial.width_ms,trial.current_ma)) {
    event.command_current_mA=0;event.pulse_width_ms=0;event.pulse_start_ms=0;event.pulse_end_ms=0;
    event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    positive_left_boost_.confirmTrial(in,0,0,1,false);return true;
  }
  event.valid=true;event.output_executed=true;event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  positive_left_boost_.confirmTrial(in,trial.current_ma,trial.width_ms,1,true);
  pending_cg_shift_mm_=run_cg_shift_mm_;pending_target_energy_j_=event.target_energy_j;
  energy_control_autonomous_pending_peak_=true;energy_control_autonomous_pending_probe_=true;
  energy_control_autonomous_pending_next_side_=1;
  energy_control_autonomous_pending_target_deg_=in.side_target_deg;
  energy_control_autonomous_pending_q_command_mA_s_=NAN;
  energy_control_autonomous_pending_model_energy_j_=NAN;
  energy_control_autonomous_pending_zero_ms_=event.zero_cross_time_ms;
  energy_control_autonomous_pending_wheel_rpm_=event.pre_input_wheel_speed_rpm;
  energy_control_autonomous_pending_wheel_age_us_=event.pre_input_wheel_speed_age_us;
  energy_control_autonomous_pending_wheel_valid_=event.pre_input_wheel_speed_valid;
  energy_control_autonomous_pending_saturated_upper_=false;
  energy_control_autonomous_pending_saturated_lower_=false;
  work_positive_pending_=false;work_negative_pending_=false;
  work_walking_pending_=false;work_walking_low_pending_=false;
  adaptive_cg_.breakPair();
  return true;
}

void ExperimentRunner::applyFootEnergyTarget(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_ms) {
  (void)t_ms;
  const int side=event.physical_next_peak_side;
  const float side_target=tilt_priority_.target();
  const float base=controlPotential(side*side_target);
  event.target_energy_j=base; // phase cap owns 8-degree foot balancing
  event.foot_balance_target_deg=side_target;
  event.foot_energy_bias=foot_energy_.bias();
  event.foot_energy_left_px=foot_energy_.leftMean();event.foot_energy_right_px=foot_energy_.rightMean();
  event.foot_energy_valid=foot_energy_.valid();event.foot_energy_phase=foot_energy_.phase();event.foot_energy_reason=foot_energy_.reason();
  event.low_peak_bias_mj=low_peak_feedback_.bias();event.low_peak_active=low_peak_feedback_.active();
}

void ExperimentRunner::capturePendingWheelWork(
    const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    const wheel_work_energy_control::Choice& work_choice,uint16_t selected_width_ms,float prediction_bias_j) {
  work_positive_pending_=wheelWorkMode() && event.physical_next_peak_side>0;
  work_positive_pending_branch_=work_choice.branch==6 ? 1 : work_choice.branch==7 ? 2 : 0;
  work_positive_low_applied_bias_mj_=work_choice.low_peak_selected ? work_choice.low_peak_applied_bias_mj : 0.f;
  work_positive_pending_raw_j_=work_positive_pending_ ?
      event.predicted_energy_j-(work_choice.valid ? prediction_bias_j : 0.f) : NAN;
  work_negative_pending_=wheelWorkMode() && work_choice.valid && event.physical_next_peak_side<0;
  work_walking_pending_=work_negative_pending_ && (work_choice.branch==10 || work_choice.branch==11);
  work_walking_low_pending_=work_walking_pending_ && work_choice.branch==11;
  work_negative_pending_assist_=work_negative_pending_ && work_choice.branch==9;
  work_negative_pending_supported_=work_negative_pending_ && work_choice.support_weight>=.5f;
  work_negative_pending_width_ms_=work_negative_pending_ ? selected_width_ms : 0;
  work_negative_pending_bias_j_=work_negative_pending_ ? prediction_bias_j : 0.f;
  work_pending_battery_mv_=event.vbat_mV;
  work_pending_cap_ms_=work_choice.allowed_cap_ms;
  wheel_work_energy_control::State pending_state{event.zero_cross_abs_rate_dps,
      event.pre_input_wheel_speed_rpm,static_cast<float>(event.vbat_mV),
      event.pre_input_wheel_speed_valid,event.previous_peak_amplitude_deg};
  pending_state.target_deg=event.target_peak_deg;pending_state.absolute_reference=true;
  work_pending_calibration_weight_=target12_state_work::weight(pending_state,event.physical_next_peak_side);
}

void ExperimentRunner::finishFootEnergyDecision(PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_ms) {
  if(wheelWorkMode() && event.physical_next_peak_side<0 &&
      (!event.valid || (event.work_control_branch!=10 && event.work_control_branch!=11)))
    walking_low_speed_guard_.interrupt();
  if(wheelWorkMode() && event.target_peak_deg==8.f) {
    if(event.physical_next_peak_side<0) {
      auto& d=phase_negative_decisions_[phase_negative_decision_next_];
      negative_preceding_positive_peaks_[phase_negative_decision_next_]=
          event.previous_peak_side>0 ? event.previous_peak_amplitude_deg : NAN;
      negative_preceding_positive_peak_ms_[phase_negative_decision_next_]=
          event.previous_peak_side>0 ? event.previous_peak_time_ms : 0;
      d.ms=t_ms;
      d.value=foot_phase_cap::classifyNegative(event.valid,event.output_executed,
          event.reason==Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE,
          event.reason==Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT,
          event.command_current_mA,event.q_command_direction,event.pulse_width_ms,event.work_control_branch);
      d.trigger_deg=std::isfinite(event.detector_angle_before_deg) &&
          std::isfinite(event.detector_angle_after_deg) && std::isfinite(event.detector_crossing_alpha)
          ? event.detector_angle_before_deg+event.detector_crossing_alpha*
              (event.detector_angle_after_deg-event.detector_angle_before_deg) : NAN;
      d.rate_dps=event.zero_cross_abs_rate_dps;
      d.rpm=event.pre_input_wheel_speed_valid && event.pre_input_wheel_speed_age_us<=50000
          ? event.pre_input_wheel_speed_rpm : NAN;
      phase_negative_decision_next_=(phase_negative_decision_next_+1)%8;
    } else if(event.physical_next_peak_side>0) {
      if(event.work_control_branch!=14 && positive_left_input_.now_ms==t_ms)
        positive_left_boost_.confirmNormal(positive_left_input_,event.command_current_mA,
            event.pulse_width_ms,event.q_command_direction,event.valid && event.output_executed);
      auto& d=span_positive_decisions_[span_positive_decision_next_];
      auto& context=span_positive_context_[span_positive_decision_next_];
      span_positive_branch_[span_positive_decision_next_]=event.work_control_branch;
      d.ms=t_ms;d.width=event.pulse_width_ms;d.ma=event.command_current_mA;
      d.sign=event.q_command_direction;d.target_mj=event.target_energy_j*1000.f;
      context.rate_dps=event.zero_cross_abs_rate_dps;
      context.rpm=event.pre_input_wheel_speed_valid && event.pre_input_wheel_speed_age_us<=50000
          ? event.pre_input_wheel_speed_rpm : NAN;
      d.executed=event.output_executed;
      foot_trial_generation_[span_positive_decision_next_]=
          foot_trial_pending_ms_==t_ms && event.output_executed ? foot_trial_pending_generation_ : 0;
      foot_trial_effective_[span_positive_decision_next_]=
          foot_trial_pending_ms_==t_ms && event.output_executed && foot_trial_pending_effective_;
      span_positive_decision_next_=(span_positive_decision_next_+1)%8;

    }
    // Legacy signed-phase event fields are no longer persisted; raw foot
    // frames and body peak records remain available for offline reconstruction.
    const auto& span=foot_span_observer_.latest();
    event.foot_span_left_deg=span.left.span;event.foot_span_right_deg=span.right.span;
    event.foot_span_trimmed_left_deg=span.left.trimmed_span;event.foot_span_trimmed_right_deg=span.right.trimmed_span;
    event.foot_span_start_ms=span.start_ms;event.foot_span_end_ms=span.end_ms;
    event.foot_span_state=static_cast<uint8_t>(long_foot_balance_.state());
    event.foot_span_accepted_cycles=static_cast<uint8_t>(long_foot_balance_.samples());
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_startup, span.startup);event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_continuous, span.continuous);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_left_low, span.left.low_signal);event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_right_low, span.right.low_signal);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_left_outlier, span.left.single_outlier);event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_right_outlier, span.right.single_outlier);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_left_edge_unverified, span.left.edge_unverified);event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_right_edge_unverified, span.right.edge_unverified);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_left_both, span.left.both_directions);event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_right_both, span.right.both_directions);
    event.setFootSpanFlag(PsramLogger::EnergyControlAutonomousZeroCrossEvent::Span_valid, span.valid && span.continuous &&
        foot_span_control::spanReliable(span.left.span,span.left.trimmed_span) &&
        foot_span_control::spanReliable(span.right.span,span.right.trimmed_span) && !span.left.single_outlier && !span.right.single_outlier &&
        !span.left.edge_unverified && !span.right.edge_unverified &&
        (span.left.both_directions || span.left.low_signal) && (span.right.both_directions || span.right.low_signal));
    const bool supported=event.valid && event.output_executed && event.work_control_branch!=0 &&
      event.command_matches_zero_cross_motion && !event.q_saturated_upper && !event.q_saturated_lower &&
      !event.energy_model_extrapolated && peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
        event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us);
    foot_energy_.decision(t_ms,event.physical_next_peak_side,supported,event.q_command_mA_s,
        event.pre_input_wheel_speed_rpm,event.vbat_mV);
    event.foot_energy_phase=foot_energy_.phase();event.foot_energy_reason=foot_energy_.reason();
  }
  if(wheelWorkMode() && event.target_peak_deg==8.f && event.physical_next_peak_side>0)
    low_peak_feedback_.command(t_ms,event.physical_next_peak_side,event.output_executed,event.command_current_mA,
        event.pulse_width_ms,event.work_control_branch,event.zero_cross_abs_rate_dps,
        peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,event.pre_input_wheel_speed_valid,
          event.pre_input_wheel_speed_age_us) ? event.pre_input_wheel_speed_rpm : NAN,event.vbat_mV);
  logger_->addEnergyControlAutonomousZeroCrossEvent(event);
}

__attribute__((noinline)) void ExperimentRunner::restoreUnsentStallRescue(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,
    bool previous_accepted_valid,uint32_t previous_accepted_ms) {
  logger_->addEnergyControlAutonomousZeroCrossEvent(event);
  energy_control_autonomous_last_accepted_zero_cross_valid_=previous_accepted_valid;
  energy_control_autonomous_last_accepted_zero_cross_ms_=previous_accepted_ms;
  energy_control_autonomous_zero_cross_consumed_for_peak_=false;
  if(!status_.emergency_stop)
    energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS;
}

__attribute__((noinline)) bool ExperimentRunner::dispatchTiltPriority(
    PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t t_test_ms,
    bool previous_accepted_valid,uint32_t previous_accepted_ms) {
  if(event.target_peak_deg==8.f && wheelWorkMode() && tryTiltPriority(event,t_test_ms)) {
    if(optional_early_event_ && optional_early_write_uncertain_) {
      // The write may have reached the roller: do not reopen this half-cycle.
      logger_->addEnergyControlAutonomousZeroCrossEvent(event);
      requestEmergencyStop("optional_early_write_unresolved");
      return true;
    }
    if((stall_rescue_event_ || optional_early_event_) && !event.output_executed) {
      restoreUnsentStallRescue(event,previous_accepted_valid,previous_accepted_ms);
      return true;
    }
    finishFootEnergyDecision(event,t_test_ms);
    if(!event.output_executed) {
      energy_control_autonomous_half_cycle_state_=EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
      resetEnergyControlAutonomousPeakTracker(true);
    }
    return true;
  }
  if(stall_rescue_event_ || optional_early_event_) {
    event.valid=false;
    event.reason=Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT;
    restoreUnsentStallRescue(event,previous_accepted_valid,previous_accepted_ms);
    return true;
  }
  return false;
}

__attribute__((noinline)) void ExperimentRunner::finishSolverAudit(
    solver_audit::Record& audit,const PsramLogger::EnergyControlAutonomousZeroCrossEvent& event,uint32_t end_us) {
    audit.end_us = end_us;
    audit.decision_us = static_cast<uint32_t>(audit.end_us - audit.start_us);
    audit.exit_sample_age_us = audit.sample_time_valid ?
        static_cast<uint32_t>(audit.end_us - audit.sample_time_us) : 0;
    audit.pulse_id_after = status_.pulse_id;
    audit.state_at_exit = static_cast<uint8_t>(status_.state);
    audit.outcome_reason = event.reason;
    audit.event_valid = event.valid;
    audit.output_executed = event.output_executed;
    audit.physical_side = event.physical_next_peak_side;
    audit.command_direction = event.q_command_direction;
    audit.i0_mA = event.i0_estimated_mA;
    audit.free_peak_deg = event.free_next_peak_amplitude_deg;
    audit.target_peak_deg = event.target_peak_deg;
    audit.target_energy_j = event.target_energy_j;
    audit.passive_energy_j = event.passive_energy_j;
    audit.q_available_mA_s = event.q_available_mA_s;
    audit.integral_mA_s = event.integral_side_mA_s;
    audit.base_gain = event.g_side_base_deg_per_mA_s;
    audit.correction_c = event.c_side_used_deg;
    audit.correction_gain = event.g_side_corrected_deg_per_mA_s;
    audit.correction_limit = Config::ENERGY_CONTROL_AUTONOMOUS_SIDE_RESPONSE_MAX_ABS_DEG;
    if (audit.solver_started) {
      audit.solver_us = static_cast<uint32_t>(audit.solver_end_us - audit.solver_start_us);
    }
    logger_->addSolverAuditEvent(audit);
}

void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(uint32_t t_test_ms, float rate_dps,
                                                                  float detector_before_deg,
                                                                  float detector_after_deg,
                                                                  float crossing_alpha,
                                                                  float interpolated_time_ms) {
  if (!energy_control_autonomous_mode_ || !logger_ || status_.pulse_active ||
      (energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::ENERGY_CONTROL &&
       energy_control_autonomous_phase_ != EnergyControlAutonomousPhase::HOLD) ||
      energy_control_autonomous_half_cycle_state_ !=
          EnergyControlAutonomousHalfCycleState::WAIT_ZERO_CROSS ||
      !energy_control_autonomous_last_peak_valid_ ||
      energy_control_autonomous_zero_cross_consumed_for_peak_) return;
  const uint32_t accepted_cross_ms = static_cast<uint32_t>(lroundf(interpolated_time_ms));
  const bool previous_accepted_valid=energy_control_autonomous_last_accepted_zero_cross_valid_;
  const uint32_t previous_accepted_ms=energy_control_autonomous_last_accepted_zero_cross_ms_;
  if (energy_control_autonomous_last_accepted_zero_cross_valid_ &&
      static_cast<uint32_t>(accepted_cross_ms - energy_control_autonomous_last_accepted_zero_cross_ms_) <
          Config::ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS) return;
  if (logger_->energyControlAutonomousEventCapacityReached()) {
    logger_->markEnergyControlAutonomousEventOverflow();
  }
  // One accepted peak owns one following accepted central passage. Do not arm
  // the next peak tracker until a normal pulse has finished (or width is zero).
  energy_control_autonomous_zero_cross_consumed_for_peak_ = true;
  energy_control_autonomous_last_accepted_zero_cross_valid_ = true;
  energy_control_autonomous_last_accepted_zero_cross_ms_ = accepted_cross_ms;
  resetEnergyControlAutonomousPeakTracker(false);
  auto rearm_for_next_peak = [this]() RW_SPEED_CODE {
    energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK;
    resetEnergyControlAutonomousPeakTracker(true);
  };
  const uint32_t v46l_decision_t0_us = micros();
  control_latency::mark(control_latency::Decision);
  PsramLogger::EnergyControlAutonomousZeroCrossEvent event;
  event.event_kind=stall_rescue_event_ ? 2 : (optional_early_event_ ? 3 : 0);
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
  if(stall_rescue_event_ && early_stall_rescue_event_)event.event_kind=4;
#endif
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
  if(stall_rescue_event_ && approach_stall_rescue_event_)event.event_kind=5;
#endif
  // V46s audit begin
  solver_audit::Record audit;
  audit.t_test_ms = t_test_ms;
  audit.start_us = v46l_decision_t0_us;
  audit.pulse_id_before = status_.pulse_id;
  audit.model_vbat_mV = status_.beta_model_vbat_mV;
  audit.gyro_sequence = imu_->reading().gyro_sequence;
  audit.sample_time_us = imu_->reading().last_gyro_update_us;
  audit.sample_time_valid = audit.gyro_sequence != 0;
  audit.entry_sample_age_us = audit.sample_time_valid ?
      static_cast<uint32_t>(micros() - audit.sample_time_us) : 0;
  auto finish_audit = [&]() RW_SPEED_CODE {
    const uint32_t end_us=micros();
    finishSolverAudit(audit,event,end_us);
  };
  solver_audit::ScopeExit<decltype(finish_audit)> audit_exit(finish_audit);
  // V46s audit end
  event.zero_cross_time_ms = accepted_cross_ms;
  event.zero_cross_rate_dps = rate_dps;
  event.zero_cross_abs_rate_dps = fabsf(rate_dps);
  event.detector_angle_before_deg = detector_before_deg;
  event.detector_angle_after_deg = detector_after_deg;
  event.detector_crossing_alpha = crossing_alpha;
  event.interpolated_crossing_time_ms = interpolated_time_ms;
  event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  if (anyNegativeProbeBatchMode() && event.physical_next_peak_side < 0) {
    event.negative_probe_batch_index = negative_probe_batch_crossing_index_;
    event.negative_probe_batch_planned_width_ms =
        negativeProbeExtendedMode()
        ? negative_probe_extended_batch::width(negative_probe_batch_seed_, negative_probe_batch_crossing_index_)
        : negative_probe_batch::width(negative_probe_batch_seed_, negative_probe_batch_crossing_index_);
    ++negative_probe_batch_crossing_index_;
  }
  int16_t selected_current_mA = energy_control_autonomous_input_current_mA_;
  const lateral_energy::CurrentTable* selected_current_table = &lateral_current_table_;
  if (positiveCurrentBatchMode())
    selectPositiveCurrentBatch(event, selected_current_mA, selected_current_table);
  event.side_mismatch_diagnostic =
      event.physical_next_peak_side != -energy_control_autonomous_last_peak_side_;
  // Historical Q1 support is on the unscaled raw-Y axis; compare in the
  // calibrated MEKF rate units now recorded by Autonomous events.
  event.rate_support_diagnostic =
      event.zero_cross_abs_rate_dps >= Config::Q1_SHADOW_RATE_SUPPORT_MIN_DPS * Config::MEKF_GYRO_Y_SCALE &&
      event.zero_cross_abs_rate_dps <= Config::Q1_SHADOW_RATE_SUPPORT_MAX_DPS * Config::MEKF_GYRO_Y_SCALE;
  event.phase = static_cast<uint8_t>(energy_control_autonomous_phase_);
  event.previous_peak_time_ms = energy_control_autonomous_last_peak_ms_;
  event.previous_peak_side = energy_control_autonomous_last_peak_side_;
  event.previous_peak_amplitude_deg = energy_control_autonomous_last_peak_amplitude_deg_;
  event.target_peak_deg = energy_control_autonomous_target_peak_deg_;
  // The selected model chooses an input from the arrival-energy error.
  // Its residual-current input always uses the zero-cross motion coordinate.
  event.q_command_direction = event.physical_next_peak_side;
  event.vbat_mV = status_.roller_battery_mV;
  const uint32_t v46l_free_model_t0_us = micros();
  const auto energy_coeff = lateral_energy::coefficients(event.physical_next_peak_side);
  const float omega = event.zero_cross_abs_rate_dps * lateral_energy::kRadPerDeg;
  const float bridge_free_angle=energy_bridge_enabled_ ?
      lateral_energy::bridgeFreeAngle(event.physical_next_peak_side,event.zero_cross_abs_rate_dps) : NAN;
  event.passive_energy_j = energy_bridge_enabled_
      ? lateral_energy::potential(lateral_energy_geometry_,bridge_free_angle)
      : lateral_energy::freeEnergy(energy_coeff,omega);
  event.cg_shift_mm=wheelWorkMode() ? run_cg_shift_mm_ : 0.f;
  event.target_energy_j = wheelWorkMode() ? controlPotential(event.physical_next_peak_side * event.target_peak_deg) : lateral_target_energy_j_;
  if(wheelWorkMode() && event.target_peak_deg==8.f) {
    event.startup_target_deg=tilt_priority_.target();
    event.startup_positive_cap_ms=tilt_priority_.widthLimit(event.physical_next_peak_side);
    event.foot_balance_left_mean=foot_balance_.leftMean();
    event.foot_balance_right_mean=foot_balance_.rightMean();
    if(foot_frame_run_id_==status_.run_id && t_test_ms>=foot_frame_ms_ && t_test_ms-foot_frame_ms_<=150) {
      event.foot_balance_left_deg=foot_frame_left_valid_ ? foot_frame_left_ : NAN;
      event.foot_balance_right_deg=foot_frame_right_valid_ ? foot_frame_right_ : NAN;
    }
    applyFootEnergyTarget(event,accepted_cross_ms);
  }
  event.free_next_peak_amplitude_deg = energy_bridge_enabled_ ? bridge_free_angle :
      lateral_energy::angle(lateral_energy_geometry_,event.passive_energy_j);
  const auto bridge_coeff=lateral_energy::bridgeCoefficients(event.physical_next_peak_side);
  event.rate_support_diagnostic = event.zero_cross_abs_rate_dps >=
      (energy_bridge_enabled_ ? bridge_coeff.rate_min_dps : energy_coeff.rate_min_dps) &&
      event.zero_cross_abs_rate_dps <=
      (energy_bridge_enabled_ ? bridge_coeff.rate_max_dps : energy_coeff.rate_max_dps);
  event.rate_baseline_reason = energy_bridge_enabled_ ? 0 : 255;
  audit.free_model_us = static_cast<uint32_t>(micros() - v46l_free_model_t0_us);
  const uint32_t now_ms = run_start_ms_ + t_test_ms;
  event.i0_estimated_mA = predicted_current_end_ms_ == 0 ? 0.0f :
      predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);

  // Capture the latest independently sampled Roller state before solving.
  // Fresh wheel speed selects a bounded, side-specific residual cell;
  // measured current remains diagnostic and never changes this pulse.
  const uint32_t pre_input_capture_us = micros();
  event.pre_input_capture_time_us = pre_input_capture_us;
  if (roller_) {
    const RollerTelemetry pre_input = roller_->telemetrySnapshot();
    event.pre_input_current_sample_time_us = pre_input.current_sample_time_us;
    event.pre_input_current_age_us = pre_input.current_sample_time_us == 0
        ? UINT32_MAX
        : static_cast<uint32_t>(pre_input_capture_us - pre_input.current_sample_time_us);
    event.pre_input_current_valid = pre_input.current_valid && pre_input.current_sample_time_us != 0;
    event.pre_input_measured_current_mA = event.pre_input_current_valid
        ? static_cast<float>(pre_input.actual_current_mA) : NAN;
    event.pre_input_wheel_speed_sample_time_us = pre_input.speed_sample_time_us;
    event.pre_input_wheel_speed_age_us = pre_input.speed_sample_time_us == 0
        ? UINT32_MAX
        : static_cast<uint32_t>(pre_input_capture_us - pre_input.speed_sample_time_us);
    event.pre_input_wheel_speed_valid = pre_input.speed_valid && pre_input.speed_sample_time_us != 0;
    event.pre_input_wheel_speed_rpm = event.pre_input_wheel_speed_valid
        ? pre_input.speed_rpm : NAN;
  }
  const float fast_v = status_.beta_model_vbat_mV > 0
      ? static_cast<float>(status_.beta_model_vbat_mV) / 1000.0f : Config::MODEL_VBAT_REFERENCE_V;

  // The fitted work models use only information available before output.
  // Search the measured command envelope; a target outside that envelope is
  // reported as saturated, never extrapolated to a longer brake pulse.
  const bool work_mode = wheelWorkMode();
  // In the 8-degree walking experiment, lean amplitude takes priority over
  // the ordinary fitted selector and foot-span corrections. Existing sensor,
  // motor and emergency-stop checks still gate the experimental command.
  if(dispatchTiltPriority(event,t_test_ms,previous_accepted_valid,previous_accepted_ms))return;
  if(work_mode && event.target_peak_deg==8.f)
    wheel_speed_balance_.crossing(event.physical_next_peak_side,event.pre_input_wheel_speed_rpm,
        peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us),t_test_ms);
  wheel_work_energy_control::Choice work_choice;
  if (work_mode)
    work_choice=chooseWheelWork(event,fast_v,&event.rate_support_diagnostic,
        &selected_current_mA,&selected_current_table,
        event.target_peak_deg==8.f && event.physical_next_peak_side>0 ? gradual_start_.positiveCapMs() : 100);
  // First classify a missing model with the existing sensor/hardware checks.
  // Only a valid normal coast or this explicit unmodeled coast can be
  // considered for the separately logged, unverified weak-side assist.
  const bool unmodeled_coast=work_mode && event.physical_next_peak_side<0 && !work_choice.valid &&
      prepareUnmodeledWalkCoast(event);
  if(work_mode && event.physical_next_peak_side<0 &&
      tryWeakSideAssist(event,work_choice,unmodeled_coast,t_test_ms,fast_v)) {
    finishFootEnergyDecision(event,t_test_ms);
    // A successful output owns its half-cycle until updatePulse stops it.
    if(!event.output_executed)rearm_for_next_peak();
    return;
  }
  if(unmodeled_coast) {
    finishFootEnergyDecision(event,t_test_ms);rearm_for_next_peak();return;
  }
  // Branch 14 is not selected here: in the 1.0.70 walk it repeatedly
  // preempted camera-based correction, while higher current commands did not
  // demonstrate a larger measured current. Keep ordinary control available
  // when the foot observations are not suitable for feedback.
  const bool phase_skip_output=work_mode && event.target_peak_deg==8.f && event.physical_next_peak_side>0 &&
      selectFootPhaseOutput(event,t_test_ms,fast_v,work_choice,selected_current_mA,selected_current_table);
  if(phase_skip_output) {
    if(event.reason==Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE)
      event.reason=status_.emergency_stop ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE :
          Config::ENERGY_CONTROL_AUTONOMOUS_REASON_PULSE_WIDTH_GUARD;
    finishFootEnergyDecision(event,t_test_ms);rearm_for_next_peak();return;
  }
  if(work_mode && event.target_peak_deg!=8.f) choosePeakMpc(event,fast_v,work_choice,selected_current_mA,selected_current_table);
  if (work_mode) captureWheelWorkDecision(event,work_choice);

  const float goal_current_mA = predictCurrentGoalMa(selected_current_mA,fast_v);
  const float aligned_initial_mA = static_cast<float>(event.physical_next_peak_side)*event.i0_estimated_mA;
  event.integral_energy_j = event.physical_next_peak_side > 0
      ? energy_control_autonomous_integral_plus_j_ : energy_control_autonomous_integral_minus_j_;
  const float integral_limit_j = .5f * event.target_energy_j;
  const float bounded_integral_j = fmaxf(-integral_limit_j,
      fminf(integral_limit_j, event.integral_energy_j));
  const float uncorrected_request_j = event.target_energy_j - event.passive_energy_j +
      bounded_integral_j;
  auto requested_mode = uncorrected_request_j > 0.f ? peak_error_adaptation::Excite :
      (uncorrected_request_j < 0.f ? peak_error_adaptation::Brake : peak_error_adaptation::Coast);
  if (energy_bridge_enabled_ && !energyInverseMode() && requested_mode == peak_error_adaptation::Brake)
    requested_mode = peak_error_adaptation::Coast;
  if (requested_mode == peak_error_adaptation::Brake &&
      event.physical_next_peak_side < 0 && energy_control_autonomous_brake_max_ms_ == 0)
    requested_mode = peak_error_adaptation::Coast;
  float prediction_bias_j = peak_adaptation_enabled_ ? peak_error_model_.correction(event.physical_next_peak_side,
      requested_mode, event.pre_input_wheel_speed_rpm, event.pre_input_wheel_speed_valid,
      event.pre_input_wheel_speed_age_us, &event.wheel_speed_bin,
      &event.adaptation_consistent_samples) : 0.f;
  // A learned correction cannot reverse the selected drive/brake decision.
  const float bias_limit_j = .5f * fabsf(uncorrected_request_j);
  prediction_bias_j = fmaxf(-bias_limit_j, fminf(bias_limit_j, prediction_bias_j));
  const bool inverse_negative = energyInverseMode() && event.physical_next_peak_side < 0;
  float inverse_residual_j = 0.f;
  if (inverse_negative && inverse_negative_residual_count_) {
    for (uint8_t i=0; i<inverse_negative_residual_count_; ++i)
      inverse_residual_j += inverse_negative_residual_j_[i];
    inverse_residual_j /= inverse_negative_residual_count_;
    // Do not let a few noisy peaks overwhelm the fitted response.
    inverse_residual_j = fmaxf(-.00020f, fminf(.00020f, inverse_residual_j));
  }
  if (work_mode)
    prediction_bias_j=work_choice.valid ? work_choice.applied_bias_mj*.001f : 0.f;
  const uint16_t fallback_positive_cap=walking_positive_cap::fallback(
      work_mode,work_choice.valid,event.physical_next_peak_side,event.target_peak_deg,
      gradual_start_.positiveCapMs(),wheel_speed_balance_.capMs(),lateral_energy::kMaxMs);
  if(work_mode && !work_choice.valid && event.physical_next_peak_side>0 && event.target_peak_deg==8.f)
    event.work_brake_cap_ms=fallback_positive_cap;
  one_peak_energy_inverse::Choice inverse_choice;
  audit.solver_started=1; audit.solver_start_us=micros();
  auto decide_selected=[&](float bias) RW_SPEED_CODE {
    if (work_mode && work_choice.valid)
      return workDecision(work_choice,event.physical_next_peak_side,
          event.passive_energy_j,event.target_energy_j,*selected_current_table,
          event.i0_estimated_mA,goal_current_mA);
    if (work_mode && event.physical_next_peak_side<0) return lateral_energy::Decision{};
    if (inverse_negative) {
      lateral_energy::Decision d;
      const one_peak_energy_inverse::State state{
          event.zero_cross_abs_rate_dps, event.pre_input_wheel_speed_rpm,
          event.previous_peak_amplitude_deg, static_cast<float>(event.vbat_mV),
          upright_reference_.pitch_deg};
      event.rate_support_diagnostic = one_peak_energy_inverse::supported(state) &&
          peak_error_adaptation::speedUsable(event.pre_input_wheel_speed_rpm,
              event.pre_input_wheel_speed_valid,event.pre_input_wheel_speed_age_us);
      if (!event.rate_support_diagnostic) return d;
      const bool protect = energy_control_autonomous_last_peak_valid_ &&
          energy_control_autonomous_last_peak_side_ > 0 &&
          energy_control_autonomous_last_peak_amplitude_deg_ <
              one_peak_energy_inverse::kLowOppositePeakDeg;
      inverse_choice = one_peak_energy_inverse::choose(state,
          event.target_energy_j*1000.f,inverse_residual_j*1000.f,
          inverse_negative_extra_ms_,protect);
      if (!inverse_choice.valid) return d;
      d.valid=true; d.ms=inverse_choice.width_ms; d.evaluations=5;
      d.command_sign=d.ms ? -1 : 0;
      d.free_j=inverse_choice.free_mj*.001f;
      d.target_j=event.target_energy_j;
      d.predicted_j=inverse_choice.energy_mj*.001f;
      d.deficit_j=d.target_j-d.free_j;
      d.commanded_deficit_j=d.deficit_j;
      d.corrected_target_j=d.target_j;
      d.energy_error_j=d.target_j-d.predicted_j;
      d.selected_q_mAs=d.ms ? -lateral_energy::charge(*selected_current_table,d.ms,
          event.i0_estimated_mA,goal_current_mA) : 0.f;
      d.requested_q_mAs=d.selected_q_mAs;
      d.lower=inverse_choice.below_reachable;
      d.upper=inverse_choice.above_reachable;
      d.extrapolated=inverse_choice.extrapolated;
      return d;
    }
    return energy_bridge_enabled_
        ? lateral_energy::decideBridge(lateral_energy_geometry_,*selected_current_table,
            event.physical_next_peak_side,event.zero_cross_abs_rate_dps,event.target_energy_j,
            event.integral_energy_j,aligned_initial_mA,goal_current_mA,bias,fallback_positive_cap)
        : lateral_energy::decide(lateral_energy_geometry_,*selected_current_table,
            event.physical_next_peak_side,event.zero_cross_abs_rate_dps,event.target_energy_j,
            event.integral_energy_j,aligned_initial_mA,goal_current_mA,
            event.physical_next_peak_side < 0 ? energy_control_autonomous_brake_max_ms_ : lateral_energy::kBrakeMaxMs,
            bias,fallback_positive_cap);
  };
  auto decision=decide_selected(prediction_bias_j);
  if (!work_mode && !inverse_negative && decision.valid && prediction_bias_j != 0.f &&
      peak_error_adaptation::modeForSignedCharge(decision.selected_q_mAs) != requested_mode) {
    // At a mode boundary, retain the original controller decision rather
    // than applying a correction learned from a different pulse direction.
    prediction_bias_j = 0.f;
    decision = decide_selected(0.f);
  }
  if (inverse_negative && decision.valid) {
    prediction_bias_j=inverse_residual_j;
    event.passive_energy_j=decision.free_j;
    event.free_next_peak_amplitude_deg=controlAngle(decision.free_j,event.physical_next_peak_side);
    event.rate_support_diagnostic=true;
  }
  if (!decision.valid || !std::isfinite(event.free_next_peak_amplitude_deg)) {
    audit.stage=1;audit.solver_end_us=micros();
    event.reason=work_mode && event.physical_next_peak_side<0 && !work_choice.valid
        ? (event.rate_support_diagnostic
            ? Config::ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_PROTECT_OPPOSITE
            : Config::ENERGY_CONTROL_AUTONOMOUS_REASON_WORK_OUTSIDE_SUPPORT)
        : Config::ENERGY_CONTROL_AUTONOMOUS_REASON_POTENTIAL_DOMAIN;
    finishFootEnergyDecision(event,t_test_ms);rearm_for_next_peak();return;
  }
  event.q_command_direction = decision.command_sign<0
      ? -event.physical_next_peak_side : event.physical_next_peak_side;
  event.command_matches_zero_cross_motion =
      event.q_command_direction == event.physical_next_peak_side;
  // In the dedicated identification package the positive-side bridge stays
  // intact. The negative side is a fixed RAW-positive pulse, not an inverse
  // prediction from the historical positive-Q angle fit.
  const bool fixed_negative_probe = negative_probe_mode_ && event.physical_next_peak_side < 0;
  // Preserve the positive swing before testing a stronger negative pulse.
  // The extended package records the planned condition even when this gate
  // skips the output, so skipped crossings cannot masquerade as observations.
  const uint16_t selected_width_ms = fixed_negative_probe
      ? selectFixedNegativeProbeWidth(event)
      : decision.ms;
  const float selected_q_mA_s = fixed_negative_probe
      ? (selected_width_ms ? -lateral_energy::charge(*selected_current_table, selected_width_ms,
            event.i0_estimated_mA, goal_current_mA) : 0.0f)
      : decision.selected_q_mAs;
  if (fixed_negative_probe) {
    event.q_command_direction = 1; // The two hardware polarity factors cancel: RAW +300 mA.
    event.command_matches_zero_cross_motion = false;
  }
  event.fixed_negative_probe = fixed_negative_probe;
  event.prediction_bias_j = prediction_bias_j;
  event.adaptation_mode = peak_error_adaptation::modeForSignedCharge(selected_q_mA_s);
  event.kinetic_energy_j=decision.kinetic_j;
  event.predicted_loss_j=decision.loss_j;
  event.delta_energy_required_j=decision.deficit_j;
  event.commanded_energy_deficit_j=decision.commanded_deficit_j;
  event.corrected_target_energy_j=decision.corrected_target_j;
  event.q_available_mA_s=decision.available_q_mAs;
  event.q_ff_energy_mA_s=(fixed_negative_probe || inverse_negative) ? NAN : decision.requested_q_mAs;
  event.q_unclamped_mA_s=(fixed_negative_probe || inverse_negative) ? NAN : decision.unclamped_q_mAs;
  event.unclamped_inverse_valid=!fixed_negative_probe && !inverse_negative && std::isfinite(decision.unclamped_q_mAs);
  event.unclamped_energy_request_j=decision.unclamped_deficit_j;
  event.maximum_input_work_j=(fixed_negative_probe || inverse_negative) ? NAN : decision.maximum_input_work_j;
  event.unapplied_energy_request_j=fixed_negative_probe ? NAN :
      (inverse_negative || (work_mode && work_choice.valid)) ? decision.target_j-decision.predicted_j :
      decision.unclamped_deficit_j - (decision.predicted_j-decision.free_j);
  event.predicted_current_goal_mA=goal_current_mA;
  event.predicted_current_rise_tau_s=selected_current_table->tau;
  event.aligned_initial_current_mA=aligned_initial_mA;
  event.q_saturated_upper=!fixed_negative_probe && decision.upper;
  event.q_saturated_lower=!fixed_negative_probe && decision.lower;
  event.work_omega_q_coefficient=energy_bridge_enabled_ ? NAN : energy_coeff.work_omega_q;
  event.work_q2_coefficient=energy_bridge_enabled_ ? NAN : energy_coeff.work_q2;
  event.energy_model_extrapolated=fixed_negative_probe || decision.extrapolated ||
      (work_mode ? (!work_choice.valid || work_choice.cross_condition || run_cg_shift_mm_!=0.f) : selected_current_mA!=300);
  event.q_command_mA_s=selected_q_mA_s;event.q_effective_pred_mA_s=selected_q_mA_s;
  event.predicted_energy_j=fixed_negative_probe ? NAN : decision.predicted_j;
  event.predicted_next_peak_amplitude_deg=fixed_negative_probe ? NAN :
      controlAngle(decision.predicted_j,event.physical_next_peak_side);
  event.predicted_observed_energy_j = fixed_negative_probe ? NAN :
      fmaxf(0.f, decision.predicted_j + ((inverse_negative || (work_mode && work_choice.valid)) ? 0.f : prediction_bias_j));
  event.predicted_observed_next_peak_deg = controlAngle(event.predicted_observed_energy_j,event.physical_next_peak_side);
  event.solver_required_width_ms=static_cast<float>(selected_width_ms);
  event.solver_selected_integer_width_ms=selected_width_ms;
  // Audit fields are retained for old log readers; current energy-unit fields
  // live on the zero-cross record. Bridge mode inverts the potential and
  // historical positive-Q angle response, not a motor-work equation.
  audit.signed_target_current_mA=event.q_command_direction*goal_current_mA;
  audit.tau_s=selected_current_table->tau;
  audit.ff_q_mA_s=fixed_negative_probe ? NAN : decision.requested_q_mAs;
  audit.ff_width_ms=65535;audit.ff_valid=!fixed_negative_probe;
  audit.corrected_target_energy_j=decision.corrected_target_j;
  audit.selected_width_ms=selected_width_ms;audit.fast_selected_width_ms=selected_width_ms;
  audit.selected_q_mA_s=selected_q_mA_s;audit.eval_count=decision.evaluations;
  audit.selected_eval_count=decision.evaluations;audit.selected_valid=true;
  audit.stage=4;audit.solver_complete=1;audit.solver_end_us=micros();
  audit.solver_us=static_cast<uint32_t>(audit.solver_end_us-audit.solver_start_us);
  audit.selected_search_us=audit.solver_us;audit.fast_solver_us=audit.solver_us;
  pending_cg_shift_mm_=work_mode ? run_cg_shift_mm_ : 0.f;
  energy_control_autonomous_pending_peak_ = true;
  inverse_negative_pending_width_ms_ = inverse_negative ? selected_width_ms : 0;
  inverse_negative_pending_bias_j_ = inverse_negative ? inverse_residual_j : 0.f;
  energy_control_autonomous_pending_probe_ = fixed_negative_probe;
  energy_control_autonomous_pending_next_side_ = event.physical_next_peak_side;
  energy_control_autonomous_pending_target_deg_ = work_mode && event.target_peak_deg==8.f
      ? event.foot_balance_target_deg : event.target_peak_deg;
  pending_target_energy_j_=event.target_energy_j;
  energy_control_autonomous_pending_q_command_mA_s_ = selected_q_mA_s;
  energy_control_autonomous_pending_model_energy_j_ = event.predicted_energy_j;
  capturePendingWheelWork(event,work_choice,selected_width_ms,prediction_bias_j);
  energy_control_autonomous_pending_wheel_rpm_ = event.pre_input_wheel_speed_rpm;
  energy_control_autonomous_pending_wheel_age_us_ = event.pre_input_wheel_speed_age_us;
  energy_control_autonomous_pending_wheel_valid_ = event.pre_input_wheel_speed_valid;
  energy_control_autonomous_pending_zero_ms_ = accepted_cross_ms;
  energy_control_autonomous_pending_mode_ =
      peak_error_adaptation::modeForSignedCharge(selected_q_mA_s);
  energy_control_autonomous_pending_saturated_upper_ = event.q_saturated_upper;
  energy_control_autonomous_pending_saturated_lower_ = event.q_saturated_lower;
  if (selected_width_ms == 0) {
    event.valid = true;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT;
    finishFootEnergyDecision(event,t_test_ms);
    rearm_for_next_peak();
    return;
  }
  if (status_.emergency_stop || status_.state != ExperimentState::RUNNING_BATCH_SWEEP) {
    if (inverse_negative || work_negative_pending_ || work_positive_pending_) {
      energy_control_autonomous_pending_peak_ = false;
      work_negative_pending_ = false;
      work_positive_pending_ = false;
    }
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE;
    finishFootEnergyDecision(event,t_test_ms);
    rearm_for_next_peak();
    return;
  }
  if (!roller_ || !roller_->ok()) {
    if (inverse_negative || work_negative_pending_ || work_positive_pending_) {
      energy_control_autonomous_pending_peak_ = false;
      work_negative_pending_ = false;
      work_positive_pending_ = false;
    }
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY;
    finishFootEnergyDecision(event,t_test_ms);
    rearm_for_next_peak();
    return;
  }
  event.pulse_width_ms = selected_width_ms;
  event.command_current_mA = selected_current_mA;
  event.pulse_start_ms = t_test_ms;
  event.pulse_end_ms = t_test_ms + selected_width_ms;
  // v45 labels are copied from the already-selected V7 command. They are
  // diagnostic only; neither field is read by the pulse/control path.
  status_.current_audit_q_target_mA_s = event.q_command_mA_s;
  status_.current_audit_q_pred_mA_s = event.q_effective_pred_mA_s;
  if (!beginEnergyControlAutonomousPulse(now_ms, t_test_ms, event.q_command_direction,
                                         selected_width_ms, selected_current_mA)) {
    if (inverse_negative || work_negative_pending_ || work_positive_pending_) {
      energy_control_autonomous_pending_peak_ = false;
      work_negative_pending_ = false;
      work_positive_pending_ = false;
    }
    event.command_current_mA = 0;
    event.pulse_width_ms = 0;
    event.pulse_start_ms = 0;
    event.pulse_end_ms = 0;
    event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED;
    finishFootEnergyDecision(event,t_test_ms);
    rearm_for_next_peak();
    return;
  }
  // V46r performs no deferred solver shadow after the pulse. The old deferred
  // computation consumed Core1 time without affecting the already-issued command.
  event.output_executed = true;
  event.valid = true;
  event.reason = Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONE;
  finishFootEnergyDecision(event,t_test_ms);
}
void ExperimentRunner::runEnergyControlAutonomousSolverShadow() {
  if (!solver_shadow_pending_ || !logger_) return;

  PsramLogger::SolverShadowEvent result = solver_shadow_event_;
  const uint32_t shadow_t0_us = micros();
  uint16_t eval_count = 0;

  const float v = result.model_vbat_mV > 0
      ? static_cast<float>(result.model_vbat_mV) / 1000.0f
      : Config::MODEL_VBAT_REFERENCE_V;
  const float signed_target_current_mA =
      static_cast<float>(result.q_command_direction) *
      predictCurrentGoalMa(energy_control_autonomous_input_current_mA_, v);
  const float tau_s = predictRiseTauS(energy_control_autonomous_input_current_mA_);

  struct FastCandidate {
    bool valid = false;
    uint16_t width_ms = 0;
    float q_mA_s = NAN;
    float energy_j = NAN;
    float error_j = NAN;
  };

  auto evaluate_width = [&](uint16_t width_ms, float target_energy_j) -> FastCandidate {
    FastCandidate c;
    c.width_ms = width_ms;
    ++eval_count;
    float q_mA_s = 0.0f;
    if (width_ms > 0) {
      const float t_s = static_cast<float>(width_ms) / 1000.0f;
      const float signed_q_mA_s =
          signed_target_current_mA * t_s +
          (result.i0_estimated_mA - signed_target_current_mA) * tau_s *
              (1.0f - expf(-t_s / tau_s));
      q_mA_s = fabsf(signed_q_mA_s);
    }
    const float predicted_peak_deg = energyControlAutonomousCorrectedPrediction(
        result.free_next_peak_deg, result.physical_next_peak_side, q_mA_s, nullptr);
    const float energy_j = energyControlPotentialJ(predicted_peak_deg);
    if (!isfinite(q_mA_s) || !isfinite(energy_j) || !isfinite(target_energy_j)) return c;
    c.valid = true;
    c.q_mA_s = q_mA_s;
    c.energy_j = energy_j;
    c.error_j = fabsf(target_energy_j - energy_j);
    return c;
  };

  auto fast_pick_width = [&](float target_energy_j) -> FastCandidate {
    uint16_t lo = Config::ENERGY_CONTROL_AUTONOMOUS_MIN_PULSE_MS;
    uint16_t hi = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS;
    while (hi > lo && static_cast<uint16_t>(hi - lo) > 8U) {
      const uint16_t third = static_cast<uint16_t>((hi - lo) / 3U);
      if (third == 0) break;
      const uint16_t m1 = static_cast<uint16_t>(lo + third);
      const uint16_t m2 = static_cast<uint16_t>(hi - third);
      const FastCandidate c1 = evaluate_width(m1, target_energy_j);
      const FastCandidate c2 = evaluate_width(m2, target_energy_j);
      if (!c1.valid || !c2.valid) return FastCandidate{};
      if (c1.error_j <= c2.error_j) hi = m2;
      else lo = m1;
    }
    FastCandidate best;
    for (uint16_t width_ms = lo; width_ms <= hi; ++width_ms) {
      const FastCandidate c = evaluate_width(width_ms, target_energy_j);
      if (!c.valid) return FastCandidate{};
      if (!best.valid || c.error_j < best.error_j) best = c;
    }
    return best;
  };

  const FastCandidate ff = fast_pick_width(result.target_energy_j);
  if (ff.valid) {
    result.fast_ff_width_ms = ff.width_ms;
    result.fast_ff_q_mA_s = ff.q_mA_s;
    const float q_unclamped_mA_s = ff.q_mA_s + result.integral_side_mA_s;
    const float corrected_q_target_mA_s =
        fmaxf(0.0f, fminf(result.q_available_mA_s, q_unclamped_mA_s));
    const float corrected_target_prediction_deg = energyControlAutonomousCorrectedPrediction(
        result.free_next_peak_deg, result.physical_next_peak_side,
        corrected_q_target_mA_s, nullptr);
    const float corrected_target_energy_j =
        energyControlPotentialJ(corrected_target_prediction_deg);
    const FastCandidate selected = fast_pick_width(corrected_target_energy_j);
    if (selected.valid) {
      result.fast_selected_width_ms = selected.width_ms;
      result.fast_selected_q_mA_s = selected.q_mA_s;
      result.fast_valid = true;
      result.ff_width_match = result.fast_ff_width_ms == result.legacy_ff_width_ms;
      result.selected_width_match =
          result.fast_selected_width_ms == result.legacy_selected_width_ms;
    }
  }

  result.fast_eval_count = eval_count;
  result.fast_shadow_us = static_cast<uint32_t>(micros() - shadow_t0_us);
  logger_->addSolverShadowEvent(result);
  solver_shadow_pending_ = false;
  solver_shadow_event_ = PsramLogger::SolverShadowEvent{};
}

void ExperimentRunner::startTimingProbe(uint8_t pulse_kind, uint32_t t_test_ms,
                                              int16_t command_mA, uint16_t pulse_width_ms,
                                              uint32_t pulse_start_us, uint32_t set_current_us,
                                              uint32_t state_update_us, uint32_t current_model_us,
                                              uint32_t update_pulse_model_us,
                                              uint32_t pulse_begin_total_us) {
  if (!logger_ || !energy_control_autonomous_mode_ || timing_probe_pending_) return;
  timing_probe_event_ = PsramLogger::TimingProbeEvent{};
  timing_probe_event_.pulse_id = status_.pulse_id;
  timing_probe_event_.pulse_kind = pulse_kind;
  timing_probe_event_.t_test_ms = t_test_ms;
  timing_probe_event_.command_mA = command_mA;
  timing_probe_event_.pulse_width_ms = pulse_width_ms;
  timing_probe_event_.pulse_start_us = pulse_start_us;
  timing_probe_event_.gyro_sequence_at_start = imu_ ? imu_->reading().gyro_sequence : 0;
  timing_probe_event_.set_current_us = set_current_us;
  timing_probe_event_.state_update_us = state_update_us;
  timing_probe_event_.current_model_us = current_model_us;
  timing_probe_event_.update_pulse_model_us = update_pulse_model_us;
  timing_probe_event_.pulse_begin_total_us = pulse_begin_total_us;
  timing_probe_pending_ = true;
  timing_probe_loop_captured_ = false;
  timing_probe_log_captured_ = false;
  timing_probe_imu_captured_ = false;
}

void ExperimentRunner::recordTimingProbeLoop(uint32_t imu_update_us, uint32_t runner_update_us,
                                               uint32_t core1_path_us) {
  if (!timing_probe_pending_ || timing_probe_loop_captured_) return;
  timing_probe_event_.imu_update_call_us = imu_update_us;
  timing_probe_event_.runner_update_call_us = runner_update_us;
  timing_probe_event_.core1_path_us = core1_path_us;
  timing_probe_loop_captured_ = true;
  maybeFinalizeTimingProbe();
}

void ExperimentRunner::maybeFinalizeTimingProbe() {
  if (!timing_probe_pending_ || !logger_) return;
  timing_probe_event_.capture_mask =
      (timing_probe_loop_captured_ ? 0x01U : 0U) |
      (timing_probe_log_captured_ ? 0x02U : 0U) |
      (timing_probe_imu_captured_ ? 0x04U : 0U);
  timing_probe_event_.complete = timing_probe_event_.capture_mask == 0x07U;
  const bool run_ended = status_.state != ExperimentState::RUNNING_BATCH_SWEEP;
  const bool terminal_partial = run_ended;
  if (!timing_probe_event_.complete && !terminal_partial) return;
  logger_->addTimingProbeEvent(timing_probe_event_);
  timing_probe_pending_ = false;
}

void ExperimentRunner::captureAngleOffsets() {
  offset_mekf_pitch_deg_ = raw_mekf_pitch_abs_deg_;
  offset_beta1_raw_deg_ = raw_beta1_raw_pitch_deg_;
  offset_beta1_bias_deg_ = raw_beta1_bias_pitch_deg_;
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    offset_dynamic_raw_deg_[i] = raw_dynamic_raw_pitch_deg_[i];
    offset_dynamic_bias_deg_[i] = raw_dynamic_bias_pitch_deg_[i];
  }
  offset_accel_deg_ = raw_accel_pitch_deg_;
  gyro_raw_deg_ = 0.0f;
  gyro_bias_corrected_deg_ = 0.0f;
}

// V46z comparison-zero begin
void ExperimentRunner::captureMekfComparisonZero(float& zero_abs_deg, uint32_t& zero_sample_us) {
  zero_abs_deg = raw_mekf_pitch_abs_deg_;
  zero_sample_us = imu_ ? imu_->reading().last_gyro_update_us : 0;
}

void ExperimentRunner::updateMekfComparisonRelativeAngles() {
  status_.pitch_mekf_start_sync_relative_deg =
      isfinite(status_.mekf_start_sync_zero_abs_deg)
          ? raw_mekf_pitch_abs_deg_ - status_.mekf_start_sync_zero_abs_deg : NAN;
  status_.pitch_mekf_measurement_relative_deg =
      isfinite(status_.mekf_measurement_zero_abs_deg)
          ? raw_mekf_pitch_abs_deg_ - status_.mekf_measurement_zero_abs_deg : NAN;
  status_.pitch_mekf_trial_relative_deg =
      isfinite(status_.mekf_trial_zero_abs_deg)
          ? raw_mekf_pitch_abs_deg_ - status_.mekf_trial_zero_abs_deg : NAN;
}
// V46z comparison-zero end

void ExperimentRunner::requestEmergencyStop(const char* reason) {
  upright_reference_capture_.cancel("upright_reference_cancelled");
  setSyncLed(false);
  stopMotor();
  if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP || status_.state == ExperimentState::TRIAL_REST ||
      status_.state == ExperimentState::END_SYNC) {
    logger_->markMeasurementDone();
  }
  status_.state = ExperimentState::ESTOP;
  status_.running = false;
  status_.emergency_stop = true;
  status_.last_error = reason ? reason : "estop";
}

void ExperimentRunner::clearFinishedOrEstop() {
  if (running()) return;
  if (logger_) logger_->clear();
  setSyncLed(false);
  status_.state = bias_ready_ ? ExperimentState::READY_TO_MEASURE : ExperimentState::STARTUP_GYRO_CALIB;
  status_.running = false;
  status_.emergency_stop = false;
  status_.last_error = "";
  status_.measure_elapsed_ms = 0;
  status_.remaining_ms = measurementTotalDurationMs();
  status_.sync_event_id = 0;
}

void ExperimentRunner::setInputSettings(int16_t current_mA, uint16_t pulse_width_ms, uint16_t input_interval_ms) {
  if (running()) return;
  if (current_mA < 0) current_mA = -current_mA;
  if (current_mA > Config::MAX_ABS_INPUT_CURRENT_MA) current_mA = Config::MAX_ABS_INPUT_CURRENT_MA;
  pulse_width_ms = constrain(pulse_width_ms, Config::MIN_PULSE_WIDTH_MS, Config::MAX_PULSE_WIDTH_MS);
  input_interval_ms = constrain(input_interval_ms, Config::MIN_INPUT_INTERVAL_MS, Config::MAX_INPUT_INTERVAL_MS);
  if (pulse_width_ms > input_interval_ms) pulse_width_ms = input_interval_ms;
  status_.current_mA_setting = current_mA;
  status_.pulse_width_ms_setting = pulse_width_ms;
  status_.input_interval_ms = input_interval_ms;
}

void ExperimentRunner::updateMidSyncLed(uint32_t now_ms) {
  const uint32_t t_test_ms = now_ms - run_start_ms_;
  if (t_test_ms < Config::MID_SYNC_FIRST_MS) {
    status_.sync_event_id = 0;
    setSyncLed(false);
    return;
  }
  const uint32_t phase_ms = (t_test_ms - Config::MID_SYNC_FIRST_MS) % Config::MID_SYNC_INTERVAL_MS;
  if (phase_ms < Config::MID_SYNC_SHORT_ON_MS) {
    status_.sync_event_id = 5;
    setSyncLed(true);
  } else if (phase_ms < Config::MID_SYNC_SHORT_ON_MS + Config::MID_SYNC_GAP_MS) {
    status_.sync_event_id = 5;
    setSyncLed(false);
  } else if (phase_ms < Config::MID_SYNC_TOTAL_MS) {
    status_.sync_event_id = 5;
    setSyncLed(true);
  } else {
    status_.sync_event_id = 0;
    setSyncLed(false);
  }
}

void ExperimentRunner::updateStartSync(uint32_t now_ms) {
  stopMotor();

  // V46 hardware workflow intentionally powers up while the mechanism is lying
  // down. For Autonomous V7, rebuild the MEKF gravity attitude from a stable
  // upright average during the first OFF segment of the unchanged START sync
  // pattern. Startup gyro bias is preserved and re-applied.
  if (energy_control_autonomous_mode_ && g_v46_mekf_run_reinit.active &&
      !g_v46_mekf_run_reinit.done && sync_step_ == 0) {
    const ImuReading& r = imu_->reading();
    if (r.accel_sequence != 0 &&
        r.accel_sequence != g_v46_mekf_run_reinit.last_accel_sequence) {
      g_v46_mekf_run_reinit.last_accel_sequence = r.accel_sequence;
      if (isUprightStableSample(r)) {
        g_v46_mekf_run_reinit.ax_sum_g += r.ax_g;
        g_v46_mekf_run_reinit.ay_sum_g += r.ay_g;
        g_v46_mekf_run_reinit.az_sum_g += r.az_g;
        ++g_v46_mekf_run_reinit.stable_sample_count;
      } else {
        // Require a contiguous stable window. A hand movement while START_SYNC
        // is beginning restarts the average instead of contaminating it.
        g_v46_mekf_run_reinit.stable_sample_count = 0;
        g_v46_mekf_run_reinit.ax_sum_g = 0.0;
        g_v46_mekf_run_reinit.ay_sum_g = 0.0;
        g_v46_mekf_run_reinit.az_sum_g = 0.0;
      }
    }

    if (static_cast<uint32_t>(now_ms - sync_step_start_ms_) >=
        UprightPoseGuide::MEKF_REINIT_AVERAGE_MS) {
      const uint32_t n = g_v46_mekf_run_reinit.stable_sample_count;
      if (n < UprightPoseGuide::MEKF_REINIT_MIN_SAMPLES) {
        g_v46_mekf_run_reinit.active = false;
        requestEmergencyStop("mekf_reinit_upright_not_stable");
        return;
      }

      const mekf6::Vec3 mean_accel_raw{
          static_cast<float>(g_v46_mekf_run_reinit.ax_sum_g / n),
          static_cast<float>(g_v46_mekf_run_reinit.ay_sum_g / n),
          static_cast<float>(g_v46_mekf_run_reinit.az_sum_g / n)};
      const mekf6::Vec3 mean_accel{
          -mean_accel_raw.x, mean_accel_raw.y, -mean_accel_raw.z};

      mekf_.reset();
      mekf_.setConfig(makeMekfConfig());
      mekf_initialized_ = mekf_.initializeFromAccel(mean_accel);
      status_.mekf_attitude = MekfAttitudeSnapshot{};
      if (mekf_initialized_ && bias_ready_) {
        mekf_.setGyroBiasRadS(mekfStartupBiasFromRaw(
            status_.gyro_bias_x_dps, status_.gyro_bias_y_dps, status_.gyro_bias_z_dps));
      }
      if (!mekf_initialized_) {
        g_v46_mekf_run_reinit.active = false;
        requestEmergencyStop("mekf_reinit_failed");
        return;
      }

      // Prime diagnostics with the same averaged gravity sample. Subsequent
      // 5-ms IMU updates continue normal adaptive accel correction for the
      // remaining START sync time before any motor command is authorized.
      mekf_.updateAccel(mean_accel);
      status_.mekf_attitude = captureMekfAttitude(mekf_, mekf_initialized_, r.last_gyro_update_us,
          mean_accel, mekfGyroRadFromRaw(r), r.last_accel_update_us);
      raw_mekf_pitch_abs_deg_ = status_.mekf_attitude.pitch_deg;
      const auto q = status_.mekf_attitude.quaternion;
      status_.mekf_q_w = q.w;
      status_.mekf_q_x = q.x;
      status_.mekf_q_y = q.y;
      status_.mekf_q_z = q.z;
      const auto d = mekf_.diagnostics();
      status_.mekf_accel_confidence = d.accel_confidence;
      status_.mekf_accel_residual_deg = d.accel_direction_residual_deg;
      status_.mekf_accel_mag_error_g = d.accel_magnitude_error_g;
      status_.mekf_accel_used = d.accel_used;

      g_v46_mekf_run_reinit.done = true;
      g_v46_mekf_run_reinit.active = false;
      if (RuntimeDiag::enabled()) Serial.printf("MEKF run reinit: n=%u mean=(%.5f,%.5f,%.5f) pitch=%.3f conf=%.3f resid=%.3f\n",
                    static_cast<unsigned>(n), mean_accel.x, mean_accel.y, mean_accel.z,
                    raw_mekf_pitch_abs_deg_, status_.mekf_accel_confidence,
                    status_.mekf_accel_residual_deg);
    }
  }
  if (updateSyncPattern(now_ms, Config::START_SYNC_PATTERN, Config::START_SYNC_PATTERN_STEP_COUNT)) {
    setSyncLed(false);
    beginMeasurementRun();
  }
}

void ExperimentRunner::beginEndSync(uint32_t now_ms) {
  stopMotor();
  status_.state = ExperimentState::END_SYNC;
  status_.running = true;
  status_.sync_event_id = 4;
  sync_step_ = 0;
  sync_step_start_ms_ = now_ms;
  sync_led_until_ms_ = 0;
  setSyncLed(Config::END_SYNC_PATTERN[0].led_on);
  logSampleNow();
}

void ExperimentRunner::updateEndSync(uint32_t now_ms) {
  stopMotor();
  if (!updateSyncPattern(now_ms, Config::END_SYNC_PATTERN, Config::END_SYNC_PATTERN_STEP_COUNT)) return;

  setSyncLed(false);
  logger_->markMeasurementDone();
  status_.state = ExperimentState::FINISHED;
  status_.running = false;
  status_.sync_event_id = 0;
}

bool ExperimentRunner::updateSyncPattern(uint32_t now_ms, const Config::LedSyncStep* pattern, uint8_t step_count) {
  if (!pattern || step_count == 0) return true;
  if (sync_step_ >= step_count) return true;

  while (sync_step_ < step_count) {
    const uint16_t duration_ms = pattern[sync_step_].duration_ms;
    if (static_cast<uint32_t>(now_ms - sync_step_start_ms_) < duration_ms) break;
    sync_step_start_ms_ += duration_ms;
    ++sync_step_;
    if (sync_step_ >= step_count) return true;
    setSyncLed(pattern[sync_step_].led_on);
  }
  return false;
}

void ExperimentRunner::finishTrial(uint32_t now_ms) {
  stopMotor();
  status_.trial_elapsed_ms = status_.trial_duration_ms;
  status_.sync_event_id = 7;
  logSampleNow();
  if (single_trial_mode_ || status_.trial_index >= Config::BETA_SWEEP_TRIAL_COUNT) {
    finishRun();
    return;
  }
  status_.state = ExperimentState::TRIAL_REST;
  rest_start_ms_ = now_ms;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
}

void ExperimentRunner::updateTrialRest(uint32_t now_ms) {
  stopMotor();
  if (static_cast<uint32_t>(now_ms - rest_start_ms_) < Config::BETA_SWEEP_INTER_TRIAL_REST_MS) return;
  beginTrial(status_.trial_index);
}

void ExperimentRunner::updateInputPulse(uint32_t now_ms) {
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP) return;
  const uint32_t t_ms = now_ms - trial_start_ms_;

  if (status_.pulse_active &&
      static_cast<uint32_t>(now_ms - active_pulse_start_ms_) >= status_.pulse_width_ms_setting) {
    stopActivePulse(now_ms);
  }
  if (status_.pulse_active || t_ms >= status_.trial_duration_ms) return;

  if (!zero_cross_mode_) {
    if (t_ms >= next_pulse_start_test_ms_) {
      beginPulse(now_ms, t_ms, next_pulse_direction_);
      next_pulse_direction_ = -next_pulse_direction_;
    }
    return;
  }

  const float angle_deg = status_.pitch_mekf_deg;
  const float rate_dps = status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
  if (calibration_enabled_) updateCalibrationPeak(now_ms, t_ms, angle_deg, rate_dps);
  if (calibration_enabled_ && !calibrationIsMainControl() && t_ms >= Config::ZERO_CROSS_CALIBRATION_TIMEOUT_MS) {
    finishCalibrationShadow(4);
  }
  // V59 may finish from the calibration state machine; never issue another
  // zero-cross pulse in the same IMU update after the run has entered END_SYNC.
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP) return;
  if (identification_mode_) updateIdentificationPeak(now_ms, angle_deg, rate_dps);
  if (!zero_cross_has_previous_angle_) {
    zero_cross_previous_angle_deg_ = angle_deg;
    zero_cross_has_previous_angle_ = true;
  }

  // Identification uses a strong bootstrap, then either rotates validation Q targets or selects a control Q.
  if (zero_cross_bootstrap_pending_) {
    status_.current_mA_setting = zero_cross_fixed_current_mA_;
    const int8_t direction = Config::ZERO_CROSS_BOOTSTRAP_DIRECTION;
    const float i0 = predicted_signed_current_end_mA_ *
        expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
    CalibrationBuildUpCommand calibration_command;
    const bool use_calibration_build_up = calibration_enabled_;
    const float q_command = use_calibration_build_up
        ? (calibration_command = calibrationBuildUpCommand(
              angle_deg, rate_dps, i0, direction,
              Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG)).q_command_mA_s
        : Config::ZERO_CROSS_IDENTIFICATION_BOOTSTRAP_Q_MAS;
    const uint16_t width = use_calibration_build_up
        ? calibration_command.width_ms
        : (identification_mode_ ? identificationWidthForTarget(q_command, i0, direction)
                                : zero_cross_fixed_pulse_ms_);
    status_.pulse_width_ms_setting = width;
    if (identification_mode_) {
      const float q_requested = use_calibration_build_up ? calibration_command.q_required_mA_s
          : (q_run_mode_ == QRunMode::CONTROL ? requiredControlQMaS(angle_deg, rate_dps, direction) : q_command);
      const float a_target = use_calibration_build_up ? Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG
          : (q_run_mode_ == QRunMode::CONTROL ? control_target_peak_deg_ : 0.0f);
      const float a_pred = use_calibration_build_up ? calibration_command.predicted_peak_deg
          : predictNextPeakAbsDeg(angle_deg, rate_dps, q_command, direction);
      const ControlQEvaluation evaluation = q_run_mode_ == QRunMode::CONTROL && !use_calibration_build_up
          ? evaluateControlQ(angle_deg, rate_dps, direction) : ControlQEvaluation{};
      beginIdentificationEvent(now_ms, t_ms, direction, q_command, q_requested, q_command,
                               a_target, a_pred, evaluation.min_predicted_peak_deg,
                               evaluation.max_predicted_peak_deg, evaluation.target_reachable,
                               true, width, false);
    }
    beginPulse(now_ms, t_ms, direction);
    if (use_calibration_build_up) {
      calibration_initial_kick_count_ = 1;
      calibration_initial_kick_q_effective_pred_mA_s_ = calibration_command.q_effective_mA_s;
      recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 0,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG,
                                    calibration_command);
      calibration_phase_ = CalibrationPhase::INITIAL_EXCITE;
    }
    zero_cross_bootstrap_pending_ = false;
    zero_cross_armed_ = false;
    zero_cross_start_refractory_until_ms_ = now_ms + Config::ZERO_CROSS_START_REFRACTORY_MS;
    zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
    zero_cross_previous_angle_deg_ = angle_deg;
    return;
  }

  zero_cross_half_cycle_peak_abs_deg_ =
      fmaxf(zero_cross_half_cycle_peak_abs_deg_, fabsf(angle_deg));
  if (static_cast<int32_t>(now_ms - zero_cross_start_refractory_until_ms_) < 0) {
    zero_cross_previous_angle_deg_ = angle_deg;
    return;
  }

  if (fabsf(angle_deg) >= Config::ZERO_CROSS_REARM_ANGLE_DEG) zero_cross_armed_ = true;
  const bool interval_ok = last_zero_cross_pulse_start_ms_ == 0 ||
                           static_cast<uint32_t>(now_ms - last_zero_cross_pulse_start_ms_) >=
                               Config::ZERO_CROSS_MIN_PULSE_INTERVAL_MS;
  bool zero_cross_detected = false;
  if (zero_cross_armed_ && interval_ok) {
    zero_cross_detected =
        (zero_cross_previous_angle_deg_ < 0.0f && angle_deg >= 0.0f &&
         rate_dps >= Config::ZERO_CROSS_MIN_RATE_DPS) ||
        (zero_cross_previous_angle_deg_ > 0.0f && angle_deg <= 0.0f &&
         rate_dps <= -Config::ZERO_CROSS_MIN_RATE_DPS);
  }
  zero_cross_previous_angle_deg_ = angle_deg;
  if (!zero_cross_detected) return;

  // The angle estimate only decides when a pulse is allowed. Its sign is not
  // trusted to decide motor direction: every accepted crossing alternates the
  // physical excitation direction after the one-time bootstrap kick.
  int8_t direction = zero_cross_next_direction_;
  zero_cross_next_direction_ = -zero_cross_next_direction_;
  const int8_t next_peak_side = rate_dps >= 0.0f ? 1 : -1;

  if (calibration_enabled_ && !calibrationIsMainControl()) {
    zero_cross_armed_ = false;
    zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
    if (calibration_phase_ == CalibrationPhase::INITIAL_EXCITE) {
      if (calibration_initial_kick_count_ >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS) {
        finishCalibrationShadow(1);
        return;
      }
      status_.current_mA_setting = zero_cross_fixed_current_mA_;
      const float i0_initial = predicted_signed_current_end_mA_ *
          expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
      const CalibrationBuildUpCommand command =
          calibrationBuildUpCommand(angle_deg, rate_dps, i0_initial, direction,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG);
      status_.pulse_width_ms_setting = command.width_ms;
      calibration_initial_kick_q_effective_pred_mA_s_ = command.q_effective_mA_s;
      ++calibration_initial_kick_count_;
      recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 0,
                                    Config::ZERO_CROSS_CALIBRATION_INITIAL_TARGET_PEAK_DEG, command);
      beginPulse(now_ms, t_ms, direction);
      return;
    }
    if (calibration_phase_ == CalibrationPhase::FREE_DECAY &&
        next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE &&
        calibration_last_half_range_valid_ && calibration_last_center_dynamic_valid_) {
      // V62 observes this unforced positive arrival only; it cannot command a
      // pulse or alter the fixed-Q sequence.
      addV62RateStateSample(calibration_last_half_range_dynamic_deg_,
                            calibration_last_center_dynamic_deg_, fabsf(rate_dps));
    }
    if (calibration_phase_ == CalibrationPhase::FREE_DECAY) return;
    if (calibration_phase_ == CalibrationPhase::Q_REBUILD) {
      // Rebuild only until the confirmed predecessor is inside the common
      // margin-reduced fitted input domain. Every pulse remains subject to the
      // existing Q cap and prediction guard used by calibrationBuildUpCommand.
      if (calibration_reverse_samples_ != 0 || !calibration_last_peak_valid_) return;
      const float support_min = fmaxf(calibration_result_.free_input_min_pos_deg,
                                      calibration_result_.free_input_min_neg_deg);
      const float support_max = fminf(calibration_result_.free_input_max_pos_deg,
                                      calibration_result_.free_input_max_neg_deg);
      const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
      const bool legacy_a_domain_valid = support_min + margin <= support_max - margin;
      // The dynamic-H model is an independently fitted, measured input domain.
      // If the legacy A domains have no common interior, only an invalid H
      // model is terminal; a valid H model may still be reached by a rebuild.
      if (!legacy_a_domain_valid && !calibration_result_.half_range_dynamic_valid) {
        finishCalibrationShadow(7);
        return;
      }
      bool predecessor_in_legacy_a_support = legacy_a_domain_valid &&
          calibration_last_peak_abs_deg_ >= support_min + margin &&
          calibration_last_peak_abs_deg_ <= support_max - margin;
      bool predecessor_in_dynamic_h_support = dynamicHalfRangePredecessorInSupport();
      if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
        finishCalibrationShadow(0);
        return;
      }

      // V59 makes this a pre-pulse admission check. A skipped crossing never
      // advances the fixed-Q plan. High or mismatched states coast freely;
      // low states use the existing bounded rebuild below.
      const float v59_hprev = calibration_last_half_range_dynamic_deg_;
      const float v59_cprev = calibration_last_center_dynamic_deg_;
      const float v59_abs_rate = fabsf(rate_dps);
      const bool v59_state_valid = calibration_last_half_range_valid_ &&
          calibration_last_center_dynamic_valid_ && isfinite(v59_hprev) && isfinite(v59_cprev) &&
          isfinite(v59_abs_rate);
      const uint8_t v59_reason = classifyV59StateGate(v59_hprev, v59_cprev, v59_abs_rate,
                                                        next_peak_side, predecessor_in_dynamic_h_support);
      const bool v59_gate_passed = v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
      const bool v59_low_state = v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_DYNAMIC_H_OUT_OF_SUPPORT ||
          v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_HPREV_BELOW ||
          v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_STATE_INVALID;
      // A V60 rebuild is followed by one whole unforced oscillation (two
      // consecutive peak-pair H updates) before the gate may consume a fixed Q.
      // Log these candidates with their full H/C/rate state, but do not count
      // them as failed gate admissions.
      if (calibration_v60_cooldown_halfcycles_remaining_ > 0) {
        --calibration_v60_cooldown_halfcycles_remaining_;
        ++calibration_result_.v60_cooldown_free_decay_count;
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 3,
                                true, false);
        if (calibration_v60_cooldown_halfcycles_remaining_ == 0) {
          calibration_v61_rebuild_plan_active_ = false;
        }
        if (logger_) logger_->setCalibrationResult(calibration_result_);
        return;
      }
      // A V61 rebuild drives only the peak side opposite to the fixed-Q
      // arrival.  This avoids the V60 alternation where a negative command
      // was immediately followed by an unrelated positive rebuild.  If the
      // measured state is already admissible, abandon any pending rebuild plan
      // and let the unchanged V59 gate enter the deterministic Q plan.
      if (v59_gate_passed) {
        calibration_v61_rebuild_plan_active_ = false;
      }
      const bool v61_rebuild_required = calibration_result_.v61_state_target_valid &&
          !v59_gate_passed && (calibration_v61_rebuild_plan_active_ || v59_low_state);
      if (v61_rebuild_required) {
        if (!calibration_v61_rebuild_plan_active_) {
          calibration_v61_rebuild_plan_active_ = true;
          calibration_v61_command_peak_target_deg_ =
              calibration_v61_nominal_controlled_peak_target_deg_;
          calibration_result_.v61_feedback_command_peak_deg =
              calibration_v61_command_peak_target_deg_;
        }
        const int8_t controlled_side = -Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
        if (next_peak_side != controlled_side) {
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support,
                                  Config::ZERO_CROSS_V59_GATE_REASON_V61_STATE_FEEDBACK, 0);
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support,
                                Config::ZERO_CROSS_V59_GATE_REASON_V61_STATE_FEEDBACK, 2);
        status_.current_mA_setting = zero_cross_fixed_current_mA_;
        const float i0_rebuild = predicted_signed_current_end_mA_ *
            expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
        const CalibrationBuildUpCommand command = calibrationBuildUpCommand(
            angle_deg, rate_dps, i0_rebuild, direction,
            calibration_v61_command_peak_target_deg_,
            Config::ZERO_CROSS_V60_REBUILD_UNRESTRICTED);
        status_.pulse_width_ms_setting = command.width_ms;
        recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 1,
                                      calibration_v61_command_peak_target_deg_, command);
        if (calibration_rebuild_attempt_in_episode_ == 0) {
          ++calibration_rebuild_episode_id_;
        }
        ++calibration_rebuild_attempt_in_episode_;
        ++calibration_rebuild_total_count_;
        calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
        calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
        calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
        calibration_rebuild_pulse_start_test_ms_ = t_ms;
        calibration_v60_cooldown_halfcycles_remaining_ = 0;
        calibration_phase_ = CalibrationPhase::WAIT_Q_REBUILD;
        beginPulse(now_ms, t_ms, direction);
        calibration_rebuild_pulse_id_ = status_.pulse_id;
        return;
      }
      if (!v59_gate_passed) {
        ++calibration_probe_wait_halfcycle_count_;
        ++calibration_probe_wait_halfcycle_total_;
        ++calibration_v59_gate_event_count_;
        ++calibration_v59_gate_skip_count_;
        calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
        calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
        calibration_result_.v59_gate_skip_count = calibration_v59_gate_skip_count_;
        if (calibration_probe_wait_halfcycle_count_ >= Config::ZERO_CROSS_V59_MAX_CONSECUTIVE_GATE_SKIPS) {
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 0);
          finishCalibrationShadow(10);
          return;
        }
        if (!v59_low_state) {
          if (next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE &&
              v59_reason == Config::ZERO_CROSS_V59_GATE_REASON_HPREV_ABOVE) {
            calibration_v60_high_desired_wait_pending_ = true;
          }
          recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                  v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 0);
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        ++calibration_v59_rebuild_from_low_state_count_;
        calibration_result_.v59_rebuild_from_low_state_count = calibration_v59_rebuild_from_low_state_count_;
        const bool v60_gate_skipped_by_decay =
            calibration_v60_high_desired_wait_pending_ &&
            next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
        if (v60_gate_skipped_by_decay) {
          ++calibration_result_.v60_gate_skipped_by_decay_count;
        }
        if (next_peak_side == Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE) {
          calibration_v60_high_desired_wait_pending_ = false;
        }
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 2,
                                false, v60_gate_skipped_by_decay);
        // Force the known-safe rebuild branch even where the broader fitted
        // support domain still happens to contain this low V59 state.
        predecessor_in_legacy_a_support = false;
        predecessor_in_dynamic_h_support = false;
      } else {
        calibration_v60_high_desired_wait_pending_ = false;
        ++calibration_v59_gate_event_count_;
        ++calibration_v59_gate_pass_count_;
        calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
        calibration_result_.v59_gate_pass_count = calibration_v59_gate_pass_count_;
        recordV59StateGateEvent(t_ms, v59_hprev, v59_cprev, v59_abs_rate, next_peak_side,
                                v59_state_valid, predecessor_in_dynamic_h_support, v59_reason, 1);
      }

      if (predecessor_in_legacy_a_support || predecessor_in_dynamic_h_support) {
        const int8_t desired_side = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_DESIRED_SIDE[
            calibration_probe_plan_index_];
        if (next_peak_side == desired_side) {
          calibration_probe_rebuild_entry_source_ = predecessor_in_legacy_a_support ? 0 : 1;
          calibration_probe_rebuild_target_reached_ = calibration_result_.v60_rebuild_target_valid &&
              calibration_last_peak_abs_deg_ >= calibration_result_.v60_rebuild_target_peak_deg;
          calibration_probe_dynamic_h_in_support_at_command_ = predecessor_in_dynamic_h_support;
          calibration_phase_ = desired_side > 0 ? CalibrationPhase::Q_CAL_POS :
                                                  CalibrationPhase::Q_CAL_NEG;
        } else {
          // This cannot occur in enabled V59 (direction is part of the gate),
          // but retain the historical free-half-cycle behavior as a guard.
          ++calibration_probe_wait_halfcycle_count_;
          ++calibration_probe_wait_halfcycle_total_;
          calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
      } else {
        if (Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS != 0 &&
            calibration_rebuild_attempt_in_episode_ >=
                Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_MAX_ATTEMPTS) {
          finishCalibrationShadow(8);
          return;
        }
        status_.current_mA_setting = zero_cross_fixed_current_mA_;
        const float i0_rebuild = predicted_signed_current_end_mA_ *
            expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
        const CalibrationBuildUpCommand command = calibrationBuildUpCommand(
            angle_deg, rate_dps, i0_rebuild, direction,
            calibration_result_.v60_rebuild_target_peak_deg,
            Config::ZERO_CROSS_V60_REBUILD_UNRESTRICTED);
        status_.pulse_width_ms_setting = command.width_ms;
        recordCalibrationBuildUpPulse(t_ms, angle_deg, rate_dps, direction, 1,
                                      calibration_result_.v60_rebuild_target_peak_deg, command);
        if (calibration_rebuild_attempt_in_episode_ == 0) {
          ++calibration_rebuild_episode_id_;
        }
        ++calibration_rebuild_attempt_in_episode_;
        ++calibration_rebuild_total_count_;
        calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
        calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
        calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
        calibration_rebuild_pulse_start_test_ms_ = t_ms;
        // Cooldown is scheduled only after a measured peak reaches the V60
        // rebuild target, not between below-target build-up pulses.
        calibration_v60_cooldown_halfcycles_remaining_ = 0;
        calibration_phase_ = CalibrationPhase::WAIT_Q_REBUILD;
        beginPulse(now_ms, t_ms, direction);
        calibration_rebuild_pulse_id_ = status_.pulse_id;
        return;
      }
    }
    const bool want_pos = calibration_phase_ == CalibrationPhase::Q_CAL_POS;
    const bool want_neg = calibration_phase_ == CalibrationPhase::Q_CAL_NEG;
    if (want_pos || want_neg) {
      if (calibration_probe_attempt_count_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_ATTEMPTS) {
        finishCalibrationShadow(5);
        return;
      }
      // Do not command from a stale pre-turn state. A confirmed extremum is
      // required before a low-Q probe can be associated with its next peak.
      if (calibration_reverse_samples_ != 0 || !calibration_last_peak_valid_) return;
      const int8_t requested_side = want_pos ? 1 : -1;
      if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT ||
          requested_side != Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_DESIRED_SIDE[
              calibration_probe_plan_index_]) {
        finishCalibrationShadow(3);
        return;
      }
      // The probe can arrive on either side, so its predecessor must lie in
      // the intersection of both selected A_prev domains, after margin.
      const float support_min = fmaxf(calibration_result_.free_input_min_pos_deg,
                                      calibration_result_.free_input_min_neg_deg);
      const float support_max = fminf(calibration_result_.free_input_max_pos_deg,
                                      calibration_result_.free_input_max_neg_deg);
      const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
      const bool predecessor_in_legacy_a_support = support_min + margin <= support_max - margin &&
          calibration_last_peak_abs_deg_ >= support_min + margin &&
          calibration_last_peak_abs_deg_ <= support_max - margin;
      const bool predecessor_in_dynamic_h_support = dynamicHalfRangePredecessorInSupport();
      if (!predecessor_in_legacy_a_support && !predecessor_in_dynamic_h_support) {
        // Never extrapolate either free-decay model. The v50 H path is allowed
        // only inside the measured dynamic-H input domain.
        finishCalibrationShadow(7);
        return;
      }
      // v46 base polarity is fixed and explicit: direction = -desired arrival
      // side. A measured non-positive gain is data, never a reason to flip it.
      direction = -requested_side;
      const uint8_t plan_index = calibration_probe_plan_index_;
      const uint8_t q_level_index = calibrationProbeQLevelIndex(plan_index);
      const float q_target_mA_s = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_Q_LEVEL_MAS[q_level_index];
      status_.current_mA_setting = zero_cross_fixed_current_mA_;
      const float i0_cal = predicted_signed_current_end_mA_ *
                           expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
      const uint16_t width_cal = calibrationProbeWidthForTarget(q_target_mA_s, i0_cal, direction);
      status_.pulse_width_ms_setting = width_cal;
      calibration_probe_previous_peak_abs_deg_ = calibration_last_peak_abs_deg_;
      calibration_probe_previous_peak_signed_dynamic_deg_ = calibration_last_peak_signed_dynamic_deg_;
      calibration_probe_previous_peak_signed_fixed_deg_ = calibration_last_peak_signed_fixed_deg_;
      calibration_probe_previous_half_range_dynamic_deg_ = calibration_last_half_range_dynamic_deg_;
      calibration_probe_previous_half_range_fixed_deg_ = calibration_last_half_range_fixed_deg_;
      calibration_probe_previous_half_range_valid_ = calibration_last_half_range_valid_;
      calibration_probe_previous_center_dynamic_deg_ = calibration_last_center_dynamic_deg_;
      calibration_probe_previous_center_dynamic_valid_ = calibration_last_center_dynamic_valid_;
      calibration_probe_previous_peak_candidate_ms_ = calibration_last_peak_candidate_ms_;
      calibration_probe_q_target_mA_s_ = q_target_mA_s;
      calibration_probe_q_effective_pred_mA_s_ = fabsf(identificationChargeMaS(i0_cal, direction, width_cal));
      calibration_probe_command_dynamic_angle_deg_ =
          status_.pitch_mekf_deg;
      calibration_probe_command_rate_raw_dps_ = status_.gyro_pitch_rate_dps;
      calibration_probe_command_rate_bias_corrected_dps_ =
          status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
      calibration_probe_requested_side_ = requested_side;
      calibration_probe_predecessor_side_ = calibration_last_peak_side_;
      calibration_probe_direction_ = direction;
      calibration_probe_active_plan_index_ = plan_index + 1;
      calibration_probe_q_level_index_ = q_level_index;
      calibration_probe_rebuild_total_count_ = calibration_rebuild_total_count_;
      calibration_probe_rebuild_episode_id_ = calibration_rebuild_episode_id_;
      calibration_probe_rebuild_attempt_in_episode_ = calibration_rebuild_attempt_in_episode_;
      calibration_probe_rebuild_entry_source_ = predecessor_in_legacy_a_support ? 0 : 1;
      calibration_probe_rebuild_target_reached_ =
          calibration_last_peak_abs_deg_ >= Config::ZERO_CROSS_CALIBRATION_Q_REBUILD_TARGET_PEAK_DEG;
      calibration_probe_dynamic_h_in_support_at_command_ = predecessor_in_dynamic_h_support;
      calibration_probe_pulse_start_test_ms_ = t_ms;
      ++calibration_probe_attempt_count_;
      ++calibration_probe_plan_index_;
      calibration_result_.probe_plan_count = calibration_probe_plan_index_;
      calibration_phase_ = requested_side > 0 ? CalibrationPhase::WAIT_Q_CAL_POS :
                                                  CalibrationPhase::WAIT_Q_CAL_NEG;
      beginPulse(now_ms, t_ms, direction);
      calibration_probe_pulse_id_ = status_.pulse_id;
    }
    return;
  }

  status_.current_mA_setting = zero_cross_fixed_current_mA_;
  float i0 = predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  float q_target = 0.0f;
  float q_requested = 0.0f;
  float q_command = 0.0f;
  float a_target = 0.0f;
  float a_pred = 0.0f;
  float a_min = 0.0f;
  float a_max = 0.0f;
  bool target_reachable = false;
  if (q_run_mode_ == QRunMode::VALIDATION) {
    const uint8_t target_index = (identification_target_index_ / 2) %
                                 Config::ZERO_CROSS_IDENTIFICATION_TARGET_COUNT;
    q_target = Config::ZERO_CROSS_IDENTIFICATION_TARGET_Q_MAS[target_index];
    q_requested = q_target;
    q_command = q_target;
    ++identification_target_index_;
    a_pred = predictNextPeakAbsDeg(angle_deg, rate_dps, q_command, direction);
  } else if (q_run_mode_ == QRunMode::CONTROL) {
    q_requested = requiredControlQMaS(angle_deg, rate_dps, direction);
    const ControlQEvaluation evaluation = evaluateControlQ(angle_deg, rate_dps, direction);
    q_command = evaluation.q_command_mA_s;
    a_pred = evaluation.predicted_peak_deg;
    a_min = evaluation.min_predicted_peak_deg;
    a_max = evaluation.max_predicted_peak_deg;
    target_reachable = evaluation.target_reachable;
    q_target = q_command;  // retained for compatibility with existing Q tooling.
    a_target = control_target_peak_deg_;
  }
  const bool control_mode = q_run_mode_ == QRunMode::CONTROL;
  const float min_control_q_mA_s = control_mode
      ? fabsf(identificationChargeMaS(i0, direction, Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS))
      : 0.0f;
  // Below the physical 5 ms minimum, a zero-energy command is more faithful
  // to the inverse-Q request than rounding it up to a nonzero pulse.
  const bool suppressed = identification_mode_ &&
      (q_command <= 0.0f || (control_mode && q_command < min_control_q_mA_s));
  uint16_t width = identification_mode_
      ? (suppressed ? 0 : (control_mode ? controlWidthForTarget(q_command, i0, direction)
                                        : identificationWidthForTarget(q_command, i0, direction)))
      : zero_cross_fixed_pulse_ms_;
  if (control_mode && !suppressed) {
    // Width inversion initially returns the first integer-ms value reaching Q.
    // Round downward when necessary: normal control must never inject more
    // energy than the bounded continuous-Q request.
    while (width > Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS &&
           fabsf(identificationChargeMaS(i0, direction, width)) > q_command + 0.0001f) {
      --width;
    }
    // Record the physically attainable integer-ms Q prediction, not just the
    // continuous inverse target used before pulse-width quantization.
    a_pred = predictNextPeakAbsDeg(angle_deg, rate_dps,
                                   fabsf(identificationChargeMaS(i0, direction, width)), direction);
  }
  status_.pulse_width_ms_setting = width;
  zero_cross_armed_ = false;
  zero_cross_half_cycle_peak_abs_deg_ = fabsf(angle_deg);
  if (identification_mode_) {
    beginIdentificationEvent(now_ms, t_ms, direction, q_target, q_requested, q_command,
                             a_target, a_pred, a_min, a_max, target_reachable,
                             false, width, suppressed);
  }
  if (!suppressed) beginPulse(now_ms, t_ms, direction);
}

void ExperimentRunner::beginPulse(uint32_t now_ms, uint32_t t_test_ms, int8_t direction) {
  // This derived firmware has no current/pulse output path. Retain the method
  // only so a legacy scheduler call fails closed without any actuator action.
  (void)now_ms;
  (void)t_test_ms;
  (void)direction;
  stopMotor();
  status_.last_error = "motor_off_only";
}

void ExperimentRunner::stopActivePulse(uint32_t now_ms) {
  q_ident_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  if (roller_) roller_->stop();
  status_.motor_cmd_mA = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  last_pulse_end_ms_ = now_ms;
  if (!zero_cross_mode_) {
    next_pulse_start_test_ms_ = active_pulse_start_test_ms_ + status_.input_interval_ms;
  }
}

float ExperimentRunner::predictedChargeMaS(float signed_i0_mA, int8_t direction, float width_ms,
                                            int16_t commanded_current_mA) const {
  if (!isfinite(signed_i0_mA) || !isfinite(width_ms) || width_ms < 0.0f ||
      (direction != -1 && direction != 1) || commanded_current_mA <= 0) return NAN;
  const float v = status_.beta_model_vbat_mV > 0 ? status_.beta_model_vbat_mV / 1000.0f :
      Config::MODEL_VBAT_REFERENCE_V;
  const float target = direction * predictCurrentGoalMa(commanded_current_mA, v);
  const float tau = predictRiseTauS(commanded_current_mA);
  const float t = width_ms / 1000.0f;
  return target * t + (signed_i0_mA - target) * tau * (1.0f - expf(-t / tau));
}

float ExperimentRunner::identificationChargeMaS(float signed_i0_mA, int8_t direction, float width_ms) const {
  return predictedChargeMaS(signed_i0_mA, direction, width_ms, zero_cross_fixed_current_mA_);
}
uint16_t ExperimentRunner::widthForChargeTarget(float target_q_mA_s, float signed_i0_mA,
                                                  int8_t direction, uint16_t min_width_ms,
                                                  uint16_t max_width_ms) const {
  uint16_t lo = min_width_ms;
  uint16_t hi = max_width_ms < min_width_ms ? min_width_ms : max_width_ms;
  for (uint8_t i = 0; i < 16 && lo < hi; ++i) {
    const uint16_t mid = static_cast<uint16_t>((lo + hi) / 2);
    if (fabsf(identificationChargeMaS(signed_i0_mA, direction, mid)) < target_q_mA_s) lo = mid + 1;
    else hi = mid;
  }
  // A target may be unattainable within the supplied interval; use its upper
  // bound and preserve the resulting Q gap in metadata.
  return lo > max_width_ms ? max_width_ms : lo;
}

uint16_t ExperimentRunner::identificationWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                          int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_IDENTIFICATION_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::calibrationProbeWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                            int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::calibrationWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                       int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS);
}

uint16_t ExperimentRunner::controlWidthForTarget(float target_q_mA_s, float signed_i0_mA,
                                                   int8_t direction) const {
  return widthForChargeTarget(target_q_mA_s, signed_i0_mA, direction,
                              Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                              Config::ZERO_CROSS_CONTROL_MAX_PULSE_MS);
}

float ExperimentRunner::predictNextPeakAbsDeg(float angle_deg, float rate_dps,
                                                  float q_command_mA_s, int8_t direction) const {
  const float d = direction >= 0 ? 1.0f : -1.0f;
  return Config::ZERO_CROSS_Q_MODEL_INTERCEPT_DEG +
         Config::ZERO_CROSS_Q_MODEL_RATE_GAIN_DEG_PER_DPS * fabsf(rate_dps) +
         Config::ZERO_CROSS_Q_MODEL_ANGLE_GAIN_DEG_PER_DEG * fabsf(angle_deg) +
         Config::ZERO_CROSS_Q_MODEL_GAIN_DEG_PER_MAS * q_command_mA_s +
         Config::ZERO_CROSS_Q_MODEL_DIRECTION_OFFSET_DEG * d +
         Config::ZERO_CROSS_Q_MODEL_DIRECTION_GAIN_DEG_PER_MAS * q_command_mA_s * d;
}

float ExperimentRunner::requiredQForTargetPeakMaS(float target_peak_deg, float angle_deg,
                                                    float rate_dps, int8_t direction) const {
  const float d = direction >= 0 ? 1.0f : -1.0f;
  const float base = Config::ZERO_CROSS_Q_MODEL_INTERCEPT_DEG +
                     Config::ZERO_CROSS_Q_MODEL_RATE_GAIN_DEG_PER_DPS * fabsf(rate_dps) +
                     Config::ZERO_CROSS_Q_MODEL_ANGLE_GAIN_DEG_PER_DEG * fabsf(angle_deg) +
                     Config::ZERO_CROSS_Q_MODEL_DIRECTION_OFFSET_DEG * d;
  const float gain = Config::ZERO_CROSS_Q_MODEL_GAIN_DEG_PER_MAS +
                     Config::ZERO_CROSS_Q_MODEL_DIRECTION_GAIN_DEG_PER_MAS * d;
  return gain > 0.0f ? (target_peak_deg - base) / gain : 0.0f;
}

float ExperimentRunner::requiredControlQMaS(float angle_deg, float rate_dps, int8_t direction) const {
  return requiredQForTargetPeakMaS(control_target_peak_deg_, angle_deg, rate_dps, direction);
}

ExperimentRunner::CalibrationBuildUpCommand ExperimentRunner::calibrationBuildUpCommand(
    float angle_deg, float rate_dps, float signed_i0_mA, int8_t direction,
    float target_peak_deg, bool unrestricted) const {
  CalibrationBuildUpCommand command;
  command.q_required_mA_s = requiredQForTargetPeakMaS(
      target_peak_deg, angle_deg, rate_dps, direction);
  command.q_command_mA_s = fmaxf(0.0f, command.q_required_mA_s);
  if (!unrestricted) {
    const float q_guard = requiredQForTargetPeakMaS(
        Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG, angle_deg, rate_dps, direction);
    if (q_guard >= 0.0f && command.q_command_mA_s > q_guard) {
      command.q_command_mA_s = q_guard;
      command.guard_limited = true;
    }
    if (command.q_command_mA_s > Config::ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS) {
      command.q_command_mA_s = Config::ZERO_CROSS_CALIBRATION_INITIAL_Q_MAX_MAS;
      command.q_cap_limited = true;
    }
  }
  const uint16_t max_width_ms = unrestricted ? static_cast<uint16_t>(65535U) :
      Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_PULSE_MS;
  command.width_ms = widthForChargeTarget(command.q_command_mA_s, signed_i0_mA, direction,
                                          Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS,
                                          max_width_ms);
  command.q_effective_mA_s = fabsf(identificationChargeMaS(signed_i0_mA, direction, command.width_ms));
  command.width_limited = command.q_effective_mA_s + 0.0001f < command.q_command_mA_s;
  command.predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, command.q_effective_mA_s, direction);
  if (!unrestricted) {
    // Integer-ms quantization must neither exceed the requested bounded energy
    // nor turn the 18 deg prediction guard into an exceedance.
    while (command.width_ms > Config::ZERO_CROSS_IDENTIFICATION_MIN_PULSE_MS &&
           (command.q_effective_mA_s > command.q_command_mA_s + 0.0001f ||
            command.predicted_peak_deg > Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG)) {
      const bool guard_before = command.predicted_peak_deg > Config::ZERO_CROSS_CALIBRATION_INITIAL_PREDICT_GUARD_DEG;
      --command.width_ms;
      command.q_effective_mA_s = fabsf(identificationChargeMaS(signed_i0_mA, direction, command.width_ms));
      command.predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, command.q_effective_mA_s, direction);
      if (guard_before) command.guard_limited = true;
    }
  }
  return command;
}

void ExperimentRunner::recordCalibrationBuildUpPulse(uint32_t t_test_ms, float angle_deg, float rate_dps,
                                                       int8_t direction, uint8_t phase, float target_peak_deg,
                                                       const CalibrationBuildUpCommand& command) {
  if (!logger_) return;
  PsramLogger::CalibrationBuildUpEvent event;
  event.start_ms = t_test_ms;
  event.theta0_cdeg = centi(angle_deg);
  event.rate0_cdps = centi(rate_dps);
  event.direction = direction;
  event.phase = phase;
  event.target_peak_cdeg = centi(target_peak_deg);
  event.vbat_mV = status_.beta_model_vbat_mV;
  event.pulse_width_ms = command.width_ms;
  event.q_required_mAms = lroundf(command.q_required_mA_s * 1000.0f);
  event.q_command_mAms = lroundf(command.q_command_mA_s * 1000.0f);
  event.q_effective_mAms = lroundf(command.q_effective_mA_s * 1000.0f);
  event.predicted_peak_cdeg = centi(command.predicted_peak_deg);
  event.q_cap_limited = command.q_cap_limited;
  event.guard_limited = command.guard_limited;
  event.width_limited = command.width_limited;
  logger_->addCalibrationBuildUpEvent(event);
}

ExperimentRunner::ControlQEvaluation ExperimentRunner::evaluateControlQ(float angle_deg, float rate_dps,
                                                                          int8_t direction) const {
  ControlQEvaluation evaluation;
  const float q_requested = requiredControlQMaS(angle_deg, rate_dps, direction);
  // Normal zero-cross control is continuous-Q. The identical 3.80 mA*s cap
  // used for calibration is an experimentally observed near-limit input, not
  // a raw width cap; its physical width is solved from I0 and Vbat below.
  evaluation.q_command_mA_s = fminf(Config::ZERO_CROSS_CONTROL_Q_MAX_MAS,
                                     fmaxf(0.0f, q_requested));
  evaluation.predicted_peak_deg = predictNextPeakAbsDeg(
      angle_deg, rate_dps, evaluation.q_command_mA_s, direction);
  evaluation.min_predicted_peak_deg = predictNextPeakAbsDeg(angle_deg, rate_dps, 0.0f, direction);
  evaluation.max_predicted_peak_deg = predictNextPeakAbsDeg(
      angle_deg, rate_dps, Config::ZERO_CROSS_CONTROL_Q_MAX_MAS, direction);
  evaluation.target_reachable = q_requested >= 0.0f &&
                                q_requested <= Config::ZERO_CROSS_CONTROL_Q_MAX_MAS;
  return evaluation;
}

void ExperimentRunner::resetCalibrationShadow() {
  calibration_enabled_ = Config::ZERO_CROSS_CALIBRATION_SHADOW_ENABLED && q_run_mode_ == QRunMode::CONTROL;
  calibration_phase_ = calibration_enabled_ ? CalibrationPhase::INITIAL_EXCITE : CalibrationPhase::INACTIVE;
  calibration_peak_tracker_ready_ = false;
  calibration_expected_candidate_side_ = 0;
  calibration_outbound_rate_sign_ = 0;
  calibration_reverse_samples_ = 0;
  calibration_peak_candidate_deg_ = 0.0f;
  calibration_peak_candidate_fixed_deg_ = 0.0f;
  calibration_peak_candidate_ms_ = 0;
  calibration_last_peak_valid_ = false;
  calibration_last_peak_side_ = 0;
  calibration_last_peak_abs_deg_ = 0.0f;
  calibration_last_peak_signed_dynamic_deg_ = 0.0f;
  calibration_last_peak_signed_fixed_deg_ = 0.0f;
  calibration_last_half_range_dynamic_deg_ = 0.0f;
  calibration_last_half_range_fixed_deg_ = 0.0f;
  calibration_last_half_range_valid_ = false;
  calibration_last_center_dynamic_deg_ = 0.0f;
  calibration_last_center_dynamic_valid_ = false;
  calibration_last_peak_candidate_ms_ = 0;
  calibration_fit_pos_ = CalibrationFit{};
  calibration_fit_neg_ = CalibrationFit{};
  calibration_half_range_dynamic_fit_ = HalfRangeFit{};
  calibration_half_range_fixed_fit_ = HalfRangeFit{};
  calibration_free_last_half_range_dynamic_deg_ = 0.0f;
  calibration_free_last_half_range_fixed_deg_ = 0.0f;
  calibration_free_last_half_range_valid_ = false;
  calibration_result_ = PsramLogger::CalibrationResult{};
  calibration_result_.enabled = calibration_enabled_;
  calibration_probe_previous_peak_abs_deg_ = 0.0f;
  calibration_probe_previous_peak_signed_dynamic_deg_ = 0.0f;
  calibration_probe_previous_peak_signed_fixed_deg_ = 0.0f;
  calibration_probe_previous_half_range_dynamic_deg_ = 0.0f;
  calibration_probe_previous_half_range_fixed_deg_ = 0.0f;
  calibration_probe_previous_half_range_valid_ = false;
  calibration_probe_previous_center_dynamic_deg_ = 0.0f;
  calibration_probe_previous_center_dynamic_valid_ = false;
  calibration_probe_previous_peak_candidate_ms_ = 0;
  calibration_probe_q_target_mA_s_ = 0.0f;
  calibration_probe_q_effective_pred_mA_s_ = 0.0f;
  calibration_probe_command_dynamic_angle_deg_ = 0.0f;
  calibration_probe_command_rate_raw_dps_ = 0.0f;
  calibration_probe_command_rate_bias_corrected_dps_ = 0.0f;
  calibration_probe_pulse_start_test_ms_ = 0;
  calibration_probe_pulse_id_ = 0;
  calibration_probe_requested_side_ = 0;
  calibration_probe_predecessor_side_ = 0;
  calibration_probe_direction_ = 0;
  calibration_probe_plan_index_ = 0;
  calibration_probe_active_plan_index_ = 0;
  calibration_probe_q_level_index_ = 0;
  calibration_probe_rebuild_total_count_ = 0;
  calibration_probe_rebuild_episode_id_ = 0;
  calibration_probe_rebuild_attempt_in_episode_ = 0;
  calibration_probe_rebuild_entry_source_ = 0;
  calibration_probe_rebuild_target_reached_ = false;
  calibration_probe_dynamic_h_in_support_at_command_ = false;
  calibration_probe_wait_halfcycle_count_ = 0;
  calibration_probe_wait_halfcycle_total_ = 0;
  calibration_probe_attempt_count_ = 0;
  calibration_v59_gate_event_count_ = 0;
  calibration_v59_gate_pass_count_ = 0;
  calibration_v59_gate_skip_count_ = 0;
  calibration_v59_rebuild_from_low_state_count_ = 0;
  calibration_v60_cooldown_halfcycles_remaining_ = 0;
  calibration_v60_high_desired_wait_pending_ = false;
  calibration_v61_rebuild_plan_active_ = false;
  calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  calibration_v61_command_peak_target_deg_ = 0.0f;
  calibration_v62_rate_fit_ = RateStateFit{};
  calibration_gain_sum_pos_ = 0.0f;
  calibration_gain_sum_neg_ = 0.0f;
  calibration_gain_sum_sq_pos_ = 0.0f;
  calibration_gain_sum_sq_neg_ = 0.0f;
  calibration_initial_kick_count_ = 0;
  calibration_initial_kick_q_effective_pred_mA_s_ = 0.0f;
  calibration_rebuild_pulse_id_ = 0;
  calibration_rebuild_pulse_start_test_ms_ = 0;
  calibration_rebuild_total_count_ = 0;
  calibration_rebuild_episode_id_ = 0;
  calibration_rebuild_attempt_in_episode_ = 0;
  if (logger_) logger_->setCalibrationResult(calibration_result_);
}

void ExperimentRunner::recordCalibrationPeak(uint32_t candidate_peak_ms, uint32_t confirmed_ms,
                                              int8_t side, float peak_signed_dynamic_deg,
                                              float peak_signed_fixed_deg, uint8_t phase,
                                              float previous_peak_abs_deg,
                                              float q_effective_pred_mA_s) {
  if (!logger_) return;
  PsramLogger::CalibrationPeakEvent event;
  event.candidate_peak_ms = candidate_peak_ms;
  event.confirmed_ms = confirmed_ms;
  event.peak_side = side;
  event.phase = phase;
  event.peak_abs_cdeg = centi(fabsf(peak_signed_dynamic_deg));
  event.prev_peak_abs_cdeg = previous_peak_abs_deg >= 0.0f ? centi(previous_peak_abs_deg) : LOG_NAN_I16;
  event.peak_signed_dynamic_cdeg = centi(peak_signed_dynamic_deg);
  event.peak_signed_fixed_cdeg = centi(peak_signed_fixed_deg);
  if (calibration_last_peak_valid_) {
    event.prev_peak_signed_dynamic_cdeg = centi(calibration_last_peak_signed_dynamic_deg_);
    event.center_dynamic_cdeg = centi(0.5f * (calibration_last_peak_signed_dynamic_deg_ +
                                               peak_signed_dynamic_deg));
    event.half_range_dynamic_cdeg = centi(0.5f * fabsf(peak_signed_dynamic_deg -
                                                        calibration_last_peak_signed_dynamic_deg_));
    event.prev_peak_signed_fixed_cdeg = centi(calibration_last_peak_signed_fixed_deg_);
    event.center_fixed_cdeg = centi(0.5f * (calibration_last_peak_signed_fixed_deg_ + peak_signed_fixed_deg));
    event.half_range_fixed_cdeg = centi(0.5f * fabsf(peak_signed_fixed_deg -
                                                      calibration_last_peak_signed_fixed_deg_));
  }
  event.q_effective_pred_mAms = lroundf(q_effective_pred_mA_s * 1000.0f);
  logger_->addCalibrationPeakEvent(event);
}

void ExperimentRunner::addHalfRangeFreePair(HalfRangeFit& fit, float previous_half_range_deg,
                                             float half_range_deg) {
  if (!isfinite(previous_half_range_deg) || !isfinite(half_range_deg) ||
      previous_half_range_deg <= 0.0f || half_range_deg <= 0.0f ||
      fit.count >= HalfRangeFit::kMaxPairs) return;
  if (fit.count == 0) {
    fit.input_min_deg = previous_half_range_deg;
    fit.input_max_deg = previous_half_range_deg;
  } else {
    fit.input_min_deg = fminf(fit.input_min_deg, previous_half_range_deg);
    fit.input_max_deg = fmaxf(fit.input_max_deg, previous_half_range_deg);
  }
  fit.x_deg[fit.count] = previous_half_range_deg;
  fit.y_deg[fit.count] = half_range_deg;
  ++fit.count;
  fit.sum_x += previous_half_range_deg; fit.sum_y += half_range_deg;
  fit.sum_x2 += previous_half_range_deg * previous_half_range_deg;
  fit.sum_xy += previous_half_range_deg * half_range_deg;
}

bool ExperimentRunner::fitHalfRangeShadow(HalfRangeFit& fit, bool dynamic) {
  const float n = static_cast<float>(fit.count);
  if (fit.count < 3) return false;
  const float det = n * fit.sum_x2 - fit.sum_x * fit.sum_x;
  if (!isfinite(det) || fabsf(det) < 1.0e-6f) return false;
  const float r = (n * fit.sum_xy - fit.sum_x * fit.sum_y) / det;
  const float c = (fit.sum_y - r * fit.sum_x) / n;
  float squared_error = 0.0f;
  for (uint8_t i = 0; i < fit.count; ++i) {
    const float error = fit.y_deg[i] - (r * fit.x_deg[i] + c);
    squared_error += error * error;
  }
  float loocv_squared_error = 0.0f;
  uint8_t loocv_count = 0;
  for (uint8_t omit = 0; omit < fit.count; ++omit) {
    const float nn = n - 1.0f;
    const float sx = fit.sum_x - fit.x_deg[omit];
    const float sy = fit.sum_y - fit.y_deg[omit];
    const float sx2 = fit.sum_x2 - fit.x_deg[omit] * fit.x_deg[omit];
    const float sxy = fit.sum_xy - fit.x_deg[omit] * fit.y_deg[omit];
    const float loo_det = nn * sx2 - sx * sx;
    if (nn < 2.0f || !isfinite(loo_det) || fabsf(loo_det) < 1.0e-6f) continue;
    const float loo_r = (nn * sxy - sx * sy) / loo_det;
    const float loo_c = (sy - loo_r * sx) / nn;
    const float error = fit.y_deg[omit] - (loo_r * fit.x_deg[omit] + loo_c);
    loocv_squared_error += error * error;
    ++loocv_count;
  }
  const float rmse = sqrtf(squared_error / n);
  const float loocv = loocv_count ? sqrtf(loocv_squared_error / loocv_count) : NAN;
  const bool valid = isfinite(r) && isfinite(c) && isfinite(rmse) && isfinite(loocv) &&
                     r > 0.0f && fit.input_min_deg > 0.0f && fit.input_max_deg >= fit.input_min_deg;
  if (dynamic) {
    calibration_result_.half_range_dynamic_r = r;
    calibration_result_.half_range_dynamic_c_deg = c;
    calibration_result_.half_range_dynamic_rmse_deg = rmse;
    calibration_result_.half_range_dynamic_loocv_rmse_deg = loocv;
    calibration_result_.half_range_dynamic_input_min_deg = fit.input_min_deg;
    calibration_result_.half_range_dynamic_input_max_deg = fit.input_max_deg;
    calibration_result_.half_range_dynamic_valid = valid;
  } else {
    calibration_result_.half_range_fixed_r = r;
    calibration_result_.half_range_fixed_c_deg = c;
    calibration_result_.half_range_fixed_rmse_deg = rmse;
    calibration_result_.half_range_fixed_loocv_rmse_deg = loocv;
    calibration_result_.half_range_fixed_input_min_deg = fit.input_min_deg;
    calibration_result_.half_range_fixed_input_max_deg = fit.input_max_deg;
    calibration_result_.half_range_fixed_valid = valid;
  }
  return valid;
}

bool ExperimentRunner::v60GateCenterInDynamicHInputSupport() const {
  const float h_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG);
  return calibration_result_.half_range_dynamic_valid &&
         isfinite(calibration_result_.half_range_dynamic_r) &&
         isfinite(calibration_result_.half_range_dynamic_c_deg) &&
         isfinite(calibration_result_.half_range_dynamic_input_min_deg) &&
         isfinite(calibration_result_.half_range_dynamic_input_max_deg) &&
         h_gate >= calibration_result_.half_range_dynamic_input_min_deg &&
         h_gate <= calibration_result_.half_range_dynamic_input_max_deg;
}

bool ExperimentRunner::configureV60RebuildTarget() {
  // V61 targets a *state*, not a raw peak.  It commands the side opposite to
  // the fixed-Q arrival, then predicts the next two unforced peaks with the
  // measured side-specific maps.  The predicted H/C must land in the unchanged
  // narrow V59 gate before the cooldown is allowed.
  const float h_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG);
  const float c_gate = 0.5f * (Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG +
                               Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG);
  calibration_result_.v60_gate_center_h_deg = h_gate;
  calibration_result_.v60_gate_center_c_deg = c_gate;
  calibration_result_.v60_rebuild_target_valid = false;
  calibration_result_.v60_rebuild_target_observed = false;
  calibration_result_.v61_state_target_valid = false;
  calibration_result_.v61_predicted_gate_h_deg = 0.0f;
  calibration_result_.v61_predicted_gate_c_deg = 0.0f;
  calibration_result_.v61_feedback_command_peak_deg = 0.0f;
  calibration_result_.v61_feedback_correction_count = 0;
  calibration_v61_rebuild_plan_active_ = false;
  calibration_v61_nominal_controlled_peak_target_deg_ = 0.0f;
  calibration_v61_command_peak_target_deg_ = 0.0f;
  calibration_result_.v60_gate_center_in_dynamic_h_input_support =
      v60GateCenterInDynamicHInputSupport();
  if (!calibration_result_.v60_gate_center_in_dynamic_h_input_support ||
      calibration_result_.free_model_pos == 0 || calibration_result_.free_model_neg == 0) {
    return false;
  }
  const float r_pos = calibration_result_.r_pos;
  const float c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg;
  const float c_neg = calibration_result_.c_neg_deg;
  if (!isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) ||
      r_pos <= 0.0f || r_neg <= 0.0f) return false;

  float a_pos = 0.0f;
  float a_neg = 0.0f;
  float controlled_peak = 0.0f;
  if (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0) {
    // controlled negative -> free positive -> free negative -> fixed-Q positive
    a_pos = (2.0f * h_gate - c_neg) / (1.0f + r_neg);
    controlled_peak = (a_pos - c_pos) / r_pos;
    a_neg = r_neg * a_pos + c_neg;
  } else {
    // controlled positive -> free negative -> free positive -> fixed-Q negative
    a_neg = (2.0f * h_gate - c_pos) / (1.0f + r_pos);
    controlled_peak = (a_neg - c_neg) / r_neg;
    a_pos = r_pos * a_neg + c_pos;
  }
  const float predicted_h = 0.5f * (a_pos + a_neg);
  const float predicted_c = 0.5f * (a_pos - a_neg);
  const bool controlled_in_support =
      (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0 &&
       controlled_peak >= calibration_result_.free_input_min_pos_deg &&
       controlled_peak <= calibration_result_.free_input_max_pos_deg &&
       a_pos >= calibration_result_.free_input_min_neg_deg &&
       a_pos <= calibration_result_.free_input_max_neg_deg) ||
      (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE < 0 &&
       controlled_peak >= calibration_result_.free_input_min_neg_deg &&
       controlled_peak <= calibration_result_.free_input_max_neg_deg &&
       a_neg >= calibration_result_.free_input_min_pos_deg &&
       a_neg <= calibration_result_.free_input_max_pos_deg);
  const bool predicted_in_gate = predicted_h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG &&
      predicted_h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG &&
      predicted_c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG &&
      predicted_c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG;
  if (!controlled_in_support || !predicted_in_gate || !isfinite(controlled_peak) ||
      controlled_peak <= 0.0f || !isfinite(predicted_h) || !isfinite(predicted_c)) return false;

  const float h_r = calibration_result_.half_range_dynamic_r;
  const float h_c = calibration_result_.half_range_dynamic_c_deg;
  const float h_after_cycle = h_r * (h_r * h_gate + h_c) + h_c;
  calibration_result_.v60_free_decay_after_one_cycle_h_deg = h_after_cycle;
  calibration_result_.v60_decay_per_cycle_h_deg = h_gate - h_after_cycle;
  calibration_result_.v60_rebuild_target_h_deg = predicted_h;
  calibration_result_.v60_rebuild_target_peak_deg = controlled_peak;
  calibration_result_.v60_rebuild_target_valid = true;
  calibration_result_.v61_state_target_valid = true;
  calibration_result_.v61_predicted_gate_h_deg = predicted_h;
  calibration_result_.v61_predicted_gate_c_deg = predicted_c;
  calibration_result_.v61_feedback_command_peak_deg = controlled_peak;
  calibration_v61_nominal_controlled_peak_target_deg_ = controlled_peak;
  calibration_v61_command_peak_target_deg_ = controlled_peak;
  calibration_v61_rebuild_plan_active_ = true;
  return true;
}

bool ExperimentRunner::predictV61GateState(float controlled_peak_abs_deg, float* predicted_h_deg,
                                            float* predicted_c_deg) const {
  if (predicted_h_deg) *predicted_h_deg = NAN;
  if (predicted_c_deg) *predicted_c_deg = NAN;
  if (!calibration_result_.v61_state_target_valid || !isfinite(controlled_peak_abs_deg) ||
      controlled_peak_abs_deg <= 0.0f) return false;
  const float r_pos = calibration_result_.r_pos;
  const float c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg;
  const float c_neg = calibration_result_.c_neg_deg;
  if (!isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) ||
      r_pos <= 0.0f || r_neg <= 0.0f) return false;
  float a_pos = 0.0f;
  float a_neg = 0.0f;
  if (Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0) {
    a_neg = controlled_peak_abs_deg;
    if (a_neg < calibration_result_.free_input_min_pos_deg ||
        a_neg > calibration_result_.free_input_max_pos_deg) return false;
    a_pos = r_pos * a_neg + c_pos;
    if (a_pos < calibration_result_.free_input_min_neg_deg ||
        a_pos > calibration_result_.free_input_max_neg_deg) return false;
    a_neg = r_neg * a_pos + c_neg;
  } else {
    a_pos = controlled_peak_abs_deg;
    if (a_pos < calibration_result_.free_input_min_neg_deg ||
        a_pos > calibration_result_.free_input_max_neg_deg) return false;
    a_neg = r_neg * a_pos + c_neg;
    if (a_neg < calibration_result_.free_input_min_pos_deg ||
        a_neg > calibration_result_.free_input_max_pos_deg) return false;
    a_pos = r_pos * a_neg + c_pos;
  }
  const float h = 0.5f * (a_pos + a_neg);
  const float c = 0.5f * (a_pos - a_neg);
  if (!isfinite(h) || !isfinite(c) || h <= 0.0f) return false;
  if (predicted_h_deg) *predicted_h_deg = h;
  if (predicted_c_deg) *predicted_c_deg = c;
  return true;
}

void ExperimentRunner::addV62RateStateSample(float hprev_deg, float cprev_deg, float abs_rate_dps) {
  if (!isfinite(hprev_deg) || !isfinite(cprev_deg) || !isfinite(abs_rate_dps) || hprev_deg <= 0.0f || abs_rate_dps <= 0.0f) return;
  RateStateFit& fit = calibration_v62_rate_fit_;
  if (fit.count == 0) {
    fit.h_min_deg = fit.h_max_deg = hprev_deg;
    fit.c_min_deg = fit.c_max_deg = cprev_deg;
  } else {
    fit.h_min_deg = fminf(fit.h_min_deg, hprev_deg); fit.h_max_deg = fmaxf(fit.h_max_deg, hprev_deg);
    fit.c_min_deg = fminf(fit.c_min_deg, cprev_deg); fit.c_max_deg = fmaxf(fit.c_max_deg, cprev_deg);
  }
  ++fit.count;
  fit.sum_h += hprev_deg; fit.sum_c += cprev_deg; fit.sum_rate += abs_rate_dps;
  fit.sum_h2 += hprev_deg * hprev_deg; fit.sum_hc += hprev_deg * cprev_deg; fit.sum_c2 += cprev_deg * cprev_deg;
  fit.sum_hrate += hprev_deg * abs_rate_dps; fit.sum_crate += cprev_deg * abs_rate_dps; fit.sum_rate2 += abs_rate_dps * abs_rate_dps;
  calibration_result_.v62_rate_sample_count = fit.count;
}

bool ExperimentRunner::fitV62RateStateModel() {
  RateStateFit& fit = calibration_v62_rate_fit_;
  calibration_result_.v62_rate_model_valid = false;
  calibration_result_.v62_rate_sample_count = fit.count;
  if (fit.count < Config::ZERO_CROSS_V62_RATE_MODEL_MIN_SAMPLES) return false;
  float m[3][4] = {{fit.sum_h2 + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_hc, fit.sum_h, fit.sum_hrate},
                   {fit.sum_hc, fit.sum_c2 + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_c, fit.sum_crate},
                   {fit.sum_h, fit.sum_c, static_cast<float>(fit.count) + Config::ZERO_CROSS_V62_RATE_MODEL_RIDGE, fit.sum_rate}};
  for (uint8_t col = 0; col < 3; ++col) {
    uint8_t pivot = col;
    for (uint8_t row = col + 1; row < 3; ++row) if (fabsf(m[row][col]) > fabsf(m[pivot][col])) pivot = row;
    if (!isfinite(m[pivot][col]) || fabsf(m[pivot][col]) < 1.0e-5f) return false;
    if (pivot != col) for (uint8_t j = col; j < 4; ++j) { const float t = m[col][j]; m[col][j] = m[pivot][j]; m[pivot][j] = t; }
    const float divisor = m[col][col];
    for (uint8_t j = col; j < 4; ++j) m[col][j] /= divisor;
    for (uint8_t row = 0; row < 3; ++row) if (row != col) { const float factor = m[row][col]; for (uint8_t j = col; j < 4; ++j) m[row][j] -= factor * m[col][j]; }
  }
  fit.a_per_s = m[0][3]; fit.b_per_s = m[1][3]; fit.offset_dps = m[2][3];
  const float sse = fit.sum_rate2 - 2.0f * (fit.a_per_s * fit.sum_hrate + fit.b_per_s * fit.sum_crate + fit.offset_dps * fit.sum_rate) + fit.a_per_s * fit.a_per_s * fit.sum_h2 + fit.b_per_s * fit.b_per_s * fit.sum_c2 + fit.offset_dps * fit.offset_dps * static_cast<float>(fit.count) + 2.0f * fit.a_per_s * fit.b_per_s * fit.sum_hc + 2.0f * fit.a_per_s * fit.offset_dps * fit.sum_h + 2.0f * fit.b_per_s * fit.offset_dps * fit.sum_c;
  const float mean = fit.sum_rate / static_cast<float>(fit.count);
  const float sst = fit.sum_rate2 - static_cast<float>(fit.count) * mean * mean;
  fit.r2 = sst > 1.0e-4f ? 1.0f - fmaxf(0.0f, sse) / sst : NAN;
  calibration_result_.v62_rate_a_per_s = fit.a_per_s; calibration_result_.v62_rate_b_per_s = fit.b_per_s;
  calibration_result_.v62_rate_offset_dps = fit.offset_dps; calibration_result_.v62_rate_r2 = fit.r2;
  calibration_result_.v62_rate_model_valid = isfinite(fit.a_per_s) && isfinite(fit.b_per_s) && isfinite(fit.offset_dps) && isfinite(fit.r2) && fit.r2 >= Config::ZERO_CROSS_V62_RATE_MODEL_MIN_R2;
  return calibration_result_.v62_rate_model_valid;
}

bool ExperimentRunner::v62HasFeasibleHCTarget() const {
  const float r_pos = calibration_result_.r_pos, c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg, c_neg = calibration_result_.c_neg_deg;
  if (calibration_result_.free_model_pos == 0 || calibration_result_.free_model_neg == 0 || !calibration_result_.half_range_dynamic_valid || !isfinite(r_pos) || !isfinite(c_pos) || !isfinite(r_neg) || !isfinite(c_neg) || r_pos <= 0.0f || r_neg <= 0.0f) return false;
  const bool desired_positive = Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0;
  const float low = desired_positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float high = desired_positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  for (float controlled = low; controlled <= high + 0.0001f; controlled += Config::ZERO_CROSS_V62_TARGET_SCAN_STEP_DEG) {
    float a_pos = 0.0f, a_neg = 0.0f; bool supported = false;
    if (desired_positive) { a_neg = controlled; a_pos = r_pos * a_neg + c_pos; supported = a_pos >= calibration_result_.free_input_min_neg_deg && a_pos <= calibration_result_.free_input_max_neg_deg; a_neg = r_neg * a_pos + c_neg; }
    else { a_pos = controlled; a_neg = r_neg * a_pos + c_neg; supported = a_neg >= calibration_result_.free_input_min_pos_deg && a_neg <= calibration_result_.free_input_max_pos_deg; a_pos = r_pos * a_neg + c_pos; }
    const float h = 0.5f * (a_pos + a_neg), c = 0.5f * (a_pos - a_neg);
    if (supported && isfinite(h) && isfinite(c) && h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG && h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG && c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG && c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG && h >= calibration_result_.half_range_dynamic_input_min_deg && h <= calibration_result_.half_range_dynamic_input_max_deg) return true;
  }
  return false;
}

bool ExperimentRunner::configureV62RebuildTarget() {
  calibration_result_.v62_hc_target_feasible = v62HasFeasibleHCTarget();
  calibration_result_.v62_state_target_valid = false;
  if (!calibration_result_.v62_hc_target_feasible || !calibration_result_.v62_rate_model_valid) return false;
  const RateStateFit& fit = calibration_v62_rate_fit_;
  const float r_pos = calibration_result_.r_pos, c_pos = calibration_result_.c_pos_deg;
  const float r_neg = calibration_result_.r_neg, c_neg = calibration_result_.c_neg_deg;
  const bool desired_positive = Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE > 0;
  const float low = desired_positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float high = desired_positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  float best_score = -1.0f, best_controlled = 0.0f, best_h = 0.0f, best_c = 0.0f, best_rate = 0.0f;
  for (float controlled = low; controlled <= high + 0.0001f; controlled += Config::ZERO_CROSS_V62_TARGET_SCAN_STEP_DEG) {
    float a_pos = 0.0f, a_neg = 0.0f; bool supported = false;
    if (desired_positive) { a_neg = controlled; a_pos = r_pos * a_neg + c_pos; supported = a_pos >= calibration_result_.free_input_min_neg_deg && a_pos <= calibration_result_.free_input_max_neg_deg; a_neg = r_neg * a_pos + c_neg; }
    else { a_pos = controlled; a_neg = r_neg * a_pos + c_neg; supported = a_neg >= calibration_result_.free_input_min_pos_deg && a_neg <= calibration_result_.free_input_max_pos_deg; a_pos = r_pos * a_neg + c_pos; }
    const float h = 0.5f * (a_pos + a_neg), c = 0.5f * (a_pos - a_neg), rate = fit.a_per_s * h + fit.b_per_s * c + fit.offset_dps;
    const bool inside = supported && isfinite(h) && isfinite(c) && isfinite(rate) && h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG && h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG && c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG && c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG && rate >= Config::ZERO_CROSS_V59_GATE_ABS_RATE_MIN_DPS && rate <= Config::ZERO_CROSS_V59_GATE_ABS_RATE_MAX_DPS && h >= calibration_result_.half_range_dynamic_input_min_deg && h <= calibration_result_.half_range_dynamic_input_max_deg && h >= fit.h_min_deg && h <= fit.h_max_deg && c >= fit.c_min_deg && c <= fit.c_max_deg;
    if (!inside) continue;
    const float score = fminf(fminf(h - Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG, Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG - h) / 0.125f, fminf(c - Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG, Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG - c) / 0.30f);
    if (score > best_score) { best_score = score; best_controlled = controlled; best_h = h; best_c = c; best_rate = rate; }
  }
  if (best_score < 0.0f) return false;
  calibration_result_.v62_state_target_valid = true;
  calibration_result_.v62_target_h_deg = best_h; calibration_result_.v62_target_c_deg = best_c; calibration_result_.v62_target_rate_dps = best_rate; calibration_result_.v62_target_controlled_peak_deg = best_controlled;
  calibration_result_.v60_rebuild_target_valid = true; calibration_result_.v60_rebuild_target_h_deg = best_h; calibration_result_.v60_rebuild_target_peak_deg = best_controlled;
  calibration_result_.v61_state_target_valid = true; calibration_result_.v61_predicted_gate_h_deg = best_h; calibration_result_.v61_predicted_gate_c_deg = best_c; calibration_result_.v61_feedback_command_peak_deg = best_controlled;
  calibration_v61_nominal_controlled_peak_target_deg_ = best_controlled; calibration_v61_command_peak_target_deg_ = best_controlled; calibration_v61_rebuild_plan_active_ = true;
  return true;
}
float ExperimentRunner::halfRangeShadowPrediction(float previous_half_range_deg, bool dynamic,
                                                   bool* in_support) const {
  if (in_support) *in_support = false;
  const bool valid = dynamic ? calibration_result_.half_range_dynamic_valid :
                               calibration_result_.half_range_fixed_valid;
  const float min_input = dynamic ? calibration_result_.half_range_dynamic_input_min_deg :
                                    calibration_result_.half_range_fixed_input_min_deg;
  const float max_input = dynamic ? calibration_result_.half_range_dynamic_input_max_deg :
                                    calibration_result_.half_range_fixed_input_max_deg;
  if (!valid || !isfinite(previous_half_range_deg) || previous_half_range_deg < min_input ||
      previous_half_range_deg > max_input) return NAN;
  if (in_support) *in_support = true;
  const float r = dynamic ? calibration_result_.half_range_dynamic_r : calibration_result_.half_range_fixed_r;
  const float c = dynamic ? calibration_result_.half_range_dynamic_c_deg : calibration_result_.half_range_fixed_c_deg;
  return r * previous_half_range_deg + c;
}

bool ExperimentRunner::dynamicHalfRangePredecessorInSupport() const {
  if (!calibration_last_half_range_valid_) return false;
  bool in_support = false;
  (void)halfRangeShadowPrediction(calibration_last_half_range_dynamic_deg_, true, &in_support);
  return in_support;
}

bool ExperimentRunner::fitCalibrationFreeDecay() {
  // Half-cycle amplitudes may be asymmetric. Each arrival-side map must stay
  // positive and monotone over its measured A_prev domain; passive decay is
  // tested on the composed same-side full cycle, not on either half cycle alone.
  auto evaluate = [&](const CalibrationFit& input, bool positive) {
    const float n = static_cast<float>(input.count);
    const bool enough = input.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE;
    const float input_min = input.input_min_deg;
    const float input_max = input.input_max_deg;
    const float sst = n > 0.0f ? input.sum_y2 - input.sum_y * input.sum_y / n : 0.0f;
    const bool usable_input = isfinite(input_min) && isfinite(input_max) &&
                              input_min > 0.0f && input_max >= input_min;
    float& prop_r = positive ? calibration_result_.free_proportional_r_pos : calibration_result_.free_proportional_r_neg;
    float& prop_rmse = positive ? calibration_result_.free_proportional_rmse_pos_deg : calibration_result_.free_proportional_rmse_neg_deg;
    float& prop_r2 = positive ? calibration_result_.free_proportional_r2_pos : calibration_result_.free_proportional_r2_neg;
    float& prop_loocv = positive ? calibration_result_.free_proportional_loocv_rmse_pos_deg : calibration_result_.free_proportional_loocv_rmse_neg_deg;
    bool& prop_valid = positive ? calibration_result_.free_proportional_valid_pos : calibration_result_.free_proportional_valid_neg;
    float& affine_r = positive ? calibration_result_.free_affine_r_pos : calibration_result_.free_affine_r_neg;
    float& affine_c = positive ? calibration_result_.free_affine_c_pos_deg : calibration_result_.free_affine_c_neg_deg;
    float& affine_rmse = positive ? calibration_result_.free_affine_rmse_pos_deg : calibration_result_.free_affine_rmse_neg_deg;
    float& affine_r2 = positive ? calibration_result_.free_affine_r2_pos : calibration_result_.free_affine_r2_neg;
    float& affine_loocv = positive ? calibration_result_.free_affine_loocv_rmse_pos_deg : calibration_result_.free_affine_loocv_rmse_neg_deg;
    bool& affine_valid = positive ? calibration_result_.free_affine_valid_pos : calibration_result_.free_affine_valid_neg;
    uint8_t& model = positive ? calibration_result_.free_model_pos : calibration_result_.free_model_neg;
    float& selected_r = positive ? calibration_result_.r_pos : calibration_result_.r_neg;
    float& selected_c = positive ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg;

    auto loocvRmse = [&](bool affine) {
      if (input.count < 3) return NAN;
      float total_squared_error = 0.0f;
      for (uint8_t held_out = 0; held_out < input.count; ++held_out) {
        float sx = 0.0f, sy = 0.0f, sx2 = 0.0f, sxy = 0.0f;
        for (uint8_t i = 0; i < input.count; ++i) {
          if (i == held_out) continue;
          const float x = input.x_deg[i], y = input.y_deg[i];
          sx += x; sy += y; sx2 += x * x; sxy += x * y;
        }
        const float train_n = static_cast<float>(input.count - 1);
        float r = 0.0f, c = 0.0f;
        if (affine) {
          const float denom = train_n * sx2 - sx * sx;
          if (fabsf(denom) < 1.0e-6f) return NAN;
          r = (train_n * sxy - sx * sy) / denom;
          c = (sy - r * sx) / train_n;
        } else {
          if (fabsf(sx2) < 1.0e-6f) return NAN;
          r = sxy / sx2;
        }
        const float error = r * input.x_deg[held_out] + c - input.y_deg[held_out];
        if (!isfinite(error)) return NAN;
        total_squared_error += error * error;
      }
      return sqrtf(total_squared_error / n);
    };

    model = 0; selected_r = 0.0f; selected_c = 0.0f;
    prop_r = 0.0f; prop_rmse = 0.0f; prop_r2 = 0.0f; prop_loocv = 0.0f; prop_valid = false;
    affine_r = 0.0f; affine_c = 0.0f; affine_rmse = 0.0f; affine_r2 = 0.0f; affine_loocv = 0.0f; affine_valid = false;
    if (n <= 0.0f) return false;

    if (fabsf(input.sum_x2) > 1.0e-6f) {
      prop_r = input.sum_xy / input.sum_x2;
      const float sse = input.sum_y2 - 2.0f * prop_r * input.sum_xy + prop_r * prop_r * input.sum_x2;
      prop_rmse = sqrtf(fmaxf(0.0f, sse / n));
      prop_r2 = sst > 1.0e-6f ? 1.0f - sse / sst : 0.0f;
      prop_loocv = loocvRmse(false);
      const float low = prop_r * input_min;
      const float high = prop_r * input_max;
      prop_valid = enough && usable_input && isfinite(prop_r) && isfinite(prop_rmse) && isfinite(prop_loocv) &&
                   prop_r > 0.0f && low > 0.0f && high > 0.0f;
    }

    const float denom = n * input.sum_x2 - input.sum_x * input.sum_x;
    const float variance = n > 0.0f ? denom / (n * n) : 0.0f;
    if (fabsf(denom) > 1.0e-6f && variance >= Config::ZERO_CROSS_CALIBRATION_MIN_X_VARIANCE_DEG2) {
      affine_r = (n * input.sum_xy - input.sum_x * input.sum_y) / denom;
      affine_c = (input.sum_y - affine_r * input.sum_x) / n;
      const float sse = input.sum_y2 - 2.0f * affine_r * input.sum_xy - 2.0f * affine_c * input.sum_y +
                        affine_r * affine_r * input.sum_x2 + 2.0f * affine_r * affine_c * input.sum_x + n * affine_c * affine_c;
      affine_rmse = sqrtf(fmaxf(0.0f, sse / n));
      affine_r2 = sst > 1.0e-6f ? 1.0f - sse / sst : 0.0f;
      affine_loocv = loocvRmse(true);
      const float low = affine_r * input_min + affine_c;
      const float high = affine_r * input_max + affine_c;
      affine_valid = enough && usable_input && isfinite(affine_r) && isfinite(affine_c) &&
                     isfinite(affine_rmse) && isfinite(affine_loocv) && affine_r > 0.0f &&
                     low > 0.0f && high > 0.0f;
    }

    if (affine_valid && (!prop_valid || affine_loocv < prop_loocv)) {
      model = 2; selected_r = affine_r; selected_c = affine_c;
    } else if (prop_valid) {
      model = 1; selected_r = prop_r; selected_c = 0.0f;
    }
    return model != 0;
  };

  auto evaluateFullCycle = [&](float first_r, float first_c, float first_input_min, float first_input_max,
                               float second_r, float second_c, float second_input_min, float second_input_max,
                               float& out_r, float& out_c, float& out_input_min, float& out_input_max,
                               bool& out_valid) {
    out_r = 0.0f; out_c = 0.0f; out_input_min = 0.0f; out_input_max = 0.0f; out_valid = false;
    if (!isfinite(first_r) || !isfinite(first_c) || !isfinite(second_r) || !isfinite(second_c) ||
        first_r <= 0.0f || second_r <= 0.0f || first_input_min <= 0.0f ||
        first_input_max < first_input_min || second_input_min <= 0.0f ||
        second_input_max < second_input_min) return false;
    const float composition_min = fmaxf(first_input_min, (second_input_min - first_c) / first_r);
    const float composition_max = fminf(first_input_max, (second_input_max - first_c) / first_r);
    const float r = second_r * first_r;
    const float c = second_r * first_c + second_c;
    const float low = r * composition_min + c;
    const float high = r * composition_max + c;
    out_r = r; out_c = c; out_input_min = composition_min; out_input_max = composition_max;
    out_valid = isfinite(composition_min) && isfinite(composition_max) && isfinite(r) && isfinite(c) &&
                composition_min > 0.0f && composition_max > composition_min && r > 0.0f &&
                isfinite(low) && isfinite(high) && low > 0.0f && high > 0.0f &&
                low < composition_min && high < composition_max;
    return out_valid;
  };

  calibration_result_.initial_kick_count = calibration_initial_kick_count_;
  calibration_result_.free_count_pos = calibration_fit_pos_.count;
  calibration_result_.free_count_neg = calibration_fit_neg_.count;
  calibration_result_.free_support_min_pos_deg = calibration_fit_pos_.support_min_deg;
  calibration_result_.free_support_max_pos_deg = calibration_fit_pos_.support_max_deg;
  calibration_result_.free_support_min_neg_deg = calibration_fit_neg_.support_min_deg;
  calibration_result_.free_support_max_neg_deg = calibration_fit_neg_.support_max_deg;
  calibration_result_.free_input_min_pos_deg = calibration_fit_pos_.input_min_deg;
  calibration_result_.free_input_max_pos_deg = calibration_fit_pos_.input_max_deg;
  calibration_result_.free_input_min_neg_deg = calibration_fit_neg_.input_min_deg;
  calibration_result_.free_input_max_neg_deg = calibration_fit_neg_.input_max_deg;
  const bool pos_ok = evaluate(calibration_fit_pos_, true);
  const bool neg_ok = evaluate(calibration_fit_neg_, false);
  calibration_result_.free_halfcycle_monotone_pos = pos_ok;
  calibration_result_.free_halfcycle_monotone_neg = neg_ok;
  const bool full_pos = pos_ok && neg_ok && evaluateFullCycle(
      calibration_result_.r_neg, calibration_result_.c_neg_deg,
      calibration_result_.free_input_min_neg_deg, calibration_result_.free_input_max_neg_deg,
      calibration_result_.r_pos, calibration_result_.c_pos_deg,
      calibration_result_.free_input_min_pos_deg, calibration_result_.free_input_max_pos_deg,
      calibration_result_.free_full_cycle_r_pos, calibration_result_.free_full_cycle_c_pos_deg,
      calibration_result_.free_full_cycle_input_min_pos_deg, calibration_result_.free_full_cycle_input_max_pos_deg,
      calibration_result_.free_full_cycle_valid_pos);
  const bool full_neg = pos_ok && neg_ok && evaluateFullCycle(
      calibration_result_.r_pos, calibration_result_.c_pos_deg,
      calibration_result_.free_input_min_pos_deg, calibration_result_.free_input_max_pos_deg,
      calibration_result_.r_neg, calibration_result_.c_neg_deg,
      calibration_result_.free_input_min_neg_deg, calibration_result_.free_input_max_neg_deg,
      calibration_result_.free_full_cycle_r_neg, calibration_result_.free_full_cycle_c_neg_deg,
      calibration_result_.free_full_cycle_input_min_neg_deg, calibration_result_.free_full_cycle_input_max_neg_deg,
      calibration_result_.free_full_cycle_valid_neg);
  return pos_ok && neg_ok && full_pos && full_neg;
}
void ExperimentRunner::finishCalibrationShadow(uint8_t failure_reason) {
  if (!calibration_enabled_) return;
  calibration_result_.initial_kick_count = calibration_initial_kick_count_;
  calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
  calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
  calibration_result_.rebuild_attempt_in_episode = calibration_rebuild_attempt_in_episode_;
  calibration_result_.free_count_pos = calibration_fit_pos_.count;
  calibration_result_.free_count_neg = calibration_fit_neg_.count;
  calibration_result_.failure_reason = failure_reason;
  calibration_result_.probe_plan_count = calibration_probe_plan_index_;
  calibration_result_.probe_wait_halfcycles = calibration_probe_wait_halfcycle_total_;
  calibration_result_.v59_gate_event_count = calibration_v59_gate_event_count_;
  calibration_result_.v59_gate_pass_count = calibration_v59_gate_pass_count_;
  calibration_result_.v59_gate_skip_count = calibration_v59_gate_skip_count_;
  calibration_result_.v59_rebuild_from_low_state_count = calibration_v59_rebuild_from_low_state_count_;
  calibration_result_.protocol_complete = failure_reason == 0 &&
      calibration_probe_plan_index_ == Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT;
  // The V59 response set intentionally contains only +arrival events, so the
  // historical signed two-side gain summary remains invalid by design.
  calibration_result_.valid = calibration_result_.protocol_complete &&
                              calibration_result_.q_count_pos >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE &&
                              calibration_result_.q_count_neg >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE &&
                              isfinite(calibration_result_.g_pos) && isfinite(calibration_result_.g_neg) &&
                              calibration_result_.g_pos > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS &&
                              calibration_result_.g_neg > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS;
  calibration_phase_ = CalibrationPhase::MAIN_CONTROL;
  if (logger_) logger_->setCalibrationResult(calibration_result_);
  // V59 is a measurement protocol, not a handoff to the normal controller.
  // Preserve the completed/failed RWLOG and end its LED signature immediately.
  if (Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED) finishRun();
}

float ExperimentRunner::calibratedPredictionDeg(float q_effective_pred_mA_s, int8_t next_peak_side) const {
  if (!calibration_result_.valid || !calibration_last_peak_valid_) return NAN;
  const bool positive = next_peak_side >= 0;
  const float support_min = positive ? calibration_result_.free_input_min_pos_deg : calibration_result_.free_input_min_neg_deg;
  const float support_max = positive ? calibration_result_.free_input_max_pos_deg : calibration_result_.free_input_max_neg_deg;
  // The selected free-decay fit is identification data, not an extrapolating
  // controller. Outside its observed amplitude support, do not emit a shadow prediction.
  if (calibration_last_peak_abs_deg_ < support_min || calibration_last_peak_abs_deg_ > support_max) return NAN;
  const float r = positive ? calibration_result_.r_pos : calibration_result_.r_neg;
  const float g = positive ? calibration_result_.g_pos : calibration_result_.g_neg;
  const float c = positive ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg;
  return r * calibration_last_peak_abs_deg_ + g * q_effective_pred_mA_s + c;
}

void ExperimentRunner::updateCalibrationPeak(uint32_t now_ms, uint32_t t_test_ms, float angle_deg, float rate_dps) {
  (void)now_ms;
  if (!calibration_enabled_) return;
  if (!calibration_peak_tracker_ready_) {
    // The first candidate can be armed from either moving side. After every
    // confirmed peak, however, only the opposite-side *outbound* leg may arm
    // the next candidate. This prevents the confirmation sample from being
    // reused as a stale same-side extremum.
    if (calibration_expected_candidate_side_ != 0) {
      const float expected = static_cast<float>(calibration_expected_candidate_side_);
      if (expected * angle_deg <= 0.0f ||
          expected * rate_dps < Config::ZERO_CROSS_MIN_RATE_DPS) return;
    } else if (fabsf(rate_dps) < Config::ZERO_CROSS_MIN_RATE_DPS) {
      return;
    }
    calibration_peak_tracker_ready_ = true;
    calibration_outbound_rate_sign_ = rate_dps >= 0.0f ? 1 : -1;
    calibration_peak_candidate_deg_ = angle_deg;
    calibration_peak_candidate_fixed_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_FIXED_B100_INDEX];
    calibration_peak_candidate_ms_ = t_test_ms;
    calibration_expected_candidate_side_ = 0;
    return;
  }

  // Once the rate has reversed, freeze the extremum candidate.  It remains
  // frozen through the confirmation window so the following half-cycle cannot
  // overwrite either its value, sign, or physical occurrence time.
  const bool reversing = rate_dps * calibration_outbound_rate_sign_ <=
                         -Config::ZERO_CROSS_MIN_RATE_DPS;
  if (!reversing) {
    calibration_reverse_samples_ = 0;
    if (fabsf(angle_deg) > fabsf(calibration_peak_candidate_deg_)) {
      calibration_peak_candidate_deg_ = angle_deg;
      calibration_peak_candidate_fixed_deg_ = status_.pitch_dynamic_beta_deg[Config::FILTER_FIXED_B100_INDEX];
      calibration_peak_candidate_ms_ = t_test_ms;
    }
    return;
  }
  if (++calibration_reverse_samples_ < Config::ZERO_CROSS_IDENTIFICATION_PEAK_CONFIRM_SAMPLES) return;

  calibration_reverse_samples_ = 0;
  const float peak_signed_dynamic = calibration_peak_candidate_deg_;
  const float peak_signed_fixed = calibration_peak_candidate_fixed_deg_;
  const float peak_abs = fabsf(peak_signed_dynamic);
  const int8_t side = peak_signed_dynamic >= 0.0f ? 1 : -1;
  const uint32_t candidate_peak_ms = calibration_peak_candidate_ms_;
  const uint32_t confirmed_ms = t_test_ms;
  // Do not reuse the confirmation-time sample as a candidate for the next
  // turn. It still lies on this peak's side and can be larger than the next
  // opposite-side extremum. Clear it, then arm only after a genuine opposite
  // outbound leg is observed.
  calibration_peak_tracker_ready_ = false;
  calibration_expected_candidate_side_ = -side;
  calibration_outbound_rate_sign_ = 0;
  calibration_peak_candidate_deg_ = 0.0f;
  calibration_peak_candidate_fixed_deg_ = 0.0f;
  calibration_peak_candidate_ms_ = 0;

  auto isConsecutive = [&]() {
    return calibration_last_peak_valid_ && candidate_peak_ms > calibration_last_peak_candidate_ms_ &&
           candidate_peak_ms - calibration_last_peak_candidate_ms_ <=
               Config::ZERO_CROSS_CALIBRATION_MAX_CONSECUTIVE_PEAK_GAP_MS;
  };
  auto setLastPeak = [&]() {
    if (calibration_last_peak_valid_) {
      calibration_last_half_range_dynamic_deg_ = 0.5f * fabsf(
          peak_signed_dynamic - calibration_last_peak_signed_dynamic_deg_);
      calibration_last_half_range_fixed_deg_ = 0.5f * fabsf(
          peak_signed_fixed - calibration_last_peak_signed_fixed_deg_);
      calibration_last_half_range_valid_ = true;
      // This is the center of the two already confirmed peaks. It is the
      // online C_prev audit state, distinct from the post-pulse event center.
      calibration_last_center_dynamic_deg_ = 0.5f * (
          peak_signed_dynamic + calibration_last_peak_signed_dynamic_deg_);
      calibration_last_center_dynamic_valid_ = true;
    } else {
      calibration_last_half_range_valid_ = false;
      calibration_last_center_dynamic_valid_ = false;
    }
    calibration_last_peak_valid_ = true;
    calibration_last_peak_side_ = side;
    calibration_last_peak_abs_deg_ = peak_abs;
    calibration_last_peak_signed_dynamic_deg_ = peak_signed_dynamic;
    calibration_last_peak_signed_fixed_deg_ = peak_signed_fixed;
    calibration_last_peak_candidate_ms_ = candidate_peak_ms;
  };
  auto addProbeEvent = [&](bool post_pulse, bool in_support, bool side_matched,
                            bool positive_gain, bool gain_stored, uint8_t result_code, float gain) {
    if (!logger_) return;
    PsramLogger::CalibrationProbeEvent event;
    event.planned_probe_index = calibration_probe_active_plan_index_;
    event.q_level_index = calibration_probe_q_level_index_;
    event.q_probe_schedule_id = q_probe_schedule_id_;
    event.polarity_mode = 0;  // BASE: direction = -requested_side.
    event.rebuild_count = calibration_probe_rebuild_total_count_;
    event.rebuild_episode_id = calibration_probe_rebuild_episode_id_;
    event.rebuild_attempt_in_episode = calibration_probe_rebuild_attempt_in_episode_;
    event.rebuild_entry_source = calibration_probe_rebuild_entry_source_;
    event.wait_halfcycle_count = calibration_probe_wait_halfcycle_count_;
    event.result_code = result_code;
    event.pulse_start_ms = calibration_probe_pulse_start_test_ms_;
    event.candidate_peak_ms = candidate_peak_ms;
    event.confirmed_ms = confirmed_ms;
    event.previous_peak_candidate_ms = calibration_probe_previous_peak_candidate_ms_;
    event.requested_side = calibration_probe_requested_side_;
    event.predecessor_side = calibration_probe_predecessor_side_;
    event.observed_side = side;
    event.pulse_direction = calibration_probe_direction_;
    event.pulse_id = calibration_probe_pulse_id_;
    event.pulse_width_ms = status_.pulse_width_ms_setting;
    event.command_dynamic_angle_cdeg = centi(calibration_probe_command_dynamic_angle_deg_);
    event.command_rate_raw_cdps = centi(calibration_probe_command_rate_raw_dps_);
    event.command_rate_bias_corrected_cdps = centi(calibration_probe_command_rate_bias_corrected_dps_);
    event.previous_peak_abs_cdeg = centi(calibration_probe_previous_peak_abs_deg_);
    event.observed_peak_abs_cdeg = centi(peak_abs);
    event.previous_peak_signed_dynamic_cdeg = centi(calibration_probe_previous_peak_signed_dynamic_deg_);
    event.observed_peak_signed_dynamic_cdeg = centi(peak_signed_dynamic);
    event.center_dynamic_cdeg = centi(0.5f * (calibration_probe_previous_peak_signed_dynamic_deg_ +
                                              peak_signed_dynamic));
    const float observed_half_range_dynamic = 0.5f * fabsf(
        peak_signed_dynamic - calibration_probe_previous_peak_signed_dynamic_deg_);
    event.half_range_previous_dynamic_cdeg = calibration_probe_previous_half_range_valid_ ?
        centi(calibration_probe_previous_half_range_dynamic_deg_) : LOG_NAN_I16;
    event.half_range_observed_dynamic_cdeg = centi(observed_half_range_dynamic);
    bool dynamic_half_range_in_support = false;
    const float predicted_half_range_dynamic = post_pulse && calibration_probe_previous_half_range_valid_ ?
        halfRangeShadowPrediction(calibration_probe_previous_half_range_dynamic_deg_, true,
                                  &dynamic_half_range_in_support) : NAN;
    const float delta_half_range_dynamic = isfinite(predicted_half_range_dynamic) ?
        observed_half_range_dynamic - predicted_half_range_dynamic : NAN;
    const float gain_half_range_dynamic = isfinite(delta_half_range_dynamic) &&
        calibration_probe_q_effective_pred_mA_s_ > 0.0f ?
        delta_half_range_dynamic / calibration_probe_q_effective_pred_mA_s_ : NAN;
    event.half_range_free_pred_dynamic_cdeg = isfinite(predicted_half_range_dynamic) ?
        centi(predicted_half_range_dynamic) : LOG_NAN_I16;
    event.delta_half_range_dynamic_cdeg = isfinite(delta_half_range_dynamic) ?
        centi(delta_half_range_dynamic) : LOG_NAN_I16;
    event.gain_half_range_dynamic_cdeg_per_mAs = isfinite(gain_half_range_dynamic) ?
        centi(gain_half_range_dynamic) : LOG_NAN_I16;
    event.previous_peak_signed_fixed_cdeg = centi(calibration_probe_previous_peak_signed_fixed_deg_);
    event.observed_peak_signed_fixed_cdeg = centi(peak_signed_fixed);
    event.center_fixed_cdeg = centi(0.5f * (calibration_probe_previous_peak_signed_fixed_deg_ + peak_signed_fixed));
    const float observed_half_range_fixed = 0.5f * fabsf(
        peak_signed_fixed - calibration_probe_previous_peak_signed_fixed_deg_);
    event.half_range_previous_fixed_cdeg = calibration_probe_previous_half_range_valid_ ?
        centi(calibration_probe_previous_half_range_fixed_deg_) : LOG_NAN_I16;
    event.half_range_observed_fixed_cdeg = centi(observed_half_range_fixed);
    bool fixed_half_range_in_support = false;
    const float predicted_half_range_fixed = post_pulse && calibration_probe_previous_half_range_valid_ ?
        halfRangeShadowPrediction(calibration_probe_previous_half_range_fixed_deg_, false,
                                  &fixed_half_range_in_support) : NAN;
    const float delta_half_range_fixed = isfinite(predicted_half_range_fixed) ?
        observed_half_range_fixed - predicted_half_range_fixed : NAN;
    const float gain_half_range_fixed = isfinite(delta_half_range_fixed) &&
        calibration_probe_q_effective_pred_mA_s_ > 0.0f ?
        delta_half_range_fixed / calibration_probe_q_effective_pred_mA_s_ : NAN;
    event.half_range_free_pred_fixed_cdeg = isfinite(predicted_half_range_fixed) ?
        centi(predicted_half_range_fixed) : LOG_NAN_I16;
    event.delta_half_range_fixed_cdeg = isfinite(delta_half_range_fixed) ?
        centi(delta_half_range_fixed) : LOG_NAN_I16;
    event.gain_half_range_fixed_cdeg_per_mAs = isfinite(gain_half_range_fixed) ?
        centi(gain_half_range_fixed) : LOG_NAN_I16;
    event.half_range_dynamic_in_support = dynamic_half_range_in_support;
    event.half_range_fixed_in_support = fixed_half_range_in_support;
    event.rebuild_target_reached = calibration_probe_rebuild_target_reached_;
    event.dynamic_h_in_support_at_command = calibration_probe_dynamic_h_in_support_at_command_;
    event.q_target_mAms = lroundf(calibration_probe_q_target_mA_s_ * 1000.0f);
    event.q_effective_pred_mAms = lroundf(calibration_probe_q_effective_pred_mA_s_ * 1000.0f);
    event.gain_cdeg_per_mAs = isfinite(gain) ? centi(gain) : LOG_NAN_I16;

    // v53 remains disconnected from every Q selector and motor path. It keeps
    // the v52 forward model fixed, then records inverse-shadow diagnostics only.
    const float shadow_hprev = calibration_probe_previous_half_range_dynamic_deg_;
    const float shadow_c_prev = calibration_probe_previous_center_dynamic_deg_;
    const float shadow_abs_rate = fabsf(calibration_probe_command_rate_bias_corrected_dps_);
    const int8_t shadow_next_peak_side = calibration_probe_requested_side_;
    const float shadow_q_effective = calibration_probe_q_effective_pred_mA_s_;
    const bool shadow_state_valid = Config::ZERO_CROSS_V52_SHADOW_ENABLED &&
        Config::ZERO_CROSS_V53_INVERSE_SHADOW_ENABLED &&
        calibration_probe_previous_half_range_valid_ &&
        calibration_probe_previous_center_dynamic_valid_ &&
        isfinite(shadow_hprev) && isfinite(shadow_c_prev) && isfinite(shadow_abs_rate) &&
        (shadow_next_peak_side == -1 || shadow_next_peak_side == 1);
    const bool shadow_h_in_support = shadow_state_valid &&
        shadow_hprev >= Config::ZERO_CROSS_V53_STATE_HPREV_MIN_DEG &&
        shadow_hprev <= Config::ZERO_CROSS_V53_STATE_HPREV_MAX_DEG;
    const bool shadow_rate_in_support = shadow_state_valid &&
        shadow_abs_rate >= Config::ZERO_CROSS_V53_STATE_ABS_RATE_MIN_DPS &&
        shadow_abs_rate <= Config::ZERO_CROSS_V53_STATE_ABS_RATE_MAX_DPS;
    const bool shadow_state_in_support = shadow_h_in_support && shadow_rate_in_support;
    const bool shadow_q_in_model_support = isfinite(shadow_q_effective) &&
        shadow_q_effective >= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS &&
        shadow_q_effective <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS;
    event.shadow_hprev_imu_cdeg = shadow_state_valid ? centi(shadow_hprev) : LOG_NAN_I16;
    event.shadow_c_prev_imu_cdeg = shadow_state_valid ? centi(shadow_c_prev) : LOG_NAN_I16;
    event.shadow_rate_abs_cdps = shadow_state_valid ? centi(shadow_abs_rate) : LOG_NAN_I16;
    event.shadow_next_peak_side = shadow_next_peak_side;
    event.shadow_q_effective_pred_mA_s = shadow_q_effective;
    event.shadow_state_valid = shadow_state_valid;
    event.shadow_h_in_support = shadow_h_in_support;
    event.shadow_rate_in_support = shadow_rate_in_support;
    event.shadow_state_in_support = shadow_state_in_support;
    event.shadow_q_in_model_support = shadow_q_in_model_support;
    // v53 never becomes a command source, including inside the support range.
    event.shadow_control_candidate = false;

    const float shadow_base = shadow_state_valid
        ? Config::ZERO_CROSS_V52_SHADOW_INTERCEPT_DEG +
              Config::ZERO_CROSS_V52_SHADOW_HPREV_GAIN * shadow_hprev +
              Config::ZERO_CROSS_V52_SHADOW_ABS_RATE_GAIN_DEG_PER_DPS * shadow_abs_rate +
              Config::ZERO_CROSS_V52_SHADOW_NEXT_PEAK_SIDE_OFFSET_DEG * shadow_next_peak_side
        : NAN;
    const bool shadow_forward_inputs_valid = isfinite(shadow_base) &&
        Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS > 0.0f;
    const bool shadow_delta_h_valid = shadow_forward_inputs_valid && shadow_q_in_model_support &&
        post_pulse && side_matched;
    const float shadow_delta_h = shadow_delta_h_valid
        ? shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS * shadow_q_effective
        : NAN;
    event.shadow_delta_h_video_pred_deg = shadow_delta_h;
    event.shadow_delta_h_valid = isfinite(shadow_delta_h);

    // This is the on-device dynamic-H proxy. Video validation remains offline.
    event.shadow_delta_h_imu_actual_deg = isfinite(delta_half_range_dynamic)
        ? delta_half_range_dynamic
        : NAN;
    event.shadow_imu_proxy_residual_valid = event.shadow_delta_h_valid &&
        isfinite(event.shadow_delta_h_imu_actual_deg);
    event.shadow_delta_h_pred_minus_imu_dynamic_deg = event.shadow_imu_proxy_residual_valid
        ? shadow_delta_h - event.shadow_delta_h_imu_actual_deg
        : NAN;

    bool shadow_h_free_imu_in_support = false;
    const float shadow_h_free_imu = shadow_state_valid
        ? halfRangeShadowPrediction(shadow_hprev, true, &shadow_h_free_imu_in_support) : NAN;
    event.shadow_h_free_imu_pred_deg = shadow_h_free_imu;
    event.shadow_h_free_valid = isfinite(shadow_h_free_imu) && shadow_h_free_imu_in_support;
    const float shadow_h_free_video = event.shadow_h_free_valid
        ? Config::ZERO_CROSS_V53_H_FREE_VIDEO_INTERCEPT_DEG +
              Config::ZERO_CROSS_V53_H_FREE_VIDEO_FROM_IMU_GAIN * shadow_h_free_imu
        : NAN;
    event.shadow_h_free_video_pred_deg = shadow_h_free_video;
    event.shadow_h_free_video_valid = isfinite(shadow_h_free_video);

    // Candidate predictions are retained even when the v51 state support is
    // false. They are diagnostics, not commands. Q=0 is explicitly extrapolated.
    const bool shadow_h_post_candidates_valid = shadow_forward_inputs_valid &&
        event.shadow_h_free_video_valid;
    const float shadow_h_post_q0 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[0] : NAN;
    const float shadow_h_post_q05 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[1] : NAN;
    const float shadow_h_post_q10 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[2] : NAN;
    const float shadow_h_post_q15 = shadow_h_post_candidates_valid
        ? shadow_h_free_video + shadow_base + Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS *
              Config::ZERO_CROSS_V53_Q_CANDIDATE_MAS[3] : NAN;
    event.shadow_h_post_pred_q0_deg = shadow_h_post_q0;
    event.shadow_h_post_pred_q05_deg = shadow_h_post_q05;
    event.shadow_h_post_pred_q10_deg = shadow_h_post_q10;
    event.shadow_h_post_pred_q15_deg = shadow_h_post_q15;
    event.shadow_h_post_candidates_valid = shadow_h_post_candidates_valid;
    event.shadow_q0_extrapolated = shadow_h_post_candidates_valid;
    // These bounds are the current discrete control-candidate endpoints, not
    // the wider forward-model measurement support. They are recorded even in
    // this log-only build so the target-reachability policy can be audited.
    event.shadow_h_supported_min_pred_deg = shadow_h_post_q05;
    event.shadow_h_supported_max_pred_deg = shadow_h_post_q15;
    const float shadow_h_post_actual_q = event.shadow_h_free_video_valid &&
        event.shadow_delta_h_valid ? shadow_h_free_video + shadow_delta_h : NAN;
    event.shadow_h_post_pred_deg = shadow_h_post_actual_q;
    event.shadow_h_post_valid = isfinite(shadow_h_post_actual_q);

    event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_REFERENCE_UNCONFIGURED;
    event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_UNAVAILABLE;
    event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_UNAVAILABLE;
    event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_UNAVAILABLE;
    event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_UNAVAILABLE;
    if (Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_h_ref_deg = Config::ZERO_CROSS_V53_H_REF_VIDEO_DEG;
      if (!shadow_forward_inputs_valid) {
        event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_INVALID;
      } else if (!event.shadow_h_free_video_valid) {
        event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_H_FREE_INVALID;
      } else {
        const float delta_h_required = event.shadow_h_ref_deg - shadow_h_free_video;
        const float q_raw = (delta_h_required - shadow_base) /
            Config::ZERO_CROSS_V52_SHADOW_Q_GAIN_DEG_PER_MAS;
        event.shadow_q_req_raw_mA_s = q_raw;
        event.shadow_q_req_clamped_mA_s = isfinite(q_raw)
            ? fminf(Config::ZERO_CROSS_V53_Q_REQUEST_MAX_MAS, fmaxf(0.0f, q_raw))
            : NAN;
        if (!isfinite(q_raw)) {
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_INVALID;
        } else if (q_raw < 0.0f) {
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_BELOW_ZERO;
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_BELOW_ZERO;
        } else if (q_raw > Config::ZERO_CROSS_V53_Q_REQUEST_MAX_MAS) {
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_ABOVE_QMAX;
          event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_ABOVE_QMAX;
        } else {
          // Region remains 0--1.5 mA*s even below the fitted Q minimum. The
          // reason and valid flag separately expose that lower-support limit.
          event.shadow_q_req_region = Config::ZERO_CROSS_V53_Q_REQ_REGION_WITHIN_ZERO_TO_QMAX;
          if (q_raw < Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS) {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_OUTSIDE_MODEL_SUPPORT;
          } else if (shadow_state_in_support) {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_WITHIN_RANGE;
            event.shadow_q_req_valid = true;
            event.shadow_future_control_eligible = true;
          } else {
            event.shadow_q_req_reason = Config::ZERO_CROSS_V53_Q_REQ_STATE_OUT_OF_SUPPORT;
          }
        }

        // v54 reports Q support independently of the state support. This
        // makes the old v53 `outside_model_support` label unambiguous without
        // rewriting its historical code or meaning.
        if (isfinite(q_raw)) {
          if (q_raw < 0.0f) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_NEGATIVE;
          } else if (q_raw < Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_BELOW_FORWARD_MODEL;
          } else if (q_raw <= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_WITHIN_CURRENT_CANDIDATES;
          } else if (q_raw <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS) {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_ABOVE_CANDIDATES_WITHIN_FORWARD_MODEL;
          } else {
            event.shadow_q_support_reason = Config::ZERO_CROSS_V54_Q_SUPPORT_ABOVE_FORWARD_MODEL;
          }
        }
      }
    }

    // v54 target-reachability policy. It is deliberately evaluated after the
    // diagnostics above and remains isolated from every motor-command path.
    if (!Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_REFERENCE_UNCONFIGURED;
    } else if (!shadow_state_valid) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_STATE_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!shadow_state_in_support) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_STATE_OUT_OF_SUPPORT;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!event.shadow_h_free_video_valid) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_H_FREE_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else if (!shadow_h_post_candidates_valid || !isfinite(event.shadow_h_supported_min_pred_deg) ||
               !isfinite(event.shadow_h_supported_max_pred_deg)) {
      event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_CANDIDATES_INVALID;
      event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
    } else {
      event.shadow_reachability_valid = true;
      const float h_ref = event.shadow_h_ref_deg;
      const float h_min = event.shadow_h_supported_min_pred_deg;
      const float h_max = event.shadow_h_supported_max_pred_deg;
      if (h_ref < h_min) {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_BELOW_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_NO_PULSE;
        event.shadow_recommended_q_mA_s = 0.0f;
      } else if (h_ref > h_max) {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_ABOVE_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_SATURATE_QMAX;
        event.shadow_recommended_q_mA_s = Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS;
      } else {
        event.shadow_reachability_reason = Config::ZERO_CROSS_V54_REACHABILITY_TARGET_WITHIN_RANGE;
        event.shadow_recommended_action = Config::ZERO_CROSS_V54_SHADOW_ACTION_INVERSE_Q;
        event.shadow_recommended_q_mA_s = event.shadow_q_req_raw_mA_s;
        event.shadow_href_in_current_control_candidate_reachable_range = true;
      }
    }

    const bool shadow_q_req_finite = isfinite(event.shadow_q_req_raw_mA_s);
    event.shadow_q_req_in_model_support = shadow_q_req_finite &&
        event.shadow_q_req_raw_mA_s >= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MIN_MAS &&
        event.shadow_q_req_raw_mA_s <= Config::ZERO_CROSS_V52_SHADOW_Q_SUPPORT_MAX_MAS;
    event.shadow_q_req_in_control_candidate_range = shadow_q_req_finite &&
        event.shadow_q_req_raw_mA_s >= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MIN_MAS &&
        event.shadow_q_req_raw_mA_s <= Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS;

    event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_UNAVAILABLE;
    if (!Config::ZERO_CROSS_V57_SUPPORT_AWARE_INVERSE_SHADOW_ENABLED) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_UNAVAILABLE;
    } else if (!Config::ZERO_CROSS_V53_H_REF_CONFIGURED) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNCONFIGURED;
    } else if (!shadow_state_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_STATE_INVALID;
    } else if (!shadow_state_in_support) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_STATE_OUT_OF_SUPPORT;
    } else if (!event.shadow_h_free_video_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_H_FREE_INVALID;
    } else if (!shadow_q_req_finite || !event.shadow_reachability_valid) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNREACHABLE;
    } else if (event.shadow_q_req_raw_mA_s < Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MIN_MAS) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_Q_BELOW_CANDIDATE_RANGE;
    } else if (event.shadow_q_req_raw_mA_s > Config::ZERO_CROSS_V54_CONTROL_CANDIDATE_Q_MAX_MAS) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_Q_ABOVE_CANDIDATE_RANGE;
    } else if (event.shadow_reachability_reason != Config::ZERO_CROSS_V54_REACHABILITY_TARGET_WITHIN_RANGE) {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_REFERENCE_UNREACHABLE;
    } else {
      event.shadow_v57_q_req_reason = Config::ZERO_CROSS_V57_Q_REQ_REASON_VALID;
    }
    event.shadow_q_req_candidate_valid =
        event.shadow_v57_q_req_reason == Config::ZERO_CROSS_V57_Q_REQ_REASON_VALID;

    // v58: retain every v57 audit record, and separately label the narrowly
    // targeted state neighborhood for inverse-Q repeatability analysis. This
    // is intentionally evaluated after every command decision and cannot feed
    // any physical control path.
    event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_UNAVAILABLE;
    if (!Config::ZERO_CROSS_V58_REPEATABILITY_STATE_GATE_ENABLED) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_DISABLED;
    } else if (!shadow_state_valid) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_STATE_INVALID;
    } else if (shadow_hprev < Config::ZERO_CROSS_V58_GATE_HPREV_MIN_DEG) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_HPREV_BELOW;
    } else if (shadow_hprev > Config::ZERO_CROSS_V58_GATE_HPREV_MAX_DEG) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_HPREV_ABOVE;
    } else if (shadow_abs_rate < Config::ZERO_CROSS_V58_GATE_ABS_RATE_MIN_DPS) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_RATE_BELOW;
    } else if (shadow_abs_rate > Config::ZERO_CROSS_V58_GATE_ABS_RATE_MAX_DPS) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_RATE_ABOVE;
    } else if (shadow_next_peak_side != Config::ZERO_CROSS_V58_GATE_NEXT_PEAK_SIDE) {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_DIRECTION_MISMATCH;
    } else {
      event.shadow_v58_repeatability_gate_reason = Config::ZERO_CROSS_V58_GATE_REASON_PASSED;
    }
    event.shadow_v58_repeatability_gate_passed =
        event.shadow_v58_repeatability_gate_reason == Config::ZERO_CROSS_V58_GATE_REASON_PASSED;
    event.v59_block_index = calibration_probe_active_plan_index_ == 0 ? 0 :
        static_cast<uint8_t>((calibration_probe_active_plan_index_ - 1) / 3 + 1);
    event.v59_block_order = calibration_probe_active_plan_index_ == 0 ? 0 :
        static_cast<uint8_t>((calibration_probe_active_plan_index_ - 1) % 3 + 1);
    // Reaching Q_CAL is possible only after this online state gate passed.
    event.v59_state_gate_reason = Config::ZERO_CROSS_V59_GATE_REASON_PASSED;
    event.v59_state_gate_passed = Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED;

    event.post_pulse = post_pulse;
    event.in_support = in_support;
    event.side_matched = side_matched;
    event.direction_matches_base = calibration_probe_direction_ == -calibration_probe_requested_side_;
    event.positive_gain = positive_gain;
    event.gain_stored = gain_stored;
    logger_->addCalibrationProbeEvent(event);
  };

  if (calibration_phase_ == CalibrationPhase::INITIAL_EXCITE) {
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 0, -1.0f,
                          calibration_initial_kick_q_effective_pred_mA_s_);
    if (peak_abs >= Config::ZERO_CROSS_CALIBRATION_INITIAL_ABORT_DEG) {
      finishCalibrationShadow(1);
    } else if (peak_abs >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MIN_DEG) {
      setLastPeak();
      calibration_phase_ = CalibrationPhase::FREE_DECAY;
      calibration_result_.initial_kick_count = calibration_initial_kick_count_;
      if (logger_) logger_->setCalibrationResult(calibration_result_);
    } else if (calibration_initial_kick_count_ >= Config::ZERO_CROSS_CALIBRATION_INITIAL_MAX_KICKS) {
      finishCalibrationShadow(1);
    }
    return;
  }

  if (calibrationIsMainControl()) {
    setLastPeak();
    return;
  }
  if (peak_abs < Config::ZERO_CROSS_CALIBRATION_MIN_PEAK_DEG) {
    finishCalibrationShadow(1);
    return;
  }

  if (calibration_phase_ == CalibrationPhase::FREE_DECAY) {
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      // A missing or non-alternating turn must never be bridged by a later one.
      finishCalibrationShadow(6);
      return;
    }
    CalibrationFit& fit = side > 0 ? calibration_fit_pos_ : calibration_fit_neg_;
    if (fit.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE) {
      // The paired arrival side has already used the bounded support-wait
      // allowance. Do not overwrite a retained sample or extrapolate V60.
      finishCalibrationShadow(9);
      return;
    }
    if (fit.count == 0) {
      fit.support_min_deg = fminf(calibration_last_peak_abs_deg_, peak_abs);
      fit.support_max_deg = fmaxf(calibration_last_peak_abs_deg_, peak_abs);
      fit.input_min_deg = calibration_last_peak_abs_deg_;
      fit.input_max_deg = calibration_last_peak_abs_deg_;
    } else {
      fit.support_min_deg = fminf(fit.support_min_deg, fminf(calibration_last_peak_abs_deg_, peak_abs));
      fit.support_max_deg = fmaxf(fit.support_max_deg, fmaxf(calibration_last_peak_abs_deg_, peak_abs));
      fit.input_min_deg = fminf(fit.input_min_deg, calibration_last_peak_abs_deg_);
      fit.input_max_deg = fmaxf(fit.input_max_deg, calibration_last_peak_abs_deg_);
    }
    fit.x_deg[fit.count] = calibration_last_peak_abs_deg_;
    fit.y_deg[fit.count] = peak_abs;
    fit.count++; fit.sum_x += calibration_last_peak_abs_deg_; fit.sum_y += peak_abs;
    fit.sum_x2 += calibration_last_peak_abs_deg_ * calibration_last_peak_abs_deg_;
    fit.sum_y2 += peak_abs * peak_abs;
    fit.sum_xy += calibration_last_peak_abs_deg_ * peak_abs;
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 1,
                          calibration_last_peak_abs_deg_, 0.0f);
    const float current_half_range_dynamic = 0.5f * fabsf(
        peak_signed_dynamic - calibration_last_peak_signed_dynamic_deg_);
    const float current_half_range_fixed = 0.5f * fabsf(
        peak_signed_fixed - calibration_last_peak_signed_fixed_deg_);
    if (calibration_free_last_half_range_valid_) {
      addHalfRangeFreePair(calibration_half_range_dynamic_fit_,
                           calibration_free_last_half_range_dynamic_deg_, current_half_range_dynamic);
      addHalfRangeFreePair(calibration_half_range_fixed_fit_,
                           calibration_free_last_half_range_fixed_deg_, current_half_range_fixed);
    }
    calibration_free_last_half_range_dynamic_deg_ = current_half_range_dynamic;
    calibration_free_last_half_range_fixed_deg_ = current_half_range_fixed;
    calibration_free_last_half_range_valid_ = true;
    setLastPeak();
    if (calibration_fit_pos_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE &&
        calibration_fit_neg_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_PER_SIDE) {
      if (!fitCalibrationFreeDecay()) {
        finishCalibrationShadow(2);
      } else {
        calibration_result_.half_range_free_count = calibration_half_range_dynamic_fit_.count;
        const bool dynamic_half_range_valid = fitHalfRangeShadow(calibration_half_range_dynamic_fit_, true);
        fitHalfRangeShadow(calibration_half_range_fixed_fit_, false);
        if (dynamic_half_range_valid) {
          calibration_result_.v62_hc_target_feasible = v62HasFeasibleHCTarget();
          if (!calibration_result_.v62_hc_target_feasible) {
            finishCalibrationShadow(Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_TARGET);
          } else if (!fitV62RateStateModel()) {
            finishCalibrationShadow(Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_RATE_TARGET);
          } else if (!configureV62RebuildTarget()) {
            finishCalibrationShadow(calibration_result_.v62_hc_target_feasible
                ? Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_RATE_TARGET
                : Config::ZERO_CROSS_CAL_FAILURE_V62_NO_FEASIBLE_HC_TARGET);
          } else {
            calibration_phase_ = CalibrationPhase::Q_REBUILD;
          }
          if (logger_) logger_->setCalibrationResult(calibration_result_);
          return;
        }
        if (!dynamic_half_range_valid) {
          finishCalibrationShadow(2);
        } else if (configureV60RebuildTarget()) {
          calibration_phase_ = CalibrationPhase::Q_REBUILD;
        } else if (!calibration_result_.v60_gate_center_in_dynamic_h_input_support) {
          // The fit is valid but its measured predecessor domain does not yet
          // contain the unchanged H-gate center. Continue unforced free decay
          // and refit at the next eligible peak; never widen the fixed-Q gate.
          calibration_result_.v60_free_decay_support_wait_count++;
          if (calibration_fit_pos_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE &&
              calibration_fit_neg_.count >= Config::ZERO_CROSS_CALIBRATION_FREE_TRANSITIONS_MAX_PER_SIDE) {
            finishCalibrationShadow(9);
          }
        } else {
          // H_gate is supported but the composed loss is not physically usable.
          finishCalibrationShadow(2);
        }
      }
      if (logger_) logger_->setCalibrationResult(calibration_result_);
    }
    return;
  }

  if (calibration_phase_ == CalibrationPhase::WAIT_Q_REBUILD) {
    const bool exact_rebuild = calibration_rebuild_pulse_id_ != 0 &&
                               status_.pulse_id == calibration_rebuild_pulse_id_;
    const bool post_rebuild = exact_rebuild &&
                              candidate_peak_ms > calibration_rebuild_pulse_start_test_ms_;
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      finishCalibrationShadow(6);
      return;
    }
    if (!post_rebuild) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                            calibration_last_peak_abs_deg_, 0.0f);
      setLastPeak();
      return;
    }
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 6,
                          calibration_last_peak_abs_deg_, 0.0f);
    bool v61_state_observed = false;
    if (calibration_v61_rebuild_plan_active_) {
      const int8_t controlled_side = -Config::ZERO_CROSS_V59_GATE_NEXT_PEAK_SIDE;
      float predicted_h = NAN;
      float predicted_c = NAN;
      const bool state_predicted = side == controlled_side &&
          predictV61GateState(peak_abs, &predicted_h, &predicted_c);
      if (state_predicted) {
        calibration_result_.v61_predicted_gate_h_deg = predicted_h;
        calibration_result_.v61_predicted_gate_c_deg = predicted_c;
        v61_state_observed = predicted_h >= Config::ZERO_CROSS_V59_GATE_HPREV_MIN_DEG &&
            predicted_h <= Config::ZERO_CROSS_V59_GATE_HPREV_MAX_DEG &&
            predicted_c >= Config::ZERO_CROSS_V59_GATE_CPREV_MIN_DEG &&
            predicted_c <= Config::ZERO_CROSS_V59_GATE_CPREV_MAX_DEG;
      }
      if (!v61_state_observed) {
        // Peak-error feedback remains unrestricted as requested.  It corrects
        // only the next command on the same controlled side; no positive-side
        // rebuild is inserted between attempts.
        calibration_v61_command_peak_target_deg_ +=
            calibration_v61_nominal_controlled_peak_target_deg_ - peak_abs;
        if (!isfinite(calibration_v61_command_peak_target_deg_) ||
            calibration_v61_command_peak_target_deg_ <= 0.0f) {
          calibration_v61_command_peak_target_deg_ =
              calibration_v61_nominal_controlled_peak_target_deg_;
        }
        calibration_result_.v61_feedback_command_peak_deg =
            calibration_v61_command_peak_target_deg_;
        ++calibration_result_.v61_feedback_correction_count;
      }
    } else {
      // Compatibility fallback: V61 configuration was not available, so retain
      // the old V60 raw-peak criterion rather than using a model outside support.
      v61_state_observed = calibration_result_.v60_rebuild_target_valid &&
          peak_abs >= calibration_result_.v60_rebuild_target_peak_deg;
    }
    if (v61_state_observed) {
      calibration_result_.v60_rebuild_target_observed = true;
      // Only a state-confirmed rebuild is followed by the required unforced
      // pair. The actual rate is still checked by the unchanged V59 gate.
      calibration_v60_cooldown_halfcycles_remaining_ =
          Config::ZERO_CROSS_V60_REBUILD_COOLDOWN_HALF_CYCLES;
    }
    setLastPeak();
    calibration_phase_ = CalibrationPhase::Q_REBUILD;
    if (logger_) logger_->setCalibrationResult(calibration_result_);
    return;
  }

  const bool wait_pos = calibration_phase_ == CalibrationPhase::WAIT_Q_CAL_POS;
  const bool wait_neg = calibration_phase_ == CalibrationPhase::WAIT_Q_CAL_NEG;
  if (!wait_pos && !wait_neg) {
    // A Q probe was deferred at a zero crossing. Keep the true free-decay
    // sequence current while waiting for the requested arrival side.
    if (!isConsecutive() || side == calibration_last_peak_side_) {
      recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                            calibration_last_peak_abs_deg_, 0.0f);
      finishCalibrationShadow(6);
      return;
    }
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                          calibration_last_peak_abs_deg_, 0.0f);
    setLastPeak();
    return;
  }

  const bool exact_probe = calibration_probe_pulse_id_ != 0 &&
                           status_.pulse_id == calibration_probe_pulse_id_;
  const bool post_pulse = exact_probe && candidate_peak_ms > calibration_probe_pulse_start_test_ms_;
  const bool consecutive = isConsecutive() && side != calibration_last_peak_side_;
  if (!consecutive) {
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 4,
                          calibration_last_peak_abs_deg_, 0.0f);
    finishCalibrationShadow(6);
    return;
  }

  if (!post_pulse) {
    // This physical peak predates the pulse, even though confirmation happened
    // later. It becomes the correct pre-pulse state; it cannot identify g.
    recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, 5,
                          calibration_last_peak_abs_deg_, 0.0f);
    addProbeEvent(false, false, side == calibration_probe_requested_side_, false, false, 1, NAN);
    setLastPeak();
    calibration_probe_previous_peak_abs_deg_ = peak_abs;
    calibration_probe_previous_peak_signed_dynamic_deg_ = peak_signed_dynamic;
    calibration_probe_previous_peak_signed_fixed_deg_ = peak_signed_fixed;
    calibration_probe_previous_half_range_dynamic_deg_ = calibration_last_half_range_dynamic_deg_;
    calibration_probe_previous_half_range_fixed_deg_ = calibration_last_half_range_fixed_deg_;
    calibration_probe_previous_half_range_valid_ = calibration_last_half_range_valid_;
    calibration_probe_previous_center_dynamic_deg_ = calibration_last_center_dynamic_deg_;
    calibration_probe_previous_center_dynamic_valid_ = calibration_last_center_dynamic_valid_;
    calibration_probe_previous_peak_candidate_ms_ = candidate_peak_ms;
    return;
  }

  const float support_min = side > 0 ? calibration_result_.free_input_min_pos_deg :
                                       calibration_result_.free_input_min_neg_deg;
  const float support_max = side > 0 ? calibration_result_.free_input_max_pos_deg :
                                       calibration_result_.free_input_max_neg_deg;
  const float margin = Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SUPPORT_MARGIN_DEG;
  const bool in_support = calibration_probe_previous_peak_candidate_ms_ < calibration_probe_pulse_start_test_ms_ &&
                          calibration_probe_previous_peak_abs_deg_ >= support_min + margin &&
                          calibration_probe_previous_peak_abs_deg_ <= support_max - margin;
  const float gain = in_support && calibration_probe_q_effective_pred_mA_s_ > 0.0f
      ? (peak_abs - ( (side > 0 ? calibration_result_.r_pos : calibration_result_.r_neg) *
                      calibration_probe_previous_peak_abs_deg_ +
                      (side > 0 ? calibration_result_.c_pos_deg : calibration_result_.c_neg_deg))) /
            calibration_probe_q_effective_pred_mA_s_
      : NAN;
  const bool positive_gain = isfinite(gain) &&
      gain > Config::ZERO_CROSS_CALIBRATION_Q_PROBE_MIN_POSITIVE_GAIN_DEG_PER_MAS;
  bool gain_stored = false;
  if (positive_gain && (side > 0
      ? calibration_result_.q_count_pos < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE
      : calibration_result_.q_count_neg < Config::ZERO_CROSS_CALIBRATION_Q_PROBE_SAMPLES_PER_SIDE)) {
    if (side > 0) {
      calibration_gain_sum_pos_ += gain;
      calibration_gain_sum_sq_pos_ += gain * gain;
      calibration_result_.q_count_pos++;
      const float n = calibration_result_.q_count_pos;
      calibration_result_.g_pos = calibration_gain_sum_pos_ / n;
      calibration_result_.g_pos_stddev = sqrtf(fmaxf(0.0f, calibration_gain_sum_sq_pos_ / n -
          calibration_result_.g_pos * calibration_result_.g_pos));
      calibration_result_.q_used_pos_mA_s = calibration_probe_q_effective_pred_mA_s_;
    } else {
      calibration_gain_sum_neg_ += gain;
      calibration_gain_sum_sq_neg_ += gain * gain;
      calibration_result_.q_count_neg++;
      const float n = calibration_result_.q_count_neg;
      calibration_result_.g_neg = calibration_gain_sum_neg_ / n;
      calibration_result_.g_neg_stddev = sqrtf(fmaxf(0.0f, calibration_gain_sum_sq_neg_ / n -
          calibration_result_.g_neg * calibration_result_.g_neg));
      calibration_result_.q_used_neg_mA_s = calibration_probe_q_effective_pred_mA_s_;
    }
    gain_stored = true;
  }

  recordCalibrationPeak(candidate_peak_ms, confirmed_ms, side, peak_signed_dynamic, peak_signed_fixed, side > 0 ? 2 : 3,
                        calibration_probe_previous_peak_abs_deg_, calibration_probe_q_effective_pred_mA_s_);
  const bool side_matched = side == calibration_probe_requested_side_;
  const uint8_t result_code = !side_matched ? 2 : !in_support ? 3 :
      !isfinite(gain) ? 4 : !positive_gain ? 5 : gain_stored ? 0 : 6;
  addProbeEvent(true, in_support, side_matched, positive_gain, gain_stored, result_code, gain);
  // A post-pulse peak completes this support-entry episode. A later support
  // loss starts a new bounded episode without hiding its run-wide total.
  calibration_rebuild_attempt_in_episode_ = 0;
  calibration_result_.rebuild_total_count = calibration_rebuild_total_count_;
  calibration_result_.rebuild_episode_id = calibration_rebuild_episode_id_;
  calibration_result_.rebuild_attempt_in_episode = 0;
  calibration_probe_wait_halfcycle_count_ = 0;
  setLastPeak();
  if (calibration_probe_plan_index_ >= Config::ZERO_CROSS_CALIBRATION_Q_PROBE_PLAN_COUNT) {
    // Complete the deterministic protocol even if a signed response is not
    // positive. protocol_complete and calibration_valid are deliberately
    // separate: only the latter can enable a log-only calibrated prediction.
    finishCalibrationShadow(0);
  } else {
    // A probe can legitimately leave the following predecessor below the
    // fitted input domain. Return through Q_REBUILD so the planned next side
    // is either reached after an unforced half-cycle or rebuilt safely.
    calibration_phase_ = CalibrationPhase::Q_REBUILD;
  }
  if (logger_) logger_->setCalibrationResult(calibration_result_);
}
void ExperimentRunner::beginIdentificationEvent(uint32_t now_ms, uint32_t t_test_ms, int8_t direction,
                                                  float target_q_mA_s, float requested_q_mA_s,
                                                  float command_q_mA_s, float target_peak_deg,
                                                  float predicted_peak_deg, float min_predicted_peak_deg,
                                                  float max_predicted_peak_deg, bool target_reachable, bool bootstrap,
                                                  uint16_t width_ms, bool suppressed) {
  const float i0 = predicted_signed_current_end_mA_ * expf(-static_cast<float>(now_ms - predicted_current_end_ms_) / 70.0f);
  const float q_effective_pred_mA_s = fabsf(identificationChargeMaS(i0, direction, width_ms));
  const float rate_dps = status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias();
  const int8_t next_peak_side = rate_dps >= 0.0f ? 1 : -1;
  const float a_pred_calibrated = calibratedPredictionDeg(q_effective_pred_mA_s, next_peak_side);
  PsramLogger::IdentificationEvent e;
  e.start_ms = t_test_ms;
  e.theta0_cdeg = centi(status_.pitch_mekf_deg);
  e.rate0_cdps = centi(status_.gyro_pitch_rate_dps - pitchBiasFromGyroBias());
  e.direction = direction;
  e.vbat_mV = status_.beta_model_vbat_mV;
  e.i0_mA = lroundf(i0);
  e.pulse_width_ms = width_ms;
  e.q_target_mAms = lroundf(target_q_mA_s * 1000.0f);
  e.q_requested_mAms = lroundf(requested_q_mA_s * 1000.0f);
  e.q_command_mAms = lroundf(command_q_mA_s * 1000.0f);
  e.q_estimated_mAms = lroundf(identificationChargeMaS(i0, direction, width_ms) * 1000.0f);
  e.a_target_cdeg = centi(target_peak_deg);
  e.a_pred_cdeg = centi(predicted_peak_deg);
  e.a_pred_shadow_cdeg = centi(predicted_peak_deg + Config::ZERO_CROSS_Q_MODEL_SHADOW_OFFSET_DEG);
  e.a_pred_calibrated_cdeg = isfinite(a_pred_calibrated) ? centi(a_pred_calibrated) : LOG_NAN_I16;
  e.prev_peak_abs_cdeg = calibration_last_peak_valid_ ? centi(calibration_last_peak_abs_deg_) : LOG_NAN_I16;
  e.q_effective_pred_mAms = lroundf(q_effective_pred_mA_s * 1000.0f);
  e.next_peak_side = next_peak_side;
  e.calibrated_prediction_valid = isfinite(a_pred_calibrated);
  e.a_min_cdeg = centi(min_predicted_peak_deg);
  e.a_max_cdeg = centi(max_predicted_peak_deg);
  e.target_reachable = target_reachable;
  e.bootstrap = bootstrap;
  e.pulse_suppressed = suppressed;
  identification_event_id_ = logger_->beginIdentificationEvent(e);
  identification_peak_waiting_ = identification_event_id_ != 0;
  identification_outbound_rate_sign_ = 0;
  identification_reverse_samples_ = 0;
  identification_peak_angle_deg_ = status_.pitch_mekf_deg;
}

void ExperimentRunner::updateIdentificationPeak(uint32_t now_ms, float angle_deg, float rate_dps) {
  if (!identification_peak_waiting_) return;
  if (fabsf(angle_deg) > fabsf(identification_peak_angle_deg_)) identification_peak_angle_deg_ = angle_deg;
  if (identification_outbound_rate_sign_==0) { if (fabsf(rate_dps)>=Config::ZERO_CROSS_MIN_RATE_DPS) identification_outbound_rate_sign_=rate_dps>=0?1:-1; return; }
  if (rate_dps*identification_outbound_rate_sign_<=-Config::ZERO_CROSS_MIN_RATE_DPS) { if (++identification_reverse_samples_>=Config::ZERO_CROSS_IDENTIFICATION_PEAK_CONFIRM_SAMPLES) { logger_->finishIdentificationEvent(identification_event_id_, now_ms-trial_start_ms_, centi(identification_peak_angle_deg_)); identification_peak_waiting_=false; } } else identification_reverse_samples_=0;
}
void ExperimentRunner::updatePulseModelPrediction() {
  uint16_t source_vbat_mV = status_.roller_battery_mV;
  float model_vbat_v = Config::MODEL_VBAT_REFERENCE_V;
  status_.beta_model_vbat_status = 0;
  if (source_vbat_mV < 1000) {
    status_.beta_model_vbat_status = 2;
  } else {
    model_vbat_v = static_cast<float>(source_vbat_mV) / 1000.0f;
    if (model_vbat_v < Config::MODEL_VBAT_MIN_V) {
      model_vbat_v = Config::MODEL_VBAT_MIN_V;
      status_.beta_model_vbat_status = 1;
    } else if (model_vbat_v > Config::MODEL_VBAT_MAX_V) {
      model_vbat_v = Config::MODEL_VBAT_MAX_V;
      status_.beta_model_vbat_status = 1;
    }
  }
  status_.beta_model_vbat_mV = static_cast<uint16_t>(lroundf(model_vbat_v * 1000.0f));
  const float goal_mA = predictCurrentGoalMa(status_.current_mA_setting, model_vbat_v);
  const float tau_rise_s = predictRiseTauS(status_.current_mA_setting);
  const float width_s = static_cast<float>(status_.pulse_width_ms_setting) / 1000.0f;
  const float peak_mA = goal_mA * (1.0f - expf(-width_s / tau_rise_s));
  status_.predicted_i_goal_mA = static_cast<int16_t>(lroundf(goal_mA));
  status_.predicted_peak_current_mA = static_cast<int16_t>(lroundf(peak_mA));
  status_.predicted_beta_min = predictBetaMin(peak_mA);
}

float ExperimentRunner::predictCurrentGoalMa(float command_mA, float model_vbat_v) const {
  const float u_mA = fabsf(command_mA);
  if (u_mA <= 0.0f) return 0.0f;
  return current_model_cache_.goal(u_mA, model_vbat_v, [&]() {
    const float i_sat_mA = Config::MODEL_I_SAT_AT_REFERENCE_MA +
                           Config::MODEL_I_SAT_SLOPE_MA_PER_V *
                               (model_vbat_v - Config::MODEL_VBAT_REFERENCE_V);
    const float ratio = u_mA / i_sat_mA;
    return u_mA / powf(1.0f + powf(ratio, Config::MODEL_I_SAT_EXPONENT),
                        1.0f / Config::MODEL_I_SAT_EXPONENT);
  });
}

float ExperimentRunner::predictRiseTauS(float command_mA) const {
  const float u_mA = fabsf(command_mA);
  return current_model_cache_.riseTime(u_mA, [&]() {
    const float ratio = u_mA / Config::MODEL_TAU_RISE_U_MA;
    const float tau_ms = Config::MODEL_TAU_RISE_MIN_MS +
                         (Config::MODEL_TAU_RISE_MAX_MS - Config::MODEL_TAU_RISE_MIN_MS) /
                             (1.0f + powf(ratio, Config::MODEL_TAU_RISE_EXPONENT));
    return tau_ms / 1000.0f;
  });
}

float ExperimentRunner::predictBetaMin(float peak_current_mA) const {
  float ratio = peak_current_mA / Config::BETA_MIN_REFERENCE_CURRENT_MA;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return Config::BETA_MIN_AT_ZERO_CURRENT +
         (Config::BETA_MIN_AT_REFERENCE_CURRENT - Config::BETA_MIN_AT_ZERO_CURRENT) * ratio;
}

float ExperimentRunner::betaFloorForStrategy(uint8_t index) const {
  return (index == Config::FILTER_FIXED_B100_INDEX || index == Config::FILTER_FIXED_B000_INDEX) ? betaCeilingForStrategy(index) : status_.predicted_beta_min;
}

float ExperimentRunner::betaCeilingForStrategy(uint8_t index) const {
  if (index >= Config::DYNAMIC_BETA_COUNT) return Config::MADGWICK_BETA_NORMAL;
  const float requested = Config::DYNAMIC_BETA_CEILINGS[index];
  // Start every series from the same gravity-converged quaternion. During
  // calibration, settling, ready, and start-sync, use beta=0.100 for all of
  // them. At t_test=0, RUNNING_BATCH_SWEEP selects each configured beta.
  if (status_.state != ExperimentState::RUNNING_BATCH_SWEEP &&
      status_.state != ExperimentState::TRIAL_REST) {
    return Config::MADGWICK_BETA_NORMAL;
  }
  return (index == Config::FILTER_FIXED_B100_INDEX || index == Config::FILTER_FIXED_B000_INDEX) ? requested : fmaxf(requested, status_.predicted_beta_min);
}

uint16_t ExperimentRunner::betaHoldAfterInputMsForStrategy(uint8_t index) const {
  if (index >= Config::DYNAMIC_BETA_COUNT) return Config::BETA_HOLD_AFTER_INPUT_MS;
  return Config::DYNAMIC_BETA_HOLD_AFTER_INPUT_MS[index];
}
void ExperimentRunner::logSampleIfDue() {
  const uint32_t now_us = micros();
  const auto observation = roller_->telemetrySnapshot();
  const bool new_wheel_observation = observation.speed_sequence != last_logged_speed_sequence_ ||
      observation.position_sequence != last_logged_position_sequence_ ||
      observation.applied_command_sequence != last_logged_command_sequence_;
  // Preserve the normal 20 ms time series. While an already-authorized pulse
  // is live, add rows at the current-audit period so fresh-read gaps can be
  // evaluated offline. Logging rate cannot alter the motor command.
  const uint32_t period_us = status_.pulse_active
      ? Config::CURRENT_AUDIT_LOG_PERIOD_US : Config::LOG_PERIOD_MS * 1000UL;
  if (!new_wheel_observation && last_log_us_ != 0 &&
      static_cast<uint32_t>(now_us - last_log_us_) < period_us) return;
  const bool probe_log = timing_probe_pending_ && !timing_probe_log_captured_ &&
      status_.pulse_active && status_.pulse_id == timing_probe_event_.pulse_id;
  const uint32_t log_start_us = probe_log ? micros() : 0;
  logSampleNow();
  if (probe_log) {
    timing_probe_event_.first_audit_log_offset_us =
        static_cast<uint32_t>(log_start_us - timing_probe_event_.pulse_start_us);
    timing_probe_event_.first_audit_log_us = static_cast<uint32_t>(micros() - log_start_us);
    timing_probe_log_captured_ = true;
    maybeFinalizeTimingProbe();
  }
}

void ExperimentRunner::logSampleNow() {
  control_work::Scope work(control_work::Stage::LogRow,
      status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
  if (!logger_ || !logger_->ready()) return;
  // V46t: copy telemetry before taking its reference clock. Never re-read it
  // while constructing this row; Core0 may publish between any two statements.
  const RollerTelemetry roller_telemetry = roller_->telemetrySnapshot();
  const uint32_t now_us = micros();
  if (logger_->full()) {
    requestEmergencyStop("log_buffer_full");
    return;
  }
  LogSample row{};
  uint32_t t_test_ms = status_.measure_elapsed_ms;
  if (status_.sync_event_id == 2) t_test_ms = 0;
  else if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP ||
           status_.state == ExperimentState::TRIAL_REST) t_test_ms = millis() - run_start_ms_;
  {
    control_work::Scope encode_work(control_work::Stage::LogEncode,
        status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
    float beta_ceilings[Config::DYNAMIC_BETA_COUNT];
    for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i)
      beta_ceilings[i] = betaCeilingForStrategy(i);
    encodeLogSample(row, status_, roller_telemetry, now_us, t_test_ms,
        run_start_us_, beta_ceilings);
  }
  {
    control_work::Scope store_work(control_work::Stage::LogStore,
        status_.state == ExperimentState::RUNNING_BATCH_SWEEP, status_.pulse_active);
    logger_->addSample(row);
  }
  last_logged_speed_sequence_ = roller_telemetry.speed_sequence;
  last_logged_position_sequence_ = roller_telemetry.position_sequence;
  last_logged_command_sequence_ = roller_telemetry.applied_command_sequence;
  if (status_.sync_event_id == 2 || status_.sync_event_id == 3 || status_.sync_event_id == 6 ||
      status_.sync_event_id == 7) {
    status_.sync_event_id = 0;
  }
  if (last_log_us_ != 0) status_.log_dt_us = now_us - last_log_us_;
  last_log_us_ = now_us;
}

void ExperimentRunner::finishRun() {
  stopMotor();
  status_.measure_elapsed_ms = measurementTotalDurationMs();
  status_.remaining_ms = 0;
  status_.sync_event_id = 3;
  sync_led_until_ms_ = 0;
  setSyncLed(false);
  logSampleNow();
  beginEndSync(millis());
}

void ExperimentRunner::stopMotor() {
  q_ident_pulse_authorized_ = false;
  energy_control_v0_pulse_authorized_ = false;
  energy_control_autonomous_pulse_authorized_ = false;
  if (roller_) roller_->stop();
  status_.motor_cmd_mA = 0;
  status_.pulse_active = false;
  status_.pulse_direction = 0;
  if (roller_) status_.roller_actual_current_mA = roller_->telemetrySnapshot().actual_current_mA;
}

void ExperimentRunner::setSyncLed(bool on) {
  digitalWrite(Config::SYNC_LED_PIN, on ? HIGH : LOW);
  status_.led_state = on;
}

uint32_t ExperimentRunner::measurementTotalDurationMs() const {
  if (energy_control_autonomous_mode_ && wheelWorkMode()) return Config::WHEEL_WORK_CONTROL_DURATION_MS;
  if (energy_control_autonomous_mode_) return anyProbeBatchMode() ?
      negative_probe_batch::kDurationMs : Config::ENERGY_CONTROL_AUTONOMOUS_DURATION_MS;
  if (energy_control_v0_mode_) return Config::ENERGY_CONTROL_V0_DURATION_MS;
  if (q_ident_mode_) return Config::Q_IDENT_DURATION_MS;
  if (passive_capture_mode_) return Config::PASSIVE_CAPTURE_DURATION_MS;
  if (zero_cross_mode_) {
    return q_run_mode_ == QRunMode::CONTROL && Config::ZERO_CROSS_V59_STATE_WAIT_FIXED_Q_ENABLED
        ? Config::ZERO_CROSS_V59_TEST_DURATION_MS : Config::ZERO_CROSS_TEST_DURATION_MS;
  }
  if (single_trial_mode_ && selected_trial_index_ < Config::BETA_SWEEP_TRIAL_COUNT) {
    return Config::BETA_SWEEP_TRIALS[selected_trial_index_].duration_ms;
  }
  return Config::BETA_SWEEP_TOTAL_DURATION_MS;
}
float ExperimentRunner::accelPitchDeg(const ImuReading& r) const {
  return Config::PITCH_SIGN * atan2f(-r.ax_g, sqrtf(r.ay_g * r.ay_g + r.az_g * r.az_g)) * 57.2957795f;
}

float ExperimentRunner::physicalRollCandidateDeg(const ImuReading& r) const {
  static_assert(Config::PITCH_SIGN == -1.0f || Config::PITCH_SIGN == 1.0f, "Pitch sign");
  return -Config::PITCH_SIGN * r.pitch_accel_only_deg;
}

float ExperimentRunner::pitchBiasFromGyroBias() const {
  return Config::GYRO_PITCH_RATE_SIGN * status_.gyro_bias_y_dps;
}

int16_t ExperimentRunner::centi(float value) const {
  static_assert(LOG_NAN_I16 == log_quantization::kMissing, "RWLOG missing value");
  return log_quantization::scaledI16(value, 100.0f);
}

int16_t ExperimentRunner::milli(float value) const {
  return log_quantization::scaledI16(value, 1000.0f);
}

int16_t ExperimentRunner::betaScaled(float value) const {
  return log_quantization::scaledI16(value, 10000.0f);
}

const char* ExperimentRunner::stateName() const {
  switch (status_.state) {
    case ExperimentState::STARTUP_GYRO_CALIB: return "STARTUP_GYRO_CALIB";
    case ExperimentState::MADGWICK_SETTLING: return "MADGWICK_SETTLING";
    case ExperimentState::READY_TO_MEASURE: return "READY_TO_MEASURE";
    case ExperimentState::RUNNING_BATCH_SWEEP: return "RUNNING_BATCH_SWEEP";
    case ExperimentState::FINISHED: return "FINISHED";
    case ExperimentState::ESTOP: return "ESTOP";
    case ExperimentState::START_SYNC: return "START_SYNC";
    case ExperimentState::END_SYNC: return "END_SYNC";
    case ExperimentState::TRIAL_REST: return "TRIAL_REST";
  }
  return "UNKNOWN";
}

