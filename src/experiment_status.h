#pragma once
#include "log_types.h"
#include "mekf_attitude_diagnostics.h"
#include "gyro_steering.h"

struct ExperimentStatus {
  ExperimentState state = ExperimentState::STARTUP_GYRO_CALIB;
  uint16_t run_id = 0;
  bool running = false;
  bool emergency_stop = false;
  uint32_t boot_elapsed_ms = 0;
  uint32_t measure_elapsed_ms = 0;
  uint32_t remaining_ms = Config::BETA_SWEEP_TOTAL_DURATION_MS;
  uint8_t trial_index = 0;
  uint8_t trial_count = Config::BETA_SWEEP_TRIAL_COUNT;
  uint32_t trial_elapsed_ms = 0;
  uint32_t trial_duration_ms = Config::BETA_SWEEP_TRIAL_DURATION_MS;
  uint32_t calibration_sample_count = 0;
  uint32_t pulse_id = 0;
  uint32_t body_rate_sample_time_us = 0;
  bool pulse_active = false;
  int8_t pulse_direction = 0;
  int16_t motor_cmd_mA = 0;
  bool led_state = false;
  uint8_t sync_event_id = 0;
  int16_t current_mA_setting = Config::DEFAULT_INPUT_CURRENT_MA;
  uint16_t pulse_width_ms_setting = Config::DEFAULT_PULSE_WIDTH_MS;
  uint16_t input_interval_ms = Config::DEFAULT_INPUT_INTERVAL_MS;
  float predicted_beta_min = Config::BETA_MIN_AT_ZERO_CURRENT;
  uint16_t beta_hold_after_input_ms_setting = Config::BETA_HOLD_AFTER_INPUT_MS;
  float beta_recovery_tau_s_setting = Config::BETA_RECOVERY_TAU_S;
  uint16_t beta_model_vbat_mV = 0;
  int16_t predicted_i_goal_mA = 0;
  int16_t predicted_peak_current_mA = 0;
  uint8_t beta_model_vbat_status = 2;
  int16_t roller_actual_current_mA = 0;
  uint16_t roller_battery_mV = 0;
  // v45 current telemetry labels only; no control path reads these values.
  float current_audit_q_target_mA_s = NAN;
  float current_audit_q_pred_mA_s = NAN;
  float gyro_bias_x_dps = 0.0f;
  float gyro_bias_y_dps = 0.0f;
  float gyro_bias_z_dps = 0.0f;
  float gyro_bias_pitch_dps = 0.0f;
  // V46 adopted attitude and online comparison diagnostics.
  MekfAttitudeSnapshot mekf_attitude;
  steering::Snapshot steering = steering::disabledSnapshot();
  float pitch_mekf_deg = 0.0f;              // run-relative control/detector angle when applicable
  float pitch_mekf_abs_deg = 0.0f;          // posterior physical/video body-frame pitch
  float pitch_mekf_predicted_abs_deg = 0.0f; // one-step-ahead control-time pitch
  // V46z comparison-only zero references. MEKF state is never reset by these.
  float pitch_mekf_start_sync_relative_deg = NAN;
  float pitch_mekf_measurement_relative_deg = NAN;
  float pitch_mekf_trial_relative_deg = NAN;
  float mekf_start_sync_zero_abs_deg = NAN;
  float mekf_measurement_zero_abs_deg = NAN;
  float mekf_trial_zero_abs_deg = NAN;
  // Autonomous zero-cross timing: measurement-relative posterior MEKF plus
  // the run's delay projection. Peak amplitude uses the unprojected posterior.
  float pitch_mekf_detector_relative_deg = NAN;
  // Retained V46aa prediction-reference diagnostics only; not used by V46ab control.
  float mekf_detector_zero_predicted_abs_deg = NAN;
  uint32_t mekf_detector_zero_sample_us = 0;
  uint32_t mekf_start_sync_zero_sample_us = 0;
  uint32_t mekf_measurement_zero_sample_us = 0;
  uint32_t mekf_trial_zero_sample_us = 0;
  uint32_t mekf_prediction_horizon_us = 0;
  float pitch_madgwick_dynamic_abs_deg = 0.0f;
  float mekf_q_w = 1.0f;
  float mekf_q_x = 0.0f;
  float mekf_q_y = 0.0f;
  float mekf_q_z = 0.0f;
  float mekf_bias_x_dps = 0.0f;
  float mekf_bias_y_dps = 0.0f;
  float mekf_bias_z_dps = 0.0f;
  float mekf_accel_confidence = 0.0f;
  float mekf_accel_residual_deg = 0.0f;
  float mekf_accel_mag_error_g = 0.0f;
  bool mekf_accel_used = false;
  uint32_t imu_last_update_us = 0;
  uint32_t imu_update_dt_us = 0;
  float pitch_madgwick_beta1_raw_deg = 0.0f;
  float pitch_madgwick_dynamic_raw_deg = 0.0f;
  float pitch_madgwick_beta1_bias_deg = 0.0f;
  float pitch_madgwick_dynamic_bias_deg = 0.0f;
  float pitch_dynamic_beta_deg[Config::DYNAMIC_BETA_COUNT] = {};
  float pitch_gyro_raw_deg = 0.0f;
  float pitch_gyro_bias_corrected_deg = 0.0f;
  float pitch_accel_only_deg = 0.0f;
  float gyro_pitch_rate_dps = 0.0f;
  float beta_target = Config::MADGWICK_BETA_NORMAL;
  float beta_smooth = Config::MADGWICK_BETA_NORMAL;
  uint8_t beta_phase_state = 0;
  float beta_phase_progress = 0.0f;
  float beta_phase_peak_angle_deg = 0.0f;
  float beta_phase_angle_deg = 0.0f;
  float beta_phase_ceiling = Config::MADGWICK_BETA_DYNAMIC_MAX;
  float beta_target_series[Config::DYNAMIC_BETA_COUNT] = {};
  float beta_smooth_series[Config::DYNAMIC_BETA_COUNT] = {};
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
  float acc_norm_g = 0.0f;
  // Static calibrated physical-roll display. These do not replace the
  // fixed-horizon/video coordinate used as the Model B teacher value.
  float physical_roll_candidate_deg = 0.0f;
  float physical_roll_abs_deg = 0.0f;
  float current_roll_deg = 0.0f;
  float physical_roll_rate_raw_dps = 0.0f;
  float physical_roll_rate_dps = 0.0f;
  float display_zero_offset_deg = 0.0f;
  float target_roll_deg = 0.0f;
  float target_error_deg = 0.0f;
  bool static_confirmed = false;
  bool ready = false;
  uint32_t loop_dt_us = 0;
  uint32_t log_dt_us = 0;
  const char* last_error = "";
};
