#pragma once
#include "../../src/log_types.h"
#pragma pack(push,1)
struct LegacyLogSample {
  uint32_t time_us;
  uint32_t t_test_ms;
  uint8_t state_id;
  uint32_t pulse_id;
  uint8_t pulse_active;
  int8_t pulse_direction;
  int16_t motor_cmd_mA;
  int16_t current_mA_setting;
  uint16_t pulse_width_ms_setting;
  uint16_t input_interval_ms;
  uint8_t trial_index;
  uint8_t trial_count;
  uint32_t trial_elapsed_ms;
  uint32_t trial_duration_ms;
  int16_t trial_current_mA;
  uint16_t trial_pulse_width_ms;
  uint16_t trial_input_interval_ms;
  int16_t trial_predicted_beta_min_x10000;
  uint16_t beta_recovery_tau_ms;
  uint16_t beta_model_vbat_mV;
  int16_t predicted_i_goal_mA;
  int16_t predicted_peak_current_mA;
  uint8_t beta_model_vbat_status;
  int16_t beta_ceiling_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t pitch_madgwick_beta1_raw_cdeg;
  int16_t pitch_madgwick_beta1_bias_cdeg;
  int16_t pitch_dynamic_series_cdeg[Config::DYNAMIC_BETA_COUNT];
  int16_t pitch_gyro_raw_cdeg;
  int16_t pitch_gyro_bias_corrected_cdeg;
  int16_t pitch_accel_only_cdeg;
  int16_t gyro_bias_x_cdps;
  int16_t gyro_bias_y_cdps;
  int16_t gyro_bias_z_cdps;
  int16_t gyro_pitch_rate_cdps;
  int16_t beta_target_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t beta_applied_series_x10000[Config::DYNAMIC_BETA_COUNT];
  int16_t ax_mg;
  int16_t ay_mg;
  int16_t az_mg;
  int16_t gx_cdps;
  int16_t gy_cdps;
  int16_t gz_cdps;
  int16_t acc_norm_mg;
  int16_t roller_actual_current_mA;
  uint16_t roller_battery_mV;
  uint8_t led_state;
  uint8_t sync_event_id;
  uint8_t log_active;
  uint8_t beta_phase_state;
  int16_t beta_phase_progress_x10000;
  int16_t beta_phase_peak_angle_cdeg;
  int16_t beta_phase_angle_cdeg;
  int16_t beta_phase_ceiling_x10000;
  // RWLOG v42: calibrated physical-roll UI state. These fields are appended
  // so v41 records remain byte-for-byte interpretable by their original layout.
  int16_t physical_roll_abs_cdeg;
  int16_t current_roll_cdeg;
  int16_t physical_roll_rate_cdps;
  int16_t target_roll_cdeg;
  int16_t target_error_cdeg;
  uint8_t static_confirmed;
  uint8_t ready;
  // RWLOG v45: actual-current freshness and observed-Q diagnostics. Values are
  // appended so v44 and older sample layouts remain unchanged.
  uint32_t roller_current_sample_time_us;
  uint32_t roller_current_sequence;
  uint32_t roller_current_age_us;
  uint32_t roller_current_read_failure_count;
  int32_t roller_q_meas_observed_mAms;
  int32_t pulse_q_target_mAms;
  int32_t pulse_q_pred_mAms;
  uint16_t roller_current_sample_count;
  uint8_t roller_current_valid;
  uint8_t roller_q_meas_observed_valid;
  // RWLOG v46: adopted MEKF plus online dynamic-beta Madgwick comparison.
  // These append-only fields preserve v45 layout and LED/current audit semantics.
  int16_t pitch_mekf_control_cdeg;
  int16_t pitch_mekf_abs_cdeg;
  int16_t pitch_madgwick_dynamic_abs_cdeg;
  int16_t mekf_q_w_x10000;
  int16_t mekf_q_x_x10000;
  int16_t mekf_q_y_x10000;
  int16_t mekf_q_z_x10000;
  int16_t mekf_bias_x_cdps;
  int16_t mekf_bias_y_cdps;
  int16_t mekf_bias_z_cdps;
  int16_t mekf_accel_confidence_x10000;
  int16_t mekf_accel_residual_cdeg;
  int16_t mekf_accel_mag_error_mg;
  uint32_t imu_update_dt_us;
  uint32_t imu_sample_age_us;
  uint8_t mekf_accel_used;
  uint8_t attitude_filter_adopted;  // 1 = MEKF
  // RWLOG v47: comparison-only event-relative MEKF angle references.
  int16_t pitch_mekf_start_sync_relative_cdeg;
  int16_t pitch_mekf_measurement_relative_cdeg;
  int16_t pitch_mekf_trial_relative_cdeg;
  int16_t mekf_start_sync_zero_abs_cdeg;
  int16_t mekf_measurement_zero_abs_cdeg;
  int16_t mekf_trial_zero_abs_cdeg;
  uint32_t mekf_start_sync_zero_sample_us;
  uint32_t mekf_measurement_zero_sample_us;
  uint32_t mekf_trial_zero_sample_us;
  // RWLOG v47 end
  // RWLOG v48: explicit predicted-MEKF control-zero coordinate.
  int16_t pitch_mekf_detector_relative_cdeg;
  int16_t mekf_detector_zero_predicted_abs_cdeg;
  uint32_t mekf_detector_zero_sample_us;
  // RWLOG v48 end
  // RWLOG v52: BMM150 observation and magnetic field only.
  uint32_t mag_sample_us, mag_sequence;
  uint8_t mag_aux[8];
};
#pragma pack(pop)
