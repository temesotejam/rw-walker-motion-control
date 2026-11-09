// Frozen log row assignments from 0.47.12 commit 4b08cb13171e13aabf4edaa6f7624d7186bbdea6.
// Adapted only to explicit inputs/clock/ceiling lookup and return-by-value.
#include "../../src/log_sample_encoder.h"
#include <cmath>
#include "log_sample_legacy.h"
namespace {
// Independent pre-optimization wire conversion; ties away from zero.
int16_t scaled(float v, float scale) {
  if (!std::isfinite(v)) return LOG_NAN_I16;
  v *= scale;
  if (v > 32767.0f) return 32767;
  if (v < -32767.0f) return -32767;
  return static_cast<int16_t>(std::lround(v));
}
int16_t centi(float v) { return scaled(v, 100.0f); }
int16_t milli(float v) { return scaled(v, 1000.0f); }
int16_t betaScaled(float v) { return scaled(v, 10000.0f); }
}
LegacyLogSample previousLogSample04712(const ExperimentStatus& status_,
    const RollerTelemetry& roller_telemetry, uint32_t now_us, uint32_t now_ms,
    uint32_t run_start_us_, uint32_t run_start_ms_, const float* ceilings) {
  const auto betaCeilingForStrategy = [&](uint8_t i) { return ceilings[i]; };
  LegacyLogSample row{};
  row.time_us = static_cast<uint32_t>(now_us - run_start_us_);
  if (status_.sync_event_id == 2) {
    row.time_us = 0;
    row.t_test_ms = 0;
  } else if (status_.state == ExperimentState::RUNNING_BATCH_SWEEP || status_.state == ExperimentState::TRIAL_REST) {
    row.t_test_ms = now_ms - run_start_ms_;
  } else {
    row.t_test_ms = status_.measure_elapsed_ms;
  }
  row.state_id = static_cast<uint8_t>(status_.state);
  row.pulse_id = status_.pulse_id;
  row.pulse_active = status_.pulse_active ? 1 : 0;
  row.pulse_direction = status_.pulse_direction;
  row.motor_cmd_mA = status_.motor_cmd_mA;
  row.current_mA_setting = status_.current_mA_setting;
  row.pulse_width_ms_setting = status_.pulse_width_ms_setting;
  row.input_interval_ms = status_.input_interval_ms;
  row.trial_index = status_.trial_index;
  row.trial_count = status_.trial_count;
  row.trial_elapsed_ms = status_.trial_elapsed_ms;
  row.trial_duration_ms = status_.trial_duration_ms;
  row.trial_current_mA = status_.current_mA_setting;
  row.trial_pulse_width_ms = status_.pulse_width_ms_setting;
  row.trial_input_interval_ms = status_.input_interval_ms;
  row.trial_predicted_beta_min_x10000 = betaScaled(status_.predicted_beta_min);
  row.beta_recovery_tau_ms = static_cast<uint16_t>(lroundf(status_.beta_recovery_tau_s_setting * 1000.0f));
  row.beta_model_vbat_mV = status_.beta_model_vbat_mV;
  row.predicted_i_goal_mA = status_.predicted_i_goal_mA;
  row.predicted_peak_current_mA = status_.predicted_peak_current_mA;
  row.beta_model_vbat_status = status_.beta_model_vbat_status;
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    row.beta_ceiling_series_x10000[i] = betaScaled(betaCeilingForStrategy(i));
    row.pitch_dynamic_series_cdeg[i] = centi(status_.pitch_dynamic_beta_deg[i]);
  }
  row.pitch_madgwick_beta1_raw_cdeg = centi(status_.pitch_madgwick_beta1_raw_deg);
  row.pitch_madgwick_beta1_bias_cdeg = centi(status_.pitch_madgwick_beta1_bias_deg);
  row.pitch_gyro_raw_cdeg = centi(status_.pitch_gyro_raw_deg);
  row.pitch_gyro_bias_corrected_cdeg = centi(status_.pitch_gyro_bias_corrected_deg);
  row.pitch_accel_only_cdeg = centi(status_.pitch_accel_only_deg);
  row.gyro_bias_x_cdps = centi(status_.gyro_bias_x_dps);
  row.gyro_bias_y_cdps = centi(status_.gyro_bias_y_dps);
  row.gyro_bias_z_cdps = centi(status_.gyro_bias_z_dps);
  row.gyro_pitch_rate_cdps = centi(status_.gyro_pitch_rate_dps);
  for (uint8_t i = 0; i < Config::DYNAMIC_BETA_COUNT; ++i) {
    row.beta_target_series_x10000[i] = betaScaled(status_.beta_target_series[i]);
    row.beta_applied_series_x10000[i] = betaScaled(status_.beta_smooth_series[i]);
  }
  row.ax_mg = milli(status_.ax_g);
  row.ay_mg = milli(status_.ay_g);
  row.az_mg = milli(status_.az_g);
  row.gx_cdps = centi(status_.gx_dps);
  row.gy_cdps = centi(status_.gy_dps);
  row.gz_cdps = centi(status_.gz_dps);
  row.acc_norm_mg = milli(status_.acc_norm_g);
  row.roller_actual_current_mA = roller_telemetry.actual_current_mA;
  row.roller_battery_mV = roller_telemetry.battery_mV;
  row.roller_current_sample_time_us = roller_telemetry.current_sample_time_us;
  row.roller_current_sequence = roller_telemetry.current_sequence;
  row.roller_current_age_us = roller_telemetry.current_sample_time_us == 0
      ? UINT32_MAX : static_cast<uint32_t>(now_us - roller_telemetry.current_sample_time_us);
  row.roller_current_read_failure_count = roller_telemetry.current_read_failure_count;
  row.roller_q_meas_observed_mAms = isfinite(roller_telemetry.q_meas_observed_mA_s)
      ? static_cast<int32_t>(lroundf(roller_telemetry.q_meas_observed_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_target_mAms = isfinite(status_.current_audit_q_target_mA_s)
      ? static_cast<int32_t>(lroundf(status_.current_audit_q_target_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_pred_mAms = isfinite(status_.current_audit_q_pred_mA_s)
      ? static_cast<int32_t>(lroundf(status_.current_audit_q_pred_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.roller_current_sample_count = roller_telemetry.current_audit_sample_count;
  row.roller_current_valid = roller_telemetry.current_valid ? 1 : 0;
  row.roller_q_meas_observed_valid = roller_telemetry.q_meas_observed_valid ? 1 : 0;
  row.pitch_mekf_control_cdeg = centi(status_.pitch_mekf_deg);
  row.pitch_mekf_abs_cdeg = centi(status_.pitch_mekf_abs_deg);
  row.pitch_madgwick_dynamic_abs_cdeg = centi(status_.pitch_madgwick_dynamic_abs_deg);
  row.mekf_q_w_x10000 = betaScaled(status_.mekf_q_w);
  row.mekf_q_x_x10000 = betaScaled(status_.mekf_q_x);
  row.mekf_q_y_x10000 = betaScaled(status_.mekf_q_y);
  row.mekf_q_z_x10000 = betaScaled(status_.mekf_q_z);
  row.mekf_bias_x_cdps = centi(status_.mekf_bias_x_dps);
  row.mekf_bias_y_cdps = centi(status_.mekf_bias_y_dps);
  row.mekf_bias_z_cdps = centi(status_.mekf_bias_z_dps);
  row.mekf_accel_confidence_x10000 = betaScaled(status_.mekf_accel_confidence);
  row.mekf_accel_residual_cdeg = centi(status_.mekf_accel_residual_deg);
  row.mekf_accel_mag_error_mg = milli(status_.mekf_accel_mag_error_g);
  row.imu_update_dt_us = status_.imu_update_dt_us;
  row.imu_sample_age_us = status_.imu_last_update_us != 0
      ? static_cast<uint32_t>(now_us - status_.imu_last_update_us) : 0xFFFFFFFFUL;
  row.mekf_accel_used = status_.mekf_accel_used ? 1 : 0;
  row.attitude_filter_adopted = 1;
  // V46z comparison-zero begin
  row.pitch_mekf_start_sync_relative_cdeg = centi(status_.pitch_mekf_start_sync_relative_deg);
  row.pitch_mekf_measurement_relative_cdeg = centi(status_.pitch_mekf_measurement_relative_deg);
  row.pitch_mekf_trial_relative_cdeg = centi(status_.pitch_mekf_trial_relative_deg);
  row.mekf_start_sync_zero_abs_cdeg = centi(status_.mekf_start_sync_zero_abs_deg);
  row.mekf_measurement_zero_abs_cdeg = centi(status_.mekf_measurement_zero_abs_deg);
  row.mekf_trial_zero_abs_cdeg = centi(status_.mekf_trial_zero_abs_deg);
  row.mekf_start_sync_zero_sample_us = status_.mekf_start_sync_zero_sample_us;
  row.mekf_measurement_zero_sample_us = status_.mekf_measurement_zero_sample_us;
  row.mekf_trial_zero_sample_us = status_.mekf_trial_zero_sample_us;
  // V46z comparison-zero end
  // V46aa control-zero log begin
  row.pitch_mekf_detector_relative_cdeg = centi(status_.pitch_mekf_detector_relative_deg);
  row.mekf_detector_zero_predicted_abs_cdeg =
      centi(status_.mekf_detector_zero_predicted_abs_deg);
  row.mekf_detector_zero_sample_us = status_.mekf_detector_zero_sample_us;
  // V46aa control-zero log end
  row.led_state = status_.led_state ? 1 : 0;
  row.sync_event_id = status_.sync_event_id;
  row.log_active =
      (status_.state == ExperimentState::START_SYNC || status_.state == ExperimentState::RUNNING_BATCH_SWEEP ||
       status_.state == ExperimentState::TRIAL_REST || status_.state == ExperimentState::END_SYNC) ? 1 : 0;
  row.beta_phase_state = status_.beta_phase_state;
  row.beta_phase_progress_x10000 = betaScaled(status_.beta_phase_progress);
  row.beta_phase_peak_angle_cdeg = centi(status_.beta_phase_peak_angle_deg);
  row.beta_phase_angle_cdeg = centi(status_.beta_phase_angle_deg);
  row.beta_phase_ceiling_x10000 = betaScaled(status_.beta_phase_ceiling);
  row.physical_roll_abs_cdeg = centi(status_.physical_roll_abs_deg);
  row.current_roll_cdeg = centi(status_.current_roll_deg);
  row.physical_roll_rate_cdps = centi(status_.physical_roll_rate_dps);
  row.target_roll_cdeg = centi(status_.target_roll_deg);
  row.target_error_cdeg = centi(status_.target_error_deg);
  row.static_confirmed = status_.static_confirmed ? 1 : 0;
  row.ready = status_.ready ? 1 : 0;
  return row;
}
