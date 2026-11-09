#include "log_sample_encoder.h"
#include "log_quantization.h"

// Keep the encoder compact. The 0.47.13 forced-inline O2 build duplicated the
// conversion at each field and regressed on hardware during the start pulse.
// No fast-math, reduced precision, changed rounding, or lower logging rate.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os", "no-fast-math")
#endif
void encodeLogSample(LogSample& row, const ExperimentStatus& status,
    const RollerTelemetry& roller, uint32_t now_us, uint32_t t_test_ms,
    uint32_t run_start_us, const float* beta_ceilings) {
  (void)beta_ceilings; // retained ABI; no unused comparison encoding
  row.time_us = status.sync_event_id == 2 ? 0 : static_cast<uint32_t>(now_us - run_start_us);
  row.t_test_ms = t_test_ms;
  row.state_id = static_cast<uint8_t>(status.state);
  row.pulse_id = status.pulse_id;
  row.pulse_active = status.pulse_active ? 1 : 0;
  row.pulse_direction = status.pulse_direction;
  row.motor_cmd_mA = status.motor_cmd_mA;
  row.pulse_width_ms_setting = status.pulse_width_ms_setting;
  row.gyro_bias_x_cdps = log_quantization::scaledI16(status.gyro_bias_x_dps, 100.0f);
  row.gyro_bias_y_cdps = log_quantization::scaledI16(status.gyro_bias_y_dps, 100.0f);
  row.gyro_bias_z_cdps = log_quantization::scaledI16(status.gyro_bias_z_dps, 100.0f);
  row.ax_mg = log_quantization::scaledI16(status.ax_g, 1000.0f);
  row.ay_mg = log_quantization::scaledI16(status.ay_g, 1000.0f);
  row.az_mg = log_quantization::scaledI16(status.az_g, 1000.0f);
  row.gx_cdps = log_quantization::scaledI16(status.gx_dps, 100.0f);
  row.gy_cdps = log_quantization::scaledI16(status.gy_dps, 100.0f);
  row.gz_cdps = log_quantization::scaledI16(status.gz_dps, 100.0f);
  row.acc_norm_mg = log_quantization::scaledI16(status.acc_norm_g, 1000.0f);
  row.roller_actual_current_mA = roller.actual_current_mA;
  row.roller_battery_mV = roller.battery_mV;
  row.led_state = status.led_state ? 1 : 0;
  row.sync_event_id = status.sync_event_id;
  row.physical_roll_abs_cdeg = log_quantization::scaledI16(status.physical_roll_abs_deg, 100.0f);
  row.roller_current_sample_time_us = roller.current_sample_time_us;
  row.roller_current_sequence = roller.current_sequence;
  row.roller_q_meas_observed_mAms = isfinite(roller.q_meas_observed_mA_s)
      ? static_cast<int32_t>(lroundf(roller.q_meas_observed_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_target_mAms = isfinite(status.current_audit_q_target_mA_s)
      ? static_cast<int32_t>(lroundf(status.current_audit_q_target_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.pulse_q_pred_mAms = isfinite(status.current_audit_q_pred_mA_s)
      ? static_cast<int32_t>(lroundf(status.current_audit_q_pred_mA_s * 1000.0f)) : LOG_NAN_I32;
  row.roller_current_valid = roller.current_valid ? 1 : 0;
  row.roller_q_meas_observed_valid = roller.q_meas_observed_valid ? 1 : 0;
  row.pitch_mekf_abs_cdeg = log_quantization::scaledI16(status.pitch_mekf_abs_deg, 100.0f);
  const auto& attitude = status.mekf_attitude;
  if (attitude.valid) {
    const auto& q = attitude.quaternion;
    const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    row.roll_mekf_abs_cdeg = log_quantization::scaledI16(
        atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159265358979323846f), 100.0f);
  } else {
    row.roll_mekf_abs_cdeg = LOG_NAN_I16;
  }
  row.pitch_mekf_measurement_relative_cdeg = log_quantization::scaledI16(status.pitch_mekf_measurement_relative_deg, 100.0f);
  row.pitch_mekf_detector_relative_cdeg = log_quantization::scaledI16(status.pitch_mekf_detector_relative_deg, 100.0f);
  row.mekf_bias_x_cdps = log_quantization::scaledI16(status.mekf_bias_x_dps, 100.0f);
  row.mekf_bias_y_cdps = log_quantization::scaledI16(status.mekf_bias_y_dps, 100.0f);
  row.mekf_bias_z_cdps = log_quantization::scaledI16(status.mekf_bias_z_dps, 100.0f);
  row.mekf_accel_confidence_x10000 = log_quantization::scaledI16(status.mekf_accel_confidence, 10000.0f);
  row.mekf_accel_residual_cdeg = log_quantization::scaledI16(status.mekf_accel_residual_deg, 100.0f);
  row.mekf_accel_mag_error_mg = log_quantization::scaledI16(status.mekf_accel_mag_error_g, 1000.0f);
  row.imu_update_dt_us = status.imu_update_dt_us;
  row.imu_sample_age_us = status.imu_last_update_us != 0
      ? static_cast<uint32_t>(now_us - status.imu_last_update_us) : 0xFFFFFFFFUL;
  row.mekf_accel_used = status.mekf_accel_used ? 1 : 0;
  row.gyro_heading_cdeg = isfinite(status.steering.yaw_deg)
      ? static_cast<int32_t>(lroundf(status.steering.yaw_deg * 100.0f)) : LOG_NAN_I32;
  row.steering_delta_cdeg = log_quantization::scaledI16(status.steering.delta_deg, 100.0f);
  row.steering_actual_difference_cdeg = log_quantization::scaledI16(status.steering.actual_difference_deg, 100.0f);
  row.steering_desired_difference_cdeg = log_quantization::scaledI16(status.steering.desired_difference_deg, 100.0f);
  row.steering_cycle_yaw_rate_cdps = log_quantization::scaledI16(status.steering.cycle_yaw_rate_dps, 100.0f);
  row.steering_cycles = status.steering.cycles;
  row.gyro_heading_valid = status.steering.gyro_valid;
  row.steering_reason = static_cast<uint8_t>(status.steering.reason);
  row.roller_speed_crpm = roller.speed_valid ? roller.speed_raw_x100 : LOG_NAN_I32;
  row.roller_speed_read_start_us = roller.speed_read_start_us;
  row.roller_speed_sample_time_us = roller.speed_sample_time_us;
  row.roller_speed_read_end_us = roller.speed_read_end_us;
  row.roller_speed_sequence = roller.speed_sequence;
  row.roller_speed_valid = roller.speed_valid ? 1 : 0;
  row.physical_roll_rate_cdps = log_quantization::scaledI16(status.physical_roll_rate_dps, 100.0f);
  row.body_rate_sample_time_us = status.body_rate_sample_time_us;
  row.roller_applied_current_mA = roller.applied_current_mA;
  row.roller_applied_command_sequence = roller.applied_command_sequence;
  row.roller_applied_command_time_us = roller.applied_command_time_us;
  row.roller_position_raw_cdeg = roller.position_raw_cdeg;
  row.roller_position_read_start_us = roller.position_read_start_us;
  row.roller_position_sample_time_us = roller.position_sample_time_us;
  row.roller_position_read_end_us = roller.position_read_end_us;
  row.roller_position_sequence = roller.position_sequence;
  row.roller_position_valid = roller.position_valid ? 1 : 0;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
