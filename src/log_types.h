#pragma once

#include <Arduino.h>
#include <limits.h>

#include "config.h"

static constexpr int16_t LOG_NAN_I16 = INT16_MIN;
static constexpr int32_t LOG_NAN_I32 = INT32_MIN;

enum class ExperimentState : uint8_t {
  STARTUP_GYRO_CALIB = 0,
  MADGWICK_SETTLING = 1,
  READY_TO_MEASURE = 2,
  RUNNING_BATCH_SWEEP = 3,
  FINISHED = 4,
  ESTOP = 5,
  START_SYNC = 6,
  END_SYNC = 7,
  TRIAL_REST = 8
};

#pragma pack(push, 1)
struct RwLogFileHeader {
  char magic[8];
  uint16_t format_version;
  uint16_t header_size;
  uint32_t run_id;
  uint64_t run_start_us;
  uint32_t metadata_json_size;
  uint32_t sample_count;
  uint32_t summary_count;
  uint32_t event_count;
  uint16_t log_sample_size;
  uint16_t summary_row_size;
  uint16_t event_row_size;
  uint16_t log_period_ms;
  uint16_t imu_period_ms;
  uint16_t roller_read_period_ms;
  uint16_t web_update_period_ms;
  uint16_t total_trials;
  uint16_t preset_id;
  uint32_t flags;
  uint32_t samples_offset;
  uint32_t summaries_offset;
  uint32_t events_offset;
  uint32_t crc_offset;
  uint32_t reserved[8];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LogSample {
  // v56: v55 prefix plus timed wheel position observations.
  // 0.47.30 leaves heading/differences unavailable, delta/cycles zero, reason 8.
  uint32_t time_us;
  uint32_t t_test_ms;
  uint8_t state_id;
  uint32_t pulse_id;
  uint8_t pulse_active;
  int8_t pulse_direction;
  int16_t motor_cmd_mA;
  uint16_t pulse_width_ms_setting;
  int16_t gyro_bias_x_cdps;
  int16_t gyro_bias_y_cdps;
  int16_t gyro_bias_z_cdps;
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
  int16_t physical_roll_abs_cdeg;
  uint32_t roller_current_sample_time_us;
  uint32_t roller_current_sequence;
  int32_t roller_q_meas_observed_mAms;
  int32_t pulse_q_target_mAms;
  int32_t pulse_q_pred_mAms;
  uint8_t roller_current_valid;
  uint8_t roller_q_meas_observed_valid;
  int16_t pitch_mekf_abs_cdeg;
  int16_t roll_mekf_abs_cdeg;
  int16_t pitch_mekf_measurement_relative_cdeg;
  int16_t pitch_mekf_detector_relative_cdeg;
  int16_t mekf_bias_x_cdps;
  int16_t mekf_bias_y_cdps;
  int16_t mekf_bias_z_cdps;
  int16_t mekf_accel_confidence_x10000;
  int16_t mekf_accel_residual_cdeg;
  int16_t mekf_accel_mag_error_mg;
  uint32_t imu_update_dt_us;
  uint32_t imu_sample_age_us;
  uint8_t mekf_accel_used;
  int32_t gyro_heading_cdeg;
  int16_t steering_delta_cdeg;
  int16_t steering_actual_difference_cdeg;
  int16_t steering_desired_difference_cdeg;
  int16_t steering_cycle_yaw_rate_cdps;
  uint16_t steering_cycles;
  uint8_t gyro_heading_valid;
  uint8_t steering_reason;
  // v55 appends wheel work observations; the v54 prefix is unchanged.
  int32_t roller_speed_crpm;
  uint32_t roller_speed_read_start_us;
  uint32_t roller_speed_sample_time_us;
  uint32_t roller_speed_read_end_us;
  uint32_t roller_speed_sequence;
  uint8_t roller_speed_valid;
  int16_t physical_roll_rate_cdps;
  uint32_t body_rate_sample_time_us;
  int16_t roller_applied_current_mA;
  uint32_t roller_applied_command_sequence;
  uint32_t roller_applied_command_time_us;
  // v56: append only. Raw INT32_MIN is a valid position; use the validity flag.
  int32_t roller_position_raw_cdeg;
  uint32_t roller_position_read_start_us;
  uint32_t roller_position_sample_time_us;
  uint32_t roller_position_read_end_us;
  uint32_t roller_position_sequence;
  uint8_t roller_position_valid;
};
#pragma pack(pop)

static_assert(sizeof(RwLogFileHeader) == 110, "RwLogFileHeader binary size changed");
static_assert(sizeof(LogSample) == 172, "LogSample binary size changed");
