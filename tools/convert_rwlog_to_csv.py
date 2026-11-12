#!/usr/bin/env python3
import argparse
import csv
import json
import math
import struct
import sys
import zlib
from pathlib import Path


# Standalone converter: keep the compact wire schema in this file.
FIELDS_V53 = [('I', 'log_time_s', 1000000), ('I', 't_test_ms', 1), ('B', 'state_id', 1), ('I', 'pulse_id', 1), ('B', 'pulse_active', 1), ('b', 'pulse_direction', 1), ('h', 'motor_cmd_mA', 1), ('H', 'pulse_width_ms_setting', 1), ('h', 'gyro_bias_x_dps', 100), ('h', 'gyro_bias_y_dps', 100), ('h', 'gyro_bias_z_dps', 100), ('h', 'ax_g', 1000), ('h', 'ay_g', 1000), ('h', 'az_g', 1000), ('h', 'gx_dps', 100), ('h', 'gy_dps', 100), ('h', 'gz_dps', 100), ('h', 'acc_norm_g', 1000), ('h', 'roller_actual_current_mA', 1), ('H', 'roller_battery_mV', 1), ('B', 'led_state', 1), ('B', 'sync_event_id', 1), ('h', 'physical_roll_abs_deg', 100), ('I', 'roller_current_sample_time_us', 1), ('I', 'roller_current_sequence', 1), ('i', 'roller_q_meas_observed_mA_s', 1000), ('i', 'pulse_q_target_mA_s', 1000), ('i', 'pulse_q_pred_mA_s', 1000), ('B', 'roller_current_valid', 1), ('B', 'roller_q_meas_observed_valid', 1), ('h', 'pitch_mekf_abs_deg', 100), ('h', 'pitch_mekf_measurement_relative_deg', 100), ('h', 'pitch_mekf_detector_relative_deg', 100), ('h', 'mekf_bias_x_dps', 100), ('h', 'mekf_bias_y_dps', 100), ('h', 'mekf_bias_z_dps', 100), ('h', 'mekf_accel_confidence', 10000), ('h', 'mekf_accel_residual_deg', 100), ('h', 'mekf_accel_mag_error_g', 1000), ('I', 'imu_update_dt_us', 1), ('I', 'imu_sample_age_us', 1), ('B', 'mekf_accel_used', 1), ('i', 'gyro_heading_deg', 100), ('h', 'steering_delta_deg', 100), ('h', 'steering_actual_difference_deg', 100), ('h', 'steering_desired_difference_deg', 100), ('h', 'steering_cycle_yaw_rate_dps', 100), ('H', 'steering_cycles', 1), ('B', 'gyro_heading_valid', 1), ('B', 'steering_reason', 1)]
SAMPLE_FORMAT_V53 = '<' + ''.join(f[0] for f in FIELDS_V53)
CSV_COLUMNS_V53 = ['time_s'] + [f[1] for f in FIELDS_V53]
FIELDS_V54 = FIELDS_V53.copy()
FIELDS_V54.insert(next(i for i, field in enumerate(FIELDS_V54) if field[1] == 'pitch_mekf_abs_deg') + 1,
                  ('h', 'roll_mekf_abs_deg', 100))
SAMPLE_FORMAT_V54 = '<' + ''.join(field[0] for field in FIELDS_V54)
CSV_COLUMNS_V54 = ['time_s'] + [field[1] for field in FIELDS_V54]
FIELDS_V55 = FIELDS_V54 + [
    ('i', 'roller_speed_rpm', 100),
    ('I', 'roller_speed_read_start_us', 1),
    ('I', 'roller_speed_sample_time_us', 1),
    ('I', 'roller_speed_read_end_us', 1),
    ('I', 'roller_speed_sequence', 1),
    ('B', 'roller_speed_valid', 1),
    ('h', 'physical_roll_rate_dps', 100),
    ('I', 'body_rate_sample_time_us', 1),
    ('h', 'roller_applied_current_mA', 1),
    ('I', 'roller_applied_command_sequence', 1),
    ('I', 'roller_applied_command_time_us', 1),
]
SAMPLE_FORMAT_V55 = '<' + ''.join(field[0] for field in FIELDS_V55)
CSV_COLUMNS_V55 = ['time_s'] + [field[1] for field in FIELDS_V55]
FIELDS_V56 = FIELDS_V55 + [
    ('i', 'roller_position_raw_cdeg', 1),
    ('I', 'roller_position_read_start_us', 1),
    ('I', 'roller_position_sample_time_us', 1),
    ('I', 'roller_position_read_end_us', 1),
    ('I', 'roller_position_sequence', 1),
    ('B', 'roller_position_valid', 1),
]
SAMPLE_FORMAT_V56 = '<' + ''.join(field[0] for field in FIELDS_V56)
CSV_COLUMNS_V56 = ['time_s'] + [field[1] for field in FIELDS_V56] + ['roller_position_deg']


def expand_tables(value, version=None):
    """Expand v53 lossless column tables for existing analysis/CSV consumers."""
    if isinstance(value, dict):
        version = value.get('firmware_revision', version)
        if set(value) == {'fields', 'rows'}:
            fields, rows = value['fields'], value['rows']
            if len(set(fields)) != len(fields) or any(len(row) != len(fields) for row in rows):
                raise ValueError('Invalid compact table')
            return [expand_tables(dict(zip(fields, row)), version) for row in rows]
        result = {key: expand_tables(item, version) for key, item in value.items()}
        if 'foot_control_observations' in result:
            obs, frames = result['foot_control_observations'], result.get('foot_frames')
            if not isinstance(obs, dict) or obs.get('version') != 1 or not isinstance(frames, list) or not isinstance(obs.get('rows'), list):
                raise ValueError('Invalid foot control observation table')
            seen = set()
            for row in obs['rows']:
                if not isinstance(row, list) or len(row) != 2:
                    raise ValueError('Invalid foot control observation row')
                index, delta = row
                if type(index) is not int or type(delta) is not int or index in seen or not 0 <= index < len(frames) or delta < 0:
                    raise ValueError('Invalid foot control observation offset')
                delivered = frames[index].get('delivered_us')
                if type(delivered) is not int or delivered < 0:
                    raise ValueError('Invalid foot delivery timestamp')
                frames[index]['control_observed_us'] = delivered + delta
                seen.add(index)
        if 'foot_span_flags' in result:
            mask = result['foot_span_flags']
            bits = 15 if version in ('1.0.95-positive-sway-probe', '1.0.96-tapered-amplitude-recovery', '1.0.97-paired-sway-rollback', '1.0.98-foot-observation-timeout', '1.0.99-alternate-foot-trial') else 14
            if type(mask) is not int or mask < 0 or mask >= (1 << bits):
                raise ValueError('Invalid foot span flag mask')
            names = ('valid','selected','startup','left_low','right_low','left_both',
                     'right_both','continuous','left_outlier','right_outlier',
                     'left_edge_unverified','right_edge_unverified')
            for bit, name in enumerate(names):
                result['foot_span_' + name] = bool(mask & (1 << bit))
            # New records omit legacy phase; provide nullable keys for older
            # consumers without implying the measurements were recorded.
            for name in ('left_deg','right_deg','start_ms','end_ms','baseline_ms','requested_ms','state','reason'):
                result.setdefault('foot_phase_' + name, None)
            result.setdefault('foot_phase_valid', False)
            result.setdefault('foot_phase_selected', False)
        return result
    if isinstance(value, list):
        return [expand_tables(item, version) for item in value]
    return value

HEADER_FORMAT = "<8sHHIQIIIIHHHHHHHHHIIIII8I"
SAMPLE_FORMAT_V23_V24 = "<IIBIBbhhHHBBIIhHHhhH" + "h" * 21 + "BBB"
SAMPLE_FORMAT_V25 = "<IIBIBbhhHHBBIIhHHhH" + "h" * 33 + "BBB"
SAMPLE_FORMAT_V29 = "<IIBIBbhhHHBBIIhHHhH" + "h" * 33 + "HBBB"
SAMPLE_FORMAT_V30 = "<IIBIBbhhHHBBIIhHHhH" + "HhhB" + "h" * 33 + "HBBB"
SAMPLE_FORMAT_V31 = SAMPLE_FORMAT_V30 + "Bhhhh"
SAMPLE_FORMAT_V32 = "<IIBIBbhhHHBBIIhHHhH" + "HhhB" + "h" * 41 + "HBBB" + "Bhhhh"
# v33 has the v31 binary size but different four-series semantics.
SAMPLE_FORMAT_V33 = SAMPLE_FORMAT_V31
SAMPLE_FORMAT_V34 = "<IIBIBbhhHHBBIIhHHhH" + "HhhB" + "h" * 37 + "HBBB" + "Bhhhh"
# v35 adds the sixth, turn-confirmed comparison series (four int16 arrays).
SAMPLE_FORMAT_V35 = "<IIBIBbhhHHBBIIhHHhH" + "HhhB" + "h" * 41 + "HBBB" + "Bhhhh"
SAMPLE_FORMAT_V42 = SAMPLE_FORMAT_V35 + "h" * 5 + "BB"
# v45 appends actual-current freshness and observed-Q audit values to v42/v44.
SAMPLE_FORMAT_V45 = SAMPLE_FORMAT_V42 + "IIIIiiiHBB"
# v46 appends MEKF/control diagnostics while preserving the complete v45 prefix.
SAMPLE_FORMAT_V46 = SAMPLE_FORMAT_V45 + "h" * 13 + "II" + "BB"
# v47 appends comparison-only posterior-MEKF zero-reference diagnostics.
SAMPLE_FORMAT_V47 = SAMPLE_FORMAT_V46 + "h" * 6 + "III"
# v48 appends explicit detector-relative control diagnostics.
SAMPLE_FORMAT_V48 = SAMPLE_FORMAT_V47 + "hhI"
# v49 changes autonomous detector semantics to posterior/no-prediction; layout is unchanged.
SAMPLE_FORMAT_V49 = SAMPLE_FORMAT_V48
# v50 adds lightweight 3 ms delay compensation; binary layout is unchanged.
SAMPLE_FORMAT_V50 = SAMPLE_FORMAT_V49
# v51 changes Autonomous amplitude/rate semantics, not the binary sample layout.
SAMPLE_FORMAT_V51 = SAMPLE_FORMAT_V50
SAMPLE_FORMAT_V52 = SAMPLE_FORMAT_V51 + "II8B"
HEADER_FIELDS = [
    "magic",
    "format_version",
    "header_size",
    "run_id",
    "run_start_us",
    "metadata_json_size",
    "sample_count",
    "summary_count",
    "event_count",
    "log_sample_size",
    "summary_row_size",
    "event_row_size",
    "log_period_ms",
    "imu_period_ms",
    "roller_read_period_ms",
    "web_update_period_ms",
    "total_trials",
    "preset_id",
    "flags",
    "samples_offset",
    "summaries_offset",
    "events_offset",
    "crc_offset",
    "reserved0",
    "reserved1",
    "reserved2",
    "reserved3",
    "reserved4",
    "reserved5",
    "reserved6",
    "reserved7",
]

CSV_COLUMNS_COMMON_PREFIX = [
    "time_s",
    "log_time_s",
    "t_test_ms",
    "state_id",
    "pulse_id",
    "pulse_active",
    "pulse_direction",
    "motor_cmd_mA",
    "current_mA_setting",
    "pulse_width_ms_setting",
    "input_interval_ms",
    "trial_index",
    "trial_count",
    "trial_elapsed_ms",
    "trial_duration_ms",
    "trial_current_mA",
    "trial_pulse_width_ms",
    "trial_input_interval_ms",
]

CSV_COLUMNS_COMMON_SUFFIX = [
    "trial_beta_tau_up_s",
    "pitch_madgwick_beta1_raw_deg",
    "pitch_madgwick_dynamic_raw_deg",
    "pitch_madgwick_beta1_bias_deg",
    "pitch_madgwick_dynamic_bias_deg",
    "pitch_gyro_raw_deg",
    "pitch_gyro_bias_corrected_deg",
    "pitch_accel_only_deg",
    "gyro_bias_x_dps",
    "gyro_bias_y_dps",
    "gyro_bias_z_dps",
    "gyro_pitch_rate_dps",
    "beta_target",
    "beta_smooth",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "acc_norm_g",
    "roller_actual_current_mA",
    "led_state",
    "sync_event_id",
    "log_active",
]

CSV_COLUMNS_V25 = CSV_COLUMNS_COMMON_PREFIX + [
    "trial_recommended_beta_low",
    "trial_beta_tau_up_s",
    "beta_low_b000",
    "beta_low_b005",
    "beta_low_b010",
    "beta_low_b020",
    "pitch_madgwick_beta1_raw_deg",
    "pitch_madgwick_beta1_bias_deg",
    "pitch_dynamic_b000_deg",
    "pitch_dynamic_b005_deg",
    "pitch_dynamic_b010_deg",
    "pitch_dynamic_b020_deg",
    "pitch_gyro_raw_deg",
    "pitch_gyro_bias_corrected_deg",
    "pitch_accel_only_deg",
    "gyro_bias_x_dps",
    "gyro_bias_y_dps",
    "gyro_bias_z_dps",
    "gyro_pitch_rate_dps",
    "beta_target_b000",
    "beta_target_b005",
    "beta_target_b010",
    "beta_target_b020",
    "beta_smooth_b000",
    "beta_smooth_b005",
    "beta_smooth_b010",
    "beta_smooth_b020",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "acc_norm_g",
    "roller_actual_current_mA",
    "roller_battery_mV",
    "led_state",
    "sync_event_id",
    "log_active",
]

CSV_COLUMNS_V30 = CSV_COLUMNS_COMMON_PREFIX + [
    "trial_predicted_beta_min",
    "beta_recovery_tau_s",
    "beta_model_vbat_mV",
    "predicted_i_goal_mA",
    "predicted_peak_current_mA",
    "beta_model_vbat_status",
    "strategy_beta_floor_fixed",
    "strategy_beta_floor_adopted",
    "strategy_beta_floor_b005",
    "strategy_beta_floor_b010",
    "pitch_madgwick_beta1_raw_deg",
    "pitch_madgwick_beta1_bias_deg",
    "pitch_fixed_b100_deg",
    "pitch_dynamic_adopted_deg",
    "pitch_dynamic_floor_b005_deg",
    "pitch_dynamic_floor_b010_deg",
    "pitch_gyro_raw_deg",
    "pitch_gyro_bias_corrected_deg",
    "pitch_accel_only_deg",
    "gyro_bias_x_dps",
    "gyro_bias_y_dps",
    "gyro_bias_z_dps",
    "gyro_pitch_rate_dps",
    "beta_target_fixed",
    "beta_target_adopted",
    "beta_target_floor_b005",
    "beta_target_floor_b010",
    "beta_applied_fixed",
    "beta_applied_adopted",
    "beta_applied_floor_b005",
    "beta_applied_floor_b010",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "acc_norm_g",
    "roller_actual_current_mA",
    "roller_battery_mV",
    "led_state",
    "sync_event_id",
    "log_active",
]


CSV_COLUMNS_V31 = CSV_COLUMNS_V30 + [
    "beta_phase_state",
    "beta_phase_progress",
    "beta_phase_peak_angle_deg",
    "beta_phase_angle_deg",
    "beta_phase_ceiling",
]

CSV_COLUMNS_V32 = CSV_COLUMNS_COMMON_PREFIX + [
    "trial_predicted_beta_min", "beta_recovery_tau_s", "beta_model_vbat_mV", "predicted_i_goal_mA", "predicted_peak_current_mA", "beta_model_vbat_status",
    "beta_ceiling_fixed", "beta_ceiling_dynamic_max025", "beta_ceiling_dynamic_max050", "beta_ceiling_dynamic_max075", "beta_ceiling_dynamic_max100", "beta_ceiling_dynamic_max125",
    "pitch_madgwick_beta1_raw_deg", "pitch_madgwick_beta1_bias_deg",
    "pitch_fixed_b100_deg", "pitch_dynamic_max025_deg", "pitch_dynamic_max050_deg", "pitch_dynamic_max075_deg", "pitch_dynamic_max100_deg", "pitch_dynamic_max125_deg",
    "pitch_gyro_raw_deg", "pitch_gyro_bias_corrected_deg", "pitch_accel_only_deg", "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps", "gyro_pitch_rate_dps",
    "beta_target_fixed", "beta_target_dynamic_max025", "beta_target_dynamic_max050", "beta_target_dynamic_max075", "beta_target_dynamic_max100", "beta_target_dynamic_max125",
    "beta_applied_fixed", "beta_applied_dynamic_max025", "beta_applied_dynamic_max050", "beta_applied_dynamic_max075", "beta_applied_dynamic_max100", "beta_applied_dynamic_max125",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "acc_norm_g", "roller_actual_current_mA", "roller_battery_mV", "led_state", "sync_event_id", "log_active",
    "beta_phase_state", "beta_phase_progress", "beta_phase_peak_angle_deg", "beta_phase_angle_deg", "beta_phase_ceiling",
]
CSV_COLUMNS_V34 = CSV_COLUMNS_COMMON_PREFIX + [
    "trial_predicted_beta_min", "beta_recovery_tau_s", "beta_model_vbat_mV", "predicted_i_goal_mA", "predicted_peak_current_mA", "beta_model_vbat_status",
    "beta_ceiling_fixed_b100", "beta_ceiling_fixed_b000", "beta_ceiling_dynamic_hold073", "beta_ceiling_dynamic_hold120", "beta_ceiling_dynamic_hold170",
    "pitch_madgwick_beta1_raw_deg", "pitch_madgwick_beta1_bias_deg",
    "pitch_fixed_b100_deg", "pitch_fixed_b000_deg", "pitch_dynamic_hold073_deg", "pitch_dynamic_hold120_deg", "pitch_dynamic_hold170_deg",
    "pitch_gyro_raw_deg", "pitch_gyro_bias_corrected_deg", "pitch_accel_only_deg", "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps", "gyro_pitch_rate_dps",
    "beta_target_fixed_b100", "beta_target_fixed_b000", "beta_target_dynamic_hold073", "beta_target_dynamic_hold120", "beta_target_dynamic_hold170",
    "beta_applied_fixed_b100", "beta_applied_fixed_b000", "beta_applied_dynamic_hold073", "beta_applied_dynamic_hold120", "beta_applied_dynamic_hold170",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "acc_norm_g", "roller_actual_current_mA", "roller_battery_mV", "led_state", "sync_event_id", "log_active",
    "beta_phase_state", "beta_phase_progress", "beta_phase_peak_angle_deg", "beta_phase_angle_deg", "beta_phase_ceiling",
]
CSV_COLUMNS_V35 = CSV_COLUMNS_COMMON_PREFIX + [

    "trial_predicted_beta_min", "beta_recovery_tau_s", "beta_model_vbat_mV", "predicted_i_goal_mA", "predicted_peak_current_mA", "beta_model_vbat_status",
    "beta_ceiling_fixed_b100", "beta_ceiling_fixed_b000", "beta_ceiling_dynamic_hold073", "beta_ceiling_dynamic_hold120", "beta_ceiling_dynamic_hold170", "beta_ceiling_dynamic_turnfast",
    "pitch_madgwick_beta1_raw_deg", "pitch_madgwick_beta1_bias_deg", "pitch_fixed_b100_deg", "pitch_fixed_b000_deg", "pitch_dynamic_hold073_deg", "pitch_dynamic_hold120_deg", "pitch_dynamic_hold170_deg", "pitch_dynamic_turnfast_deg",
    "pitch_gyro_raw_deg", "pitch_gyro_bias_corrected_deg", "pitch_accel_only_deg", "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps", "gyro_pitch_rate_dps",
    "beta_target_fixed_b100", "beta_target_fixed_b000", "beta_target_dynamic_hold073", "beta_target_dynamic_hold120", "beta_target_dynamic_hold170", "beta_target_dynamic_turnfast",
    "beta_applied_fixed_b100", "beta_applied_fixed_b000", "beta_applied_dynamic_hold073", "beta_applied_dynamic_hold120", "beta_applied_dynamic_hold170", "beta_applied_dynamic_turnfast",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "acc_norm_g", "roller_actual_current_mA", "roller_battery_mV", "led_state", "sync_event_id", "log_active",
    "turn_fast_state", "turn_fast_recovery_progress", "turn_fast_peak_angle_deg", "turn_fast_integrated_angle_deg", "turn_fast_beta_target",
]
CSV_COLUMNS_V42 = CSV_COLUMNS_V35 + [
    "physical_roll_abs_deg", "current_roll_deg", "physical_roll_rate_dps", "static_confirmed", "target_roll_deg", "target_error_deg", "ready",
]
CSV_COLUMNS_V45 = CSV_COLUMNS_V42 + [
    "roller_current_sample_time_us", "roller_current_sequence", "roller_current_age_us",
    "roller_current_read_failure_count", "roller_q_meas_observed_mA_s",
    "pulse_q_target_mA_s", "pulse_q_pred_mA_s", "roller_current_sample_count",
    "roller_current_valid", "roller_q_meas_observed_valid",
]
CSV_COLUMNS_V46 = CSV_COLUMNS_V45 + [
    "pitch_mekf_control_deg", "pitch_mekf_abs_deg", "pitch_madgwick_dynamic_abs_deg",
    "mekf_q_w", "mekf_q_x", "mekf_q_y", "mekf_q_z",
    "mekf_bias_x_dps", "mekf_bias_y_dps", "mekf_bias_z_dps",
    "mekf_accel_confidence", "mekf_accel_residual_deg", "mekf_accel_mag_error_g",
    "imu_update_dt_us", "imu_sample_age_us", "mekf_accel_used", "attitude_filter_adopted",
]
CSV_COLUMNS_V47 = CSV_COLUMNS_V46 + [
    "pitch_mekf_start_sync_relative_deg",
    "pitch_mekf_measurement_relative_deg",
    "pitch_mekf_trial_relative_deg",
    "mekf_start_sync_zero_abs_deg",
    "mekf_measurement_zero_abs_deg",
    "mekf_trial_zero_abs_deg",
    "mekf_start_sync_zero_sample_us",
    "mekf_measurement_zero_sample_us",
    "mekf_trial_zero_sample_us",
]
CSV_COLUMNS_V48 = CSV_COLUMNS_V47 + [
    "pitch_mekf_detector_relative_deg",
    "mekf_detector_zero_predicted_abs_deg",
    "mekf_detector_zero_sample_us",
]
CSV_COLUMNS_V49 = CSV_COLUMNS_V48
CSV_COLUMNS_V50 = CSV_COLUMNS_V49
CSV_COLUMNS_V51 = CSV_COLUMNS_V50
MAG_COLUMNS_V52 = ['mag_sample_us', 'mag_sequence', 'mag_raw_x', 'mag_raw_y', 'mag_raw_z', 'mag_rhall', 'mag_factory_ok', 'mag_value_valid', 'mag_body_x_uT', 'mag_body_y_uT', 'mag_body_z_uT', 'mag_norm_uT']
CSV_COLUMNS_V52 = CSV_COLUMNS_V51 + MAG_COLUMNS_V52
CSV_COLUMNS_V33 = CSV_COLUMNS_COMMON_PREFIX + [
    "trial_predicted_beta_min", "beta_recovery_tau_s", "beta_model_vbat_mV", "predicted_i_goal_mA", "predicted_peak_current_mA", "beta_model_vbat_status",
    "beta_ceiling_fixed", "beta_ceiling_dynamic_hold073", "beta_ceiling_dynamic_hold120", "beta_ceiling_dynamic_hold170",
    "pitch_madgwick_beta1_raw_deg", "pitch_madgwick_beta1_bias_deg",
    "pitch_fixed_b100_deg", "pitch_dynamic_hold073_deg", "pitch_dynamic_hold120_deg", "pitch_dynamic_hold170_deg",
    "pitch_gyro_raw_deg", "pitch_gyro_bias_corrected_deg", "pitch_accel_only_deg", "gyro_bias_x_dps", "gyro_bias_y_dps", "gyro_bias_z_dps", "gyro_pitch_rate_dps",
    "beta_target_fixed", "beta_target_dynamic_hold073", "beta_target_dynamic_hold120", "beta_target_dynamic_hold170",
    "beta_applied_fixed", "beta_applied_dynamic_hold073", "beta_applied_dynamic_hold120", "beta_applied_dynamic_hold170",
    "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "acc_norm_g", "roller_actual_current_mA", "roller_battery_mV", "led_state", "sync_event_id", "log_active",
    "beta_phase_state", "beta_phase_progress", "beta_phase_peak_angle_deg", "beta_phase_angle_deg", "beta_phase_ceiling",
]
def csv_columns_for_version(format_version: int) -> list[str]:
    if format_version >= 56:
        return CSV_COLUMNS_V56
    if format_version >= 55:
        return CSV_COLUMNS_V55
    if format_version >= 54:
        return CSV_COLUMNS_V54
    if format_version >= 53:
        return CSV_COLUMNS_V53
    if format_version >= 52:
        return CSV_COLUMNS_V52
    if format_version >= 50:
        return CSV_COLUMNS_V50
    if format_version >= 49:
        return CSV_COLUMNS_V49
    if format_version >= 48:
        return CSV_COLUMNS_V48
    if format_version >= 47:
        return CSV_COLUMNS_V47
    if format_version >= 46:
        return CSV_COLUMNS_V46
    if format_version >= 45:
        return CSV_COLUMNS_V45
    if format_version >= 42:
        return CSV_COLUMNS_V42
    if format_version >= 35:
        return CSV_COLUMNS_V35
    if format_version >= 34:
        return CSV_COLUMNS_V34
    if format_version >= 33:
        return CSV_COLUMNS_V33
    if format_version >= 32:
        return CSV_COLUMNS_V32
    if format_version >= 31:
        return CSV_COLUMNS_V31
    if format_version >= 30:
        return CSV_COLUMNS_V30
    if format_version >= 25:
        return CSV_COLUMNS_V25
    if format_version >= 24:
        beta_columns = ["trial_recommended_beta_low", "trial_beta_high"]
    else:
        beta_columns = ["trial_beta_pulse", "trial_beta_low"]
    return CSV_COLUMNS_COMMON_PREFIX + beta_columns + CSV_COLUMNS_COMMON_SUFFIX


def sample_format_for_version(format_version: int) -> str:
    if format_version >= 56:
        return SAMPLE_FORMAT_V56
    if format_version >= 55:
        return SAMPLE_FORMAT_V55
    if format_version >= 54:
        return SAMPLE_FORMAT_V54
    if format_version >= 53:
        return SAMPLE_FORMAT_V53
    if format_version >= 52:
        return SAMPLE_FORMAT_V52
    if format_version >= 50:
        return SAMPLE_FORMAT_V50
    if format_version >= 49:
        return SAMPLE_FORMAT_V49
    if format_version >= 48:
        return SAMPLE_FORMAT_V48
    if format_version >= 47:
        return SAMPLE_FORMAT_V47
    if format_version >= 46:
        return SAMPLE_FORMAT_V46
    if format_version >= 45:
        return SAMPLE_FORMAT_V45
    if format_version >= 42:
        return SAMPLE_FORMAT_V42
    if format_version >= 35:
        return SAMPLE_FORMAT_V35
    if format_version >= 34:
        return SAMPLE_FORMAT_V34
    if format_version >= 33:
        return SAMPLE_FORMAT_V33
    if format_version >= 32:
        return SAMPLE_FORMAT_V32
    if format_version >= 31:
        return SAMPLE_FORMAT_V31
    if format_version >= 30:
        return SAMPLE_FORMAT_V30
    if format_version >= 29:
        return SAMPLE_FORMAT_V29
    if format_version >= 25:
        return SAMPLE_FORMAT_V25
    return SAMPLE_FORMAT_V23_V24


def parse_header(data: bytes) -> dict:
    size = struct.calcsize(HEADER_FORMAT)
    values = struct.unpack_from(HEADER_FORMAT, data, 0)
    header = dict(zip(HEADER_FIELDS, values))
    header["magic"] = header["magic"].rstrip(b"\x00").decode("ascii", errors="replace")
    if header["magic"] != "RWLOG01":
        raise ValueError(f"bad magic: {header['magic']!r}")
    if header["header_size"] != size:
        raise ValueError(f"unsupported header size {header['header_size']} != {size}")
    return header


def verify_crc(data: bytes, header: dict) -> bool:
    crc_offset = header["crc_offset"]
    if crc_offset + 4 > len(data):
        raise ValueError("file is truncated before crc")
    stored = struct.unpack_from("<I", data, crc_offset)[0]
    computed = zlib.crc32(data[:crc_offset]) & 0xFFFFFFFF
    return stored == computed


def convert_sample_v25(values, has_battery_mV: bool = False):
    (
        _time_us,
        t_test_ms,
        state_id,
        pulse_id,
        pulse_active,
        pulse_direction,
        motor_cmd_mA,
        current_mA_setting,
        pulse_width_ms_setting,
        input_interval_ms,
        trial_index,
        trial_count,
        trial_elapsed_ms,
        trial_duration_ms,
        trial_current_mA,
        trial_pulse_width_ms,
        trial_input_interval_ms,
        trial_recommended_beta_low_x10000,
        trial_beta_tau_up_ms,
        beta_low_b000_x10000,
        beta_low_b005_x10000,
        beta_low_b010_x10000,
        beta_low_b020_x10000,
        pitch_madgwick_beta1_raw_cdeg,
        pitch_madgwick_beta1_bias_cdeg,
        pitch_dynamic_b000_cdeg,
        pitch_dynamic_b005_cdeg,
        pitch_dynamic_b010_cdeg,
        pitch_dynamic_b020_cdeg,
        pitch_gyro_raw_cdeg,
        pitch_gyro_bias_corrected_cdeg,
        pitch_accel_only_cdeg,
        gyro_bias_x_cdps,
        gyro_bias_y_cdps,
        gyro_bias_z_cdps,
        gyro_pitch_rate_cdps,
        beta_target_b000_x10000,
        beta_target_b005_x10000,
        beta_target_b010_x10000,
        beta_target_b020_x10000,
        beta_smooth_b000_x10000,
        beta_smooth_b005_x10000,
        beta_smooth_b010_x10000,
        beta_smooth_b020_x10000,
        ax_mg,
        ay_mg,
        az_mg,
        gx_cdps,
        gy_cdps,
        gz_cdps,
        acc_norm_mg,
        roller_actual_current_mA,
        *tail,
    ) = values
    if has_battery_mV:
        roller_battery_mV, led_state, sync_event_id, log_active = tail
    else:
        led_state, sync_event_id, log_active = tail
        roller_battery_mV = ""
    return {
        "time_s": f"{t_test_ms / 1000.0:.3f}",
        "log_time_s": f"{_time_us / 1000000.0:.6f}",
        "t_test_ms": t_test_ms,
        "state_id": state_id,
        "pulse_id": pulse_id,
        "pulse_active": pulse_active,
        "pulse_direction": pulse_direction,
        "motor_cmd_mA": motor_cmd_mA,
        "current_mA_setting": current_mA_setting,
        "pulse_width_ms_setting": pulse_width_ms_setting,
        "input_interval_ms": input_interval_ms,
        "trial_index": trial_index,
        "trial_count": trial_count,
        "trial_elapsed_ms": trial_elapsed_ms,
        "trial_duration_ms": trial_duration_ms,
        "trial_current_mA": trial_current_mA,
        "trial_pulse_width_ms": trial_pulse_width_ms,
        "trial_input_interval_ms": trial_input_interval_ms,
        "trial_recommended_beta_low": f"{trial_recommended_beta_low_x10000 / 10000.0:.5f}",
        "trial_beta_tau_up_s": f"{trial_beta_tau_up_ms / 1000.0:.3f}",
        "beta_low_b000": f"{beta_low_b000_x10000 / 10000.0:.5f}",
        "beta_low_b005": f"{beta_low_b005_x10000 / 10000.0:.5f}",
        "beta_low_b010": f"{beta_low_b010_x10000 / 10000.0:.5f}",
        "beta_low_b020": f"{beta_low_b020_x10000 / 10000.0:.5f}",
        "pitch_madgwick_beta1_raw_deg": f"{pitch_madgwick_beta1_raw_cdeg / 100.0:.3f}",
        "pitch_madgwick_beta1_bias_deg": f"{pitch_madgwick_beta1_bias_cdeg / 100.0:.3f}",
        "pitch_dynamic_b000_deg": f"{pitch_dynamic_b000_cdeg / 100.0:.3f}",
        "pitch_dynamic_b005_deg": f"{pitch_dynamic_b005_cdeg / 100.0:.3f}",
        "pitch_dynamic_b010_deg": f"{pitch_dynamic_b010_cdeg / 100.0:.3f}",
        "pitch_dynamic_b020_deg": f"{pitch_dynamic_b020_cdeg / 100.0:.3f}",
        "pitch_gyro_raw_deg": f"{pitch_gyro_raw_cdeg / 100.0:.3f}",
        "pitch_gyro_bias_corrected_deg": f"{pitch_gyro_bias_corrected_cdeg / 100.0:.3f}",
        "pitch_accel_only_deg": f"{pitch_accel_only_cdeg / 100.0:.3f}",
        "gyro_bias_x_dps": f"{gyro_bias_x_cdps / 100.0:.5f}",
        "gyro_bias_y_dps": f"{gyro_bias_y_cdps / 100.0:.5f}",
        "gyro_bias_z_dps": f"{gyro_bias_z_cdps / 100.0:.5f}",
        "gyro_pitch_rate_dps": f"{gyro_pitch_rate_cdps / 100.0:.4f}",
        "beta_target_b000": f"{beta_target_b000_x10000 / 10000.0:.5f}",
        "beta_target_b005": f"{beta_target_b005_x10000 / 10000.0:.5f}",
        "beta_target_b010": f"{beta_target_b010_x10000 / 10000.0:.5f}",
        "beta_target_b020": f"{beta_target_b020_x10000 / 10000.0:.5f}",
        "beta_smooth_b000": f"{beta_smooth_b000_x10000 / 10000.0:.5f}",
        "beta_smooth_b005": f"{beta_smooth_b005_x10000 / 10000.0:.5f}",
        "beta_smooth_b010": f"{beta_smooth_b010_x10000 / 10000.0:.5f}",
        "beta_smooth_b020": f"{beta_smooth_b020_x10000 / 10000.0:.5f}",
        "ax_g": f"{ax_mg / 1000.0:.4f}",
        "ay_g": f"{ay_mg / 1000.0:.4f}",
        "az_g": f"{az_mg / 1000.0:.4f}",
        "gx_dps": f"{gx_cdps / 100.0:.4f}",
        "gy_dps": f"{gy_cdps / 100.0:.4f}",
        "gz_dps": f"{gz_cdps / 100.0:.4f}",
        "acc_norm_g": f"{acc_norm_mg / 1000.0:.4f}",
        "roller_actual_current_mA": roller_actual_current_mA,
        "roller_battery_mV": roller_battery_mV,
        "led_state": led_state,
        "sync_event_id": sync_event_id,
        "log_active": log_active,
    }


def convert_sample_v30(values):
    beta_model_vbat_mV = values[19]
    predicted_i_goal_mA = values[20]
    predicted_peak_current_mA = values[21]
    beta_model_vbat_status = values[22]
    legacy_layout_values = values[:19] + values[23:]
    row = convert_sample_v25(legacy_layout_values, has_battery_mV=True)
    renamed = {
        "trial_recommended_beta_low": "trial_predicted_beta_min",
        "trial_beta_tau_up_s": "beta_recovery_tau_s",
        "beta_low_b000": "strategy_beta_floor_fixed",
        "beta_low_b005": "strategy_beta_floor_adopted",
        "beta_low_b010": "strategy_beta_floor_b005",
        "beta_low_b020": "strategy_beta_floor_b010",
        "pitch_dynamic_b000_deg": "pitch_fixed_b100_deg",
        "pitch_dynamic_b005_deg": "pitch_dynamic_adopted_deg",
        "pitch_dynamic_b010_deg": "pitch_dynamic_floor_b005_deg",
        "pitch_dynamic_b020_deg": "pitch_dynamic_floor_b010_deg",
        "beta_target_b000": "beta_target_fixed",
        "beta_target_b005": "beta_target_adopted",
        "beta_target_b010": "beta_target_floor_b005",
        "beta_target_b020": "beta_target_floor_b010",
        "beta_smooth_b000": "beta_applied_fixed",
        "beta_smooth_b005": "beta_applied_adopted",
        "beta_smooth_b010": "beta_applied_floor_b005",
        "beta_smooth_b020": "beta_applied_floor_b010",
    }
    for old_name, new_name in renamed.items():
        row[new_name] = row.pop(old_name)
    row["beta_model_vbat_mV"] = beta_model_vbat_mV
    row["predicted_i_goal_mA"] = predicted_i_goal_mA
    row["predicted_peak_current_mA"] = predicted_peak_current_mA
    row["beta_model_vbat_status"] = beta_model_vbat_status
    return row


def convert_sample_v31(values):
    row = convert_sample_v30(values[:-5])
    (
        beta_phase_state,
        beta_phase_progress_x10000,
        beta_phase_peak_angle_cdeg,
        beta_phase_angle_cdeg,
        beta_phase_ceiling_x10000,
    ) = values[-5:]
    row["beta_phase_state"] = beta_phase_state
    row["beta_phase_progress"] = f"{beta_phase_progress_x10000 / 10000.0:.5f}"
    row["beta_phase_peak_angle_deg"] = f"{beta_phase_peak_angle_cdeg / 100.0:.3f}"
    row["beta_phase_angle_deg"] = f"{beta_phase_angle_cdeg / 100.0:.3f}"
    row["beta_phase_ceiling"] = f"{beta_phase_ceiling_x10000 / 10000.0:.5f}"
    return row

def convert_sample_v32(values):
    def deg(value):
        return f"{value / 100.0:.3f}"

    def beta(value):
        return f"{value / 10000.0:.5f}"

    row = {
        "time_s": f"{values[1] / 1000.0:.3f}", "log_time_s": f"{values[0] / 1000000.0:.6f}",
        "t_test_ms": values[1], "state_id": values[2], "pulse_id": values[3], "pulse_active": values[4],
        "pulse_direction": values[5], "motor_cmd_mA": values[6], "current_mA_setting": values[7],
        "pulse_width_ms_setting": values[8], "input_interval_ms": values[9], "trial_index": values[10],
        "trial_count": values[11], "trial_elapsed_ms": values[12], "trial_duration_ms": values[13],
        "trial_current_mA": values[14], "trial_pulse_width_ms": values[15], "trial_input_interval_ms": values[16],
        "trial_predicted_beta_min": beta(values[17]), "beta_recovery_tau_s": f"{values[18] / 1000.0:.3f}",
        "beta_model_vbat_mV": values[19], "predicted_i_goal_mA": values[20], "predicted_peak_current_mA": values[21],
        "beta_model_vbat_status": values[22],
    }
    ceiling_names = ["beta_ceiling_fixed", "beta_ceiling_dynamic_max025", "beta_ceiling_dynamic_max050", "beta_ceiling_dynamic_max075", "beta_ceiling_dynamic_max100", "beta_ceiling_dynamic_max125"]
    pitch_names = ["pitch_fixed_b100_deg", "pitch_dynamic_max025_deg", "pitch_dynamic_max050_deg", "pitch_dynamic_max075_deg", "pitch_dynamic_max100_deg", "pitch_dynamic_max125_deg"]
    target_names = ["beta_target_fixed", "beta_target_dynamic_max025", "beta_target_dynamic_max050", "beta_target_dynamic_max075", "beta_target_dynamic_max100", "beta_target_dynamic_max125"]
    applied_names = ["beta_applied_fixed", "beta_applied_dynamic_max025", "beta_applied_dynamic_max050", "beta_applied_dynamic_max075", "beta_applied_dynamic_max100", "beta_applied_dynamic_max125"]
    for i, name in enumerate(ceiling_names): row[name] = beta(values[23 + i])
    row["pitch_madgwick_beta1_raw_deg"] = deg(values[29])
    row["pitch_madgwick_beta1_bias_deg"] = deg(values[30])
    for i, name in enumerate(pitch_names): row[name] = deg(values[31 + i])
    row["pitch_gyro_raw_deg"] = deg(values[37]); row["pitch_gyro_bias_corrected_deg"] = deg(values[38]); row["pitch_accel_only_deg"] = deg(values[39])
    row["gyro_bias_x_dps"] = f"{values[40] / 100.0:.5f}"; row["gyro_bias_y_dps"] = f"{values[41] / 100.0:.5f}"; row["gyro_bias_z_dps"] = f"{values[42] / 100.0:.5f}"; row["gyro_pitch_rate_dps"] = f"{values[43] / 100.0:.4f}"
    for i, name in enumerate(target_names): row[name] = beta(values[44 + i])
    for i, name in enumerate(applied_names): row[name] = beta(values[50 + i])
    row["ax_g"] = f"{values[56] / 1000.0:.4f}"; row["ay_g"] = f"{values[57] / 1000.0:.4f}"; row["az_g"] = f"{values[58] / 1000.0:.4f}"
    row["gx_dps"] = f"{values[59] / 100.0:.4f}"; row["gy_dps"] = f"{values[60] / 100.0:.4f}"; row["gz_dps"] = f"{values[61] / 100.0:.4f}"; row["acc_norm_g"] = f"{values[62] / 1000.0:.4f}"
    row["roller_actual_current_mA"] = values[63]; row["roller_battery_mV"] = values[64]; row["led_state"] = values[65]; row["sync_event_id"] = values[66]; row["log_active"] = values[67]
    row["beta_phase_state"] = values[68]; row["beta_phase_progress"] = beta(values[69]); row["beta_phase_peak_angle_deg"] = deg(values[70]); row["beta_phase_angle_deg"] = deg(values[71]); row["beta_phase_ceiling"] = beta(values[72])
    return row
def convert_sample_v33(values):
    def deg(value):
        return f"{value / 100.0:.3f}"

    def beta(value):
        return f"{value / 10000.0:.5f}"

    row = {
        "time_s": f"{values[1] / 1000.0:.3f}", "log_time_s": f"{values[0] / 1000000.0:.6f}",
        "t_test_ms": values[1], "state_id": values[2], "pulse_id": values[3], "pulse_active": values[4],
        "pulse_direction": values[5], "motor_cmd_mA": values[6], "current_mA_setting": values[7],
        "pulse_width_ms_setting": values[8], "input_interval_ms": values[9], "trial_index": values[10],
        "trial_count": values[11], "trial_elapsed_ms": values[12], "trial_duration_ms": values[13],
        "trial_current_mA": values[14], "trial_pulse_width_ms": values[15], "trial_input_interval_ms": values[16],
        "trial_predicted_beta_min": beta(values[17]), "beta_recovery_tau_s": f"{values[18] / 1000.0:.3f}",
        "beta_model_vbat_mV": values[19], "predicted_i_goal_mA": values[20], "predicted_peak_current_mA": values[21],
        "beta_model_vbat_status": values[22],
    }
    ceiling_names = ["beta_ceiling_fixed", "beta_ceiling_dynamic_hold073", "beta_ceiling_dynamic_hold120", "beta_ceiling_dynamic_hold170"]
    pitch_names = ["pitch_fixed_b100_deg", "pitch_dynamic_hold073_deg", "pitch_dynamic_hold120_deg", "pitch_dynamic_hold170_deg"]
    target_names = ["beta_target_fixed", "beta_target_dynamic_hold073", "beta_target_dynamic_hold120", "beta_target_dynamic_hold170"]
    applied_names = ["beta_applied_fixed", "beta_applied_dynamic_hold073", "beta_applied_dynamic_hold120", "beta_applied_dynamic_hold170"]
    for i, name in enumerate(ceiling_names): row[name] = beta(values[23 + i])
    row["pitch_madgwick_beta1_raw_deg"] = deg(values[27])
    row["pitch_madgwick_beta1_bias_deg"] = deg(values[28])
    for i, name in enumerate(pitch_names): row[name] = deg(values[29 + i])
    row["pitch_gyro_raw_deg"] = deg(values[33]); row["pitch_gyro_bias_corrected_deg"] = deg(values[34]); row["pitch_accel_only_deg"] = deg(values[35])
    row["gyro_bias_x_dps"] = f"{values[36] / 100.0:.5f}"; row["gyro_bias_y_dps"] = f"{values[37] / 100.0:.5f}"; row["gyro_bias_z_dps"] = f"{values[38] / 100.0:.5f}"; row["gyro_pitch_rate_dps"] = f"{values[39] / 100.0:.4f}"
    for i, name in enumerate(target_names): row[name] = beta(values[40 + i])
    for i, name in enumerate(applied_names): row[name] = beta(values[44 + i])
    row["ax_g"] = f"{values[48] / 1000.0:.4f}"; row["ay_g"] = f"{values[49] / 1000.0:.4f}"; row["az_g"] = f"{values[50] / 1000.0:.4f}"
    row["gx_dps"] = f"{values[51] / 100.0:.4f}"; row["gy_dps"] = f"{values[52] / 100.0:.4f}"; row["gz_dps"] = f"{values[53] / 100.0:.4f}"; row["acc_norm_g"] = f"{values[54] / 1000.0:.4f}"
    row["roller_actual_current_mA"] = values[55]; row["roller_battery_mV"] = values[56]; row["led_state"] = values[57]; row["sync_event_id"] = values[58]; row["log_active"] = values[59]
    row["beta_phase_state"] = values[60]; row["beta_phase_progress"] = beta(values[61]); row["beta_phase_peak_angle_deg"] = deg(values[62]); row["beta_phase_angle_deg"] = deg(values[63]); row["beta_phase_ceiling"] = beta(values[64])
    return row
def convert_sample_v34(values):
    def deg(v): return f"{v / 100.0:.3f}"
    def beta(v): return f"{v / 10000.0:.5f}"
    row = {"time_s": f"{values[1] / 1000.0:.3f}", "log_time_s": f"{values[0] / 1000000.0:.6f}", "t_test_ms": values[1], "state_id": values[2], "pulse_id": values[3], "pulse_active": values[4], "pulse_direction": values[5], "motor_cmd_mA": values[6], "current_mA_setting": values[7], "pulse_width_ms_setting": values[8], "input_interval_ms": values[9], "trial_index": values[10], "trial_count": values[11], "trial_elapsed_ms": values[12], "trial_duration_ms": values[13], "trial_current_mA": values[14], "trial_pulse_width_ms": values[15], "trial_input_interval_ms": values[16], "trial_predicted_beta_min": beta(values[17]), "beta_recovery_tau_s": f"{values[18] / 1000.0:.3f}", "beta_model_vbat_mV": values[19], "predicted_i_goal_mA": values[20], "predicted_peak_current_mA": values[21], "beta_model_vbat_status": values[22]}
    ceilings=["beta_ceiling_fixed_b100","beta_ceiling_fixed_b000","beta_ceiling_dynamic_hold073","beta_ceiling_dynamic_hold120","beta_ceiling_dynamic_hold170"]; pitches=["pitch_fixed_b100_deg","pitch_fixed_b000_deg","pitch_dynamic_hold073_deg","pitch_dynamic_hold120_deg","pitch_dynamic_hold170_deg"]; targets=["beta_target_fixed_b100","beta_target_fixed_b000","beta_target_dynamic_hold073","beta_target_dynamic_hold120","beta_target_dynamic_hold170"]; applied=["beta_applied_fixed_b100","beta_applied_fixed_b000","beta_applied_dynamic_hold073","beta_applied_dynamic_hold120","beta_applied_dynamic_hold170"]
    for i,n in enumerate(ceilings): row[n]=beta(values[23+i])
    row["pitch_madgwick_beta1_raw_deg"]=deg(values[28]); row["pitch_madgwick_beta1_bias_deg"]=deg(values[29])
    for i,n in enumerate(pitches): row[n]=deg(values[30+i])
    for n,i in [("pitch_gyro_raw_deg",35),("pitch_gyro_bias_corrected_deg",36),("pitch_accel_only_deg",37)]: row[n]=deg(values[i])
    for n,i in [("gyro_bias_x_dps",38),("gyro_bias_y_dps",39),("gyro_bias_z_dps",40),("gyro_pitch_rate_dps",41)]: row[n]=f"{values[i]/100.0:.5f}"
    for i,n in enumerate(targets): row[n]=beta(values[42+i])
    for i,n in enumerate(applied): row[n]=beta(values[47+i])
    for n,i,s in [("ax_g",52,1000),("ay_g",53,1000),("az_g",54,1000),("gx_dps",55,100),("gy_dps",56,100),("gz_dps",57,100),("acc_norm_g",58,1000)]: row[n]=f"{values[i]/s:.4f}"
    row["roller_actual_current_mA"]=values[59]; row["roller_battery_mV"]=values[60]; row["led_state"]=values[61]; row["sync_event_id"]=values[62]; row["log_active"]=values[63]; row["beta_phase_state"]=values[64]; row["beta_phase_progress"]=beta(values[65]); row["beta_phase_peak_angle_deg"]=deg(values[66]); row["beta_phase_angle_deg"]=deg(values[67]); row["beta_phase_ceiling"]=beta(values[68]); return row
def convert_sample_v35(values):
    def deg(v): return f"{v / 100.0:.3f}"
    def beta(v): return f"{v / 10000.0:.5f}"
    row = {"time_s": f"{values[1] / 1000.0:.3f}", "log_time_s": f"{values[0] / 1000000.0:.6f}", "t_test_ms": values[1], "state_id": values[2], "pulse_id": values[3], "pulse_active": values[4], "pulse_direction": values[5], "motor_cmd_mA": values[6], "current_mA_setting": values[7], "pulse_width_ms_setting": values[8], "input_interval_ms": values[9], "trial_index": values[10], "trial_count": values[11], "trial_elapsed_ms": values[12], "trial_duration_ms": values[13], "trial_current_mA": values[14], "trial_pulse_width_ms": values[15], "trial_input_interval_ms": values[16], "trial_predicted_beta_min": beta(values[17]), "beta_recovery_tau_s": f"{values[18] / 1000.0:.3f}", "beta_model_vbat_mV": values[19], "predicted_i_goal_mA": values[20], "predicted_peak_current_mA": values[21], "beta_model_vbat_status": values[22]}
    names = [(23, ["beta_ceiling_fixed_b100","beta_ceiling_fixed_b000","beta_ceiling_dynamic_hold073","beta_ceiling_dynamic_hold120","beta_ceiling_dynamic_hold170","beta_ceiling_dynamic_turnfast"], beta), (31, ["pitch_fixed_b100_deg","pitch_fixed_b000_deg","pitch_dynamic_hold073_deg","pitch_dynamic_hold120_deg","pitch_dynamic_hold170_deg","pitch_dynamic_turnfast_deg"], deg), (44, ["beta_target_fixed_b100","beta_target_fixed_b000","beta_target_dynamic_hold073","beta_target_dynamic_hold120","beta_target_dynamic_hold170","beta_target_dynamic_turnfast"], beta), (50, ["beta_applied_fixed_b100","beta_applied_fixed_b000","beta_applied_dynamic_hold073","beta_applied_dynamic_hold120","beta_applied_dynamic_hold170","beta_applied_dynamic_turnfast"], beta)]
    for start, labels, convert in names:
        for i, label in enumerate(labels): row[label] = convert(values[start + i])
    row["pitch_madgwick_beta1_raw_deg"] = deg(values[29]); row["pitch_madgwick_beta1_bias_deg"] = deg(values[30])
    for label, i in [("pitch_gyro_raw_deg",37),("pitch_gyro_bias_corrected_deg",38),("pitch_accel_only_deg",39)]: row[label] = deg(values[i])
    for label, i in [("gyro_bias_x_dps",40),("gyro_bias_y_dps",41),("gyro_bias_z_dps",42),("gyro_pitch_rate_dps",43)]: row[label] = f"{values[i]/100.0:.5f}"
    for label, i, scale in [("ax_g",56,1000),("ay_g",57,1000),("az_g",58,1000),("gx_dps",59,100),("gy_dps",60,100),("gz_dps",61,100),("acc_norm_g",62,1000)]: row[label] = f"{values[i]/scale:.4f}"
    row["roller_actual_current_mA"] = values[63]; row["roller_battery_mV"] = values[64]; row["led_state"] = values[65]; row["sync_event_id"] = values[66]; row["log_active"] = values[67]
    row["turn_fast_state"] = values[68]; row["turn_fast_recovery_progress"] = beta(values[69]); row["turn_fast_peak_angle_deg"] = deg(values[70]); row["turn_fast_integrated_angle_deg"] = deg(values[71]); row["turn_fast_beta_target"] = beta(values[72])
    return row
def convert_sample_v42(values):
    def deg(v): return f"{v / 100.0:.3f}"
    row = convert_sample_v35(values[:73])
    row["physical_roll_abs_deg"] = deg(values[73])
    row["current_roll_deg"] = deg(values[74])
    row["physical_roll_rate_dps"] = f"{values[75] / 100.0:.4f}"
    row["static_confirmed"] = values[78]
    row["target_roll_deg"] = deg(values[76])
    row["target_error_deg"] = deg(values[77])
    row["ready"] = values[79]
    return row
def convert_sample_v45(values):
    row = convert_sample_v42(values[:80])
    row["roller_current_sample_time_us"] = values[80]
    row["roller_current_sequence"] = values[81]
    row["roller_current_age_us"] = values[82]
    row["roller_current_read_failure_count"] = values[83]
    row["roller_q_meas_observed_mA_s"] = "" if values[84] == -2147483648 else f"{values[84] / 1000.0:.6f}"
    row["pulse_q_target_mA_s"] = "" if values[85] == -2147483648 else f"{values[85] / 1000.0:.6f}"
    row["pulse_q_pred_mA_s"] = "" if values[86] == -2147483648 else f"{values[86] / 1000.0:.6f}"
    row["roller_current_sample_count"] = values[87]
    row["roller_current_valid"] = values[88]
    row["roller_q_meas_observed_valid"] = values[89]
    return row


def convert_sample_v46(values):
    row = convert_sample_v45(values[:90])
    row["pitch_mekf_control_deg"] = f"{values[90] / 100.0:.3f}"
    row["pitch_mekf_abs_deg"] = f"{values[91] / 100.0:.3f}"
    row["pitch_madgwick_dynamic_abs_deg"] = f"{values[92] / 100.0:.3f}"
    row["mekf_q_w"] = f"{values[93] / 10000.0:.6f}"
    row["mekf_q_x"] = f"{values[94] / 10000.0:.6f}"
    row["mekf_q_y"] = f"{values[95] / 10000.0:.6f}"
    row["mekf_q_z"] = f"{values[96] / 10000.0:.6f}"
    row["mekf_bias_x_dps"] = f"{values[97] / 100.0:.5f}"
    row["mekf_bias_y_dps"] = f"{values[98] / 100.0:.5f}"
    row["mekf_bias_z_dps"] = f"{values[99] / 100.0:.5f}"
    row["mekf_accel_confidence"] = f"{values[100] / 10000.0:.5f}"
    row["mekf_accel_residual_deg"] = f"{values[101] / 100.0:.3f}"
    row["mekf_accel_mag_error_g"] = f"{values[102] / 1000.0:.4f}"
    row["imu_update_dt_us"] = values[103]
    row["imu_sample_age_us"] = values[104]
    row["mekf_accel_used"] = values[105]
    row["attitude_filter_adopted"] = values[106]
    return row



def convert_sample_v47(values):
    row = convert_sample_v46(values[:107])
    def deg_or_blank(value):
        return "" if value == -32768 else f"{value / 100.0:.3f}"
    row["pitch_mekf_start_sync_relative_deg"] = deg_or_blank(values[107])
    row["pitch_mekf_measurement_relative_deg"] = deg_or_blank(values[108])
    row["pitch_mekf_trial_relative_deg"] = deg_or_blank(values[109])
    row["mekf_start_sync_zero_abs_deg"] = deg_or_blank(values[110])
    row["mekf_measurement_zero_abs_deg"] = deg_or_blank(values[111])
    row["mekf_trial_zero_abs_deg"] = deg_or_blank(values[112])
    row["mekf_start_sync_zero_sample_us"] = values[113]
    row["mekf_measurement_zero_sample_us"] = values[114]
    row["mekf_trial_zero_sample_us"] = values[115]
    return row


def convert_sample_v48(values):
    row = convert_sample_v47(values[:116])
    def deg_or_blank(value):
        return "" if value == -32768 else f"{value / 100.0:.3f}"
    row["pitch_mekf_detector_relative_deg"] = deg_or_blank(values[116])
    row["mekf_detector_zero_predicted_abs_deg"] = deg_or_blank(values[117])
    row["mekf_detector_zero_sample_us"] = values[118]
    return row


def magnetic_values(aux, trim, sequence):
    # Bosch BMM150 float equations; factory sensitivity/temperature compensation.
    # No hard/soft-iron calibration or heading fusion is implied.
    p = bytes(aux)
    x = int.from_bytes(p[0:2], "little", signed=True) >> 3
    y = int.from_bytes(p[2:4], "little", signed=True) >> 3
    z = int.from_bytes(p[4:6], "little", signed=True) >> 1
    h = int.from_bytes(p[6:8], "little") >> 2
    keys = ("x1", "y1", "x2", "y2", "xy1", "xy2", "z1", "z2", "z3", "z4", "xyz1")
    ok = isinstance(trim, dict) and all(isinstance(trim.get(k), (int, float)) and math.isfinite(trim[k]) for k in keys)
    ok = bool(ok and trim['z1'] and trim['z2'] and trim['xyz1'])
    result = [x, y, z, h, int(ok), 0, "", "", "", ""]
    if not ok or not sequence or not h or x == -4096 or y == -4096 or z == -16384:
        return result
    t = trim
    a = t['xyz1'] * 16384.0 / h - 16384.0
    b = t['xy2'] * a*a / 268435456.0 + a*t['xy1']/16384.0 + 256.0
    den = t['z2'] + t['z1']*h/32768.0
    if abs(den) < 1e-6:
        return result
    X = (x*b*(t['x2']+160)/8192.0 + t['x1']*8)/16.0
    Y = (y*b*(t['y2']+160)/8192.0 + t['y1']*8)/16.0
    Z = ((z-t['z4'])*131072.0 - t['z3']*(h-t['xyz1']))/(den*4)/16.0
    norm = math.sqrt(X*X+Y*Y+Z*Z)
    if all(math.isfinite(v) for v in (X,Y,Z,norm)):
        result[5:] = [1] + [f"{v:.6f}" for v in (X,Y,Z,norm)]
    return result


def convert_sample(values, format_version: int, mag_trim=None):
    if format_version in (53, 54, 55, 56):
        row = {}
        fields = FIELDS_V56 if format_version == 56 else FIELDS_V55 if format_version == 55 else FIELDS_V54 if format_version == 54 else FIELDS_V53
        for value, (fmt, name, scale) in zip(values, fields):
            invalid = (fmt == 'h' and value == -32768) or (fmt == 'i' and value == -2147483648 and name != 'roller_position_raw_cdeg')
            row[name] = '' if invalid else value / scale if scale != 1 else value
        if format_version == 56:
            row['roller_position_deg'] = row['roller_position_raw_cdeg'] / 100 if row['roller_position_valid'] else ''
        row['time_s'] = values[1] / 1000.0
        return row
    if format_version >= 52:
        row = convert_sample_v48(values[:119])
        sample_us, sequence = values[119:121]
        fields = [sample_us, sequence] + magnetic_values(values[121:129], mag_trim, sequence)
        row.update(zip(MAG_COLUMNS_V52, fields))
        return row
    if format_version >= 50:
        return convert_sample_v48(values)
    if format_version >= 49:
        return convert_sample_v48(values)
    if format_version >= 48:
        return convert_sample_v48(values)
    if format_version >= 47:
        return convert_sample_v47(values)
    if format_version >= 46:
        return convert_sample_v46(values)
    if format_version >= 45:
        return convert_sample_v45(values)
    if format_version >= 42:
        return convert_sample_v42(values)
    if format_version >= 35:
        return convert_sample_v35(values)
    if format_version >= 34:
        return convert_sample_v34(values)
    if format_version >= 33:
        return convert_sample_v33(values)
    if format_version >= 32:
        return convert_sample_v32(values)
    if format_version >= 31:
        return convert_sample_v31(values)
    if format_version >= 30:
        return convert_sample_v30(values)
    if format_version >= 25:
        return convert_sample_v25(values, has_battery_mV=format_version >= 29)
    (
        _time_us,
        t_test_ms,
        state_id,
        pulse_id,
        pulse_active,
        pulse_direction,
        motor_cmd_mA,
        current_mA_setting,
        pulse_width_ms_setting,
        input_interval_ms,
        trial_index,
        trial_count,
        trial_elapsed_ms,
        trial_duration_ms,
        trial_current_mA,
        trial_pulse_width_ms,
        trial_input_interval_ms,
        trial_beta_first_x10000,
        trial_beta_second_x10000,
        trial_beta_tau_up_ms,
        pitch_madgwick_beta1_raw_cdeg,
        pitch_madgwick_dynamic_raw_cdeg,
        pitch_madgwick_beta1_bias_cdeg,
        pitch_madgwick_dynamic_bias_cdeg,
        pitch_gyro_raw_cdeg,
        pitch_gyro_bias_corrected_cdeg,
        pitch_accel_only_cdeg,
        gyro_bias_x_cdps,
        gyro_bias_y_cdps,
        gyro_bias_z_cdps,
        gyro_pitch_rate_cdps,
        beta_target_x10000,
        beta_smooth_x10000,
        ax_mg,
        ay_mg,
        az_mg,
        gx_cdps,
        gy_cdps,
        gz_cdps,
        acc_norm_mg,
        roller_actual_current_mA,
        led_state,
        sync_event_id,
        log_active,
    ) = values
    row = {
        "time_s": f"{t_test_ms / 1000.0:.3f}",
        "log_time_s": f"{_time_us / 1000000.0:.6f}",
        "t_test_ms": t_test_ms,
        "state_id": state_id,
        "pulse_id": pulse_id,
        "pulse_active": pulse_active,
        "pulse_direction": pulse_direction,
        "motor_cmd_mA": motor_cmd_mA,
        "current_mA_setting": current_mA_setting,
        "pulse_width_ms_setting": pulse_width_ms_setting,
        "input_interval_ms": input_interval_ms,
        "trial_index": trial_index,
        "trial_count": trial_count,
        "trial_elapsed_ms": trial_elapsed_ms,
        "trial_duration_ms": trial_duration_ms,
        "trial_current_mA": trial_current_mA,
        "trial_pulse_width_ms": trial_pulse_width_ms,
        "trial_input_interval_ms": trial_input_interval_ms,
        "trial_beta_tau_up_s": f"{trial_beta_tau_up_ms / 1000.0:.3f}",
        "pitch_madgwick_beta1_raw_deg": f"{pitch_madgwick_beta1_raw_cdeg / 100.0:.3f}",
        "pitch_madgwick_dynamic_raw_deg": f"{pitch_madgwick_dynamic_raw_cdeg / 100.0:.3f}",
        "pitch_madgwick_beta1_bias_deg": f"{pitch_madgwick_beta1_bias_cdeg / 100.0:.3f}",
        "pitch_madgwick_dynamic_bias_deg": f"{pitch_madgwick_dynamic_bias_cdeg / 100.0:.3f}",
        "pitch_gyro_raw_deg": f"{pitch_gyro_raw_cdeg / 100.0:.3f}",
        "pitch_gyro_bias_corrected_deg": f"{pitch_gyro_bias_corrected_cdeg / 100.0:.3f}",
        "pitch_accel_only_deg": f"{pitch_accel_only_cdeg / 100.0:.3f}",
        "gyro_bias_x_dps": f"{gyro_bias_x_cdps / 100.0:.5f}",
        "gyro_bias_y_dps": f"{gyro_bias_y_cdps / 100.0:.5f}",
        "gyro_bias_z_dps": f"{gyro_bias_z_cdps / 100.0:.5f}",
        "gyro_pitch_rate_dps": f"{gyro_pitch_rate_cdps / 100.0:.4f}",
        "beta_target": f"{beta_target_x10000 / 10000.0:.5f}",
        "beta_smooth": f"{beta_smooth_x10000 / 10000.0:.5f}",
        "ax_g": f"{ax_mg / 1000.0:.4f}",
        "ay_g": f"{ay_mg / 1000.0:.4f}",
        "az_g": f"{az_mg / 1000.0:.4f}",
        "gx_dps": f"{gx_cdps / 100.0:.4f}",
        "gy_dps": f"{gy_cdps / 100.0:.4f}",
        "gz_dps": f"{gz_cdps / 100.0:.4f}",
        "acc_norm_g": f"{acc_norm_mg / 1000.0:.4f}",
        "roller_actual_current_mA": roller_actual_current_mA,
        "led_state": led_state,
        "sync_event_id": sync_event_id,
        "log_active": log_active,
    }
    if format_version >= 24:
        row["trial_recommended_beta_low"] = f"{trial_beta_first_x10000 / 10000.0:.5f}"
        row["trial_beta_high"] = f"{trial_beta_second_x10000 / 10000.0:.5f}"
    else:
        row["trial_beta_pulse"] = f"{trial_beta_first_x10000 / 10000.0:.5f}"
        row["trial_beta_low"] = f"{trial_beta_second_x10000 / 10000.0:.5f}"
    return row


E2_SHADOW_PEAK_COLUMNS = [
    "event_index", "e2_armed", "release_detected_ms", "turn_index",
    "candidate_peak_ms", "confirmed_ms", "turn_side_from_rate", "alternating_side_ok",
    "previous_peak", "current_peak", "halfcycle_gyro_integral_deg", "H_prev_gyro_deg",
    "H_prev", "H_next_E2_deg", "H_next_E2", "static_anchor_abs_deg",
    "peak_gyro_integrated_abs_deg", "peak_accel_abs_diag_deg", "passive_next_peak",
    "physical_next_peak_side", "target_peak", "delta_A_required", "shadow_gain",
    "Q_req_shadow", "shadow_valid", "shadow_invalid_reason", "shadow_invalid_reason_code",
]

def write_e2_shadow_peaks(metadata: dict, out_dir: Path) -> int:
    """Write the E2 gyro-halfcycle H-space audit table when present in metadata."""
    events = metadata.get("e2_shadow_peak_events")
    if not isinstance(events, list):
        return 0
    with (out_dir / "e2_shadow_peaks.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=E2_SHADOW_PEAK_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for event in events:
            if isinstance(event, dict):
                writer.writerow({name: event.get(name) for name in E2_SHADOW_PEAK_COLUMNS})
    return len(events)


Q1_SHADOW_COLUMNS = [
    "q1_shadow_event_index", "zero_cross_time_ms", "zero_cross_rate_dps",
    "zero_cross_abs_rate_dps", "physical_next_peak_side", "q1_intercept_deg",
    "detector_crossing_direction", "detector_angle_before_deg", "detector_angle_after_deg",
    "crossing_interpolation_alpha", "interpolated_zero_cross_time_ms",
    "physical_roll_rate_before_dps", "physical_roll_rate_after_dps",
    "interpolated_physical_roll_rate_dps", "sign_gate_passed",
    "q1_rate_term_deg", "q1_side_term_deg", "q1_baseline_next_peak_abs_deg",
    "target_next_peak_abs_deg", "delta_peak_required_deg", "q1_gain_deg_per_mAs",
    "q_model_axis_mA_s", "q_req_shadow_mA_s", "q1_shadow_valid",
    "q1_shadow_invalid_reason", "q1_shadow_invalid_reason_code",
]


def write_q1_shadow_events(metadata: dict, out_dir: Path) -> int:
    """Write direct-video Q1 zero-cross shadow records when present."""
    events = metadata.get("q1_shadow_events")
    if not isinstance(events, list):
        return 0
    with (out_dir / "q1_shadow_events.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=Q1_SHADOW_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for event in events:
            if isinstance(event, dict):
                writer.writerow({name: event.get(name) for name in Q1_SHADOW_COLUMNS})
    return len(events)

Q_IDENT_COLUMNS = [
    "q_ident_event_index", "q_ident_run_schedule_id", "zero_cross_time_ms",
    "zero_cross_rate_dps", "zero_cross_abs_rate_dps", "interpolated_zero_cross_time_ms",
    "interpolated_physical_roll_rate_dps", "detector_crossing_direction",
    "detector_angle_before_deg", "detector_angle_after_deg", "sign_gate_passed",
    "physical_roll_abs_diag_deg", "current_roll_deg", "physical_roll_rate_dps",
    "physical_next_peak_side", "side_occurrence_index", "q_ident_armed",
    "arm_consecutive_count", "q_schedule_target_mA_s", "q_command_direction",
    "command_current_mA", "pulse_width_ms", "q_target_mA_s", "q_effective_pred_mA_s",
    "vbat_mV", "i0_estimated_mA", "solver_required_width_ms",
    "solver_selected_integer_width_ms", "pulse_width_guard_max_ms",
    "pulse_start_ms", "pulse_end_ms", "q_ident_valid", "q_ident_invalid_reason",
    "q_ident_invalid_reason_code",
]


def write_q_ident_events(metadata: dict, out_dir: Path) -> int:
    """Write every fixed-Q identification event, including invalid/no-pulse events."""
    events = metadata.get("q_ident_events")
    if not isinstance(events, list):
        return 0
    with (out_dir / "q_ident_events.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=Q_IDENT_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for event in events:
            if isinstance(event, dict):
                writer.writerow({name: event.get(name) for name in Q_IDENT_COLUMNS})
    return len(events)


ENERGY_CONTROL_V0_COLUMNS = [
    "event_index", "zero_cross_time_ms", "zero_cross_rate_dps", "zero_cross_abs_rate_dps",
    "physical_next_peak_side", "output_gate_state", "previous_accepted_physical_next_peak_side",
    "candidate_physical_next_peak_side", "rearm_excursion_seen",
    "maximum_excursion_since_previous_cross_deg", "rearm_threshold_deg",
    "side_alternation_passed", "output_authorized", "output_blocked_reason",
    "output_blocked_reason_code", "q_command_direction", "passive_next_peak_abs_deg",
    "passive_energy_j", "target_peak_abs_deg", "target_energy_j", "delta_energy_required_j",
    "q1_gain_deg_per_mA_s", "q_selected_mA_s", "q_effective_pred_mA_s",
    "predicted_next_peak_abs_deg", "predicted_next_energy_j", "q_saturated_at_max",
    "rate_support_status", "q_support_status", "vbat_mV", "i0_estimated_mA",
    "solver_required_width_ms", "solver_selected_integer_width_ms", "command_current_mA",
    "pulse_width_ms", "pulse_start_ms", "pulse_end_ms", "output_executed", "valid",
    "reason", "reason_code",
]


def write_energy_control_v0_events(metadata: dict, out_dir: Path) -> int:
    """Write every accepted Energy Control V0 zero-cross decision."""
    events = metadata.get("energy_control_v0_events")
    if not isinstance(events, list):
        return 0
    with (out_dir / "energy_control_v0_events.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=ENERGY_CONTROL_V0_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for event in events:
            if isinstance(event, dict):
                writer.writerow({name: event.get(name) for name in ENERGY_CONTROL_V0_COLUMNS})
    return len(events)

ENERGY_CONTROL_AUTONOMOUS_PEAK_COLUMNS = [
    "cg_shift_mm",
    "peak_index", "peak_time_ms", "physical_peak_side", "peak_amplitude_deg",
    "target_peak_deg", "peak_error_deg", "phase", "integral_plus_mA_s",
    "integral_minus_mA_s", "first_peak", "pending_command_matched",
    "pending_q_command_mA_s", "antiwindup_upper_hold", "antiwindup_lower_hold",
]

ENERGY_CONTROL_AUTONOMOUS_ZERO_CROSS_COLUMNS = [
    "cg_shift_mm",
    "event_index", "event_kind", "zero_cross_time_ms", "sway_trim_before_ms", "sway_trim_cap_ms", "fast_trim_before_ms", "fast_trim_cap_ms", "zero_cross_rate_dps",
    "zero_cross_abs_rate_dps", "previous_peak_time_ms", "previous_peak_side",
    "previous_peak_amplitude_deg", "physical_next_peak_side", "phase",
    "free_next_peak_amplitude_deg", "free_model_revision", "passive_energy_j",
    "p1_free_peak_before_rate_deg", "rate_baseline_peak_deg",
    "rate_baseline_correction_deg", "rate_baseline_reason",
    "target_peak_deg", "target_energy_j", "delta_energy_required_j",
    "q1_gain_deg_per_mA_s", "q_ff_energy_mA_s", "q_angle_diagnostic_mA_s",
    "integral_side_mA_s", "q_unclamped_mA_s", "q_command_mA_s",
    "q_effective_pred_mA_s", "q_available_mA_s", "q_gain_extrapolated",
    "a_pred_base_deg", "a_pred_corrected_deg", "correction_deg", "c_side_used_deg",
    "g_side_base_deg_per_mA_s", "g_side_corrected_deg_per_mA_s", "correction_blend_lambda",
    "predicted_next_peak_amplitude_deg", "q_saturated_upper", "q_saturated_lower",
    "q_command_direction", "command_matches_zero_cross_motion", "vbat_mV",
    "i0_estimated_mA", "pre_input_capture_time_us", "pre_input_measured_current_mA",
    "pre_input_current_sample_time_us", "pre_input_current_age_us", "pre_input_current_valid",
    "pre_input_wheel_speed_rpm", "pre_input_wheel_speed_sample_time_us",
    "pre_input_wheel_speed_age_us", "pre_input_wheel_speed_valid",
    "solver_required_width_ms", "solver_selected_integer_width_ms",
    "command_current_mA", "pulse_width_ms", "pulse_start_ms", "pulse_end_ms",
    "output_executed", "valid", "reason", "reason_code",
    "kinetic_energy_j", "predicted_loss_j", "integral_energy_j",
    "commanded_energy_deficit_j", "unclamped_energy_request_j",
    "maximum_input_work_j", "maximum_effective_arrival_energy_increment_j",
    "unapplied_energy_request_j", "corrected_target_energy_j",
    "unclamped_inverse_valid", "predicted_energy_j", "prediction_bias_j",
    "predicted_observed_energy_j", "predicted_observed_next_peak_deg",
    "energy_model_extrapolated", "rate_support_diagnostic",
    "arrival_energy_model_revision", "work_control_branch",
    "work_prediction_cross_condition", "work_brake_cap_ms", "work_bias_samples", "work_model_weight",
    "predicted_wheel_work_j", "predicted_input_work_j",
    "predicted_post_input_work_j", "predicted_work_change_j",
    "foot_span_left_deg", "foot_span_right_deg", "foot_span_trimmed_left_deg", "foot_span_trimmed_right_deg",
    "foot_span_start_ms", "foot_span_end_ms", "foot_span_baseline_ms", "foot_span_requested_ms",
    "wheel_guard_legacy_ms", "wheel_guard_extra_ms", "wheel_guard_trial_eligible",
    "weak_probe_baseline_ms", "weak_probe_requested_ms", "weak_probe_executed",
    "long_foot_counterfactual_ms", "long_foot_actual_ms", "long_foot_effective",
    "negative_recovery_baseline_ms", "negative_recovery_candidate_ms", "negative_recovery_executed", "negative_recovery_state",
    "positive_sway_probe_baseline_ms", "positive_sway_probe_candidate_ms", "positive_sway_probe_executed",
    "foot_span_state", "foot_span_accepted_cycles", "foot_span_valid", "foot_span_selected",
    "foot_span_startup", "foot_span_left_low", "foot_span_right_low", "foot_span_left_both",
    "foot_span_right_both", "foot_span_continuous", "foot_span_left_outlier", "foot_span_right_outlier",
    "foot_span_left_edge_unverified", "foot_span_right_edge_unverified",
]


def write_energy_control_autonomous_events(metadata: dict, out_dir: Path) -> tuple[int, int]:
    """Write V1 autonomous Energy Control peak and zero-cross decision tables."""
    peaks = metadata.get("energy_control_autonomous_peak_events")
    zeros = metadata.get("energy_control_autonomous_zero_cross_events")
    peak_count = 0
    zero_count = 0
    if isinstance(peaks, list):
        with (out_dir / "energy_control_autonomous_peak_events.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=ENERGY_CONTROL_AUTONOMOUS_PEAK_COLUMNS, extrasaction="ignore")
            writer.writeheader()
            for event in peaks:
                if isinstance(event, dict):
                    writer.writerow({name: event.get(name) for name in ENERGY_CONTROL_AUTONOMOUS_PEAK_COLUMNS})
                    peak_count += 1
    if isinstance(zeros, list):
        with (out_dir / "energy_control_autonomous_zero_cross_events.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=ENERGY_CONTROL_AUTONOMOUS_ZERO_CROSS_COLUMNS, extrasaction="ignore")
            writer.writeheader()
            for event in zeros:
                if isinstance(event, dict):
                    if metadata.get("firmware_revision") in ("1.0.95-positive-sway-probe", "1.0.96-tapered-amplitude-recovery", "1.0.97-paired-sway-rollback", "1.0.98-foot-observation-timeout", "1.0.99-alternate-foot-trial") and int(event.get("foot_span_flags", 0)) & (1 << 14):
                        event = dict(event)
                        event["positive_sway_probe_baseline_ms"] = event.pop("long_foot_counterfactual_ms", None)
                        event["positive_sway_probe_candidate_ms"] = event.pop("long_foot_actual_ms", None)
                        event["positive_sway_probe_executed"] = event.pop("long_foot_effective", None)
                    elif metadata.get("firmware_revision") in ("1.0.91-two-cycle-recovery", "1.0.92-paired-foot-sequence", "1.0.93-blocked-candidate-retry", "1.0.94-foot-trial-loss", "1.0.95-positive-sway-probe", "1.0.96-tapered-amplitude-recovery", "1.0.97-paired-sway-rollback", "1.0.98-foot-observation-timeout", "1.0.99-alternate-foot-trial") and event.get("physical_next_peak_side") == -1:
                        event = dict(event)
                        event["negative_recovery_baseline_ms"] = event.pop("long_foot_counterfactual_ms", None)
                        event["negative_recovery_candidate_ms"] = event.pop("long_foot_actual_ms", None)
                        event["negative_recovery_executed"] = event.pop("long_foot_effective", None)
                        event["negative_recovery_state"] = (int(event.get("foot_span_flags", 0)) >> 12) & 3
                    writer.writerow({name: event.get(name) for name in ENERGY_CONTROL_AUTONOMOUS_ZERO_CROSS_COLUMNS})
                    zero_count += 1
    return peak_count, zero_count

FOOT_COLUMNS = ["sequence", "run_id", "frame_us", "delivered_us", "control_observed_us", "log_time_us",
    "measurement_time_us", "imu_sample_us", "state_id", "led_state", "sync_event_id",
    "processing_us", "timestamp_valid", "frame_valid", "zero_ready", "right_valid",
    "left_valid", "right_in_range", "left_in_range", "right_x", "left_x", "right_deg",
    "left_deg", "right_contrast", "left_contrast", "right_scan_y", "left_scan_y",
    "right_weight", "left_weight", "right_reason", "left_reason", "right_templates", "left_templates",
    "right_candidates", "left_candidates", "right_ambiguity", "left_ambiguity", "zero_reason"]

def write_foot_frames(metadata: dict, out_dir: Path) -> int:
    rows = metadata.get("foot_frames")
    if not isinstance(rows, list):
        return 0
    with (out_dir / "foot_angles.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FOOT_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in FOOT_COLUMNS})
    (out_dir / "foot_calibration.json").write_text(
        json.dumps(metadata.get("foot_observation", {}), indent=2), encoding="utf-8")
    return len(rows)


def write_cad_support_shadow(metadata: dict, out_dir: Path) -> int:
    block = metadata.get('cad_support_shadow')
    if block is None:
        return 0
    if not isinstance(block, dict) or block.get('output_enabled') is not False:
        raise ValueError('Invalid CAD shadow metadata')
    rows = block.get('records')
    if not isinstance(rows, list):
        raise ValueError('Invalid CAD shadow records')
    frames = {f['sequence']: f for f in metadata.get('foot_frames', [])}
    seen = set()
    joined = []
    for row in rows:
        seq = row.get('sequence')
        if seq not in frames or seq in seen:
            raise ValueError('Unmatched or repeated CAD shadow sequence')
        seen.add(seq)
        f = frames[seq]
        joined.append({**{k: f.get(k) for k in ('run_id', 'frame_us', 'delivered_us', 'log_time_us', 'measurement_time_us', 'left_deg', 'right_deg')}, **row})
    if joined:
        with (out_dir / 'cad_support_shadow.csv').open('w', newline='', encoding='utf8') as f:
            writer = csv.DictWriter(f, fieldnames=list(joined[0]))
            writer.writeheader()
            writer.writerows(joined)
    return len(joined)

def write_foot_support_diagnostics(metadata: dict, out_dir: Path) -> int:
    frames = {f['sequence']: f for f in metadata.get('foot_frames', [])}
    count = 0
    for name in ('foot_pose_prediction', 'foot_support_assist'):
        block = metadata.get(name)
        if block is None:
            continue
        if not isinstance(block, dict) or not isinstance(block.get('records'), list):
            raise ValueError('Invalid foot support diagnostics')
        if name == 'foot_support_assist' and block.get('output_enabled') is not True:
            raise ValueError('Unspecified foot support output policy')
        joined, seen = [], set()
        for row in block['records']:
            seq = row.get('sequence')
            key = seq if name == 'foot_pose_prediction' else (row.get('time_ms'), row.get('peak_ms'))
            if seq not in frames or key in seen:
                raise ValueError('Unmatched or repeated foot support diagnostic')
            seen.add(key)
            frame = frames[seq]
            joined.append({**{k: frame.get(k) for k in ('run_id', 'frame_us', 'delivered_us', 'log_time_us', 'measurement_time_us')}, **row})
        if joined:
            with (out_dir / (name + '.csv')).open('w', newline='', encoding='utf8') as f:
                writer = csv.DictWriter(f, fieldnames=list(joined[0]))
                writer.writeheader()
                writer.writerows(joined)
        count += len(joined)
    return count


def convert(path: Path, out_dir: Path) -> None:
    data = path.read_bytes()
    header = parse_header(data)
    if header["format_version"] not in (23, 24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56):
        raise ValueError(f"this converter expects rwlog format v23-v27, v29-v56, got v{header['format_version']}")
    sample_format = sample_format_for_version(header["format_version"])
    if header["log_sample_size"] != struct.calcsize(sample_format):
        raise ValueError("unexpected sample size")
    crc_ok = verify_crc(data, header)
    if not crc_ok:
        raise ValueError("RWLOG CRC mismatch; refusing to convert corrupted data")
    if header["crc_offset"] + 4 != len(data):
        raise ValueError("unexpected trailing bytes")
    metadata_end = header["header_size"] + header["metadata_json_size"]
    samples_end = header["samples_offset"] + header["sample_count"] * header["log_sample_size"]
    if metadata_end != header["samples_offset"] or samples_end > header["crc_offset"]:
        raise ValueError("inconsistent metadata/sample offsets")

    out_dir.mkdir(parents=True, exist_ok=True)
    metadata_start = header["header_size"]
    metadata_end = metadata_start + header["metadata_json_size"]
    metadata = expand_tables(json.loads(data[metadata_start:metadata_end].decode("utf-8")))
    (out_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    foot_count = write_foot_frames(metadata, out_dir)
    cad_count = write_cad_support_shadow(metadata, out_dir)
    support_count = write_foot_support_diagnostics(metadata, out_dir)
    e2_shadow_peak_count = write_e2_shadow_peaks(metadata, out_dir)
    q1_shadow_event_count = write_q1_shadow_events(metadata, out_dir)
    q_ident_event_count = write_q_ident_events(metadata, out_dir)
    energy_control_v0_event_count = write_energy_control_v0_events(metadata, out_dir)
    autonomous_peak_count, autonomous_zero_cross_count = write_energy_control_autonomous_events(metadata, out_dir)

    with (out_dir / "header.json").open("w", encoding="utf-8") as f:
        json.dump({k: v for k, v in header.items() if not k.startswith("reserved")}, f, indent=2)
        f.write("\n")

    sample_size = header["log_sample_size"]
    sample_offset = header["samples_offset"]
    csv_columns = csv_columns_for_version(header["format_version"])
    with (out_dir / "timeseries.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns)
        writer.writeheader()
        for i in range(header["sample_count"]):
            offset = sample_offset + i * sample_size
            values = struct.unpack_from(sample_format, data, offset)
            writer.writerow(convert_sample(values, header["format_version"], metadata.get("magnetometer", {}).get("factory_trim")))

    print(f"format_version={header['format_version']}")
    print(f"samples={header['sample_count']}")
    print(f"crc_ok={crc_ok}")
    if foot_count:
        print(f"foot_frames={foot_count}")
    if cad_count:
        print(f"cad_support_shadow_records={cad_count}")
    if support_count:
        print(f"foot_support_diagnostic_records={support_count}")
    if e2_shadow_peak_count:
        print(f"e2_shadow_peak_events={e2_shadow_peak_count}")
    if q1_shadow_event_count:
        print(f"q1_shadow_events={q1_shadow_event_count}")
    if q_ident_event_count:
        print(f"q_ident_events={q_ident_event_count}")
    if energy_control_v0_event_count:
        print(f"energy_control_v0_events={energy_control_v0_event_count}")
    if autonomous_peak_count:
        print(f"energy_control_autonomous_peak_events={autonomous_peak_count}")
    if autonomous_zero_cross_count:
        print(f"energy_control_autonomous_zero_cross_events={autonomous_zero_cross_count}")
    print(f"wrote={out_dir}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert supported RWLOG v23-v56 files to CSV, including control and diagnostic metadata events.")
    parser.add_argument("rwlog", type=Path)
    parser.add_argument("--out", type=Path, default=Path("converted_dynamic_beta_hold73_tau73_compare"))
    args = parser.parse_args()
    try:
        convert(args.rwlog, args.out)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

