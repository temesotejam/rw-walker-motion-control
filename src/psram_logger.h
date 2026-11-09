#pragma once

#include <Arduino.h>
#include "psram_string.h"
#include <math.h>

#include "log_types.h"
#include "solver_audit.h"
#include "upright_reference.h"
#include "lateral_cg_energy.h"
#include "adaptive_lateral_cg.h"

class PsramLogger {
 public:
  struct IdentificationEvent {
    uint16_t event_id = 0; uint32_t start_ms = 0; uint32_t peak_ms = 0;
    int16_t theta0_cdeg = 0; int16_t rate0_cdps = 0; int16_t theta_peak_cdeg = 0; int16_t i0_mA = 0;
    int8_t direction = 0; uint16_t vbat_mV = 0; uint16_t pulse_width_ms = 0;
    int32_t q_target_mAms = 0; int32_t q_requested_mAms = 0;
    int32_t q_command_mAms = 0; int32_t q_estimated_mAms = 0;
    int16_t a_target_cdeg = 0; int16_t a_pred_cdeg = 0; int16_t a_pred_shadow_cdeg = 0;
    int16_t a_pred_calibrated_cdeg = LOG_NAN_I16; int16_t prev_peak_abs_cdeg = LOG_NAN_I16;
    int32_t q_effective_pred_mAms = 0; int8_t next_peak_side = 0;
    int16_t a_min_cdeg = 0; int16_t a_max_cdeg = 0;
    bool bootstrap = false; bool target_reachable = false; bool calibrated_prediction_valid = false;
    bool pulse_suppressed = false; bool peak_detected = false;
  };
  struct CalibrationPeakEvent {
    uint16_t peak_index = 0; uint32_t candidate_peak_ms = 0; uint32_t confirmed_ms = 0;
    int8_t peak_side = 0; uint8_t phase = 0;
    int16_t peak_abs_cdeg = 0; int16_t prev_peak_abs_cdeg = LOG_NAN_I16;
    // Candidate extrema are found from dynamic_hold073. The fixed-b100 values
    // are sampled at that same physical candidate time, never independently retimed.
    int16_t peak_signed_dynamic_cdeg = LOG_NAN_I16;
    int16_t prev_peak_signed_dynamic_cdeg = LOG_NAN_I16;
    int16_t center_dynamic_cdeg = LOG_NAN_I16;
    int16_t half_range_dynamic_cdeg = LOG_NAN_I16;
    int16_t peak_signed_fixed_cdeg = LOG_NAN_I16;
    int16_t prev_peak_signed_fixed_cdeg = LOG_NAN_I16;
    int16_t center_fixed_cdeg = LOG_NAN_I16;
    int16_t half_range_fixed_cdeg = LOG_NAN_I16;
    int32_t q_effective_pred_mAms = 0;
  };
  struct CalibrationBuildUpEvent {
    uint16_t pulse_index = 0; uint32_t start_ms = 0;
    int16_t theta0_cdeg = 0; int16_t rate0_cdps = 0;
    int8_t direction = 0; uint8_t phase = 0; uint16_t vbat_mV = 0; uint16_t pulse_width_ms = 0;
    int16_t target_peak_cdeg = 0;
    int32_t q_required_mAms = 0; int32_t q_command_mAms = 0; int32_t q_effective_mAms = 0;
    int16_t predicted_peak_cdeg = 0;
    bool q_cap_limited = false; bool guard_limited = false; bool width_limited = false;
  };
  struct CalibrationProbeEvent {
    uint8_t probe_index = 0; uint8_t planned_probe_index = 0; uint8_t q_level_index = 0;
    uint8_t q_probe_schedule_id = 0;
    uint8_t polarity_mode = 0; uint8_t rebuild_count = 0; uint8_t rebuild_episode_id = 0;
    uint8_t rebuild_attempt_in_episode = 0; uint8_t wait_halfcycle_count = 0;
    // v50: source that admitted the Q predecessor (0=A input-domain, 1=dynamic H).
    uint8_t rebuild_entry_source = 0; uint8_t result_code = 0; uint32_t pulse_start_ms = 0;
    uint32_t candidate_peak_ms = 0; uint32_t confirmed_ms = 0;
    uint32_t previous_peak_candidate_ms = 0;
    int8_t requested_side = 0; int8_t predecessor_side = 0; int8_t observed_side = 0; int8_t pulse_direction = 0;
    uint16_t pulse_id = 0; uint16_t pulse_width_ms = 0;
    int16_t command_dynamic_angle_cdeg = LOG_NAN_I16;
    int16_t command_rate_raw_cdps = LOG_NAN_I16;
    int16_t command_rate_bias_corrected_cdps = LOG_NAN_I16;
    int16_t previous_peak_abs_cdeg = LOG_NAN_I16; int16_t observed_peak_abs_cdeg = LOG_NAN_I16;
    int16_t previous_peak_signed_dynamic_cdeg = LOG_NAN_I16;
    int16_t observed_peak_signed_dynamic_cdeg = LOG_NAN_I16;
    int16_t center_dynamic_cdeg = LOG_NAN_I16;
    int16_t half_range_previous_dynamic_cdeg = LOG_NAN_I16;
    int16_t half_range_observed_dynamic_cdeg = LOG_NAN_I16;
    int16_t half_range_free_pred_dynamic_cdeg = LOG_NAN_I16;
    int16_t delta_half_range_dynamic_cdeg = LOG_NAN_I16;
    int16_t gain_half_range_dynamic_cdeg_per_mAs = LOG_NAN_I16;
    int16_t previous_peak_signed_fixed_cdeg = LOG_NAN_I16;
    int16_t observed_peak_signed_fixed_cdeg = LOG_NAN_I16;
    int16_t center_fixed_cdeg = LOG_NAN_I16;
    int16_t half_range_previous_fixed_cdeg = LOG_NAN_I16;
    int16_t half_range_observed_fixed_cdeg = LOG_NAN_I16;
    int16_t half_range_free_pred_fixed_cdeg = LOG_NAN_I16;
    int16_t delta_half_range_fixed_cdeg = LOG_NAN_I16;
    int16_t gain_half_range_fixed_cdeg_per_mAs = LOG_NAN_I16;
    int32_t q_target_mAms = 0; int32_t q_effective_pred_mAms = 0; int16_t gain_cdeg_per_mAs = LOG_NAN_I16;
    // v53 inverse-shadow audit. Forward-model state is captured before the
    // physical Q pulse. Every v53 field below is log-only.
    int16_t shadow_hprev_imu_cdeg = LOG_NAN_I16;
    int16_t shadow_c_prev_imu_cdeg = LOG_NAN_I16;
    int16_t shadow_rate_abs_cdps = LOG_NAN_I16;
    int8_t shadow_next_peak_side = 0;
    float shadow_q_effective_pred_mA_s = NAN;
    float shadow_delta_h_video_pred_deg = NAN;
    float shadow_h_free_imu_pred_deg = NAN;
    // v53 absolute predictions are in the video half-range coordinate.
    float shadow_h_free_video_pred_deg = NAN;
    float shadow_h_post_pred_deg = NAN;  // actual selected-Q diagnostic
    float shadow_h_post_pred_q0_deg = NAN;
    float shadow_h_post_pred_q05_deg = NAN;
    float shadow_h_post_pred_q10_deg = NAN;
    float shadow_h_post_pred_q15_deg = NAN;
    float shadow_h_ref_deg = NAN;
    float shadow_q_req_raw_mA_s = NAN;
    float shadow_q_req_clamped_mA_s = NAN;
    // v54 log-only target reachability and recommended action. These values
    // never feed the physical Q selector, pulse width, or motor command.
    float shadow_h_supported_min_pred_deg = NAN;
    float shadow_h_supported_max_pred_deg = NAN;
    float shadow_recommended_q_mA_s = NAN;
    // v57 keeps the existing video-coordinate forward prediction, then logs
    // the on-device dynamic-H proxy separately. The residual is explicitly
    // not a video validation; video is joined only in offline analysis.
    float shadow_delta_h_imu_actual_deg = NAN;
    float shadow_delta_h_pred_minus_imu_dynamic_deg = NAN;
    uint8_t shadow_q_req_region = 0;
    uint8_t shadow_q_req_reason = 0;
    uint8_t shadow_reachability_reason = 0;
    uint8_t shadow_recommended_action = 0;
    uint8_t shadow_q_support_reason = 0;
    uint8_t shadow_v57_q_req_reason = 0;
    uint8_t shadow_v58_repeatability_gate_reason = 0;
    // V59 is an online state-admission record for the fixed-Q probe; the
    // inverse-Q shadow fields above remain diagnostic only.
    uint8_t v59_block_index = 0; uint8_t v59_block_order = 0;
    uint8_t v59_state_gate_reason = 0;
    bool post_pulse = false; bool in_support = false; bool side_matched = false;
    bool direction_matches_base = false; bool positive_gain = false; bool gain_stored = false;
    bool half_range_dynamic_in_support = false; bool half_range_fixed_in_support = false;
    bool rebuild_target_reached = false; bool dynamic_h_in_support_at_command = false;
    bool shadow_state_valid = false; bool shadow_state_in_support = false;
    bool shadow_q_in_model_support = false; bool shadow_delta_h_valid = false;
    bool shadow_h_free_valid = false; bool shadow_h_free_video_valid = false;
    bool shadow_h_post_valid = false; bool shadow_h_post_candidates_valid = false;
    bool shadow_q0_extrapolated = false; bool shadow_future_control_eligible = false;
    bool shadow_control_candidate = false; bool shadow_q_req_valid = false;
    bool shadow_href_in_current_control_candidate_reachable_range = false;
    bool shadow_v58_repeatability_gate_passed = false;
    bool v59_state_gate_passed = false;
    bool shadow_reachability_valid = false;
    bool shadow_h_in_support = false;
    bool shadow_rate_in_support = false;
    bool shadow_q_req_in_model_support = false;
    bool shadow_q_req_in_control_candidate_range = false;
    bool shadow_q_req_candidate_valid = false;
    bool shadow_imu_proxy_residual_valid = false;
  };
  struct CalibrationStateGateEvent {
    uint8_t event_index = 0; uint8_t planned_probe_index = 0; uint8_t q_level_index = 0;
    uint8_t reason = 0; uint8_t action = 0; uint8_t wait_halfcycle_count = 0;
    uint8_t rebuild_total_count = 0; uint32_t crossing_ms = 0;
    int16_t hprev_cdeg = LOG_NAN_I16; int16_t cprev_cdeg = LOG_NAN_I16;
    int16_t rate_abs_cdps = LOG_NAN_I16; int8_t next_peak_side = 0;
    bool state_valid = false; bool dynamic_h_in_support = false;
    // V60 transition diagnostics. A cooldown is an intentionally unforced
    // candidate; skipped_by_decay means the same desired arrival was observed
    // high and then low without an accepted gate between them.
    bool v60_cooldown_free_decay = false; bool v60_gate_skipped_by_decay = false;
  };
  // A peak-level record made only during passive capture. Q_req_shadow is an
  // effective-Q request in mA*s; no field in this struct can command output.
  struct E2ShadowPeakEvent {
    uint16_t event_index = 0;
    bool e2_armed = false;
    uint32_t release_detected_ms = 0;
    uint16_t turn_index = 0;
    uint32_t candidate_peak_ms = 0;
    uint32_t confirmed_ms = 0;
    int8_t turn_side_from_rate = 0;
    bool alternating_side_ok = false;
    // Legacy aliases now refer to the gyro-integrated absolute diagnostic.
    float previous_peak_deg = NAN;
    float current_peak_deg = NAN;
    float halfcycle_gyro_integral_deg = NAN;
    float h_prev_gyro_deg = NAN;
    float h_prev_deg = NAN;
    float h_next_e2_deg = NAN;
    float h_next_e2_explicit_deg = NAN;
    float static_anchor_abs_deg = NAN;
    float peak_gyro_integrated_abs_deg = NAN;
    float peak_accel_abs_diag_deg = NAN;
    // Absolute-peak / Q fields are retained as null diagnostics until the
    // gyro absolute estimator has been validated against fixed-horizon video.
    float passive_next_peak_deg = NAN;
    int8_t physical_next_peak_side = 0;
    float target_peak_deg = NAN;
    float delta_a_required_deg = NAN;
    float shadow_gain_deg_per_mA_s = NAN;
    float q_req_shadow_mA_s = NAN;
    bool shadow_valid = false;
    uint8_t shadow_invalid_reason = 0;
  };
  // One direct Q1 calculation made at an accepted zero-cross during passive
  // capture. It is a motor-OFF audit record; Q_req is never an actuator input.
  struct Q1ShadowEvent {
    uint16_t q1_shadow_event_index = 0;
    uint32_t zero_cross_time_ms = 0;
    float zero_cross_rate_dps = NAN;
    float zero_cross_abs_rate_dps = NAN;
    int8_t physical_next_peak_side = 0;
    // Detector and linear-interpolation diagnostics. The Q1 calculation keeps
    // using zero_cross_time_ms and zero_cross_rate_dps above.
    int8_t detector_crossing_direction = 0;
    float detector_angle_before_deg = NAN;
    float detector_angle_after_deg = NAN;
    float crossing_interpolation_alpha = NAN;
    uint32_t interpolated_zero_cross_time_ms = 0;
    float physical_roll_rate_before_dps = NAN;
    float physical_roll_rate_after_dps = NAN;
    float interpolated_physical_roll_rate_dps = NAN;
    bool sign_gate_passed = false;
    float q1_intercept_deg = NAN;
    float q1_rate_term_deg = NAN;
    float q1_side_term_deg = NAN;
    float q1_baseline_next_peak_abs_deg = NAN;
    float target_next_peak_abs_deg = NAN;
    float delta_peak_required_deg = NAN;
    float q1_gain_deg_per_mA_s = NAN;
    // Same coordinate as canonical `q_target_mA_s`; logged even if invalid.
    float q_model_axis_mA_s = NAN;
    float q_req_shadow_mA_s = NAN;
    bool q1_shadow_valid = false;
    uint8_t q1_shadow_invalid_reason = 0;
  };
  // Every accepted Q_IDENT zero-cross is retained, including both arming
  // crossings, support exclusions, Q=0 baselines, and failed output attempts.
  struct QIdentEvent {
    uint16_t q_ident_event_index = 0;
    uint8_t q_ident_run_schedule_id = 0;  // 1--4.
    uint32_t zero_cross_time_ms = 0;
    float zero_cross_rate_dps = NAN;
    float zero_cross_abs_rate_dps = NAN;
    uint32_t interpolated_zero_cross_time_ms = 0;
    float interpolated_physical_roll_rate_dps = NAN;
    int8_t detector_crossing_direction = 0;
    float detector_angle_before_deg = NAN;
    float detector_angle_after_deg = NAN;
    bool sign_gate_passed = false;
    // Dynamic accelerometer angle is diagnostic only; video remains the
    // absolute-angle teacher for later energy-ready analysis.
    float physical_roll_abs_diag_deg = NAN;
    float current_roll_deg = NAN;
    float physical_roll_rate_dps = NAN;
    int8_t physical_next_peak_side = 0;
    uint8_t side_occurrence_index = 0;
    bool q_ident_armed = false;
    uint8_t arm_consecutive_count = 0;
    float q_schedule_target_mA_s = NAN;
    int8_t q_command_direction = 0;
    int16_t command_current_mA = 0;
    uint16_t pulse_width_ms = 0;
    float q_target_mA_s = NAN;
    float q_effective_pred_mA_s = NAN;
    uint16_t vbat_mV = 0;
    float i0_estimated_mA = NAN;
    float solver_required_width_ms = NAN;
    uint16_t solver_selected_integer_width_ms = 0;
    uint16_t pulse_width_guard_max_ms = 0;
    uint32_t pulse_start_ms = 0;
    uint32_t pulse_end_ms = 0;
    bool q_ident_valid = false;
    uint8_t q_ident_invalid_reason = 0;
  };
  // One row per Q1 detector candidate seen by V0. It records the independent
  // V0.2 output gate before any energy calculation or motor authorization.
  struct EnergyControlV0Event {
    uint16_t event_index = 0;
    uint32_t zero_cross_time_ms = 0;
    float zero_cross_rate_dps = NAN;
    float zero_cross_abs_rate_dps = NAN;
    int8_t physical_next_peak_side = 0;
    uint8_t output_gate_state = 0;
    int8_t previous_accepted_physical_next_peak_side = 0;
    int8_t candidate_physical_next_peak_side = 0;
    bool rearm_excursion_seen = false;
    float maximum_excursion_since_previous_cross_deg = NAN;
    float rearm_threshold_deg = NAN;
    bool side_alternation_passed = false;
    bool output_authorized = false;
    uint8_t output_blocked_reason = 0;
    int8_t q_command_direction = 0;
    float passive_next_peak_abs_deg = NAN;
    float passive_energy_j = NAN;
    float target_peak_abs_deg = NAN;
    float target_energy_j = NAN;
    float delta_energy_required_j = NAN;
    float q1_gain_deg_per_mA_s = NAN;
    float q_selected_mA_s = NAN;
    float q_effective_pred_mA_s = NAN;
    float predicted_next_peak_abs_deg = NAN;
    float predicted_next_energy_j = NAN;
    bool q_saturated_at_max = false;
    uint8_t rate_support_status = 0;  // 0=below, 1=within, 2=above.
    uint8_t q_support_status = 0;     // 0=zero, 1=below, 2=within, 3=above.
    uint16_t vbat_mV = 0;
    float i0_estimated_mA = NAN;
    float solver_required_width_ms = NAN;
    uint16_t solver_selected_integer_width_ms = 0;
    int16_t command_current_mA = 0;
    uint16_t pulse_width_ms = 0;
    uint32_t pulse_start_ms = 0;
    uint32_t pulse_end_ms = 0;
    bool output_executed = false;
    bool valid = false;
    uint8_t reason = 0;
  };
  // V4 records the qualified detector extremum and its following central-passage decision.`r`n  // Event-store overflow is diagnostic-only; motor control continues.
  struct EnergyControlAutonomousPeakEvent {
    float cg_shift_mm = 0;
    uint16_t peak_index = 0;
    uint32_t peak_time_ms = 0;
    int8_t physical_peak_side = 0;
    float peak_amplitude_deg = NAN;
    float detector_peak_angle_deg = NAN;
    float target_peak_deg = NAN;
    float peak_error_deg = NAN;
    float target_energy_j=NAN, measured_energy_j=NAN, energy_error_j=NAN;
    float integral_plus_j=0, integral_minus_j=0;
    uint8_t phase = 0;
    float integral_plus_mA_s = 0.0f;
    float integral_minus_mA_s = 0.0f;
    bool first_peak = false;
    bool pending_command_matched = false;
    float pending_q_command_mA_s = NAN;
    float model_residual_j = NAN;  // measured peak minus uncorrected model prediction
    bool model_residual_observed = false;
    // Snapshot at confirmation, not at the earlier geometric extremum.
    uint32_t peak_telemetry_capture_time_ms = 0;
    float peak_wheel_speed_rpm = NAN, peak_measured_current_mA = NAN;
    uint32_t peak_speed_age_us = UINT32_MAX, peak_current_age_us = UINT32_MAX;
    bool peak_speed_valid = false, peak_current_valid = false;
    bool antiwindup_upper_hold = false;
    bool antiwindup_lower_hold = false;
  };
  struct EnergyControlAutonomousZeroCrossEvent {
    float cg_shift_mm = 0;
    uint16_t event_index = 0;
    uint8_t event_kind = 0;  // 0=zero_cross, 1=start_kick, 2=inward_stall_rescue.
    uint32_t zero_cross_time_ms = 0;
    float zero_cross_rate_dps = NAN;
    float zero_cross_abs_rate_dps = NAN;
    float detector_angle_before_deg = NAN;
    float detector_angle_after_deg = NAN;
    float detector_crossing_alpha = NAN;
    float interpolated_crossing_time_ms = NAN;
    bool q_gain_extrapolated = false;
    bool rate_support_diagnostic = false;
    // These guarded pulse widths are capped at 150 ms by tilt_priority.
    uint8_t weak_probe_baseline_ms=0,weak_probe_requested_ms=0;
    float q_available_mA_s = NAN;
    float predicted_energy_j = NAN;
    float prediction_bias_j = 0.0f;
    float predicted_observed_energy_j = NAN;
    float predicted_observed_next_peak_deg = NAN;
    int8_t adaptation_mode = 0;
    uint8_t wheel_speed_bin = 255;
    uint8_t adaptation_consistent_samples = 0;
    uint32_t previous_peak_time_ms = 0;
    int8_t previous_peak_side = 0;
    float previous_peak_amplitude_deg = NAN;
    int8_t physical_next_peak_side = 0;
    bool side_mismatch_diagnostic = false;
    uint8_t phase = 0;
    float free_next_peak_amplitude_deg = NAN;
    float p1_free_peak_before_rate_deg = NAN;
    float rate_baseline_peak_deg = NAN;
    float rate_baseline_correction_deg = NAN;  // retired P1-difference column
    uint8_t rate_baseline_reason = 255;
    float passive_energy_j = NAN;
    float target_peak_deg = NAN;
    float foot_balance_target_deg = NAN, foot_balance_left_deg = NAN, foot_balance_right_deg = NAN;
    float foot_balance_left_mean = NAN, foot_balance_right_mean = NAN;
    float startup_target_deg=NAN;
    uint16_t startup_positive_cap_ms=0;
    uint16_t wheel_guard_legacy_ms=0,wheel_guard_extra_ms=0;
    bool wheel_guard_trial_eligible=false;
    bool weak_probe_executed=false;
    float foot_energy_bias=NAN,foot_energy_left_px=NAN,foot_energy_right_px=NAN;
    bool foot_energy_valid=false;uint8_t foot_energy_phase=0,foot_energy_reason=0;
    float low_peak_bias_mj=NAN,low_peak_base_prediction_mj=NAN,low_peak_applied_bias_mj=NAN;
    uint16_t low_peak_base_width_ms=0;bool low_peak_active=false,low_peak_selected=false;
    float foot_span_left_deg=NAN,foot_span_right_deg=NAN,foot_span_trimmed_left_deg=NAN,foot_span_trimmed_right_deg=NAN;
    uint32_t foot_span_start_ms=0,foot_span_end_ms=0;
    uint16_t foot_span_baseline_ms=0,foot_span_requested_ms=0;
    uint8_t foot_span_state=0,foot_span_accepted_cycles=0;
    enum FootSpanFlag : uint16_t {
      Span_valid=uint16_t(1u<<0),
      Span_selected=uint16_t(1u<<1),
      Span_startup=uint16_t(1u<<2),
      Span_left_low=uint16_t(1u<<3),
      Span_right_low=uint16_t(1u<<4),
      Span_left_both=uint16_t(1u<<5),
      Span_right_both=uint16_t(1u<<6),
      Span_continuous=uint16_t(1u<<7),
      Span_left_outlier=uint16_t(1u<<8),
      Span_right_outlier=uint16_t(1u<<9),
      Span_left_edge_unverified=uint16_t(1u<<10),
      Span_right_edge_unverified=uint16_t(1u<<11),
      NegativeRecoveryStateBit0=uint16_t(1u<<12),
      NegativeRecoveryStateBit1=uint16_t(1u<<13),
      PositiveSwayProbe=uint16_t(1u<<14),
    };
    uint16_t foot_span_flags=0;
    void setFootSpanFlag(FootSpanFlag flag,bool value) {
      if(value) foot_span_flags |= static_cast<uint16_t>(flag);
      else foot_span_flags &= static_cast<uint16_t>(~static_cast<uint16_t>(flag));
    }
    bool footSpanFlag(FootSpanFlag flag) const { return (foot_span_flags & static_cast<uint16_t>(flag))!=0; }
    float target_energy_j = NAN;
    float delta_energy_required_j = NAN;
    float kinetic_energy_j=NAN, predicted_loss_j=NAN, integral_energy_j=NAN;
    float commanded_energy_deficit_j=NAN, corrected_target_energy_j=NAN;
    float unclamped_energy_request_j=NAN, maximum_input_work_j=NAN;
    float unapplied_energy_request_j=NAN;
    float predicted_current_goal_mA=NAN, predicted_current_rise_tau_s=NAN;
    float aligned_initial_current_mA=NAN;
    bool unclamped_inverse_valid=false;
    float work_omega_q_coefficient=NAN, work_q2_coefficient=NAN;
    bool energy_model_extrapolated=false;
    uint8_t work_control_branch=0;
    float predicted_wheel_work_j=NAN, predicted_input_work_j=NAN;
    float predicted_post_input_work_j=NAN, predicted_work_change_j=NAN;
    bool work_prediction_cross_condition=false;
    uint16_t work_brake_cap_ms=0;
    uint8_t work_bias_samples=0;
    float work_model_weight=NAN;
    bool mpc_valid=false,mpc_alternate=false;
    float mpc_second_peak_deg=NAN,mpc_speed_proxy_rpm=NAN;
    bool fixed_negative_probe=false;
    uint16_t negative_probe_batch_index = UINT16_MAX;
    uint16_t negative_probe_batch_planned_width_ms = 0;
    bool negative_probe_batch_rate_skipped = false;
    uint16_t positive_current_batch_index = UINT16_MAX;
    int16_t positive_current_batch_planned_mA = 0;
    float q1_gain_deg_per_mA_s = NAN;
    float q_ff_energy_mA_s = NAN;
    float q_angle_diagnostic_mA_s = NAN;
    float integral_side_mA_s = NAN;
    float q_unclamped_mA_s = NAN;
    float q_command_mA_s = NAN;
    float q_effective_pred_mA_s = NAN;
    float predicted_next_peak_amplitude_deg = NAN;
    float a_pred_base_deg = NAN;
    float a_pred_corrected_deg = NAN;
    float correction_deg = NAN;
    float c_side_used_deg = NAN;
    float g_side_base_deg_per_mA_s = NAN;
    float g_side_corrected_deg_per_mA_s = NAN;
    float correction_blend_lambda = NAN;
    bool q_saturated_upper = false;
    bool q_saturated_lower = false;
    int8_t q_command_direction = 0;
    bool command_matches_zero_cross_motion = false;
    uint16_t vbat_mV = 0;
    float i0_estimated_mA = NAN;

    // V46ak observation-only snapshot taken before the pulse command is queued.
    // Fresh speed selects the learned correction cell; current is diagnostic.
    uint32_t pre_input_capture_time_us = 0;
    float pre_input_measured_current_mA = NAN;
    uint32_t pre_input_current_sample_time_us = 0;
    uint32_t pre_input_current_age_us = UINT32_MAX;
    bool pre_input_current_valid = false;
    float pre_input_wheel_speed_rpm = NAN;
    uint32_t pre_input_wheel_speed_sample_time_us = 0;
    uint32_t pre_input_wheel_speed_age_us = UINT32_MAX;
    bool pre_input_wheel_speed_valid = false;

    float solver_required_width_ms = NAN;
    uint16_t solver_selected_integer_width_ms = 0;
    int16_t command_current_mA = 0;
    uint16_t pulse_width_ms = 0;
    uint32_t pulse_start_ms = 0;
    uint32_t pulse_end_ms = 0;
    bool output_executed = false;
    bool valid = false;
    uint8_t reason = 0;
  };  struct CalibrationResult {
    bool enabled = false; bool protocol_complete = false; bool valid = false; uint8_t failure_reason = 0;
    uint8_t probe_plan_count = 0; uint8_t probe_wait_halfcycles = 0;
    uint8_t v59_gate_event_count = 0; uint8_t v59_gate_pass_count = 0;
    uint8_t v59_gate_skip_count = 0; uint8_t v59_rebuild_from_low_state_count = 0;
    bool v60_rebuild_target_valid = false;
    bool v60_rebuild_target_observed = false;
    bool v60_gate_center_in_dynamic_h_input_support = false;
    // V61 records the modeled post-cooldown H/C state, rather than treating a
    // raw rebuild peak as sufficient evidence of gate admission.
    bool v61_state_target_valid = false;
    float v61_predicted_gate_h_deg = 0.0f;
    float v61_predicted_gate_c_deg = 0.0f;
    float v61_feedback_command_peak_deg = 0.0f;
    uint8_t v61_feedback_correction_count = 0;
    bool v62_rate_model_valid = false; bool v62_hc_target_feasible = false;
    bool v62_state_target_valid = false; uint8_t v62_rate_sample_count = 0;
    float v62_rate_a_per_s = 0.0f; float v62_rate_b_per_s = 0.0f;
    float v62_rate_offset_dps = 0.0f; float v62_rate_r2 = 0.0f;
    float v62_target_h_deg = 0.0f; float v62_target_c_deg = 0.0f;
    float v62_target_rate_dps = 0.0f;
    float v62_target_controlled_peak_deg = 0.0f;

    uint8_t v60_free_decay_support_wait_count = 0;
    uint8_t v60_cooldown_free_decay_count = 0;
    uint8_t v60_gate_skipped_by_decay_count = 0;
    float v60_gate_center_h_deg = 0.0f; float v60_gate_center_c_deg = 0.0f;
    float v60_free_decay_after_one_cycle_h_deg = 0.0f;
    float v60_decay_per_cycle_h_deg = 0.0f;
    float v60_rebuild_target_h_deg = 0.0f;
    float v60_rebuild_target_peak_deg = 0.0f;
    uint8_t initial_kick_count = 0;
    uint8_t rebuild_total_count = 0; uint8_t rebuild_episode_id = 0;
    uint8_t rebuild_attempt_in_episode = 0;
    // Selected free-decay model used by the log-only calibrated prediction:
    // 0=none, 1=proportional (r*A), 2=affine (r*A+c).
    uint8_t free_model_pos = 0; uint8_t free_model_neg = 0;
    bool free_halfcycle_monotone_pos = false; bool free_halfcycle_monotone_neg = false;
    bool free_full_cycle_valid_pos = false; bool free_full_cycle_valid_neg = false;
    float free_full_cycle_r_pos = 0.0f; float free_full_cycle_r_neg = 0.0f;
    float free_full_cycle_c_pos_deg = 0.0f; float free_full_cycle_c_neg_deg = 0.0f;
    float free_full_cycle_input_min_pos_deg = 0.0f; float free_full_cycle_input_max_pos_deg = 0.0f;
    float free_full_cycle_input_min_neg_deg = 0.0f; float free_full_cycle_input_max_neg_deg = 0.0f;
    float r_pos = 0.0f; float r_neg = 0.0f; float g_pos = 0.0f; float g_neg = 0.0f;
    float g_pos_stddev = 0.0f; float g_neg_stddev = 0.0f;
    float c_pos_deg = 0.0f; float c_neg_deg = 0.0f;
    uint8_t free_count_pos = 0; uint8_t free_count_neg = 0;
    float free_support_min_pos_deg = 0.0f; float free_support_max_pos_deg = 0.0f;
    float free_support_min_neg_deg = 0.0f; float free_support_max_neg_deg = 0.0f;
    // Domain of A_prev for each selected arrival-side model.
    float free_input_min_pos_deg = 0.0f; float free_input_max_pos_deg = 0.0f;
    float free_input_min_neg_deg = 0.0f; float free_input_max_neg_deg = 0.0f;
    float free_proportional_r_pos = 0.0f; float free_proportional_r_neg = 0.0f;
    float free_proportional_rmse_pos_deg = 0.0f; float free_proportional_rmse_neg_deg = 0.0f;
    float free_proportional_r2_pos = 0.0f; float free_proportional_r2_neg = 0.0f;
    float free_proportional_loocv_rmse_pos_deg = 0.0f; float free_proportional_loocv_rmse_neg_deg = 0.0f;
    bool free_proportional_valid_pos = false; bool free_proportional_valid_neg = false;
    float free_affine_r_pos = 0.0f; float free_affine_r_neg = 0.0f;
    float free_affine_c_pos_deg = 0.0f; float free_affine_c_neg_deg = 0.0f;
    float free_affine_rmse_pos_deg = 0.0f; float free_affine_rmse_neg_deg = 0.0f;
    float free_affine_r2_pos = 0.0f; float free_affine_r2_neg = 0.0f;
    float free_affine_loocv_rmse_pos_deg = 0.0f; float free_affine_loocv_rmse_neg_deg = 0.0f;
    bool free_affine_valid_pos = false; bool free_affine_valid_neg = false;
    uint8_t q_count_pos = 0; uint8_t q_count_neg = 0;
    float q_used_pos_mA_s = 0.0f; float q_used_neg_mA_s = 0.0f;
    // v49 shadow-only full-half-cycle range models. These never participate
    // in pulse choice, safety guards, or calibration_valid.
    uint8_t half_range_free_count = 0;
    float half_range_dynamic_r = 0.0f; float half_range_dynamic_c_deg = 0.0f;
    float half_range_dynamic_rmse_deg = 0.0f; float half_range_dynamic_loocv_rmse_deg = 0.0f;
    float half_range_dynamic_input_min_deg = 0.0f; float half_range_dynamic_input_max_deg = 0.0f;
    bool half_range_dynamic_valid = false;
    float half_range_fixed_r = 0.0f; float half_range_fixed_c_deg = 0.0f;
    float half_range_fixed_rmse_deg = 0.0f; float half_range_fixed_loocv_rmse_deg = 0.0f;
    float half_range_fixed_input_min_deg = 0.0f; float half_range_fixed_input_max_deg = 0.0f;
    bool half_range_fixed_valid = false;
  };
  static constexpr uint16_t kMaxIdentificationEvents = 96;
  static constexpr uint8_t kMaxCalibrationPeakEvents = 64;
  static constexpr uint8_t kMaxCalibrationProbeEvents = 32;
  static constexpr uint8_t kMaxCalibrationStateGateEvents = 96;
  static constexpr uint8_t kMaxCalibrationBuildUpEvents = 40;
  static constexpr uint16_t kMaxE2ShadowPeakEvents = Config::E2_SHADOW_MAX_PEAK_EVENTS;
  static constexpr uint16_t kMaxQ1ShadowEvents = Config::Q1_SHADOW_MAX_EVENTS;
  static constexpr uint16_t kMaxQIdentEvents = Config::Q_IDENT_MAX_EVENTS;
  static constexpr uint16_t kMaxEnergyControlV0Events = Config::ENERGY_CONTROL_V0_MAX_EVENTS;
  static constexpr uint16_t kMaxEnergyControlAutonomousEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;
  static constexpr uint16_t kMaxTimingProbeEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;
  static constexpr uint16_t kMaxSolverShadowEvents = Config::ENERGY_CONTROL_AUTONOMOUS_MAX_EVENTS;
  bool begin();
  void clear();
  void startRun(uint16_t run_id, uint64_t run_start_us, int16_t current_mA, uint16_t pulse_width_ms,
                uint16_t input_interval_ms, bool identification_mode = false,
                uint8_t q_run_mode = 0, int16_t control_target_cdeg = 0,
                uint8_t q_probe_schedule_id = 0, bool passive_capture = false,
                 float q1_shadow_target_peak_abs_deg = NAN, bool q_ident_mode = false,
                 uint8_t q_ident_run_schedule_id = 0, bool energy_control_v0_mode = false,
                 bool energy_control_autonomous_mode = false,
                 float input_peak_percent = 50.0f, uint16_t brake_max_ms = 50,
                 bool peak_adaptation_enabled = true,
                 bool energy_bridge_enabled = true,
                 bool negative_probe_mode = false,
                 const upright_reference::Record& upright_reference = upright_reference::Record{},
                 uint32_t negative_probe_batch_seed = 0);
  void recordMeasurementStartAngle(float absolute_deg) {
    if (!sealed_ && energy_control_autonomous_mode_ && !isfinite(measurement_start_abs_deg_))
      measurement_start_abs_deg_ = absolute_deg;
  }
  uint16_t beginIdentificationEvent(const IdentificationEvent& event);
  void finishIdentificationEvent(uint16_t event_id, uint32_t peak_ms, int16_t theta_peak_cdeg);
  void addCalibrationPeakEvent(const CalibrationPeakEvent& event);
  void addCalibrationProbeEvent(const CalibrationProbeEvent& event);
  void addCalibrationStateGateEvent(const CalibrationStateGateEvent& event);
  void addCalibrationBuildUpEvent(const CalibrationBuildUpEvent& event);
  void addE2ShadowPeakEvent(const E2ShadowPeakEvent& event);
  void addQ1ShadowEvent(const Q1ShadowEvent& event);
  void addQIdentEvent(const QIdentEvent& event);
  void addEnergyControlV0Event(const EnergyControlV0Event& event);
  void addEnergyControlAutonomousPeakEvent(const EnergyControlAutonomousPeakEvent& event);
  struct SolverShadowEvent {
    uint16_t event_index = 0;
    uint32_t pulse_id = 0;
    uint32_t t_test_ms = 0;
    int8_t physical_next_peak_side = 0;
    int8_t q_command_direction = 0;
    int16_t command_current_mA = 0;
    uint16_t model_vbat_mV = 0;
    float i0_estimated_mA = NAN;
    float free_next_peak_deg = NAN;
    float target_peak_deg = NAN;
    float target_energy_j = NAN;
    float q_available_mA_s = NAN;
    float integral_side_mA_s = NAN;
    uint16_t legacy_ff_width_ms = 0;
    float legacy_ff_q_mA_s = NAN;
    uint16_t legacy_selected_width_ms = 0;
    float legacy_selected_q_mA_s = NAN;
    uint16_t fast_ff_width_ms = 0;
    float fast_ff_q_mA_s = NAN;
    uint16_t fast_selected_width_ms = 0;
    float fast_selected_q_mA_s = NAN;
    uint32_t free_model_us = 0;
    uint32_t legacy_ff_scan_us = 0;
    uint32_t legacy_selected_scan_us = 0;
    uint32_t legacy_decision_us = 0;
    uint32_t fast_shadow_us = 0;
    uint16_t fast_eval_count = 0;
    bool fast_valid = false;
    bool ff_width_match = false;
    bool selected_width_match = false;
  };
  struct TimingProbeEvent {
    uint16_t event_index = 0;
    uint32_t pulse_id = 0;
    uint8_t pulse_kind = 0;  // 1=strong_start_kick, 2=normal_zero_cross.
    uint32_t t_test_ms = 0;
    int16_t command_mA = 0;
    uint16_t pulse_width_ms = 0;
    uint32_t pulse_start_us = 0;
    uint32_t gyro_sequence_at_start = 0;
    uint32_t set_current_us = 0;
    uint32_t state_update_us = 0;
    uint32_t current_model_us = 0;
    uint32_t update_pulse_model_us = 0;
    uint32_t pulse_begin_total_us = 0;
    uint32_t first_audit_log_offset_us = 0;
    uint32_t first_audit_log_us = 0;
    uint32_t imu_update_call_us = 0;
    uint32_t runner_update_call_us = 0;
    uint32_t core1_path_us = 0;
    uint32_t first_imu_dt_after_start_us = 0;
    uint32_t first_imu_sample_offset_us = 0;
    uint8_t capture_mask = 0;  // bit0=Core1 loop, bit1=2ms audit log, bit2=fresh gyro.
    bool complete = false;
  };
  void addEnergyControlAutonomousZeroCrossEvent(const EnergyControlAutonomousZeroCrossEvent& event);
  void addSolverShadowEvent(const SolverShadowEvent& event);
  void addSolverAuditEvent(const solver_audit::Record& event) { if (!sealed_ && solver_audit_) solver_audit_->push(event); }
  void addTimingProbeEvent(const TimingProbeEvent& event);
  bool energyControlAutonomousEventCapacityReached() const {
    return energy_control_autonomous_peak_event_count_ >= kMaxEnergyControlAutonomousEvents ||
        energy_control_autonomous_zero_cross_event_count_ >= kMaxEnergyControlAutonomousEvents;
  }
  void markEnergyControlAutonomousEventOverflow() { energy_control_autonomous_event_overflow_ = true; }
  bool energyControlV0EventCapacityReached() const {
    return energy_control_v0_event_count_ >= kMaxEnergyControlV0Events;
  }
  bool q1ShadowEventCapacityReached() const {
    return q1_shadow_event_count_ >= kMaxQ1ShadowEvents;
  }
  void markEnergyControlV0EventOverflow() { energy_control_v0_event_overflow_ = true; }
  void setCalibrationResult(const CalibrationResult& result);
  void markMeasurementDone();

  bool ready() const { return ready_; }
  const char* lastError() const { return last_error_; }
  uint16_t currentRunId() const { return current_run_id_; }
  uint64_t runStartUs() const { return run_start_us_; }
  bool lastMeasurementDone() const { return last_measurement_done_; }
  bool downloading() const { return downloading_; }

  bool addSample(const LogSample& row);

  size_t sampleCount() const { return sample_count_; }
  size_t sampleCapacity() const { return sample_capacity_; }
  uint8_t usagePercent() const;
  bool warningLevel() const;
  bool full() const { return sample_count_ >= sample_capacity_; }
  bool rwlogDownloadable() const;
  void downloadFilename(char* out, size_t out_len) const;

  size_t psramTotal() const;
  size_t psramFree() const;

  // The control owner seals only after terminal events have been committed.
  void seal() { sealed_ = true; last_measurement_done_ = true; }
  bool sealed() const { return sealed_; }
  const uint8_t* sampleBytes() const { return reinterpret_cast<const uint8_t*>(samples_); }
  static constexpr size_t kMetadataJsonReserveBytes = 768U * 1024U;
  void recordAdaptiveCg(const lateral_cg::AdaptiveSnapshot& s) { run_adaptive_cg_=s; }
  void recordLateralCg(const lateral_cg::Record& r) { run_lateral_cg_ = r; }
  bool buildMetadataJson(PsramString& json) const;
  RwLogFileHeader buildHeader(uint32_t metadata_size) const;
  static size_t eventStorageBytes() { return sizeof(EventStorage); }

private:


  struct EventStorage {
    IdentificationEvent identification_events_[kMaxIdentificationEvents] = {};
    CalibrationPeakEvent calibration_peak_events_[kMaxCalibrationPeakEvents] = {};
    CalibrationProbeEvent calibration_probe_events_[kMaxCalibrationProbeEvents] = {};
    CalibrationStateGateEvent calibration_state_gate_events_[kMaxCalibrationStateGateEvents] = {};
    CalibrationBuildUpEvent calibration_build_up_events_[kMaxCalibrationBuildUpEvents] = {};
    E2ShadowPeakEvent e2_shadow_peak_events_[kMaxE2ShadowPeakEvents] = {};
    Q1ShadowEvent q1_shadow_events_[kMaxQ1ShadowEvents] = {};
    QIdentEvent q_ident_events_[kMaxQIdentEvents] = {};
    EnergyControlV0Event energy_control_v0_events_[kMaxEnergyControlV0Events] = {};
    EnergyControlAutonomousPeakEvent energy_control_autonomous_peak_events_[kMaxEnergyControlAutonomousEvents] = {};
    EnergyControlAutonomousZeroCrossEvent energy_control_autonomous_zero_cross_events_[kMaxEnergyControlAutonomousEvents] = {};
    SolverShadowEvent solver_shadow_events_[kMaxSolverShadowEvents] = {};
    TimingProbeEvent timing_probe_events_[kMaxTimingProbeEvents] = {};
  };
  EventStorage* events_ = nullptr;
  bool sealed_ = true;
  LogSample* samples_ = nullptr;
  size_t sample_capacity_ = 0;
  size_t sample_count_ = 0;
  uint16_t current_run_id_ = 0;
  uint64_t run_start_us_ = 0;
  int16_t run_current_mA_ = 0;
  uint16_t run_pulse_width_ms_ = 0;
  uint16_t run_input_interval_ms_ = 0;
  bool identification_mode_ = false;
  uint8_t q_run_mode_ = 0;
  uint8_t q_probe_schedule_id_ = 0;
  int16_t control_target_cdeg_ = 0;
  bool passive_capture_ = false;
  float q1_shadow_target_peak_abs_deg_ = NAN;
  bool q_ident_mode_ = false;
  uint8_t q_ident_run_schedule_id_ = 0;
  bool energy_control_v0_mode_ = false;
  bool energy_control_autonomous_mode_ = false;
  float autonomous_input_peak_percent_ = 0;
  uint16_t autonomous_brake_max_ms_ = 50;
  bool peak_adaptation_enabled_ = true;
  bool energy_bridge_enabled_ = true;
  bool negative_probe_mode_ = false;
  uint32_t negative_probe_batch_seed_ = 0;
  upright_reference::Record run_upright_reference_;
  lateral_cg::Record run_lateral_cg_;
  lateral_cg::AdaptiveSnapshot run_adaptive_cg_;
  float measurement_start_abs_deg_ = NAN;
  uint32_t autonomous_timing_compensation_us_ = 0;  // Immutable metadata for the saved run.
  uint16_t identification_event_count_ = 0;
  uint8_t calibration_peak_event_count_ = 0;
  uint8_t calibration_probe_event_count_ = 0;
  uint8_t calibration_state_gate_event_count_ = 0;
  uint8_t calibration_build_up_event_count_ = 0;
  uint16_t e2_shadow_peak_event_count_ = 0;
  bool e2_shadow_peak_event_overflow_ = false;
  uint16_t q1_shadow_event_count_ = 0;
  bool q1_shadow_event_overflow_ = false;
  uint16_t q_ident_event_count_ = 0;
  bool q_ident_event_overflow_ = false;
  uint16_t energy_control_v0_event_count_ = 0;
  bool energy_control_v0_event_overflow_ = false;
  uint16_t energy_control_autonomous_peak_event_count_ = 0;
  uint16_t energy_control_autonomous_zero_cross_event_count_ = 0;
  bool energy_control_autonomous_event_overflow_ = false;
  uint16_t solver_shadow_event_count_ = 0;
  bool solver_shadow_event_overflow_ = false;
  solver_audit::Buffer<128>* solver_audit_ = nullptr;
  uint16_t timing_probe_event_count_ = 0;
  bool timing_probe_event_overflow_ = false;
  CalibrationResult calibration_result_;
  bool last_measurement_done_ = false;
  bool downloading_ = false;
  bool ready_ = false;
  const char* last_error_ = "not_initialized";
};




