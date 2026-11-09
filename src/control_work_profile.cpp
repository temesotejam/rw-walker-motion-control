#include "control_work_profile.h"
namespace control_work {
Profile profile;
String Profile::json() const {
  static const char* const names[] = {"owner_iteration", "service", "imu_delivery", "pose_guide",
      "filter", "angle_display", "current_roll", "motion", "log_row", "snapshot", "publish",
      "mekf_predict", "mekf_accel", "mekf_attitude", "madgwick", "log_encode", "log_store"};
  static_assert(sizeof(names) / sizeof(names[0]) == static_cast<uint8_t>(Stage::Count), "Stage names");
  String out; out.reserve(3200);
  out = "{\"revision\":\"control_work_04720\",\"scope\":\"measurement_only;host_wall_time_includes_preemption;nested_stages_not_additive\"";
  out += ",\"filter_scope\":\"MEKF_and_comparison_preparation;autonomous_measurement_Madgwick_after_motion_before_log;Madgwick_cohort_from_input_capture\"";
  out += ",\"mekf_attitude_scope\":\"posterior_copy_and_control_pitch;display_euler_from_snapshot_on_demand\"";
  out += ",\"mekf_math_compiler\":\"GCC_O2_no_fast_math\",\"madgwick_pitch\":\"upstream_2.4.0_pitch_only\"";
  out += ",\"target_code_placement\":\"IRAM_MEKF_log_encoder_normal_update_and_Madgwick_IMU;external_calls_and_data_may_use_flash\"";
  out += ",\"normal_update_compiler\":\"selected_routines_GCC_O2_no_fast_math;upstream_Madgwick_2.4.0_function_bodies_unchanged\"";
  out += ",\"log_encoder\":\"GCC_Os_no_fast_math_shared_noinline_quantizer\",\"log_substages\":\"encode_includes_beta_ceilings;store_is_synchronous_psram_addSample\"";
  out += ",\"cohort\":\"pulse_active_at_stage_entry;owner_includes_terminal_iteration;start_sync_excluded\"";
  for (uint8_t group = 0; group < 2; ++group) {
    out += group ? ",\"pulse_on\":{" : ",\"pulse_off\":{";
    for (uint8_t i = 0; i < static_cast<uint8_t>(Stage::Count); ++i) {
      const auto& s = stages[group][i];
      if (i) out += ",";
      out += "\"" + String(names[i]) + "\":{\"count\":" + String(s.count);
      out += ",\"max_us\":" + String(s.max_us);
      out += ",\"sum_us\":" + String(static_cast<double>(s.sum_us), 0);
      out += ",\"mean_us\":" + String(s.count ? static_cast<double>(s.sum_us) / s.count : 0.0, 3) + "}";
    }
    out += "}";
  }
  return out + "}";
}
}
