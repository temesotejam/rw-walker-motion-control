#pragma once
#include <Arduino.h>
#include "foot_tracking_config.h"
#include "foot_calibration.h"

inline String footCalibrationDiagnosticsJson() {
  String json; json.reserve(1400);
  json = "{\"revision\":\"fixed_leg_five_pose_repeat_20261101\",\"provisional\":true";
  json += ",\"method\":\"piecewise_linear_with_endpoint_extrapolation\"";
  json += ",\"reference\":\"baseline_mekf_roll_minus_pose_mekf_roll\"";
  json += ",\"assumption\":\"feet_fixed_upper_body_camera_rotate_about_same_leg_joint\"";
  json += ",\"zero\":\"per_boot_subtract_calibrated_angle_at_zero_x\"";
  json += ",\"fit_poses\":5,\"heldout_hand_supported_poses\":0";
  json += ",\"independent_angular_accuracy_validated\":false";
  json += ",\"outside_range_policy\":\"extrapolate_nearest_endpoint_segment_and_save\"";
  json += ",\"right_deg_per_px\":" + String(appcfg::kFootAngleADegPerPx, 9);
  json += ",\"left_deg_per_px\":" + String(appcfg::kFootAngleBDegPerPx, 9);
  json += ",\"source_file\":\"foot_calibration_20261101.json\"";
  json += ",\"source_sha256\":\"ceee4309646f2850f1bbf150ee3148438d60853f488b03f4c2245fe0ff85d146\"";
  for (int side=0; side<2; ++side) {
    json += side==0 ? ",\"right_points\":[" : ",\"left_points\":[";
    const auto* points = side==0 ? foot_calibration::right : foot_calibration::left;
    for (size_t i=0; i<foot_calibration::count; ++i) {
      if(i) json += ",";
      json += "[" + String(points[i].x, 5) + "," + String(points[i].angle, 6) + "]";
    }
    json += "]";
  }
  json += "}"; return json;
}

// One source for API, image diagnostics and RWLOG range metadata.
inline String footRangeDiagnosticsJson(bool live=false) {
  const auto pair = [](float lo, float hi) { return "[" + String(lo, 4) + "," + String(hi, 4) + "]"; };
  String json; json.reserve(700);
  if(live)return "{\"right_support_x\":"+pair(appcfg::kFootAngleAMinCalXPx,appcfg::kFootAngleAMaxCalXPx)+",\"left_support_x\":"+pair(appcfg::kFootAngleBMinCalXPx,appcfg::kFootAngleBMaxCalXPx)+"}";
  json = "{\"revision\":\"fixed_leg_five_pose_repeat_20261101\"";
  json += ",\"basis\":\"five_fixed_leg_poses_with_repeat_endpoint\"";
  json += ",\"right_support_x\":" + pair(appcfg::kFootAngleAMinCalXPx, appcfg::kFootAngleAMaxCalXPx);
  json += ",\"left_support_x\":" + pair(appcfg::kFootAngleBMinCalXPx, appcfg::kFootAngleBMaxCalXPx);
  json += ",\"original_right_support_x\":" + pair(appcfg::kFootAngleAOriginalMinCalXPx, appcfg::kFootAngleAOriginalMaxCalXPx);
  json += ",\"original_left_support_x\":" + pair(appcfg::kFootAngleBOriginalMinCalXPx, appcfg::kFootAngleBOriginalMaxCalXPx);
  json += ",\"observed_right_x\":" + pair(appcfg::kFootObservedAMinXPx, appcfg::kFootObservedAMaxXPx);
  json += ",\"observed_left_x\":" + pair(appcfg::kFootObservedBMinXPx, appcfg::kFootObservedBMaxXPx);
  json += ",\"margin_px\":" + String(appcfg::kFootObservedMarginPx, 1);
  json += ",\"source_manifest\":\"foot_calibration_20261101.json\"";
  json += ",\"extension_angle_accuracy_validated\":false";
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
  json += ",\"control_extension_deg\":" + String(foot_control_range::kExtensionDeg,1);
  json += ",\"control_extension_policy\":\"nearest_measured_end_segment_max5deg;in_range_flags_remain_measured_support\"";
#endif
  json += "}";
  return json;
}
