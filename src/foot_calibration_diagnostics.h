#pragma once
#include <Arduino.h>
#include "foot_tracking_config.h"
#include "foot_calibration_table.h"
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
