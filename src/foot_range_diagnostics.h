#pragma once
#include <Arduino.h>
#include "foot_tracking_config.h"

// One source for API, image diagnostics and RWLOG range metadata.
inline String footRangeDiagnosticsJson() {
  const auto pair = [](float lo, float hi) { return "[" + String(lo, 4) + "," + String(hi, 4) + "]"; };
  String json; json.reserve(700);
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
  json += ",\"extension_angle_accuracy_validated\":false}";
  return json;
}
