#pragma once
#include <Arduino.h>
#include "mekf6.hpp"

// Copied by the exclusive control owner. HTTP never reads a live filter.
// Preserve the complete unprojected posterior on every update. Only control
// pitch is extracted here; HTTP derives roll/pitch from this copied quaternion.
// None of the run/display pitch zeros are subtracted.
struct MekfAttitudeSnapshot {
  bool valid = false;
  uint32_t sample_us = 0;
  float pitch_deg = NAN;
  mekf6::Quaternion quaternion{NAN, NAN, NAN, NAN};
  uint32_t accel_sample_us = 0;
  mekf6::Vec3 accel_g{NAN, NAN, NAN}, gyro_rad_s{NAN, NAN, NAN};
  mekf6::Vec3 bias_rad_s{NAN, NAN, NAN};
  mekf6::Diagnostics accel_update;
};

MekfAttitudeSnapshot RW_SPEED_CODE captureMekfAttitude(const mekf6::Mekf6& filter,
                                              bool initialized, uint32_t sample_us,
                                              const mekf6::Vec3& accel_g = {NAN, NAN, NAN},
                                              const mekf6::Vec3& gyro_rad_s = {NAN, NAN, NAN},
                                              uint32_t accel_sample_us = 0);

inline String mekfAttitudeJson(const MekfAttitudeSnapshot& s, uint32_t now_us, bool imu_ok) {
  // This runs in the diagnostic consumer, never the 400 Hz capture path.
  // All axes belong to s.sample_us, even if the live filter has since advanced.
  const auto e = s.valid ? mekf6::Mekf6::eulerDegFromQuaternion(s.quaternion)
                        : mekf6::EulerDeg{NAN, NAN, NAN};
  const auto num = [](float x) { return isfinite(x) ? String(x, 6) : String("null"); };
  const uint32_t age_us = now_us - s.sample_us; // wrap-safe host microsecond clock
  const auto vector = [&num](const mekf6::Vec3& v, float scale) {
    return String("[") + num(v.x * scale) + "," + num(v.y * scale) + "," + num(v.z * scale) + "]";
  };
  const auto finite = [](const mekf6::Vec3& v) { return isfinite(v.x) && isfinite(v.y) && isfinite(v.z); };
  const bool inputs_valid = s.valid && finite(s.accel_g) && finite(s.gyro_rad_s) && finite(s.bias_rad_s);
  String json; json.reserve(1024);
  json = "{\"valid\":" + String(s.valid ? "true" : "false");
  json += ",\"fresh\":" + String(s.valid && imu_ok && age_us < 500000 ? "true" : "false");
  json += ",\"sample_us\":" + String(s.sample_us);
  json += ",\"age_us\":" + (s.valid ? String(age_us) : String("null"));
  json += ",\"estimate\":\"posterior\",\"frame\":\"mekf\",\"euler_order\":\"ZYX\"";
  json += ",\"roll_deg\":" + num(e.roll) + ",\"pitch_deg\":" + num(e.pitch);
  json += ",\"quaternion\":{\"w\":" + num(s.quaternion.w) + ",\"x\":" + num(s.quaternion.x);
  json += ",\"y\":" + num(s.quaternion.y) + ",\"z\":" + num(s.quaternion.z) + "}";
  json += ",\"inputs\":{\"valid\":" + String(inputs_valid ? "true" : "false");
  json += ",\"frame\":\"mekf\",\"axis_order\":\"xyz\",\"accel_g\":" + vector(s.accel_g, 1.0f);
  json += ",\"gyro_dps\":" + vector(s.gyro_rad_s, mekf6::radToDeg(1.0f));
  json += ",\"gyro_bias_dps\":" + vector(s.bias_rad_s, mekf6::radToDeg(1.0f));
  json += ",\"accel_sample_us\":" + String(s.accel_sample_us);
  json += ",\"accel_age_us\":" + (inputs_valid ? String(static_cast<uint32_t>(now_us - s.accel_sample_us)) : String("null"));
  json += ",\"last_accel_update\":{\"used\":" + String(s.accel_update.accel_used ? "true" : "false");
  json += ",\"confidence\":" + num(s.accel_update.accel_confidence);
  json += ",\"residual_deg\":" + num(s.accel_update.accel_direction_residual_deg) + "}}}";
  return json;
}
