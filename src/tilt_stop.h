#pragma once
#include <math.h>
#include <stdint.h>
#include "mekf6.hpp"

namespace tilt_stop {
// Track the continuous fore/aft roll branch from an upright run start.
// In the calibrated body frame, gravity projected onto the YZ plane is
// cos(lateral_pitch) * [sin(fore_aft_roll), cos(fore_aft_roll)]. Sideways
// passage through +/-90 reverses BOTH components, not the fore/aft angle.
// Choose the sign continuous with the preceding sample to reject that reversal.
// This assumes <90 degrees of fore/aft motion between observable samples.
// No Euler extraction, inverse trig, allocation or extra estimator is used.
class ForeAftGuard {
 public:
  void reset() { previous_up_ = 1.0f; previous_forward_ = 0.0f; }
  const char* reason(const mekf6::Quaternion& q, bool valid, uint32_t age_us,
                     const mekf6::Vec3& upright) {
    const float n = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (!valid || !isfinite(n) || n < 0.9f || n > 1.1f) return "tilt_invalid_mekf";
    if (age_us > 10000) return "tilt_stale_mekf";
    const float gx = 2.0f * (q.x*q.z - q.w*q.y);
    const float gy = 2.0f * (q.y*q.z + q.w*q.x);
    const float gz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
    float up = gx*upright.x + gy*upright.y + gz*upright.z;
    // The calibrated Y axis is proportional to {0, upright.z, -upright.y}.
    // Its constant positive scale does not affect the zero boundary.
    float forward = gy*upright.z - gz*upright.y;
    const float projection_sq = up*up + forward*forward;
    // At exact sideways +/-90 the fore/aft gravity projection is unobservable.
    // Retain the previous branch; sideways posture itself must not cause STOP.
    if (projection_sq <= 1.0e-8f*n*n) return nullptr;
    if (up*previous_up_ + forward*previous_forward_ < 0.0f) {
      up = -up; forward = -forward;
    }
    previous_up_ = up; previous_forward_ = forward;
    // Relative tolerance keeps the 90-degree edge independent of lateral tilt.
    return up <= 0.0f || up*up <= 1.0e-12f*projection_sq ? "fore_aft_tilt_90deg" : nullptr;
  }
 private:
  float previous_up_ = 1.0f, previous_forward_ = 0.0f;
};
}
