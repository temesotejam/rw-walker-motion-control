#pragma once
namespace control_angle_reference {
// Work control uses the estimator's gravity-referenced pitch, never a captured pose.
inline float origin(bool work_mode, float saved_pitch) {
  return work_mode ? 0.f : saved_pitch;
}
}
