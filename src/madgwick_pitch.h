#pragma once
#include <math.h>
#include "realtime_code.h"

// Adafruit AHRS 2.4.0 getPitch() invokes computeAngles(), including two atan2
// calls and a gravity vector that these comparison series never consume.
// Keep its exact pitch expression and degree factor, with NO extra quaternion
// normalization or clamp. The filter update and its lazy all-axis API stay intact.
// The pinned upstream source is checked by tools/verify_madgwick_dependency.py.
namespace madgwick_pitch {
template <class Filter>
inline float RW_SPEED_CODE readPitchDeg(Filter& filter) {
  float w, x, y, z;
  filter.getQuaternion(&w, &x, &y, &z);
  return asinf(-2.0f * (x * z - w * y)) * 57.29578f;
}
}
