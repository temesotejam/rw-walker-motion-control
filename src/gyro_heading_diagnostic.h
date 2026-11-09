#pragma once
#include "gyro_steering.h"

// Diagnostic only. Never supplies a target or actuator decision.
// Raw sensor -> physical body frame is Ry(pi), matching MEKF; apply only
// the fixed startup gyro calibration (never the magnetic field).
class GyroHeadingDiagnostic {
 public:
  bool start(float ax, float ay, float az, uint32_t gyro_us,
             bool bias_ready, bool accel_fresh);
  void sample(float gx, float gy, float gz, float bx, float by, float bz,
              float gyro_y_scale, uint32_t gyro_us);
  bool valid() const { return heading_.valid(); }
  float yaw() const { return heading_.yaw(); }
 private:
  steering::GyroHeading heading_;
};
