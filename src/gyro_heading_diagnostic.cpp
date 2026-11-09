#include "gyro_heading_diagnostic.h"

__attribute__((noinline)) bool GyroHeadingDiagnostic::start(
    float ax, float ay, float az, uint32_t gyro_us, bool bias_ready, bool accel_fresh) {
  heading_ = steering::GyroHeading{};
  return bias_ready && accel_fresh && gyro_us != 0 && heading_.reset(-ax, ay, -az, gyro_us);
}

__attribute__((noinline)) void GyroHeadingDiagnostic::sample(
    float gx, float gy, float gz, float bx, float by, float bz,
    float gyro_y_scale, uint32_t gyro_us) {
  heading_.update(-(gx-bx), (gy-by)*gyro_y_scale, -(gz-bz), gyro_us);
}
