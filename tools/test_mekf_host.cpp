#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/mekf6.hpp"

// V46f calibrated physical sensor-to-body transform identified from the real installation:
// R_y(pi)=diag(-1,+1,-1). Raw upright gravity is -Z and physical/video pitch
// rate follows +raw_gy.
static mekf6::Vec3 accelFilter(float ax, float ay, float az) {
  return {-ax, ay, -az};
}
static constexpr float kGyroYScale = 0.908911f;
static mekf6::Vec3 gyroFilter(float gx, float gy, float gz) {
  return {mekf6::degToRad(-gx), mekf6::degToRad(gy * kGyroYScale), mekf6::degToRad(-gz)};
}
static float reportedPitch(const mekf6::Mekf6& f) { return f.eulerDeg().pitch; }

int main() {
  // 1) Static physical/video sign. Positive fixed-horizon pitch has +raw ax.
  mekf6::Mekf6 f;
  const float a10 = mekf6::degToRad(10.0f);
  auto a = accelFilter(std::sin(a10), 0.0f, -std::cos(a10));
  assert(f.initializeFromAccel(a));
  if (std::fabs(reportedPitch(f) - 10.0f) > 0.05f) return 1;

  // 1b) Actual measured upright vector must initialize on the near-upright
  // Euler branch and accept gravity immediately.
  f.reset();
  const auto measured_upright = accelFilter(0.021626f, 0.033568f, -0.999202f);
  if (!f.initializeFromAccel(measured_upright)) return 9;
  if (!f.predict(gyroFilter(0, 0, 0), 0.005f)) return 10;
  const bool upright_used = f.updateAccel(measured_upright);
  const auto upright_diag = f.diagnostics();
  const auto upright_euler = f.eulerDeg();
  std::printf("measured_upright roll=%.3f pitch=%.3f used=%d conf=%.3f resid=%.3f\n",
              upright_euler.roll, upright_euler.pitch, upright_used ? 1 : 0,
              upright_diag.accel_confidence, upright_diag.accel_direction_residual_deg);
  if (!upright_used || !upright_diag.accel_used) return 11;
  if (upright_diag.accel_confidence < 0.99f ||
      upright_diag.accel_direction_residual_deg > 0.1f) return 12;
  if (std::fabs(upright_euler.roll) > 5.0f) return 13;
  if (std::fabs(upright_euler.pitch - 1.239f) > 0.1f) return 14;

  // 2) Dynamic sign and calibrated scale measured from synchronized video.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  for (int i = 0; i < 20; ++i) f.predict(gyroFilter(0, 90, 0), 0.005f);
  const float p = reportedPitch(f);
  std::printf("reported_pitch_after_100ms_raw_gy_+90=%.3f\n", p);
  if (!(p > 8.0f && p < 8.4f)) return 2;

  // 3) Physically consistent +45-deg sweep: +raw gy and +raw ax.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  for (int i = 1; i <= 100; ++i) {
    const float theta = mekf6::degToRad(90.0f * i * 0.005f);
    if (!f.predict(gyroFilter(0, 90.0f / kGyroYScale, 0), 0.005f)) return 3;
    if (!f.updateAccel(accelFilter(std::sin(theta), 0, -std::cos(theta)))) return 4;
  }
  const auto sweep_diag = f.diagnostics();
  std::printf("synthetic_sweep_pitch=%.3f conf=%.3f resid=%.3f\n",
              reportedPitch(f), sweep_diag.accel_confidence,
              sweep_diag.accel_direction_residual_deg);
  if (std::fabs(reportedPitch(f) - 45.0f) > 0.5f) return 5;
  if (!sweep_diag.accel_used || sweep_diag.accel_confidence < 0.99f) return 6;

  // 3b) V46g one-step prediction must not mutate the posterior.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  const float posterior_before = reportedPitch(f);
  const auto pred = f.predictEulerDeg(gyroFilter(0, 90, 0), 0.0025f);
  const float posterior_after = reportedPitch(f);
  std::printf("v46g_predicted_pitch_2p5ms=%.4f posterior_after=%.4f\n", pred.pitch, posterior_after);
  if (!(pred.pitch > 0.19f && pred.pitch < 0.22f)) return 15;
  if (std::fabs(posterior_before - posterior_after) > 1.0e-6f) return 16;

  // 4) Strong translational acceleration must still be rejected.
  f.reset();
  assert(f.initializeFromAccel(accelFilter(0, 0, -1)));
  f.predict(gyroFilter(0, 0, 0), 0.005f);
  const bool used = f.updateAccel(accelFilter(0.80f, 0.0f, -1.0f));
  const auto reject_diag = f.diagnostics();
  std::printf("accel_reject used=%d conf=%.3f norm=%.3f resid=%.3f\n",
              used ? 1 : 0, reject_diag.accel_confidence,
              reject_diag.accel_norm_g, reject_diag.accel_direction_residual_deg);
  if (used || reject_diag.accel_used || reject_diag.accel_confidence >= 0.05f) return 7;

  const auto q = f.quaternion();
  const float qn = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  std::printf("q_norm=%.7f\n", qn);
  if (std::fabs(qn - 1.0f) > 1e-5f) return 8;

  std::puts("V46g MEKF 400Hz prediction / 200Hz accel test passed");
  return 0;
}
