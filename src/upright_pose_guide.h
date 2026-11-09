#pragma once

#include <Arduino.h>
#include <math.h>

#include "imu_manager.h"
#include "foot_tracking_config.h"

namespace UprightPoseGuide {

// Measured from the stationary START_SYNC portion of
// 20260913_122233_294_501c (V46 hardware run).
// Mean raw acceleration was approximately (+0.0218,+0.0338,-1.0066) g.
// The normalized vector below is used only as an upright-direction reference.
static constexpr float REF_AX = 0.021626f;
static constexpr float REF_AY = 0.033568f;
static constexpr float REF_AZ = -0.999202f;

// User workflow: power up while the mechanism is safely lying down. Ten seconds
// after boot, illuminate the sync LED to say "now stand the mechanism upright".
static constexpr uint32_t GUIDE_LED_ON_AFTER_BOOT_MS = 10000UL;
static constexpr uint32_t UPRIGHT_STABLE_HOLD_MS = 400UL;

// START and boot foot-zero use the same pitch-axis gravity reference. Roll
// need not match the historical upright pose, but the IMU and feet must settle.
static constexpr float UPRIGHT_MIN_ACCEL_NORM_G = 0.85f;
static constexpr float UPRIGHT_MAX_ACCEL_NORM_G = 1.15f;
static constexpr float UPRIGHT_MAX_GYRO_NORM_DPS = 5.0f;

// Autonomous V7 re-initializes MEKF during the first OFF segment of the
// unchanged START synchronization pattern (the first segment is 1000 ms).
static constexpr uint32_t MEKF_REINIT_AVERAGE_MS = 800UL;
static constexpr uint32_t MEKF_REINIT_MIN_SAMPLES = 60UL;

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline float accelNormG(const ImuReading& r) {
  return sqrtf(r.ax_g * r.ax_g + r.ay_g * r.ay_g + r.az_g * r.az_g);
}

inline float gyroNormDps(const ImuReading& r) {
  return sqrtf(r.gx_dps * r.gx_dps + r.gy_dps * r.gy_dps + r.gz_dps * r.gz_dps);
}

inline float directionErrorDeg(const ImuReading& r, float n) {
  if (!isfinite(n) || n < 0.2f) return 180.0f;
  const float dot = clampf((r.ax_g * REF_AX + r.ay_g * REF_AY + r.az_g * REF_AZ) / n,
                           -1.0f, 1.0f);
  return acosf(dot) * 57.29577951308232f;
}

inline float directionErrorDeg(const ImuReading& r) { return directionErrorDeg(r, accelNormG(r)); }

// Raw X maps to the adopted MEKF pitch axis. Its normalized gravity component
// is unchanged by rotation about X (the fore/aft roll axis). The reference is
// the measured upright pose, rather than an absolute MEKF Euler zero.
inline float pitchErrorDeg(const ImuReading& r, float n) {
  if (!isfinite(n) || n < 0.2f) return 180.0f;
  const float ref_n = sqrtf(REF_AX * REF_AX + REF_AY * REF_AY + REF_AZ * REF_AZ);
  const float pitch = asinf(clampf(r.ax_g / n, -1.0f, 1.0f));
  const float reference = asinf(clampf(REF_AX / ref_n, -1.0f, 1.0f));
  return fabsf(pitch - reference) * 57.29577951308232f;
}
inline float pitchErrorDeg(const ImuReading& r) { return pitchErrorDeg(r, accelNormG(r)); }

// The IMU consumer retains the same acceleration over gyro-only deliveries.
// Reuse its geometric diagnostics until that sensor's sequence changes. Age,
// health and the elapsed upright hold are still evaluated on every control step.
struct CachedMetrics {
  float accel_norm_g = 0, direction_error_deg = 180, pitch_error_deg = 180, gyro_norm_dps = 0;
  uint32_t accel_sequence = 0, gyro_sequence = 0;
  bool have_accel = false, have_gyro = false;
  void update(const ImuReading& r) {
    if (!have_accel || accel_sequence != r.accel_sequence) {
      accel_norm_g = r.acc_norm_g;
      direction_error_deg = directionErrorDeg(r, accel_norm_g);
      pitch_error_deg = pitchErrorDeg(r, accel_norm_g);
      accel_sequence = r.accel_sequence; have_accel = true;
    }
    if (!have_gyro || gyro_sequence != r.gyro_sequence) {
      gyro_norm_dps = gyroNormDps(r);
      gyro_sequence = r.gyro_sequence; have_gyro = true;
    }
  }
};

inline bool isUprightStableSample(const ImuReading& r) {
  const float a_norm = accelNormG(r);
  if (!isfinite(a_norm) || a_norm < UPRIGHT_MIN_ACCEL_NORM_G ||
      a_norm > UPRIGHT_MAX_ACCEL_NORM_G) {
    return false;
  }
  // Refuse an inverted pose; allow a stable nonzero fore/aft roll offset.
  if (!(r.az_g < 0.0f) || pitchErrorDeg(r, a_norm) > appcfg::kAutoZeroMaxPitchErrorDeg) return false;
  return gyroNormDps(r) <= UPRIGHT_MAX_GYRO_NORM_DPS;
}

}  // namespace UprightPoseGuide
