#pragma once
#include <cmath>
#include <cstdint>
#include "config.h"

namespace previous_peak_control {
enum Reason : uint8_t {
  PREV_REASON_APPLIED = 0,
  PREV_REASON_DISABLED = 1,
  PREV_REASON_BEFORE_ENABLE_TIME = 2,
  PREV_REASON_TARGET_UNSUPPORTED = 3,
  PREV_REASON_OUTSIDE_SUPPORT = 4,
  PREV_REASON_INVALID_INPUT = 5,
};
struct Result {
  float raw_correction_deg = NAN;
  float applied_correction_deg = 0.0f;
  float corrected_free_peak_deg = NAN;
  Reason reason = PREV_REASON_INVALID_INPUT;
  bool applied = false;
  bool clamped = false;
};
inline Result evaluate(float base_free_peak_deg, float previous_peak_deg, int8_t next_side,
                       float selected_mean_target_deg, uint32_t t_test_ms) {
  Result r;
  r.corrected_free_peak_deg = base_free_peak_deg;
  if (!Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_CONTROL_ENABLED) {
    r.reason = PREV_REASON_DISABLED; return r;
  }
  if (!std::isfinite(base_free_peak_deg) || !std::isfinite(previous_peak_deg) ||
      !std::isfinite(selected_mean_target_deg) || base_free_peak_deg < 0.0f ||
      previous_peak_deg < 0.0f || (next_side != 1 && next_side != -1)) {
    r.corrected_free_peak_deg = NAN; r.reason = PREV_REASON_INVALID_INPUT; return r;
  }
  if (t_test_ms < Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_ENABLE_AFTER_MS) {
    r.reason = PREV_REASON_BEFORE_ENABLE_TIME; return r;
  }
  // Qualify the calibrated operating regime using the user's mean target.
  // A transient side setpoint must not toggle this residual model on/off.
  if (std::fabs(selected_mean_target_deg - Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_TARGET_DEG) >
      Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_TARGET_TOLERANCE_DEG) {
    r.reason = PREV_REASON_TARGET_UNSUPPORTED; return r;
  }
  const float support_min = next_side > 0
      ? Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_PLUS_SUPPORT_MIN_DEG
      : Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_MINUS_SUPPORT_MIN_DEG;
  const float support_max = next_side > 0
      ? Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_PLUS_SUPPORT_MAX_DEG
      : Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_MINUS_SUPPORT_MAX_DEG;
  if (previous_peak_deg < support_min || previous_peak_deg > support_max) {
    r.reason = PREV_REASON_OUTSIDE_SUPPORT; return r;
  }
  const float c = next_side > 0
      ? Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_PLUS_C_AT_8_DEG
      : Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_MINUS_C_AT_8_DEG;
  const float k = next_side > 0
      ? Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_PLUS_K_PER_DEG
      : Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_MINUS_K_PER_DEG;
  r.raw_correction_deg = c + k * (previous_peak_deg - 8.0f);
  const float limit = Config::ENERGY_CONTROL_AUTONOMOUS_PREVIOUS_PEAK_MAX_ABS_CORRECTION_DEG;
  r.applied_correction_deg = std::fmax(-limit, std::fmin(limit, r.raw_correction_deg));
  r.clamped = std::fabs(r.applied_correction_deg - r.raw_correction_deg) > 1.0e-6f;
  r.corrected_free_peak_deg = std::fmax(0.0f, base_free_peak_deg + r.applied_correction_deg);
  r.reason = PREV_REASON_APPLIED; r.applied = true;
  return r;
}
} // namespace previous_peak_control
