#pragma once
#include <cmath>
#include <cstdint>

// V46ah direct rate baseline in the measured state range. Q gains stay at V46af
// values; this fit is predictive, not an independently identified Q response.
namespace rate_baseline {
constexpr uint32_t ENABLE_AFTER_MS = 10000;
constexpr float PLUS_AT_65_DPS = 7.217460941f;
constexpr float PLUS_PER_DPS = 0.286814471f;
constexpr float MINUS_AT_65_DPS = 8.399746959f;
constexpr float MINUS_PER_DPS = 0.130807354f;
enum Reason : uint8_t { APPLIED=0, WARMUP=1, OTHER_SETTINGS=2, OUTSIDE_STATE=3,
                        NONFINITE=4, NOT_EVALUATED=255 };
struct Result {
  float p1_deg;
  float rate_deg;
  float correction_deg;
  float adjusted_deg;
  Reason reason;
};
inline Result evaluate(float p1_deg, float abs_rate_dps, int8_t side,
                       float previous_peak_deg, float target_deg,
                       uint32_t t_test_ms, uint32_t compensation_us) {
  Result r{p1_deg, NAN, 0.0f, p1_deg, NOT_EVALUATED};
  if (!std::isfinite(p1_deg) || !std::isfinite(abs_rate_dps) ||
      !std::isfinite(previous_peak_deg) || !std::isfinite(target_deg) ||
      p1_deg < 0 || abs_rate_dps < 0 || (side != 1 && side != -1)) {
    r.reason=NONFINITE; return r;
  }
  r.rate_deg = side > 0 ? PLUS_AT_65_DPS + PLUS_PER_DPS*(abs_rate_dps-65.0f)
                       : MINUS_AT_65_DPS + MINUS_PER_DPS*(abs_rate_dps-65.0f);
  if (target_deg != 8.0f || compensation_us != 3000) {
    r.reason=OTHER_SETTINGS; return r;
  }
  if (t_test_ms < ENABLE_AFTER_MS) { r.reason=WARMUP; return r; }
  const bool supported = side > 0
      ? (previous_peak_deg >= 8.3f && previous_peak_deg <= 9.8f &&
         abs_rate_dps >= 61.5f && abs_rate_dps <= 72.0f)
      : (previous_peak_deg >= 6.7f && previous_peak_deg <= 9.0f &&
         abs_rate_dps >= 59.0f && abs_rate_dps <= 70.0f);
  if (!supported) { r.reason=OUTSIDE_STATE; return r; }
  // Use the rate model directly: no blend, no limit relative to the old P1.
  // P1 and the full difference remain logged; all preceding gates are retained.
  r.adjusted_deg=r.rate_deg;
  r.correction_deg=r.adjusted_deg-p1_deg;
  r.reason=APPLIED;
  return r;
}
} // namespace rate_baseline
