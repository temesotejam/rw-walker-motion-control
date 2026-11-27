#pragma once
#include "realtime_code.h"
#include <math.h>
#include <stdint.h>

// Pure model arithmetic. All times supplied to the integral are seconds; the
// final actuator command is an integer number of milliseconds in [0, 100].
namespace direct_q {
constexpr uint16_t kMaxWidthMs = 100;
constexpr char kRevision[] = "direct_q_branch_inverse_04718";

struct Target {
  bool valid = false, upper = false, lower = false;
  float requested_ff = NAN, feedforward = NAN, unclamped = NAN, charge = NAN;
};

// Invert free + base*Q + clamp(c + (gain-base)*Q, -limit, limit).
// The deployed correction is disabled (c=0, gain=base). Keep the bounded
// correction's three monotone pieces supported without iterative angle search.
// Internal linkage keeps Xtensa's literals with any emitted IRAM body rather
// than in a weak COMDAT literal section after the instructions that load them.
static inline RW_SPEED_CODE Target target(float free_peak, float target_peak, float base, float c,
                     float gain, float limit, float available, float integral) {
  Target out;
  if (!isfinite(free_peak) || !isfinite(target_peak) || !isfinite(base) ||
      !isfinite(c) || !isfinite(gain) || !isfinite(limit) ||
      !isfinite(available) || !isfinite(integral) || free_peak < 0.0f ||
      target_peak <= 0.0f || base <= 0.0f || gain <= 0.0f ||
      limit < 0.0f || available < 0.0f) return out;
  float q = (target_peak - free_peak - c) / gain;
  const float correction = c + (gain - base) * q;
  if (!isfinite(q) || !isfinite(correction)) return out;
  if (correction > limit) q = (target_peak - free_peak - limit) / base;
  else if (correction < -limit) q = (target_peak - free_peak + limit) / base;
  if (!isfinite(q)) return out;
  out.requested_ff = fmaxf(0.0f, q);
  // Preserve feedforward saturation BEFORE adding the existing side integral.
  // Only the old intermediate 1 ms quantization is removed.
  out.feedforward = fminf(available, out.requested_ff);
  out.unclamped = out.feedforward + integral;
  if (!isfinite(out.unclamped)) return Target{};
  out.upper = out.requested_ff > available || out.unclamped > available;
  out.lower = out.unclamped < 0.0f;
  out.charge = fmaxf(0.0f, fminf(available, out.unclamped));
  out.valid = true;
  return out;
}

struct Width {
  bool valid = false;
  uint16_t ms = 0, evaluations = 0;
  float charge = NAN, error = NAN;
};

// Solve |integral(I(t), 0..t)| = requested, including reversed residual current.
// Signed charge has at most one turning point (where I(t)=0). On each monotone
// integer interval, bracket BOTH +requested and -requested. Adjacent integers,
// interval ends and the two integers around the turning point cover the minimum
// absolute Q error. Exact error ties select the shorter pulse, including zero.
static inline RW_SPEED_CODE Width width(float requested, float initial_current, float final_current,
                   float tau_s, uint16_t max_ms = kMaxWidthMs) {
  Width out;
  if (!isfinite(requested) || !isfinite(initial_current) ||
      !isfinite(final_current) || !isfinite(tau_s) || requested < 0.0f ||
      final_current == 0.0f || tau_s <= 0.0f || max_ms > kMaxWidthMs) return out;
  const float direction = final_current > 0.0f ? 1.0f : -1.0f;
  const float goal = fabsf(final_current), initial = direction * initial_current;
  out.valid = true; out.charge = 0.0f; out.error = requested;
  if (requested == 0.0f || max_ms == 0) return out;
  struct Point { uint16_t ms; float signed_q; };
  bool finite = true;
  auto point = [&](uint16_t ms) RW_SPEED_CODE -> Point {
    const float t = static_cast<float>(ms) / 1000.0f;
    const float signed_q = ms == 0 ? 0.0f :
        goal * t + (initial - goal) * tau_s * (1.0f - expf(-t / tau_s));
    if (ms) ++out.evaluations;
    if (!isfinite(signed_q)) { finite = false; return {ms, NAN}; }
    const float q = fabsf(signed_q), error = fabsf(q - requested);
    if (error < out.error || (error == out.error && ms < out.ms)) {
      out.ms = ms; out.charge = q; out.error = error;
    }
    return {ms, signed_q};
  };
  auto bracket = [&](Point lo, Point hi, float signed_target, bool increasing) RW_SPEED_CODE {
    if (!finite || lo.ms == hi.ms) return;
    const float low_q = increasing ? lo.signed_q : hi.signed_q;
    const float high_q = increasing ? hi.signed_q : lo.signed_q;
    if (signed_target < low_q || signed_target > high_q) return;
    // At most seven bisections for an interval of at most 100 ms.
    for (uint8_t step = 0; step < 7 && hi.ms - lo.ms > 1; ++step) {
      const Point mid = point(static_cast<uint16_t>((lo.ms + hi.ms) / 2));
      if (!finite) return;
      if (increasing ? mid.signed_q < signed_target : mid.signed_q > signed_target) lo = mid;
      else hi = mid;
    }
  };
  const Point zero{0, 0.0f}, end = point(max_ms);
  if (initial >= 0.0f) {
    bracket(zero, end, requested, true);
  } else {
    const float turn_ms = tau_s * log1pf(-initial / goal) * 1000.0f;
    if (!isfinite(turn_ms)) return Width{};
    if (turn_ms >= static_cast<float>(max_ms)) {
      bracket(zero, end, -requested, false);
    } else {
      const uint16_t lower_ms = static_cast<uint16_t>(floorf(turn_ms));
      const uint16_t upper_ms = static_cast<uint16_t>(ceilf(turn_ms));
      const Point lower = point(lower_ms);
      const Point upper = upper_ms == lower_ms ? lower : point(upper_ms);
      bracket(zero, lower, -requested, false);
      bracket(upper, end, -requested, true);
      bracket(upper, end, requested, true);
    }
  }
  if (!finite) return Width{};
  return out;
}
}  // namespace direct_q
