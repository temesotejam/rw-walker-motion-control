#pragma once
#include <stdint.h>
#include <math.h>
#include "foot_tracking_config.h"

enum class FootZeroReason : uint8_t { Waiting, BodyMoving, MarkerInvalid, PositionMismatch, MarkerMoving, Collecting, Ready };
inline const char* footZeroReasonName(FootZeroReason reason) {
  switch (reason) {
    case FootZeroReason::Waiting: return "waiting";
    case FootZeroReason::BodyMoving: return "body_moving";
    case FootZeroReason::MarkerInvalid: return "marker_invalid";
    case FootZeroReason::PositionMismatch: return "position_mismatch";
    case FootZeroReason::MarkerMoving: return "marker_moving";
    case FootZeroReason::Collecting: return "collecting";
    case FootZeroReason::Ready: return "ready";
  }
  return "unknown";
}

// No automatic re-zero after lock. Both continuous IMU stability and continuous
// valid images are required; IMU epoch detects movement between two frames.
class FootZero {
 public:
  void observe(uint32_t now_ms, uint32_t epoch, bool stable,
               bool valid, float a, float b) {
    if (ready) return;
    if (!stable) { reset(); reason = FootZeroReason::BodyMoving; return; }
    if (!valid || !isfinite(a) || !isfinite(b)) { reset(); reason = FootZeroReason::MarkerInvalid; return; }
    if (fabsf(a - appcfg::kFootAngleAZeroXPx) > appcfg::kAutoZeroMaxNominalOffsetXPx ||
        fabsf(b - appcfg::kFootAngleBZeroXPx) > appcfg::kAutoZeroMaxNominalOffsetXPx) {
      reset(); reason = FootZeroReason::PositionMismatch; return;
    }
    if (count && (epoch != epoch_ || static_cast<uint32_t>(now_ms - last_ms_) > 300)) reset();
    if (count && (fmaxf(max_a_, a) - fminf(min_a_, a) > appcfg::kAutoZeroMaxSpreadXPx ||
                  fmaxf(max_b_, b) - fminf(min_b_, b) > appcfg::kAutoZeroMaxSpreadXPx)) {
      reset(); reason = FootZeroReason::MarkerMoving; return;
    }
    if (!count) { first_ms_ = now_ms; epoch_ = epoch; min_a_ = max_a_ = a; min_b_ = max_b_ = b; }
    min_a_ = fminf(min_a_, a); max_a_ = fmaxf(max_a_, a);
    min_b_ = fminf(min_b_, b); max_b_ = fmaxf(max_b_, b);
    last_ms_ = now_ms; sum_a_ += a; sum_b_ += b; ++count;
    reason = FootZeroReason::Collecting;
    if (count >= appcfg::kAutoZeroMinVisionSamples &&
        static_cast<uint32_t>(now_ms - first_ms_) >= appcfg::kAutoZeroStableMs) {
      a_zero = sum_a_ / count; b_zero = sum_b_ / count; ready = true; reason = FootZeroReason::Ready;
    }
  }
  bool ready = false;
  FootZeroReason reason = FootZeroReason::Waiting;
  float a_zero = appcfg::kFootAngleAZeroXPx, b_zero = appcfg::kFootAngleBZeroXPx;
  uint32_t count = 0;
 private:
  void reset() { count = 0; sum_a_ = sum_b_ = 0; }
  uint32_t first_ms_ = 0, last_ms_ = 0, epoch_ = 0;
  double sum_a_ = 0, sum_b_ = 0;
  float min_a_ = 0, max_a_ = 0, min_b_ = 0, max_b_ = 0;
};
