#pragma once
#include <stddef.h>
#include <cmath>
#include <stdint.h>
#include <math.h>
#include "foot_tracking_config.h"

namespace foot_calibration {
struct Point { float x; float angle; };
static constexpr Point right[] = {
  {67.5192000f, 20.6265010f},
  {111.0505000f, 14.1099580f},
  {152.5749000f, 8.3712810f},
  {182.4695000f, 3.6884650f},
  {203.0819000f, 0.0000000f},
};
static constexpr Point left[] = {
  {63.6918000f, 20.6265010f},
  {102.7030000f, 14.1099580f},
  {144.3562000f, 8.3712810f},
  {173.8169000f, 3.6884650f},
  {201.6569000f, 0.0000000f},
};
static constexpr size_t count = 5;
// Interpolate inside measured support; use the nearest end segment outside.
inline float angleAt(const Point* p, float x, float* gain = nullptr) {
  size_t i = 0;
  while (i + 2 < count && x > p[i+1].x) ++i;
  const float slope = (p[i+1].angle-p[i].angle)/(p[i+1].x-p[i].x);
  if (gain) *gain = -slope;
  return p[i].angle + slope*(x-p[i].x);
}
} // namespace foot_calibration

// A bounded operating tolerance, not newly measured calibration support.
// angleAt already extends the nearest measured segment without clipping.
namespace foot_control_range {
constexpr float kExtensionDeg=5.f;
struct Assessment {bool measured=false,accepted=false;float extension_deg=NAN;};
inline Assessment assess(float x,bool right){
  Assessment out;if(!std::isfinite(x))return out;
  const auto* p=right?foot_calibration::right:foot_calibration::left;
  out.measured=x>=p[0].x && x<=p[foot_calibration::count-1].x;
  const float angle=foot_calibration::angleAt(p,x);
  if(!std::isfinite(angle))return out;
  const float edge=x<p[0].x?p[0].angle:p[foot_calibration::count-1].angle;
  out.extension_deg=out.measured?0.f:std::fabs(angle-edge);
  out.accepted=out.extension_deg<=kExtensionDeg+1e-5f;
  return out;
}
inline bool allows(bool measured,float x,bool right){return measured || assess(x,right).accepted;}
}

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
