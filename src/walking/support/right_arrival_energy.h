#pragma once
#include "models/lateral_energy_control.h"

// Effective arrival potential after a right-going ordinary mechanical ZERO.
// Source: the eight preceding 2026-11-20 runs, independent of the two A/B runs.
namespace right_arrival_energy {
enum class Reason : uint8_t { NotEvaluated=0, Invalid=1, OutsideSupport=2, Kept=3, Reduced=4, Coast=5 };
struct Result {
  bool eligible=false;
  uint16_t width_ms=0;
  float coast_j=NAN, original_j=NAN, selected_j=NAN, target_j=NAN;
  Reason reason=Reason::NotEvaluated;
};

// Piecewise envelope inside the convex hull of observed (rate, command Q).
// Every charge from coast up to this limit has interpolation support.
inline float maximumCharge(float rate_dps) {
  static constexpr float rate[]={21.f,36.f,40.f,47.f,51.f};
  static constexpr float cap[]={12.4f,11.f,8.5f,3.3f,0.f};
  if (!std::isfinite(rate_dps) || rate_dps<rate[0] || rate_dps>rate[4]) return NAN;
  for (unsigned i=1;i<5;++i) {
    if (rate_dps<=rate[i]) {
      const float fraction=(rate_dps-rate[i-1])/(rate[i]-rate[i-1]);
      return cap[i-1]+fraction*(cap[i]-cap[i-1]);
    }
  }
  return NAN;
}

inline float predictedJ(float rate_dps,float charge_mas) {
  const float cap=maximumCharge(rate_dps);
  if (!std::isfinite(cap) || !std::isfinite(charge_mas) || charge_mas<0.f || charge_mas>cap) return NAN;
  const float omega=rate_dps*lateral_energy::kRadPerDeg;
  return .000152963884f+.000465018868f*omega*omega+.0000934855564f*omega*charge_mas;
}

inline Result limit(const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
                    float rate_dps,float incoming_peak_deg,float target_deg,float initial_ma,
                    float goal_ma,uint16_t guarded_ms) {
  Result out;
  out.width_ms=guarded_ms;
  if (!current.ready || guarded_ms>lateral_energy::kMaxMs || !std::isfinite(initial_ma) ||
      !std::isfinite(goal_ma) || goal_ma<=0.f || !std::isfinite(incoming_peak_deg) || !std::isfinite(target_deg)) {
    out.reason=Reason::Invalid; return out;
  }
  const float original_q=guarded_ms ? lateral_energy::charge(current,guarded_ms,initial_ma,goal_ma) : 0.f;
  out.coast_j=predictedJ(rate_dps,0.f);
  out.original_j=predictedJ(rate_dps,original_q);
  if (incoming_peak_deg<2.13f || incoming_peak_deg>6.f || target_deg<4.5f || target_deg>8.f ||
      !std::isfinite(out.original_j)) {
    out.reason=Reason::OutsideSupport; return out;
  }
  out.target_j=lateral_energy::potential(geometry,target_deg);
  if (!std::isfinite(out.target_j)) {out.reason=Reason::Invalid;return out;}
  out.eligible=true;
  out.selected_j=out.original_j;
  out.reason=Reason::Kept;
  if (!guarded_ms || out.original_j<=out.target_j) return out;

  const float j_per_mas=.0000934855564f*rate_dps*lateral_energy::kRadPerDeg;
  const float target_q=(out.target_j-out.coast_j)/j_per_mas;
  out.width_ms=0;
  out.selected_j=out.coast_j;
  if (target_q>0.f) {
    uint16_t lo=0,hi=guarded_ms;
    while (hi-lo>1) {
      const uint16_t mid=(lo+hi)/2;
      if (lateral_energy::charge(current,mid,initial_ma,goal_ma)<target_q) lo=mid; else hi=mid;
    }
    for (uint16_t width:{lo,hi}) {
      const float q=width ? lateral_energy::charge(current,width,initial_ma,goal_ma) : 0.f;
      if (q<0.f) continue;
      const float j=out.coast_j+j_per_mas*q;
      if (std::fabs(j-out.target_j)<std::fabs(out.selected_j-out.target_j)) {
        out.width_ms=width;out.selected_j=j;
      }
    }
  }
  out.reason=out.width_ms==guarded_ms ? Reason::Kept : out.width_ms ? Reason::Reduced : Reason::Coast;
  return out;
}
}
