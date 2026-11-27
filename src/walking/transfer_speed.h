#pragma once
#include "walking/contact_geometry.h"
#include "walking/coast_motion.h"
#include "models/lateral_energy_control.h"

// The active controller reuses measured leftward ZERO speeds that reached the
// receiving arc, and a rightward ZERO-to-next-peak fit. A bilateral regression
// for speed immediately after actual load transfer has not been identified.
// Point-mass pivot projection is used only by the diagnostic planner.
namespace return_crossing_work {
struct Reference {
  uint32_t zero_ms=0, peak_ms=0;
  float speed_dps=0, target_j=0;
  void observe(uint32_t zero,uint32_t peak,float magnitude,float speed,float target,bool completed) {
    *this={};
    const auto fit=walking_energy_policy::passiveCoefficients(-1);
    // Flat feet give the largest absolute CAD entry angle. A peak beyond that
    // angle crossed the geometric threshold for any pitch in the model.
    if (completed && zero && peak>zero && peak-zero<=1500 &&
        std::isfinite(magnitude) && magnitude>=-support_transfer_plan::arcEntryDeg(-1,0) &&
        speed>=fit.rate_min_dps && speed<=fit.rate_max_dps && std::isfinite(target) && target>0) {
      zero_ms=zero;peak_ms=peak;speed_dps=speed;target_j=target;
    }
  }
};

inline bool usableReference(const Reference& source,uint32_t departure,uint32_t now,float target) {
  return source.zero_ms && source.peak_ms<departure && now>departure && now-departure<=1500 &&
      now-source.peak_ms<=4000 && std::isfinite(target) && std::fabs(source.target_j-target)<=1.e-9f;
}

#if SUPPORT_TRANSFER_STALL_V3
// Recent leftward centre passages that subsequently reached the receiving
// sole's CAD entry. The speed was sampled before that particular main pulse.
struct MainPassage {
  uint32_t zero_ms,peak_ms;
  float speed_dps,target_j,main_j;
  uint16_t main_ms;
};
struct MainPassages {
  MainPassage recent[3]{};

  void observe(uint32_t zero,uint32_t peak,float magnitude,float speed,float target,
               uint32_t elapsed,float work,bool completed) {
    if (!completed || !zero || peak<=zero || peak-zero>1500 || peak<=recent[0].peak_ms ||
        !(std::isfinite(magnitude) && magnitude>=-support_transfer_plan::arcEntryDeg(-1,0) &&
          speed>=support_transfer_plan::kCarryDps && speed<=walking_energy_policy::passiveCoefficients(-1).rate_max_dps &&
          std::isfinite(target) && target>0 && std::isfinite(work) && work>0 &&
          elapsed>0 && elapsed<=walking_energy_policy::kMaxPulseMs)) return;
    recent[2]=recent[1]; recent[1]=recent[0];
    recent[0]={zero,peak,speed,target,work,static_cast<uint16_t>(elapsed)};
  }
};
#endif
}  // namespace return_crossing_work

namespace support_transfer_plan {
namespace detail {
// The diagnostic planner projects incoming point-mass motion onto the new
// pivot's tangent. The two outputs scale incoming squared angular speed
// into energy and give the fraction of that energy left after projection.
struct PivotTransfer {
  float incoming_j_per_dps2;
  float retained_fraction;
};

inline bool projectPivotTransfer(const Pose& at, int side, PivotTransfer& result) {
  const auto from = side>0 ? at.left : at.right;
  const auto to = side>0 ? at.right : at.left;
  const float ax=at.cg_x_mm-from.x_mm, az=at.cg_z_mm-from.z_mm;
  const float bx=at.cg_x_mm-to.x_mm, bz=at.cg_z_mm-to.z_mm;
  const float pre_r2=ax*ax+az*az, post_r2=bx*bx+bz*bz;
  const float alignment=ax*bx+az*bz;
  if (!(alignment>0 && pre_r2>0 && post_r2>0)) return false;
  result.retained_fraction=std::fmin(1.f,alignment*alignment/(pre_r2*post_r2));
  result.incoming_j_per_dps2=.5f*kMassKg*pre_r2*1e-6f*kRad*kRad;
  return true;
}
}  // namespace detail
}  // namespace support_transfer_plan

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
