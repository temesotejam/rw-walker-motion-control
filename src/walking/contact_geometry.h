#pragma once
#include <cmath>
#include <cstdint>
#include "realtime_code.h"

// Foot and body angles -> relative sole heights, geometric transfer point and
// travel to the receiving arc. Equal heights indicate possible contact;
// how much weight the new foot carries is not measured by this model.
// The controller uses the nominal rocker below for peak angle <-> energy.
namespace lateral_energy {
constexpr float kRadPerDeg = 0.01745329251994329577f;
struct Geometry {
  float mass_g, height, radius, inner_x, center, inner_angle, outer_angle;
  Geometry(float m=.1997f, float gravity=9.80665f, float h=.120f,
           float r=.150f, float x=.005f, float outer=.045f)
      : mass_g(m*gravity),height(h),radius(r),inner_x(x),
        center(std::sqrt(r*r-x*x)),inner_angle(std::asin(x/r)),
        outer_angle(std::asin(outer/r)) {}
};
// Stable height differences; shape-only constants are prepared once.
static inline RW_SPEED_CODE float potential(const Geometry& g, float degrees) {
  const float raw=std::fabs(degrees)*kRadPerDeg;
  if (!std::isfinite(raw) || raw>g.outer_angle+1e-6f || !(g.mass_g>0)) return NAN;
  const float a=std::fmin(raw,g.outer_angle);
  const float half_sine=std::sin(.5f*a);
  const float dh=a<=g.inner_angle
      ? -2.f*g.height*half_sine*half_sine+g.inner_x*std::sin(a)
      : (g.radius-g.center)+2.f*(g.center-g.height)*half_sine*half_sine;
  return g.mass_g*dh;
}
static inline RW_SPEED_CODE float angle(const Geometry& g, float energy) {
  if (!std::isfinite(energy) || energy<0 || !(g.mass_g>0)) return NAN;
  if (energy==0) return 0;
  const float boundary=potential(g,g.inner_angle/kRadPerDeg);
  const float y=energy/g.mass_g;
  float a;
  if (energy<=boundary) {
    const float disc=g.inner_x*g.inner_x-(2.f*g.height+y)*y;
    if (disc<0) return NAN;
    const float t=y/(g.inner_x+std::sqrt(disc));
    a=2.f*std::atan(t);
  } else {
    const float term=(y-(g.radius-g.center))/(2.f*(g.center-g.height));
    if (!(term>=0 && term<=1)) return NAN;
    a=2.f*std::asin(std::sqrt(term));
  }
  return a<=g.outer_angle+1e-6f ? a/kRadPerDeg : NAN;
}
}  // namespace lateral_energy

namespace support_transfer_plan {
constexpr float kRad = lateral_energy::kRadPerDeg;
constexpr float kMassKg = .1997f;
constexpr float kGravity = 9.80665f;
constexpr float kCgAboveHingeMm = 37.174236f;
constexpr float kExitDeg = 1.f;  // Travel into the receiving sole after equality.
constexpr float kCarryDps = 5.f;  // Trial residual forward-speed target at that event.
constexpr uint16_t kForecastLimitMs = 150;  // Validity of local motion extrapolation.
constexpr uint16_t kReplanMs = 25;

struct Input {
  int side = 0;  // Next body peak, NOT the departing peak or raw motor polarity.
  float angle_deg = NAN, rate_dps = NAN, accel_dps2 = NAN;
  float fore_aft_deg = NAN, left_deg = NAN, right_deg = NAN;
  float left_rate_dps = NAN, right_rate_dps = NAN;
  float initial_current_ma = NAN, goal_current_ma = NAN;  // Aligned to side.
  uint16_t cap_ms = 0;
};

namespace detail {
struct Contact {float x_mm, z_mm;};
struct Pose {Contact left, right; float cg_x_mm, cg_z_mm;};

// Exact bounded minimum of the circular arc reconstructed in
// analysis/measurement/analyze_energy_neutral_v14.py. Minimize z_world analytically over
// the 150 mm circle and the two longitudinal sole edges, skipping mesh scans.
inline Contact contact(bool right, float foot_pitch_deg, float s, float c) {
  const float cx = right ? .997904f : -1.002092f;
  const float a = foot_pitch_deg * kRad, ca = std::cos(a), sa = std::sin(a);
  const float cy = sa >= 0 ? -28.f : 27.f;
  const float tangent = 150.f * s / std::sqrt(s*s+c*c*ca*ca);
  const float u = right ? std::fmax(5.f,std::fmin(45.f,tangent))
                        : std::fmax(-45.f,std::fmin(-5.f,tangent));
  const float x = cx + u;
  const float z = sa*cy + ca*(67.09089f-std::sqrt(22500.f-u*u));
  return {c*x+s*z, -s*x+c*z};
}

inline Pose pose(float left, float right, float pitch, float roll) {
  const float s = std::sin(roll*kRad), c = std::cos(roll*kRad);
  const float cg = kCgAboveHingeMm*std::cos(pitch*kRad);
  return {contact(false,left+pitch,s,c), contact(true,right+pitch,s,c), s*cg, c*cg};
}

inline float gap(const Pose& p) {return p.left.z_mm-p.right.z_mm;}
inline float height(const Pose& p) {return p.cg_z_mm-std::fmin(p.left.z_mm,p.right.z_mm);}

inline Pose forecast(const Input& in, float ms) {
  const float t = ms*.001f;
  return pose(in.left_deg+in.left_rate_dps*t, in.right_deg+in.right_rate_dps*t,
              in.fore_aft_deg, in.angle_deg+in.rate_dps*t);
}

inline float withinAngle(float angle, float rate, float bound, float limit_ms) {
  if (rate == 0) return limit_ms;
  const float edge = std::copysign(bound,rate);
  return std::fmin(limit_ms,1000.f*(edge-angle)/rate);
}

// Nominal event clock uses current body rate and observed foot velocities.
// Energy feasibility uses observed acceleration separately; even a predicted
// coast stall can therefore have an event to plan towards. Candidate work
// does not retime the nominal foot path within this single shooting step.
inline bool findTransfer(const Input& in, float& crossing_ms, Pose& at, Pose& beyond) {
  const float exit_ms = 1000.f*kExitDeg/std::fabs(in.rate_dps);
  float limit = withinAngle(in.angle_deg,in.rate_dps,15.f,kForecastLimitMs);
  limit = withinAngle(in.left_deg,in.left_rate_dps,20.f,limit);
  limit = withinAngle(in.right_deg,in.right_rate_dps,20.f,limit);
  const float last = limit-exit_ms;
  if (!(last>0) || in.side*gap(forecast(in,0)) >= 0) return false;
  float lo = 0;
  while (lo < last) {
    float hi = std::fmin(lo+kReplanMs,last);
    if (in.side*gap(forecast(in,hi)) >= 0) {
      // 25 ms maximum bracket, uncertainty below 0.1 ms for the assumed path.
      for (unsigned i=0;i<8;++i) {
        const float mid = .5f*(lo+hi);
        if (in.side*gap(forecast(in,mid)) >= 0) hi=mid; else lo=mid;
      }
      crossing_ms=hi;
      at=forecast(in,hi);
      beyond=forecast(in,hi+exit_ms);
      return in.side*gap(beyond)>0;
    }
    lo=hi;
  }
  return false;
}

inline bool validGeometry(const Input& in) {
  return (in.side==1 || in.side==-1) && std::fabs(in.angle_deg)<=15 &&
    std::fabs(in.fore_aft_deg)<=10 && std::fabs(in.left_deg)<=20 && std::fabs(in.right_deg)<=20;
}

}  // namespace detail

inline float supportGap(float left_deg,float right_deg,float fore_aft_deg,float angle_deg) {
  return detail::gap(detail::pose(left_deg,right_deg,fore_aft_deg,angle_deg));
}

// At the inner lateral edge the unconstrained tangent of the circular sole
// reaches |u|=5 mm (radius 150 mm). This event is defined by the receiving
// foot's CAD arc and pitch, independently of body ZERO and load sensing.
inline float arcEntryDeg(int side, float receiving_pitch_deg) {
  const float ca=std::cos(receiving_pitch_deg*kRad);
  return side*std::atan2(5.f*ca,std::sqrt(22500.f-25.f))/kRad;
}

}  // namespace support_transfer_plan
