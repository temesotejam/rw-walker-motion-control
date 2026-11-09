#pragma once
#include <cmath>
#include <cstdint>
#include "lateral_energy_control.h"

namespace lateral_cg {
constexpr uint32_t kMagic=0x43475831;
constexpr float kMaxShiftMm=2.f;
struct Record {
  uint32_t magic=0,version=1,revision=0,upright_revision=0,source=0;
  float shift_mm=0,upright_abs_deg=0,equilibrium_abs_deg=0;
};
inline bool valid(const Record& r) {
  return r.magic==kMagic && (r.version==1 || r.version==2) && r.revision>0 &&
      r.upright_revision>0 && (r.source==1 || r.source==2) &&
      std::isfinite(r.shift_mm) && std::fabs(r.shift_mm)<=kMaxShiftMm &&
      std::isfinite(r.upright_abs_deg) && std::isfinite(r.equilibrium_abs_deg);
}
inline bool matches(const Record& r,uint32_t revision,float pitch) {
  return valid(r) && r.upright_revision==revision &&
      std::fabs(r.upright_abs_deg-pitch)<.0001f;
}
inline bool matchesAbsolute(const Record& r) {
  return valid(r) && r.version==2 && r.upright_abs_deg==0.f;
}
// Positive shift raises the CG when the signed body angle increases.
// All energies are J, referred to geometric upright, with U(0)=0.
inline float potential(const lateral_energy::Geometry& g,float signed_deg,float shift_mm) {
  if (!std::isfinite(shift_mm) || std::fabs(shift_mm)>kMaxShiftMm) return NAN;
  const float base=lateral_energy::potential(g,signed_deg);
  if (shift_mm==0.f) return base;
  return base+g.mass_g*shift_mm*.001f*std::sin(signed_deg*lateral_energy::kRadPerDeg);
}
// First outward turning point from the centre. Prefer the rising inner branch
// if an energy barrier exists; otherwise use the outward circular-sole root.
inline float angle(const lateral_energy::Geometry& g,float energy,int8_t side,float shift_mm) {
  if ((side!=1 && side!=-1) || !std::isfinite(energy) || energy<0 ||
      !std::isfinite(shift_mm) || std::fabs(shift_mm)>kMaxShiftMm) return NAN;
  if (shift_mm==0.f) return lateral_energy::angle(g,energy);
  if (energy==0.f) return 0.f;
  const float y=energy/g.mass_g;
  const float b=static_cast<float>(side)*shift_mm*.001f;
  const float inner_x=g.inner_x+b;
  const float disc=inner_x*inner_x-(2.f*g.height+y)*y;
  if (inner_x>0 && disc>=0) {
    const float a=2.f*std::atan(y/(inner_x+std::sqrt(disc)));
    if (a>=0 && a<=g.inner_angle+1e-6f) return a/lateral_energy::kRadPerDeg;
  }
  const float a0=g.center-g.height;
  const float length=std::sqrt(a0*a0+b*b);
  const float c=(g.radius-g.height-y)/length;
  if (c < -1.f-1e-6f || c > 1.f+1e-6f) return NAN;
  const float a=std::acos(std::fmax(-1.f,std::fmin(1.f,c)))-std::atan2(b,a0);
  return a>=g.inner_angle-1e-5f && a<=g.outer_angle+1e-6f ?
      a/lateral_energy::kRadPerDeg : NAN;
}
// A finite contact patch or restraint does not identify x from a static angle.
// Admit only a stable equilibrium on the known circular rolling branch.
inline bool fromEquilibrium(const lateral_energy::Geometry& g,float relative_deg,float& shift_mm) {
  if (!std::isfinite(relative_deg)) return false;
  const float a=relative_deg*lateral_energy::kRadPerDeg;
  if (std::fabs(a)<=g.inner_angle || std::fabs(a)>=g.outer_angle) return false;
  const float x=-(g.center-g.height)*std::tan(a)*1000.f;
  if (!std::isfinite(x) || std::fabs(x)>kMaxShiftMm) return false;
  shift_mm=x;return true;
}
bool load(Record& record);
bool save(const Record& record);
} // namespace lateral_cg
