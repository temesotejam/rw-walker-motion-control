#pragma once
#include <cmath>
#include <cstdint>
#include "lateral_energy_control.h"
#include "work_target_policy.h"

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

namespace lateral_cg {
// Prior fitted to the supported 10-30s observations in run 124463989.
// These are model-relative equivalent parameters, not a physical CG survey.
constexpr float kAdaptiveSeedMm=-.20587803f;
constexpr float kAdaptiveSeedBiasMj=.02262225f;
struct AdaptiveSnapshot {
  bool enabled=false;
  float initial_mm=0,shift_mm=0,common_bias_mj=0;
  uint32_t pairs=0,updates=0,rejected=0;
};
class Adaptive {
 public:
  void reset(float initial_mm=kAdaptiveSeedMm,float initial_bias_mj=kAdaptiveSeedBiasMj) {
    *this=Adaptive{};
    state_.enabled=true;
    state_.initial_mm=state_.shift_mm=std::isfinite(initial_mm) ?
        std::fmax(-kMaxShiftMm,std::fmin(kMaxShiftMm,initial_mm)) : kAdaptiveSeedMm;
    state_.common_bias_mj=std::isfinite(initial_bias_mj) ? std::fmax(-.25f,std::fmin(.25f,initial_bias_mj)) : 0.f;
  }
  const AdaptiveSnapshot& snapshot() const { return state_; }
  void breakPair() { pending_[0].valid=pending_[1].valid=false; }
  // r = U0(measured peak)-raw predicted arrival energy = b-k*dx.
  // k is signed mJ/mm. Learn only after both adjacent sides have arrived.
  bool observe(const lateral_energy::Geometry& g,int8_t side,float amplitude_deg,
      float raw_prediction_mj,uint32_t time_ms) {
    if (!state_.enabled || (side!=1 && side!=-1) || !std::isfinite(amplitude_deg) ||
        amplitude_deg<6.f || amplitude_deg>13.f || !std::isfinite(raw_prediction_mj) ||
        raw_prediction_mj<0.f || time_ms<5000 || time_ms<=last_ms_) return reject();
    last_ms_=time_ms;
    const float residual=lateral_energy::potential(g,amplitude_deg)*1000.f-raw_prediction_mj;
    if (!std::isfinite(residual) || std::fabs(residual)>.5f) return reject();
    const unsigned i=side>0 ? 0 : 1;
    pending_[i]={g.mass_g*std::sin(side*amplitude_deg*lateral_energy::kRadPerDeg),residual,time_ms,true};
    if (!pending_[1-i].valid) return false;
    const uint32_t other=pending_[1-i].time;
    if (time_ms-other>1400) { pending_[1-i].valid=false;return reject(false); }
    for(unsigned j=0;j<2;++j) points_[2*next_+j]=pending_[j];
    pending_[0].valid=pending_[1].valid=false;
    next_=(next_+1)%8;
    if (count_<8) ++count_;
    ++state_.pairs;
    if (count_<3) return false;
    float sk=0,sr=0,skk=0,skr=0;
    const unsigned n=2*count_;
    for(unsigned j=0;j<n;++j) {
      const auto& p=points_[j];sk+=p.k;sr+=p.r;skk+=p.k*p.k;skr+=p.k*p.r;
    }
    const float denominator=skk-sk*sk/n;
    if (!(denominator>.02f)) return reject(false);
    const float estimate=-(skr-sk*sr/n)/denominator;
    const float shift=std::fmax(-kMaxShiftMm,std::fmin(kMaxShiftMm,estimate));
    const float bias=std::fmax(-.25f,std::fmin(.25f,(sr+shift*sk)/n));
    state_.shift_mm+=std::fmax(-.025f,std::fmin(.025f,.2f*(shift-state_.shift_mm)));
    state_.common_bias_mj+=std::fmax(-.01f,std::fmin(.01f,.2f*(bias-state_.common_bias_mj)));
    ++state_.updates;return true;
  }
 private:
  struct Point {
    float k=0,r=0;uint32_t time=0;bool valid=false;
    Point()=default;
    Point(float k0,float r0,uint32_t t0,bool v0):k(k0),r(r0),time(t0),valid(v0) {}
  };
  bool reject(bool clear=true) {
    ++state_.rejected;
    if(clear) pending_[0].valid=pending_[1].valid=false;
    return false;
  }
  AdaptiveSnapshot state_;
  Point pending_[2],points_[16];
  unsigned next_=0,count_=0;
  uint32_t last_ms_=0;
};
} // namespace lateral_cg

namespace wheel_work_energy_control {
// Store U0(observed)-Eraw and its signed CG coefficient. Re-express the
// residual using the CURRENT CG estimate, so a paired CG update is not added
// a second time to an already-corrected model prediction. All values are mJ.
class SideEnergyFeedback {
 public:
  unsigned count() const { return count_; }
  __attribute__((noinline)) bool observe(const lateral_energy::Geometry& g, int side,
      float amplitude_deg, float raw_mj,float target_deg=8.f,
      float observed_cg_mm=0.f,float applied_bias_mj=0.f) {
    if ((side!=1 && side!=-1) || !std::isfinite(amplitude_deg) ||
        amplitude_deg<6.f || !std::isfinite(raw_mj) || raw_mj<0.f ||
        !work_target_policy::valid(target_deg)) return false;
    const float limit=.5f*std::fmax(1.f,lateral_energy::potential(g,target_deg)/lateral_energy::potential(g,8.f));
    const float residual=1000.f*lateral_energy::potential(g,amplitude_deg)-raw_mj;
    if(side==1 && target_deg==12.f && std::isfinite(residual) &&
        std::isfinite(observed_cg_mm) && std::isfinite(applied_bias_mj)) {
      coast_only_=amplitude_deg>work_target_policy::observationCeiling(target_deg) || std::fabs(residual)>limit;
      const float coefficient=g.mass_g*std::sin(amplitude_deg*lateral_energy::kRadPerDeg);
      const float error=residual+coefficient*observed_cg_mm-applied_bias_mj;
      const float tolerance=1000.f*(lateral_energy::potential(g,target_deg+.5f)-lateral_energy::potential(g,target_deg));
      underestimated_=error>tolerance;
      guard_residual_=residual;guard_coefficient_=coefficient;
    }
    if(amplitude_deg>work_target_policy::observationCeiling(target_deg)) return false;
    if (!std::isfinite(residual) || std::fabs(residual)>limit) return false;
    limit_mj_=limit;
    residual_[next_]=residual;
    coefficient_[next_]=g.mass_g*std::sin(side*amplitude_deg*lateral_energy::kRadPerDeg);
    next_=(next_+1)%3;if(count_<3) ++count_;
    return true;
  }
  __attribute__((noinline)) float biasMj(float cg_mm,float fallback_mj) const {
    if (count_<2 || !std::isfinite(cg_mm)) return fallback_mj;
    float sum=0;
    for(unsigned i=0;i<count_;++i) sum+=residual_[i]+coefficient_[i]*cg_mm;
    return std::fmax(-limit_mj_,std::fmin(limit_mj_,sum/count_));
  }
  // Use the latest same-model residual immediately after a large positive
  // underprediction. MAX avoids adding it a second time to the running mean.
  float positiveBiasMj(float cg_mm,float fallback_mj,float target_deg) const {
    const float average=biasMj(cg_mm,fallback_mj);
    if(target_deg!=12.f || !underestimated_ || !std::isfinite(cg_mm)) return average;
    const float latest=std::fmax(-limit_mj_,std::fmin(limit_mj_,guard_residual_+guard_coefficient_*cg_mm));
    return std::fmax(average,latest);
  }
  bool coastOnly() const { return coast_only_; }
 private:
  float residual_[3]={},coefficient_[3]={};
  unsigned count_=0,next_=0;
  float limit_mj_=.5f;
  float guard_residual_=0.f,guard_coefficient_=0.f;
  bool underestimated_=false,coast_only_=false;
};
}
