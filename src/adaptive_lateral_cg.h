#pragma once
#include "lateral_cg_energy.h"

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
