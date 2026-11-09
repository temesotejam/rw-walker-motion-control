#pragma once
#include <cmath>
#include <cstdint>

// Positive-side return that is almost stopped before reaching the centre.
// Observational heuristic: not proof that the next pulse restores motion.
namespace stall_rescue {
constexpr uint16_t kMaxPulseMs=15;
class Detector {
 public:
  void reset() { pending_ms_=peak_ms_=0; samples_=0; }
  bool observe(uint32_t now,uint32_t prior_ms,uint32_t peak_ms,
      float peak_deg,float prior_angle,float angle,float prior_rate,float rate) {
    const bool finite=std::isfinite(peak_deg) && std::isfinite(prior_angle) &&
      std::isfinite(angle) && std::isfinite(prior_rate) && std::isfinite(rate);
    if(!finite || !peak_ms || now<=prior_ms || now-prior_ms>10 || now<peak_ms ||
       now-peak_ms<200 || now-peak_ms>800 || peak_deg<5.f || angle<.35f ||
       angle>3.25f || peak_deg-angle<2.f || prior_angle+.03f<angle ||
       rate < -6.5f || rate>=-.1f || (peak_ms_ && peak_ms_!=peak_ms)) {
      reset();return false;
    }
    if(!pending_ms_) {
      if(prior_rate>=-6.f || rate< -6.f)return false;
      pending_ms_=now;peak_ms_=peak_ms;samples_=1;return false;
    }
    if(now-pending_ms_>30) {reset();return false;}
    ++samples_;
    if(samples_>=3 && now-pending_ms_>=6) {reset();return true;}
    return false;
  }
 private:
  uint32_t pending_ms_=0,peak_ms_=0;
  unsigned samples_=0;
};
}
