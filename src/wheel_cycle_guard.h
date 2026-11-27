#pragma once
#include <cmath>
#include <cstdint>

// Experimental: preserve the early alternating wheel motion. A pair of
// fresh pre-input observations estimates its midpoint, not ground motion.
// Only shorten the pulse that would push an already biased wheel farther in
// the same direction; do not add torque, reverse torque or force a stop.
namespace wheel_cycle_guard {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  // Four alternating, command-matched peaks form two observed pairs. Any
  // invalid, stale, weak or falling peak immediately removes extra action.
  void __attribute__((noinline)) peak(int side,float angle,uint32_t ms,bool matched) {
    if(!matched || !std::isfinite(angle) || angle<0.f || !ms || (side!=1 && side!=-1)) {
      clearPeaks();return;
    }
    if(peak_count_ && (side==peak_side_[peak_count_-1] || ms<=peak_ms_ || ms-peak_ms_>1500))
      clearPeaks();
    if(peak_count_==4) {
      for(int i=0;i<3;++i){peak_side_[i]=peak_side_[i+1];peak_angle_[i]=peak_angle_[i+1];}
      peak_count_=3;
    }
    peak_side_[peak_count_]=side;peak_angle_[peak_count_]=angle;++peak_count_;peak_ms_=ms;
  }
  uint16_t legacyWidth() const {return legacy_width_;}
  uint16_t extraReduction() const {return extra_reduction_;}
  bool trialEligible() const {return trial_eligible_;}
  uint16_t limit(int side, float rpm, bool valid, uint32_t ms, uint16_t width) {
    legacy_width_=width;extra_reduction_=0;trial_eligible_=false;
    if (!valid || !std::isfinite(rpm) || !ms || (side!=1 && side!=-1)) {
      reset(); return width;
    }
    float midpoint=0.f; bool paired=false;
    if (last_side_==-side && ms>last_ms_ && ms-last_ms_<=1500) {
      midpoint=.5f*(last_rpm_+rpm); paired=true;
    }
    last_side_=side; last_rpm_=rpm; last_ms_=ms;
    if (!paired || !width) return width;
    const float bias=midpoint*side;
    if(bias>150.f) {
      const float strength=std::fmin(1.f,(bias-150.f)/250.f);
      legacy_width_=static_cast<uint16_t>(std::ceil(width*(1.f-.25f*strength)));
    }
    trial_eligible_=qualified(side,ms);
    if(!trial_eligible_ || bias<=100.f) return legacy_width_;
    const float early=std::fmin(1.f,(bias-100.f)/250.f);
    const uint16_t candidate=static_cast<uint16_t>(std::ceil(width*(1.f-.25f*early)));
    uint16_t reduced=candidate<legacy_width_ ? candidate : legacy_width_;
    if(legacy_width_-reduced>5)reduced=legacy_width_-5;
    extra_reduction_=legacy_width_-reduced;
    return reduced;
  }
 private:
  void clearPeaks(){peak_count_=0;peak_ms_=0;}
  bool qualified(int side,uint32_t now) const {
    if(peak_count_!=4 || !peak_ms_ || now<peak_ms_ || now-peak_ms_>1500)return false;
    for(int i=0;i<4;++i)if(peak_angle_[i]<4.5f)return false;
    if(std::fabs(peak_angle_[0]-peak_angle_[1])>2.f ||
       std::fabs(peak_angle_[2]-peak_angle_[3])>2.f)return false;
    // A trial is disqualified if either side has recently fallen.
    if(peak_angle_[2]<peak_angle_[0]-.4f || peak_angle_[3]<peak_angle_[1]-.4f)return false;
    int last=-1,opposite=-1,previous=-1;
    for(int i=3;i>=0;--i) {
      if(peak_side_[i]==side){if(last<0)last=i;else if(previous<0)previous=i;}
      else if(opposite<0)opposite=i;
    }
    if(last<0 || previous<0 || opposite<0)return false;
    // Do not add shortening to the already weaker or declining body side.
    return peak_angle_[last]>=peak_angle_[opposite]-.3f &&
           peak_angle_[last]>=peak_angle_[previous]-.4f;
  }
  float last_rpm_=NAN;
  uint32_t last_ms_=0;
  int last_side_=0;
  float peak_angle_[4]={}; int8_t peak_side_[4]={};uint8_t peak_count_=0;
  uint32_t peak_ms_=0;uint16_t legacy_width_=0,extra_reduction_=0;
  bool trial_eligible_=false;
};
}

// Experimental bias guard: compare successive, fresh pre-input speeds.
// This limits an existing excitation; it never commands reverse torque.
namespace wheel_speed_balance {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  void __attribute__((noinline)) peak(int side,float angle) {
    if(!std::isfinite(angle)) return;
    if(side>0) positive_peak_=angle; else if(side<0) negative_peak_=angle;
  }
  void __attribute__((noinline)) crossing(int side,float rpm,bool fresh,uint32_t ms) {
    if(!fresh || !std::isfinite(rpm)) { reset(); return; }
    if(side<0) { negative_rpm_=rpm;negative_ms_=ms;have_negative_=true;return; }
    if(side!=1) return;
    if(!have_negative_ || ms<=negative_ms_ || ms-negative_ms_>1500) { cap_=100;biased_=0;have_negative_=false;return; }
    // Both crossings rotating in the same direction indicates sustained
    // momentum, including states beyond the negative-side model's domain.
    // Reduce only the positive excitation, slowly; never invent brake torque.
    const bool sustained=negative_rpm_>=400.f && rpm>=150.f;
    if(sustained) {
      if(++biased_>=2) {if(cap_>60)cap_-=5;biased_=0;}
      have_negative_=false;return;
    }
    if(!std::isfinite(positive_peak_) || positive_peak_<5.f ||
       !std::isfinite(negative_peak_) || negative_peak_<4.f) { cap_=100;biased_=0;have_negative_=false;return; }
    const bool biased=negative_rpm_>=250.f && rpm>=-20.f && rpm<=150.f && negative_rpm_-rpm>=250.f;
    if(biased) { if(++biased_>=2) {if(cap_>80)cap_-=5;biased_=0;} }
    else {biased_=0;if(cap_<100)cap_+=5;}
    have_negative_=false;
  }
  uint16_t capMs() const {return cap_;}
 private:
  float negative_rpm_=NAN,positive_peak_=NAN,negative_peak_=NAN;
  uint32_t negative_ms_=0;uint16_t cap_=100;unsigned biased_=0;bool have_negative_=false;
};
}
