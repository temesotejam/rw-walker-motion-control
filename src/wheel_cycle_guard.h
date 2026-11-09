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
