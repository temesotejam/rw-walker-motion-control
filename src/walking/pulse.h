#pragma once
#include <cstdint>
#include "models/lateral_energy_control.h"
#include <cmath>

// Both factors are ±1, so this factor converts logical excitation to the
// Roller register and converts its signed current reading back to excitation.
constexpr int walkingCurrentPolarity(int normal_output_sign, int driver_polarity) {
  return normal_output_sign * driver_polarity;
}

namespace walking_positive_cap {
// Scope: only the otherwise-uncapped positive fallback of target-8 work mode.
// All other selectors retain their own established limits.
inline uint16_t fallback(bool work_mode,bool work_choice_valid,int side,float target_deg,
                         uint16_t gradual_cap,uint16_t wheel_cap,uint16_t maximum) {
  if(!work_mode || work_choice_valid || side<=0 || target_deg!=8.f)return maximum;
  uint16_t result=gradual_cap<wheel_cap ? gradual_cap : wheel_cap;
  return result<maximum ? result : maximum;
}
}

// Post-command prediction using actual elapsed SOFTWARE time. The current
// rise and inherited pulse-Q arrival response remain empirical models, not a
// measurement of delivered charge/work or identified physical body inertia.
namespace energy_pulse_completion {
constexpr char kRevision[]="actual_elapsed_pulse_completion_v16";
struct Input {
  int8_t side=0;
  float rate_dps=NAN,initial_current_ma=NAN,goal_current_ma=NAN,tau_s=NAN;
  float available_j=NAN,request_j=NAN;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
};
struct Result {
  // valid covers inputs and finite current/charge arithmetic. A negative net
  // signed Q is retained for audit but does NOT have a valid work prediction.
  bool valid=false,work_valid=false,duration_completed=false;
  bool width_supported=false,rate_supported=false,charge_supported=false,extrapolated=false;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
  float elapsed_s=NAN,q_mas=NAN,current_end_ma=NAN;
  float work_j=NAN,arrival_j=NAN,unmet_j=NAN;
};
inline Result evaluate(const Input& in){
  Result out;out.actual_elapsed_ms=in.actual_elapsed_ms;out.planned_ms=in.planned_ms;
  out.duration_completed=in.planned_ms>0 && in.actual_elapsed_ms>=in.planned_ms;
  // This flag describes model coverage, never an integration clamp. A planned
  // or actual duration above the historical 100ms range remains fully counted.
  out.width_supported=in.planned_ms>0 && in.planned_ms<=lateral_energy::kMaxMs &&
      in.actual_elapsed_ms<=lateral_energy::kMaxMs;
  if((in.side!=1 && in.side!=-1) || !std::isfinite(in.rate_dps) || in.rate_dps<0 ||
      !std::isfinite(in.initial_current_ma) || !std::isfinite(in.goal_current_ma) || in.goal_current_ma<=0 ||
      !std::isfinite(in.tau_s) || in.tau_s<=0 || !std::isfinite(in.available_j) || in.available_j<0 ||
      !std::isfinite(in.request_j) || in.request_j<0)return out;
  const auto c=lateral_energy::coefficients(in.side);
  out.rate_supported=in.rate_dps>=c.rate_min_dps && in.rate_dps<=c.rate_max_dps;
  const double seconds=static_cast<double>(in.actual_elapsed_ms)*.001;
  const double tail=std::exp(-seconds/in.tau_s);
  const double rise=-std::expm1(-seconds/in.tau_s);
  const double q=in.goal_current_ma*seconds+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*in.tau_s*rise;
  out.elapsed_s=static_cast<float>(seconds);
  out.q_mas=in.actual_elapsed_ms?static_cast<float>(q):0.f;
  out.current_end_ma=in.actual_elapsed_ms?static_cast<float>(in.goal_current_ma+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*tail):in.initial_current_ma;
  if(!std::isfinite(out.q_mas) || !std::isfinite(out.current_end_ma))return out;
  out.valid=true;
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.width_supported || !out.rate_supported || !out.charge_supported;
  if(out.q_mas<0){out.extrapolated=true;return out;}
  out.work_j=in.actual_elapsed_ms?
      lateral_energy::work(c,in.rate_dps*lateral_energy::kRadPerDeg,out.q_mas):0.f;
  if(!std::isfinite(out.work_j) || out.work_j<0){out.work_j=NAN;out.extrapolated=true;return out;}
  out.arrival_j=in.available_j+out.work_j;
  out.unmet_j=std::fmax(0.f,in.request_j-out.work_j);
  if(!std::isfinite(out.arrival_j) || !std::isfinite(out.unmet_j)){
    out.work_j=out.arrival_j=out.unmet_j=NAN;out.extrapolated=true;return out;
  }
  out.work_valid=true;return out;
}
}

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
