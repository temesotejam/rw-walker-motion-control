#pragma once
#include <cmath>
#include <cstdint>

// A bounded experimental command when ordinary negative-side selection is
// coast or unsupported. This is a heuristic, NOT an energy or gait-response
// prediction. The caller must report it separately from the fitted models,
// enforce all actuator/attitude/emergency guards, confirm the actual command,
// and retain negative-peak and wheel-speed observations for later evaluation.
namespace weak_side_assist {
struct Input {
  float target_preset_deg=NAN;      // operator's requested target
  float active_target_deg=NAN;      // currently ramped target, same frame as peaks
  int8_t physical_next_side=0;
  float last_negative_peak_deg=NAN;
  uint32_t last_negative_peak_ms=0;
  uint32_t now_ms=0;
  uint16_t negative_peak_count=0;   // actually confirmed in this run
  float crossing_rate_dps=NAN;
  float wheel_rpm=NAN;
  float battery_mv=NAN;
  float last_positive_peak_deg=NAN;
  bool hardware_healthy=false;      // includes no STOP, motor/IMU/roller readiness
  bool speed_fresh=false;           // actual measured, not modeled
  bool selector_valid=false;
  uint16_t selector_width_ms=0;
  uint16_t hardware_max_width_ms=0; // caller supplies tighter of existing limits
};
struct Choice {
  bool selected=false;
  uint16_t width_ms=0;
  int16_t current_ma=0;
  int8_t motion_sign=0;             // +1 relative to the zero-cross motion
  float deficit_deg=NAN;            // observation, NOT predicted response
};
// Keep the current at 300 mA. A pair of prior
// confirmed negative peaks prevents an initial seed/half step from masquerading
// as a developed weak side. No model-support cutoffs in rate, voltage, or rpm.
inline Choice choose(const Input& in) {
  Choice out;
  if(in.target_preset_deg!=8.f || in.physical_next_side!=-1 ||
     !in.hardware_healthy || !in.speed_fresh || in.negative_peak_count<2 ||
     !std::isfinite(in.active_target_deg) || in.active_target_deg<=0.f ||
     !std::isfinite(in.last_negative_peak_deg) || in.last_negative_peak_deg<=0.f ||
     !std::isfinite(in.last_positive_peak_deg) || in.last_positive_peak_deg<=0.f ||
     !std::isfinite(in.crossing_rate_dps) || in.crossing_rate_dps<=0.f ||
     !std::isfinite(in.wheel_rpm) || !std::isfinite(in.battery_mv) || in.battery_mv<=0.f ||
     !in.last_negative_peak_ms || in.now_ms<in.last_negative_peak_ms ||
     in.now_ms-in.last_negative_peak_ms>1500 ||
     in.last_negative_peak_deg>=16.f || in.last_positive_peak_deg>=16.f ||
     in.hardware_max_width_ms==0) return out;
  // A normal powered choice, whether excitation or braking, is left to its
  // own selector. The weak-side assist is only a replacement for no pulse.
  if(in.selector_valid && in.selector_width_ms>0)return out;
  const float deficit=in.active_target_deg-in.last_negative_peak_deg;
  if(!(deficit>.5f))return out;
  // Explicitly experimental proportional fallback: 20ms at 0.5 degrees of
  // deficit; 15ms/degree, capped by 150ms and the caller's timing limit.
  const float raw=20.f+15.f*(deficit-.5f);
  const float limit=std::fmin(150.f,static_cast<float>(in.hardware_max_width_ms));
  const float bounded=std::fmin(limit,std::fmax(20.f,raw));
  out.width_ms=static_cast<uint16_t>(std::floor(bounded+.5f));
  out.selected=true;out.current_ma=300;out.motion_sign=1;out.deficit_deg=deficit;
  return out;
}

// Experimental bounded response tracker. A change in the next negative peak
// is an observation, not an identified effect of this pulse: body motion,
// opposite-side inputs, contact and other state changes can all contribute.
// The tracker starts at 50ms and can explore upward after comparable
// observations, up to the caller's timing and hardware limits. It backs off after
// repeated unfavorable observations. Missing/stale/confounded observations
// discard evidence, never latch a stop or mark an unsuccessful send as data.
class Controller {
 public:
  void reset() { *this=Controller{}; }
  void interrupt() {
    pending_=false;stable_count_=0;down_count_=0;
    if(trial_) {limit_ms_=rollback_ms_;trial_=false;}
    exploring_=false;trial_samples_=0;bad_count_=0;
  }
  uint16_t limitMs() const { return limit_ms_; }

  Choice propose(const Input& in) {
    observe(in);
    Choice out=choose(in);
    if(out.selected) {
      // Once the full preset is active and the measured side remains at
      // least a degree short, actually try the initial limit instead of
      // allowing the proportional seed to block all at-limit observations.
      // A prior rollback or the caller's timing cap always wins.
      if(!exploring_ && in.active_target_deg>=7.95f && out.deficit_deg>=1.f) {
        uint16_t seed=limit_ms_<50 ? limit_ms_ : 50;
        if(seed>in.hardware_max_width_ms)seed=in.hardware_max_width_ms;
        if(out.width_ms<seed)out.width_ms=seed;
      }
      // At the final target, a confirmed experiment can exceed the initial
      // deficit formula while staying within the physical hardware cap.
      if(exploring_ && in.active_target_deg>=7.95f)
        out.width_ms=limit_ms_<in.hardware_max_width_ms ? limit_ms_ : in.hardware_max_width_ms;
      else if(out.width_ms>limit_ms_)out.width_ms=limit_ms_;
    }
    proposed_ms_=out.selected?out.width_ms:0;proposed_at_=in.now_ms;
    return out;
  }

  // Call only after the actuator interface accepted this exact proposal.
  void confirm(const Input& in, uint16_t actual_width_ms) {
    pending_=false;
    const Choice candidate=choose(in);
    if(!candidate.selected || actual_width_ms==0 ||
       proposed_at_!=in.now_ms || actual_width_ms!=proposed_ms_ ||
       actual_width_ms>limit_ms_ || actual_width_ms>in.hardware_max_width_ms ||
       in.last_negative_peak_ms>=in.now_ms) {interrupt();return;}
    before_=in.last_negative_peak_deg;target_=in.active_target_deg;
    opposite_=in.last_positive_peak_deg;rpm_=in.wheel_rpm;
    command_ms_=in.now_ms;prior_peak_ms_=in.last_negative_peak_ms;
    applied_limit_=actual_width_ms==limit_ms_;pending_=true;
  }

 private:
  void observe(const Input& in) {
    if(!pending_)return;
    pending_=false;
    if(!in.hardware_healthy || !in.speed_fresh ||
       !std::isfinite(in.last_negative_peak_deg) ||
       !std::isfinite(in.active_target_deg) ||
       !std::isfinite(in.last_positive_peak_deg) || !std::isfinite(in.wheel_rpm) ||
       in.now_ms<=command_ms_ || in.now_ms-command_ms_>1800 ||
       in.last_negative_peak_ms<=command_ms_ ||
       in.last_negative_peak_ms<=prior_peak_ms_ || in.last_negative_peak_ms>=in.now_ms ||
       std::fabs(in.active_target_deg-target_)>.05f ||
       std::fabs(in.last_positive_peak_deg-opposite_)>.6f ||
       std::fabs(in.wheel_rpm-rpm_)>100.f) {interrupt();return;}
    const float delta=in.last_negative_peak_deg-before_;
    const bool evaluating_trial=trial_;
    if(trial_ && !applied_limit_) {interrupt();return;}
    if(trial_ && applied_limit_) {
      ++trial_samples_;
      if(in.last_negative_peak_deg<trial_reference_-.35f)++bad_count_;
      else bad_count_=0;
      if(bad_count_>=2) {
        limit_ms_=rollback_ms_;trial_=false;stable_count_=0;
        cooldown_=4;return;
      }
      if(trial_samples_>=2 && bad_count_==0)trial_=false;
    }
    // Preserve the existing pre-trial trend guard: two successive sizeable
    // drops reduce the permitted width. This is an observation heuristic,
    // not proof that the prior input caused the drops.
    if(!evaluating_trial) {
      if(delta<-.15f)++down_count_;else down_count_=0;
      if(down_count_>=2) {
        limit_ms_=limit_ms_>30?limit_ms_-10:20;
        down_count_=stable_count_=0;cooldown_=4;return;
      }
    }
    if(cooldown_){--cooldown_;stable_count_=0;return;}
    // Wait until the start ramp reaches the 8-degree preset. Require actual
    // at-cap pulses and several comparable peaks remaining at least 1 degree
    // short. These observations do not identify the effect of extra input.
    if(!applied_limit_ || in.active_target_deg<7.95f ||
       in.last_negative_peak_deg>=in.active_target_deg-1.f || std::fabs(delta)>.2f) {
      stable_count_=0;return;
    }
    if(!stable_count_ || std::fabs(in.last_negative_peak_deg-stable_base_)>.35f) {
      stable_base_=in.last_negative_peak_deg;stable_sum_=in.last_negative_peak_deg;stable_count_=1;
    } else {stable_sum_+=in.last_negative_peak_deg;++stable_count_;}
    if(stable_count_>=2 && !trial_ && limit_ms_<150 &&
       in.hardware_max_width_ms>limit_ms_) {
      rollback_ms_=limit_ms_;trial_reference_=stable_sum_/stable_count_;
      limit_ms_=static_cast<uint16_t>(limit_ms_+10);
      if(limit_ms_>150)limit_ms_=150;
      if(limit_ms_>in.hardware_max_width_ms)limit_ms_=in.hardware_max_width_ms;
      trial_=true;exploring_=true;trial_samples_=bad_count_=stable_count_=0;
    }
  }
  float before_=0.f,target_=0.f,opposite_=0.f,rpm_=0.f;
  float stable_base_=0.f,stable_sum_=0.f,trial_reference_=0.f;
  uint32_t command_ms_=0,prior_peak_ms_=0,proposed_at_=0;
  uint16_t limit_ms_=50,rollback_ms_=50,proposed_ms_=0;
  uint8_t stable_count_=0,trial_samples_=0,bad_count_=0,cooldown_=0,down_count_=0;
  bool pending_=false,applied_limit_=false,trial_=false,exploring_=false;
};
}
