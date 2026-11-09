#pragma once
#include "wheel_work_energy_control.h"

// Effective free-foot energy decrease, not an independently measured impact
// or friction loss. Fitted ONLY to 32 unpowered negative half-cycles in
// run 50881356. All energies here are mJ. Never add this loss to the old
// side-bias correction: this is a separate, mutually exclusive branch.
namespace walking_energy_loss {
constexpr char kRevision[]="walking_coast_eight_runs_v6";
inline bool supported(const wheel_work_energy_control::State& s) {
  return s.absolute_reference && s.target_deg==8.f &&
      wheel_work_energy_control::freshFinite(s) &&
      s.rate_dps>=13.75f && s.rate_dps<=37.80f &&
      s.wheel_rpm>=274.90f && s.wheel_rpm<=603.21f &&
      s.battery_mv>=7000.f && s.battery_mv<=8120.f &&
      s.previous_peak_deg>=4.3006f && s.previous_peak_deg<=7.65f;
}
inline float coastMj(float rate_dps) {
  return .09046158f+.00018973f*rate_dps*rate_dps;
}

// The low-speed range has unpowered arrival evidence only; do not treat the
// transferred powered response as a measured causal gain.
inline bool lowSpeedSupported(const wheel_work_energy_control::State& s) {
  return s.absolute_reference && s.target_deg==8.f && wheel_work_energy_control::freshFinite(s) &&
      s.rate_dps>=13.75f && s.rate_dps<=37.80f && s.wheel_rpm>=240.f && s.wheel_rpm<274.90f &&
      s.battery_mv>=7000.f && s.battery_mv<=8120.f &&
      s.previous_peak_deg>=4.3006f && s.previous_peak_deg<=7.65f;
}
class LowSpeedGuard {
 public:
  uint16_t capMs(uint32_t now,float target) const {
    return disabled_ || !last_ms_ || now<last_ms_ || now-last_ms_>4000 ||
        !last_negative_ms_ || now<last_negative_ms_ || now-last_negative_ms_>1500 ||
        !std::isfinite(target) || std::fabs(target-last_target_)>.03f ? 0 : cap_;
  }
  unsigned coastCount(uint32_t now,float target) const {
    if(disabled_ || !last_ms_ || now<last_ms_ || now-last_ms_>4000 ||
       !last_negative_ms_ || now<last_negative_ms_ || now-last_negative_ms_>1500 ||
       !std::isfinite(target) || std::fabs(target-last_target_)>.03f)return 0;
    return coasts_;
  }
  void interrupt() {coasts_=0;qualified_=0;cap_=0;last_ms_=0;last_negative_ms_=0;last_target_=0;coast_sum_=0;powered_sum_=0;}
  // Large prediction mismatch or overshoot locks this extension for this run.
  void observe(float actual,float prediction,float target,uint16_t width,bool fresh,uint32_t ms) {
    if(disabled_)return;
    if(last_ms_ && (ms<=last_ms_ || ms-last_ms_>4000 || !last_negative_ms_ ||
        ms<=last_negative_ms_ || ms-last_negative_ms_>1500))interrupt();
    if(!fresh || !std::isfinite(actual) || !std::isfinite(prediction) || !std::isfinite(target) ||
       actual<0.f || prediction<0.f || target<=0.f || std::fabs(actual-prediction)>.06f || actual>=target) {
      disabled_=true;interrupt();return;
    }
    if(last_ms_ && std::fabs(target-last_target_)>.03f)interrupt();
    last_ms_=ms;last_negative_ms_=ms;last_target_=target;
    if(width==0) {
      qualified_=0;powered_sum_=0;
      if(coasts_<2) {coast_sum_+=actual-prediction;++coasts_;}
      if(coasts_>=2 && cap_==0)cap_=10;
      return;
    }
    if(coasts_<2 || width>cap_ || width<10) {disabled_=true;interrupt();return;}
    // A capped arrival must remain below target and match its prediction.
    // This check alone cannot establish the causal powered response.
    if(width==cap_ && actual<target-.03f) {
      powered_sum_+=actual-prediction;
      if(++qualified_>=2) {
        if(powered_sum_/qualified_<coast_sum_/coasts_-.03f) {disabled_=true;interrupt();return;}
        if(cap_<25)cap_+=5;
        qualified_=0;powered_sum_=0;
      }
    } else {qualified_=0;powered_sum_=0;}
  }
  // Normally supported negative crossings can lie between two low-speed
  // arrivals. Retain the low samples only if this intervening response is
  // fresh, near the same target and agrees with its own prediction.
  void bridge(float actual,float prediction,float target,float rpm,bool fresh,uint32_t ms) {
    if(!last_ms_ || disabled_)return;
    if(!fresh || !std::isfinite(actual) || !std::isfinite(prediction) || !std::isfinite(target) ||
       !std::isfinite(rpm) || rpm<240.f || rpm>400.f || ms<=last_ms_ || ms-last_ms_>4000 ||
       !last_negative_ms_ || ms<=last_negative_ms_ || ms-last_negative_ms_>1500 ||
       std::fabs(target-last_target_)>.03f || std::fabs(actual-prediction)>.06f || actual>=target) {
      interrupt();return;
    }
    last_negative_ms_=ms;
  }
 private:
  uint16_t cap_=0;unsigned coasts_=0,qualified_=0;bool disabled_=false;uint32_t last_ms_=0,last_negative_ms_=0;float last_target_=0,coast_sum_=0,powered_sum_=0;
};
class Feedback {
 public:
  unsigned count() const { return count_; }
  float biasMj() const {
    if(count_<2) return 0.f;
    float sum=0.f;for(unsigned i=0;i<count_;++i)sum+=residual_[i];
    return std::fmax(-.10f,std::fmin(.10f,sum/count_));
  }
  // Two matched coast arrivals first. Inputs above 60 ms are experimental.
  // Raise the cap only after two completed
  // matching powered arrivals below target at the current cap.
  uint16_t capMs() const { return count_>=2 && !overshot_ ? cap_ms_ : 0; }
  void interrupt() {
    if(trial_active_) cap_ms_=trial_previous_cap_;
    trial_active_=false;trial_count_=0;trial_sum_=0;
    baseline_count_=0;baseline_sum_=0;limited_count_=0;extended_count_=0;
  }
  bool observe(float actual_mj,float raw_mj,float target_mj,uint16_t width_ms=0,
               float arrival_rpm=NAN,bool arrival_rpm_fresh=false) {
    if(!std::isfinite(actual_mj)||!std::isfinite(raw_mj)||
       !std::isfinite(target_mj)||actual_mj<0.f||raw_mj<0.f||target_mj<=0.f) {interrupt();return false;}
    overshot_=actual_mj>=target_mj;
    if(overshot_) { cap_ms_=cap_ms_>25 ? cap_ms_-5 : 25; limited_count_=0;extended_count_=0;trial_active_=false;trial_count_=0;trial_sum_=0;baseline_count_=0;baseline_sum_=0; }
    if(cap_ms_>60 && (!arrival_rpm_fresh || !std::isfinite(arrival_rpm) || arrival_rpm<30.f)) {
      cap_ms_=60;extended_count_=0;trial_active_=false;trial_count_=0;trial_sum_=0;baseline_count_=0;baseline_sum_=0;
    }
    if(std::fabs(actual_mj-raw_mj)>.15f) { interrupt();return false; }
    if(trial_active_ && width_ms<cap_ms_) interrupt();
    // Do not continue increasing pulse width solely because the target is
    // missed. Compare matched, capped arrivals before and after each increase.
    // This is a conservative online guard, not an identified causal model.
    if(trial_active_ && width_ms>=cap_ms_) {
      trial_sum_+=actual_mj;
      if(++trial_count_>=3) {
        if(trial_sum_/trial_count_ < trial_baseline_-.03f) {
          cap_ms_=trial_previous_cap_;
          ceiling_ms_=trial_previous_cap_;
          limited_count_=extended_count_=0;
        }
        trial_active_=false;trial_count_=0;trial_sum_=0;
      }
    }
    if(width_ms>0 && width_ms>=cap_ms_ && actual_mj<target_mj-.03f) {
      if(!trial_active_) { baseline_sum_+=actual_mj; ++baseline_count_; }
      if(cap_ms_<60) {
        if(++limited_count_>=2 && !trial_active_) {
          if(cap_ms_>=45 && cap_ms_<ceiling_ms_ && baseline_count_>=2) startTrial();
          else if(cap_ms_<45 && cap_ms_<ceiling_ms_) {cap_ms_+=5;baseline_count_=0;baseline_sum_=0;}
          limited_count_=0;
        }
      } else if(arrival_rpm_fresh && std::isfinite(arrival_rpm) && arrival_rpm>=30.f) {
        // Experimental 65–75 ms: three matched, capped, below-target
        // arrivals with a fresh same-direction wheel speed still positive.
        if(++extended_count_>=3 && !trial_active_) {if(cap_ms_<75 && cap_ms_<ceiling_ms_ && baseline_count_>=2)startTrial();extended_count_=0;}
      } else extended_count_=0;
    } else if(width_ms>0) {limited_count_=0;extended_count_=0;}
    residual_[next_]=actual_mj-raw_mj;next_=(next_+1)%3;
    if(count_<3)++count_;
    return true;
  }
 private:
  void startTrial() {
    trial_baseline_=baseline_sum_/baseline_count_;trial_previous_cap_=cap_ms_;
    cap_ms_+=5;trial_active_=true;trial_count_=0;trial_sum_=0;
    baseline_sum_=0;baseline_count_=0;
  }
  float residual_[3]={};unsigned count_=0,next_=0,limited_count_=0,extended_count_=0;
  float baseline_sum_=0,trial_sum_=0,trial_baseline_=0;
  unsigned baseline_count_=0,trial_count_=0;
  uint16_t ceiling_ms_=75,trial_previous_cap_=0;
  bool trial_active_=false;
  uint16_t cap_ms_=25;bool overshot_=false;
};
__attribute__((noinline)) inline wheel_work_energy_control::Choice choose(
    const lateral_energy::Geometry& g,const wheel_work_energy_control::State& s,
    const lateral_energy::CurrentTable& table,float initial_ma,float goal_ma,
    float target_mj,const Feedback& feedback,float cg_mm,uint16_t requested_cap_ms=100,
    const LowSpeedGuard* low_guard=nullptr,uint32_t now_ms=0) {
  using namespace wheel_work_energy_control;
  Choice c;
  const bool low=lowSpeedSupported(s);
  if((!supported(s) && !low)||!std::isfinite(target_mj)||target_mj<=0.f||
      !std::isfinite(initial_ma)||!std::isfinite(goal_ma)||goal_ma<=0.f||
      !std::isfinite(cg_mm)||std::fabs(cg_mm)>1e-6f) return c;
  const float previous_mj=1000.f*lateral_energy::potential(g,s.previous_peak_deg);
  const float passive=coastMj(s.rate_dps)+feedback.biasMj();
  // Signed Uprev-Ecoast may be negative from motion or energy exchange.
  const float loss=previous_mj-passive;
  if(!std::isfinite(loss)||passive<=0.f)return c;
  const uint16_t guard_cap=low?(low_guard?low_guard->capMs(now_ms,target_mj):0):feedback.capMs();
  const uint16_t cap=guard_cap<requested_cap_ms ? guard_cap : requested_cap_ms;
  for(uint16_t ms=0;ms<=cap;ms=ms==0 ? 10 : ms+1) {
    const float q=ms ? lateral_energy::charge(table,ms,initial_ma,goal_ma) : 0.f;
    if(!std::isfinite(q)||q<0.f||q>25.f)continue;
    Prediction p;
    // Same provisional incremental-gain transfer as the existing negative
    // assist model. Coast fit is measured; powered negative gain is NOT.
    const float gain=.22502117220f*.06189960458f*q*s.rate_dps*kRadPerDeg;
    p.energy_mj=previous_mj-loss+gain;p.free_mj=passive;
    p.valid=std::isfinite(p.energy_mj)&&p.energy_mj>=0.f;
    p.branch=low?11:10;p.cross_condition=true;p.support_weight=0.f;
    p.applied_bias_mj=feedback.biasMj();
    consider(c,300,ms,q,p,target_mj);
  }
  c.command_sign=1;c.allowed_cap_ms=cap;c.walking_loss_mj=loss;
  finish(c,target_mj);return c;
}
}
