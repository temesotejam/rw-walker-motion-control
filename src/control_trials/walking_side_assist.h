#pragma once
#include <cmath>
#include <cstdint>

// One bounded identification probe, not an identified motion model.
// Spans are camera relative, not ground stride; no outcome is persisted.
namespace positive_sway_probe {
class Controller {
 public:
  void reset(){*this=Controller{};}
  bool active()const{return active_;}
  bool attempted()const{return attempted_;}
  uint32_t lastApplied()const{return last_applied_;}
  bool effectiveCommand(uint32_t ms)const {for(unsigned i=0;i<8;++i)if(ms && commands_[i]==ms)return true;return false;}
  void interrupt(){if(active_)active_=false;count_=0;next_=0;}
  void cycle(uint32_t start,uint32_t end,float left,float right,float plus,float minus,
             bool valid,bool healthy,bool neutral,bool effective) {
    if(!healthy){interrupt();return;}
    if(!start || end<=start || !valid || !std::isfinite(left) || !std::isfinite(right) ||
       !std::isfinite(plus) || !std::isfinite(minus) || left<0 || right<0 || plus<=0 || minus<=0) {
      interrupt();return;
    }
    if(active_) {
      // Overlapping samples cannot measure response. Invalid/ineffective
      // post-change samples interrupt, never become missing zeroes.
      if(start<=started_)return;
      if(!effective){interrupt();return;}
      if(response_end_ && start!=response_end_){interrupt();return;}
      if(plus>9.5f || minus>9.5f || plus<ref_plus_*.65f || minus<ref_minus_*.65f ||
         left<ref_left_*.4f || right<ref_right_*.4f) {interrupt();return;}
      response_end_=end;
      if(++responses_>=2)interrupt(); // always restore; observational trial only
      return;
    }
    if(attempted_ || !neutral || (last_end_ && start!=last_end_)){count_=0;next_=0;}
    if(attempted_ || !neutral){last_end_=end;return;}
    if(plus>=6.f || minus<4.5f || minus>8.5f || minus-plus<1.f || left<.2f || right<.2f) {
      count_=0;next_=0;last_end_=end;return;
    }
    plus_[next_]=plus;minus_[next_]=minus;left_[next_]=left;right_[next_]=right;next_=(next_+1)%4;
    if(count_<4)++count_;
    ref_plus_=ref_minus_=ref_left_=ref_right_=0;
    for(unsigned i=0;i<count_;++i){ref_plus_+=plus_[i];ref_minus_+=minus_[i];ref_left_+=left_[i];ref_right_+=right_[i];}
    ref_plus_/=count_;ref_minus_/=count_;ref_left_/=count_;ref_right_/=count_;last_end_=end;
  }
  bool candidate(uint32_t now,bool healthy,bool foot_idle)const {
    return healthy && foot_idle && !active_ && !attempted_ && count_>=4 &&
      now>=last_end_ && now-last_end_<=1500;
  }
  bool request(uint32_t now,bool healthy,bool foot_idle) {
    if(active_ && (!healthy || !foot_idle || !now || now<started_ || now-started_>3000))interrupt();
    return active_ || candidate(now,healthy,foot_idle);
  }
  void confirm(uint32_t now,uint16_t baseline,uint16_t actual,bool accepted,bool start) {
    if(!accepted || !now || actual<=baseline || actual-baseline>5){if(active_)interrupt();return;}
    if(!active_) {if(!start || attempted_)return;attempted_=true;active_=true;started_=now;responses_=0;response_end_=0;}
    last_applied_=now;commands_[command_next_]=now;command_next_=(command_next_+1)%8;
  }
 private:
  uint32_t last_end_=0,started_=0,last_applied_=0,response_end_=0;
  unsigned count_=0,responses_=0,next_=0,command_next_=0;
  uint32_t commands_[8]={};float plus_[4]={},minus_[4]={},left_[4]={},right_[4]={};
  float ref_plus_=0,ref_minus_=0,ref_left_=0,ref_right_=0;
  bool active_=false,attempted_=false;
};
}

// Exploratory input for +pitch, provisionally assumed to be physical left.
// Current is a command, not a measured torque. Trial outcomes are deliberately
// excluded from the ordinary fitted model by the caller. Peak differences are
// observations, not a causal isolation of the pulse from the opposite input,
// wheel speed, contact, ground, or foot slip.
namespace positive_left_boost {
struct Input {
  uint32_t now_ms=0, opposite_peak_ms=0;
  float preset_deg=NAN, side_target_deg=NAN, opposite_peak_deg=NAN;
  float wheel_rpm=NAN, battery_mv=NAN;
  int8_t next_side=0;
  int16_t normal_ma=0;
  uint16_t normal_width_ms=0, normal_cap_ms=0, hardware_cap_ms=0;
  bool normal_valid=false, normal_motion_aligned=false;
  bool hardware_healthy=false, speed_fresh=false;
};
struct Choice { bool selected=false; int16_t current_ma=0; uint16_t width_ms=0; };

class Controller {
 public:
  void reset() { *this=Controller{}; }
  // A failed experiment never stops ordinary walking control.
  void interrupt() {
    pending_=0;baseline_count_=0;baseline_sum_=0.f;baseline_diff_sum_=0.f;
    baseline_mean_=0.f;baseline_diff_=0.f;accepted_=false;proposed_=false;
    stage_=0;last_good_stage_=255;backoff_=false;
  }
  unsigned baselineCount()const{return baseline_count_;}
  bool accepted()const{return accepted_;}
  bool awaitingPeak()const{return pending_!=0;}

  Choice propose(const Input& in) {
    Choice out;proposed_=false;
    if(!valid(in)) {interrupt();return out;}
    if(pending_ && (in.now_ms<command_ms_ || in.now_ms-command_ms_>1800)) {
      interrupt();cooldown_=4;return out;
    }
    if(pending_ || cooldown_ || baseline_count_<2 ||
       in.opposite_peak_deg>=in.preset_deg ||
       baseline_mean_>=in.side_target_deg-.5f ||
       (accepted_ && last_good_peak_>=in.side_target_deg-.5f))return out;
    out.selected=true;
    out.current_ma=stageCurrent(stage_);
    const uint16_t desired=stageWidth(stage_);
    out.width_ms=desired<in.hardware_cap_ms?desired:in.hardware_cap_ms;
    proposed_width_=out.width_ms;proposed_ma_=out.current_ma;
    proposed_=true;proposed_ms_=in.now_ms;return out;
  }

  // Only actual positive-direction normal commands establish the baseline.
  // We compare matched pairs of positive and following negative peaks.
  void confirmNormal(const Input& in,int16_t actual_ma,uint16_t actual_width_ms,int8_t actual_sign,bool executed) {
    if(!valid(in) || !executed || actual_ma!=in.normal_ma ||
       actual_ma<=0 || actual_sign!=1 || actual_width_ms!=in.normal_width_ms || pending_) {interrupt();return;}
    command_ms_=in.now_ms;command_width_=actual_width_ms;
    pending_=1;proposed_=false;
  }
  void confirmTrial(const Input& in,int16_t actual_ma,uint16_t actual_width_ms,int8_t actual_sign,bool executed) {
    if(!valid(in) || !proposed_ || proposed_ms_!=in.now_ms || !executed ||
       actual_ma!=proposed_ma_ || actual_sign!=1 || actual_width_ms!=proposed_width_ ||
       actual_width_ms>in.hardware_cap_ms || actual_width_ms>150 || pending_) {interrupt();cooldown_=4;return;}
    command_ms_=in.now_ms;command_width_=actual_width_ms;opposite_before_=in.opposite_peak_deg;
    pending_=2;proposed_=false;accepted_=false;
  }

  __attribute__((noinline)) void peak(int side,uint32_t ms,float angle,bool fresh) {
    if(!pending_)return;
    if(!fresh || !std::isfinite(angle) || angle<=0.f || angle>=16.f ||
       ms<=command_ms_ || ms-command_ms_>1800 ||
       ((pending_==1 || pending_==2) && ms-command_ms_<command_width_)) {
      interrupt();cooldown_=4;return;
    }
    if(pending_==1 || pending_==2) {
      if(side!=1) {interrupt();cooldown_=4;return;}
      positive_=angle;positive_ms_=ms;pending_=(pending_==1?4:3);return;
    }
    if(side!=-1 || ms<=positive_ms_) {interrupt();cooldown_=4;return;}
    if(pending_==4) {
      const float diff=positive_-angle;
      if(baseline_count_<2) {
        baseline_sum_+=positive_;baseline_diff_sum_+=diff;++baseline_count_;
        baseline_mean_=baseline_sum_/baseline_count_;
        baseline_diff_=baseline_diff_sum_/baseline_count_;
      } else {
        baseline_mean_=.5f*(baseline_mean_+positive_);
        baseline_diff_=.5f*(baseline_diff_+diff);
      }
      pending_=0;if(cooldown_)--cooldown_;return;
    }
    const float diff=positive_-angle;
    // Comparing consecutive + then - peaks rejects equal rises on both sides.
    // A following negative-side pulse and changing conditions remain confounds.
    const bool improved=diff>=baseline_diff_+.2f;
    const bool opposite_kept=angle>=opposite_before_-.8f && angle>=.7f*opposite_before_;
    const bool deteriorated=diff<baseline_diff_-.4f ||
        (last_good_stage_!=255 && diff<last_good_diff_-.25f);
    pending_=0;
    if(!opposite_kept || deteriorated) {
      if(last_good_stage_!=255 && stage_!=last_good_stage_ && opposite_kept) {
        stage_=last_good_stage_;backoff_=true;accepted_=true;
      } else {interrupt();cooldown_=4;}
      return;
    }
    // The other side has already reached its nominal target; increasing the
    // test input could amplify both sides. Keep ordinary control in charge.
    if(angle>=opposite_target_snapshot_) {interrupt();cooldown_=4;return;}
    accepted_=improved;
    if(improved) {
      last_good_stage_=stage_;last_good_diff_=diff;last_good_peak_=positive_;
    }
    if(positive_>=target_snapshot_-.5f) {
      // Reaching the positive target without a differential improvement is
      // inconclusive, not a reason to drive both amplitudes still higher.
      if(!improved) {interrupt();cooldown_=4;}
      return;
    }
    // Inconclusive common-mode increases are not called improvement; try
    // the next level while keeping ordinary guards and the 150 ms ceiling.
    if(!backoff_) {
      if(stage_<5)++stage_;
      else if(last_good_stage_!=255) {stage_=last_good_stage_;backoff_=true;}
      else {interrupt();cooldown_=4;}
    }
  }

 private:
  static int16_t stageCurrent(uint8_t stage) {
    return stage==0?500:stage==1?700:stage==2?900:1200;
  }
  static uint16_t stageWidth(uint8_t stage) {
    return stage<4?100:stage==4?125:150;
  }
  bool valid(const Input& in) {
    if(in.preset_deg!=8.f || !std::isfinite(in.side_target_deg) || in.next_side!=1 ||
       !in.hardware_healthy || !in.speed_fresh || !std::isfinite(in.wheel_rpm) ||
       !std::isfinite(in.battery_mv) || in.battery_mv<=0.f ||
       !std::isfinite(in.opposite_peak_deg) || in.opposite_peak_deg<=0.f || in.opposite_peak_deg>=16.f ||
       !in.opposite_peak_ms || in.now_ms<=in.opposite_peak_ms ||
       in.now_ms-in.opposite_peak_ms>1500 || !in.normal_valid || !in.normal_motion_aligned ||
       in.normal_ma<=0 || !in.normal_width_ms || in.hardware_cap_ms<100 ||
       in.normal_width_ms>in.normal_cap_ms || in.normal_width_ms>in.hardware_cap_ms) return false;
    target_snapshot_=in.side_target_deg;opposite_target_snapshot_=in.preset_deg;return true;
  }
  float baseline_sum_=0.f,baseline_diff_sum_=0.f,baseline_mean_=0.f,baseline_diff_=0.f;
  float opposite_before_=0.f,positive_=0.f,target_snapshot_=NAN,opposite_target_snapshot_=NAN;
  float last_good_peak_=0.f,last_good_diff_=0.f;
  uint32_t command_ms_=0,positive_ms_=0,proposed_ms_=0;
  uint16_t command_width_=0,proposed_width_=0;
  int16_t proposed_ma_=0;
  uint8_t baseline_count_=0,pending_=0,cooldown_=0,stage_=0,last_good_stage_=255;
  bool accepted_=false,proposed_=false,backoff_=false;
};
}

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
