#pragma once
#include <cmath>
#include <cstdint>

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
