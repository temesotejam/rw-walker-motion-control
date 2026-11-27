#pragma once
#include <cmath>
#include <cstdint>

// Experimental positive-side candidate restriction for a phase-mapped gait.
// The caller must establish that the positive body peak corresponds to the
// left foot and supply calibrated, time-matched complete-cycle observations.
// Restriction changes candidate feasibility, not the energy prediction itself.
namespace foot_phase_cap {
enum class State : uint8_t { Waiting=0, Trial=1, Hold=2, Release=3, Locked=4 };
enum class Reason : uint8_t { None=0, Stale=1, Invalid=2, InputFailed=3, NegativeLoss=4,
  WeakPeak=5, Worsened=6, Inconclusive=7, Reversed=8, Disallowed=9, TargetChanged=10, ModeChanged=11 };
enum class NegativeMode : uint8_t { Invalid=0, Coast=1, Powered=2, UnmodeledCoast=3 };
struct NegativeDecision {
  NegativeMode mode=NegativeMode::Invalid;
  int16_t ma=0; int8_t sign=0; uint16_t width=0; uint8_t branch=0;
  bool same(const NegativeDecision& other) const {
    return mode!=NegativeMode::Invalid && mode==other.mode && branch==other.branch &&
        (mode==NegativeMode::Coast || mode==NegativeMode::UnmodeledCoast ||
         (ma==other.ma && sign==other.sign && width==other.width));
  }
};
// Classify only an explicit ordinary no-output result as coast; all failed,
// unsupported or inconsistent decisions remain invalid.
inline NegativeDecision classifyNegative(bool valid,bool executed,bool reason_none,bool reason_coast,
    int16_t ma,int sign,uint16_t width,uint8_t branch) {
  NegativeDecision d;
  if(valid && branch && !executed && reason_coast && !reason_none && width==0 && ma==0) {
    d.mode=branch==12 ? NegativeMode::UnmodeledCoast : NegativeMode::Coast;d.branch=branch;
  } else if(valid && branch && executed && reason_none && !reason_coast && width>0 && ma>0 &&
            (sign==1||sign==-1)) {
    d.mode=NegativeMode::Powered;d.ma=ma;d.sign=static_cast<int8_t>(sign);d.width=width;d.branch=branch;
  }
  return d;
}
class Controller {
 public:
  void reset(){*this=Controller{};}
  // One complete, independently observed phase pair; now is its end time and
  // start is its first boundary. Units are calibrated camera-relative degrees.
  __attribute__((noinline)) void pair(uint32_t now,uint32_t start,float left,float right,
      float negative_peak,NegativeDecision negative,bool valid) {
    if(state_==State::Locked)return;
    if(last_pair_ && now<=last_pair_)return; // never count a cycle twice
    if(!valid||negative.mode==NegativeMode::Invalid||!std::isfinite(left)||!std::isfinite(right)||
       !std::isfinite(negative_peak)||negative_peak<=0.f||
       std::fabs(left)>25.f||std::fabs(right)>25.f||start>now||now-start<250||now-start>2000) {
      stop(Reason::Invalid);return;
    }
    if(last_pair_ && (now-last_pair_>2000 || start!=last_pair_)) {
      stop(Reason::Stale);return;
    }
    const Actual* cycle_actual=nullptr;unsigned cycle_actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=history_[(actual_next_+kHistory-actual_count_+i)%kHistory];
      if(a.ms>start && a.ms<now) {cycle_actual=&a;++cycle_actions;}
    }
    if(cycle_actions!=1 || !cycle_actual || !cycle_actual->executed ||
       cycle_actual->ma!=400 || cycle_actual->sign!=1 ||
       ((state_==State::Trial||state_==State::Hold) && trial_started_ && start>trial_start_ &&
        cycle_actual->width!=desired_width_)) {
      stop(Reason::InputFailed);return;
    }
    last_pair_=now;
    const float motion=std::fabs(left)+std::fabs(right);
    // Preserve each foot's sign and compare calibrated degrees directly: a
    // normalized ratio can hide a substantial change when one foot is negative.
    const float e=left-right;
    if(!std::isfinite(e) || motion<.30f || (left<=0.f && right<=0.f)) {stop(Reason::Invalid);return;}
    if((state_==State::Trial || state_==State::Hold) && e<-.30f) {stop(Reason::Reversed);return;}
    err_[next_]=e;peak_[next_]=negative_peak;left_[next_]=left;right_[next_]=right;
    negative_[next_]=negative;targets_[next_]=cycle_actual->target_mj;
    widths_[next_]=cycle_actual->width;
    next_=(next_+1)%2;if(count_<2)++count_;
    if((state_!=State::Trial && state_!=State::Hold) || !trial_started_ || start<=trial_start_)return;
    if(negative_peak<reference_peak_-.8f) {stop(Reason::WeakPeak);return;}
    if(negative_peak<reference_peak_-.5f)++weak_count_;else weak_count_=0;
    if(weak_count_>=2) {stop(Reason::WeakPeak);return;}
    if(e>reference_error_+.30f || right<reference_right_-.15f ||
       left+right<reference_total_-.30f) {stop(Reason::Worsened);return;}
    if(!negative.same(reference_negative_)) {
      // A missing model is not a physical fault and does not by itself stop
      // balancing. It is also not comparable evidence of improvement.
      if(negative.mode==NegativeMode::UnmodeledCoast) return;
      if(++noncomparable_count_>=2) stop(Reason::Inconclusive);
      return;
    }
    noncomparable_count_=0;
    if(state_==State::Hold)return;
    trial_error_sum_+=e;++trial_count_;
    if(trial_count_>=2) {
      const float trial_error=trial_error_sum_/trial_count_;
      if(trial_error<=reference_error_-.10f) {state_=State::Hold;reason_=Reason::None;}
      else stop(Reason::Inconclusive);
    }
  }
  // Invalid decisions interrupt immediately. Explicit model-selected coast is
  // observed and compared at the completed-cycle boundary.
  void negativeDecision(uint32_t now,NegativeDecision decision) {
    (void)now;
    if((state_==State::Trial||state_==State::Hold) && decision.mode==NegativeMode::Invalid) {
      stop(Reason::Invalid);
    }
  }
  // Caller skips the current positive output when a required branch-5
  // candidate cannot be formed; the next crossing uses the normal selector.
  void modeChanged() {state_=State::Locked;reason_=Reason::ModeChanged;waiting_confirmation_=false;}
  // Existing safety, gradual-start and emergency decision must be reflected in
  // allowed/existing_cap. A false allowed must also suppress the motor output
  // externally when appropriate. Only a 400mA positive baseline is eligible.
  // `baseline_ms` is the width from the ordinary (unmodified) selector.
  __attribute__((noinline)) uint16_t limit(uint32_t now,uint16_t baseline_ms,
      int16_t baseline_ma,int sign,uint16_t existing_cap,bool allowed,float target_mj) {
    const uint16_t safe=baseline_ms<existing_cap?baseline_ms:existing_cap;
    if(require_neutral_)return 0;
    // Once a trial ends, no stale observation or mode transition may rearm it
    // before the next explicit run reset.
    if(state_==State::Locked)return safe;
    if(!allowed) {request_target_mj_=NAN;stop(Reason::Disallowed);return safe;}
    if(!std::isfinite(target_mj)||target_mj<=0.f) {
      request_target_mj_=NAN;
      if(state_==State::Trial||state_==State::Hold||state_==State::Release) {
        state_=State::Locked;reason_=Reason::Invalid;waiting_confirmation_=false;return 0;
      }
      stop(Reason::Invalid);return safe;
    }
    request_target_mj_=target_mj;
    if(last_pair_ && (now<last_pair_ || now-last_pair_>2000))stop(Reason::Stale);
    if(state_==State::Trial||state_==State::Hold) {
      if(baseline_ma!=400||sign!=1) {
        // Before any experimental pulse there is no modified command to unwind.
        if(state_==State::Trial && !trial_started_ && !waiting_confirmation_ && last_width_==anchor_width_) {
          state_=State::Locked;reason_=Reason::InputFailed;return safe;
        }
        // The ordinary solver changed modes before a pulse was issued. Skip
        // this positive crossing, discard the experiment, then return to the
        // ordinary selector. An actually executed mismatch is handled below.
        state_=State::Locked;reason_=Reason::InputFailed;waiting_confirmation_=false;return 0;
      }
      if(std::fabs(target_mj-reference_target_mj_)>.001f)stop(Reason::TargetChanged);
      // A changed ordinary selector invalidates the like-for-like comparison
      // and can otherwise turn a single 5 ms probe into a larger restriction.
      else if(baseline_ms!=anchor_width_)stop(Reason::InputFailed);
      else if(baseline_ms<75||baseline_ms>100)stop(Reason::InputFailed);
    }
    if(state_==State::Waiting) {
      if(count_<2||!last_pair_||now<last_pair_||now-last_pair_>1500||
         baseline_ma!=400||sign!=1||baseline_ms<75||baseline_ms>100||safe!=baseline_ms||
         err_[0]<=.30f||err_[1]<=.30f||!negative_[0].same(negative_[1])||
         peak_[1-next_]<peak_[next_]-.30f||!actual_valid_||actual_ma_!=400||actual_sign_!=1||
         now<actual_ms_||now-actual_ms_>1500||actual_width_!=baseline_ms ||
         widths_[0]!=baseline_ms||widths_[1]!=baseline_ms||
         !std::isfinite(targets_[0])||!std::isfinite(targets_[1])||
         std::fabs(targets_[0]-target_mj)>.001f||std::fabs(targets_[1]-target_mj)>.001f)return safe;
      anchor_width_=baseline_ms;last_width_=actual_width_;reference_target_mj_=target_mj;startTrial();
    }
    if(state_==State::Trial) {
      if(waiting_confirmation_) {stop(Reason::InputFailed);return safe;}
      requested_=safe<desired_width_?safe:desired_width_;
      if(requested_==0 || requested_>=baseline_ms ||
         (last_width_>5 && requested_<last_width_-5)) {
        stop(Reason::InputFailed);requested_=safe;restore_target_=baseline_ms;
        waiting_confirmation_=true;return safe;
      }
      waiting_confirmation_=true;return requested_;
    }
    if(state_==State::Hold) {
      requested_=safe<last_width_?safe:last_width_;waiting_confirmation_=true;return requested_;
    }
    if(state_==State::Release) {
      // Returning to the ordinary selector is at most +5ms per executed
      // positive pulse; an existing tighter safety cap takes precedence.
      if(baseline_ma!=400||sign!=1 || (actual_valid_ && (actual_ma_!=400||actual_sign_!=1))) {
        state_=State::Locked;reason_=Reason::InputFailed;waiting_confirmation_=false;return 0;
      }
      restore_target_=baseline_ms;
      const uint16_t next_width=static_cast<uint16_t>(last_width_+5);
      requested_=safe<next_width?safe:next_width;
      waiting_confirmation_=true;return requested_;
    }
    return safe;
  }
  // Confirm the *actual* command after it is sent. Unexecuted, wrong current,
  // wrong direction or a width that does not match the proposed cap is not
  // evidence for a probe. Output confirmation is not motor-current telemetry.
  __attribute__((noinline)) void confirm(uint32_t now,bool executed,int16_t current_ma,int sign,uint16_t width) {
    // Keep actual history even before the first trial; a theoretical baseline
    // is not evidence that the previous physical command was 80ms.
    const bool pending=waiting_confirmation_;
    Actual& record=history_[actual_next_];
    record.ms=now;record.width=width;record.ma=current_ma;record.sign=sign;
    record.target_mj=request_target_mj_;record.executed=executed;
    actual_next_=(actual_next_+1)%kHistory;if(actual_count_<kHistory)++actual_count_;
    if(executed && width>0 && (sign==1||sign==-1)) {
      actual_valid_=true;actual_ms_=now;actual_ma_=current_ma;actual_sign_=sign;actual_width_=width;
      actual_target_mj_=request_target_mj_;
      if(state_==State::Trial||state_==State::Hold||state_==State::Release)last_width_=width;
    } else actual_valid_=false;
    if(!pending) {
      if(state_==State::Trial||state_==State::Hold)stop(Reason::InputFailed);
      return;
    }
    waiting_confirmation_=false;
    if(!executed||width==0) {stop(Reason::InputFailed);return;}
    if(current_ma!=400||sign!=1) {require_neutral_=true;stop(Reason::InputFailed);return;}
    if(width!=requested_) {stop(Reason::InputFailed);return;}
    if(state_==State::Trial && !trial_started_) {trial_started_=true;trial_start_=now;}
    if(state_==State::Release && width>=restore_target_)state_=State::Locked;
  }
  // Monitoring can stop a trial even if no new complete pair arrives.
  void tick(uint32_t now) {
    if(state_==State::Trial && trial_started_ &&
        (now<trial_start_ || now-trial_start_>5000)) {stop(Reason::Inconclusive);return;}
    if((state_==State::Trial||state_==State::Hold) && last_pair_ &&
        (now<last_pair_||now-last_pair_>2000))stop(Reason::Stale);
  }
  State state()const{return state_;} Reason reason()const{return reason_;}
  uint16_t requested()const{return requested_;} uint16_t anchorWidth()const{return anchor_width_;}
  bool requireNeutral()const{return require_neutral_;}
 private:
  static constexpr unsigned kHistory=4;
  struct Actual {uint32_t ms=0;uint16_t width=0;int16_t ma=0;int sign=0;float target_mj=NAN;bool executed=false;};
  void startTrial(){
    reference_error_=(err_[0]+err_[1])*.5f;reference_peak_=(peak_[0]+peak_[1])*.5f;
    reference_negative_=negative_[0];
    reference_right_=(right_[0]+right_[1])*.5f;
    reference_total_=(left_[0]+left_[1]+right_[0]+right_[1])*.5f;
    desired_width_=static_cast<uint16_t>(last_width_>5?last_width_-5:0);
    // One exploratory 5 ms reduction per run. A two-cycle response is not
    // sufficient evidence to automatically take further steps.
    const uint16_t floor=anchor_width_>5?anchor_width_-5:0;
    if(desired_width_<floor)desired_width_=floor;
    trial_count_=weak_count_=noncomparable_count_=0;trial_error_sum_=0;trial_started_=false;
    waiting_confirmation_=false;reason_=Reason::None;state_=State::Trial;
  }
  void stop(Reason why){
    if(state_==State::Locked)return;
    waiting_confirmation_=false;reason_=why;
    if(state_==State::Trial||state_==State::Hold||state_==State::Release)state_=State::Release;
    else {state_=State::Waiting;count_=0;last_pair_=0;}
  }
  State state_=State::Waiting;Reason reason_=Reason::None;
  float err_[2]={},peak_[2]={},left_[2]={},right_[2]={},reference_error_=0,reference_peak_=0;
  float reference_right_=0,reference_total_=0,trial_error_sum_=0,targets_[2]={};
  float reference_target_mj_=0,request_target_mj_=NAN,actual_target_mj_=NAN;
  Actual history_[kHistory];unsigned actual_next_=0,actual_count_=0;
  uint32_t last_pair_=0,trial_start_=0,actual_ms_=0;unsigned next_=0,count_=0,trial_count_=0,weak_count_=0;
  uint16_t anchor_width_=0,last_width_=0,desired_width_=0,requested_=0,restore_target_=0,actual_width_=0;
  uint16_t widths_[2]={};
  int16_t actual_ma_=0;int actual_sign_=0;
  NegativeDecision negative_[2],reference_negative_;
  unsigned noncomparable_count_=0;
  bool trial_started_=false;
  bool waiting_confirmation_=false,actual_valid_=false,require_neutral_=false;
};
}
