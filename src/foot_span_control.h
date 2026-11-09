#pragma once
#include <cmath>
#include <cstdint>
#include "foot_trend.h"
#include "foot_gap_trend.h"

// Online experimental correction of body-relative camera marker spans. These
// values are not ground step lengths, and the initial control polarity is an
// empirically motivated hypothesis, not a guaranteed actuator response.
namespace foot_span_control {
// A single sampled extremum may be a fast real swing or a false detection.
// Neither the raw nor trimmed span alone can decide which; withhold feedback
// when discarding that extremum changes the result substantially.
inline bool spanReliable(float raw,float trimmed) {
  return std::isfinite(raw) && std::isfinite(trimmed) && raw>=0.f && trimmed>=0.f &&
      !(raw-trimmed>1.5f && trimmed<raw*.5f);
}
struct Cycle {
  uint32_t start_ms=0, end_ms=0, positive_ms=0;
  float left=NAN, right=NAN, negative_peak=NAN, target_mj=NAN;
  // Context of the negative input: comparisons must not silently treat
  // different early-trigger angles or incoming mechanical states as equal.
  float negative_trigger_deg=0.f,negative_rate_dps=0.f,negative_rpm=0.f;
  float positive_rate_dps=0.f,positive_rpm=0.f;
  uint32_t negative_key=0;
  uint16_t positive_width=0;
  int16_t positive_ma=0;
  int8_t positive_sign=0;
  bool complete=false, negative_known=false, positive_executed=false;
  // Independent quality gate for historical observation. It uses the same
  // image checks as complete except prior-cycle continuity; missing periods
  // are explicitly skipped and never used to assess an intervention.
  bool gap_quality=false, gap_allow=false;
};
enum class State : uint8_t { Collect, ProbeMinus, Restore, ProbePlus, Hold, Fault, ProbeOnce };
class Controller {
 public:
  void reset() { *this=Controller{}; }
  State state()const{return state_;}
  bool fault()const{return state_==State::Fault;}
  bool requireNeutral()const{return fault();}
  uint16_t requested()const{return requested_;}
  uint16_t anchor()const{return baseline_ms_;}
  int16_t anchorCurrent()const{return request_ma_;}
  int direction()const{return offset_ms_<0?-1:offset_ms_>0?1:0;}
  float referenceError()const{return filtered_error_;}
  unsigned acceptedProbeCycles()const{return updates_;}
  int offset()const{return offset_ms_;}
  // Earlier 400mA runs tried the negative offset repeatedly while right
  // turning remained. The 1200mA branch starts its own exploratory trial in
  // the opposite direction; its response is not assumed to be the same.
  void preferHighCurrentFirst(){if(!high_current_initial_){polarity_=1;high_current_initial_=true;}}
  bool selectorUnavailable(uint32_t now) {
    if(fault())return false;
    // A different selector is not a hardware fault. Cancel experimental
    // feedback and reacquire observation before trying another candidate.
    (void)now;gap_trend_.reset();invalidateObservation();pending_=false;request_eligible_=false;return true;
  }
  bool cancelUnsent(uint16_t baseline) {
    (void)baseline;
    if(fault())return false;
    if(confirmed_offset_==0 && !trial_active_) {
      requested_=baseline;discardUnsent();return true;
    }
    // Candidate not sent: restore the ordinary selector; do not pretend that
    // its resulting command tested the requested experimental width.
    if(trial_active_ && confirmed_offset_!=0)retry_after_restore_=true;
    pending_=false;request_changed_=false;offset_ms_=0;requested_=baseline;trial_active_=false;have_observation_=false;
    proposal_sign_=0;
    pending_change_=confirmed_offset_!=0;
    consecutive_=0;last_usable_end_=0;have_reference_=false;state_=State::Collect;return true;
  }
  void cycle(const Cycle& c) {
    if(c.end_ms<=last_end_ || fault())return;
    last_end_=c.end_ms;
    // The gap-tolerant record is observation-only. Verify the exact actual
    // input before retaining either a period or a skipped image period.
    // Never retain observations across an active experiment or restore.
    const Actual* gap_action=nullptr;unsigned gap_actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=actual_[(actual_next_+8-actual_count_+i)%8];
      if(a.ms>c.start_ms && a.ms<c.end_ms){gap_action=&a;++gap_actions;}
    }
    const bool gap_context=c.gap_allow && !trial_active_ && !one_return_ && !retry_wait_ &&
        confirmed_offset_==0 && offset_ms_==0 && !pending_change_ &&
        c.negative_known && c.positive_executed && c.positive_sign==1 &&
        c.positive_ma==1200 && c.start_ms<c.positive_ms && c.positive_ms<c.end_ms &&
        std::isfinite(c.negative_peak) && c.negative_peak>0 &&
        std::isfinite(c.negative_trigger_deg) && std::isfinite(c.negative_rate_dps) &&
        std::isfinite(c.negative_rpm) && std::isfinite(c.positive_rate_dps) &&
        std::isfinite(c.positive_rpm) && gap_actions==1 && gap_action &&
        gap_action->ms==c.positive_ms && gap_action->executed && gap_action->ma==c.positive_ma &&
        gap_action->sign==c.positive_sign && gap_action->width==c.positive_width &&
        gap_action->feedback_changed_ms==last_feedback_changed_ms_ &&
        std::isfinite(gap_action->target_mj) && std::isfinite(c.target_mj) &&
        std::fabs(gap_action->target_mj-c.target_mj)<=.001f &&
        gap_trend_.compatible(c.positive_ma,c.target_mj);
    if(!gap_context)gap_trend_.reset();
    else if(c.gap_quality)gap_trend_.accept(c.start_ms,c.end_ms,c.left,c.right,c.positive_ma,c.target_mj);
    else gap_trend_.skip(c.start_ms,c.end_ms);
    if(!c.complete || !c.negative_known || !c.positive_executed || !supportedCurrent(c.positive_ma) ||
       c.positive_sign!=1 || c.start_ms>=c.positive_ms || c.positive_ms>=c.end_ms ||
       !std::isfinite(c.left) || !std::isfinite(c.right) || c.left<0 || c.right<0 ||
       !std::isfinite(c.negative_peak) || c.negative_peak<=0 ||
       !std::isfinite(c.target_mj) || c.target_mj<=0 ||
       !std::isfinite(c.negative_trigger_deg) || !std::isfinite(c.negative_rate_dps) ||
       !std::isfinite(c.negative_rpm) || !std::isfinite(c.positive_rate_dps) ||
       !std::isfinite(c.positive_rpm)) {
      invalidateObservation();return;
    }
    const Actual* action=nullptr;unsigned actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=actual_[(actual_next_+8-actual_count_+i)%8];
      if(a.ms>c.start_ms && a.ms<c.end_ms){action=&a;++actions;}
    }
    if(actions!=1 || !action || action->ms!=c.positive_ms || !action->executed ||
       action->ma!=c.positive_ma || action->sign!=c.positive_sign ||
       action->width!=c.positive_width || !std::isfinite(action->target_mj) ||
       std::fabs(action->target_mj-c.target_mj)>.001f) {invalidateObservation();return;}
    // Each actual command retains the input-change and feedback-change
    // timestamps that were known at that instant. Future ordinary commands
    // must not retrospectively invalidate this earlier camera cycle.
    // An older feedback generation after a new intervention is not a
    // response to the new intervention and must not increment it again.
    if(action->feedback_changed_ms!=last_feedback_changed_ms_)return;
    // After an interrupted applied trial has actually returned to baseline,
    // allow the ordinary controller to settle before reacquiring evidence.
    // Only cycles that start after the wait count toward a new proposal.
    if(retry_wait_) {
      const uint32_t elapsed=c.start_ms-retry_start_ms_;
      if(elapsed>=0x80000000u || elapsed<5000u) {invalidateObservation();return;}
      retry_wait_=false;
    }
    const bool continuous=last_usable_end_ && c.start_ms==last_usable_end_;
    consecutive_=continuous?consecutive_+1:1;last_usable_end_=c.end_ms;
    // The four-cycle trend describes observed foot motion even as ordinary
    // input settings evolve. A separate two-cycle window is required as a
    // comparable reference before any correction may be issued.
    const float e=(c.left-c.right)/(c.left+c.right+.1f);
    const int base_delta=static_cast<int>(action->baseline)-static_cast<int>(trend_baseline_);
    if(trial_active_ && trial_count_>0 && !continuous) {invalidateObservation();return;}
    if(matched_.count() && (c.positive_ma!=trend_context_.positive_ma ||
       std::fabs(c.target_mj-trend_context_.target_mj)>.05f || base_delta>5 || base_delta< -5 ||
       !comparableNegative(c.negative_key,trend_context_.negative_key) ||
       std::fabs(c.negative_trigger_deg-trend_context_.negative_trigger_deg)>.5f ||
       std::fabs(c.negative_rate_dps-trend_context_.negative_rate_dps)>10.f ||
       std::fabs(c.negative_rpm-trend_context_.negative_rpm)>100.f ||
       std::fabs(c.positive_rate_dps-trend_context_.positive_rate_dps)>15.f ||
       std::fabs(c.positive_rpm-trend_context_.positive_rpm)>150.f)) {
      if(trial_active_ && trial_count_>0) {invalidateObservation();return;}
      matched_.reset();
    }
    if(!continuous) {trend_.reset();matched_.reset();}
    if(matched_.count()) {matched_previous_=trend_context_;matched_previous_baseline_=trend_baseline_;}
    trend_context_=c;trend_baseline_=action->baseline;
    if(!trend_.cycle(c.start_ms,c.end_ms,c.left,c.right,c.negative_peak,true) ||
       !matched_.cycle(c.start_ms,c.end_ms,c.left,c.right,c.negative_peak,true)) {
      invalidateObservation();return;
    }
    const auto avg=trend_.average();
    const auto recent=matched_.average();
    const float trend_error=trend_.ready()?avg.error:e;
    const int sign=trend_error>.10f ? 1 : trend_error<-.10f ? -1 : 0;
    const int recent_sign=matched_.ready() ? (recent.error>0.f ? 1 : recent.error<0.f ? -1 : 0) : 0;
    imbalance_sign_=sign;
    imbalance_count_=trend_.ready() && sign && sign==recent_sign ? 2u : 0u;
    // A proposed direction needs the long trend and the latest comparable
    // observations to agree; a safety rollback is independent.
    const int single_sign=e>.10f ? 1 : e<-.10f ? -1 : 0;
    const bool one_supported=trend_.ready() && sign && sign==single_sign;
    if(pending_change_ && proposal_sign_ &&
       (gap_proposal_ ? gap_trend_.direction(c.end_ms)!=proposal_sign_ :
        (!one_supported || sign!=proposal_sign_ ||
         (!one_proposal_ && (imbalance_count_<2 || imbalance_sign_!=proposal_sign_))))) {
      offset_ms_=confirmed_offset_;pending_change_=false;proposal_sign_=0;one_proposal_=false;gap_proposal_=false;
      state_=offset_ms_==0 ? State::Collect : State::Hold;
    }
    // Mixed-context trend averages must never become the trial reference.
    observed_left_=matched_.ready()?recent.left:c.left;
    observed_right_=matched_.ready()?recent.right:c.right;
    observed_peak_=matched_.ready()?recent.negative_peak:c.negative_peak;
    observed_error_=matched_.ready()?recent.error:e;
    observed_current_=c.positive_ma;observed_target_=c.target_mj;
    observed_negative_key_=c.negative_key;
    observed_negative_trigger_=c.negative_trigger_deg;
    observed_negative_rate_=c.negative_rate_dps;observed_negative_rpm_=c.negative_rpm;
    observed_positive_rate_=c.positive_rate_dps;observed_positive_rpm_=c.positive_rpm;
    observed_baseline_=action->baseline;
    have_observation_=true;
    if(!have_reference_) {
      filtered_error_=e;reference_peak_=c.negative_peak;
      reference_left_=c.left;reference_right_=c.right;have_reference_=true;return;
    }
    filtered_error_=trend_error;
    if(offset_ms_==0) {
      reference_peak_=.8f*reference_peak_+.2f*c.negative_peak;
      reference_left_=.8f*reference_left_+.2f*c.left;
      reference_right_=.8f*reference_right_+.2f*c.right;
    }
    // Protect the gait even in the overlapping cycle, before waiting for a
    // complete post-intervention trend. This unwinds the whole offset.
    if(trial_active_ && c.end_ms>last_feedback_changed_ms_ &&
       c.negative_peak<trial_peak_*.7f && c.negative_peak<trial_peak_-1.f) {
      polarity_=-polarity_;invalidateObservation();return;
    }
    if(pending_change_ || consecutive_<2 ||
       (action->feedback_changed_ms && c.start_ms<=action->feedback_changed_ms))return;
    // Capture a complete trend window after the actual changed command.
    // Negative-side input/contact can change too: this is trend evidence,
    // never a claim that the pulse caused the measured difference.
    if(trial_active_) {
      const int base_delta=static_cast<int>(action->baseline)-static_cast<int>(trial_baseline_);
      if(!trial_reference_valid_ || c.positive_ma!=trial_current_ || std::fabs(c.target_mj-trial_target_)>.05f ||
         base_delta>5 || base_delta< -5 || !comparableNegative(c.negative_key,trial_negative_key_) ||
         std::fabs(c.negative_trigger_deg-trial_negative_trigger_)>.5f ||
         std::fabs(c.negative_rate_dps-trial_negative_rate_)>10.f ||
         std::fabs(c.negative_rpm-trial_negative_rpm_)>100.f ||
         std::fabs(c.positive_rate_dps-trial_positive_rate_)>15.f ||
         std::fabs(c.positive_rpm-trial_positive_rpm_)>150.f) {
        invalidateObservation();return;
      }
      const float weak=trial_right_?c.right:c.left;
      trial_ratio_sum_+=e;trial_weak_sum_+=weak;trial_peak_sum_+=c.negative_peak;
      if(++trial_count_<foot_trend::Window::kCycles)return;
      const float scale=1.f/foot_trend::Window::kCycles;
      const float ratio=std::fabs(trial_ratio_sum_*scale),weak_mean=trial_weak_sum_*scale,peak=trial_peak_sum_*scale;
      trial_active_=false;
      const bool weak_kept=weak_mean>=trial_weak_*.8f && weak_mean>=trial_weak_-.10f;
      // Balance is only a useful success when the initially weaker foot has
      // actually increased. A shrinking stronger foot alone must not be
      // mistaken for a response to the changed command. This is a small
      // observation threshold, not a precision or ground-stride guarantee.
      const bool weak_improved=weak_mean>=trial_weak_*1.05f && weak_mean>=trial_weak_+.05f;
      const bool peak_kept=peak>=trial_peak_*.85f && peak>=trial_peak_-.5f;
      // No clear ratio improvement, loss of the smaller foot, or loss of
      // negative oscillation: unwind this offset step and explore oppositely.
      if(!weak_kept || !weak_improved || !peak_kept || ratio+.02f>=trial_ratio_) {
        polarity_=-polarity_;
        const int restored=towardZero(offset_ms_, c.positive_ma==1200 ? 5 : 10);
        state_=restored==0?State::Collect:State::Restore;
        if(restored!=offset_ms_){offset_ms_=restored;pending_change_=true;proposal_sign_=0;}
        return;
      }
    }
    // Maintain a weaker-side/peak guard even after the offset reaches a cap.
    const float weak=reference_right_<=reference_left_?c.right:c.left;
    const float refweak=std::fmin(reference_left_,reference_right_);
    const bool weak_loss=weak<refweak*.8f && weak<refweak-.05f;
    const bool peak_loss=c.negative_peak<reference_peak_-.5f;
    // Baseline motion alone is not the result of an applied trial. Do not
    // reverse the exploratory polarity when there is nothing to roll back.
    if(offset_ms_!=0 && (weak_loss || peak_loss))++weak_count_;else weak_count_=0;
    if(weak_count_>=2) {
      weak_count_=0;polarity_=-polarity_;
      const int restored=towardZero(offset_ms_, c.positive_ma==1200 ? 5 : 10);
      state_=restored==0?State::Collect:State::Restore;
      if(restored!=offset_ms_) {offset_ms_=restored;pending_change_=true;proposal_sign_=0;}
      return;
    }
    if(one_return_)return; // never accumulate a one-shot intervention
    if(!trend_.ready()) {
      // Strict four-consecutive-cycle trials take priority. If image quality
      // interrupted only the historical trend, permit one bounded 1200 mA
      // observation probe with two fresh, matched consecutive cycles.
      const int gs=gap_trend_.direction(c.end_ms);
      if(gs && matched_.ready() && recent_sign==gs && single_sign==gs &&
         confirmed_offset_==0 && offset_ms_==0 && !pending_change_ && !trial_active_ &&
         !one_return_ && c.positive_ma==1200) {
        offset_ms_=gs*polarity_*5;pending_change_=true;proposal_sign_=gs;
        one_proposal_=true;gap_proposal_=true;state_=State::ProbeOnce;return;
      }
      if(offset_ms_==0 && !pending_change_)state_=State::Collect;
      return;
    }
    if(std::fabs(filtered_error_)<=.10f){state_=State::Hold;return;}
    // The strict two-cycle reference has priority. When it is unavailable or
    // disagrees with the long trend, allow only a single 1200 mA exploration
    // supported by a substantial same-sign latest cycle and long trend.
    // A stale long-window imbalance should not launch another new input when
    // the freshest full cycle has already balanced or reversed. The long
    // trend is still required: a single imbalanced cycle cannot trigger this.
    const bool strict=matched_.ready() && imbalance_count_>=2 && one_supported &&
        (filtered_error_>0.f ? 1 : -1)==imbalance_sign_;
    if(!strict) {
      if(!matched_.ready() && confirmed_offset_==0 && offset_ms_==0 && !pending_change_ &&
         !trial_active_ && c.positive_ma==1200 && one_supported) {
        offset_ms_=sign*polarity_*5;pending_change_=true;proposal_sign_=sign;
        one_proposal_=true;state_=State::ProbeOnce;
      } else if(offset_ms_==0 && !pending_change_)state_=State::Collect;
      return;
    }
    const int step=(filtered_error_>0.f ? 1 : -1)*polarity_*(c.positive_ma==1200 ? 5 : 10);
    const int proposal=offset_ms_+step;
    const int bounded=proposal < -30 ? -30 : proposal > 30 ? 30 : proposal;
    state_=bounded<0?State::ProbeMinus:bounded>0?State::ProbePlus:State::Collect;
    if(bounded!=offset_ms_) {offset_ms_=bounded;pending_change_=true;proposal_sign_=imbalance_sign_;}
  }

  uint16_t limit(uint32_t now,uint16_t baseline,int16_t ma,int sign,
                 uint16_t existing_cap,bool allowed,float target_mj,
                 float positive_rate_dps=0.f,float positive_rpm=0.f) {
    const uint16_t cap=existing_cap<100?existing_cap:100;
    const uint16_t safe=baseline<cap?baseline:cap;
    baseline_ms_=baseline;request_ma_=ma;request_target_mj_=target_mj;
    requested_=safe;pending_=false;request_changed_=false;request_eligible_=false;
    if(!allowed || sign!=1 || !gap_trend_.compatible(ma,target_mj))gap_trend_.reset();
    if(fault()){requested_=0;return 0;}
    // The ordinary selector may change current every step. Keep the feedback
    // offset and track the actual current/width; a time offset at a different
    // current is not claimed to deliver the same physical energy.
    if(!allowed || !supportedCurrent(ma) || sign!=1 || !std::isfinite(target_mj) || target_mj<=0 || safe<10) {
      // The caller suppresses real hardware faults and uses the normal
      // selector for unsupported modes, rather than latching a zero output.
      state_=State::Collect;return safe;
    }
    request_eligible_=true;
    // Once a single exploratory pulse has actually been sent, the next
    // eligible positive command is the ordinary baseline, not another trial.
    if(one_return_) {
      offset_ms_=0;pending_change_=confirmed_offset_!=0;requested_=safe;
      pending_=pending_change_;state_=State::ProbeOnce;return safe;
    }
    // An offset identified at one current is not transferable to a different
    // current. Restore the ordinary selector and reacquire fresh evidence.
    if(confirmed_offset_!=0 && ma!=confirmed_current_) {
      invalidateObservation();return safe;
    }
    // A candidate proposed from an older camera cycle must not be submitted
    // after a missing/invalid observation. Ordinary output remains available.
    if(!have_observation_ && offset_ms_!=0)invalidateObservation();
    if(last_usable_end_ && (now<last_usable_end_ || now-last_usable_end_>2500)) {
      // Stale camera data cannot justify retaining any experimental offset.
      // Continue with the ordinary selector while observations reacquire.
      invalidateObservation();
    }
    if(!last_usable_end_)return safe;
    // Do not submit a proposed change if its own pre-input state already
    // differs from the reference. Checking only later cycles would allow the
    // intervention itself to occur at a quite different wheel/body speed.
    if(pending_change_ && offset_ms_!=0) {
      const int delta=static_cast<int>(observed_baseline_)-static_cast<int>(baseline);
      if((gap_proposal_ && gap_trend_.direction(now)!=proposal_sign_) ||
         !have_observation_ || !std::isfinite(positive_rate_dps) || !std::isfinite(positive_rpm) ||
         observed_current_!=ma || std::fabs(observed_target_-target_mj)>.05f ||
         delta>5 || delta< -5 || std::fabs(observed_positive_rate_-positive_rate_dps)>15.f ||
         std::fabs(observed_positive_rpm_-positive_rpm)>150.f ||
         ((!one_proposal_ || gap_proposal_) && (!matched_.ready() ||
          matched_previous_.positive_ma!=ma || std::fabs(matched_previous_.target_mj-target_mj)>.05f ||
          static_cast<int>(matched_previous_baseline_)-static_cast<int>(baseline)>5 ||
          static_cast<int>(matched_previous_baseline_)-static_cast<int>(baseline)< -5 ||
          std::fabs(matched_previous_.positive_rate_dps-positive_rate_dps)>15.f ||
          std::fabs(matched_previous_.positive_rpm-positive_rpm)>150.f))) {
        // With no applied feedback there is nothing to undo. Keep the
        // measured long trend, and reacquire two comparable input cycles.
        if(confirmed_offset_==0 && !trial_active_)discardUnsent();
        else invalidateObservation();
        return safe;
      }
    }
    // Recheck the context for every pulse in an active experiment. A staged
    // target change makes another corrected pulse incomparable even when the
    // offset itself has not changed since its first confirmed application.
    const int trial_base_delta=static_cast<int>(baseline)-static_cast<int>(trial_baseline_);
    if(trial_active_ && (ma!=trial_current_ || std::fabs(target_mj-trial_target_)>.05f ||
       trial_base_delta>5 || trial_base_delta< -5 || !std::isfinite(positive_rate_dps) ||
       !std::isfinite(positive_rpm) || std::fabs(positive_rate_dps-trial_positive_rate_)>15.f ||
       std::fabs(positive_rpm-trial_positive_rpm_)>150.f)) {
      invalidateObservation();return safe;
    }
    const int trial_step=ma==1200 ? 5 : 10;
    // If a newly requested first step has no room in that direction, try the
    // other direction explicitly instead of repeatedly counting a blocked
    // proposal. This is an exploratory intervention, not proof that the
    // opposite sign improves gait. Use the current-dependent feedback step.
    if(pending_change_ && confirmed_offset_==0 && offset_ms_==trial_step && safe==cap && safe>=10+trial_step) {
      polarity_=-polarity_;offset_ms_=-trial_step;state_=one_proposal_?State::ProbeOnce:State::ProbeMinus;
    } else if(pending_change_ && confirmed_offset_==0 && offset_ms_==-trial_step && safe==10 && cap>=10+trial_step) {
      polarity_=-polarity_;offset_ms_=trial_step;state_=one_proposal_?State::ProbeOnce:State::ProbePlus;
    }
    int wanted=static_cast<int>(safe)+offset_ms_;
    // Anti-windup: a correction blocked by a physical cap is not an
    // applied intervention. Do not hide an unexecuted offset that could
    // suddenly reappear when the ordinary cap later increases.
    if(wanted<10) {wanted=10;offset_ms_=10-static_cast<int>(safe);}
    if(wanted>cap) {wanted=cap;offset_ms_=static_cast<int>(cap)-safe;}
    // A changing cap can itself remove an already-applied offset. Record the
    // offset transition only after the resulting command is confirmed too.
    if(offset_ms_!=confirmed_offset_)pending_change_=true;
    if(one_proposal_ && offset_ms_==0) {discardUnsent();return safe;}
    if(offset_ms_==0 && (state_==State::ProbeMinus || state_==State::ProbePlus))state_=State::Collect;
    // Each feedback OFFSET update is limited to its current-dependent step. The ordinary
    // selector can change its own width or mode by more than 10ms; do not
    // suppress that normal output merely because its model branch changed.
    // The real absolute hardware/gradual-start cap still takes precedence.
    requested_=static_cast<uint16_t>(wanted);
    request_changed_=requested_!=safe;
    pending_=request_changed_;
    return requested_;
  }
  void confirm(uint32_t now,bool executed,int16_t ma,int sign,uint16_t width) {
    Actual& a=actual_[actual_next_];a.ms=now;a.width=width;a.ma=ma;a.sign=sign;
    a.executed=executed;a.target_mj=request_target_mj_;a.baseline=baseline_ms_;
    actual_next_=(actual_next_+1)%8;if(actual_count_<8)++actual_count_;
    if(pending_ && (!executed || ma!=request_ma_ || sign!=1 || width!=requested_)) {
      gap_trend_.reset();gap_proposal_=false;state_=State::Fault;pending_=false;return;
    }
    if(executed && supportedCurrent(ma) && sign==1 && width>0) {
      const bool applied_feedback=pending_change_ && request_eligible_ &&
          ma==request_ma_ && sign==1 && width==requested_ && offset_ms_!=confirmed_offset_;
      // Count only confirmed feedback transitions.
      if(applied_feedback){
        const bool one_applied=one_proposal_ && confirmed_offset_==0 && offset_ms_!=0;
        const bool one_restored=one_return_ && offset_ms_==0;
        matched_.reset();trend_.reset();gap_trend_.reset();gap_proposal_=false;
        last_feedback_changed_ms_=now;confirmed_offset_=offset_ms_;confirmed_current_=offset_ms_!=0 ? ma : 0;proposal_sign_=0;++updates_;
        if(one_applied) {one_proposal_=false;one_return_=true;state_=State::ProbeOnce;}
        if(one_restored) {
          one_return_=false;retry_after_restore_=false;retry_wait_=true;retry_start_ms_=now;
          have_observation_=false;last_usable_end_=0;consecutive_=0;have_reference_=false;state_=State::Collect;
        }
        if(confirmed_offset_==0 && retry_after_restore_) {
          retry_after_restore_=false;retry_wait_=true;retry_start_ms_=now;
        }
        trial_active_=!one_applied && offset_ms_!=0 && have_observation_;
        if(trial_active_) {
          trial_ratio_=std::fabs(observed_error_);
          trial_right_=observed_right_<=observed_left_;
          trial_weak_=trial_right_?observed_right_:observed_left_;
          trial_peak_=observed_peak_;trial_current_=ma;trial_target_=request_target_mj_;trial_baseline_=baseline_ms_;
          trial_negative_key_=observed_negative_key_;
          trial_negative_trigger_=observed_negative_trigger_;
          trial_negative_rate_=observed_negative_rate_;trial_negative_rpm_=observed_negative_rpm_;
          trial_positive_rate_=observed_positive_rate_;trial_positive_rpm_=observed_positive_rpm_;
          const int before_delta=static_cast<int>(observed_baseline_)-static_cast<int>(baseline_ms_);
          trial_reference_valid_=observed_current_==ma && std::fabs(observed_target_-request_target_mj_)<=.05f &&
              before_delta<=5 && before_delta>= -5;
          trial_ratio_sum_=trial_weak_sum_=trial_peak_sum_=0.f;trial_count_=0;
        }
      }
      if(request_eligible_ && ma==request_ma_ && sign==1 && width==requested_)pending_change_=false;
    }
    a.feedback_changed_ms=last_feedback_changed_ms_;
    pending_=false;
  }
 private:
  void discardUnsent() {
    // This drops only an unexecuted proposal/reference. A failed or active
    // intervention must go through invalidateObservation() and restoration.
    matched_.reset();gap_trend_.reset();gap_proposal_=false;offset_ms_=0;proposal_sign_=0;pending_change_=false;one_proposal_=false;
    pending_=false;request_changed_=false;have_observation_=false;
    imbalance_count_=0;state_=State::Collect;
  }
  void invalidateObservation() {
    // Drop both an active trial and a proposal not yet sent. Keeping the
    // proposal after clearing the reference would issue an untracked pulse.
    if(trial_active_ && confirmed_offset_!=0)retry_after_restore_=true;
    trend_.reset();matched_.reset();
    consecutive_=0;last_usable_end_=0;weak_count_=0;have_reference_=false;
    imbalance_count_=0;imbalance_sign_=0;proposal_sign_=0;one_proposal_=false;gap_proposal_=false;
    have_observation_=false;trial_active_=false;offset_ms_=0;
    pending_change_=confirmed_offset_!=0;
    state_=pending_change_?State::Restore:State::Collect;
  }
  static bool supportedCurrent(int16_t ma){return ma==300 || ma==350 || ma==400 || ma==1200;}
  // Keep distinct negative inputs out of the same response comparison.
  // A 10 ms tolerance accommodates observed normal negative pulse changes;
  // similar commands still do not establish an isolated causal response.
  static bool comparableNegative(uint32_t a,uint32_t b){
    if(a==1 || b==1)return a==1 && b==1;
    if(!(a&0x80000000u) || !(b&0x80000000u) || ((a^b)&0x7fff8000u))return false;
    const int delta=static_cast<int>(a&0x7fffu)-static_cast<int>(b&0x7fffu);
    return delta>=-10 && delta<=10;
  }
  static int towardZero(int value,int step){return value>step?value-step:value< -step?value+step:0;}
  struct Actual {uint32_t ms=0,feedback_changed_ms=0;
    uint16_t width=0,baseline=0;int16_t ma=0;int8_t sign=0;bool executed=false;float target_mj=NAN;};
  Actual actual_[8];unsigned actual_next_=0,actual_count_=0;
  foot_trend::Window trend_;foot_trend::WindowT<2> matched_;
  Cycle trend_context_{},matched_previous_{};uint16_t trend_baseline_=0,matched_previous_baseline_=0;
  State state_=State::Collect;
  uint32_t last_end_=0,last_usable_end_=0,last_feedback_changed_ms_=0,retry_start_ms_=0;
  uint32_t observed_negative_key_=0,trial_negative_key_=0;
  uint16_t baseline_ms_=0,requested_=0;
  int16_t request_ma_=0;
  float request_target_mj_=NAN,filtered_error_=0,reference_peak_=0,reference_left_=0,reference_right_=0;
  float observed_left_=0,observed_right_=0,observed_peak_=0,observed_error_=0,observed_target_=0;
  float observed_negative_trigger_=0,observed_negative_rate_=0,observed_negative_rpm_=0;
  float trial_negative_trigger_=0,trial_negative_rate_=0,trial_negative_rpm_=0;
  float observed_positive_rate_=0,observed_positive_rpm_=0;
  float trial_positive_rate_=0,trial_positive_rpm_=0;
  float trial_ratio_=0,trial_weak_=0,trial_peak_=0,trial_target_=0;
  float trial_ratio_sum_=0,trial_weak_sum_=0,trial_peak_sum_=0;
  int16_t observed_current_=0,trial_current_=0,confirmed_current_=0;uint16_t observed_baseline_=0,trial_baseline_=0;unsigned trial_count_=0;
  bool have_observation_=false,trial_active_=false,trial_right_=true,trial_reference_valid_=false;
  int offset_ms_=0,confirmed_offset_=0,polarity_=-1;
  int imbalance_sign_=0,proposal_sign_=0;unsigned imbalance_count_=0;
  unsigned consecutive_=0,weak_count_=0,updates_=0;
  bool have_reference_=false,pending_=false,request_changed_=false,pending_change_=false,request_eligible_=false;
  bool high_current_initial_=false,retry_after_restore_=false,retry_wait_=false;
  bool one_proposal_=false,one_return_=false,gap_proposal_=false;
  // Keep the historical window after established control members.
  foot_gap_trend::Window gap_trend_;
};
}
