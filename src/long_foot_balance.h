#pragma once
#include <cmath>
#include <cstdint>

// Experimental feedback from body-relative camera motion, not ground stride.
// Physical response direction is learned by bounded trials, not assumed.
namespace long_foot_balance {
class Controller {
 public:
  static constexpr unsigned kWindow=5;
  static constexpr unsigned kPairWindow=3;
  static constexpr uint32_t kMaxAgeMs=12000;
  static constexpr uint32_t kProposalAgeMs=1500;
  static constexpr int kStepMs=5;
  static constexpr int kMaxOffsetMs=20;
  enum class State : uint8_t { Collect, Trial, Hold, Restore };
  void reset(){*this=Controller{};}
  State state()const{return state_;}
  unsigned samples()const{return n_;}
  int offset()const{return offset_;}
  int requestedOffset()const{return requested_;}
  uint32_t lastChangeMs()const{return changed_at_;}
  // The negative-side experiment may start only with no pending or applied
  // foot-width feedback, so it cannot silently discard a useful correction.
  bool idleForNegativeTrial()const{return offset_==0 && !pending_ &&
      (state_==State::Collect || state_==State::Hold) && !trialActive();}
  int trialReferenceOffset()const{return active_reference_;}
  int proposedTrialReferenceOffset()const{return pending_ && state_!=State::Restore ? offset_ : active_reference_;}
  uint32_t trialGeneration()const{return trial_generation_;}
  bool trialActive()const{return state_==State::Trial || (state_==State::Hold && trial_generation_ && offset_!=active_reference_);}
  float error()const{return error_;}
  // 'valid' is an image-quality decision; a missing image is not zero swing.
  // 'healthy' is independent actuator/IMU/run safety, not an image miss.
  void observe(uint32_t start,uint32_t now,float left,float right,float plus_peak,float minus_peak,bool valid,bool healthy,bool trial_effective=true,unsigned window=kWindow,float sequence_excess=NAN) {
    if(!healthy) {clearForSafety();return;}
    if(!now || start>=now || (changed_at_ && now<=changed_at_) ||
       !std::isfinite(plus_peak) || !std::isfinite(minus_peak) || plus_peak<=0 || minus_peak<=0)return;
    // A serious drop in either rocking peak must unwind without waiting for
    // a complete foot-response window. Existing emergency limits remain primary.
    if((offset_!=0 || state_==State::Trial) && safety_plus_>0 && safety_minus_>0 &&
       (plus_peak<safety_plus_*.65f || minus_peak<safety_minus_*.65f)) {
      if(offset_==0) {clearForSafety();return;}
      requested_=0;state_=State::Restore;pending_=true;clearSamples();return;
    }
    if((changed_at_ && start<=changed_at_) || (fresh_after_ && start<=fresh_after_))return;
    if(!valid || !std::isfinite(left) || !std::isfinite(right) || left<0 || right<0)return;
    if(trialActive() && !trial_effective)return;
    if(last_sample_ && now<=last_sample_)return;
    if(last_sample_ && now-last_sample_>kMaxAgeMs)clearSamples();
    last_sample_=now;
    // Keep the reference moving while an unsent proposal waits behind a
    // guard. Freeze only at send time, never reuse an old startup window.
    if(state_!=State::Trial && state_!=State::Restore) {
      referenceSample(start,now,left,right,plus_peak,minus_peak,window,sequence_excess);
      return;
    }
    if(pending_)return;
    if(n_ && (now<window_start_ || now-window_start_>kMaxAgeMs))clearSamples();
    if(!n_)window_start_=start;
    left_sum_+=left;right_sum_+=right;plus_sum_+=plus_peak;minus_sum_+=minus_peak;
    if(std::isfinite(sequence_excess)) {sequence_sum_+=sequence_excess;++sequence_count_;}
    ++n_;
    if(n_<window || state_==State::Restore)return;
    const float inv=1.f/static_cast<float>(n_);
    const float l=left_sum_*inv,r=right_sum_*inv,p=plus_sum_*inv,m=minus_sum_*inv;
    // Sum spans before dividing so tiny noisy cycles do not have equal
    // influence to strong, well-observed foot motion.
    const float e=(left_sum_-right_sum_)/(left_sum_+right_sum_+.1f*window);error_=e;
    if(l+r<.3f) {clearSamples();return;}
    if(state_==State::Trial) {
      const float weak=right_weak_ ? r:l;
      // A rapidly changing body oscillation confounds the test. Neither
      // credit nor reverse it; collect another complete response window.
      if(p>ref_plus_*1.15f || m>ref_minus_*1.15f) {clearSamples();return;}
      const float scale=(p+m)/(ref_plus_+ref_minus_);
      const float normalized_weak=weak/scale;
      const bool better=std::fabs(e)+.02f<std::fabs(ref_error_) && normalized_weak>=ref_weak_+.05f &&
        normalized_weak>=ref_weak_*1.05f && p>=ref_plus_*.85f && m>=ref_minus_*.85f &&
        (!std::isfinite(ref_sequence_) || (sequence_count_==window && sequence_sum_/window<=ref_sequence_+.1f));
      if(!better) {requested_=previous_;state_=State::Restore;pending_=true;polarity_=-polarity_;clearSamples();return;}
      state_=State::Hold;clearSamples();clearReference();return;
    }
    clearSamples();
  }
  // Pair adjacent complete cycles, never pairing across a missing period or
  // an input change. Each member must be an effective, accepted trial when
  // an intervention is active. The per-cycle safety guard runs even for an
  // unusable camera period, before any pairing or statistical averaging.
  void observeCycle(uint32_t start,uint32_t end,float left,float right,float plus,float minus,bool valid,bool healthy,bool effective=true) {
    observe(start,end,NAN,NAN,plus,minus,false,healthy,false,kPairWindow);
    if(!healthy || !start || end<=start) {pair_pending_=false;early_bad_count_=0;return;}
    // A foot trial cannot be evaluated without fresh camera observations.
    // Two consecutive wholly post-command invalid cycles make the trial
    // inconclusive: restore its prior setting. After the restore is accepted,
    // the next fresh experiment explores the other direction; this is not a
    // claim that either direction improved or harmed walking.
    // An invalid observation is never treated as a zero-length step.
    if(state_==State::Trial && !pending_ && changed_at_ && start>changed_at_) {
      if(!valid) {
        missing_cycles_=(missing_end_==start ? missing_cycles_+1 : 1);
        missing_end_=end;
        if(missing_cycles_>=2) {
          requested_=previous_;pending_=true;state_=State::Restore;reverse_after_restore_=true;
          missing_cycles_=0;missing_end_=0;pair_pending_=false;
          clearSamples();return;
        }
      } else {missing_cycles_=0;missing_end_=0;}
    } else {missing_cycles_=0;missing_end_=0;}
    if(pair_last_end_ && start!=pair_last_end_) {
      // An unknown cycle boundary loses the alternating phase. Never compare
      // the old phase window with newly anchored pairs. A trial with an
      // unresolvable phase returns to the last accepted setting.
      clearSamples();clearReference();pair_pending_=false;pair_next_second_=false;early_bad_count_=0;
      if(state_==State::Trial) {requested_=previous_;pending_=true;state_=State::Restore;}
      else if(pending_ && state_!=State::Restore) {requested_=offset_;pending_=false;}
    }
    pair_last_end_=end;
    const bool first=!pair_next_second_;
    pair_next_second_=!pair_next_second_;
    if(!valid ||
       !std::isfinite(left) || !std::isfinite(right) || left<0 || right<0 ||
       !std::isfinite(plus) || !std::isfinite(minus) || plus<=0 || minus<=0 ||
       (changed_at_ && start<=changed_at_) || (fresh_after_ && start<=fresh_after_) ||
       (trialActive() && !effective)) {
      pair_pending_=false;early_bad_count_=0;return;
    }
    if(first) {
      pair_start_=start;pair_end_=end;pair_left_=left;pair_right_=right;
      pair_plus_=plus;pair_minus_=minus;pair_effective_=effective;
      pair_generation_=trial_generation_;pair_pending_=true;return;
    }
    if(!pair_pending_ || pair_end_!=start || pair_generation_!=trial_generation_) {
      pair_pending_=false;early_bad_count_=0;return;
    }
    const uint32_t pair_start=pair_start_;
    const float l=(pair_left_+left)*.5f,r=(pair_right_+right)*.5f;
    const float p=(pair_plus_+plus)*.5f,m=(pair_minus_+minus)*.5f;
    const bool pair_effective=pair_effective_ && effective;
    pair_pending_=false;
    // Experimental loss limiter: demand two adjacent, individually bad
    // effective pairs. Thresholds are provisional, not identified plant
    // dynamics, and camera-relative motion is not ground stride. Reject
    // comparisons during appreciable body-amplitude changes.
    if(state_==State::Trial && !pending_ && pair_effective &&
       ref_plus_>0.f && ref_minus_>0.f) {
      const bool comparable=p>=ref_plus_*.85f && p<=ref_plus_*1.15f &&
          m>=ref_minus_*.85f && m<=ref_minus_*1.15f;
      const float scale=(p+m)/(ref_plus_+ref_minus_);
      const float weak=(right_weak_?r:l)/scale;
      const float e=(l-r)/(l+r+.1f);
      const bool same_side=(right_weak_ ? e>0.f : e<0.f);
      const bool bad=comparable && same_side && weak<ref_weak_*.70f &&
          weak<ref_weak_-.15f && std::fabs(e)>std::fabs(ref_error_)+.15f;
      if(bad && (!early_bad_end_ || pair_start==early_bad_end_)) ++early_bad_count_;
      else early_bad_count_=bad?1u:0u;
      early_bad_end_=end;
      if(early_bad_count_>=2) {
        requested_=previous_;state_=State::Restore;pending_=true;
        // Only reverse search after the restore command is accepted; this
        // guard does not prove the physical response polarity.
        reverse_after_restore_=true;
        early_bad_count_=0;early_bad_end_=0;clearSamples();return;
      }
    } else {early_bad_count_=0;early_bad_end_=0;}
    // This is a camera-relative sequence metric, not ground-forward travel.
    // It becomes nonzero when a weak right period precedes a large left one.
    const float sequence=pair_right_<pair_left_*.65f ? std::fmax(0.f,left-pair_right_) : 0.f;
    observe(pair_start,end,l,r,p,m,true,healthy,pair_effective,kPairWindow,sequence);
  }
  // The returned value is a pre-guard candidate. Always compare against the
  // ordinary baseline passed through an independent copy of the same guard.
  uint16_t propose(uint32_t now,uint16_t baseline,uint16_t cap,bool healthy) {
    if(!healthy) {clearForSafety();return baseline;}
    // A single healthy first cycle is not yet a pair. Do not discard it
    // merely because an ordinary output arrives before its partner.
    if(!last_sample_)return baseline;
    if(now<last_sample_ || now-last_sample_>kMaxAgeMs) {
      clearForSafety();return baseline;
    }
    if(state_==State::Trial && changed_at_ &&
       (now<changed_at_ || now-changed_at_>kMaxAgeMs)) {
      // Healthy, recent observations with inconclusive response are not a
      // sensor failure. Revert to the last accepted setting, without
      // misclassifying the exploration direction as good or bad.
      requested_=previous_;pending_=true;state_=State::Restore;clearSamples();
    }
    if(pending_ && state_!=State::Restore && proposal_at_ &&
       (now<proposal_at_ || now-proposal_at_>kProposalAgeMs)) {
      requested_=offset_;pending_=false;
      state_=trial_generation_ && offset_!=active_reference_ ? State::Hold : State::Collect;
      clearSamples();
    }
    const int delta=pending_?requested_:offset_;
    int out=static_cast<int>(baseline)+delta;
    if(out<0)out=0;
    if(out>static_cast<int>(cap))out=cap;
    return static_cast<uint16_t>(out);
  }
  uint16_t current(uint16_t baseline,uint16_t cap)const {
    int out=static_cast<int>(baseline)+offset_;
    if(out<0)out=0;
    if(out>static_cast<int>(cap))out=cap;
    return static_cast<uint16_t>(out);
  }
  // current and actual are AFTER identical guards; acceptance indicates
  // command start accepted, not measured current or completed motor motion.
  void confirm(uint32_t now,uint16_t current_guarded,uint16_t actual,bool accepted) {
    if(!pending_ || !accepted || !now)return;
    if(state_==State::Restore) {
      if((requested_>offset_ && actual<current_guarded) ||
         (requested_<offset_ && actual>current_guarded))return;
      const bool reinstate=previous_active_ && requested_==previous_ && requested_!=0;
      if(reverse_after_restore_){polarity_=-polarity_;reverse_after_restore_=false;}
      offset_=requested_;pending_=false;changed_at_=now;++trial_generation_;pair_pending_=false;
      if(reinstate){active_reference_=previous_active_reference_;state_=State::Hold;
        safety_plus_=previous_safety_plus_;safety_minus_=previous_safety_minus_;}
      else {active_reference_=offset_;state_=State::Collect;}
      clearSamples();clearReference();
      return;
    }
    // A candidate clipped to the same guarded command is no experiment.
    if((requested_>offset_ && actual<=current_guarded) ||
       (requested_<offset_ && actual>=current_guarded)) {
      // A candidate swallowed by an output cap contains no evidence about
      // physical response. If two accepted commands in the same direction
      // were blocked, reacquire a full fresh window before trying the other
      // direction. Preserve the last accepted setting and generation.
      if(actual==current_guarded && ++blocked_>=2) {
        blocked_=0;polarity_=-polarity_;pending_=false;requested_=offset_;fresh_after_=now;
        state_=trialActive()?State::Hold:State::Collect;
        clearSamples();clearReference();
        // Preserve alternating phase but discard any first half collected
        // before the blocked decision; it must not seed the fresh window.
        pair_pending_=false;
      }
      return;
    }
    blocked_=0;
    const int actual_change=static_cast<int>(actual)-static_cast<int>(current_guarded);
    if(actual_change>kStepMs || actual_change< -kStepMs)return;
    previous_active_=trialActive();previous_active_reference_=active_reference_;
    previous_safety_plus_=safety_plus_;previous_safety_minus_=safety_minus_;
    active_reference_=previous_;safety_plus_=ref_plus_;safety_minus_=ref_minus_;
    offset_=requested_;pending_=false;state_=State::Trial;changed_at_=now;++trial_generation_;pair_pending_=false;clearSamples();clearReference();
  }
 private:
  struct Observation { uint32_t start=0,end=0;float left=0,right=0,plus=0,minus=0,sequence=NAN; };
  void clearReference(){reference_count_=reference_next_=0;}
  void referenceSample(uint32_t start,uint32_t now,float l,float r,float p,float m,unsigned window,float sequence) {
    Observation& item=reference_[reference_next_];
    item.start=start;item.end=now;item.left=l;item.right=r;item.plus=p;item.minus=m;item.sequence=sequence;
    reference_next_=(reference_next_+1)%window;
    if(reference_count_<window)++reference_count_;
    n_=reference_count_;
    if(reference_count_<window)return;
    const Observation& oldest=reference_[reference_next_];
    if(now<oldest.start || now-oldest.start>kMaxAgeMs) {
      // Keep only the latest point, so invalid/missing periods are not zero.
      Observation latest=item;clearReference();reference_[0]=latest;
      reference_count_=1;reference_next_=1;n_=1;pending_=false;return;
    }
    float sl=0,sr=0,sp=0,sm=0,ss=0,minp=p,maxp=p,minm=m,maxm=m;unsigned ns=0;
    for(unsigned i=0;i<window;++i){const Observation& x=reference_[i];
      sl+=x.left;sr+=x.right;sp+=x.plus;sm+=x.minus;
      if(std::isfinite(x.sequence)){ss+=x.sequence;++ns;}
      if(x.plus<minp)minp=x.plus;
      if(x.plus>maxp)maxp=x.plus;
      if(x.minus<minm)minm=x.minus;
      if(x.minus>maxm)maxm=x.minus;
    }
    const float avgp=sp/window,avgm=sm/window;
    // Avoid a startup-growth window, without demanding identical RPM.
    if(maxp-minp>avgp*.35f || maxm-minm>avgm*.35f) {pending_=false;return;}
    const float e=(sl-sr)/(sl+sr+.1f*window);error_=e;
    if((sl+sr)/window<.3f || std::fabs(e)<=.10f) {pending_=false;blocked_=0;state_=State::Hold;return;}
    const int sign=e>0.f?1:-1;
    if(decision_sign_ && sign!=decision_sign_)polarity_=-polarity_;
    decision_sign_=sign;
    const int next=offset_+polarity_*kStepMs;
    if(next< -kMaxOffsetMs || next>kMaxOffsetMs){pending_=false;blocked_=0;return;}
    if(!pending_ || next!=requested_)blocked_=0;
    ref_error_=e;right_weak_=sr<=sl;ref_weak_=(right_weak_?sr:sl)/window;
    ref_plus_=avgp;ref_minus_=avgm;ref_sequence_=ns==window?ss/window:NAN;previous_=offset_;requested_=next;
    pending_=true;proposal_at_=now;
  }
  void clearSamples(){n_=0;window_start_=0;left_sum_=right_sum_=plus_sum_=minus_sum_=sequence_sum_=0;sequence_count_=0;early_bad_count_=0;early_bad_end_=0;}
  void clearForSafety(){*this=Controller{};}
  uint32_t last_sample_=0,changed_at_=0,proposal_at_=0,window_start_=0,fresh_after_=0;unsigned n_=0;
  uint32_t trial_generation_=0;
  float left_sum_=0,right_sum_=0,plus_sum_=0,minus_sum_=0,error_=0;
  float ref_error_=0,ref_weak_=0,ref_plus_=0,ref_minus_=0;
  float safety_plus_=0,safety_minus_=0,previous_safety_plus_=0,previous_safety_minus_=0;
  float sequence_sum_=0,ref_sequence_=NAN;unsigned sequence_count_=0;
  int offset_=0,requested_=0,previous_=0,active_reference_=0,polarity_=1,decision_sign_=0;
  int previous_active_reference_=0;
  bool pending_=false,right_weak_=false,previous_active_=false,reverse_after_restore_=false;
  unsigned blocked_=0;
  unsigned early_bad_count_=0;uint32_t early_bad_end_=0;
  unsigned missing_cycles_=0;uint32_t missing_end_=0;
  uint32_t pair_start_=0,pair_end_=0,pair_generation_=0,pair_last_end_=0;
  float pair_left_=0,pair_right_=0,pair_plus_=0,pair_minus_=0;
  bool pair_pending_=false,pair_effective_=false,pair_next_second_=false;
  State state_=State::Collect;
  Observation reference_[kWindow];unsigned reference_count_=0,reference_next_=0;
};
}
