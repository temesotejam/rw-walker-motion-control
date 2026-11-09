#pragma once
#include <cmath>
#include <cstdint>

// A bounded experiment on the negative-side cap, not an identified model.
// Motor, speed, timing, wheel and sensor protections remain with the caller.
namespace negative_cap_recovery {
// Do not allow the foot controller to learn from any cycle that overlaps a
// negative-side trial or its most recently accepted intervention.
inline bool normalFootReferenceAllowed(bool was_active,bool now_active,uint32_t start,uint32_t last_applied) {
  return !was_active && !now_active && (!last_applied || start>last_applied);
}
class Controller {
 public:
  static constexpr uint32_t kFreshMs=1500, kWindowMs=12000, kTrialMs=8000;
  static constexpr unsigned kWindow=6, kResponse=6, kHoldResponse=4;
  void reset(){*this=Controller{};}
  bool active()const{return active_;}
  bool attempted()const{return attempted_;}
  bool completed()const{return completed_;}
  bool evaluated()const{return evaluated_;}
  bool improved()const{return improved_;}
  bool holding()const{return holding_;}
  uint8_t stateCode()const{return active_ ? (holding_ ? 2u : 1u) : (completed_ ? 3u : 0u);}
  void interrupt(){abort();clearPeaks();clearFeet();}
  unsigned responses()const{return responses_;}
  // Each qualifying peak must be matched to an actually accepted command.
  // A repeated side, stale pair, invalid value or hardware fault clears the
  // observation history; a hardware fault terminates a trial.
  void peak(int side,uint32_t ms,float angle,bool matched,bool healthy) {
    if(!healthy){abort();clearPeaks();clearFeet();return;}
    if(!ms || (side!=1 && side!=-1) || !matched || !std::isfinite(angle) || angle<=0.f) {
      clearPeaks();return;
    }
    if(last_peak_ && (ms<=last_peak_ || ms-last_peak_>1500 || side==last_side_))clearPeaks();
    if(active_ && (angle>9.5f || (side>0 && ref_plus_>0.f && angle<ref_plus_*.65f) ||
                   (side<0 && ref_floor_>0.f && angle<ref_floor_*.65f))) {
      abort();clearPeaks();return;
    }
    last_peak_=ms;last_side_=static_cast<int8_t>(side);
    if(side>0) {plus_=angle;plus_time_=ms;return;}
    if(!plus_time_ || ms<=plus_time_ || ms-plus_time_>1000) {clearPeaks();return;}
    Peak& p=peaks_[peak_next_];p.ms=ms;p.plus=plus_;p.minus=angle;
    peak_next_=(peak_next_+1)%kWindow;if(peak_count_<kWindow)++peak_count_;
    plus_time_=0;
    if(active_ && command_effective_ && ms>command_ms_ && ms-command_ms_<=1500) {
      command_effective_=false;
      effective_peak_ms_=ms;
      if(responses_<kResponse) response_[responses_++]=angle;
      // The window is only evidence about the observed response. Its actual
      // causal source is not proven, and this is never an integral update.
      if(responses_>=(holding_?kHoldResponse:kResponse))pending_finish_=true;
    }
  }
  // Image misses are skipped, not filled with zeros. Samples must be ordered;
  // callers must pass only completed camera-relative full-cycle observations.
  void footCycle(uint32_t start,uint32_t end,float left,float right,bool valid,bool healthy,bool neutral_context) {
    if(!healthy){abort();clearFeet();return;}
    if(!valid) {if(active_ && pending_finish_ && end==effective_peak_ms_)finish(end);return;}
    if(!start || end<=start || (last_foot_ && end<=last_foot_) ||
       !std::isfinite(left) || !std::isfinite(right) || left<0.f || right<0.f) {clearFeet();return;}
    // Compare multi-cycle means; a single small observation is expected in
    // alternating gait and is not sufficient evidence to stop the trial.
    if(active_ && start>started_ && end==effective_peak_ms_ && foot_response_count_<4) {
      foot_response_left_+=left;foot_response_right_+=right;
      if(++foot_response_count_>=4) {
        const float mean=foot_response_right_/static_cast<float>(foot_response_count_);
        if(ref_right_>0.f && mean<ref_right_*.5f && mean<ref_right_-.1f)abort();
      }
    }
    Foot& f=feet_[foot_next_];f.start=start;f.end=end;f.left=left;f.right=right;f.neutral=neutral_context;
    foot_next_=(foot_next_+1)%kWindow;if(foot_count_<kWindow)++foot_count_;last_foot_=end;
    if(active_ && pending_finish_ && end==effective_peak_ms_)finish(end);
  }
  bool candidate(uint32_t now,float target,uint16_t cap,bool foot_idle,bool healthy) const {
    if(active_ || attempted_ || !healthy || !foot_idle || target!=8.f || cap<20 || cap>35 ||
       peak_count_<kWindow || foot_count_<kWindow || !last_peak_ || !last_foot_ ||
       now<last_peak_ || now-last_peak_>kFreshMs || now<last_foot_ || now-last_foot_>kFreshMs) return false;
    const Peak& first=peaks_[peak_next_];const Foot& first_foot=feet_[foot_next_];
    if(now<first.ms || now-first.ms>kWindowMs || now<first_foot.start || now-first_foot.start>kWindowMs)return false;
    const Foot& last=feet_[(foot_next_+kWindow-1)%kWindow];
    const Foot& prior=feet_[(foot_next_+kWindow-2)%kWindow];
    const Peak& last_peak=peaks_[(peak_next_+kWindow-1)%kWindow];
    const Peak& prior_peak=peaks_[(peak_next_+kWindow-2)%kWindow];
    if(!last.neutral || !prior.neutral || last.start!=prior.end ||
       last.end!=last_peak.ms || prior.end!=prior_peak.ms)return false;
    // The newest complete alternating pair must still exhibit the imbalance;
    // old weak periods must not trigger a probe after the gait has recovered.
    // Sum the pair, rather than requiring both halves to be alike.
    const float pair_left=last.left+prior.left,pair_right=last.right+prior.right;
    if(pair_left<2.f || pair_right>=pair_left*.65f || pair_left-pair_right<1.f)return false;
    unsigned weak_peaks=0,weak_feet=0;
    for(unsigned i=0;i<kWindow;++i) {
      if(peaks_[i].plus>9.5f || peaks_[i].minus>9.5f)return false;
      if(peaks_[i].minus<6.5f)++weak_peaks;
      if(feet_[i].left>=1.f && feet_[i].right<feet_[i].left*.65f &&
         feet_[i].left-feet_[i].right>=.5f)++weak_feet;
    }
    return weak_peaks>=5 && weak_feet>=4;
  }
  // Invoke on every prospective negative input. Once timed out, no later
  // call in the same run may revive the experiment.
  uint16_t extraCap(uint32_t now,bool healthy) {
    const uint32_t deadline_start=holding_?hold_started_:started_;
    if(!healthy || (active_ && (!now || now<deadline_start || now-deadline_start>kTrialMs ||
       !last_foot_ || now<last_foot_ || now-last_foot_>2500 ||
       !last_peak_ || now<last_peak_ || now-last_peak_>kFreshMs)))abort();
    return active_ && !pending_finish_ ? 5u : 0u;
  }
  // The candidate may start only when the caller evaluated candidate() just
  // before this command. baseline and actual are both post wheel-guard.
  void command(uint32_t now,uint16_t baseline,uint16_t actual,bool accepted,bool candidate_start) {
    if(!now || !accepted || actual<=baseline || actual-baseline>5) {command_effective_=false;return;}
    if(!active_) {
      if(!candidate_start || attempted_)return;
      attempted_=true;active_=true;started_=now;responses_=0;
      const Peak& p=peaks_[(peak_next_+kWindow-1)%kWindow];
      const Peak& q=peaks_[(peak_next_+kWindow-2)%kWindow];
      const Foot& f=feet_[(foot_next_+kWindow-1)%kWindow];
      const Foot& g=feet_[(foot_next_+kWindow-2)%kWindow];
      ref_floor_=ref_pair_floor_=std::fmin(p.minus,q.minus);
      ref_plus_=(p.plus+q.plus)*.5f;
      ref_right_=(f.right+g.right)*.5f;
      const float l=(f.left+g.left)*.5f;
      ref_foot_error_=std::fabs((l-ref_right_)/(l+ref_right_+.1f));
    }
    command_ms_=now;command_effective_=true;
  }
 private:
  struct Peak {uint32_t ms=0;float plus=0.f,minus=0.f;};
  struct Foot {uint32_t start=0,end=0;float left=0.f,right=0.f;bool neutral=false;};
  void finish(uint32_t now){
    const float r1=std::fmin(response_[0],response_[1]);
    const float r2=std::fmin(response_[2],response_[3]);
    const float floor=holding_ ? (r1+r2)*.5f :
      (r1+r2+std::fmin(response_[4],response_[5]))/3.f;
    evaluated_=foot_response_count_>=4;
    improved_=false;
    if(evaluated_){
      const float l=foot_response_left_/4.f,r=foot_response_right_/4.f;
      const float err=std::fabs((l-r)/(l+r+.1f));
      // A smaller strong foot alone can improve the ratio while worsening
      // walking. Require an observable absolute gain in the weaker foot.
      improved_=floor>=ref_pair_floor_*1.05f && floor>=ref_pair_floor_+.15f &&
        r>=ref_right_*1.05f && r>=ref_right_+.03f && err+.02f<ref_foot_error_;
    }
    pending_finish_=false;
    if(improved_ && now && (!holding_ || (now>=hold_started_ && now-hold_started_<=kTrialMs))) {
      if(!holding_){holding_=true;hold_started_=now;}
      responses_=0;foot_response_left_=foot_response_right_=0.f;foot_response_count_=0;
    } else {active_=false;completed_=true;holding_=false;}
  }
  void clearPeaks(){peak_count_=peak_next_=0;last_peak_=plus_time_=0;last_side_=0;}
  void clearFeet(){foot_count_=foot_next_=0;last_foot_=0;}
  void abort(){if(active_){completed_=true;evaluated_=false;improved_=false;}
    active_=false;holding_=false;command_effective_=false;pending_finish_=false;}
  Peak peaks_[kWindow];Foot feet_[kWindow];float response_[kResponse]={};
  uint32_t last_peak_=0,plus_time_=0,last_foot_=0,started_=0,hold_started_=0,command_ms_=0,effective_peak_ms_=0;
  unsigned peak_count_=0,peak_next_=0,foot_count_=0,foot_next_=0,responses_=0;
  float plus_=0.f,ref_floor_=0.f,ref_plus_=0.f,ref_right_=0.f,ref_pair_floor_=0.f,ref_foot_error_=0.f;
  float foot_response_left_=0.f,foot_response_right_=0.f;unsigned foot_response_count_=0;
  int8_t last_side_=0;bool attempted_=false,active_=false,holding_=false,completed_=false,evaluated_=false,improved_=false,command_effective_=false,pending_finish_=false;
};
}
