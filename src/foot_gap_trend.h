#pragma once
#include <cmath>
#include <cstdint>

// Historical camera-relative observations only: never ground displacement,
// a powered-response estimate, or a substitute for the latest input context.
// Missing cycles are not observations. Caller must hard-reset on mode/target/
// motor/sensor faults, during trials, and after any confirmed output change.
namespace foot_gap_trend {
class Window {
 public:
  static constexpr uint32_t kMaxWindowMs=6000;
  static constexpr uint32_t kMaxAgeMs=1500;
  static constexpr uint32_t kMaxSkipped=3;
  void reset() {*this=Window{};}
  bool compatible(int ma,float target) const {
    return ma==1200 && std::isfinite(target) && target>0.f &&
        (!have_context_ || (ma==current_ma_ && std::fabs(target-target_)<=.05f));
  }
  // A soft quality miss is a period with known boundaries but unusable camera
  // data. It never contributes a number. An unknown/malformed period must
  // instead be treated as a hard reset by the caller.
  void skip(uint32_t start,uint32_t end) {
    if(!advance(start,end))return;
    if(++consecutive_skips_>kMaxSkipped) reset();
  }
  bool accept(uint32_t start,uint32_t end,float left,float right,int ma,float target) {
    if(!std::isfinite(left)||!std::isfinite(right)||left<0||right<0||left+right<.3f ||
       ma!=1200 || !std::isfinite(target) || target<=0.f) {reset();return false;}
    if(have_context_ && (ma!=current_ma_ || std::fabs(target-target_)>.05f))reset();
    if(!advance(start,end))return false;
    have_context_=true;current_ma_=ma;target_=target;consecutive_skips_=0;
    Sample& x=history_[next_];x.start=start;x.end=end;x.ordinal=ordinal_;
    x.error=(left-right)/(left+right+.1f);
    next_=(next_+1)%4;if(count_<4)++count_;
    return true;
  }
  // 4 accepted periods; at most 3 rejected periods among them; last two
  // must be adjacent and independently agree with the long-term sign. The
  // earliest accepted period's end to latest end must fit within 6 seconds.
  // Latest data must be fresh when an action is proposed.
  int direction(uint32_t now) const {
    if(count_<4)return 0;
    const Sample& first=history_[next_];
    const Sample& latest=history_[(next_+3)%4];
    const Sample& prior=history_[(next_+2)%4];
    if(now<latest.end || now-latest.end>kMaxAgeMs || latest.end<first.start ||
       latest.end-first.start>kMaxWindowMs || latest.ordinal-first.ordinal>3+kMaxSkipped ||
       latest.ordinal!=ordinal_ || latest.ordinal!=prior.ordinal+1 || latest.start!=prior.end)return 0;
    const int sign=latest.error>.1f?1:latest.error<-.1f?-1:0;
    if(!sign || (sign>0?prior.error<=.1f:prior.error>=-.1f))return 0;
    float sum=0;unsigned agree=0;
    for(unsigned i=0;i<4;++i) {sum+=history_[i].error; if(sign*history_[i].error>.1f)++agree;}
    return agree>=3 && sign*sum/4.f>.1f?sign:0;
  }
  unsigned count()const{return count_;}
 private:
  struct Sample {uint32_t start=0,end=0,ordinal=0;float error=0;};
  bool advance(uint32_t start,uint32_t end) {
    if(!start || end<=start || (last_end_ && start!=last_end_)) {reset();return false;}
    last_end_=end;++ordinal_;return true;
  }
  Sample history_[4];unsigned next_=0,count_=0;
  uint32_t last_end_=0,ordinal_=0,consecutive_skips_=0;
  float target_=0;int current_ma_=0;bool have_context_=false;
};
}
