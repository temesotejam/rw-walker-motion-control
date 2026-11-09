#pragma once
#include <cmath>
#include <cstdint>

// Four consecutive complete camera-relative foot cycles, approximately
// eight alternating body peaks. The signal is not a ground step measurement.
#ifndef FOOT_TREND_CYCLES
#define FOOT_TREND_CYCLES 4
#endif
namespace foot_trend {
struct Average {
  float left=NAN, right=NAN, negative_peak=NAN, error=NAN;
  uint32_t start_ms=0, end_ms=0;
};
template <unsigned N>
class WindowT {
 public:
  static constexpr unsigned kCycles=N;
  static_assert(kCycles>=2, "foot trend needs at least two cycles");
  void reset() { *this=WindowT{}; }
  // A bad or incomplete cycle discards history; a non-contiguous valid
  // cycle starts a new window with itself. end<=start is never usable.
  bool cycle(uint32_t start,uint32_t end,float left,float right,float peak,bool valid) {
    if(!valid || end<=start || !std::isfinite(left) || !std::isfinite(right) ||
       !std::isfinite(peak) || left<0.f || right<0.f || peak<=0.f ||
       left+right<.3f || !std::isfinite(left+right)) {
      reset();return false;
    }
    if(count_ && start!=last_end_)reset();
    Entry& e=entries_[next_];e.start=start;e.end=end;e.left=left;e.right=right;e.peak=peak;
    e.error=(left-right)/(left+right+.1f);
    next_=(next_+1)%kCycles;if(count_<kCycles)++count_;
    last_end_=end;return true;
  }
  unsigned count()const{return count_;}
  bool ready()const{return count_==kCycles;}
  // The mean of per-cycle normalized errors avoids treating a naturally
  // larger cycle as multiple observations. Call only when ready().
  Average average()const {
    Average out;if(!ready())return out;
    float l=0.f,r=0.f,p=0.f,e=0.f;
    for(unsigned i=0;i<kCycles;++i) {l+=entries_[i].left/kCycles;r+=entries_[i].right/kCycles;
      p+=entries_[i].peak/kCycles;e+=entries_[i].error/kCycles;}
    out.left=l;out.right=r;out.negative_peak=p;out.error=e;
    out.start_ms=entries_[next_].start;out.end_ms=last_end_;return out;
  }
 private:
  struct Entry {uint32_t start=0,end=0;float left=0,right=0,peak=0,error=0;};
  Entry entries_[kCycles];unsigned next_=0,count_=0;uint32_t last_end_=0;
};
using Window=WindowT<FOOT_TREND_CYCLES>;
}
