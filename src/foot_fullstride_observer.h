#pragma once
#include <cmath>
#include <cstdint>

// Time-aligned per-foot camera-relative span, not a ground step. The frame
// values are fixed-reference calibrated angles; outside calibration points the
// caller's end-segment extrapolation is an unverified estimate. A body cycle
// does not prove the exact back-to-front interval of either foot.
namespace foot_fullstride_observer {
struct Foot {
  float span=NAN, trimmed_span=NAN, drift=NAN;
  float rise=0.f, fall=0.f; // maximum time-ordered increases / decreases
  float spike_size=0.f; // strongest isolated one-sample reversal
  uint32_t min_ms=0, max_ms=0, first_ms=0, last_ms=0, max_gap_ms=0;
  uint16_t samples=0;
  bool valid=false, low_signal=false, both_directions=false, single_outlier=false;
  bool edge_unverified=false; // extrema at an edge lack a nearby outside sample
};
struct Cycle {
  uint32_t start_ms=0,end_ms=0,sequence=0;
  Foot left,right;
  bool valid=false, startup=false, continuous=false;
};
class Observer {
 public:
  void reset(uint16_t run_id) { *this=Observer{}; run_id_=run_id; }
  // Use exposure timestamp in the same run-relative timebase as body peaks.
  bool frame(uint16_t id,uint32_t ms,bool left_valid,bool right_valid,float left,float right) {
    if(id!=run_id_ || (have_frame_ && ms<=last_frame_ms_))return false;
    have_frame_=true;last_frame_ms_=ms;
    Frame &f=frames_[next_];f.ms=ms;f.l=left;f.r=right;
    f.lv=left_valid && std::isfinite(left);f.rv=right_valid && std::isfinite(right);
    next_=(next_+1)%kFrames;if(count_<kFrames)++count_;
    if(pending_ && ms>=end_ && ms-end_>=kDeliveryAllowanceMs)finish();
    return true;
  }
  // Require an alternating negative-positive-negative set of body peaks.
  // The first completed cycle after reset is always identified as startup,
  // never a steady span. Missing peaks reacquire a complete period.
  bool peak(uint16_t id,int side,uint32_t ms) {
    if(id!=run_id_ || (side!=1 && side!=-1) || (have_peak_ && ms<=last_peak_ms_))return false;
    have_peak_=true;last_peak_ms_=ms;
    if(side>0) { if(have_boundary_ && !have_positive_ && ms>boundary_)have_positive_=true; else {have_boundary_=false;have_positive_=false;} return true; }
    if(pending_)finish();
    if(have_boundary_ && have_positive_ && ms>boundary_) {
      start_=boundary_;end_=ms;pending_=true;
      if(have_frame_ && last_frame_ms_>=end_ && last_frame_ms_-end_>=kDeliveryAllowanceMs)finish();
    }
    boundary_=ms;have_boundary_=true;have_positive_=false;
    return true;
  }
  const Cycle& latest()const{return latest_;}
  uint32_t sequence()const{return latest_.sequence;}
 private:
  static constexpr unsigned kFrames=32;
  static constexpr uint32_t kMaxGapMs=250;
  // A missing adjacent frame weakens isolated-edge verification even if the
  // wider period is otherwise usable; keep that uncertainty separate.
  static constexpr uint32_t kEdgeContextGapMs=150;
  static constexpr uint32_t kDeliveryAllowanceMs=110;
  static constexpr unsigned kMinSamples=5;
  // This is a diagnostic marker, not a measurement support boundary.
  static constexpr float kNoiseDiagnosticDeg=.2f;
  static constexpr float kDirectionDiagnosticDeg=.1f;
  // Diagnostic is deliberately conservative, not a calibrated sensor bound.
  // A fast real movement may still look like an isolated spike.
  static constexpr float kIsolatedSpikeFloorDeg=.75f;
  static constexpr float kIsolatedSpikeMedianFactor=6.f;
  struct Frame {uint32_t ms=0;float l=NAN,r=NAN;bool lv=false,rv=false;};
  struct Accum {
    Foot out;
    float lo=INFINITY,second_lo=INFINITY,hi=-INFINITY,second_hi=-INFINITY;
    float first=0,last=0;
    float outside_before=0.f,outside_after=0.f;
    uint32_t before_ms=0,after_ms=0;
    bool have_before=false,have_after=false;
    float values[kFrames]={};
    void outsideBefore(uint32_t ms,float x){outside_before=x;before_ms=ms;have_before=true;}
    void outsideAfter(uint32_t ms,float x){if(!have_after){outside_after=x;after_ms=ms;have_after=true;}}
    void checkSpike(float before,float after,float threshold){
      if((before>0.f && after<0.f)||(before<0.f && after>0.f)) {
        const float spike=std::fmin(std::fabs(before),std::fabs(after));
        out.spike_size=std::fmax(out.spike_size,spike);
        if(spike>threshold)out.single_outlier=true;
      }
    }
    void add(uint32_t ms,float x) {
      if(!out.samples){out.first_ms=ms;first=x;}
      else {const uint32_t gap=ms-out.last_ms;if(gap>out.max_gap_ms)out.max_gap_ms=gap;
        out.rise=std::fmax(out.rise,x-lo);out.fall=std::fmax(out.fall,hi-x);
      }
      values[out.samples]=x;
      last=x;out.last_ms=ms;++out.samples;
      if(x<lo){second_lo=lo;lo=x;out.min_ms=ms;} else if(x<second_lo)second_lo=x;
      if(x>hi){second_hi=hi;hi=x;out.max_ms=ms;} else if(x>second_hi)second_hi=x;
    }
    Foot done(uint32_t start,uint32_t end) {
      if(out.samples){out.span=hi-lo;out.drift=last-first;out.low_signal=out.span<kNoiseDiagnosticDeg;}
      if(out.samples>=2)out.trimmed_span=std::fmax(0.f,second_hi-second_lo);
      // Diagnostic only: one sharp outlier can also yield both directions;
      // compare against trimmed_span, drift, and adjacent cycles.
      out.both_directions=out.rise>=kDirectionDiagnosticDeg && out.fall>=kDirectionDiagnosticDeg;
      // A single isolated extreme can manufacture both directions. Estimate
      // the ordinary per-frame change by the median of all adjacent changes;
      // flag only a strong reversal on both sides of an interior sample.
      // This neither clips the reported span nor identifies the true cause.
      if(out.samples>=kMinSamples) {
        float diffs[kFrames-1]={};
        const unsigned nd=out.samples-1;
        for(unsigned i=0;i<nd;++i) {
          const float v=std::fabs(values[i+1]-values[i]);
          unsigned j=i;while(j>0 && diffs[j-1]>v){diffs[j]=diffs[j-1];--j;}
          diffs[j]=v;
        }
        const float median=nd%2?diffs[nd/2]:(diffs[nd/2-1]+diffs[nd/2])*.5f;
        const float threshold=std::fmax(kIsolatedSpikeFloorDeg,kIsolatedSpikeMedianFactor*median);
        for(unsigned i=1;i+1<out.samples;++i)
          checkSpike(values[i]-values[i-1],values[i+1]-values[i],threshold);
        // Include the nearest available samples just outside the body period
        // when testing an edge point. They are used only to assess quality;
        // they do not change the reported within-period span.
        const bool first_context=have_before && before_ms<out.first_ms &&
            out.first_ms-before_ms<=kEdgeContextGapMs;
        const bool last_context=have_after && after_ms>out.last_ms &&
            after_ms-out.last_ms<=kEdgeContextGapMs;
        if(first_context)
          checkSpike(values[0]-outside_before,values[1]-values[0],threshold);
        if(last_context)
          checkSpike(values[out.samples-1]-values[out.samples-2],outside_after-values[out.samples-1],threshold);
        const bool first_extreme=out.min_ms==out.first_ms || out.max_ms==out.first_ms;
        const bool last_extreme=out.min_ms==out.last_ms || out.max_ms==out.last_ms;
        out.edge_unverified=(first_extreme && !first_context) || (last_extreme && !last_context);
      }
      out.valid=out.samples>=kMinSamples && out.first_ms>=start && out.first_ms-start<=kMaxGapMs &&
          out.last_ms<=end && end-out.last_ms<=kMaxGapMs && out.max_gap_ms<=kMaxGapMs;
      return out;
    }
  };
  void finish() {
    if(!pending_) return;
    pending_=false;
    Accum a,b;
    for(unsigned i=0;i<count_;++i){const Frame &f=frames_[(next_+kFrames-count_+i)%kFrames];
      if(f.ms<start_) {
        if(f.lv) a.outsideBefore(f.ms,f.l);
        if(f.rv) b.outsideBefore(f.ms,f.r);
      } else if(f.ms>end_) {
        if(f.lv) a.outsideAfter(f.ms,f.l);
        if(f.rv) b.outsideAfter(f.ms,f.r);
      } else {
        if(f.lv) a.add(f.ms,f.l);
        if(f.rv) b.add(f.ms,f.r);
      }
    }
    Cycle out;out.sequence=latest_.sequence+1;out.start_ms=start_;out.end_ms=end_;
    out.left=a.done(start_,end_);out.right=b.done(start_,end_);
    out.startup=!have_complete_cycle_;have_complete_cycle_=true;
    out.valid=!out.startup && out.left.valid && out.right.valid;
    // This flag checks temporal adjacency and sample quality only. It does not
    // mean the spans are statistically stable or represent a ground stride.
    out.continuous=out.valid && latest_.end_ms==start_ && latest_.left.valid && latest_.right.valid;
    latest_=out;
  }
  Frame frames_[kFrames];unsigned next_=0,count_=0;
  uint16_t run_id_=0;uint32_t last_frame_ms_=0,last_peak_ms_=0,boundary_=0,start_=0,end_=0;
  bool have_frame_=false,have_peak_=false,have_boundary_=false,have_positive_=false,pending_=false,have_complete_cycle_=false;
  Cycle latest_;
};
}
