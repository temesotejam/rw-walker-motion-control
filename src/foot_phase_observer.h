#pragma once
#include <cmath>
#include <cstdint>
#include "foot_tracking_config.h"
#include "foot_calibration.h"

// Calibrated camera-relative angles, NOT ground step distance. Positive body
// peak is paired with the left foot and negative with the right. This is the
// phase relation observed in these logs, not a proof of physical axis sign.
// All ms arguments are camera-frame/peak times relative to THE SAME run start;
// do not pass a frame delivery time or a sync-start-relative log timestamp.
namespace foot_phase_observer {
class Observer {
 public:
  void reset(uint16_t id) { *this = Observer{}; run_id_=id; }

  // Only the phase controller needs strict calibration support. Keep raw
  // detections and extrapolated diagnostic values available to other users.
  bool calibratedFrame(uint16_t id, uint32_t ms, bool lv, bool rv,
                       bool left_in_range, bool right_in_range, float left, float right) {
    return frame(id,ms,lv && left_in_range,rv && right_in_range,left,right);
  }

  // Phase-only fixed-reference angle. Unlike UI foot angles, this never uses
  // the per-boot zero, so a late zero lock cannot step the control signal.
  // Use endpoint extrapolation over the full detected image. Values outside
  // calibration measurements are estimates, not verified angle measurements.
  static bool boundedMarker(bool detected, float x, float minimum, float maximum) {
    (void)minimum; (void)maximum;
    return detected && std::isfinite(x);
  }
  static float fixedAngle(const foot_calibration::Point* points,float x,float fixed_x) {
    return foot_calibration::angleAt(points,x)-foot_calibration::angleAt(points,fixed_x);
  }
  bool rawFrame(uint16_t id,uint32_t ms,bool left_detected,bool right_detected,
                float left_x,float right_x) {
    const bool lv=boundedMarker(left_detected,left_x,appcfg::kFootAngleBMinCalXPx,appcfg::kFootAngleBMaxCalXPx);
    const bool rv=boundedMarker(right_detected,right_x,appcfg::kFootAngleAMinCalXPx,appcfg::kFootAngleAMaxCalXPx);
    const float left=lv ? fixedAngle(foot_calibration::left,left_x,appcfg::kFootAngleBZeroXPx) : NAN;
    const float right=rv ? fixedAngle(foot_calibration::right,right_x,appcfg::kFootAngleAZeroXPx) : NAN;
    return frame(id,ms,
        lv,rv,left,right);
  }

  // Camera frame timestamps must be strictly increasing. Their precise
  // relationship to sensor exposure is not independently validated.
  // Missing/invalid frames
  // advance the watermark but cannot become a peak-matched measurement.
  __attribute__((noinline)) bool frame(uint16_t id, uint32_t ms, bool lv, bool rv, float left, float right) {
    if (id!=run_id_ || (have_frame_ && ms<=last_frame_ms_)) return false;
    have_frame_=true; last_frame_ms_=ms;
    Frame f;f.ms=ms;f.left=left;f.right=right;
    f.valid=lv && rv && std::isfinite(left) && std::isfinite(right);
    frames_[frame_next_]=f; frame_next_=(frame_next_+1)%kFrames;
    if(frame_count_<kFrames) ++frame_count_;
    if(pending_) {
      consider(f);
      if(ms>pending_ms_ && ms-pending_ms_>kWindowMs) finishPeak();
    }
    return true;
  }

  // A peak can be delivered after its timestamp: search the frame ring
  // both before and after the peak; select the nearest valid frame. If an
  // already received frame establishes the end of the window, finish now.
  __attribute__((noinline)) bool peak(uint16_t id, int side, uint32_t ms) {
    if(id!=run_id_ || (side!=1 && side!=-1) || (have_peak_ && ms<=last_peak_ms_)) return false;
    if(pending_) finishPeak();
    have_peak_=true;last_peak_ms_=ms;pending_=true;pending_side_=side;pending_ms_=ms;candidate_=false;
    for(unsigned i=0;i<frame_count_;++i)
      consider(frames_[(frame_next_+kFrames-frame_count_+i)%kFrames]);
    if(have_frame_ && last_frame_ms_>ms && last_frame_ms_-ms>kWindowMs) finishPeak();
    return true;
  }

  // At least two consecutive complete cycles with bounded dispersion and a
  // recent end point are required before feedback is eligible. A stale or
  // missing peak cannot silently reuse the previous pair.
  __attribute__((noinline)) bool valid(uint32_t now_ms, uint32_t max_age_ms=2000, float max_spread_deg=2.f) const {
    if(cycle_count_<2 || now_ms<cycle_ms_ || now_ms-cycle_ms_>max_age_ms ||
       (pending_ && now_ms>=pending_ms_ && now_ms-pending_ms_>500) ||
       !std::isfinite(max_spread_deg) || max_spread_deg<0.f) return false;
    float lo_l=left_[0],hi_l=left_[0],lo_r=right_[0],hi_r=right_[0];
    for(unsigned i=1;i<cycle_count_;++i) {
      lo_l=std::fmin(lo_l,left_[i]);hi_l=std::fmax(hi_l,left_[i]);
      lo_r=std::fmin(lo_r,right_[i]);hi_r=std::fmax(hi_r,right_[i]);
    }
    return hi_l-lo_l<=max_spread_deg && hi_r-lo_r<=max_spread_deg;
  }
  float leftMean() const { return mean(left_); }
  float rightMean() const { return mean(right_); }
  // Single most recently completed cycle; use these with cycleStart()/Time()
  // for post-intervention evaluation so old rolling-window cycles do not leak
  // into an allegedly new response.
  float leftLatest() const {return cycle_count_?left_[(cycle_next_+2)%3]:NAN;}
  float rightLatest() const {return cycle_count_?right_[(cycle_next_+2)%3]:NAN;}
  unsigned count() const { return cycle_count_; }
  uint32_t cycleStart() const { return cycle_start_ms_; }
  uint32_t cycleTime() const { return cycle_ms_; }
  uint32_t sequence() const { return cycle_seq_; }

 private:
  static constexpr unsigned kFrames=24;
  static constexpr uint32_t kWindowMs=100;
  struct Frame { uint32_t ms=0; float left=NAN,right=NAN; bool valid=false; };
  struct Peak { uint32_t ms=0; float left=NAN,right=NAN; };
  void consider(const Frame& f) {
    if(!pending_ || !f.valid) return;
    const uint32_t delta=f.ms>=pending_ms_? f.ms-pending_ms_:pending_ms_-f.ms;
    if(delta>kWindowMs) return;
    if(!candidate_ || delta<candidate_delta_ || (delta==candidate_delta_ && f.ms<candidate_frame_.ms)) {
      candidate_=true;candidate_delta_=delta;candidate_frame_=f;
    }
  }
  void clearCycle() { have_negative_=false;have_positive_=false;cycle_count_=0;cycle_next_=0;cycle_start_ms_=0;cycle_ms_=0; }
  __attribute__((noinline)) void finishPeak() {
    if(!pending_) return;
    pending_=false;
    if(!candidate_) {clearCycle();return;}
    // A single camera frame is not independent evidence for two different
    // body peaks, even when their +/-100ms windows overlap.
    if(have_matched_frame_ && candidate_frame_.ms==matched_frame_ms_) {
      clearCycle();return;
    }
    matched_frame_ms_=candidate_frame_.ms;have_matched_frame_=true;
    Peak p;p.ms=pending_ms_;p.left=candidate_frame_.left;p.right=candidate_frame_.right;
    if(pending_side_>0) {
      if(!have_negative_ || have_positive_ || p.ms<=negative_.ms || p.ms-negative_.ms>1500) {clearCycle();return;}
      positive_=p;have_positive_=true;return;
    }
    if(!have_negative_) {negative_=p;have_negative_=true;return;}
    if(!have_positive_ || p.ms<=positive_.ms || p.ms-positive_.ms>1500 ||
       p.ms<=negative_.ms || p.ms-negative_.ms<250 || p.ms-negative_.ms>2000) {
      clearCycle();negative_=p;have_negative_=true;return;
    }
    // Average the same-foot negative endpoints to reduce cycle drift.
    left_[cycle_next_]=positive_.left-(negative_.left+p.left)*.5f;
    right_[cycle_next_]=(negative_.right+p.right)*.5f-positive_.right;
    cycle_next_=(cycle_next_+1)%3;if(cycle_count_<3)++cycle_count_;
    cycle_start_ms_=negative_.ms;cycle_ms_=p.ms;++cycle_seq_;
    negative_=p;have_negative_=true;have_positive_=false;
  }
  float mean(const float* values) const {
    if(!cycle_count_)return NAN;
    float sum=0;for(unsigned i=0;i<cycle_count_;++i)sum+=values[i];
    return sum/static_cast<float>(cycle_count_);
  }
  uint16_t run_id_=0;
  Frame frames_[kFrames],candidate_frame_;
  unsigned frame_next_=0,frame_count_=0;
  uint32_t last_frame_ms_=0,last_peak_ms_=0,pending_ms_=0,candidate_delta_=0,matched_frame_ms_=0;
  bool have_frame_=false,have_peak_=false,pending_=false,candidate_=false,have_matched_frame_=false;
  int pending_side_=0;
  Peak negative_,positive_;
  bool have_negative_=false,have_positive_=false;
  float left_[3]={},right_[3]={};
  unsigned cycle_next_=0,cycle_count_=0;
  uint32_t cycle_start_ms_=0,cycle_ms_=0,cycle_seq_=0;
};
}

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
