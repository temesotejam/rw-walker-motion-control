#pragma once
#include <cmath>
#include <cstdint>
#include "foot_tracking_config.h"
#include "foot_calibration_table.h"

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
