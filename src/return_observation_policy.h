#pragma once
#include "walking_energy_policy.h"

// v18 trial observer. The IMU evidence and the latest completed camera frame
// have separate lifetimes. Missing markers never become measured foot angles.
// The energy is an effective rate-energy reachability proxy, not identified
// inertia, contact load, moving-foot potential, or measured mechanical work.
namespace return_observation_policy {
enum class Evidence : uint8_t {
  Unavailable=0, ObservedShape=1, RecognitionDropout=2, RangeRejected=3,
  ObservedUnseparated=4
};
struct FootFrame {
  uint16_t run_id=0;
  uint32_t frame_ms=0,ready_ms=0;
  bool timestamp_valid=false,frame_valid=false,zero_ready=false;
  bool left_detected=false,right_detected=false;
  // These must include the separately configured 5 degree range allowance.
  // A missing marker is not itself evidence of a range violation.
  bool left_range_ok=false,right_range_ok=false;
  float left_deg=NAN,right_deg=NAN;
};
struct Input {
  uint16_t run_id=0;
  uint32_t now=0,peak_ms=0,sample_us=0,sample_age_us=0;
  int side=0;
  float peak_deg=NAN,angle_deg=NAN,rate_dps=NAN,target_j=NAN;
  bool active=false;
  FootFrame foot;
};
struct Choice {
  bool selected=false;
  float request_j=0,deceleration_dps2=0,stopping_distance_deg=0;
  Evidence evidence=Evidence::Unavailable;
  uint32_t foot_age_ms=0,deceleration_span_ms=0;
  bool range_veto=false;
};

inline Evidence assessFoot(const Input& in) {
  const auto& f=in.foot;
  if(!in.run_id || f.run_id!=in.run_id || !f.timestamp_valid || !f.frame_valid || !f.zero_ready ||
     !f.frame_ms || !f.ready_ms || f.ready_ms<f.frame_ms || in.now<f.ready_ms ||
     in.now<f.frame_ms || in.now-f.frame_ms>160u)return Evidence::Unavailable;
  // Only an actually detected finite marker can establish a known range
  // rejection. Do not reinterpret an ambiguous marker's invalid range flag.
  if((f.left_detected && std::isfinite(f.left_deg) && !f.left_range_ok) ||
     (f.right_detected && std::isfinite(f.right_deg) && !f.right_range_ok))return Evidence::RangeRejected;
  if((f.left_detected && !std::isfinite(f.left_deg)) ||
     (f.right_detected && !std::isfinite(f.right_deg)))return Evidence::Unavailable;
  if(!f.left_detected || !f.right_detected)return Evidence::RecognitionDropout;
  if(!f.left_range_ok || !f.right_range_ok)return Evidence::Unavailable;
  return std::fabs(f.left_deg-f.right_deg)>=3.f?Evidence::ObservedShape:Evidence::ObservedUnseparated;
}

class Observer {
 public:
  void reset(){*this=Observer{};}
  // Call only when the aid is accepted for output. A gap, stale image or
  // inactive sample must not rearm the same peak after an accepted aid.
  void consume(uint16_t run_id,uint32_t peak_ms) {
    if(!run_id || !peak_ms)return;
    if(run_!=run_id || peak_!=peak_ms){reset();run_=run_id;peak_=peak_ms;}
    consumed_=true;clearMotion();
  }
  Choice observe(const Input& in) {
    Choice out;
    // Missing identifiers cannot undo an accepted aid for the previous key.
    // An explicit reset or a new nonzero run/peak is required to rearm it.
    if(!in.run_id || !in.peak_ms){clearMotion();return out;}
    if(run_!=in.run_id || peak_!=in.peak_ms){reset();run_=in.run_id;peak_=in.peak_ms;}
    out.evidence=assessFoot(in);
    if(out.evidence!=Evidence::Unavailable)out.foot_age_ms=in.now-in.foot.frame_ms;
    if(out.evidence==Evidence::RangeRejected)range_rejected_=true;
    else if(out.evidence==Evidence::ObservedShape || out.evidence==Evidence::ObservedUnseparated)range_rejected_=false;
    out.range_veto=range_rejected_;

    const float distance=in.angle_deg*in.side,speed=-in.rate_dps*in.side;
    if(consumed_ || !in.active || (in.side!=1 && in.side!=-1) ||
       !std::isfinite(distance) || !std::isfinite(speed) || !std::isfinite(in.peak_deg) ||
       !std::isfinite(in.target_j) || in.target_j<=0 || !in.sample_us || in.sample_age_us>10000u ||
       in.now<in.peak_ms || in.now-in.peak_ms<60u || in.now-in.peak_ms>800u ||
       distance<.35f || distance>12.f || in.peak_deg-distance<.5f || speed<3.f || speed>35.f) {
      clearMotion();return out;
    }
    // Unique fresh IMU samples only. Both delivery and acquisition gaps reset
    // the derivative; a camera dropout alone does not reset this history.
    if(!anchor_sample_us_ || in.now<=last_ms_ || in.now-last_ms_>10u ||
       in.sample_us<=last_sample_us_ || in.sample_us-last_sample_us_>10000u) {
      clearMotion();anchor_sample_us_=last_sample_us_=in.sample_us;
      last_ms_=in.now;anchor_speed_=speed;return out;
    }
    last_ms_=in.now;last_sample_us_=in.sample_us;
    if(in.sample_us-anchor_sample_us_>=20000u) {
      const uint32_t span=in.sample_us-anchor_sample_us_;
      const float loss=anchor_speed_-speed;
      acceleration_=loss/(span*.000001f);
      anchor_sample_us_=in.sample_us;anchor_speed_=speed;
      if(acceleration_>=60.f) {
        strong_span_us_+=span;strong_speed_loss_+=loss;
        if(strong_windows_<255)++strong_windows_;
      }else {strong_span_us_=0;strong_speed_loss_=0;strong_windows_=0;}
    }
    out.deceleration_dps2=acceleration_;
    out.deceleration_span_ms=strong_span_us_/1000u;
    const bool dropout=out.evidence==Evidence::RecognitionDropout;
    const bool observed=out.evidence==Evidence::ObservedShape;
    if((!dropout && !observed) || (dropout && range_rejected_) ||
       acceleration_<(dropout?60.f:40.f)) {clearPending();return out;}
    out.stopping_distance_deg=speed*speed/(2.f*acceleration_);
    if(out.stopping_distance_deg+1.f>=distance ||
       (dropout && (strong_windows_<2 || strong_span_us_<40000u || strong_speed_loss_<4.f))) {
      clearPending();return out;
    }
    if(pending_evidence_!=out.evidence || !pending_ms_) {
      pending_evidence_=out.evidence;pending_ms_=in.now;
    }
    const float c=walking_energy_policy::passiveCoefficients(-in.side).omega2_j;
    const float rad=lateral_energy::kRadPerDeg;
    const float requested=c*rad*rad*(2.f*acceleration_*distance-speed*speed);
    out.request_j=std::fmin((dropout?.25f:.5f)*in.target_j,std::fmax(0.f,requested));
    out.selected=out.request_j>0 && in.now-pending_ms_>=(dropout?12u:6u);
    return out;
  }
 private:
  void clearPending(){pending_ms_=0;pending_evidence_=Evidence::Unavailable;}
  void clearMotion() {
    anchor_sample_us_=last_sample_us_=last_ms_=0;anchor_speed_=acceleration_=0;
    strong_span_us_=0;strong_windows_=0;strong_speed_loss_=0;clearPending();
  }
  uint16_t run_=0;
  uint32_t peak_=0,anchor_sample_us_=0,last_sample_us_=0,last_ms_=0,pending_ms_=0,strong_span_us_=0;
  float anchor_speed_=0,acceleration_=0,strong_speed_loss_=0;
  uint8_t strong_windows_=0;
  Evidence pending_evidence_=Evidence::Unavailable;
  bool consumed_=false,range_rejected_=false;
};
}
