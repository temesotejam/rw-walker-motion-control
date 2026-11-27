#pragma once

// 2. Coast to the next support: measured body rate and deceleration -> stopping distance.
// Foot observations qualify the relevant sole shape during the approach.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "walking/contact_geometry.h"
#if SUPPORT_TRANSFER_STALL_V3
#include "periodic_foot_state.h"
#endif

namespace coast_motion {
// Positive speed and deceleration magnitudes; angles in degrees, time in seconds.
inline float stoppingDistanceDeg(float speed_dps, float deceleration_dps2) {
  return speed_dps * speed_dps / (2.f * deceleration_dps2);
}
} // namespace coast_motion

namespace walking_energy_policy {
constexpr uint16_t kMaxPulseMs=80;
constexpr float kCoastFallTauS=.070f; // Provisional signed residual audit only.
struct PassiveCoefficients {float offset_j,omega2_j,rate_min_dps,rate_max_dps;};
inline PassiveCoefficients passiveCoefficients(int8_t side){
  // Train: no-new-pulse coast first chronological next-peak transitions in
  // 137267573/267557373/141282685/611622951. Held out:97855607/241508524.
  // mJ coefficients converted to J. Positive intercept is an empirical
  // arrival reservoir/coupling term, not kinetic energy at zero speed.
  return side>0?PassiveCoefficients{.0001630156686f,.0003345881499f,22.2512f,37.5379f}:
      PassiveCoefficients{.0002624748594f,.0002164565952f,20.313f,35.5657f};
}
} // namespace walking_energy_policy

// Confirm loss of *inward* body speed using only samples already received.
// This is neither a contact detector nor a decision to apply motor current.
// Intended for the runner's 5 ms candidate check; fixed storage, no allocation.
namespace foot_support_deceleration {
class Tracker {
 public:
  void reset(){*this=Tracker{};}
  bool update(uint16_t run,uint32_t peak,uint32_t now,float inward_speed){
    if(run!=run_ || peak!=peak_){reset();run_=run;peak_=peak;}
    if(!run || !peak || !std::isfinite(inward_speed) || inward_speed<=0.f){reset();return false;}
    if(have_time_ && now<=last_ms_){reset();return false;}
    if(have_time_ && now-last_ms_>15u){
      // The current valid sample can seed a new observation window, but no
      // speed maximum or confirmation survives an interruption.
      reset();run_=run;peak_=peak;
    }
    have_time_=true;last_ms_=now;
    raw_[raw_next_]=inward_speed;raw_next_=(raw_next_+1u)%3u;
    if(raw_count_<3u)++raw_count_;
    if(raw_count_<3u)return false;
    const float filtered=median(raw_[0],raw_[1],raw_[2]);
    history_[next_].ms=now;history_[next_].speed=filtered;
    next_=(next_+1u)%capacity;
    if(count_<capacity)++count_;
    float highest=filtered;
    for(unsigned i=0;i<count_;++i){
      // The window expires from actual sample times, never sample count alone.
      if(now-history_[i].ms<=100u && history_[i].speed>highest)highest=history_[i].speed;
    }
    if(highest-filtered<4.f){pending_=false;pending_count_=0;return false;}
    if(!pending_){pending_=true;pending_since_ms_=now;pending_count_=1;}
    else if(pending_count_<3u)++pending_count_;
    return pending_count_>=3u && now-pending_since_ms_>=10u;
  }

 private:
  struct Sample {uint32_t ms=0;float speed=0;};
  static constexpr unsigned capacity=24;
  Sample history_[capacity]{};
  float raw_[3]={};
  unsigned next_=0,count_=0,raw_next_=0,raw_count_=0,pending_count_=0;
  uint16_t run_=0;uint32_t peak_=0,last_ms_=0,pending_since_ms_=0;
  bool have_time_=false,pending_=false;
  static float median(float a,float b,float c){
    if(a>b){const float x=a;a=b;b=x;}
    if(b>c){const float x=b;b=c;c=x;}
    if(a>b)b=a;
    return b;
  }
};
}

// Positive-side return that is almost stopped before reaching the centre.
// Observational heuristic: not proof that the next pulse restores motion.
namespace stall_rescue {
constexpr uint16_t kMaxPulseMs=15;
class Detector {
 public:
  void reset() { pending_ms_=peak_ms_=0; samples_=0; }
  bool observe(uint32_t now,uint32_t prior_ms,uint32_t peak_ms,
      float peak_deg,float prior_angle,float angle,float prior_rate,float rate,
      float trigger_speed=6.f,float min_angle=.35f,float max_angle=3.25f) {
    const bool finite=std::isfinite(peak_deg) && std::isfinite(prior_angle) &&
      std::isfinite(angle) && std::isfinite(prior_rate) && std::isfinite(rate);
    if(!finite || !peak_ms || now<=prior_ms || now-prior_ms>10 || now<peak_ms ||
       now-peak_ms<200 || now-peak_ms>800 || peak_deg<5.f || angle<min_angle ||
       angle>max_angle || peak_deg-angle<2.f || prior_angle+.03f<angle ||
       rate < -(trigger_speed+.5f) || rate>=-.1f || (peak_ms_ && peak_ms_!=peak_ms)) {
      reset();return false;
    }
    if(!pending_ms_) {
      if(prior_rate>=-trigger_speed || rate< -trigger_speed)return false;
      pending_ms_=now;peak_ms_=peak_ms;samples_=1;return false;
    }
    if(now-pending_ms_>30) {reset();return false;}
    ++samples_;
    if(samples_>=3 && now-pending_ms_>=6) {reset();return true;}
    return false;
  }
 private:
  uint32_t pending_ms_=0,peak_ms_=0;
  unsigned samples_=0;
};
}

// Trial policy only: camera-relative forward swing is a shape-change proxy,
// not a measured contact, calibrated CAD root, energy deficit or torque.
// No actuator calls or state consumption occur in this portable selector.
namespace foot_support_assist {
constexpr uint32_t kMaxFrameAgeMs=160;
constexpr uint32_t kMaxPeakAgeMs=1500;
constexpr uint16_t kMaxPulseMs=60;
constexpr uint16_t kCoastAssistPulseMs=15;
constexpr float kMinForwardDeg=1.f;
constexpr float kMaxUncertaintyDeg=.75f;
constexpr float kMinBodyDistanceDeg=.8f;
constexpr float kMinInwardSpeedDps=6.f,kMaxInwardSpeedDps=45.f;
constexpr float kMinLeadMarginMs=20.f,kMaxEstimatedLeadMs=250.f;

enum class Reason : uint8_t {
  Selected,Inactive,Identity,Time,PoseUnavailable,PoseUncertain,StaleFrame,
  Nonfinite,SmallForward,WrongMotion,OutsideWindow,Speed,NoOrdinaryDrive,
  Width,InsufficientLead,TooEarly,CoastNeedsEvidence,
  CoastNeedsDeceleration=CoastNeedsEvidence // compatibility: numeric code 16
};
struct Input {
  uint16_t run_id=0,foot_run_id=0;
  // peak_id is the latest accepted peak's run-relative timestamp, not a count.
  uint32_t peak_id=0,now_ms=0,frame_ms=0;
  int8_t peak_side=0;
  bool active=false,pose_ready=false,pose_confident=false;
  bool normal_drive_selected=false,normal_coast_selected=false;
  // A separate causal body-rate detector owns this evidence. A single slow
  // sample, predicted foot phase or large previous peak is not deceleration.
  bool inward_decelerating=false;
  // Caller-validated shape trial: dominant lower-margin foot Returning,
  // forward and asymmetry lower margins >=1deg, incoming body peak <=8.5deg.
  // This anticipates support transfer while foot shape changes; it is NOT
  // measured future deceleration, contact, or a body-peak walking criterion.
  bool anticipated_shape_need=false;
  float left_deg=NAN,right_deg=NAN;
  // Internal uncertainty estimates, not validated physical error bounds.
  float left_uncertainty_deg=NAN,right_uncertainty_deg=NAN;
  float body_angle_deg=NAN,body_rate_dps=NAN;
  uint16_t normal_width_ms=0;
};
struct Decision {
  bool selected=false;
  bool coast_assist=false;
  Reason reason=Reason::Inactive;
  uint16_t width_ms=0;
  uint32_t frame_age_ms=0;
  float forward_deg=NAN,asymmetry_deg=NAN,threshold_deg=NAN;
  float body_distance_deg=NAN,inward_speed_dps=NAN,estimated_zero_ms=NAN;
};
inline Decision decide(const Input& in) {
  Decision out;
  auto reject=[&](Reason r) {out.reason=r;return out;};
  if(!in.active)return reject(Reason::Inactive);
  if(!in.run_id || in.foot_run_id!=in.run_id || !in.peak_id ||
      (in.peak_side!=1 && in.peak_side!=-1))return reject(Reason::Identity);
  if(!in.frame_ms || in.frame_ms>in.now_ms || in.peak_id>=in.now_ms ||
      in.now_ms-in.peak_id>kMaxPeakAgeMs)return reject(Reason::Time);
  out.frame_age_ms=in.now_ms-in.frame_ms;
  if(out.frame_age_ms>kMaxFrameAgeMs)return reject(Reason::StaleFrame);
  if(!in.pose_ready)return reject(Reason::PoseUnavailable);
  if(!in.pose_confident)return reject(Reason::PoseUncertain);
  if(!std::isfinite(in.left_deg)||!std::isfinite(in.right_deg)||
      !std::isfinite(in.left_uncertainty_deg)||!std::isfinite(in.right_uncertainty_deg)||
      !std::isfinite(in.body_angle_deg)||!std::isfinite(in.body_rate_dps))
    return reject(Reason::Nonfinite);
  if(in.left_uncertainty_deg<0 || in.right_uncertainty_deg<0 ||
      in.left_uncertainty_deg>kMaxUncertaintyDeg || in.right_uncertainty_deg>kMaxUncertaintyDeg)
    return reject(Reason::PoseUncertain);
  // Subtract estimated uncertainty instead of rewarding an uncertain large
  // prediction. Foot-axis polarity is user-confirmed: forward is positive.
  out.forward_deg=std::fmax(0.f,std::fmax(in.left_deg-in.left_uncertainty_deg,
                                       in.right_deg-in.right_uncertainty_deg));
  out.asymmetry_deg=std::fmax(0.f,std::fabs(in.left_deg-in.right_deg)-
      in.left_uncertainty_deg-in.right_uncertainty_deg);
  if(out.forward_deg<kMinForwardDeg)return reject(Reason::SmallForward);
  // Bounded experimental timing map; these coefficients are not CAD fits.
  out.threshold_deg=std::fmax(1.5f,std::fmin(5.f,
      1.f+1.2f*out.forward_deg+.3f*out.asymmetry_deg));
  out.body_distance_deg=in.body_angle_deg*in.peak_side;
  out.inward_speed_dps=-in.body_rate_dps*in.peak_side;
  if(out.body_distance_deg<=0 || out.inward_speed_dps<=0)return reject(Reason::WrongMotion);
  if(out.body_distance_deg<kMinBodyDistanceDeg || out.body_distance_deg>out.threshold_deg)
    return reject(Reason::OutsideWindow);
  if(out.inward_speed_dps<kMinInwardSpeedDps || out.inward_speed_dps>kMaxInwardSpeedDps)
    return reject(Reason::Speed);
  if(in.normal_drive_selected==in.normal_coast_selected)return reject(Reason::NoOrdinaryDrive);
  const bool coast=in.normal_coast_selected;
  if(coast && !in.inward_decelerating && !in.anticipated_shape_need)
    return reject(Reason::CoastNeedsEvidence);
  if(coast && in.normal_width_ms)return reject(Reason::Width);
  const uint16_t requested=coast?kCoastAssistPulseMs:in.normal_width_ms;
  // Preserve the ordinary proposal at this decision state. Never truncate a
  // wider ordinary pulse into a weaker early pulse to obtain eligibility.
  if(!requested || requested>kMaxPulseMs)return reject(Reason::Width);
  out.estimated_zero_ms=1000.f*out.body_distance_deg/out.inward_speed_dps;
  if(out.estimated_zero_ms<requested+kMinLeadMarginMs)
    return reject(Reason::InsufficientLead);
  if(out.estimated_zero_ms>kMaxEstimatedLeadMs)return reject(Reason::TooEarly);
  out.selected=true;out.coast_assist=coast;out.reason=Reason::Selected;out.width_ms=requested;
  return out;
}

// Recheck after the actual tilt/wheel/timing guards. A smaller guarded width
// declines early intervention; ordinary ZERO keeps its existing opportunity.
// The runner must also recheck current identity, phase, clocks and hardware.
inline bool widthPreserved(const Decision& d,uint16_t guarded_width_ms) {
  return d.selected && d.width_ms>0 && d.width_ms<=kMaxPulseMs &&
      guarded_width_ms==d.width_ms;
}
}

// Separate v12 trial. Reuses the v11 input vocabulary without changing its
// selector or firmware behavior. This is not a contact/torque safety proof.
namespace foot_support_assist_v12 {
constexpr uint32_t kMaxFrameAgeMs=foot_support_assist::kMaxFrameAgeMs;
constexpr uint32_t kMaxPeakAgeMs=foot_support_assist::kMaxPeakAgeMs;
constexpr uint16_t kMaxPulseMs=80;
constexpr uint16_t kCoastAssistPulseMs=foot_support_assist::kCoastAssistPulseMs;
constexpr float kMinForwardDeg=foot_support_assist::kMinForwardDeg;
constexpr float kMaxUncertaintyDeg=foot_support_assist::kMaxUncertaintyDeg;
constexpr float kMinBodyDistanceDeg=foot_support_assist::kMinBodyDistanceDeg;
constexpr float kMinInwardSpeedDps=foot_support_assist::kMinInwardSpeedDps;
constexpr float kMaxInwardSpeedDps=foot_support_assist::kMaxInwardSpeedDps;
constexpr float kMinLeadMarginMs=foot_support_assist::kMinLeadMarginMs;
constexpr float kMaxEstimatedLeadMs=foot_support_assist::kMaxEstimatedLeadMs;
constexpr float kMaxStallEstimatedLeadMs=500.f;

enum class PredictionSource : uint8_t { Unknown,History,Local };
enum class Mode : uint8_t { None,OrdinaryRelocation,CoastAssist,StallOrdinaryRelocation,StallCoastAssist };
enum class Reason : uint8_t {
  Selected,Inactive,Identity,Time,PoseUnavailable,PoseUncertain,StaleFrame,
  Nonfinite,SmallForward,WrongMotion,OutsideWindow,Speed,NoOrdinaryDrive,
  Width,InsufficientLead,TooEarly,CoastNeedsEvidence,StallNeedsEvidence,StallTooEarly,
  CoastNeedsDeceleration=CoastNeedsEvidence
};
struct Input : foot_support_assist::Input {
  // Causal local observations must confirm the dominant lower-margin foot is
  // Returning. A historical forecast or a single falling/noisy frame is not
  // sufficient for the >250ms near-stall route.
  bool local_returning_confirmed=false;
  PredictionSource prediction_source=PredictionSource::Unknown;
};
// Can only reduce the compiled envelope, for controlled ablation/replay.
// Root integration uses defaults. A higher supplied width cannot exceed 80ms.
struct Limits {
  uint16_t max_ordinary_width_ms=80;
  bool allow_stall_route=true;
};
struct Decision {
  bool selected=false,coast_assist=false,stall_route=false;
  Reason reason=Reason::Inactive;
  Mode mode=Mode::None;
  PredictionSource prediction_source=PredictionSource::Unknown;
  uint16_t width_ms=0;
  uint32_t frame_age_ms=0;
  float forward_deg=NAN,asymmetry_deg=NAN,threshold_deg=NAN;
  float body_distance_deg=NAN,inward_speed_dps=NAN,estimated_zero_ms=NAN;
};
inline Decision decide(const Input& in,const Limits& limits=Limits{}) {
  Decision out;out.prediction_source=in.prediction_source;
  auto reject=[&](Reason reason){out.reason=reason;return out;};
  if(!in.active)return reject(Reason::Inactive);
  if(!in.run_id || in.foot_run_id!=in.run_id || !in.peak_id ||
      (in.peak_side!=1 && in.peak_side!=-1))return reject(Reason::Identity);
  if(!in.frame_ms || in.frame_ms>in.now_ms || in.peak_id>=in.now_ms ||
      in.now_ms-in.peak_id>kMaxPeakAgeMs)return reject(Reason::Time);
  out.frame_age_ms=in.now_ms-in.frame_ms;
  if(out.frame_age_ms>kMaxFrameAgeMs)return reject(Reason::StaleFrame);
  if(!in.pose_ready)return reject(Reason::PoseUnavailable);
  if(!in.pose_confident)return reject(Reason::PoseUncertain);
  if(!std::isfinite(in.left_deg)||!std::isfinite(in.right_deg)||
      !std::isfinite(in.left_uncertainty_deg)||!std::isfinite(in.right_uncertainty_deg)||
      !std::isfinite(in.body_angle_deg)||!std::isfinite(in.body_rate_dps))
    return reject(Reason::Nonfinite);
  if(in.left_uncertainty_deg<0 || in.right_uncertainty_deg<0 ||
      in.left_uncertainty_deg>kMaxUncertaintyDeg || in.right_uncertainty_deg>kMaxUncertaintyDeg)
    return reject(Reason::PoseUncertain);
  out.forward_deg=std::fmax(0.f,std::fmax(in.left_deg-in.left_uncertainty_deg,
                                       in.right_deg-in.right_uncertainty_deg));
  out.asymmetry_deg=std::fmax(0.f,std::fabs(in.left_deg-in.right_deg)-
      in.left_uncertainty_deg-in.right_uncertainty_deg);
  if(out.forward_deg<kMinForwardDeg)return reject(Reason::SmallForward);
  out.threshold_deg=std::fmax(1.5f,std::fmin(5.f,
      1.f+1.2f*out.forward_deg+.3f*out.asymmetry_deg));
  out.body_distance_deg=in.body_angle_deg*in.peak_side;
  out.inward_speed_dps=-in.body_rate_dps*in.peak_side;
  if(out.body_distance_deg<=0 || out.inward_speed_dps<=0)return reject(Reason::WrongMotion);
  if(out.body_distance_deg<kMinBodyDistanceDeg || out.body_distance_deg>out.threshold_deg)
    return reject(Reason::OutsideWindow);
  // Keep motion toward centre: a stopped or already reversed body cannot
  // authorize this trial, even with a previously latched deceleration flag.
  if(out.inward_speed_dps<kMinInwardSpeedDps || out.inward_speed_dps>kMaxInwardSpeedDps)
    return reject(Reason::Speed);
  if(in.normal_drive_selected==in.normal_coast_selected)return reject(Reason::NoOrdinaryDrive);
  const bool coast=in.normal_coast_selected;
  if(coast && !in.inward_decelerating && !in.anticipated_shape_need)
    return reject(Reason::CoastNeedsEvidence);
  if(coast && in.normal_width_ms)return reject(Reason::Width);
  const uint16_t cap=limits.max_ordinary_width_ms<kMaxPulseMs?limits.max_ordinary_width_ms:kMaxPulseMs;
  const uint16_t requested=coast?kCoastAssistPulseMs:in.normal_width_ms;
  if(!requested || (!coast && requested>cap) || (coast && requested>kCoastAssistPulseMs))
    return reject(Reason::Width);
  out.estimated_zero_ms=1000.f*out.body_distance_deg/out.inward_speed_dps;
  // Never relax the short-lead rejection. This is a constant-speed estimate,
  // not a promise that the pulse ends before a physical support transition.
  if(out.estimated_zero_ms<requested+kMinLeadMarginMs)
    return reject(Reason::InsufficientLead);
  if(out.estimated_zero_ms>kMaxEstimatedLeadMs) {
    if(!limits.allow_stall_route)return reject(Reason::TooEarly);
    // Do not reinterpret any distant slow motion as a stall. Shape evidence
    // includes forward/asymmetry margins and caller's incoming peak <=8.5deg.
    if(!in.inward_decelerating || !in.anticipated_shape_need ||
        !in.local_returning_confirmed || out.asymmetry_deg<1.f)
      return reject(Reason::StallNeedsEvidence);
    if(out.estimated_zero_ms>kMaxStallEstimatedLeadMs)return reject(Reason::StallTooEarly);
    out.stall_route=true;
  }
  out.selected=true;out.coast_assist=coast;out.reason=Reason::Selected;out.width_ms=requested;
  out.mode=out.stall_route?(coast?Mode::StallCoastAssist:Mode::StallOrdinaryRelocation):
      (coast?Mode::CoastAssist:Mode::OrdinaryRelocation);
  return out;
}
inline bool widthPreserved(const Decision& d,uint16_t guarded_width_ms) {
  return d.selected && d.width_ms>0 &&
      d.width_ms<=(d.coast_assist?kCoastAssistPulseMs:kMaxPulseMs) &&
      guarded_width_ms==d.width_ms;
}
}

// Unidentified bounded trial, not an inverse contact/energy model. The gain
// below expresses an experimental width increment; it is not measured torque.
namespace foot_support_assist_v13 {
using Reason=foot_support_assist_v12::Reason;
using Mode=foot_support_assist_v12::Mode;
enum class PredictionSource : uint8_t { Unknown,History,Local,LocalTurn };
constexpr uint16_t kMaxPulseMs=80,kMaxBoostMs=12;
constexpr uint32_t kMaxFrameAgeMs=160;
struct Input : foot_support_assist_v12::Input {
  PredictionSource prediction_source=PredictionSource::Unknown;
  bool local_turn_confirmed=false;
  uint16_t raw_normal_width_ms=0,raw_boost_ms=0,boosted_guarded_width_ms=0;
};
struct Decision : foot_support_assist_v12::Decision {
  PredictionSource prediction_source=PredictionSource::Unknown;
  uint16_t raw_boost_ms=0;
};
// Restrictive ablations only: neither option can enlarge the compiled limits.
struct Limits {bool allow_first_turn=true,allow_amount_boost=true;};
inline uint16_t boostMs(const Input& in){
  if(!in.pose_ready || !in.pose_confident || !in.normal_drive_selected ||
     in.normal_coast_selected || !std::isfinite(in.left_deg) || !std::isfinite(in.right_deg) ||
     !std::isfinite(in.left_uncertainty_deg) || !std::isfinite(in.right_uncertainty_deg) ||
     in.left_uncertainty_deg<0 || in.right_uncertainty_deg<0 ||
     in.left_uncertainty_deg>.75f || in.right_uncertainty_deg>.75f ||
     !in.raw_normal_width_ms || in.raw_normal_width_ms>=kMaxPulseMs)return 0;
  const float forward=std::fmax(in.left_deg-in.left_uncertainty_deg,in.right_deg-in.right_uncertainty_deg);
  const float asymmetry=std::fabs(in.left_deg-in.right_deg)-in.left_uncertainty_deg-in.right_uncertainty_deg;
  if(forward<=2.f || asymmetry<1.f)return 0;
  float levels=std::ceil(forward-2.f);
  if(levels>6.f)levels=6.f;
  uint16_t extra=static_cast<uint16_t>(levels)*2u;
  const uint16_t room=kMaxPulseMs-in.raw_normal_width_ms;
  return extra<room?extra:room;
}
inline Decision decide(const Input& in,const Limits& limits=Limits{}){
  Decision out;out.prediction_source=in.prediction_source;
  auto reject=[&](Reason reason){out.reason=reason;return out;};
  auto base=static_cast<const foot_support_assist_v12::Input&>(in);
  const bool turn=in.prediction_source==PredictionSource::LocalTurn;
  if(turn!=in.local_turn_confirmed)return reject(Reason::PoseUncertain);
  if(turn && !limits.allow_first_turn)return reject(Reason::PoseUncertain);
  if(in.normal_drive_selected && (!in.raw_normal_width_ms || in.raw_normal_width_ms>kMaxPulseMs))
    return reject(Reason::Width);
  if(turn){
    const float distance=in.body_angle_deg*in.peak_side;
    const float speed=-in.body_rate_dps*in.peak_side;
    const float forward=std::fmax(in.left_deg-in.left_uncertainty_deg,in.right_deg-in.right_uncertainty_deg);
    const float asymmetry=std::fabs(in.left_deg-in.right_deg)-in.left_uncertainty_deg-in.right_uncertainty_deg;
    if(in.now_ms<in.frame_ms || in.now_ms-in.frame_ms>100 ||
       !in.normal_drive_selected || in.normal_coast_selected || distance<2.f ||
       speed<12.f || speed>40.f || forward<3.f || asymmetry<2.f)
      return reject(Reason::PoseUncertain);
    base.local_returning_confirmed=false; // A first reversal never opens stall ETA.
  }
  const auto extra=limits.allow_amount_boost?boostMs(in):0;
  if(in.raw_boost_ms!=extra)return reject(Reason::Width);
  if(in.normal_drive_selected){
    if(in.boosted_guarded_width_ms<in.normal_width_ms ||
       in.boosted_guarded_width_ms>in.normal_width_ms+kMaxBoostMs)
      return reject(Reason::Width);
    base.normal_width_ms=in.boosted_guarded_width_ms;
  }
  const auto selected=foot_support_assist_v12::decide(base);
  static_cast<foot_support_assist_v12::Decision&>(out)=selected;
  out.prediction_source=in.prediction_source;out.raw_boost_ms=extra;
  // Leave time for the existing <=5ms backend recheck. Do not relax the
  // width+20ms invariant when a longer support input is requested.
  if(out.selected && out.estimated_zero_ms<out.width_ms+25.f){
    out.selected=false;out.reason=Reason::InsufficientLead;
  }
  return out;
}
inline bool widthPreserved(const Decision& in,uint16_t width){
  return foot_support_assist_v12::widthPreserved(in,width);
}
}

#if defined(FOOT_SUPPORT_TRANSFER) && FOOT_SUPPORT_TRANSFER
namespace foot_support_policy=foot_support_assist_v13;
#elif defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
namespace foot_support_policy=foot_support_assist_v12;
#else
namespace foot_support_policy=foot_support_assist;
#endif

namespace walking_return_energy {
struct ReturnInput {
  uint32_t now=0,peak_ms=0,foot_frame_ms=0,foot_ready_ms=0;
  int side=0;float peak_deg=0,angle_deg=0,rate_dps=0,left_deg=NAN,right_deg=NAN,target_j=0;
  bool active=false,foot_valid=false;
};
struct ReturnChoice {bool selected=false;float request_j=0,deceleration_dps2=0,stopping_distance_deg=0;};
class ReturnDetector {
 public:
  void reset(){*this=ReturnDetector{};}
  ReturnChoice observe(const ReturnInput& in){
    ReturnChoice out;
    const float distance=in.angle_deg*in.side,speed=-in.rate_dps*in.side;
    if(!in.active || (in.side!=1 && in.side!=-1) || !std::isfinite(distance) || !std::isfinite(speed) ||
       !std::isfinite(in.peak_deg) || !in.peak_ms || in.now<in.peak_ms || in.now-in.peak_ms<60 || in.now-in.peak_ms>800 ||
       distance<.35f || distance>12 || in.peak_deg-distance<.5f || speed<3 || speed>35 ||
       !in.foot_valid || !in.foot_frame_ms || !in.foot_ready_ms || in.foot_ready_ms<in.foot_frame_ms ||
       in.now<in.foot_ready_ms || in.now<in.foot_frame_ms || in.now-in.foot_frame_ms>160 ||
       !std::isfinite(in.left_deg) || !std::isfinite(in.right_deg) || std::fabs(in.left_deg-in.right_deg)<3 ||
       !std::isfinite(in.target_j) || in.target_j<=0){reset();return out;}
    if(peak_!=in.peak_ms || !anchor_ms_ || in.now<=last_ms_ || in.now-last_ms_>10){
      reset();peak_=in.peak_ms;anchor_ms_=last_ms_=in.now;anchor_speed_=speed;return out;}
    last_ms_=in.now;
    if(in.now-anchor_ms_>=20){
      const float dt=(in.now-anchor_ms_)*.001f;
      acceleration_=(anchor_speed_-speed)/dt;anchor_ms_=in.now;anchor_speed_=speed;
    }
    if(acceleration_<40){pending_=0;return out;}
    out.deceleration_dps2=acceleration_;out.stopping_distance_deg=coast_motion::stoppingDistanceDeg(speed,acceleration_);
    if(out.stopping_distance_deg+1.f>=distance){pending_=0;return out;}
    // Extend the locally observed loss of the effective rate-energy proxy
    // toward mechanical zero. This is a provisional reachability budget,
    // not a calibrated potential barrier or an inferred energy-neutral point.
    const float c=walking_energy_policy::passiveCoefficients(-in.side).omega2_j;
    const float rad=lateral_energy::kRadPerDeg;
    out.request_j=std::fmin(.5f*in.target_j,c*rad*rad*(2*acceleration_*distance-speed*speed));
    if(!pending_)pending_=in.now;
    out.selected=in.now-pending_>=6;
    return out;
  }
 private:
  uint32_t peak_=0,anchor_ms_=0,last_ms_=0,pending_=0;
  float anchor_speed_=0,acceleration_=0;
};
} // namespace walking_return_energy

// v18 trial observer. The IMU evidence and the latest completed camera frame
// have separate lifetimes. Missing markers never become measured foot angles.
// The energy is an effective rate-energy reachability proxy, not identified
// inertia, contact load, moving-foot potential, or measured mechanical work.
namespace return_observation_policy {
enum class Evidence : uint8_t {
  Unavailable=0, ObservedShape=1, RecognitionDropout=2, RangeRejected=3,
  ObservedUnseparated=4, MeasuredArcShortfall=5, PredictedArcShortfall=6, PostAidShortfall=7,
  ContinuationShortfall=8, RecheckedShortfall=9
};
inline bool targetsReceivingArc(Evidence reason) {
  return reason==Evidence::ContinuationShortfall || reason==Evidence::RecheckedShortfall;
}
inline bool isPostAid(Evidence reason) {
  return reason==Evidence::PostAidShortfall || targetsReceivingArc(reason);
}
struct FootFrame {
  uint16_t run_id=0;
  uint32_t frame_ms=0,ready_ms=0;
#if SUPPORT_TRANSFER_CARRY2 || SUPPORT_TRANSFER_STALL_V3
  uint32_t sequence=0;
  uint64_t image_us=0,received_us=0;
#endif
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
#if SUPPORT_TRANSFER_STALL_V3
  const periodic_foot::Packet* periodic=nullptr;
  float fore_aft_deg=NAN, periodic_acceleration_dps2=NAN;
  bool predict_left_return=false;
  float current_rise_tau_s=NAN;
  float current_goal_ma=NAN;
  uint32_t current_age_us=UINT32_MAX;
#endif
};
struct Choice {
  bool selected=false;
  float request_j=0,deceleration_dps2=0,stopping_distance_deg=0;
  Evidence evidence=Evidence::Unavailable;
  uint32_t foot_age_ms=0,deceleration_span_ms=0;
  bool range_veto=false;
#if SUPPORT_TRANSFER_STALL_V3
  float predicted_transfer_ms=NAN,terminal_deg=NAN;
  uint16_t loss_cap_ms=0;
  float aligned_current_ma=NAN, gross_work_limit_j=NAN;
  float release_deceleration_dps2=NAN;
  float previous_arc_deceleration_dps2=NAN;
  float response_window_ms=NAN;
#endif
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

#if SUPPORT_TRANSFER_STALL_V3
// Use the deceleration observed before issuing the new command. A rise in
// speed while powered is evidence of momentum, not a new estimate of the
// resistance the body will encounter after release.
inline bool canCoastToArc(float angle,float signed_rate,float terminal,float decel) {
  const float carry=support_transfer_plan::kCarryDps;
  return angle>0 && signed_rate<0 && terminal<0 && decel>0 &&
      std::isfinite(angle) && std::isfinite(signed_rate) && std::isfinite(terminal) && std::isfinite(decel) &&
      signed_rate*signed_rate >= 2.f*decel*(angle-terminal)+carry*carry;
}

// A completed left aid is a new observation horizon. While the body is still
// moving towards ZERO, check whether the measured speed gain has dissipated.
// Each accepted aid gets at most one such decision, including a vetoed one.
class PostAidObserver {
 public:
  bool releaseOnCarry(uint32_t sample_us,uint32_t age_us,uint32_t elapsed_ms,float angle,
      float rate,float terminal,float decel) {
    // The driver's queue uses millis() and the gyro uses micros(). Allow a
    // full ms of rounding when proving acquisition followed command start.
    const bool enough=sample_us && age_us<=10000 && elapsed_ms>age_us/1000+1 &&
        canCoastToArc(angle,rate,terminal,decel);
    const uint32_t gap=sample_us-ready_sample_us_;
    const bool confirmed=enough && ready_sample_us_ && gap>0 && gap<=10000;
    ready_sample_us_=enough?sample_us:0;
    return confirmed;
  }
  void arm(uint32_t peak,uint32_t end,float onset_speed,float delivered_work) {
    *this=PostAidObserver{};
    if (peak && end>peak && std::isfinite(onset_speed) && onset_speed>0 &&
        std::isfinite(delivered_work) && delivered_work>0) {
      peak_=peak;end_=end;onset_speed_=onset_speed;first_work_=delivered_work;
    }
  }
  float priorWork() const { return first_work_; }
  // After the second pulse the current model may have reported zero shortfall.
  // Its work counts against the gross-work ceiling. Reachability is inferred
  // again from subsequent motion under a zero-current command.
  void armRecheck(uint32_t peak,uint32_t end,float gross_work_sum_j,float previous_decel) {
    *this=PostAidObserver{};
    if (peak && end>peak && std::isfinite(gross_work_sum_j) && gross_work_sum_j>0 &&
        std::isfinite(previous_decel) && previous_decel>0) {
      peak_=peak;end_=end;first_work_=gross_work_sum_j;recheck_decel_=previous_decel;
    }
  }
  Choice observeRecheck(const Input& in,uint64_t now_us,float aligned_current_ma,bool event_allocation=false) {
    if (!(recheck_decel_>0)) return {};
    return observe(in,now_us,aligned_current_ma,true,event_allocation);
  }
  Choice observe(const Input& in,uint64_t now_us,float aligned_current_ma=NAN,bool continuation=false,
                 bool event_allocation=false) {
    Choice out;
    if (!peak_) return out;
    const bool recheck=recheck_decel_>0;
    if (in.peak_ms!=peak_ || in.side!=1 || in.now<end_ ||
        in.now-end_>support_transfer_plan::kForecastLimitMs ||
        (in.sample_us && in.sample_age_us<=10000 && (in.angle_deg<=0 || (!recheck && in.rate_dps>=0)))) {
      peak_=0; return out;
    }
    const auto* p=in.periodic;
    if (!p || !p->count || p->count==packet_) return out;
    if (p->published_us<p->epoch_us+uint64_t(end_)*1000 || !std::isfinite(in.periodic_acceleration_dps2)) return out;
    const bool adjacent=packet_ && p->count-packet_==1 && p->scheduled_us==slot_+periodic_foot::period_us;
    const float last_decel=adjacent ? (recheck?recheck_decel_:decel_) : 0.f, last_speed=speed_;
    const float last_angle=angle_;
    const uint64_t last_image=image_;
    packet_=p->count;slot_=p->scheduled_us;decel_=0;speed_=0;image_=0;
    if (!adjacent) max_speed_=0;
    const auto& l=p->left;
    const auto& r=p->right;
    const float speed=-in.rate_dps;
    if (recheck && in.sample_us && in.sample_age_us<=10000 &&
        (speed<=0 || (adjacent && last_image && in.angle_deg>=last_angle))) {
      peak_=0;return out;
    }
    if (!in.active || !in.sample_us || in.sample_age_us>10000 || assessFoot(in)==Evidence::RangeRejected ||
        (recheck && (in.run_id==0 || in.foot.run_id!=in.run_id ||
                     in.now-end_<=in.sample_age_us/1000u+1)) ||
        !(in.angle_deg>0 && in.angle_deg<=12 &&
        speed>=support_transfer_plan::kCarryDps && speed<=35 && in.target_j>0 && std::isfinite(in.target_j)) ||
        !periodic_foot::matchesPair(*p,now_us,l.sequence,l.image_us) || !l.sequence ||
        !(std::fabs(in.fore_aft_deg)<=10 && std::fabs(l.measured)<=20 && std::fabs(r.measured)<=20 &&
          std::fabs(l.estimated)<=20 && std::fabs(r.estimated)<=20 && l.measured>r.measured && l.estimated>r.estimated) ||
        !(support_transfer_plan::supportGap(l.measured,r.measured,in.fore_aft_deg,in.angle_deg)<0 &&
          support_transfer_plan::supportGap(l.estimated,r.estimated,in.fore_aft_deg,in.angle_deg)<0)) {
      max_speed_=0; return out;
    }
    image_=l.image_us;speed_=speed;angle_=in.angle_deg;max_speed_=std::fmax(max_speed_,speed);
    const float a=in.periodic_acceleration_dps2;
    if (!(std::isfinite(a) && a>0 && speed*speed<2.f*a*in.angle_deg)) return out;
    decel_=a;
    const float kinetic=walking_energy_policy::passiveCoefficients(-1).omega2_j*
                        lateral_energy::kRadPerDeg*lateral_energy::kRadPerDeg;
    // The foot camera is slower than the IMU. For a recheck the images certify
    // geometry; two new body samples, not two new exposures, certify the loss.
    const bool post_motion=recheck ? in.angle_deg<last_angle : image_>=p->epoch_us+uint64_t(end_)*1000;
    if (!(last_decel>0 && speed<last_speed && last_image && image_>last_image &&
          post_motion && std::isfinite(aligned_current_ma))) return out;
    // A passed work budget describes the command model. It need not all
    // appear as observable lateral kinetic energy, especially across impact.
    const float conservative=std::fmin(last_decel,a);
    const float stopping_distance=coast_motion::stoppingDistanceDeg(speed,conservative);
    const bool lost_first_work=speed<onset_speed_ && max_speed_>onset_speed_ &&
          kinetic*(max_speed_*max_speed_-speed*speed)>=first_work_;
    // A newly inferred lack of momentum inside one foot-observation slot
    // may resolve at the next sample. Preserve that time for the main input.
    const float one_slot_travel=speed*(periodic_foot::period_us*1.e-6f);
    if (!lost_first_work && (!continuation || in.angle_deg-stopping_distance<=one_slot_travel)) return out;
    const float higher_decel=std::fmax(last_decel,a);
    const float time_to_stop=speed/higher_decel;
    if (!(in.current_rise_tau_s>0 && std::isfinite(in.current_rise_tau_s))) return out;
    const float dispatch_s=.005f;
    float response_s=in.current_rise_tau_s+dispatch_s;
    if (time_to_stop<response_s && event_allocation && continuation && in.current_age_us<=50000 &&
        aligned_current_ma>0 && std::isfinite(in.current_goal_ma) && aligned_current_ma<in.current_goal_ma) {
      // Compare at the same current as a cold start at one time constant.
      // Propagate the last zero-command current to the latest allowed start.
      const float at_start=aligned_current_ma*std::exp(-(in.current_age_us*1.e-6f+dispatch_s)/
                                                       walking_energy_policy::kCoastFallTauS);
      const float rise_fraction=std::fmax(0.f,1.f+std::log1p(-at_start/in.current_goal_ma));
      response_s=dispatch_s+in.current_rise_tau_s*rise_fraction;
    }
    if (!(time_to_stop>=response_s && time_to_stop>=dispatch_s+.001f)) return out;
    out.response_window_ms=1000.f*response_s;
    const float carry=support_transfer_plan::kCarryDps;
    out.gross_work_limit_j=.5f*in.target_j-first_work_;
    const float terminal=continuation ? support_transfer_plan::arcEntryDeg(-1,l.measured+in.fore_aft_deg) : 0.f;
    out.request_j=std::fmin(out.gross_work_limit_j,kinetic*(2.f*conservative*(in.angle_deg-terminal)+carry*carry-speed*speed));
    if (!(out.request_j>0)) return out;
    // Keep the command inside the observed inward interval even allowing
    // the same five ms of dispatch latency as the normal freshness gate.
    out.loss_cap_ms=static_cast<uint16_t>(std::fmin(1000.f*(time_to_stop-dispatch_s),walking_energy_policy::kMaxPulseMs));
    out.selected=true;
    out.evidence=recheck?Evidence::RecheckedShortfall:(continuation?Evidence::ContinuationShortfall:Evidence::PostAidShortfall);
    out.terminal_deg=terminal;
    // Both frames must predict enough residual momentum before releasing
    // the new input. The higher loss is the stricter coast projection.
    if (continuation) out.release_deceleration_dps2=higher_decel;
    out.foot_age_ms=static_cast<uint32_t>((now_us-image_)/1000);
    out.deceleration_dps2=conservative;out.stopping_distance_deg=stopping_distance;
    out.aligned_current_ma=aligned_current_ma;peak_=0;
    return out;
  }
 private:
  uint32_t peak_=0,end_=0,packet_=0,ready_sample_us_=0;
  uint64_t slot_=0,image_=0;
  float onset_speed_=0, first_work_=0, max_speed_=0, speed_=0, decel_=0, angle_=0, recheck_decel_=0;
};
#endif

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
    if(!in.run_id || !in.peak_ms){
#if SUPPORT_TRANSFER_STALL_V3
      arc_confirming_=false;
      approaching_gap_=NAN;
#endif
      clearMotion();return out;
    }
    if(run_!=in.run_id || peak_!=in.peak_ms){reset();run_=in.run_id;peak_=in.peak_ms;}
    out.evidence=assessFoot(in);
    if(out.evidence!=Evidence::Unavailable)out.foot_age_ms=in.now-in.foot.frame_ms;
    if(out.evidence==Evidence::RangeRejected)range_rejected_=true;
    else if(out.evidence==Evidence::ObservedShape || out.evidence==Evidence::ObservedUnseparated)range_rejected_=false;
    out.range_veto=range_rejected_;
#if SUPPORT_TRANSFER_STALL_V3
    const auto arc=observeSmallArc(in,out.evidence);
    if (arc.selected) return arc;
#endif

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
    out.stopping_distance_deg=coast_motion::stoppingDistanceDeg(speed,acceleration_);
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
#if SUPPORT_TRANSFER_STALL_V3
  // A 25 ms pair must show the same receiving sole, decreasing inward speed,
  // and a projected stop on the departing side of mechanical ZERO twice.
  Choice observeSmallArc(const Input& in, Evidence evidence) {
    Choice out;
    const auto* p=in.periodic;
    const float speed=-in.rate_dps;
    const bool approaching=!consumed_ && in.active && in.side==1 && in.sample_us &&
        in.sample_age_us<=10000 && in.angle_deg>=.35f && in.angle_deg<=12.f &&
        in.peak_deg-in.angle_deg>=.5f && in.now>=in.peak_ms && in.now-in.peak_ms>=60 &&
        in.now-in.peak_ms<=800 && speed>=3.f && speed<=35.f &&
        evidence==Evidence::ObservedUnseparated && in.foot.left_deg>in.foot.right_deg;
    if (!approaching) {
      arc_confirming_=false;
      // A measured change back towards the departing side after a forecast
      // sample ends this approach. A later gyro fluctuation is not a new step.
      if (in.predict_left_return && approaching_image_ &&
          in.sample_us && in.sample_age_us<=10000 && speed<=0) approach_reversed_=true;
      approaching_gap_=NAN;
    }
    if (!p || !p->count || !p->epoch_us || p->count==arc_packet_) return out;
    const uint32_t dt=in.sample_us-arc_sample_us_;
    const bool adjacent=arc_packet_ && p->count-arc_packet_==1 && dt>=15000 && dt<=50000 &&
        p->scheduled_us-arc_slot_us_==periodic_foot::period_us;
    // The control core already differentiates live gyro rates at adjacent
    // periodic frames. It passes that result only on the originating IMU tick.
    const float deceleration=in.periodic_acceleration_dps2;
    arc_packet_=p->count; arc_sample_us_=in.sample_us; arc_slot_us_=p->scheduled_us;
    const bool previous=arc_confirming_;
    const float previous_deceleration=arc_deceleration_;
    const uint64_t previous_image=arc_image_us_;
    const float previous_gap=approaching_gap_;
    const uint64_t previous_gap_image=approaching_image_;
    arc_confirming_=false;
    approaching_gap_=NAN;
    const auto& l=p->left;
    const auto& r=p->right;
    const uint32_t now_us=in.sample_us+in.sample_age_us;
    if (!approaching || !std::isfinite(deceleration) || deceleration<60.f || !l.sequence ||
        l.sequence!=r.sequence || l.sequence!=in.foot.sequence || !l.image_us ||
        l.image_us!=r.image_us || l.image_us!=in.foot.image_us || l.image_us<p->epoch_us ||
        p->published_us<l.image_us || p->published_us-l.image_us>periodic_foot::maximum_age_us ||
        now_us-static_cast<uint32_t>(p->published_us)>periodic_foot::period_us ||
        now_us-static_cast<uint32_t>(l.image_us)>periodic_foot::maximum_age_us ||
        !(std::fabs(in.fore_aft_deg)<=10 && std::fabs(l.measured)<=20 && std::fabs(r.measured)<=20 &&
          l.measured>r.measured && l.measured-r.measured<3 && in.target_j>0 && std::isfinite(in.target_j))) return out;
    const float stop=speed*speed/(2.f*deceleration);
    if (stop>=in.angle_deg) return out;
    const float gap=support_transfer_plan::supportGap(l.measured,r.measured,in.fore_aft_deg,in.angle_deg);
    Evidence reason=Evidence::Unavailable;
    if (gap<0 && l.image_us>previous_image) {
      arc_confirming_=true; arc_image_us_=l.image_us;arc_deceleration_=deceleration;
      if (adjacent && previous) {
        reason=Evidence::MeasuredArcShortfall;
        out.previous_arc_deceleration_dps2=previous_deceleration;
      }
    }
    // During current rise a still-higher receiving sole can reach the ground.
    // Forecast its gap from consecutive measured poses, only while the body
    // is still moving left and both snapshots predict a stop before ZERO.
    if (in.predict_left_return && !approach_reversed_ && speed>=support_transfer_plan::kCarryDps &&
        std::isfinite(in.current_rise_tau_s) && in.current_rise_tau_s>0 && std::isfinite(gap) &&
        l.image_us>previous_gap_image) {
      approaching_gap_=gap;
      approaching_image_=l.image_us;
      if (adjacent && previous_gap>0 && previous_gap>gap && speed<approaching_speed_) {
        const float until_stop=speed/deceleration;
        const float lead=std::fmin(in.current_rise_tau_s,2.e-6f*periodic_foot::period_us);
        const float until_transfer=std::fmax(0.f,gap)*float(dt)*1.e-6f/(previous_gap-gap);
        if (until_transfer<=lead && until_transfer<until_stop) {
          reason=Evidence::PredictedArcShortfall;
          out.predicted_transfer_ms=1000.f*until_transfer;
        }
      }
      approaching_speed_=speed;
    }
    if (reason==Evidence::Unavailable) return out;
    const float entry=support_transfer_plan::arcEntryDeg(-1,l.measured+in.fore_aft_deg);
    if (reason==Evidence::PredictedArcShortfall) out.terminal_deg=entry;
    const float c=walking_energy_policy::passiveCoefficients(-1).omega2_j;
    const float rad=lateral_energy::kRadPerDeg, carry=support_transfer_plan::kCarryDps;
    const float needed=c*rad*rad*(2*deceleration*(in.angle_deg-entry)-speed*speed+carry*carry);
    out.request_j=std::fmin(.25f*in.target_j,needed);
    out.selected=out.request_j>0; out.evidence=reason;
    out.deceleration_dps2=deceleration; out.stopping_distance_deg=stop;
    out.foot_age_ms=in.now-in.foot.frame_ms;
    return out;
  }
  uint32_t arc_packet_=0,arc_sample_us_=0;
  uint64_t arc_image_us_=0,arc_slot_us_=0;
  bool arc_confirming_=false;
  float arc_deceleration_=NAN;
  float approaching_gap_=NAN,approaching_speed_=0;
  uint64_t approaching_image_=0;
  bool approach_reversed_=false;
#endif
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

// A separate, bounded launch from rest after a walking cycle has stalled.
// Real crossings and signed excursions are observations. A queued command
// alone never confirms a restart.
namespace settled_walking_restart {
constexpr uint32_t quiet_ms=500, inactive_ms=1000, retry_ms=1600;
constexpr uint16_t episode_ms=80;
constexpr uint8_t max_attempts=2; // per measurement, including failed writes
struct Input {
  uint32_t now_ms=0,sample_us=0;
  float angle_deg=NAN,rate_dps=NAN,fore_aft_deg=NAN,fore_aft_rate_dps=NAN;
  float left_deg=NAN,right_deg=NAN;
  bool run_active=false,guarded=false,feet_fresh=false,pulse_active=false,had_peak=false,crossed_zero=false;
};
struct Attempt {
  uint32_t start=0, cross=0, opposite_peak=0, elapsed=0, timeout=0;
  int8_t direction=0;
  uint16_t width=0;
  float angle=NAN, reference_speed=NAN, demand_j=NAN, predicted_j=NAN;
  bool queued=false;
};
class Detector {
 public:
  int8_t observe(const Input& in) {
    if(!in.run_active || !in.sample_us){quiet_start_=0;return 0;}
    if(in.sample_us==sample_)return 0;
    if(in.sample_us-sample_>30000 || !in.guarded) {
      quiet_start_=0;reference_candidate_=0;last_excursion_side_=0;
    }
    sample_=in.sample_us;
    if(pending_ && in.now_ms-records[count-1].start>retry_ms) {
      records[count-1].timeout=in.now_ms;
      pending_=0;
    }
    if(in.guarded && in.crossed_zero && std::isfinite(in.rate_dps)) {
      last_active_=in.now_ms;
      const int8_t side=in.rate_dps>0?1:-1;
      const float speed=std::fabs(in.rate_dps);
      reference_candidate_=last_excursion_side_==-side && speed>=6 && speed<=95 ? side : 0;
      candidate_speed_=speed;
      if(episode_side_ && side==episode_side_)bracket_=true;
    }
    if(in.guarded && std::fabs(in.angle_deg)>=.5f) {
      const int8_t excursion=in.angle_deg>0?1:-1;
      if(excursion==reference_candidate_ && excursion!=last_excursion_side_) {
        auto& history=reference_[excursion>0];
        history.speeds[history.next]=candidate_speed_;
        history.next=(history.next+1)%3;
        if(history.size<3)++history.size;
      }
      reference_candidate_=0;last_excursion_side_=excursion;
    }
    if(in.guarded && episode_side_ && bracket_ && in.angle_deg*episode_side_>=.5f) {
      if(pending_)records[count-1].cross=in.now_ms;
      pending_=episode_side_=0;bracket_=false;used_=0;
    }
    if(in.pulse_active)last_active_=in.now_ms;
    const float lean=std::fabs(in.angle_deg);
    if(!in.guarded || !in.feet_fresh || !in.had_peak || in.pulse_active ||
       !last_active_ || in.now_ms-last_active_<inactive_ms ||
       count>=max_attempts || write_failed_ || (count && in.now_ms-records[count-1].start<retry_ms) ||
       !(lean>=1.f && lean<=9.f && std::fabs(in.rate_dps)<=4.f &&
         std::fabs(in.fore_aft_deg)<=6.f && std::fabs(in.fore_aft_rate_dps)<=12.f &&
         std::fabs(in.left_deg)<=10.f && std::fabs(in.right_deg)<=10.f)) {
      quiet_start_=0;return 0;
    }
    const int8_t direction=in.angle_deg>0?-1:1;
    if(!quiet_start_ || std::fabs(in.angle_deg-quiet_angle_)>.45f ||
        std::fabs(in.left_deg-quiet_left_)>.45f || std::fabs(in.right_deg-quiet_right_)>.45f) {
      quiet_start_=in.now_ms;quiet_angle_=in.angle_deg;quiet_left_=in.left_deg;quiet_right_=in.right_deg;
      return 0;
    }
    return in.now_ms-quiet_start_>=quiet_ms && referenceSpeed(direction)>0 && remaining()>0 ? direction : 0;
  }
  float referenceSpeed(int8_t direction) const {
    const auto& h=reference_[direction>0];
    if(!h.size)return 0;
    if(h.size==1)return h.speeds[0];
    if(h.size==2)return .5f*(h.speeds[0]+h.speeds[1]);
    const float a=h.speeds[0],b=h.speeds[1],c=h.speeds[2];
    return std::max(std::min(a,b),std::min(std::max(a,b),c));
  }
  uint16_t remaining()const{return used_<episode_ms?episode_ms-used_:0;}
  int8_t pending()const{return pending_;}
  bool bracketed()const{return bracket_;}
  Attempt& attempt(uint32_t t,int8_t direction) {
    auto& a=records[count++];a.start=t;a.direction=direction;
    quiet_start_=0;last_active_=t;
    return a;
  }
  void accepted(uint16_t width) {
    auto& a=records[count-1];a.queued=true;a.width=width;
    pending_=episode_side_=a.direction;bracket_=false;reserved_=width;used_+=width;
  }
  void failed(){write_failed_=true;}
  void completed(uint32_t elapsed) {
    if(!reserved_)return;
    records[count-1].elapsed=elapsed;
    if(used_>=reserved_)used_=std::min<uint32_t>(episode_ms,used_-reserved_+elapsed);
    reserved_=0;
  }
  void peak(uint32_t t,int8_t direction) {
    if(!count)return;
    auto& a=records[count-1];
    if(a.cross && !a.opposite_peak && t>=a.cross && a.direction==direction)a.opposite_peak=t;
  }
  Attempt records[max_attempts]{};
  uint8_t count=0;
 private:
  uint32_t sample_=0,quiet_start_=0,last_active_=0;
  uint16_t used_=0,reserved_=0;
  int8_t pending_=0,episode_side_=0,reference_candidate_=0,last_excursion_side_=0;
  bool bracket_=false,write_failed_=false;
  float quiet_angle_=0,quiet_left_=0,quiet_right_=0,candidate_speed_=0;
  struct Reference {float speeds[3]{};uint8_t size=0,next=0;} reference_[2];
};
} // namespace settled_walking_restart
