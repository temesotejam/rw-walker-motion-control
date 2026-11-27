#pragma once
#include <cmath>
#include <cstdint>

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
