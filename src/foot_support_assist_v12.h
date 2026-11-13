#pragma once
#include "foot_support_assist.h"

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
