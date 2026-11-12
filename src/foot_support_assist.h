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
