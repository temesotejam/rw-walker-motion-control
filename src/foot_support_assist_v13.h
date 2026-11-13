#pragma once
#include "foot_support_assist_v12.h"

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
