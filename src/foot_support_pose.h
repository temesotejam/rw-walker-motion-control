#pragma once
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"

// One source-selection function shared by firmware and causal replay. Local
// evidence never increases the history-match score or makes it a probability.
namespace foot_support_pose {
enum class Source : uint8_t { Unknown,History,Local };
struct Selection {
  foot_phase_prediction::Result pose;
  Source source=Source::Unknown;
  bool usable=false,local_returning_confirmed=false;
  float local_evidence=0;
};
inline foot_phase_prediction::Phase phase(foot_local_prediction::Phase p){
  using L=foot_local_prediction::Phase;using H=foot_phase_prediction::Phase;
  switch(p){case L::Stationary:return H::Stationary;case L::Forward:return H::Forward;
    case L::Turning:return H::Turning;case L::Returning:return H::Returning;default:return H::Unknown;}
}
inline bool margins(float l,float r){return std::isfinite(l)&&std::isfinite(r)&&l>=0&&r>=0&&l<=.75f&&r<=.75f;}
inline Selection select(const foot_phase_prediction::Projection& history,
    const foot_local_prediction::Projection& local,uint32_t now){
  Selection out;out.pose=history.predict(now,0);
  if(out.pose.ready() && out.pose.age_ms<=160 && out.pose.confidence>=.5f &&
     out.pose.method==foot_phase_prediction::Method::History &&
     margins(out.pose.left_uncertainty,out.pose.right_uncertainty)){
    out.source=Source::History;out.usable=true;return out;
  }
  // Both projections must describe the same run and image, including losses.
  if(!history.run_id || history.run_id!=local.run_id || history.frame_ms!=local.frame_ms)return out;
  const auto q=local.predict(now,0);
  if(!q.ready() || q.age_ms>160 || !margins(q.left_uncertainty,q.right_uncertainty) ||
     !std::isfinite(q.left)||!std::isfinite(q.right))return out;
  const bool left=q.left-q.left_uncertainty>=q.right-q.right_uncertainty;
  if((left?q.phase_left:q.phase_right)!=foot_local_prediction::Phase::Returning)return out;
  // Preserve the historical score for logging; local evidence is separate.
  out.pose.reason=foot_phase_prediction::Reason::Ready;
  out.pose.left=q.left;out.pose.right=q.right;out.pose.left_rate=q.left_rate;out.pose.right_rate=q.right_rate;
  out.pose.left_uncertainty=q.left_uncertainty;out.pose.right_uncertainty=q.right_uncertainty;
  out.pose.phase_left=phase(q.phase_left);out.pose.phase_right=phase(q.phase_right);
  out.pose.age_ms=q.age_ms;out.pose.horizon_ms=q.horizon_ms;
  out.source=Source::Local;out.usable=true;out.local_returning_confirmed=true;out.local_evidence=q.confidence;
  return out;
}
}
