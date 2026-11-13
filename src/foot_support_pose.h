#pragma once
#include "foot_phase_prediction.h"
#include "foot_local_prediction.h"

// One source-selection function shared by firmware and causal replay. Local
// evidence never increases the history-match score or makes it a probability.
namespace foot_support_pose {
enum class Source : uint8_t { Unknown,History,Local,LocalTurn };
struct Selection {
  foot_phase_prediction::Result pose;
  Source source=Source::Unknown;
  bool usable=false,local_returning_confirmed=false;
  bool local_turn_confirmed=false;
  float local_evidence=0;
};
inline foot_phase_prediction::Phase phase(foot_local_prediction::Phase p){
  using L=foot_local_prediction::Phase;using H=foot_phase_prediction::Phase;
  switch(p){case L::Stationary:return H::Stationary;case L::Forward:return H::Forward;
    case L::Turning:return H::Turning;case L::Returning:return H::Returning;default:return H::Unknown;}
}
inline bool margins(float l,float r){return std::isfinite(l)&&std::isfinite(r)&&l>=0&&r>=0&&l<=.75f&&r<=.75f;}
inline Selection select(const foot_phase_prediction::Projection& history,
    const foot_local_prediction::Projection& local,uint32_t now,bool allow_first_turn=false){
  Selection out;out.pose=history.predict(now,0);
  if(out.pose.ready() && out.pose.age_ms<=160 && out.pose.confidence>=.5f &&
     out.pose.method==foot_phase_prediction::Method::History &&
     margins(out.pose.left_uncertainty,out.pose.right_uncertainty)){
    out.source=Source::History;out.usable=true;return out;
  }
  // Both projections must describe the same run and image, including losses.
  if(!history.run_id || history.run_id!=local.run_id || history.frame_ms!=local.frame_ms)return out;
  const auto q=local.predict(now,0);
  if(!q.ready() || q.age_ms>160 ||
     !std::isfinite(q.left)||!std::isfinite(q.right))return out;
  auto adopted=q;
  // A first small drop after a substantial rise is *tentative*, not confirmed
  // Returning. Use an observed angle (no extrapolation) for at most 100ms.
  // This new source is opt-in: v12 calls keep their original behavior.
  bool turn=false;
  if(allow_first_turn && q.age_ms<=100){
    const bool ld=q.left>=q.right;
    const auto& m=ld?local.left_model:local.right_model;
    const auto& other=ld?local.right_model:local.left_model;
    const float drop=-m.last_change;
    if(m.phase==foot_local_prediction::Phase::Turning && m.prior_change>=1.f &&
       drop>=.15f && drop<=1.5f && m.latest_rate<=-1.5f && m.latest_rate>=-20.f &&
       m.last_interval_ms>=60 && m.last_interval_ms<=150 &&
       other.phase==foot_local_prediction::Phase::Stationary && other.qualified){
      const float margin=.45f+.5f*drop+.002f*q.age_ms;
      const float other_margin=ld?q.right_uncertainty:q.left_uncertainty;
      if(m.observed-margin>=3.f && std::fabs(q.left-q.right)-margin-other_margin>=2.f){
        if(ld){adopted.left=m.observed;adopted.left_uncertainty=margin;}
        else {adopted.right=m.observed;adopted.right_uncertainty=margin;}
        turn=true;
      }
    }
  }
  if(!margins(adopted.left_uncertainty,adopted.right_uncertainty))return out;
  const bool left=adopted.left-adopted.left_uncertainty>=adopted.right-adopted.right_uncertainty;
  if(!turn && (left?q.phase_left:q.phase_right)!=foot_local_prediction::Phase::Returning)return out;
  // Preserve the historical score for logging; local evidence is separate.
  out.pose.reason=foot_phase_prediction::Reason::Ready;
  out.pose.left=adopted.left;out.pose.right=adopted.right;out.pose.left_rate=q.left_rate;out.pose.right_rate=q.right_rate;
  out.pose.left_uncertainty=adopted.left_uncertainty;out.pose.right_uncertainty=adopted.right_uncertainty;
  out.pose.phase_left=phase(q.phase_left);out.pose.phase_right=phase(q.phase_right);
  out.pose.age_ms=q.age_ms;out.pose.horizon_ms=q.horizon_ms;
  out.source=turn?Source::LocalTurn:Source::Local;out.usable=true;
  out.local_returning_confirmed=!turn;out.local_turn_confirmed=turn;out.local_evidence=q.confidence;
  return out;
}
}
