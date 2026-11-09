#pragma once
#include "wheel_work_energy_control.h"

// Re-evaluate one proposed positive-side pulse with the SAME calibrated
// current response, physical state, biases and branch as the normal selector.
// This helper does not establish that a camera span or an input response is a
// ground stride. The caller retains all hardware and posture guard authority.
namespace foot_span_candidate {
inline bool supportedBranch(uint8_t branch) {return branch==5 || branch==6 || branch==7;}
inline bool supportedCurrent(int16_t current) {return current==300 || current==350 || current==400;}
inline wheel_work_energy_control::Choice choose(const lateral_energy::Geometry& geometry,
    const wheel_work_energy_control::State& state,const wheel_work_energy_control::Choice& baseline,
    const lateral_energy::CurrentTable& table,uint16_t width,float initial_current_ma,
    float goal_ma,float target_mj,float work_bias_mj,float bridge_bias_mj) {
  using namespace wheel_work_energy_control;
  Choice result;
  const int16_t ma=baseline.current_ma;
  if(!baseline.valid || !supportedBranch(baseline.branch) || !supportedCurrent(ma) ||
     baseline.command_sign!=1 || width<10 || width>100 || width>baseline.allowed_cap_ms ||
     !std::isfinite(target_mj) || target_mj<=0.f) return result;
  const float q=lateral_energy::charge(table,width,initial_current_ma,goal_ma);
  if(!std::isfinite(q) || q<0.f || q>25.f)return result;
  const Prediction p=positiveBoundary(geometry,state,q,work_bias_mj,bridge_bias_mj);
  // Model switching is not permission to choose an intervention from a
  // different response equation than the already-chosen ordinary candidate.
  if(!p.valid || p.branch!=baseline.branch)return result;
  consider(result,ma,width,q,p,target_mj);
  result.allowed_cap_ms=width;
  result.minimum_mj=baseline.minimum_mj;result.maximum_mj=p.energy_mj;
  finish(result,target_mj);
  return result;
}
}
