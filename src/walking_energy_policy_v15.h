#pragma once
#include "walking_energy_policy.h"

// v15 keeps the v14 potential, paired-peak corrections and empirical response
// coefficients. Only integer-width selection and early admission change.
// Reaching the model's energy request is not a physical walking guarantee.
namespace walking_energy_policy_v15 {
using Input=walking_energy_policy::Input;
using Decision=walking_energy_policy::Decision;
using Reason=walking_energy_policy::Reason;
constexpr char kRevision[]="walking_early_energy_main_v15";
constexpr uint16_t kMaxPulseMs=walking_energy_policy::kMaxPulseMs;
// Numerical/serialization tolerance only; not an allowance for identified
// model error or a substitute for sufficient pulse work.
constexpr float kEnergyRoundoffJ=1e-9f;

namespace detail {
inline Decision atWidth(Decision out,const lateral_energy::CurrentTable& current,
    const Input& in,uint16_t width){
  out.width_ms=width;out.q_mas=0;out.predicted_work_j=0;
  out.predicted_arrival_j=out.available_j;out.unmet_j=out.request_j;
  out.coast=out.request_j==0.f;
  if(width){
    const auto c=lateral_energy::coefficients(in.side);
    out.q_mas=lateral_energy::charge(current,width,in.initial_current_ma,in.goal_current_ma);
    // Never count reverse residual charge as positive added work via abs(Q)
    // or via the quadratic term outside its identified excitation branch.
    if(!std::isfinite(out.q_mas) || out.q_mas<0){
      out.valid=out.selected=false;out.saturated=true;out.reason=Reason::Invalid;
      out.predicted_work_j=out.predicted_arrival_j=NAN;return out;
    }
    out.predicted_work_j=lateral_energy::work(c,in.rate_dps*lateral_energy::kRadPerDeg,out.q_mas);
    if(!std::isfinite(out.predicted_work_j) || out.predicted_work_j<0){
      out.valid=out.selected=false;out.saturated=true;out.reason=Reason::Invalid;return out;
    }
    out.predicted_arrival_j=out.available_j+out.predicted_work_j;
    out.unmet_j=std::fmax(0.f,out.request_j-out.predicted_work_j);
  }
  out.saturated=out.unmet_j>kEnergyRoundoffJ;
  const auto c=lateral_energy::coefficients(in.side);
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  // A guard blocking all input while energy is missing is not surplus coast
  // and cannot become a zero-width drive. Retain its energy audit, fail closed.
  if(!width && !out.coast){out.valid=out.selected=false;out.reason=Reason::CapLimited;}
  return out;
}
}

inline Decision decide(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in){
  // Reuse the frozen v14 energy accounting and validity checks (including
  // cap0 invalid). Its nearest-width choice is discarded below.
  auto out=walking_energy_policy::decide(geometry,current,in);
  if(!out.valid)return out;
  out.evaluations=0;
  if(out.request_j==0.f)return detail::atWidth(out,current,in,0);
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float maximum_q=lateral_energy::charge(current,in.cap_ms,in.initial_current_ma,in.goal_current_ma);
  if(!std::isfinite(maximum_q) || maximum_q<0){
    return detail::atWidth(out,current,in,in.cap_ms);
  }
  // 80ms is the inherited engineering envelope, not a physics-derived bound.
  // If even the cap cannot cover the request, report its remaining deficit.
  if(out.maximum_work_j<out.request_j)return detail::atWidth(out,current,in,in.cap_ms);
  uint16_t lo=0,hi=in.cap_ms;
  while(hi-lo>1){
    const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
    const float q=lateral_energy::charge(current,mid,in.initial_current_ma,in.goal_current_ma);
    ++out.evaluations;
    if(!std::isfinite(q)){
      out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
    }
    if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;
    else hi=mid;
  }
  // Unlike nearest-energy rounding, this is the least integer width whose
  // predicted work reaches a positive request. Tiny requests never round down
  // to surplus coast.
  return detail::atWidth(out,current,in,hi);
}

inline Decision evaluateWidth(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in,uint16_t actual_width_ms){
  auto out=walking_energy_policy_v15::decide(geometry,current,in);
  // A post-selection guard may only shorten the selected command. In
  // particular, a zero-request coast cannot acquire a positive pulse here.
  if(!out.valid || actual_width_ms>in.cap_ms || actual_width_ms>out.width_ms){
    out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
  }
  return detail::atWidth(out,current,in,actual_width_ms);
}

inline bool earlyAdmissible(const Decision& post_guard,float epsilon_j=kEnergyRoundoffJ){
  return std::isfinite(epsilon_j) && epsilon_j>=0 && epsilon_j<=kEnergyRoundoffJ &&
      post_guard.valid && post_guard.selected && post_guard.width_ms>0 && !post_guard.coast &&
      !post_guard.saturated && std::isfinite(post_guard.request_j) && post_guard.request_j>0 &&
      std::isfinite(post_guard.predicted_work_j) && post_guard.predicted_work_j>=0 &&
      std::isfinite(post_guard.q_mas) && post_guard.q_mas>=0 &&
      post_guard.predicted_work_j+epsilon_j>=post_guard.request_j;
}
}
