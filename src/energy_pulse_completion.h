#pragma once
#include "lateral_energy_control.h"

// Post-command prediction using actual elapsed SOFTWARE time. The current
// rise and inherited pulse-Q arrival response remain empirical models, not a
// measurement of delivered charge/work or identified physical body inertia.
namespace energy_pulse_completion {
constexpr char kRevision[]="actual_elapsed_pulse_completion_v16";
struct Input {
  int8_t side=0;
  float rate_dps=NAN,initial_current_ma=NAN,goal_current_ma=NAN,tau_s=NAN;
  float available_j=NAN,request_j=NAN;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
};
struct Result {
  // valid covers inputs and finite current/charge arithmetic. A negative net
  // signed Q is retained for audit but does NOT have a valid work prediction.
  bool valid=false,work_valid=false,duration_completed=false;
  bool width_supported=false,rate_supported=false,charge_supported=false,extrapolated=false;
  uint32_t actual_elapsed_ms=0;
  uint16_t planned_ms=0;
  float elapsed_s=NAN,q_mas=NAN,current_end_ma=NAN;
  float work_j=NAN,arrival_j=NAN,unmet_j=NAN;
};
inline Result evaluate(const Input& in){
  Result out;out.actual_elapsed_ms=in.actual_elapsed_ms;out.planned_ms=in.planned_ms;
  out.duration_completed=in.planned_ms>0 && in.actual_elapsed_ms>=in.planned_ms;
  // This flag describes model coverage, never an integration clamp. A planned
  // or actual duration above the historical 100ms range remains fully counted.
  out.width_supported=in.planned_ms>0 && in.planned_ms<=lateral_energy::kMaxMs &&
      in.actual_elapsed_ms<=lateral_energy::kMaxMs;
  if((in.side!=1 && in.side!=-1) || !std::isfinite(in.rate_dps) || in.rate_dps<0 ||
      !std::isfinite(in.initial_current_ma) || !std::isfinite(in.goal_current_ma) || in.goal_current_ma<=0 ||
      !std::isfinite(in.tau_s) || in.tau_s<=0 || !std::isfinite(in.available_j) || in.available_j<0 ||
      !std::isfinite(in.request_j) || in.request_j<0)return out;
  const auto c=lateral_energy::coefficients(in.side);
  out.rate_supported=in.rate_dps>=c.rate_min_dps && in.rate_dps<=c.rate_max_dps;
  const double seconds=static_cast<double>(in.actual_elapsed_ms)*.001;
  const double tail=std::exp(-seconds/in.tau_s);
  const double rise=-std::expm1(-seconds/in.tau_s);
  const double q=in.goal_current_ma*seconds+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*in.tau_s*rise;
  out.elapsed_s=static_cast<float>(seconds);
  out.q_mas=in.actual_elapsed_ms?static_cast<float>(q):0.f;
  out.current_end_ma=in.actual_elapsed_ms?static_cast<float>(in.goal_current_ma+
      (static_cast<double>(in.initial_current_ma)-in.goal_current_ma)*tail):in.initial_current_ma;
  if(!std::isfinite(out.q_mas) || !std::isfinite(out.current_end_ma))return out;
  out.valid=true;
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.width_supported || !out.rate_supported || !out.charge_supported;
  if(out.q_mas<0){out.extrapolated=true;return out;}
  out.work_j=in.actual_elapsed_ms?
      lateral_energy::work(c,in.rate_dps*lateral_energy::kRadPerDeg,out.q_mas):0.f;
  if(!std::isfinite(out.work_j) || out.work_j<0){out.work_j=NAN;out.extrapolated=true;return out;}
  out.arrival_j=in.available_j+out.work_j;
  out.unmet_j=std::fmax(0.f,in.request_j-out.work_j);
  if(!std::isfinite(out.arrival_j) || !std::isfinite(out.unmet_j)){
    out.work_j=out.arrival_j=out.unmet_j=NAN;out.extrapolated=true;return out;
  }
  out.work_valid=true;return out;
}
}
