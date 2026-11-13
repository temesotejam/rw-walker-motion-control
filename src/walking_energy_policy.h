#pragma once
#include "lateral_energy_control.h"

// Energy-domain walking input selection, separate from the old +/-ms peak
// adaptation. U is the nominal rocker potential. Passive-rate and inherited
// pulse-Q coefficients are effective ARRIVAL-energy surrogates; they are
// not measured body inertia,
// actuator efficiency, or a validated moving-foot contact model.
namespace walking_energy_policy {
constexpr char kRevision[]="walking_peak_rate_energy_v14";
constexpr uint16_t kMaxPulseMs=80;
constexpr float kCommonPeakFeedbackGain=.25f;
constexpr float kRedistributionGain=.125f;
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
enum class Reason : uint8_t { Invalid,Coast,Drive,CapLimited };
struct Input {
  int8_t side=0; // Intended NEXT body peak side; rate is a magnitude.
  float rate_dps=NAN,body_angle_deg=NAN,target_deg=NAN;
  float previous_peak_deg=NAN,same_side_peak_deg=NAN; // Peak amplitudes >=0.
  // Signed relative to intended excitation. Never abs() a residual current.
  float initial_current_ma=NAN,goal_current_ma=NAN;
  float extra_energy_j=0;
  uint16_t cap_ms=kMaxPulseMs;
};
struct Decision {
  bool valid=false,selected=false,coast=false,saturated=false,extrapolated=false;
  bool rate_supported=false,charge_supported=false,early_state_transfer=false;
  bool passive_rate_supported=false,work_rate_supported=false,passive_intercept_rolloff=false;
  bool coast_residual_modelled=false,extra_energy_clamped=false;
  Reason reason=Reason::Invalid;
  uint16_t width_ms=0,evaluations=0;
  float peak_energy_j=NAN,same_side_peak_energy_j=NAN;
  float mean_peak_energy_j=NAN,side_imbalance_j=NAN;
  float common_correction_j=0,redistribution_j=0;
  float potential_j=NAN,effective_kinetic_j=NAN,fitted_offset_j=NAN;
  float passive_omega2_coefficient=NAN;
  float available_j=NAN,target_j=NAN,peak_correction_j=0,extra_energy_j=0;
  float deficit_j=NAN,request_j=NAN,q_mas=0,predicted_work_j=0;
  float predicted_arrival_j=NAN,unmet_j=NAN,maximum_work_j=NAN;
  float coast_residual_q_mas=NAN,inbound_loss_proxy_j=NAN;
};
inline Decision decide(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in){
  Decision out;
  if((in.side!=1 && in.side!=-1) || !current.ready || !in.cap_ms || in.cap_ms>kMaxPulseMs ||
     !std::isfinite(in.rate_dps) || in.rate_dps<=0 || !std::isfinite(in.body_angle_deg) ||
     !std::isfinite(in.target_deg) || in.target_deg<=0 ||
     !std::isfinite(in.previous_peak_deg) || in.previous_peak_deg<0 ||
     (!std::isnan(in.same_side_peak_deg) && (!std::isfinite(in.same_side_peak_deg) || in.same_side_peak_deg<0)) ||
     !std::isfinite(in.initial_current_ma) || !std::isfinite(in.goal_current_ma) || in.goal_current_ma<=0 ||
     !std::isfinite(in.extra_energy_j) || in.extra_energy_j<0)return out;
  const auto c=lateral_energy::coefficients(in.side);
  const auto passive=passiveCoefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  out.target_j=lateral_energy::potential(geometry,in.target_deg);
  out.potential_j=lateral_energy::potential(geometry,in.body_angle_deg);
  out.peak_energy_j=lateral_energy::potential(geometry,in.previous_peak_deg);
  if(!std::isfinite(out.target_j) || out.target_j<=0 || !std::isfinite(out.potential_j) ||
     !std::isfinite(out.peak_energy_j))return out;
  // Positive-rate callers include slow stall-rescue proposals; their detector
  // floors belong to the caller. Low-speed predictions remain extrapolations.
  // Low-rate walking fit throughout the current range, with out-of-domain
  // flags. Do not blend to the inconsistent old passive fit at35..45dps:
  // doing so decreases predicted energy when angular speed increases.
  out.passive_omega2_coefficient=passive.omega2_j;
  out.effective_kinetic_j=passive.omega2_j*omega*omega;
  const float scale=std::fmin(1.f,in.rate_dps/20.f);
  out.fitted_offset_j=passive.offset_j*scale*scale;
  out.passive_intercept_rolloff=in.rate_dps<20.f; // Unvalidated static-end rolloff.
  out.available_j=out.potential_j+out.effective_kinetic_j+out.fitted_offset_j;
  out.inbound_loss_proxy_j=std::fmax(0.f,out.peak_energy_j-out.available_j);
  // This is an incoming-to-predicted-arrival discrepancy, not observed
  // mechanical dissipation. Do not subtract it again from the same surrogate.
  // Paired completed peaks describe cycle mean and side imbalance. They are
  // successive observations, not simultaneous independent energy reservoirs.
  // Common correction changes total excitation; redistribution changes sign
  // when the two side observations are exchanged and therefore sums to zero.
  if(std::isfinite(in.same_side_peak_deg)){
    out.same_side_peak_energy_j=lateral_energy::potential(geometry,in.same_side_peak_deg);
    if(!std::isfinite(out.same_side_peak_energy_j))return out;
    out.mean_peak_energy_j=.5f*(out.same_side_peak_energy_j+out.peak_energy_j);
    out.side_imbalance_j=.5f*(out.peak_energy_j-out.same_side_peak_energy_j);
    const float limit=.125f*out.target_j;
    const float common=kCommonPeakFeedbackGain*(out.target_j-out.mean_peak_energy_j);
    const float redistribution=kRedistributionGain*out.side_imbalance_j;
    out.common_correction_j=std::fmax(-limit,std::fmin(limit,common));
    out.redistribution_j=std::fmax(-limit,std::fmin(limit,redistribution));
    out.peak_correction_j=out.common_correction_j+out.redistribution_j;
  }
  out.extra_energy_j=std::fmin(in.extra_energy_j,.5f*out.target_j);
  out.extra_energy_clamped=out.extra_energy_j!=in.extra_energy_j;
  out.deficit_j=out.target_j+out.peak_correction_j+out.extra_energy_j-out.available_j;
  out.request_j=std::fmax(0.f,out.deficit_j);
  out.coast_residual_q_mas=in.initial_current_ma*kCoastFallTauS;
  // The inherited response was fitted to pulse-only Q, with pulse/coast effects
  // folded into its arrival response. Adding a fresh tail to its Q would
  // silently change that fit. Log the signed pre-existing coast residual as a
  // model gap; no zero-current or measured-work claim follows from width=0.
  out.coast_residual_modelled=false;
  out.passive_rate_supported=in.rate_dps>=passive.rate_min_dps && in.rate_dps<=passive.rate_max_dps;
  out.work_rate_supported=in.rate_dps>=c.rate_min_dps && in.rate_dps<=c.rate_max_dps;
  out.rate_supported=out.passive_rate_supported && out.work_rate_supported;
  out.early_state_transfer=std::fabs(in.body_angle_deg)>.01f;
  const float maximum_q=lateral_energy::charge(current,in.cap_ms,in.initial_current_ma,in.goal_current_ma);
  if(!std::isfinite(out.available_j) || !std::isfinite(out.request_j) || !std::isfinite(maximum_q))return out;
  out.maximum_work_j=maximum_q>0?lateral_energy::work(c,omega,maximum_q):0.f;
  if(!std::isfinite(out.maximum_work_j) || out.maximum_work_j<0)return out;
  out.predicted_arrival_j=out.available_j;
  float error=out.request_j; // The zero-width candidate: no new pulse work.
  // W(Q) is monotone for Q>=0. Find the crossing then compare nearby integer
  // widths by ENERGY error. No fixed +/-ms rule sets the selected command.
  uint16_t lo=0,hi=in.cap_ms;
  if(out.request_j>0){
    while(hi-lo>1){
      const uint16_t mid=static_cast<uint16_t>((lo+hi)/2);
      const float q=lateral_energy::charge(current,mid,in.initial_current_ma,in.goal_current_ma);
      ++out.evaluations;
      if(q<0 || lateral_energy::work(c,omega,q)<out.request_j)lo=mid;else hi=mid;
    }
    const uint16_t candidates[3]={lo,hi,in.cap_ms};
    for(uint16_t width:candidates){
      const float q=lateral_energy::charge(current,width,in.initial_current_ma,in.goal_current_ma);
      ++out.evaluations;
      if(q<0 || !std::isfinite(q))continue;
      const float work=lateral_energy::work(c,omega,q);
      if(!std::isfinite(work) || work<0)continue;
      const float candidate_error=std::fabs(work-out.request_j);
      if(candidate_error<error || (candidate_error==error && width<out.width_ms)){
        error=candidate_error;out.width_ms=width;out.q_mas=q;
        out.predicted_work_j=work;out.predicted_arrival_j=out.available_j+work;
      }
    }
  }
  out.unmet_j=std::fmax(0.f,out.request_j-out.predicted_work_j);
  out.saturated=out.request_j>out.maximum_work_j+1e-9f;
  out.charge_supported=out.q_mas>=0 && out.q_mas<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.valid=out.selected=true;out.coast=out.width_ms==0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  return out;
}
// Re-evaluate the *actual* post-guard width with the identical energy model.
// This does not authorize a different command or relax any guard.
inline Decision evaluateWidth(const lateral_energy::Geometry& geometry,
    const lateral_energy::CurrentTable& current,const Input& in,uint16_t actual_width_ms){
  auto out=decide(geometry,current,in);
  if(!out.valid || actual_width_ms>in.cap_ms){out.valid=out.selected=false;out.reason=Reason::Invalid;return out;}
  const auto c=lateral_energy::coefficients(in.side);
  const float omega=in.rate_dps*lateral_energy::kRadPerDeg;
  const float q=actual_width_ms?lateral_energy::charge(current,actual_width_ms,in.initial_current_ma,in.goal_current_ma):0.f;
  const float work=actual_width_ms?lateral_energy::work(c,omega,q):0.f;
  if(!std::isfinite(q) || q<0 || !std::isfinite(work) || work<0){
    out.valid=out.selected=false;out.reason=Reason::Invalid;return out;
  }
  out.width_ms=actual_width_ms;out.q_mas=q;out.predicted_work_j=work;
  out.predicted_arrival_j=out.available_j+work;
  out.unmet_j=std::fmax(0.f,out.request_j-work);
  out.saturated=out.unmet_j>1e-9f;out.coast=actual_width_ms==0;
  out.charge_supported=q<=c.charge_max_mAs;
  out.extrapolated=!out.rate_supported || !out.charge_supported || out.early_state_transfer || out.extra_energy_j>0;
  out.reason=out.coast?Reason::Coast:out.saturated?Reason::CapLimited:Reason::Drive;
  return out;
}
}
