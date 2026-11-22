#pragma once
#include "half_cycle_energy_budget.h"
#include "receiving_arc_aid.h"
#include "support_transfer_plan.h"
#include "wheel_cycle_guard.h"

namespace return_crossing_work {
struct Reference {
  uint32_t zero_ms=0, peak_ms=0;
  float speed_dps=0, target_j=0;
  void observe(uint32_t zero,uint32_t peak,float magnitude,float speed,float target,bool completed) {
    *this={};
    const auto fit=walking_energy_policy::passiveCoefficients(-1);
    // Flat feet give the largest absolute CAD entry angle. A peak beyond that
    // angle crossed the geometric threshold for any pitch in the model.
    if (completed && zero && peak>zero && peak-zero<=1500 &&
        std::isfinite(magnitude) && magnitude>=-support_transfer_plan::arcEntryDeg(-1,0) &&
        speed>=fit.rate_min_dps && speed<=fit.rate_max_dps && std::isfinite(target) && target>0) {
      zero_ms=zero;peak_ms=peak;speed_dps=speed;target_j=target;
    }
  }
};

struct Audit {
  uint32_t zero_ms=0, peak_ms=0;
  float speed_dps=0, request_j=0;
  uint16_t base_ms=0, final_ms=0;
  float transfer_ms=NAN, terminal_deg=NAN, arc_request_j=NAN;
};

inline bool usableReference(const Reference& source,uint32_t departure,uint32_t now,float target) {
  return source.zero_ms && source.peak_ms<departure && now>departure && now-departure<=1500 &&
      now-source.peak_ms<=4000 && std::isfinite(target) && std::fabs(source.target_j-target)<=1.e-9f;
}

inline float energyShortfall(float angle,float terminal,float speed,float residual,float decel) {
  const float rad=lateral_energy::kRadPerDeg;
  return walking_energy_policy::passiveCoefficients(-1).omega2_j*rad*rad*
      (2.f*decel*(angle-terminal)+residual*residual-speed*speed);
}

// Called only after current-foot geometry already qualified an actual arc
// extension. The source speed was measured before its ordinary ZERO pulse.
inline Audit apply(const Reference& source,uint32_t departure,uint32_t now,float decel,
    uint16_t envelope,uint16_t reserve,float rpm,const wheel_cycle_guard::Controller& wheel,
    wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width,
    uint16_t history_limit=UINT16_MAX) {
  Audit audit;
  if (!usableReference(source,departure,now,selected.target_j) || in.side!=-1 ||
      !(in.body_angle_deg>0 && in.body_angle_deg<=12 && in.rate_dps>=3 && in.rate_dps<=35 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        std::isfinite(decel) && decel>0 && std::isfinite(rpm)) ||
      !selected.valid || !width || width>=selected.width_ms || reserve>=envelope) return audit;
  audit.zero_ms=source.zero_ms;audit.peak_ms=source.peak_ms;audit.speed_dps=source.speed_dps;
  audit.base_ms=audit.final_ms=width;
  // Independent centre-speed boundary. Take the larger requirement for
  // reaching ZERO or the arc endpoint, since both cover the same approach.
  const float centre=energyShortfall(in.body_angle_deg,0,in.rate_dps,source.speed_dps,decel);
  audit.request_j=std::fmin(.5f*selected.target_j,std::fmax(0.f,centre));
  if (audit.request_j<=selected.request_j) return audit;
  auto proposed=in;
  proposed.return_work_j=audit.request_j;
  const auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,proposed);
  if (!inverse.valid || inverse.coast || inverse.width_ms<=selected.width_ms) return audit;
  auto fresh_guard=wheel;
  auto allowed=fresh_guard.limit(-1,rpm,true,now,inverse.width_ms);
  if (allowed>envelope-reserve) allowed=envelope-reserve;
  // One observed speed is a reference, not an identified optimum. Admit at
  // most the unguarded width already requested for today's certified arc.
  if (allowed>selected.width_ms) allowed=selected.width_ms;
  if (allowed>history_limit) allowed=history_limit;
  if (allowed<=width) return audit;
  const auto result=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,proposed,allowed);
  if (!result.valid || !result.charge_supported || !(result.predicted_work_j>0 &&
      result.predicted_work_j<=.5f*result.target_j)) return audit;
  selected=inverse;in=proposed;chosen_guard=fresh_guard;width=allowed;audit.final_ms=width;
  return audit;
}

#if SUPPORT_TRANSFER_STALL_V3
// Forecast using the deceleration that qualified the existing aid. Require a
// closing gap over both halves of the interval, ending on the receiving sole.
// Interpolate the time within the bracketing half; this is a predicted CAD
// height equality, not an observed impact or load measurement.
inline float closingTime(const receiving_arc_aid::Input& arc,float left,float right,float horizon,float fore_rate) {
  const auto gap=[&](float t) {
    const float l=left+arc.left_rate*t,r=right+arc.right_rate*t;
    const float fore=arc.fore_aft_deg+fore_rate*t;
    if (!(std::fabs(l)<=20 && std::fabs(r)<=20 && std::fabs(fore)<=10 && l>r)) return float(NAN);
    const float body=arc.body_deg-t*(arc.speed_dps-.5f*arc.decel_dps2*t);
    return support_transfer_plan::supportGap(l,r,fore,body);
  };
  const float first=gap(0),half=.5f*horizon,middle=gap(half),end=gap(horizon);
  if (!(first>middle && middle>end && end<0)) return NAN;
  if (first<=0) return 0;
  if (middle<=0) return half*first/(first-middle);
  return half+half*middle/(middle-end);
}

// Only an already selected, fresh left aid can use this forecast. Changes
// to the waveform begin at the old cutoff, so search for an arriving pivot
// during that original width plus one current-rise time constant. Never
// extend the assumed direction beyond the estimated passive stopping time.
inline Audit applyApproach(const receiving_arc_aid::Input& arc,const Reference& source,
    uint32_t departure,uint32_t now,uint16_t envelope,uint16_t reserve,float rpm,
    const wheel_cycle_guard::Controller& wheel,wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width,
    float fore_rate_dps) {
  Audit audit;
  if (!(arc.observed_shape && arc.side==-1 && in.side==-1 && now>departure && now-departure<=1500 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        arc.body_deg>0 && arc.body_deg<=12 && arc.speed_dps>=support_transfer_plan::kCarryDps &&
        arc.speed_dps<=35 && std::isfinite(arc.decel_dps2) && arc.decel_dps2>0 &&
        std::fabs(arc.fore_aft_deg)<=10 && std::isfinite(fore_rate_dps) &&
        std::fabs(arc.left_rate)<=80 && std::fabs(arc.right_rate)<=80 &&
        arc.left_deg-arc.right_deg>=3 && selected.valid && !selected.coast && width>0 &&
        current.ready && current.tau>0 && std::isfinite(current.tau) && std::isfinite(rpm) &&
        reserve<envelope && width<envelope-reserve)) return audit;
  if (!(support_transfer_plan::supportGap(arc.left_deg,arc.right_deg,arc.fore_aft_deg,arc.body_deg)>=0)) return audit;
  const float stop_s=arc.speed_dps/arc.decel_dps2;
  const float horizon=std::fmin(.001f*support_transfer_plan::kForecastLimitMs,
      std::fmin(stop_s,.001f*width+current.tau));
  const float measured=closingTime(arc,arc.left_deg,arc.right_deg,horizon,0);
  const float estimated=closingTime(arc,arc.left_est,arc.right_est,horizon,0);
  if (!(std::isfinite(measured) && std::isfinite(estimated))) return audit;
  // A changing fore-aft pitch can lift the receiving foot even while the
  // roll closes the fixed-pitch gap. Retain the later plausible boundary.
  const float pitched=closingTime(arc,arc.left_est,arc.right_est,horizon,fore_rate_dps);
  if (!std::isfinite(pitched)) return audit;
  const float transfer=std::fmax(pitched,std::fmax(measured,estimated));
  if (!(transfer<stop_s)) return audit;
  audit.transfer_ms=1000.f*transfer;
  // Of the frozen and moving-pitch forecasts, the receiving sole closest
  // to horizontal needs the greatest leftward travel to reach its arc.
  const float fixed_pitch=arc.left_est+transfer*arc.left_rate+arc.fore_aft_deg;
  const float moving_pitch=fixed_pitch+transfer*fore_rate_dps;
  const float closest_pitch=fixed_pitch*moving_pitch<=0 ? 0.f :
      std::fmin(std::fabs(fixed_pitch),std::fabs(moving_pitch));
  audit.terminal_deg=support_transfer_plan::arcEntryDeg(-1,closest_pitch);
  audit.base_ms=audit.final_ms=width;
  const float cap=.5f*selected.target_j;
  audit.arc_request_j=std::fmin(cap,std::fmax(0.f,energyShortfall(arc.body_deg,audit.terminal_deg,
      arc.speed_dps,support_transfer_plan::kCarryDps,arc.decel_dps2)));
  audit.request_j=audit.arc_request_j;
  if (usableReference(source,departure,now,selected.target_j)) {
    audit.zero_ms=source.zero_ms;audit.peak_ms=source.peak_ms;audit.speed_dps=source.speed_dps;
    const float centre=energyShortfall(arc.body_deg,0,arc.speed_dps,source.speed_dps,arc.decel_dps2);
    audit.request_j=std::fmin(cap,std::fmax(audit.arc_request_j,centre));
  }
  // The crossing and arc endpoint share the same approach. Use the larger
  // requirement once, then apply the actual wheel and main-reserve limits.
  if (!(audit.request_j>selected.request_j)) return audit;
  auto proposed=in;
  proposed.return_work_j=audit.request_j;
  const auto inverse=half_cycle_energy_budget::decideReturnAid(geometry,current,proposed);
  if (!inverse.valid || inverse.coast || inverse.width_ms<=width) return audit;
  auto candidate_guard=wheel;
  uint16_t allowed=candidate_guard.limit(-1,rpm,true,now,inverse.width_ms);
  if (allowed>envelope-reserve) allowed=envelope-reserve;
  if (allowed<=width) return audit;
  const auto result=half_cycle_energy_budget::evaluateReturnAidWidth(geometry,current,proposed,allowed);
  if (!result.valid || !result.charge_supported || !(result.predicted_work_j>0 && result.predicted_work_j<=cap))
    return audit;
  selected=inverse;in=proposed;chosen_guard=candidate_guard;width=allowed;audit.final_ms=width;
  return audit;
}
#endif
}
