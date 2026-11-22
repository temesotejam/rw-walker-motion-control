#pragma once
#include "half_cycle_energy_budget.h"
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
};

// Called only after current-foot geometry already qualified an actual arc
// extension. The source speed was measured before its ordinary ZERO pulse.
inline Audit apply(const Reference& source,uint32_t departure,uint32_t now,float decel,
    uint16_t envelope,uint16_t reserve,float rpm,const wheel_cycle_guard::Controller& wheel,
    wheel_cycle_guard::Controller& chosen_guard,
    const lateral_energy::Geometry& geometry,const lateral_energy::CurrentTable& current,
    walking_return_energy::Input& in,walking_return_energy::Decision& selected,uint16_t& width,
    uint16_t history_limit=UINT16_MAX) {
  Audit audit;
  if (!source.zero_ms || source.peak_ms>=departure || now<=departure || now-departure>1500 ||
      now-source.peak_ms>4000 || !std::isfinite(selected.target_j) ||
      std::fabs(source.target_j-selected.target_j)>1.e-9f || in.side!=-1 ||
      !(in.body_angle_deg>0 && in.body_angle_deg<=12 && in.rate_dps>=3 && in.rate_dps<=35 &&
        in.previous_peak_deg>in.target_deg && in.same_side_peak_deg>0 && in.same_side_peak_deg<in.target_deg &&
        std::isfinite(decel) && decel>0 && std::isfinite(rpm)) ||
      !selected.valid || !width || width>=selected.width_ms || reserve>=envelope) return audit;
  audit.zero_ms=source.zero_ms;audit.peak_ms=source.peak_ms;audit.speed_dps=source.speed_dps;
  audit.base_ms=audit.final_ms=width;
  // Independent centre-speed boundary. Take the larger requirement for
  // reaching ZERO or the arc endpoint, since both cover the same approach.
  const float rad=lateral_energy::kRadPerDeg;
  const float centre=walking_energy_policy::passiveCoefficients(-1).omega2_j*rad*rad*
      (2.f*decel*in.body_angle_deg+source.speed_dps*source.speed_dps-in.rate_dps*in.rate_dps);
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
}
