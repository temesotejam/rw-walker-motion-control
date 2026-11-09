#pragma once
#include "walking_energy_loss.h"

// A model-coverage miss is not itself a hardware failure. In that case the
// negative half-cycle may coast naturally; this makes no arrival prediction
// and must never bypass the normal motor/IMU/emergency/angle safety checks.
namespace unmodeled_walk_coast {
inline bool eligible(const wheel_work_energy_control::State& s,float target_mj,
                     bool physical_safety_ok) {
  return physical_safety_ok && s.absolute_reference && s.target_deg==8.f &&
      wheel_work_energy_control::freshFinite(s) && s.rate_dps>0.f &&
      s.battery_mv>0.f && std::isfinite(s.previous_peak_deg) && s.previous_peak_deg>0.f &&
      std::isfinite(target_mj) && target_mj>0.f &&
      !walking_energy_loss::supported(s) && !walking_energy_loss::lowSpeedSupported(s) &&
      wheel_work_energy_control::negativeBoundaryWeight(s)<=0.f;
}
}
