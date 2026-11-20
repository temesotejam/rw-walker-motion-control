#pragma once

#include "gait_turn_observer.h"
#include "walking_return_energy.h"

namespace gait_turn_energy {
// One-step identification trial on the observed right-foot/negative-yaw pattern.
// 0.20 deg is the size of the project's earlier side-target response trial.
// Matched historical commands were compared at differences of at least 5 ms.
// Stay inside that step and at most 10% of the already-guarded pulse time.
constexpr float equivalent_target_step_deg = .20f;
inline uint16_t widthStep(uint16_t baseline_ms) {
  const uint16_t tenth = baseline_ms / 10;
  return tenth < 5 ? tenth : 5;
}

inline float adjustment(const gait_turn::Status& state, int side, uint32_t now,
                        const walking_return_energy::Input& in,
                        const walking_return_energy::Decision& base,
                        const lateral_energy::Geometry& geometry,
                        const lateral_energy::CurrentTable& current) {
  const auto& cycle = state.latest;
  if (!state.enabled || cycle.reason != gait_turn::Reason::Confirmed || cycle.weak_side != 1 ||
      cycle.turn_sign != -1 || cycle.end_ms <= cycle.start_ms || now < cycle.end_ms ||
      now-cycle.end_ms > cycle.end_ms-cycle.start_ms || (side != -1 && side != 1) ||
      in.side != side || !base.valid || !base.selected || base.coast || !base.width_ms ||
      !base.work_rate_supported || !base.charge_supported ||
      !(base.request_j > 0 && base.predicted_work_j > 0) ||
      !(in.target_deg > equivalent_target_step_deg && in.target_deg <= 8.f) ||
      !std::isfinite(in.same_side_peak_deg)) return 0;
  // On the positive half only reduce stimulation if the previously observed
  // positive body peak already met its current target and the full request was covered.
  if (side == 1 && (!base.passive_rate_supported || in.same_side_peak_deg < in.target_deg ||
                   base.unmet_j > walking_energy_policy_v15::kEnergyRoundoffJ)) return 0;
  if (!(cycle.left_swing_deg > cycle.right_swing_deg && cycle.right_swing_deg >= 0)) return 0;
  const float asymmetry = std::isfinite(cycle.paired_swing_asymmetry) ? cycle.paired_swing_asymmetry :
      (cycle.left_swing_deg-cycle.right_swing_deg)/(cycle.left_swing_deg+cycle.right_swing_deg);
  if (!(asymmetry > 0 && asymmetry <= 1)) return 0;
  const float delta_u = lateral_energy::potential(geometry, in.target_deg - side*equivalent_target_step_deg)
                      - lateral_energy::potential(geometry, in.target_deg);
  if (!std::isfinite(delta_u) || side*delta_u >= 0) return 0;
  const uint16_t step = widthStep(base.width_ms);
  if (!step) return 0;
  const uint16_t edge = side > 0 ? base.width_ms-step :
      static_cast<uint16_t>(std::fmin(in.cap_ms, base.width_ms+step));
  const float q = lateral_energy::charge(current, edge, in.initial_current_ma, in.goal_current_ma);
  const float edge_work = lateral_energy::work(lateral_energy::coefficients(side),
      in.rate_dps*lateral_energy::kRadPerDeg, q);
  const float room = -side * (edge_work - base.predicted_work_j);
  if (!(q >= 0 && std::isfinite(edge_work) && room > 0)) return 0;
  // Anchor the allowed work movement at the command after the wheel guard.
  // An old clipped request may still exceed this whole interval; retain it.
  const float desired = base.predicted_work_j - side*asymmetry*std::fmin(std::fabs(delta_u), room);
  const float request_delta = desired - base.request_j;
  return -side*request_delta > 0 ? request_delta : 0;
}

inline bool admissible(const walking_return_energy::Decision& base,
                       const walking_return_energy::Decision& candidate, float adjustment_j) {
  if (!candidate.valid || !candidate.selected || candidate.coast || !candidate.width_ms ||
      !candidate.work_rate_supported || !candidate.charge_supported ||
      !(candidate.request_j > 0) || candidate.unmet_j > walking_energy_policy_v15::kEnergyRoundoffJ)
    return false;
  const float difference = candidate.predicted_work_j-base.predicted_work_j;
  const int change_ms = static_cast<int>(candidate.width_ms) - base.width_ms;
  return difference*adjustment_j > 0 &&
         (candidate.width_ms > base.width_ms) == (adjustment_j > 0) &&
         change_ms != 0 && std::abs(change_ms) <= widthStep(base.width_ms);
}
} // namespace gait_turn_energy
