// Replay only the production observation policies. This process never
// constructs a motor transport and never uses samples beyond each decision.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#define WALKING_RETURN_HISTORY 1
#include "return_observation_policy.h"
#include "half_cycle_energy_budget.h"
#include "settled_walking_restart.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string token;
  std::cin >> token;
  return std::stof(token);
}

int main() {
  return_observation_policy::PostAidObserver observer;
  return_observation_policy::Input in;
  periodic_foot::Packet packet;
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable current_model;
  walking_return_energy::Input inverse;
  settled_walking_restart::Detector restart;
  char tag;
  std::cout << std::setprecision(9);
  while (std::cin >> tag) {
    if (tag == 'A') {
      uint32_t end;
      in = {}; packet = {}; observer = {};
      in.side = 1;
      in.run_id = in.foot.run_id = 1;
      std::cin >> in.peak_ms >> end >> packet.epoch_us;
      const float first_work = number();
      unsigned elapsed;
      std::cin >> elapsed;
      const float i0=number(), goal=number(), onset_rate=number(), nominal_work=number(), decel=number();
      in.target_j = number();
      in.current_rise_tau_s = number();
      inverse = {}; inverse.side = -1;
      inverse.target_deg=number(); inverse.previous_peak_deg=number(); inverse.same_side_peak_deg=number();
      if(!current_model.prepare(in.current_rise_tau_s)) return 1;
      // Walk6 accounts for the second pulse's actually observed starting
      // current. Reconstruct its modeled work for the measured elapsed time.
      const float q=lateral_energy::charge(current_model, elapsed, i0, goal);
      const float post_work=std::isfinite(nominal_work) ? nominal_work :
          lateral_energy::work(lateral_energy::coefficients(-1),onset_rate*lateral_energy::kRadPerDeg,q);
      observer.armRecheck(in.peak_ms, end, first_work+post_work, decel);
    } else if (tag == 'I') {
      uint64_t now;
      std::cin >> now >> in.now >> in.sample_us >> in.sample_age_us;
      in.angle_deg = number(); in.rate_dps = number();
      in.periodic = nullptr;
      observer.observeRecheck(in, now, NAN);
    } else if (tag == 'S') {
      uint64_t now;
      std::cin >> now >> in.now >> in.sample_us >> in.sample_age_us >> in.active
               >> packet.count >> packet.scheduled_us >> packet.published_us;
      for (auto* foot : {&packet.left, &packet.right}) {
        std::cin >> foot->sequence >> foot->image_us;
        foot->measured = number(); foot->estimated = number();
      }
      unsigned flags;
      auto& f = in.foot;
      std::cin >> flags >> f.frame_ms >> f.ready_ms;
      f.timestamp_valid = flags & 1; f.frame_valid = flags & 2; f.zero_ready = flags & 4;
      f.left_detected = flags & 8; f.right_detected = flags & 16;
      f.left_range_ok = flags & 32; f.right_range_ok = flags & 64;
      f.left_deg = number(); f.right_deg = number();
      in.angle_deg = number(); in.rate_dps = number();
      in.fore_aft_deg = number(); in.periodic_acceleration_dps2 = number();
      const float current = number();
      const float goal = number();
      unsigned guarded_cap;
      std::cin >> guarded_cap;
      in.periodic = &packet;
      const auto result = observer.observeRecheck(in, now, current);
      if (result.selected && guarded_cap > 0) {
        inverse.body_angle_deg = in.angle_deg; inverse.rate_dps = -in.rate_dps;
        inverse.initial_current_ma = current; inverse.goal_current_ma = goal;
        inverse.return_work_j=result.request_j;
        inverse.cap_ms=std::min<unsigned>(guarded_cap, result.loss_cap_ms);
        const auto selected=half_cycle_energy_budget::selectPostAid(geometry,current_model,inverse,result.gross_work_limit_j);
        const float extra=selected.valid ? selected.predicted_work_j-half_cycle_energy_budget::postAidCoastWork(
            inverse.rate_dps,current,selected.width_ms) : 0;
        std::cout << "R " << in.peak_ms << ' ' << in.now << ' ' << result.request_j << ' '
                  << result.loss_cap_ms << ' ' << in.angle_deg << ' ' << -in.rate_dps
                  << ' ' << selected.width_ms << ' ' << extra << '\n';
      }
    } else if (tag == 'D') {
      settled_walking_restart::Input s;
      std::cin >> s.now_ms >> s.sample_us >> s.run_active >> s.guarded >> s.feet_fresh
               >> s.pulse_active >> s.had_peak >> s.crossed_zero;
      s.angle_deg = number(); s.rate_dps = number();
      s.fore_aft_deg = number(); s.fore_aft_rate_dps = number();
      s.left_deg = number(); s.right_deg = number();
      bool hardware;
      std::cin >> hardware;
      const float measured = number(), goal = number(), tau = number();
      const int direction = restart.observe(s);
      if (direction && hardware) {
        if (!current_model.prepare(tau)) return 4;
        const float reference = restart.referenceSpeed(direction);
        const auto result = settled_walking_restart::plan(current_model, direction, reference,
            s.rate_dps, direction*measured, goal, restart.remaining());
        if (!result.width) continue;
        std::cout << "D " << s.now_ms << ' ' << direction << ' ' << s.angle_deg << ' '
                  << s.rate_dps << ' ' << s.left_deg << ' ' << s.right_deg << ' ' << reference
                  << ' ' << result.width << ' ' << result.demand_j << ' ' << result.predicted_j << '\n';
        return 0;  // Changes the commanded input. Later motion is counterfactual.
      }
    } else return 2;
    if (!std::cin) return 3;
  }
}
