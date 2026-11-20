#define WALKING_TURN_OBSERVER 1
#include "gait_turn_energy.h"
#include "wheel_cycle_guard.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() { std::string token; std::cin >> token; return std::stof(token); }

int main() {
  const lateral_energy::Geometry geometry;
  wheel_cycle_guard::Controller wheel;
  gait_turn::Status gait;
  gait.enabled = true;
  char event;
  while (std::cin >> event) {
    if (event == 'K') {
      int side, matched;
      uint32_t peak;
      std::cin >> side >> peak >> matched;
      wheel.peak(side, number(), peak, matched);
    } else if (event == 'C') {
      auto& cycle = gait.latest;
      int weak, turn, reason;
      std::cin >> cycle.start_ms >> cycle.end_ms >> reason >> weak >> turn;
      cycle.reason = static_cast<gait_turn::Reason>(reason);
      cycle.weak_side = weak;
      cycle.turn_sign = turn;
      cycle.left_swing_deg = number();
      cycle.right_swing_deg = number();
    } else if (event == 'E') {
      // The original command, even if replay selected a different pulse. Each
      // new decision is conditional on the physical state that actually occurred.
      int side, width;
      uint32_t t;
      std::cin >> side >> t >> width;
      wheel.limit(side, number(), true, t, width);
    } else if (event == 'M') {
      walking_return_energy::Input input;
      int side, eligible;
      uint32_t t;
      std::cin >> side >> t >> eligible >> input.cap_ms;
      input.side = side;
      input.rate_dps = number();
      input.body_angle_deg = number();
      input.target_deg = number();
      input.previous_peak_deg = number();
      input.same_side_peak_deg = number();
      input.initial_current_ma = number();
      input.goal_current_ma = number();
      input.arrival_bias_j = number();
      lateral_energy::CurrentTable table;
      if (!table.prepare(number())) return 3;
      const float rpm = number();
      const auto normal = walking_return_energy::decide(geometry, table, input);
      auto base_guard = wheel;
      const auto old_width = normal.coast ? 0 : base_guard.limit(side, rpm, true, t, normal.width_ms);
      const auto base = walking_return_energy::evaluateWidth(geometry, table, input, old_width);
      const float delta = eligible ? gait_turn_energy::adjustment(gait, side, t, input, base, geometry, table) : 0;
      auto actual = base, proposed = base;
      if (delta != 0) {
        input.turn_adjustment_j = delta;
        const auto proposal = walking_return_energy::decide(geometry, table, input);
        auto proposal_guard = wheel;
        const auto width = proposal_guard.limit(side, rpm, true, t, proposal.width_ms);
        proposed = walking_return_energy::evaluateWidth(geometry, table, input, width);
        if (gait_turn_energy::admissible(base, proposed, delta)) actual = proposed;
      }
      std::cout << std::setprecision(9) << t << ' ' << normal.width_ms << ' ' << old_width << ' '
                << base.request_j << ' ' << base.predicted_work_j << ' ' << delta << ' '
                << proposed.width_ms << ' ' << proposed.unmet_j << ' ' << proposed.predicted_work_j << ' ' << actual.width_ms
                << ' ' << actual.predicted_work_j << '\n';
    } else return 1;
    if (!std::cin) return 2;
  }
}
