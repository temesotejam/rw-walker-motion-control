// Host adapter for the production left-return history, work inverse and wheel guard.
#include "half_cycle_energy_budget.h"
#include "receiving_arc_aid.h"
#include "wheel_cycle_guard.h"
#include <algorithm>
#include <iomanip>
#include <iostream>

int main() {
  receiving_arc_aid::ReturnHistory history;
  receiving_arc_aid::ReturnState state;
  wheel_cycle_guard::Controller wheel;
  lateral_energy::Geometry geometry;
  std::cout << std::setprecision(9);
  char cmd;
  while (std::cin >> cmd) {
    if (cmd == 'P') {
      unsigned t, departure, zero;
      int side, matched, powered;
      float angle;
      std::cin >> t >> side >> angle >> powered >> departure >> zero >> matched;
      wheel.peak(side, angle, t, powered);
      history.peak(side, t, angle, departure, zero, matched);
    } else if (cmd == 'W') {
      unsigned t, width;
      int side;
      float rpm;
      std::cin >> t >> side >> rpm >> width;
      wheel.limit(side, rpm, true, t, width);
    } else if (cmd == 'C') {
      unsigned end, elapsed;
      int qualified;
      std::cin >> end >> elapsed >> qualified;
      history.completedAid(state, end, elapsed, qualified);
    } else if (cmd == 'D') {
      unsigned t, cap, baseline, requested, reserve, envelope;
      float original_j, tau, rpm;
      walking_return_energy::Input in;
      std::cin >> t >> state.departure_ms >> state.image_age_ms >> state.body_deg
               >> state.speed_dps >> state.left_deg >> state.right_deg >> state.fore_aft_deg
               >> state.deceleration >> state.terminal_deg >> state.target_j;
      std::cin >> in.target_deg >> in.previous_peak_deg >> in.initial_current_ma
               >> in.goal_current_ma >> tau >> rpm >> cap >> baseline >> requested >> reserve
               >> envelope >> original_j;
      in.side = -1;
      in.body_angle_deg = state.body_deg;
      in.rate_dps = state.speed_dps;
      in.cap_ms = cap;
      in.return_work_j = original_j;
      lateral_energy::CurrentTable current;
      if (!current.prepare(tau)) return 1;
      const auto comparison = history.compare(state, t);
      const auto old = half_cycle_energy_budget::decideReturnAid(geometry, current, in);
      auto preview = wheel;
      const unsigned old_guard = preview.limit(-1, rpm, true, t, requested);
      unsigned inverse_width = requested, guarded = old_guard, final_width = baseline;
      if (comparison.request_j > original_j && comparison.limit_ms > baseline) {
        in.return_work_j = comparison.request_j;
        const auto candidate = half_cycle_energy_budget::decideReturnAid(geometry, current, in);
        if (candidate.valid && candidate.width_ms > baseline) {
          inverse_width = candidate.width_ms;
          preview = wheel;
          guarded = preview.limit(-1, rpm, true, t, inverse_width);
          const auto bound = receiving_arc_aid::boundWidth(baseline, guarded, envelope, reserve);
          const auto width = std::min(bound, comparison.limit_ms);
          const auto result = half_cycle_energy_budget::evaluateReturnAidWidth(geometry, current, in, width);
          if (width > baseline && result.valid && result.charge_supported && result.predicted_work_j > 0 &&
              result.predicted_work_j <= .5f * result.target_j)
            final_width = width;
        }
      }
      if (final_width == baseline) in.return_work_j = original_j;
      const auto final = half_cycle_energy_budget::evaluateReturnAidWidth(geometry, current, in, final_width);
      std::cout << t << ' ' << comparison.first_peak_ms << ' ' << comparison.second_peak_ms << ' '
                << comparison.deceleration << ' ' << comparison.request_j << ' ' << comparison.limit_ms
                << ' ' << old.width_ms << ' ' << old_guard << ' ' << inverse_width << ' '
                << guarded << ' ' << final_width << ' ' << final.predicted_work_j << '\n';
    } else return 2;
    if (!std::cin) return 3;
  }
}
