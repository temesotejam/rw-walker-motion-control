// Evaluate the production v14 inverse using only a recorded dispatch snapshot.
#include "walking_energy_policy.h"
#include "wheel_cycle_guard.h"
#include "energy_admission_v19.h"
#include <iomanip>
#include <iostream>

int main() {
  lateral_energy::Geometry geometry;
  lateral_energy::CurrentTable current;
  wheel_cycle_guard::Controller wheel;
  std::cout << std::setprecision(9);
  char event;
  while (std::cin >> event) {
    if (event == 'P') {
      unsigned time;
      int side, matched;
      float amplitude;
      std::cin >> time >> side >> amplitude >> matched;
      wheel.peak(side, amplitude, time, matched);
    } else if (event == 'D') {
      unsigned time, cap, observed_request;
      int side, early, committed;
      float tau, rpm, same_peak;
      walking_energy_policy::Input input;
      std::cin >> time >> side >> early >> input.rate_dps >> input.body_angle_deg
               >> input.target_deg >> input.previous_peak_deg >> same_peak
               >> input.initial_current_ma >> input.goal_current_ma >> tau
               >> rpm >> cap >> observed_request >> committed;
      if (!std::cin || !current.prepare(tau)) return 1;
      input.side = side;
      input.same_side_peak_deg = same_peak < 0 ? NAN : same_peak;
      if (!early) input.body_angle_deg = 0;
      input.cap_ms = early && cap > 15 ? 15 : cap;
      const auto request = walking_energy_policy::decide(geometry, current, input);
      auto preview = wheel;
      const unsigned duration = request.coast ? 0 :
          preview.limit(side, rpm, true, time, request.width_ms);
      const auto pulse = walking_energy_policy::evaluateWidth(geometry, current, input, duration);
      const auto scope = early ? energy_admission_v19::Scope::SmallStepEarly
                               : energy_admission_v19::Scope::OrdinaryZero;
      const bool admitted = energy_admission_v19::assess(scope, pulse).allowed;
      std::cout << request.valid << ' ' << request.width_ms << ' ' << duration << ' '
                << admitted << ' ' << pulse.q_mas << ' ' << pulse.predicted_work_j << '\n';
      // Force the next snapshot to use the same wheel history as the recording.
      if (committed) wheel.limit(side, rpm, true, time, observed_request);
    } else return 2;
    if (!std::cin) return 3;
  }
}
