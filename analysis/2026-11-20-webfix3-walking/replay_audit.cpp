// Replay archived worker snapshots, including gated observations used for history.
#include "support_transfer_plan.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string field;
  std::cin >> field;
  return std::stof(field);  // Null observations are sent as "nan".
}

int main() {
  using namespace support_transfer_plan;
  lateral_energy::CurrentTable current;
  if (!current.prepare(number())) return 1;
  Progress progress{};
  uint32_t peak, packet, gate;
  std::cout << std::setprecision(9);
  while (std::cin >> peak >> packet >> gate) {
    Input in{};
    std::cin >> in.side;
    in.angle_deg = number();
    in.rate_dps = number();
    in.accel_dps2 = number();
    in.fore_aft_deg = number();
    in.left_deg = number();
    in.right_deg = number();
    in.left_rate_dps = number();
    in.right_rate_dps = number();
    in.initial_current_ma = number();
    in.goal_current_ma = number();
    std::cin >> in.cap_ms;
    progress.observe(peak, packet, in, (gate & 7u) == 0);
    Decision old{}, next{};
    if (!(gate & ~256u)) {
      old = select(current, in);
      next = select(current, in, progress);
    }
    std::cout << int(old.reason) << ' ' << old.delay_ms << ' ' << old.width_ms
              << ' ' << int(next.phase) << ' ' << int(next.anchor)
              << ' ' << int(next.reason) << ' ' << next.delay_ms << ' ' << next.width_ms
              << ' ' << next.work_supported << ' ' << next.terminal_deg
              << ' ' << next.terminal_ms << ' ' << next.coast_margin_j << ' ' << next.margin_j << '\n';
  }
  return std::cin.eof() ? 0 : 1;
}
