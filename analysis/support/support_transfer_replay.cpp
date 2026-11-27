// PC adapter for the production estimator and event-horizon solver.
#include "periodic_foot_state.h"
#include "support_transfer_plan.h"
#include <iomanip>
#include <iostream>

int main() {
  uint64_t epoch;
  float tau;
  if (!(std::cin >> epoch >> tau)) return 1;
  periodic_foot::Estimator foot;
  foot.start(epoch);
  lateral_energy::CurrentTable current;
  if (!current.prepare(tau)) return 1;
  std::cout << std::setprecision(9);
  char op;
  while (std::cin >> op) {
    if (op == 'F') {
      uint64_t receipt, image;
      uint32_t sequence;
      bool clock, lv, la, rv, ra;
      float left, right;
      std::cin >> receipt >> sequence >> image >> clock >> lv >> la >> left >> rv >> ra >> right;
      foot.observe(sequence, image, receipt, clock, lv, la, left, rv, ra, right);
    } else if (op == 'T') {
      uint64_t tick;
      std::cin >> tick;
      if (!foot.publish(tick)) return 1;
    } else if (op == 'D') {
      support_transfer_plan::Input in{};
      std::cin >> in.side >> in.angle_deg >> in.rate_dps >> in.accel_dps2 >> in.fore_aft_deg
               >> in.initial_current_ma >> in.goal_current_ma >> in.cap_ms;
      const auto& p = foot.packet;
      in.left_deg = p.left.estimated;
      in.right_deg = p.right.estimated;
      in.left_rate_dps = p.left.rate_dps;
      in.right_rate_dps = p.right.rate_dps;
      const auto d = support_transfer_plan::select(current, in);
      std::cout << "D " << int(d.reason) << ' ' << d.delay_ms << ' ' << d.width_ms << ' '
                << d.transfer_ms << ' ' << d.transfer_deg << ' ' << d.coast_margin_j << ' '
                << d.margin_j << ' ' << d.work_j << ' ' << d.work_supported << ' '
                << p.left.sequence << ' ' << p.right.sequence << '\n';
    } else if (op == 'G') {
      float l, r, fore, sway;
      std::cin >> l >> r >> fore >> sway;
      std::cout << "G " << support_transfer_plan::supportGap(l, r, fore, sway) << '\n';
    } else {
      return 1;
    }
    if (!std::cin) return 1;
  }
  return std::cin.eof() ? 0 : 1;
}
