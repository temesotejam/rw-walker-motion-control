// Thin host adapter for the observer used by the firmware. Times are in the
// original hardware clock and no recorded decision is used as an input.
#include "gait_turn_observer.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string value;
  std::cin >> value;
  return std::stof(value);
}

static void readSide(periodic_foot::Side& side) {
  std::cin >> side.sequence >> side.image_us;
  side.measured = number();
  side.estimated = number();
}

int main() {
  gait_turn::Observer observer;
  observer.reset();
  char kind;
  while (std::cin >> kind) {
    if (kind == 'P') {
      periodic_foot::Packet packet;
      std::cin >> packet.count >> packet.epoch_us >> packet.published_us;
      readSide(packet.left);
      readSide(packet.right);
      observer.observe(packet);
    } else if (kind == 'K') {
      int side;
      uint32_t known_ms;
      std::cin >> side >> known_ms;
      const float heading = number();
      if (!observer.peak(side, known_ms, heading)) continue;
      const auto& result = observer.latest();
      std::cout << std::setprecision(7)
                << result.start_ms << ' ' << result.mid_ms << ' ' << result.end_ms << ' '
                << result.samples << ' ' << result.left_deg << ' ' << result.right_deg << ' '
                << result.left_swing_deg << ' ' << result.right_swing_deg << ' ' << result.yaw_delta_deg
                << ' ' << result.pair_yaw_delta_deg
                << ' ' << int(result.weak_side) << ' ' << int(result.turn_sign) << ' '
                << int(result.streak) << ' ' << int(result.reason) << '\n';
    } else {
      return 1;
    }
    if (!std::cin) return 2;
  }
}
