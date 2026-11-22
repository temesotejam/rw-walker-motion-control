// Run both production observers on identical events. Stop at each observer's first
// decision; the ensuing body trajectory under a different input is unknown.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "return_observation_policy.h"
#include <iomanip>
#include <iostream>
#include <string>

static float number() {
  std::string token;
  std::cin >> token;
  return std::stof(token);
}

int main() {
  using namespace return_observation_policy;
  Observer observers[2];
  bool chosen[2]{};
  Input in;
  periodic_foot::Packet packet;
  in.periodic = &packet;
  in.side = 1;
  in.foot.run_id = in.run_id = 1;
  char kind;
  while (std::cin >> kind) {
    if (kind == 'K') {
      for (auto& observer : observers) observer.reset();
      chosen[0] = chosen[1] = false;
      packet = {};
      std::cin >> in.peak_ms >> packet.epoch_us;
      in.peak_deg = number();
      in.target_j = number();
      in.current_rise_tau_s = number();
    } else if (kind == 'F') {
      auto& f = in.foot;
      unsigned flags;
      std::cin >> flags >> f.sequence >> f.image_us >> f.received_us >> f.frame_ms >> f.ready_ms;
      f.timestamp_valid = flags & 1;
      f.frame_valid = flags & 2;
      f.zero_ready = flags & 4;
      f.left_detected = flags & 8;
      f.right_detected = flags & 16;
      f.left_range_ok = flags & 32;
      f.right_range_ok = flags & 64;
      f.left_deg = number();
      f.right_deg = number();
    } else if (kind == 'P') {
      std::cin >> packet.count >> packet.scheduled_us >> packet.published_us;
      for (auto side : {&packet.left, &packet.right}) {
        std::cin >> side->sequence >> side->image_us;
        side->measured = number();
      }
      in.fore_aft_deg = number();
    } else if (kind == 'I') {
      std::cin >> in.now >> in.sample_us >> in.sample_age_us >> in.active;
      in.angle_deg = number();
      in.rate_dps = number();
      in.periodic_acceleration_dps2 = number();
      for (int policy = 0; policy < 2; ++policy) {
        if (chosen[policy]) continue;
        in.predict_left_return = policy;
        auto result = observers[policy].observe(in);
        if (result.selected) {
          chosen[policy] = true;
          std::cout << in.peak_ms << ' ' << policy << ' ' << in.now << ' '
                    << static_cast<unsigned>(result.evidence) << ' ' << std::setprecision(9)
                    << result.request_j * 1.e6f << ' ' << packet.count << '\n';
        }
      }
    } else return 1;
    if (!std::cin) return 2;
  }
}
