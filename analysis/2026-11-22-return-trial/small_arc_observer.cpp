// Host-only stdin adapter for the actual left-return observer, scoped to run 195727101.
#define SUPPORT_TRANSFER_V39 1
#define SUPPORT_TRANSFER_CARRY2 1
#define SUPPORT_TRANSFER_STALL_V3 1
#include "return_observation_policy.h"
#include <iomanip>
#include <iostream>

int main() {
  using namespace return_observation_policy;
  Observer observer;
  Input in;
  periodic_foot::Packet packet;
  if (!(std::cin >> in.run_id >> packet.epoch_us >> in.peak_ms >> in.peak_deg >> in.target_j)) return 1;
  in.active = true;  // Isolate the observer, assuming the control mode and pulse ledger admit an aid.
  in.side = 1;
  in.periodic = &packet;
  auto& frame = in.foot;
  frame.run_id = in.run_id;
  // The Python reader checks these flags against each archived image.
  frame.timestamp_valid = frame.frame_valid = frame.zero_ready = true;
  frame.left_detected = frame.right_detected = frame.left_range_ok = frame.right_range_ok = true;
  while (std::cin >> in.now) {
    if (!(std::cin >> in.sample_us >> in.sample_age_us >> in.angle_deg >> in.rate_dps
          >> in.periodic_acceleration_dps2 >> in.fore_aft_deg
          >> packet.count >> packet.scheduled_us >> packet.published_us
          >> packet.left.sequence >> packet.left.image_us >> packet.left.measured >> packet.right.measured
          >> frame.sequence >> frame.image_us >> frame.ready_ms >> frame.left_deg >> frame.right_deg)) return 2;
    packet.right.sequence = packet.left.sequence;
    packet.right.image_us = packet.left.image_us;
    frame.frame_ms = (frame.image_us - packet.epoch_us) / 1000;
    const auto result = observer.observe(in);
    std::cout << std::setprecision(9) << in.now << ' ' << result.selected << ' '
              << static_cast<unsigned>(result.evidence) << ' ' << result.request_j * 1.e6f << '\n';
  }
}
