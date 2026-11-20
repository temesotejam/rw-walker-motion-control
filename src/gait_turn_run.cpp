#include "gait_turn_run.h"
#if WALKING_TURN_OBSERVER
#include "foot_fast_policy.h"
#include "psram_string.h"

void GaitTurnRun::peak(int side, uint32_t confirmed_ms, float yaw_deg) {
  if (!status_.enabled || !observer_.peak(side, confirmed_ms, yaw_deg)) return;
  const auto& result = observer_.latest();
  status_.latest = result;
  if (result.reason == gait_turn::Reason::Confirmed) {
    status_.confirmed_ms = result.end_ms;
    status_.confirmed_side = result.weak_side;
  }
  ++total_;
  if (rows_.available() && stored_ < capacity) rows_[stored_++] = result;
}

void GaitTurnRun::append(PsramString& json) const {
  // Keep the existing headroom for the support/forecast tables and footer.
  const size_t stop = foot_fast_policy::kSupportAuditEndBytes - 4096;
  if (!json.ok() || json.length() + 512 >= stop) return;
  json += ",\"gait_turn\":{\"revision\":2,\"enabled\":";
  json += status_.enabled ? "true" : "false";
  json += ",\"archive_available\":";
  json += rows_.available() ? "true" : "false";
  json += ",\"total\":" + String(total_) + ",\"stored\":" + String(stored_);
  json += ",\"confirmed_ms\":" + String(status_.confirmed_ms) +
          ",\"confirmed_side\":" + String(static_cast<int>(status_.confirmed_side));
  json += ",\"columns\":[\"start_ms\",\"mid_ms\",\"end_ms\",\"samples\",\"left_deg\",\"right_deg\","
          "\"left_swing_deg\",\"right_swing_deg\",\"yaw_delta_deg\",\"pair_yaw_delta_deg\","
          "\"weak_side\",\"turn_sign\",\"streak\",\"reason\",\"paired_swing_asymmetry\"],\"rows\":[";
  const auto number = [](float value) {
    return std::isfinite(value) ? String(value, 4) : String("null");
  };
  uint16_t exported = 0;
  for (; exported < stored_; ++exported) {
    const auto& r = rows_[exported];
    String row = String(exported ? ",[" : "[") + r.start_ms + "," + r.mid_ms + "," + r.end_ms + "," + r.samples;
    row += "," + number(r.left_deg) + "," + number(r.right_deg) + "," + number(r.left_swing_deg);
    row += "," + number(r.right_swing_deg) + "," + number(r.yaw_delta_deg) + "," + number(r.pair_yaw_delta_deg);
    row += "," + String(static_cast<int>(r.weak_side)) + "," + String(static_cast<int>(r.turn_sign));
    row += "," + String(static_cast<unsigned>(r.streak)) + ",\"" + gait_turn::reasonName(r.reason) + "\",";
    row += number(r.paired_swing_asymmetry) + "]";
    if (json.length() + row.length() + 32 >= stop) break;
    json += row;
  }
  json += "],\"exported\":" + String(exported) + "}";
}
#endif
