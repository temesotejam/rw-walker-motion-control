#pragma once
#include <cmath>
#include <cstdint>
// Shared by the UI and the actual Start command. An angle-looking nominal
// value before zero lock must not authorize this foot-dependent experiment.
namespace foot_support_start {
template<class Snapshot> bool ready(const Snapshot& s,uint64_t now){
  const auto& f=s.latest;
  return s.available && s.zero_ready && f.zero_ready && f.frame_valid && f.timestamp_valid &&
      f.left_valid && f.right_valid && f.left_in_range && f.right_in_range &&
      f.frame_us && f.frame_us<=now && now-f.frame_us<=200000 &&
      std::isfinite(f.left_deg) && std::isfinite(f.right_deg);
}
}
