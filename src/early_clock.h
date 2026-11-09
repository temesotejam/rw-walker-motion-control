#pragma once
#include <cstdint>
namespace early_clock {
// Maximum bounded run-relative time; refuse negative or wrapped clocks.
constexpr uint64_t kMaxRunUs=600000000ULL;
inline bool relativeMs(uint64_t absolute_us,uint64_t epoch_us,uint32_t& out) {
  if(!epoch_us || absolute_us<epoch_us || absolute_us-epoch_us>kMaxRunUs)return false;
  out=static_cast<uint32_t>((absolute_us-epoch_us)/1000ULL);return true;
}
inline bool epochSupported(uint64_t epoch_us) {
  return epoch_us && epoch_us/1000ULL<=UINT32_MAX;
}
}
