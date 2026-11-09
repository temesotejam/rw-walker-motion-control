#pragma once
#include <cstdint>

namespace negative_probe_batch {
// One condition of each duration per five accepted negative ZERO crossings.
// The seed is captured once per run and saved with the event-level assignments.
static constexpr uint16_t kSelectorMs = 25;
static constexpr uint32_t kDurationMs = 30000;
static constexpr uint8_t kBlockLength = 5;
static constexpr uint16_t kWidthsMs[kBlockLength] = {0, 10, 15, 20, 25};
static constexpr uint16_t kSchedulePreviewLength = 40;

inline uint32_t next(uint32_t& state) {
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

inline uint16_t width(uint32_t seed, uint16_t negative_crossing_index) {
  const uint16_t block = negative_crossing_index / kBlockLength;
  uint32_t state = seed ^ (0x9e3779b9U * (static_cast<uint32_t>(block) + 1U));
  if (!state) state = 0x6d2b79f5U;
  uint8_t order[kBlockLength] = {0, 1, 2, 3, 4};
  for (uint8_t i = kBlockLength - 1; i > 0; --i) {
    const uint8_t j = static_cast<uint8_t>(next(state) % (i + 1));
    const uint8_t tmp = order[i];
    order[i] = order[j];
    order[j] = tmp;
  }
  return kWidthsMs[order[negative_crossing_index % kBlockLength]];
}
}  // namespace negative_probe_batch
