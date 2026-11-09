#pragma once
#include <cstdint>
#include "negative_probe_batch.h"

namespace positive_current_batch {
static constexpr uint16_t kSelectorMs = 50;
static constexpr uint16_t kNegativeWidthMs = 40;
static constexpr uint32_t kDurationMs = 30000;
static constexpr uint8_t kBlockLength = 3;
static constexpr int16_t kCurrentsMa[kBlockLength] = {300, 350, 400};
static constexpr uint16_t kSchedulePreviewLength = 40;

inline __attribute__((noinline)) int16_t current(uint32_t seed, uint16_t positive_crossing_index) {
  const uint16_t block = positive_crossing_index / kBlockLength;
  uint32_t state = seed ^ (0x7f4a7c15U * (static_cast<uint32_t>(block) + 1U));
  if (!state) state = 0x6d2b79f5U;
  uint8_t order[kBlockLength] = {0, 1, 2};
  for (uint8_t i = kBlockLength - 1; i > 0; --i) {
    const uint8_t j = static_cast<uint8_t>(negative_probe_batch::next(state) % (i + 1));
    const uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
  }
  return kCurrentsMa[order[positive_crossing_index % kBlockLength]];
}
}  // namespace positive_current_batch
