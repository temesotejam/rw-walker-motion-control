#pragma once
#include <cstdint>
#include "negative_probe_batch.h"

namespace negative_probe_extended_batch {
static constexpr uint16_t kSelectorMs = 40;
static constexpr uint32_t kDurationMs = 30000;
static constexpr uint8_t kBlockLength = 3;
static constexpr uint16_t kWidthsMs[kBlockLength] = {30, 35, 40};
static constexpr uint16_t kSchedulePreviewLength = 40;

inline __attribute__((noinline)) uint16_t width(uint32_t seed, uint16_t negative_crossing_index) {
  const uint16_t block = negative_crossing_index / kBlockLength;
  uint32_t state = seed ^ (0x9e3779b9U * (static_cast<uint32_t>(block) + 1U));
  if (!state) state = 0x6d2b79f5U;
  uint8_t order[kBlockLength] = {0, 1, 2};
  for (uint8_t i = kBlockLength - 1; i > 0; --i) {
    const uint8_t j = static_cast<uint8_t>(negative_probe_batch::next(state) % (i + 1));
    const uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
  }
  return kWidthsMs[order[negative_crossing_index % kBlockLength]];
}
}  // namespace negative_probe_extended_batch
