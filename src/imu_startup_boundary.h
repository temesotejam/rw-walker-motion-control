#pragma once
#include <stdint.h>

// Only the consumer changes this object, once per idle -> START_SYNC transition.
// Sequence comparisons are wrap-safe for runs shorter than 2^31 samples.
struct ImuStartupBoundary {
  uint32_t epoch_us = 0, cutoff = 0, discarded_idle_samples = 0;
  void enter(uint32_t now_us, uint32_t latest_sequence) {
    epoch_us = now_us;
    cutoff = latest_sequence;
    discarded_idle_samples = 0;
  }
  bool accepts(uint32_t sequence) const {
    return static_cast<int32_t>(sequence - cutoff) > 0;
  }
};
