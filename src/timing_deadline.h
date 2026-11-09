#pragma once
#include <stdint.h>

// Observations, not a safety override. Strict > budget; no tolerance silently
// folded into a pass. No floating point, heap allocation, or I/O when recording.
namespace timing_deadline {
struct Counter {
  uint32_t count = 0, over = 0, maximum = 0;
  uint64_t sum = 0;
  void add(uint32_t value, uint32_t budget) {
    ++count; sum += value;
    if (value > maximum) maximum = value;
    if (value > budget) ++over;
  }
  bool passed() const { return count != 0 && over == 0; }
};
}
