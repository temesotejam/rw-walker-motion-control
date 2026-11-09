#pragma once
#include <stdint.h>

namespace stack_scan {
// Owner-local schedule. The supplied scan must run in the task being checked.
// A deferred scan remains due, so the first idle beat refreshes the lifetime
// high-water mark. Heartbeats and overflow protection are independent of this.
struct Schedule {
  bool seen = false;
  uint32_t last_ms = 0;
  template<class Scan>
  bool sampleIfDue(uint32_t now_ms, bool allowed, Scan scan) {
    if (!allowed || (seen && static_cast<uint32_t>(now_ms - last_ms) < 1000)) return false;
    scan();
    last_ms = now_ms;
    seen = true;
    return true;
  }
};
}
