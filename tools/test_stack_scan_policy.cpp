#include <cassert>
#include <cstdint>
#include <iostream>
#include "../src/stack_scan_policy.h"

int main() {
  stack_scan::Schedule s;
  uint32_t calls = 0;
  auto scan = [&]() { ++calls; };
  // Startup can be paused before the first scan; a zero timestamp is valid.
  assert(!s.sampleIfDue(0, false, scan) && calls == 0 && !s.seen);
  assert(s.sampleIfDue(0, true, scan) && calls == 1 && s.seen);
  assert(!s.sampleIfDue(999, true, scan));
  assert(s.sampleIfDue(1000, true, scan) && calls == 2);
  // Five-second START_SYNC, thirty-second RUNNING and five-second END_SYNC.
  // Invocations continue at 1 kHz, but the expensive callback never runs.
  for (uint32_t ms = 1500; ms < 41500; ++ms)
    assert(!s.sampleIfDue(ms, false, scan));
  assert(calls == 2 && s.last_ms == 1000);
  assert(s.sampleIfDue(41500, true, scan) && calls == 3);
  assert(!s.sampleIfDue(42499, true, scan));
  assert(s.sampleIfDue(42500, true, scan) && calls == 4);
  // Another run / immediate ESTOP resumes only if the scan was due.
  assert(!s.sampleIfDue(43000, false, scan));
  assert(!s.sampleIfDue(43001, true, scan));
  assert(s.sampleIfDue(43500, true, scan));
  stack_scan::Schedule wrapped;
  assert(wrapped.sampleIfDue(UINT32_MAX - 200, true, scan));
  assert(!wrapped.sampleIfDue(798, true, scan));
  assert(!wrapped.sampleIfDue(799, false, scan));
  assert(wrapped.sampleIfDue(799, true, scan));
  std::cout << "Stack scan callback excluded during full run, retained after stop, idle cadence and wrap PASS\n";
}
