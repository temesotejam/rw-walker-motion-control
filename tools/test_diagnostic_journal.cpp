#include "diagnostic_journal.h"
#include <assert.h>
#include <stdio.h>

struct Evidence { uint32_t boot, stage, heartbeat, count; };
using Journal = diagnostic_journal::Journal<Evidence, 0x74657374>;
int main() {
  Journal journal{}; Evidence out{}; uint32_t seq = 0;
  assert(journal.latest(out) == -1);
  journal.save({7, 12, 1500, 42});
  assert(journal.latest(out, &seq) == 0 && out.boot == 7 && out.count == 42 && seq == 1);
  const Journal good = journal;
  journal.save({8, 3, 1800, 55});
  assert(journal.latest(out, &seq) == 1 && out.boot == 8 && seq == 2);
  // Reset after each byte of the newly written slot, including CRC and final
  // commit marker: either the complete new sample or the preceding one survives.
  const Journal complete = journal;
  for (size_t n = 0; n <= sizeof(Journal::Slot); ++n) {
    Journal torn = good;
    memcpy(&torn.slots[1], &complete.slots[1], n);
    assert(torn.latest(out) >= 0);
    assert((out.boot == 7 && out.count == 42) || (out.boot == 8 && out.count == 55));
  }
  // Single-bit corruption in any byte of the newest slot falls back safely.
  for (size_t byte = 0; byte < sizeof(Journal::Slot); ++byte) {
    for (unsigned bit = 0; bit < 8; ++bit) {
      Journal broken = complete;
      reinterpret_cast<uint8_t*>(&broken.slots[1])[byte] ^= 1U << bit;
      assert(broken.latest(out) == 0 && out.boot == 7 && out.count == 42);
    }
  }
  journal = good;
  journal.slots[0].sequence = UINT32_MAX;
  journal.slots[0].crc = Journal::checksum(journal.slots[0]);
  journal.save({9, 1, 10, 1});
  assert(journal.latest(out, &seq) == 1 && out.boot == 9 && seq == 0);
  journal.clear(); assert(journal.latest(out) == -1);
  puts("RTC journal: partial writes, bit corruption, sequence wrap and clear passed");
}
