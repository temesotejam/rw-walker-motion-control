#include "export_transport_recovery.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
  ExportTransportRecovery policy;
  constexpr uint32_t quiet = ExportTransportRecovery::kQuietMs;
  assert(!policy.requestRepair(quiet, true, false));  // idle before a trial
  policy.startRun();
  assert(!policy.requestRepair(quiet * 2, false, true));
  assert(!policy.requestRepair(quiet * 2, false, false));

  uint32_t now = quiet * 3;
  policy.sealed(now);
  policy.sealed(now + quiet - 1);  // duplicate prepare does not restart the timer
  assert(!policy.requestRepair(now + quiet - 1, true, false));
  assert(!policy.requestRepair(now + quiet, false, false));  // stop guard at execution
  assert(policy.requestRepair(now + quiet, true, false));
  assert(!policy.requestRepair(now + quiet + 1, true, true));  // manual / auto deduplication
  policy.sealed(now + quiet + 2);
  assert(!policy.requestRepair(now + quiet * 3, true, false));  // closed tab: one attempt

  // A new run may remain active for a long time. Its full grace period starts at seal.
  now += quiet * 5;
  policy.startRun();
  assert(!policy.requestRepair(now, true, false));
  policy.sealed(now);
  for (int i = 0; i < 6; ++i) {
    now += quiet - 1;
    assert(!policy.requestRepair(now, true, false));
    policy.onRequest(now);  // ordinary browsing before choosing Download
  }
  constexpr uint32_t stall = ExportTransportRecovery::kTransferStallMs;
  constexpr uint32_t client = 184, other = 721;
  policy.downloadRequested(now, client);
  policy.buildProgress(now + 100, 800);
  policy.buildProgress(now + 1000, 800);  // repeated identical manifest
  policy.downloadRequested(now + 2000, client);  // browser retry of the same POST
  policy.onRequest(now + stall);  // status is still being polled in another tab
  assert(!policy.requestRepair(now + stall + 99, true, false));
  assert(policy.requestRepair(now + stall + 100, true, false));
  now += stall + 100;
  assert(!policy.requestRepair(now + quiet - 1, true, false));
  assert(policy.requestRepair(now + quiet, true, false));
  assert(!policy.requestRepair(now + quiet * 2, true, false));  // bounded if the client has gone away
  now += quiet * 3;
  policy.chunkRequested(now, 8192);  // an actual resume from a cached prefix
  policy.chunkRequested(now + stall - 1, 8192);  // stuck retry of the same chunk
  assert(policy.requestRepair(now + stall, true, false));
  assert(!policy.pauseDownload(other));
  assert(policy.pauseDownload(client));
  assert(policy.pauseDownload(client));  // lost pause acknowledgement
  policy.downloadRequested(now + quiet, client);  // delayed retry preceding Pause
  policy.onRequest(now + quiet * 2);
  policy.chunkRequested(now + quiet * 2, 0);
  assert(!policy.requestRepair(now + quiet * 3, true, false));
  now += quiet * 4;
  policy.downloadRequested(now, other);
  assert(!policy.pauseDownload(client));  // older browser attempt cannot pause a newer one
  for (uint32_t offset : {8192U, 12288U, 0U, 4096U}) {
    now += stall - 1;
    policy.chunkRequested(now, offset);
    assert(!policy.requestRepair(now, true, false));
  }
  policy.downloadComplete();  // browser verifies CRC, including any locally cached tail
  policy.sealed(now + 1);
  assert(!policy.requestRepair(now + quiet * 2, true, false));

  constexpr const char* token = "0123456789abcdef";
  const auto matches = [&](bool ready, const char* received_token, uint32_t crc, uint32_t run) {
    return ExportTransportRecovery::matchesCompletion(ready, token, 0xbeef, 17, received_token, crc, run);
  };
  assert(matches(true, token, 0xbeef, 17));
  assert(!matches(false, token, 0xbeef, 17));
  assert(!matches(true, "0123456789abcde0", 0xbeef, 17));
  assert(!matches(true, nullptr, 0xbeef, 17));
  assert(!matches(true, token, 0, 17));
  assert(!matches(true, token, 0xbeef, 65536 + 17));  // oversized IDs cannot alias a valid run

  // A last-chunk response can itself be lost. Without a verified client acknowledgement
  // the quiet interval still permits a repair, even across the 32-bit clock rollover.
  ExportTransportRecovery rollover;
  now = UINT32_MAX - 10000;
  rollover.sealed(now - 5000);
  rollover.downloadRequested(now - 3000, 0);  // legacy browser
  rollover.chunkRequested(now, 0);
  assert(!rollover.requestRepair(now + stall - 1, true, false));
  assert(rollover.requestRepair(now + stall, true, false));
  assert(!rollover.requestRepair(now + stall + 1, true, true));

  ExportTransportRecovery manual;
  ExportTransportRecovery stream;
  stream.sealed(500); stream.downloadRequested(501, 1); stream.chunkRequested(502, 0);
  stream.chunkServed(502 + stall, 65536);
  assert(!stream.requestRepair(503 + stall, true, false));
  stream.chunkServed(501 + 2 * stall, 65536);
  assert(stream.requestRepair(502 + 2 * stall, true, false));
  assert(manual.requestRepair(0, true, true));  // explicit test while stopped at READY
  assert(!manual.requestRepair(quiet - 1, true, true));
  manual.startRun(); manual.sealed(quiet);
  assert(manual.requestRepair(quiet + 1, true, true));
  assert(!manual.requestRepair(quiet * 3, true, false));  // this trial's budget is consumed
  std::puts("PASS sealed export Wi-Fi repair timing, progress, acknowledgement and deduplication");
}
