#include "export_transport_recovery.h"
#include "wifi_transport_restart.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main() {
  using Repair = ExportTransportRecovery::Repair;
  ExportTransportRecovery policy;
  constexpr uint32_t quiet = ExportTransportRecovery::kQuietMs;
  assert(policy.requestRepair(quiet, true, false) == Repair::None);  // idle before a trial
  policy.startRun();
  assert(policy.requestRepair(quiet * 2, false, true) == Repair::None);
  assert(policy.requestRepair(quiet * 2, false, false) == Repair::None);

  uint32_t now = quiet * 3;
  policy.sealed(now);
  policy.sealed(now + quiet - 1);  // duplicate prepare does not restart the timer
  assert(policy.requestRepair(now + quiet - 1, true, false) == Repair::None);
  assert(policy.requestRepair(now + quiet, false, false) == Repair::None);  // stop guard at execution
  assert(policy.requestRepair(now + quiet, true, false) == Repair::WifiRadio);
  assert(policy.requestRepair(now + quiet + 1, true, true) == Repair::None);  // manual / auto deduplication
  policy.sealed(now + quiet + 2);
  assert(policy.requestRepair(now + quiet * 3, true, false) == Repair::None);  // closed tab: one attempt

  // A page may genuinely stall after a sealed run already used its quiet-period
  // repair. A closed tab, a successful retry, and an active/resumed transfer
  // must not acquire a second automatic radio repair.
  {
    ExportTransportRecovery link;
    HttpStallRecovery page;
    const uint32_t start = UINT32_MAX - quiet;
    assert(!link.sealedAfterRadioRepair());
    link.sealed(start);
    assert(!link.sealedAfterRadioRepair());
    assert(link.requestRepair(start + quiet, true, false) == Repair::WifiRadio);
    assert(link.sealedAfterRadioRepair());
    uint32_t t = start + quiet + 1;
    assert(!page.due(t + quiet) && !link.awaitingDownload()); // page simply left closed
    page.stalled(t);
    page.clear(); // another request succeeded after an isolated stalled page
    assert(!page.due(t + quiet));
    page.stalled(t);
    assert(!page.due(t + quiet - 1));
    assert(page.due(t + quiet));
    assert(link.requestRepair(t + quiet, false, true) == Repair::None); // final stop guard
    assert(link.requestRepair(t + quiet, true, true) == Repair::WifiRadio);
    page.repairIssued();
    t += quiet;
    page.stalled(t);
    assert(!page.due(t + quiet)); // one recovery per observed outage
    assert(link.requestRepair(t + quiet - 1, true, true) == Repair::None); // shared cooldown
    link.downloadRequested(t, 17);
    assert(!link.sealedAfterRadioRepair());
    assert(link.pauseDownload(17) && !link.sealedAfterRadioRepair());
    link.downloadRequested(t + 1, 18);
    assert(!link.sealedAfterRadioRepair());
    link.downloadComplete();
    assert(!link.sealedAfterRadioRepair());
    link.startRun();
    link.sealed(t + 2);
    assert(!link.sealedAfterRadioRepair());
  }

  // A new run may remain active for a long time. Its full grace period starts at seal.
  now += quiet * 5;
  policy.startRun();
  assert(policy.requestRepair(now, true, false) == Repair::None);
  policy.sealed(now);
  for (int i = 0; i < 6; ++i) {
    now += quiet - 1;
    assert(policy.requestRepair(now, true, false) == Repair::None);
    policy.onRequest(now);  // ordinary browsing before choosing Download
  }
  constexpr uint32_t stall = ExportTransportRecovery::kTransferStallMs;
  constexpr uint32_t client = 184, other = 721;
  policy.downloadRequested(now, client);
  policy.buildProgress(now + 100, 800);
  policy.buildProgress(now + 1000, 800);  // repeated identical manifest
  policy.downloadRequested(now + 2000, client);  // browser retry of the same POST
  policy.onRequest(now + stall);  // status is still being polled in another tab
  assert(policy.requestRepair(now + stall + 99, true, false) == Repair::None);
  assert(policy.requestRepair(now + stall + 100, false, false) == Repair::None);
  assert(policy.requestRepair(now + stall + 100, true, false) == Repair::HttpListener);
  now += stall + 100;
  assert(policy.requestRepair(now + quiet - 1, true, false) == Repair::None);
  assert(policy.requestRepair(now + quiet, true, false) == Repair::WifiRadio);
  assert(policy.requestRepair(now + quiet * 2, true, false) == Repair::None);  // bounded if the client has gone away
  now += quiet * 3;
  policy.chunkRequested(now, 8192);  // an actual resume from a cached prefix
  policy.chunkRequested(now + stall - 1, 8192);  // stuck retry of the same chunk
  assert(policy.requestRepair(now + stall, true, false) == Repair::HttpListener);
  assert(!policy.pauseDownload(other));
  assert(policy.pauseDownload(client));
  assert(policy.pauseDownload(client));  // lost pause acknowledgement
  policy.downloadRequested(now + quiet, client);  // delayed retry preceding Pause
  policy.onRequest(now + quiet * 2);
  policy.chunkRequested(now + quiet * 2, 0);
  assert(policy.requestRepair(now + quiet * 3, true, false) == Repair::None);
  now += quiet * 4;
  policy.downloadRequested(now, other);
  assert(!policy.pauseDownload(client));  // older browser attempt cannot pause a newer one
  for (uint32_t offset : {8192U, 12288U, 0U, 4096U}) {
    now += stall - 1;
    policy.chunkRequested(now, offset);
    assert(policy.requestRepair(now, true, false) == Repair::None);
  }
  assert(policy.requestRepair(now + stall, true, false) == Repair::HttpListener);
  assert(policy.requestRepair(now + stall + quiet, true, false) == Repair::None); // new client cannot renew radio budget
  policy.downloadComplete();  // browser verifies CRC, including any locally cached tail
  policy.sealed(now + 1);
  assert(policy.requestRepair(now + quiet * 2, true, false) == Repair::None);

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
  assert(rollover.requestRepair(now + stall - 1, true, false) == Repair::None);
  assert(rollover.requestRepair(now + stall, true, false) == Repair::HttpListener);
  assert(rollover.requestRepair(now + stall + quiet, true, false) == Repair::WifiRadio);
  assert(rollover.requestRepair(now + stall + quiet + 1, true, true) == Repair::None);

  ExportTransportRecovery manual;
  ExportTransportRecovery stream;
  stream.sealed(500); stream.downloadRequested(501, 1); stream.chunkRequested(502, 0);
  stream.chunkServed(502 + stall, 65536);
  assert(stream.requestRepair(503 + stall, true, false) == Repair::None);
  stream.chunkServed(501 + 2 * stall, 65536);
  assert(stream.requestRepair(502 + 2 * stall, true, false) == Repair::HttpListener);
  assert(manual.requestRepair(0, true, true) == Repair::WifiRadio);  // explicit test while stopped at READY
  assert(manual.requestRepair(quiet - 1, true, true) == Repair::None);
  manual.startRun(); manual.sealed(quiet);
  assert(manual.requestRepair(quiet + 1, true, true) == Repair::WifiRadio);
  assert(manual.requestRepair(quiet * 3, true, false) == Repair::None);  // this trial's budget is consumed

  // Model the old run's repeated 35--39 s intervals of intermittent delivery.
  // Even if TCP has stalled each time, retain the association for every range.
  ExportTransportRecovery intermittent;
  now = 500;
  intermittent.sealed(now);
  intermittent.downloadRequested(now, client);
  for (uint32_t chunk = 1; chunk <= 24; ++chunk) {
    assert(intermittent.requestRepair(now + stall, true, false) == Repair::HttpListener);
    now += quiet + (chunk % 5) * 1000;
    assert(intermittent.requestRepair(now, true, false) == Repair::None);
    intermittent.chunkRequested(now, 4096 * chunk);
    intermittent.chunkServed(now, 4096 * (chunk + 1));
  }
  assert(intermittent.listenerRecoveryCount() == 24);
  assert(intermittent.requestRepair(now + stall, true, false) == Repair::HttpListener);
  now += stall;
  // Repeated requests still reaching the device never call for a radio reset.
  for (unsigned i = 0; i < 8; ++i) {
    now += quiet - 1;
    if (i % 2) intermittent.onRequest(now);
    else intermittent.chunkRequested(now, 4096 * 24);
    assert(intermittent.requestRepair(now, true, false) == Repair::None);
  }
  assert(intermittent.requestRepair(now + quiet - 1, true, false) == Repair::None);
  assert(intermittent.requestRepair(now + quiet, true, false) == Repair::WifiRadio);
  intermittent.buildProgress(now + quiet + 1, 16384);
  intermittent.downloadRequested(now + quiet + 2, other);
  now += quiet + 2;
  assert(intermittent.requestRepair(now + stall, true, false) == Repair::HttpListener);
  assert(intermittent.requestRepair(now + stall + quiet, true, false) == Repair::None);
  intermittent.startRun();
  assert(intermittent.listenerRecoveryCount() == 0);
  intermittent.sealed(now + stall + quiet);
  assert(intermittent.requestRepair(now + stall + 2 * quiet, true, false) == Repair::WifiRadio);

  ExportTransportRecovery camera;
  now = UINT32_MAX - 8000;
  camera.sealed(now);
  camera.exportContact(now); // automatic seal or a manifest check alone never parks the observer
  assert(!camera.priorityRequested(now));
  camera.downloadRequested(now, client);
  assert(camera.priorityRequested(now));
  camera.onRequest(now + quiet - 5);
  camera.buildProgress(now + quiet - 1, 8192);
  assert(camera.priorityRequested(now + quiet - 1));
  assert(!camera.priorityRequested(now + quiet)); // abandoned while status/worker remained active, past rollover
  now += quiet;
  camera.chunkRequested(now, 0); // resumed after abandonment
  assert(camera.priorityRequested(now));
  now += quiet - 1;
  camera.chunkRequested(now, 0); // receiver retry is still an active client
  assert(camera.priorityRequested(now + quiet - 1));
  now += quiet - 1;
  camera.exportContact(now); // explicit manifest poll whilst sealing
  now += quiet - 1;
  assert(camera.priorityRequested(now));
  assert(camera.pauseDownload(client));
  camera.downloadRequested(++now, client); // late duplicate prepare/manifest/chunk after Pause
  camera.exportContact(now);
  camera.chunkRequested(now, 4096);
  assert(!camera.priorityRequested(now));
  camera.downloadRequested(++now, other);
  assert(camera.priorityRequested(now));
  camera.downloadComplete();
  camera.exportContact(now);
  assert(!camera.priorityRequested(now));
  camera.downloadRequested(++now, client);
  assert(camera.priorityRequested(now));
  camera.startRun(); // accepted Clear/next measurement
  camera.chunkRequested(now, 8192);
  assert(!camera.priorityRequested(now));
  std::puts("PASS sealed export Wi-Fi repair timing, progress, acknowledgement and deduplication");
}
