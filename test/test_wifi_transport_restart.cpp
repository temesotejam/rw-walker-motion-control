#include "export_transport_recovery.h"
#include <cassert>

struct Host {
  unsigned closes = 0, stops = 0, aps = 0, binds = 0;
  bool stop_ok = true, ap_ok = false, bind_ok = false;
  void closeHttpTransport() { ++closes; }
  bool stopWifiRadio() { ++stops; return stop_ok; }
  bool startWifiAp() { ++aps; return ap_ok; }
  bool startHttpListener() { ++binds; return bind_ok; }
};

int main() {
  {
    HttpStallRecovery stall;
    const uint32_t at = UINT32_MAX - 10000;
    assert(!stall.due(at));
    stall.stalled(at);
    stall.stalled(at + 30000);
    assert(!stall.due(at + 34999) && stall.due(at + 35000));
    stall.repairIssued();
    stall.stalled(at + 36000);
    assert(!stall.due(at + 72000));
    stall.clear();
    stall.stalled(at + 73000);
    stall.clear(); // a fully sent response, or entry to measurement
    assert(!stall.due(at + 150000));
  }
  {
    WifiTransportRestart startup;
    Host boot;
    const uint32_t startup_ms = UINT32_MAX - 10;
    startup.afterStartup(startup_ms, false, false);
    assert(startup.active());
    startup.update(startup_ms + 999, boot);
    assert(boot.aps == 0 && boot.binds == 0);
    startup.update(startup_ms + 1000, boot);
    assert(boot.aps == 1 && boot.binds == 0);
    boot.ap_ok = true;
    startup.update(startup_ms + 2000, boot);
    assert(startup.active() && boot.aps == 2 && boot.binds == 1);
    boot.bind_ok = true;
    startup.update(startup_ms + 3000, boot);
    assert(!startup.active() && boot.aps == 2 && boot.binds == 2);
    startup.update(startup_ms + 120000, boot);
    assert(boot.aps == 2 && boot.binds == 2 && !boot.stops && !boot.closes && !startup.count());
  }
  {
    WifiTransportRestart startup;
    Host boot;
    startup.afterStartup(10, true, true);
    startup.update(200000, boot);
    assert(!startup.active() && !boot.stops && !boot.closes && !boot.aps && !boot.binds);
    startup.afterStartup(200100, true, false);
    boot.bind_ok = true;
    startup.update(201100, boot);
    assert(!startup.active() && !boot.stops && !boot.closes && !boot.aps && boot.binds == 1);
  }
  WifiTransportRestart restart;
  Host host;
  const uint32_t start = UINT32_MAX - 99;
  restart.request();
  restart.update(start, host);
  assert(restart.active() && host.closes == 1 && host.stops == 1);
  restart.request(); // duplicate while running does not stop the radio twice
  restart.update(start + 249, host);
  assert(host.aps == 0 && host.binds == 0);
  restart.update(start + 250, host);
  assert(host.aps == 1 && host.binds == 0 && restart.active());
  host.ap_ok = true;
  restart.update(start + 1249, host);
  assert(host.aps == 1);
  restart.update(start + 1250, host);
  assert(host.aps == 2 && host.binds == 1 && restart.active());
  host.bind_ok = true;
  restart.update(start + 2250, host);
  assert(host.aps == 2 && host.binds == 2 && host.stops == 1);
  assert(!restart.active() && restart.count() == 1);

  // If the driver reports an error during radio shutdown, explicitly restore
  // AP configuration and port 80 but do not claim a full radio restart.
  host.stop_ok = false;
  restart.request(); restart.update(3000, host); restart.update(3250, host);
  assert(host.aps == 3 && host.binds == 3 && !restart.active() && restart.count() == 1);
}
