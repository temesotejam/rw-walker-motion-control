#include "wifi_transport_restart.h"
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
