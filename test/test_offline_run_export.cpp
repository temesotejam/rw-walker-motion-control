#include "offline_run_session.h"
#include <cassert>
#include <cstdio>

using Result = OfflineRunSession::StartResult;
using Phase = OfflineRunSession::Phase;

struct Fixture {
  OfflineRunSession session;
  uint32_t now = 0;
  Result result = Result::Rejected;
  bool active = false, sealed = false, listener_available = true;
  unsigned prepare_checks = 0, prepares = 0, restores = 0, ready = 0, stops = 0;

  bool stopServer() { return true; }
  bool queueStart() { result = Result::Pending; return true; }
  Result startResult() const { return result; }
  bool runActive() const { return active; }
  void cancelStart() { ++stops; }
  bool prepareCompletedExport() {
    assert(!active);
    ++prepare_checks;
    if (!sealed) return false;
    ++prepares;
    return true;
  }
  bool restoreTransport() { ++restores; return listener_available; }
  void transportReady() { ++ready; }

  void tick(uint32_t elapsed = 20) {
    now += elapsed;
    session.update(now, *this);
  }
  void queue() {
    assert(session.queue(now));
    result = Result::Rejected; active = false; sealed = false;
  }
  void start() {
    queue(); tick(350); assert(session.phase() == Phase::Starting);
    result = Result::Started; active = true;
    tick(); assert(session.phase() == Phase::Running);
  }
  void seal() { active = false; sealed = true; }
};

int main() {
  Fixture f;
  f.start(); f.tick();
  assert(f.prepares == 0);
  f.listener_available = false;
  f.seal(); f.tick();
  assert(f.prepares == 1 && f.prepare_checks == 1 && f.restores == 0 && f.session.phase() == Phase::Restoring);
  f.session.cancel(f.now);
  assert(f.session.phase() == Phase::Restoring);
  f.tick(); assert(f.restores == 1 && f.ready == 0);
  f.listener_available = true;
  f.tick(999); assert(f.restores == 1);
  f.tick(1); f.tick();
  assert(f.prepares == 1 && f.restores == 2 && f.ready == 1 && !f.session.busy());

  // A second run reaches ESTOP and seals before HTTP observes the START acknowledgement.
  f.queue(); f.tick(350);
  f.result = Result::Started; f.seal(); f.tick();
  assert(f.prepares == 2 && f.restores == 2);
  f.tick(); f.tick();
  assert(f.prepares == 2 && f.ready == 2 && !f.session.busy());

  f.queue(); f.tick(350); f.result = Result::Rejected; f.tick(); f.tick();
  assert(f.prepare_checks == 2 && f.prepares == 2 && !f.session.busy());
  f.queue(); f.session.cancel(f.now); f.tick(); f.tick();
  assert(f.prepares == 2 && f.stops == 1 && !f.session.busy());

  f.start(); f.session.cancel(f.now); f.session.cancel(f.now);
  f.tick(); assert(f.prepares == 2 && f.session.phase() == Phase::Cancelling);
  // Consuming STOP overwrites the command result with "stopped" (ok=false).
  f.result = Result::Rejected; f.seal(); f.tick();
  assert(f.prepares == 3 && f.session.phase() == Phase::Restoring);
  f.tick(); f.tick(); assert(f.prepares == 3 && !f.session.busy());

  // START can execute before HTTP observes it, then STOP can replace its result.
  f.queue(); f.tick(350); f.session.cancel(f.now);
  f.result = Result::Rejected; f.seal(); f.tick();
  assert(f.prepares == 4 && f.session.phase() == Phase::Restoring);
  f.tick(); assert(!f.session.busy());
  std::puts("PASS offline session sealed export, listener retry, second run, early ESTOP and STOP");
}
