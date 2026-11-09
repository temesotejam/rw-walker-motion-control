#include <cassert>
#include <iostream>
#include "offline_run_session.h"
#include "usb_diag_control.h"
#include "runtime_diagnostics.h"
using Session = OfflineRunSession;
struct Host {
  // Deliberately no radio methods: the lifecycle must depend only on HTTP.
  bool listening = true, active = false, stop_ok = true, queue_ok = true, restore_ok = true;
  int stopped = 0, queued = 0, cancelled = 0, restored = 0, ready = 0;
  Session::StartResult result = Session::StartResult::Pending;
  bool stopServer() { ++stopped; if (stop_ok) listening=false; return stop_ok; }
  bool queueStart() { assert(!listening); ++queued; return queue_ok; }
  Session::StartResult startResult() { return result; }
  bool runActive() { return active; }
  void cancelStart() { ++cancelled; }
  bool restoreTransport() { assert(!active); ++restored; listening=restore_ok; return restore_ok; }
  void transportReady() { assert(listening); ++ready; }
};
void enter(Session& s, Host& h, uint32_t origin = 0) {
  const int stopped=h.stopped, queued=h.queued;
  assert(s.queue(origin)); assert(!s.queue(origin)); assert(s.serving());
  s.update(origin+349,h); assert(h.stopped==stopped && h.queued==queued);
  s.update(origin+350,h); assert(h.stopped==stopped+1 && h.queued==queued+1);
  assert(!h.listening && !s.serving()); // No AP event or radio restart is needed.
}
int main() {
  for (uint32_t origin : {0U, 0xffffff00U}) {
    Session s; Host h; enter(s,h,origin);
    h.result=Session::StartResult::Started; h.active=true;
    s.update(origin+500,h); assert(!s.serving() && h.restored==0);
    s.update(origin+46000,h); assert(h.restored==0); // Never resume on a display timer.
    h.active=false; s.update(origin+47000,h);
    h.restore_ok=false; s.update(origin+47020,h); assert(h.restored==1);
    s.update(origin+47500,h); assert(h.restored==1);
    h.restore_ok=true; s.update(origin+48020,h);
    assert(!s.busy() && h.ready==1 && h.queued==1);
    s.update(origin+100000,h); assert(h.stopped==1 && h.restored==2);
    h.result=Session::StartResult::Pending;
    enter(s,h,origin+100001); // A complete second run must rebind HTTP again.
    h.result=Session::StartResult::Started; h.active=true; s.update(origin+100501,h);
    assert(!s.serving() && !h.listening && h.queued==2);
    h.active=false; s.update(origin+141001,h); s.update(origin+141021,h);
    assert(!s.busy() && h.ready==2 && h.restored==3 && h.listening);
  }
  { Session s; Host h; h.stop_ok=false; assert(s.queue(0)); s.update(350,h); s.update(400,h);
    assert(!s.busy() && h.queued==0 && s.error()==std::string("http_stop_failed")); }
  { Session s; Host h; assert(s.queue(0)); s.cancel(100); h.result=Session::StartResult::Rejected;
    s.update(120,h); s.update(140,h);
    assert(!s.busy() && h.queued==0 && h.cancelled==1 && h.listening); }
  { Session s; Host h; h.queue_ok=false; enter(s,h); s.update(500,h); assert(!s.busy()); }
  { Session s; Host h; enter(s,h); h.result=Session::StartResult::Rejected;
    s.update(500,h); s.update(520,h); assert(!s.busy()); }
  { Session s; Host h; enter(s,h); h.result=Session::StartResult::Started;
    s.update(500,h); s.update(520,h); assert(!s.busy()); } // Immediate ESTOP/sealed run.
  { Session s; Host h; enter(s,h); s.update(5450,h); s.update(5470,h);
    assert(h.cancelled==1 && h.restored==0);
    h.active=true; h.result=Session::StartResult::Started; s.update(5490,h); assert(h.restored==0);
    h.active=false; s.update(5510,h); s.update(5530,h); assert(!s.busy()); }
  assert(!RuntimeDiag::enabled() && !RuntimeDiag::heavyAllowed());
  RuntimeDiag::setEnabled(true); assert(RuntimeDiag::heavyAllowed());
  RuntimeDiag::setRunActive(true); assert(!RuntimeDiag::heavyAllowed());
  RuntimeDiag::setEnabled(false); RuntimeDiag::setRunActive(false); assert(!RuntimeDiag::heavyAllowed());
  usb_diag::Parser parser;
  auto line=[&](const char* p) { usb_diag::Command r{}; while(*p) r=parser.feed(*p++); return r; };
  assert(line("DIAG ON\r\n")==usb_diag::Command::Enable);
  assert(line("DIAG OFF\n")==usb_diag::Command::Disable);
  assert(line("DIAG STATUS\n")==usb_diag::Command::Status);
  assert(line("ENTER DOWNLOAD\n")==usb_diag::Command::Download);
  assert(line("DIAG ONxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n")==usb_diag::Command::Unknown);
  assert(line("DIAG ON\n")==usb_diag::Command::Enable);
  std::cout << "HTTP pause lifecycle: response drain, stop-before-START, no radio dependency, start rejection/timeout, early STOP, bind retry, two full runs, clock wrap; USB opt-in PASS\n";
}
