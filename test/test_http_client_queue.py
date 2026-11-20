"""Exercise the actual bounded server against Arduino's single-client waiting semantics."""
from pathlib import Path
import subprocess
import tempfile

from _project import ROOT


SERVER = r"""
#pragma once
#include <Arduino.h>
using PGM_P = const char*;
inline void vTaskDelay(unsigned ticks) { delay(ticks); }
enum HTTPClientStatus { HC_NONE, HC_WAIT_READ, HC_WAIT_CLOSE };
struct Connection { bool readable = false, closed = false; int served = 0; };
struct Client {
  Connection* socket = nullptr;
  int fd() const { return -1; }
  int available() { return socket && !socket->closed && socket->readable; }
  void stop() { if (socket) socket->closed = true; }
};
struct Listener {
  std::deque<Connection*> queued;
  unsigned probes = 0;
  bool hasClient() { ++probes; return !queued.empty(); }
  Client available() {
    if (queued.empty()) return {};
    auto* socket = queued.front(); queued.pop_front(); return {socket};
  }
  explicit operator bool() const { return true; }
};
class WebServer {
 public:
  explicit WebServer(int) {}
  void queue(Connection& c) { _server.queued.push_back(&c); }
  unsigned probes() const { return _server.probes; }
  void stop() { _currentStatus = HC_NONE; }
  virtual void handleClient() {
    if (_currentStatus == HC_NONE) {
      if (_server.queued.empty()) return;
      _currentClient.socket = _server.queued.front();
      _server.queued.pop_front();
      _currentStatus = HC_WAIT_READ;
      _statusChange = millis();
    }
    if (_currentClient.available()) ++_currentClient.socket->served;
    else if (!_currentClient.socket->closed && millis() - _statusChange <= 5000) return;
    _currentStatus = HC_NONE;
    _currentClient = {};
  }
 protected:
  Listener _server;
  Client _currentClient;
  HTTPClientStatus _currentStatus = HC_NONE;
  uint32_t _statusChange = 0;
  virtual size_t _currentClientWrite(const char*, size_t) { return 0; }
  virtual size_t _currentClientWrite_P(PGM_P, size_t) { return 0; }
};
"""

CASES = r"""
#include "bounded_web_server.h"
#include <cassert>
int main() {
  Connection idle, status{true}, slow, queued{true}, late, other{true};
  WebServer baseline(80);
  baseline.queue(idle);
  baseline.queue(status);
  baseline.handleClient();
  delay(2500);
  baseline.handleClient();
  assert(!idle.closed && status.served == 0);

  BoundedWriteWebServer fixed;
  fixed.queue(idle);
  fixed.handleClient();
  fixed.queue(status);
  delay(249); fixed.handleClient();
  assert(!idle.closed && status.served == 0);
  delay(1); fixed.handleClient(); fixed.handleClient();
  assert(idle.closed && status.served == 1);

  fixed.queue(slow); fixed.handleClient();
  delay(250); fixed.handleClient();
  const auto checks = fixed.probes();
  delay(10); fixed.handleClient();
  assert(fixed.probes() == checks && !slow.closed);
  slow.readable = true;
  fixed.queue(queued);
  delay(40); fixed.handleClient(); fixed.handleClient();
  assert(!slow.closed && slow.served == 1 && queued.served == 1);

  fixed.queue(late); fixed.handleClient();
  delay(800); fixed.handleClient();
  assert(!late.closed);
  fixed.queue(other); delay(50); fixed.handleClient(); fixed.handleClient();
  assert(late.closed && other.served == 1);
  Connection current, cached{true};
  fixed.queue(current); fixed.handleClient(); fixed.queue(cached);
  fixed.closeTransport();
  assert(current.closed && cached.closed);
}
"""


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "WebServer.h").write_text(SERVER, encoding="utf8")
        (root / "lwip").mkdir()
        (root / "lwip/sockets.h").write_text(
            "#define MSG_DONTWAIT 0\ninline int send(int,const void*,size_t n,int){return int(n);}\n",
            encoding="utf8",
        )
        test = root / "queue.cpp"
        test.write_text(CASES, encoding="utf8")
        binary = root / "http-queue.exe"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", root,
                        "-I", ROOT / "test/fixtures/arduino", "-I", ROOT / "src", test,
                        ROOT / "src/bounded_web_server.cpp", "-o", binary], check=True)
        subprocess.run([binary], check=True)
    print("PASS status behind an idle connection, readable and unqueued clients, and throttled queue checks")


if __name__ == "__main__":
    main()
