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
  int fd() const { return socket && !socket->closed ? 3 : -1; }
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
  void (*dispatch)(WebServer&) = nullptr;
  void reply() {
    _currentClientWrite("H", 1);
    _currentClientWrite_P("B", 1);
  }
  void stop() { _currentStatus = HC_NONE; }
  virtual void handleClient() {
    if (_currentStatus == HC_NONE) {
      if (_server.queued.empty()) return;
      _currentClient.socket = _server.queued.front();
      _server.queued.pop_front();
      _currentStatus = HC_WAIT_READ;
      _statusChange = millis();
    }
    if (_currentClient.available()) {
      ++_currentClient.socket->served;
      if (dispatch) dispatch(*this);
    }
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
#include <cerrno>
int socket_error = 0;
int sent_calls = 0;
int send(int, const void*, size_t n, int) {
  if (++sent_calls == 1 || !socket_error) return int(n);  // first send is the header
  errno = socket_error;
  return -1;
}
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

  using Response = BoundedWriteWebServer::Response;
  fixed.dispatch = [](WebServer& s) { s.reply(); };
  for (const int error : {0, EPIPE, ECONNRESET, ENOMEM, ENOBUFS, EAGAIN, EINTR}) {
    Connection client{true};
    socket_error = error; sent_calls = 0;
    fixed.queue(client); fixed.handleClient();
    const auto expected = !error ? Response::Complete :
        (error == EAGAIN || error == ENOMEM || error == ENOBUFS) ? Response::Stalled : Response::Incomplete;
    assert(fixed.lastResponse() == expected);
    fixed.handleClient();
    assert(fixed.lastResponse() == Response::None);
  }
  // Repeated short writes can each succeed while the transaction as a whole
  // exceeds its deadline. Later requests must get the socket immediately.
  fixed.dispatch = [](WebServer& s) {
    auto& b = static_cast<BoundedWriteWebServer&>(s);
    b.beginExportResponse();
    for (unsigned i = 0; i < 8; ++i) {
      s.reply();
      if (b.lastResponse() != Response::Complete) break;
      delay(1100);
    }
  };
  socket_error = 0; sent_calls = 0;
  Connection trickle{true}, resumed{true};
  fixed.queue(trickle); fixed.queue(resumed); fixed.handleClient();
  assert(fixed.lastResponse() == Response::Incomplete && trickle.closed && sent_calls == 12);
  fixed.dispatch = [](WebServer& s) { s.reply(); };
  fixed.handleClient();
  assert(resumed.served == 1 && fixed.lastResponse() == Response::Complete);
}
"""


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "WebServer.h").write_text(SERVER, encoding="utf8")
        (root / "lwip").mkdir()
        (root / "lwip/opt.h").write_text("#define TCP_MSS 1436\n", encoding="utf8")
        (root / "lwip/sockets.h").write_text(
            "#define MSG_DONTWAIT 0\nint send(int,const void*,size_t,int);\n",
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
