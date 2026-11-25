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
  void replyLarge(bool packets) {
    static const char body[42156] = {};
    _currentClientWrite("header", 6);
    if (!packets) { _currentClientWrite_P(body, sizeof(body)); return; }
    for (size_t at = 0; at < sizeof(body); at += 4096)
      if (!_currentClientWrite(body + at, std::min(size_t(4096), sizeof(body) - at))) break;
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
#include <esp_netif.h>
#include <lwip/api.h>
#include <cerrno>
#include <cstdio>
#define assert(test) do { if (!(test)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); std::exit(1); } } while (false)
int socket_error = 0;
int sent_calls = 0;
tcp_pcb host_pcb{};
netconn host_conn{NETCONN_TCP, {&host_pcb}};
int ack_delay = 0;
uint32_t last_ack_ms = 0;
size_t peak_outstanding = 0, transferred = 0;
esp_err_t esp_netif_tcpip_exec(esp_err_t(*fn)(void*), void* arg) {
  if (ack_delay >= 0 && millis() - last_ack_ms >= unsigned(ack_delay)) {
    host_pcb.lastack = host_pcb.snd_lbb;
    host_pcb.snd_buf = 5760;
    host_pcb.snd_queuelen = 0;
    last_ack_ms = millis();
  }
  return fn(arg);
}
int send(int, const void*, size_t n, int) {
  if (++sent_calls == 1 || !socket_error) {
    assert(n <= host_pcb.mss && n <= host_pcb.snd_buf && host_pcb.snd_queuelen < 2);
    host_pcb.snd_lbb += n; host_pcb.snd_buf -= n; ++host_pcb.snd_queuelen;
    const uint32_t outstanding = host_pcb.snd_lbb - host_pcb.lastack;
    assert(outstanding <= 2 * host_pcb.mss);
    peak_outstanding = std::max(peak_outstanding, size_t(outstanding));
    transferred += n;
    return int(n);
  }
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
  // Both the root page (one flash write) and many RAM packet writes share ACK
  // credit with the header. Cross TCP's 32-bit sequence rollover on each run.
  for (const bool packets : {false, true}) {
    fixed.dispatch = packets ? +[](WebServer& s) {
      static_cast<BoundedWriteWebServer&>(s).beginExportResponse(); s.replyLarge(true);
    } : +[](WebServer& s) { s.replyLarge(false); };
    host_pcb.lastack = host_pcb.snd_lbb = UINT32_MAX - 500;
    host_pcb.snd_queuelen = 0; host_pcb.snd_buf = 5760;
    ack_delay = 35; last_ack_ms = millis(); transferred = peak_outstanding = 0;
    Connection paged{true}; fixed.queue(paged); fixed.handleClient();
    assert(transferred == 42162 && peak_outstanding == 2 * host_pcb.mss);
    assert(fixed.lastResponse() == Response::Complete);
  }
  host_pcb.lastack = host_pcb.snd_lbb; host_pcb.snd_queuelen = 0; host_pcb.snd_buf = 5760;
  ack_delay = -1; transferred = 0;
  Connection unacked{true}; fixed.queue(unacked); fixed.handleClient();
  assert(unacked.closed && fixed.lastResponse() == Response::Stalled);
  assert(transferred <= 2 * host_pcb.mss);
}
"""


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "WebServer.h").write_text(SERVER, encoding="utf8")
        (root / "lwip").mkdir()
        (root / "lwip/priv").mkdir()
        (root / "esp_netif.h").write_text("#pragma once\nusing esp_err_t=int; constexpr int ESP_OK=0;\nesp_err_t esp_netif_tcpip_exec(esp_err_t(*)(void*),void*);\n", encoding="utf8")
        (root / "lwip/api.h").write_text(r"""
#pragma once
enum {NETCONN_TCP=16, ESTABLISHED=4, CLOSE_WAIT=5};
struct tcp_pcb {
  unsigned mss=1436, snd_buf=5760, snd_queuelen=0;
  uint32_t lastack=0, snd_lbb=0;
  int state=ESTABLISHED;
};
struct netconn { int type; struct {tcp_pcb* tcp;} pcb; };
extern netconn host_conn;
#define NETCONNTYPE_GROUP(x) (x)
#define netconn_type(c) ((c)->type)
""", encoding="utf8")
        (root / "lwip/tcp.h").write_text("#define tcp_mss(p) ((p)->mss)\n#define tcp_sndbuf(p) ((p)->snd_buf)\n#define tcp_sndqueuelen(p) ((p)->snd_queuelen)\n",encoding="utf8")
        (root / "lwip/priv/sockets_priv.h").write_text("struct lwip_sock { netconn* conn; };\ninline lwip_sock* lwip_socket_dbg_get_socket(int fd) { static lwip_sock s{&host_conn}; return fd==3?&s:nullptr; }\n",encoding="utf8")
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
