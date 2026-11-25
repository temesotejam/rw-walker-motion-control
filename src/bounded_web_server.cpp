#include "bounded_web_server.h"
#include "runtime_diagnostics.h"
#include <Arduino.h>
#include <algorithm>
#include <esp_netif.h>
#include <errno.h>
#include <lwip/api.h>
#include <lwip/opt.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>
extern "C" {
#include <lwip/priv/sockets_priv.h>  // pinned ESP-IDF exports this fd lookup
}

namespace {
struct SendCredit {
  int fd;
  size_t bytes;
  bool connected;
};

esp_err_t inspectSendCredit(void* context) {
  auto& credit = *static_cast<SendCredit*>(context);
  const auto* socket = lwip_socket_dbg_get_socket(credit.fd);
  if (!socket || !socket->conn || NETCONNTYPE_GROUP(netconn_type(socket->conn)) != NETCONN_TCP)
    return ESP_OK;
  const auto* pcb = socket->conn->pcb.tcp;
  if (!pcb || (pcb->state != ESTABLISHED && pcb->state != CLOSE_WAIT)) return ESP_OK;
  credit.connected = true;
  const uint32_t mss = tcp_mss(pcb);
  const uint32_t limit = 2 * mss;
  // Sequence-number subtraction includes bytes queued but not yet sent as
  // well as bytes sent but not yet acknowledged, including sequence wrap.
  const uint32_t queued = pcb->snd_lbb - pcb->lastack;
  if (queued < limit && tcp_sndqueuelen(pcb) < 2) {
    credit.bytes = std::min<uint32_t>(std::min<uint32_t>(mss, limit - queued), tcp_sndbuf(pcb));
  }
  return ESP_OK;
}
}

void BoundedWriteWebServer::beginExportResponse() {
  export_started_ms_ = millis();
  export_response_ = true;
}

void BoundedWriteWebServer::closeTransport() {
  _currentClient.stop();
  // hasClient() may have accepted a peer before the radio restart. Arduino's
  // WiFiServer::end() closes the listener, but drops this cached FD on begin().
  if (listening()) _server.available().stop();
  WebServer::stop();
}

void BoundedWriteWebServer::handleClient() {
  response_ = Response::None;
  export_response_ = false;
  // The Arduino server gives one TCP client up to 5 s to send its first byte.
  // A browser can preconnect an empty socket ahead of the 2.5 s status poll.
  // Give that client a grace period, then let a queued connection use the slot.
  const uint32_t now = millis();
  if (_currentStatus == HC_WAIT_READ &&
      static_cast<uint32_t>(now - _statusChange) >= 250 &&
      static_cast<uint32_t>(now - last_queue_probe_ms_) >= 50) {
    last_queue_probe_ms_ = now;
    if (_server.hasClient() && !_currentClient.available()) _currentClient.stop();
  }
  WebServer::handleClient();
}

size_t BoundedWriteWebServer::pacedWrite(const uint8_t* buffer, size_t length, const char*) {
  if (!length) return 0;
  if (response_ == Response::None) response_ = Response::Complete;
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpSend);
  const int fd = _currentClient.fd();
  if (fd < 0) {
    if (response_ != Response::Stalled) response_ = Response::Incomplete;
    return 0;
  }
  size_t total = 0;
  const uint32_t begin = export_response_ ? export_started_ms_ : millis();
  const uint32_t total_limit_ms = export_response_ ? 6000 : 15000;
  uint32_t last_progress = millis();
  // Leave time for the browser to retry; a trickle through many sendContent()
  // calls must not keep the single HTTP handler occupied for minutes.
  int write_error = 0;
  while (total < length &&
         static_cast<uint32_t>(millis() - begin) < total_limit_ms &&
         static_cast<uint32_t>(millis() - last_progress) < 3000) {
    // SO_SNDBUF is unimplemented in this SDK. Inspect the PCB on the TCP/IP
    // thread while the HTTP owner holds the fd; no lwIP pointer crosses tasks.
    // Keep enough internal heap for the radio to receive the ACK that releases
    // each window. This also applies to the initial HTML page and its header.
    SendCredit credit{fd, 0, false};
    if (esp_netif_tcpip_exec(inspectSendCredit, &credit) != ESP_OK) {
      write_error = ENOBUFS; vTaskDelay(5); continue;
    }
    if (!credit.connected) { write_error = EPIPE; break; }
    if (!credit.bytes) { write_error = EAGAIN; vTaskDelay(5); continue; }
    const size_t n = std::min(length - total, credit.bytes);
    const int sent = ::send(fd, buffer + total, n, MSG_DONTWAIT);
    if (sent > 0) { total += sent; last_progress = millis(); write_error = 0; }
    else {
      write_error = sent == 0 ? EPIPE : errno;
      if (write_error != EAGAIN && write_error != EWOULDBLOCK &&
          write_error != ENOMEM && write_error != ENOBUFS && write_error != EINTR) break;
    }
    vTaskDelay(sent < 0 ? 5 : 1);
  }
  RuntimeDiag::httpWriteResult(total, length, total < length && !write_error ? ETIMEDOUT : write_error);
  if (total != length) {
    // A browser can abandon a request normally, or a large page can reach the
    // total deadline while bytes are still moving. Only sustained send-buffer
    // backpressure with no progress is evidence for a possible transport repair.
    const bool backpressure = write_error == EAGAIN || write_error == EWOULDBLOCK ||
        write_error == ENOMEM || write_error == ENOBUFS;
    if (backpressure && millis() - last_progress >= 3000) response_ = Response::Stalled;
    else if (response_ != Response::Stalled) response_ = Response::Incomplete;
    RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpClose);
    _currentClient.stop();
  }
  return total;
}
size_t BoundedWriteWebServer::_currentClientWrite(const char* buffer, size_t length) {
  return pacedWrite(reinterpret_cast<const uint8_t*>(buffer), length, "ram");
}
size_t BoundedWriteWebServer::_currentClientWrite_P(PGM_P buffer, size_t length) {
  return pacedWrite(reinterpret_cast<const uint8_t*>(buffer), length, "flash");
}
