#include "bounded_web_server.h"
#include "runtime_diagnostics.h"
#include <Arduino.h>
#include <errno.h>
#include <lwip/sockets.h>

size_t BoundedWriteWebServer::pacedWrite(const uint8_t* buffer, size_t length, const char*) {
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpSend);
  const int fd = _currentClient.fd();
  if (fd < 0) return 0;
  size_t total = 0;
  const uint32_t begin = millis();
  uint32_t last_progress = begin;
  // A complete runtime page is about 96 kB. A fixed 1.2 s total limit can cut
  // off its trailing script on a slow AP even while bytes arrive. Permit
  // slow progress, but retain both an idle timeout and a finite total budget.
  // These limits do not bound stop() or all of handleClient().
  while (total < length &&
         static_cast<uint32_t>(millis() - begin) < 15000 &&
         static_cast<uint32_t>(millis() - last_progress) < 3000) {
    const size_t n = length - total < 1024 ? length - total : 1024;
    const int sent = ::send(fd, buffer + total, n, MSG_DONTWAIT);
    if (sent > 0) { total += sent; last_progress = millis(); }
    else if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOMEM && errno != EINTR)) break;
    vTaskDelay(1);
  }
  if (total != length) {
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
