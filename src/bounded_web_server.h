#pragma once

#include <WebServer.h>

// Transport-only wrapper around Arduino WebServer. It preserves every route,
// handler and response body, but bounds PROGMEM socket writes so a large HTML
// page does not require the TCP stack to absorb the whole payload at once.
class BoundedWriteWebServer : public WebServer {
 public:
  explicit BoundedWriteWebServer(int port = 80) : WebServer(port) {}
  bool listening() { return static_cast<bool>(_server); }
  void handleClient() override;
  void closeTransport();
  bool takeResponseActivity() {
    const bool seen = response_activity_;
    response_activity_ = false;
    return seen;
  }

 protected:
  size_t _currentClientWrite(const char* buffer, size_t length) override;
  size_t _currentClientWrite_P(PGM_P buffer, size_t length) override;

 private:
  uint32_t last_queue_probe_ms_ = 0;
  bool response_activity_ = false;
  size_t pacedWrite(const uint8_t* buffer, size_t length, const char* source_tag);
};
