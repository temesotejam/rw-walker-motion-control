#pragma once

#include <WebServer.h>

// Transport-only wrapper around Arduino WebServer. It preserves every route,
// handler and response body, but bounds PROGMEM socket writes so a large HTML
// page does not require the TCP stack to absorb the whole payload at once.
class BoundedWriteWebServer : public WebServer {
 public:
  enum class Response { None, Complete, Incomplete, Stalled };
  explicit BoundedWriteWebServer(int port = 80) : WebServer(port) {}
  bool listening() { return static_cast<bool>(_server); }
  void handleClient() override;
  void closeTransport();
  Response lastResponse() const { return response_; }
  // Header and all packet writes share one deadline for the requested log range.
  void beginExportResponse();

 protected:
  size_t _currentClientWrite(const char* buffer, size_t length) override;
  size_t _currentClientWrite_P(PGM_P buffer, size_t length) override;

 private:
  uint32_t last_queue_probe_ms_ = 0;
  uint32_t export_started_ms_ = 0;
  bool export_response_ = false;
  Response response_ = Response::None;
  size_t pacedWrite(const uint8_t* buffer, size_t length, const char* source_tag);
};
