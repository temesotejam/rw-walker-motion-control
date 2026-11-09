#include "tcp_transport_debug.h"

#include <WiFi.h>
#include "esp_heap_caps.h"

namespace {

constexpr uint16_t kDiagPort = 81;
constexpr uint32_t kClientTimeoutMs = 1200;
constexpr size_t kMaxRequestBytes = 512;

WiFiServer g_server(kDiagPort);
WiFiClient g_client;
uint32_t g_client_open_ms = 0;
uint32_t g_last_heartbeat_ms = 0;
uint32_t g_accept_count = 0;
uint32_t g_request_count = 0;
uint32_t g_response_count = 0;
uint32_t g_request_bytes = 0;
char g_first_line[192] = {};
size_t g_first_line_len = 0;
bool g_first_line_done = false;

void resetClientState() {
  g_client_open_ms = 0;
  g_request_bytes = 0;
  g_first_line_len = 0;
  g_first_line[0] = '\0';
  g_first_line_done = false;
}

void printHeap(const char* tag) {
  Serial.printf(
      "NETDBG,%s,ms=%lu,ap_clients=%u,raw_accepts=%lu,raw_requests=%lu,raw_responses=%lu,"
      "internal=%u,largest_internal=%u,dma=%u,largest_dma=%u,psram=%u\n",
      tag,
      static_cast<unsigned long>(millis()),
      static_cast<unsigned>(WiFi.softAPgetStationNum()),
      static_cast<unsigned long>(g_accept_count),
      static_cast<unsigned long>(g_request_count),
      static_cast<unsigned long>(g_response_count),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
      static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}

void sendResponseAndClose(const char* reason) {
  if (!g_client) return;

  static const char body[] = "raw tcp port 81 ok\n";
  char header[192];
  const int header_len = snprintf(
      header, sizeof(header),
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Content-Length: %u\r\n"
      "Connection: close\r\n"
      "Cache-Control: no-store\r\n"
      "\r\n",
      static_cast<unsigned>(sizeof(body) - 1));

  const size_t wrote_header = g_client.write(
      reinterpret_cast<const uint8_t*>(header),
      header_len > 0 ? static_cast<size_t>(header_len) : 0);
  const size_t wrote_body = g_client.write(
      reinterpret_cast<const uint8_t*>(body), sizeof(body) - 1);
  g_client.flush();

  if (wrote_header == static_cast<size_t>(header_len) &&
      wrote_body == sizeof(body) - 1) {
    ++g_response_count;
  }

  Serial.printf(
      "NETDBG,response,reason=%s,request_bytes=%lu,first_line=\"%s\","
      "header_written=%u,body_written=%u,connected=%u\n",
      reason,
      static_cast<unsigned long>(g_request_bytes),
      g_first_line,
      static_cast<unsigned>(wrote_header),
      static_cast<unsigned>(wrote_body),
      g_client.connected() ? 1U : 0U);

  g_client.stop();
  resetClientState();
  printHeap("after_raw_response");
}

}  // namespace

void tcpTransportDebugBegin() {
  g_server.begin();
  resetClientState();
  g_last_heartbeat_ms = millis();

  const IPAddress ip = WiFi.softAPIP();
  Serial.printf("NETDBG,begin,ip=%u.%u.%u.%u,raw_port=%u\n",
                ip[0], ip[1], ip[2], ip[3], static_cast<unsigned>(kDiagPort));
  Serial.println("NETDBG,test_raw=http://192.168.4.1:81/ test_webserver=http://192.168.4.1/net-probe");
  printHeap("boot");
}

void tcpTransportDebugUpdate() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - g_last_heartbeat_ms) >= 2000UL) {
    g_last_heartbeat_ms = now;
    printHeap("heartbeat");
  }

  if (!g_client || !g_client.connected()) {
    if (g_client) g_client.stop();
    resetClientState();

    WiFiClient incoming = g_server.available();
    if (incoming) {
      g_client = incoming;
      g_client_open_ms = now;
      ++g_accept_count;
      Serial.printf("NETDBG,accept,count=%lu,remote=%s,available=%d\n",
                    static_cast<unsigned long>(g_accept_count),
                    g_client.remoteIP().toString().c_str(),
                    g_client.available());
      printHeap("after_accept");
    }
  }

  if (!g_client || !g_client.connected()) return;

  while (g_client.available() > 0 && g_request_bytes < kMaxRequestBytes) {
    const int v = g_client.read();
    if (v < 0) break;
    const char ch = static_cast<char>(v);
    ++g_request_bytes;

    if (!g_first_line_done) {
      if (ch == '\n') {
        g_first_line_done = true;
        g_first_line[g_first_line_len] = '\0';
      } else if (ch != '\r' && g_first_line_len + 1 < sizeof(g_first_line)) {
        g_first_line[g_first_line_len++] = ch;
        g_first_line[g_first_line_len] = '\0';
      }
    }
  }

  if (g_first_line_done) {
    ++g_request_count;
    Serial.printf("NETDBG,request,count=%lu,bytes=%lu,line=\"%s\"\n",
                  static_cast<unsigned long>(g_request_count),
                  static_cast<unsigned long>(g_request_bytes),
                  g_first_line);
    sendResponseAndClose("request_line");
    return;
  }

  if (g_request_bytes >= kMaxRequestBytes) {
    ++g_request_count;
    sendResponseAndClose("request_limit");
    return;
  }

  if (g_client_open_ms != 0 &&
      static_cast<uint32_t>(now - g_client_open_ms) >= kClientTimeoutMs) {
    Serial.printf("NETDBG,client_timeout,bytes=%lu,connected=%u\n",
                  static_cast<unsigned long>(g_request_bytes),
                  g_client.connected() ? 1U : 0U);
    sendResponseAndClose("timeout");
  }
}
