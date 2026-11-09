#!/usr/bin/env python3
from pathlib import Path
import hashlib

ROOT = Path(__file__).resolve().parents[1]
web = (ROOT / "src/web_ui.cpp").read_text(encoding="utf-8").replace("\r\n", "\n")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8").replace("\r\n", "\n")

def git_blob_sha_bytes(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def git_blob_sha(path: Path) -> str:
    return git_blob_sha_bytes(path.read_bytes())

def extract_function(text: str, signature: str) -> str:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        c = text[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    raise AssertionError(f"unterminated function: {signature}")

old_constants = "static constexpr size_t STREAM_CHUNK_BYTES = 4096;\n"
new_constants = "// RWLOG download transport only. Small chunks preserve TCP pbuf headroom while\n// the camera driver remains initialized. ENOMEM can shrink this to 128/64 B.\nstatic constexpr size_t STREAM_CHUNK_BYTES = 256;\nstatic constexpr size_t STREAM_MIN_CHUNK_BYTES = 64;\nstatic constexpr uint32_t STREAM_SUCCESS_PACE_MS = 2UL;\nstatic constexpr uint32_t STREAM_NO_PROGRESS_TIMEOUT_MS = 15000UL;\n"
old_write = "bool PsramLogger::writeBytes(WebServer& server, const uint8_t* data, size_t len) {\n  WiFiClient client = server.client();\n  while (len > 0) {\n    const size_t n = len > STREAM_CHUNK_BYTES ? STREAM_CHUNK_BYTES : len;\n    if (client.write(data, n) != n) return false;\n    data += n;\n    len -= n;\n    delay(0);\n  }\n  return true;\n}"
new_write = "bool PsramLogger::writeBytes(WebServer& server, const uint8_t* data, size_t len) {\n  WiFiClient client = server.client();\n  const int socket_fd = client.fd();\n  if (socket_fd < 0 || !client.connected()) {\n    rwlogDownloadDiagFail(\"socket_not_connected\", 0);\n    return false;\n  }\n\n  // Header bytes live in internal RAM and have always transmitted successfully.\n  // Metadata String storage and the 6 MiB sample buffer can live in PSRAM.\n  // Always stage each body chunk through this small internal task-stack buffer\n  // before giving it to lwIP, so all four RWLOG sections use the same source\n  // memory class at the socket boundary.\n  alignas(4) uint8_t staging[STREAM_CHUNK_BYTES];\n\n  size_t chunk_limit = STREAM_CHUNK_BYTES;\n  uint32_t last_progress_ms = millis();\n  while (len > 0) {\n    const size_t want = len < chunk_limit ? len : chunk_limit;\n    memcpy(staging, data, want);\n    rwlogDownloadDiagRequest(want, chunk_limit);\n\n    errno = 0;\n    const int result = ::send(\n        socket_fd,\n        reinterpret_cast<const void*>(staging),\n        want,\n        MSG_DONTWAIT);\n\n    if (result > 0) {\n      const size_t written = static_cast<size_t>(result);\n      data += written;\n      len -= written;\n      last_progress_ms = millis();\n      rwlogDownloadDiagProgress(written);\n      // Match the bounded WebServer path: do not burst-fill lwIP pbufs.\n      // 256 B every 2 ms is still ~128 kB/s before protocol overhead, so a\n      // 1 MB RWLOG has an ~8 s floor rather than exhausting the send pool.\n      delay(STREAM_SUCCESS_PACE_MS);\n      continue;\n    }\n\n    const int error = errno;\n    const bool retryable =\n        result == 0 ||\n        (result < 0 &&\n         (error == EAGAIN || error == EWOULDBLOCK || error == ENOMEM));\n\n    if (retryable && client.connected()) {\n      rwlogDownloadDiagRetry(error);\n      if (error == ENOMEM && chunk_limit > STREAM_MIN_CHUNK_BYTES) {\n        chunk_limit /= 2;\n        if (chunk_limit < STREAM_MIN_CHUNK_BYTES) chunk_limit = STREAM_MIN_CHUNK_BYTES;\n        rwlogDownloadDiagShrink(chunk_limit);\n      }\n      if (static_cast<uint32_t>(millis() - last_progress_ms) >\n          STREAM_NO_PROGRESS_TIMEOUT_MS) {\n        rwlogDownloadDiagFail(\"no_progress_timeout\", error);\n        return false;\n      }\n      delay(2);\n      continue;\n    }\n\n    rwlogDownloadDiagFail(\n        client.connected() ? \"socket_send_error\" : \"client_disconnected\",\n        error);\n    return false;\n  }\n  return true;\n}"
old_stream = "bool PsramLogger::streamRwLog(WebServer& server) {\n  if (!rwlogDownloadable()) {\n    last_error_ = \"rwlog_not_ready\";\n    server.send(409, \"text/plain\", last_error_);\n    return false;\n  }\n\n  downloading_ = true;\n  const String metadata = buildMetadataJson();\n  const RwLogFileHeader header = buildHeader(metadata.length());\n  const uint32_t crc = calculateCrc(header, metadata);\n  char filename[72];\n  downloadFilename(filename, sizeof(filename));\n\n  server.sendHeader(\"Content-Disposition\", String(\"attachment; filename=\\\"\") + filename + \"\\\"\");\n  server.sendHeader(\"Cache-Control\", \"no-store, no-cache, must-revalidate, max-age=0\");\n  server.setContentLength(header.crc_offset + sizeof(crc));\n  server.send(200, \"application/octet-stream\", \"\");\n\n  bool ok = true;\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&header), sizeof(header));\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(metadata.c_str()), metadata.length());\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(samples_), sample_count_ * sizeof(LogSample));\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));\n\n  downloading_ = false;\n  last_error_ = ok ? \"\" : \"rwlog_stream_failed\";\n  return ok;\n}"
new_stream = "bool PsramLogger::streamRwLog(WebServer& server) {\n  if (!rwlogDownloadable()) {\n    last_error_ = \"rwlog_not_ready\";\n    server.send(409, \"text/plain\", last_error_);\n    return false;\n  }\n\n  downloading_ = true;\n  const uint32_t prepare_start_ms = millis();\n  Serial.printf(\"RWLOGDL,prepare_begin,samples=%u,psram_free=%u\\n\",\n                static_cast<unsigned>(sample_count_),\n                static_cast<unsigned>(ESP.getFreePsram()));\n  const String metadata = buildMetadataJson();\n  const RwLogFileHeader header = buildHeader(metadata.length());\n  const uint32_t crc = calculateCrc(header, metadata);\n  char filename[72];\n  downloadFilename(filename, sizeof(filename));\n  Serial.printf(\"RWLOGDL,prepare_end,ms=%lu,metadata=%u,total=%u,file=%s\\n\",\n                static_cast<unsigned long>(millis() - prepare_start_ms),\n                static_cast<unsigned>(metadata.length()),\n                static_cast<unsigned>(header.crc_offset + sizeof(crc)),\n                filename);\n\n  server.sendHeader(\"Content-Disposition\", String(\"attachment; filename=\\\"\") + filename + \"\\\"\");\n  server.sendHeader(\"Cache-Control\", \"no-store, no-cache, must-revalidate, max-age=0\");\n  server.sendHeader(\"Connection\", \"close\");\n  const uint32_t total_bytes = header.crc_offset + sizeof(crc);\n  const uint32_t sample_bytes = sample_count_ * sizeof(LogSample);\n  rwlogDownloadDiagBegin(total_bytes, metadata.length(), sample_bytes);\n\n  server.setContentLength(total_bytes);\n  server.send(200, \"application/octet-stream\", \"\");\n\n  const uint32_t stream_start_ms = millis();\n  bool ok = true;\n  rwlogDownloadDiagSetPhase(\"header\");\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&header), sizeof(header));\n  rwlogDownloadDiagSetPhase(\"metadata\");\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(metadata.c_str()), metadata.length());\n  rwlogDownloadDiagSetPhase(\"samples\");\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(samples_), sample_bytes);\n  rwlogDownloadDiagSetPhase(\"crc\");\n  ok = ok && writeBytes(server, reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));\n\n  downloading_ = false;\n  last_error_ = ok ? \"\" : \"rwlog_stream_failed\";\n  rwlogDownloadDiagFinish(ok);\n  if (!ok) {\n    WiFiClient failed_client = server.client();\n    failed_client.stop();\n  }\n  Serial.printf(\"RWLOGDL,stream_end,ok=%u,ms=%lu,error=%s\\n\",\n                ok ? 1U : 0U,\n                static_cast<unsigned long>(millis() - stream_start_ms),\n                ok ? \"none\" : last_error_);\n  return ok;\n}"
old_begin = "function beginDownload(){\n  downloading=true;\n  apply(lastStatus);\n  setTimeout(()=>{\n    downloading=false;\n    refresh();\n  },3000);\n}"
new_begin = "function beginDownload(){\n  // Keep the browser's 1 Hz status traffic off the AP while the native\n  // attachment transfer owns the socket. No fetch/blob buffering is used.\n  downloading=true;\n  apply(lastStatus);\n  setTimeout(()=>{\n    downloading=false;\n    refresh();\n  },60000);\n}"

assert git_blob_sha(ROOT / "src/psram_logger.h") == "63a16781660142a5e3a82721f90cadd9cc2ce9b7"
assert git_blob_sha(ROOT / "tools/convert_rwlog_to_csv.py") == "7a2c1229376e1ec204a3d9305cedf0b67af7a231"

assert new_constants in logger
assert extract_function(logger, "bool PsramLogger::writeBytes(") == new_write
assert "MSG_DONTWAIT" in logger
assert "rwlogDownloadDiagFail" in logger
assert "rwlogDownloadDiagRetry" in logger
assert "STREAM_CHUNK_BYTES = 256" in logger
assert "STREAM_MIN_CHUNK_BYTES = 64" in logger
assert "STREAM_SUCCESS_PACE_MS = 2UL" in logger
assert "delay(STREAM_SUCCESS_PACE_MS);" in logger
assert "rwlogDownloadDiagShrink" in logger
assert "alignas(4) uint8_t staging[STREAM_CHUNK_BYTES];" in logger
assert "memcpy(staging, data, want);" in logger
assert 'server_->on("/rwlog-download-health", HTTP_GET' in web
assert extract_function(logger, "bool PsramLogger::streamRwLog(") == new_stream
assert 'server.sendHeader("Connection", "close");' in logger
assert "RWLOGDL,prepare_begin" in logger
assert "RWLOGDL,stream_end" in logger

compact_start = logger.index("  // Compact Autonomous metadata begin\n")
compact_end = logger.index("  // Compact Autonomous metadata end\n", compact_start) + len("  // Compact Autonomous metadata end\n")
compact_block = logger[compact_start:compact_end]
for token in (
    'autonomous_compact_v1',
    'energy_control_autonomous_peak_events',
    'energy_control_autonomous_zero_cross_events',
    'v46n_imu_acquisition',
    'v46p_control_worker',
    'v46u_current_timing',
    'metadata_omitted_sections',
):
    assert token in compact_block, token
for token in (
    'calibration_probe_events',
    'q_ident_events',
    'e2_shadow_peak_events',
    'q1_shadow_events',
    'v46s_solver_audit',
    'v46k_timing_probe_events',
    '\"columns\"',
):
    assert token not in compact_block, token

normalized_logger = logger[:compact_start] + logger[compact_end:]
normalized_logger = normalized_logger.replace('#include "rwlog_download_diag.h"\n', "", 1)
normalized_logger = normalized_logger.replace("#include <errno.h>\n#include <lwip/sockets.h>\n", "", 1)
normalized_logger = normalized_logger.replace(new_constants, old_constants, 1)
normalized_logger = normalized_logger.replace(new_write, old_write, 1)
normalized_logger = normalized_logger.replace(new_stream, old_stream, 1)
assert git_blob_sha_bytes(normalized_logger.encode()) == "e61167fba2869ad948df37d999a7bcb6e5346817"

assert 'server_->on("/download/rwlog", HTTP_GET, [this]() { handleRwLog(); });' in web
assert extract_function(web, "void WebUi::handleRwLog()") == """void WebUi::handleRwLog() {
  if (run_control.active()) { server_->send(409, "text/plain", "run_in_progress"); return; }
  if (runner_->running()) {
    server_->send(409, "text/plain", "measurement_running");
    return;
  }
  logger_->streamRwLog(*server_);
}"""
assert new_begin in web
assert "if(downloading||refreshInFlight)return;" in web

normalized_web = web.replace('#include "rwlog_download_diag.h"\n', "", 1)
normalized_web = normalized_web.replace('''  server_->on("/rwlog-download-health", HTTP_GET, [this]() {
    server_->sendHeader("Cache-Control", "no-store");
    server_->send(200, "application/json", rwlogDownloadDiagnosticsJson());
  });
''', "", 1)
normalized_web = normalized_web.replace(new_begin, old_begin, 1)
normalized_web = normalized_web.replace(
    "if(downloading||refreshInFlight)return;",
    "if(refreshInFlight)return;",
    1,
)
assert git_blob_sha_bytes(normalized_web.encode()) == "c63bb11581c8252fe92f151fb97c9208175bd336"

print("RWLOG download recovery PASS: format/converter frozen; native download + partial-write retry + polling suppression")
