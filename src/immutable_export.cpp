#include "immutable_export.h"
#include "runtime_diagnostics.h"
#include <esp_system.h>

bool ImmutableExport::reserveMetadata() {
  // Reserve before the camera and Wi-Fi allocate scattered PSRAM blocks.
  return metadata_.reserve(PsramLogger::kMetadataJsonReserveBytes);
}
bool ImmutableExport::begin(PsramLogger& logger) {
  logger_ = &logger; boot_nonce_ = esp_random();
  if (!reserveMetadata()) return false;
  return xTaskCreatePinnedToCore(entry, "log_export", 12288, this, 1, &task_, 0) == pdPASS;
}
ImmutableExport::Status ImmutableExport::status() const {
  portENTER_CRITICAL(&mux_); const auto copy = status_; portEXIT_CRITICAL(&mux_); return copy;
}
bool ImmutableExport::prepare() {
  const auto s = status();
  if (s.phase == Phase::Building || s.phase == Phase::Ready) return true; // idempotent retry
  if (!task_ || !logger_->rwlogDownloadable()) return false;
  portENTER_CRITICAL(&mux_);
  status_ = Status{}; status_.phase = Phase::Building;
  portEXIT_CRITICAL(&mux_);
  xTaskNotifyGive(task_); return true;
}
bool ImmutableExport::reset() {
  if (status().phase == Phase::Building) return false;
  metadata_.clear();
  portENTER_CRITICAL(&mux_); status_ = Status{}; portEXIT_CRITICAL(&mux_); return true;
}
void ImmutableExport::loop() {
  for (;;) {
    RuntimeDiag::beat(RuntimeDiag::Lane::Export);
    RuntimeDiag::phase(RuntimeDiag::Lane::Export, RuntimeDiag::Phase::Wait);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); build();
  }
}
void ImmutableExport::build() {
  RuntimeDiag::phase(RuntimeDiag::Lane::Export, RuntimeDiag::Phase::ExportMetadata);
  if (!logger_->buildMetadataJson(metadata_)) {
    portENTER_CRITICAL(&mux_);
    snprintf(status_.error, sizeof(status_.error), "metadata_build_failed_at_%lu_bytes",
        static_cast<unsigned long>(metadata_.length()));
    status_.phase = Phase::Error;
    portEXIT_CRITICAL(&mux_); return;
  }
  RuntimeDiag::phase(RuntimeDiag::Lane::Export, RuntimeDiag::Phase::ExportCrc);
  header_ = logger_->buildHeader(metadata_.length());
  export_protocol::Span spans[] = {
    {reinterpret_cast<const uint8_t*>(&header_), sizeof(header_)},
    {reinterpret_cast<const uint8_t*>(metadata_.c_str()), metadata_.length()},
    {logger_->sampleBytes(), header_.sample_count * sizeof(LogSample)}
  };
  crc_ = 0;
  uint32_t hashed = 0;
  portENTER_CRITICAL(&mux_); status_.bytes = header_.crc_offset + 4; portEXIT_CRITICAL(&mux_);
  for (const auto& span : spans) {
    for (size_t offset = 0; offset < span.length;) {
      const size_t length = span.length - offset < 4096 ? span.length - offset : 4096;
      crc_ = export_protocol::crc32(crc_, span.data + offset, length);
      offset += length; hashed += length;
      portENTER_CRITICAL(&mux_); status_.hashed_bytes = hashed; portEXIT_CRITICAL(&mux_);
      RuntimeDiag::beat(RuntimeDiag::Lane::Export, hashed);
      vTaskDelay(1);
    }
  }
  // Publish only after the complete CRC and every byte/offset are immutable.
  RuntimeDiag::phase(RuntimeDiag::Lane::Export, RuntimeDiag::Phase::Publish);
  Status complete;
  complete.bytes = header_.crc_offset + 4; complete.hashed_bytes = complete.bytes;
  complete.crc = crc_;
  snprintf(complete.token, sizeof(complete.token), "%08lx%08lx",
      static_cast<unsigned long>(boot_nonce_), static_cast<unsigned long>(++generation_));
  logger_->downloadFilename(complete.filename, sizeof(complete.filename));
  complete.phase = Phase::Ready;
  portENTER_CRITICAL(&mux_); status_ = complete; portEXIT_CRITICAL(&mux_);
}
bool ImmutableExport::chunk(const char* token, uint32_t offset, uint32_t length, uint8_t* out) const {
  const auto s = status();
  if (s.phase != Phase::Ready || !token || strcmp(s.token, token)) return false;
  const export_protocol::Span spans[] = {
    {reinterpret_cast<const uint8_t*>(&header_), sizeof(header_)},
    {reinterpret_cast<const uint8_t*>(metadata_.c_str()), metadata_.length()},
    {logger_->sampleBytes(), header_.sample_count * sizeof(LogSample)},
    {reinterpret_cast<const uint8_t*>(&crc_), sizeof(crc_)}
  };
  return export_protocol::copy(spans, 4, s.bytes, offset, out, length);
}
