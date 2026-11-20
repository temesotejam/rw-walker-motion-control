#pragma once
#include "psram_logger.h"
#include "export_protocol.h"
#include <freertos/task.h>

// Web owns lifecycle. Export task reads sealed logs only, never runs during a
// measurement. Start/Clear are rejected until this worker has returned to idle.
class ImmutableExport {
 public:
  enum class Phase : uint8_t { Empty, Building, Ready, Error };
  struct Status {
    Phase phase = Phase::Empty;
    uint16_t run_id = 0;
    uint32_t bytes = 0, hashed_bytes = 0, crc = 0;
    uint32_t metadata_bytes = 0;
    char token[24] = {}, filename[80] = {}, error[80] = {};
  };
  bool reserveMetadata();
  bool begin(PsramLogger& logger);
  bool ready() const { return task_ != nullptr; }
  bool prepare();
  bool reset();
  Status status() const;
  bool chunk(const char* token, uint32_t offset, uint32_t length, uint8_t* out) const;
 private:
  static void entry(void* p) { static_cast<ImmutableExport*>(p)->loop(); }
  void loop();
  void build();
  static void metadataProgress(void* context, size_t bytes);
  uint32_t metadata_yield_us_ = 0;
  PsramLogger* logger_ = nullptr;
  TaskHandle_t task_ = nullptr;
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Status status_;
  PsramString metadata_;
  RwLogFileHeader header_{};
  uint32_t crc_ = 0, boot_nonce_ = 0, generation_ = 0;
};
