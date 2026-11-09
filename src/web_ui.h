#pragma once
#include "bounded_web_server.h"
#include "offline_run_session.h"
#include "run_control_worker.h"
#include "foot_observer.h"
#include "immutable_export.h"

class WebUi {
 public:
  void begin(BoundedWriteWebServer&, RunControlWorker&, FootObserver&, ImmutableExport&);
  void update();
 private:
  friend class OfflineRunSession;
  bool stopServer();
  bool queueStart();
  OfflineRunSession::StartResult startResult() const;
  bool runActive() const;
  void cancelStart();
  bool restoreTransport();
  void transportReady();
  bool quietResponse();
  void status();
  void command(RunControlWorker::Command);
  void manifest();
  void chunk();
  void previewCapture();
  void previewChunk();
  bool previewAllowed() const;
  BoundedWriteWebServer* server_ = nullptr;
  RunControlWorker* control_ = nullptr;
  FootObserver* feet_ = nullptr;
  ImmutableExport* export_ = nullptr;
  uint8_t chunk_buffer_[export_protocol::kChunkBytes + 16] = {};
  uint8_t* preview_buffer_ = nullptr;
  uint32_t preview_token_ = 0;
  uint32_t boot_id_ = 0;
  OfflineRunSession offline_;
  uint32_t offline_poll_ms_ = 0;
  uint32_t start_command_id_ = 0;
  float start_input_peak_percent_ = autonomous_input_percent::kDefaultPercent;
  int16_t start_input_current_mA_ = autonomous_input_current::kDefaultMa;
  uint16_t start_brake_max_ms_ = autonomous_brake_limit::kDefaultMs;
  bool start_peak_adaptation_enabled_ = true;
  bool start_energy_bridge_enabled_ = true;
  bool start_negative_probe_mode_ = false;
  float start_target_deg_ = Config::ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG;
  bool start_submitted_ = false, cancel_sent_ = false;
};
