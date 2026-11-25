#pragma once
#include "bounded_web_server.h"
#include "offline_run_session.h"
#include "run_control_worker.h"
#include "foot_observer.h"
#include "immutable_export.h"
#include "export_transport_recovery.h"
#include "wifi_transport_restart.h"

class WebUi {
 public:
  void begin(BoundedWriteWebServer&, RunControlWorker&, FootObserver&, ImmutableExport&);
  void update();
  bool requestWifiRecovery(); // Call only on the HTTP owner, also for USB requests.
 private:
  friend class OfflineRunSession;
  friend class WifiTransportRestart;
  bool requestWifiRecovery(bool manual);
  void updateExportPriority();
  bool deferCameraDiagnostics();
  void closeHttpTransport();
  bool stopWifiRadio();
  bool startWifiAp();
  bool startHttpListener();
  bool stopServer();
  bool queueStart();
  OfflineRunSession::StartResult startResult();
  bool runActive() const;
  void cancelStart();
  bool restoreTransport();
  void transportReady();
  bool prepareCompletedExport();
  bool quietResponse();
  void status();
#if defined(FOOT_HISTOGRAM_V32) && FOOT_HISTOGRAM_V32
  void cameraLatency();
#endif
  void command(RunControlWorker::Command);
  void manifest();
  void confirmExport();
  bool reserveChunkBuffer();
  bool makeExportPacket(const char* token, uint32_t offset, uint32_t length);
  bool sendCompressedWindow(const char* token, uint32_t offset, uint32_t length, const char*& reason);
  void chunk();
  void previewCapture();
  void previewChunk();
  bool previewAllowed() const;
  BoundedWriteWebServer* server_ = nullptr;
  RunControlWorker* control_ = nullptr;
  FootObserver* feet_ = nullptr;
  ImmutableExport* export_ = nullptr;
  PsramString status_json_;
#if defined(FOOT_HISTOGRAM_V32) && FOOT_HISTOGRAM_V32
  uint32_t latency_status_requests_=0,latency_preview_requests_=0;
#endif
  uint8_t* chunk_buffer_ = nullptr;
  uint8_t* preview_buffer_ = nullptr;
  uint32_t preview_token_ = 0;
  uint32_t boot_id_ = 0;
  OfflineRunSession offline_;
  ExportTransportRecovery recovery_;
  bool export_priority_active_ = false;
  ReadyHttpStallRecovery ready_http_stall_;
  WifiTransportRestart wifi_restart_;
  uint32_t recovery_poll_ms_ = 0;
  bool listener_rebind_pending_ = false;
  uint32_t last_listener_bind_ms_ = 0;
  bool start_observed_ = false;
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
