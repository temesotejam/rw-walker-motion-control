#pragma once
#include <stddef.h>
#include <stdint.h>

#define RUNTIME_VERSION "1.0.99-alternate-foot-trial"
#if defined(FOOT_EARLY_TRIAL) && FOOT_EARLY_TRIAL
#if defined(ANGLE_EARLY_TRIAL) && ANGLE_EARLY_TRIAL
#if defined(WALK_SMOOTH_TRIAL) && WALK_SMOOTH_TRIAL
#if defined(SYMMETRIC_RESCUE_TRIAL) && SYMMETRIC_RESCUE_TRIAL
#if defined(EARLY_RESCUE_TRIAL) && EARLY_RESCUE_TRIAL
#if defined(SWAY_BALANCE_TRIAL) && SWAY_BALANCE_TRIAL
#if defined(APPROACH_RESCUE_TRIAL) && APPROACH_RESCUE_TRIAL
#if defined(FAST_CROSSING_SHADOW) && FAST_CROSSING_SHADOW
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-fast-crossing-shadow-v9"
#elif defined(FAST_CROSSING_TRIAL) && FAST_CROSSING_TRIAL
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-fast-crossing-v8"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-approach-rescue-v7"
#endif
#else
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
#if defined(PULSE_LIFECYCLE_FIX) && PULSE_LIFECYCLE_FIX
#if defined(WEB_PAGE_TRANSPORT_FIX) && WEB_PAGE_TRANSPORT_FIX
#if defined(WALKING_AUDIT_PSRAM) && WALKING_AUDIT_PSRAM
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
#if defined(WALKING_ADMISSION_V19) && WALKING_ADMISSION_V19
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
#if defined(EXPORT_RETRY_FIX) && EXPORT_RETRY_FIX
#if defined(EXPORT_COOPERATIVE_FIX) && EXPORT_COOPERATIVE_FIX
#if defined(CAMERA_FRAME_READY_V21) && CAMERA_FRAME_READY_V21
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
#if defined(FOOT_COPY_DELTA_V27) && FOOT_COPY_DELTA_V27
#if defined(FOOT_TEMPLATE_PRUNE_V28) && FOOT_TEMPLATE_PRUNE_V28
#if defined(FOOT_OVERLAP_ALWAYS_V29) && FOOT_OVERLAP_ALWAYS_V29
#if defined(FOOT_RECEIVE_DIAG_V30) && FOOT_RECEIVE_DIAG_V30
#if defined(FOOT_RECEIVE_CORE1_V31) && FOOT_RECEIVE_CORE1_V31
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-receive-core1-v31"
#elif defined(SUPPORT_TRANSFER_V39) && SUPPORT_TRANSFER_V39
#if WALKING_TURN_OBSERVER
#if SUPPORT_TRANSFER_ARC_FEEDBACK
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-support-transfer-v39-turn2"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-support-transfer-v39-turn2-gait-only"
#endif
#elif SUPPORT_TRANSFER_CARRY2
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-support-transfer-v39-carry2"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-support-transfer-v39-carry1"
#endif
#elif defined(FOOT_PERIODIC_V38) && FOOT_PERIODIC_V38
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-periodic40-v38"
#elif defined(FOOT_CADENCE20_V37) && FOOT_CADENCE20_V37
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-cadence20-v37"
#elif defined(FOOT_STABLE40_V36) && FOOT_STABLE40_V36
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-stable40-v36"
#elif defined(FOOT_LIVE_STATUS_V35) && FOOT_LIVE_STATUS_V35
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-live-status-v35"
#elif defined(FOOT_WORKER26_V34) && FOOT_WORKER26_V34
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-worker26-v34"
#elif defined(FOOT_NATIVE_HB80_V33) && FOOT_NATIVE_HB80_V33
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-hblank80-v33"
#elif defined(FOOT_HISTOGRAM_V32) && FOOT_HISTOGRAM_V32
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-histogram-v32"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-receive-diag-v30"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-overlap-always-v29"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-template-prune-v28"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-native-stream-v27"
#endif
#else
#if defined(FOOT_EXPORT_WEBFIX) && FOOT_EXPORT_WEBFIX
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-native-stream-v26-webfix1"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-native-stream-v26"
#endif
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-native-v25"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-overlap-v24"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-capture-v23"
#endif
#elif defined(FOOT_RECEIPT_AUDIT) && FOOT_RECEIPT_AUDIT
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-speed-v22-fix1-receipt"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-speed-v22-fix1"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-speed-v22"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-frame-ready-v21"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-fast-v20-downloadfix2"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-fast-v20-downloadfix1"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-foot-fast-v20"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-energy-admission-v19"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-energy-transfer-v18"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "walking-return-energy-v17-rangefix1"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "energy-pulse-lifecycle-v16-webfix2"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "energy-pulse-lifecycle-v16-webfix1"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "energy-pulse-lifecycle-v16"
#endif
#elif defined(EARLY_ENERGY_MAIN) && EARLY_ENERGY_MAIN
#define RUNTIME_DIAGNOSTIC_BUILD_ID "early-energy-main-v15"
#elif defined(FOOT_SUPPORT_ENERGY) && FOOT_SUPPORT_ENERGY
#define RUNTIME_DIAGNOSTIC_BUILD_ID "foot-support-energy-v14"
#elif defined(FOOT_SUPPORT_TRANSFER) && FOOT_SUPPORT_TRANSFER
#define RUNTIME_DIAGNOSTIC_BUILD_ID "foot-support-transfer-v13"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "foot-support-local-v12"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "foot-support-assist-v11"
#endif
#elif defined(CAD_SUPPORT_SHADOW) && CAD_SUPPORT_SHADOW
#define RUNTIME_DIAGNOSTIC_BUILD_ID "cad-support-shadow-v10"
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-balance-trim-v6"
#endif
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-early-rescue-v5"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-symmetric-rescue-v4"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-early-smooth-v3"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "angle-early-weak-zero-v2"
#endif
#else
#define RUNTIME_DIAGNOSTIC_BUILD_ID "early-input-geometry-v1"
#endif
#endif

namespace RuntimeDiag {
struct ResetSnapshot {
  uint32_t reason = 0;
  const char* name = "UNKNOWN";
  bool previous_export_valid = false;
  uint32_t previous_export_phase = 0, previous_export_bytes = 0;
};
struct MemorySnapshot { uint32_t at_ms, internal_free, internal_min, largest, dma_free, psram_free; };
struct ExecutionSnapshot { int32_t camera_init_core = -1, camera_deinit_core = -1, wifi_event_core = -1; };
enum class Lane : uint32_t { Control, Imu, Camera, Http, Roller, Export, Count };
enum class Stage : uint32_t {
  UsbWindow, M5, Logger, Imu, Camera, Roller, Runner, Control, Feet, Export, Web, Ready
};
enum class Phase : uint32_t {
  Unseen, Wait, ControlService, ControlImu, ControlCommand, ControlRunner,
  ControlFinish, ControlIdle, Snapshot, Publish, ImuRead, ImuAudit,
  CameraCapture, CameraProcess, CameraRelease, CameraStop,
  HttpPoll, HttpRoot, HttpStatus, HttpCommand, HttpManifest, HttpChunk,
  HttpJson, HttpMemory, HttpSend, HttpClose, RollerInit, RollerIo,
  ExportMetadata, ExportCrc
};
#if defined(ARDUINO_ARCH_ESP32)
void begin();
ResetSnapshot resetSnapshot();
bool enabled();
void setEnabled(bool enabled);
bool runActive();
void setRunActive(bool active);
bool takeWifiRecoveryRequest();
void wifiTransportRestored();
MemorySnapshot memorySnapshot();
ExecutionSnapshot executionSnapshot();
void cameraLifecycleCore(bool initialize, int core);
void boot(Stage stage);
void result(bool ok);
// Single task owns each lane. These probes never log or acquire application locks.
void phase(Lane lane, Phase phase);
Phase currentPhase(Lane lane);
// Real-time owners defer the periodic stack scan during a run. Heartbeat and
// phase publication continue; the first due idle beat refreshes the watermark.
void beat(Lane lane, uint32_t detail = 0, bool allow_stack_scan = true);
void sampleMemory(); // HTTP owner only; deliberately excluded from the USB observer.
void cameraDriverTask(uint32_t requested, uint32_t allocated, bool created);
void cameraDriverStack(uint32_t free_bytes);
void cameraDriverStopped();
void wifiEvent(uint32_t event, int client_change = 0, int ap_active = -1);
void httpWriteResult(size_t sent, size_t wanted, int error);
void pollFallback(); // Only used if the independent observer task could not be created.
#else
inline ResetSnapshot resetSnapshot() { return {}; }
inline bool& enabledState() { static bool value = false; return value; }
inline bool& runState() { static bool value = false; return value; }
inline bool enabled() { return enabledState(); }
inline void setEnabled(bool value) { enabledState() = value; }
inline bool runActive() { return runState(); }
inline void setRunActive(bool value) { runState() = value; }
inline void wifiTransportRestored() {}
inline void httpWriteResult(size_t, size_t, int) {}
inline MemorySnapshot memorySnapshot() { return {}; }
inline ExecutionSnapshot executionSnapshot() { return {}; }
inline void cameraLifecycleCore(bool, int) {}
inline void phase(Lane, Phase) {}
inline Phase currentPhase(Lane) { return Phase::Unseen; }
inline void beat(Lane, uint32_t = 0, bool = true) {}
inline void cameraDriverTask(uint32_t, uint32_t, bool) {}
inline void cameraDriverStack(uint32_t) {}
inline void cameraDriverStopped() {}
#endif
inline bool heavyAllowed() { return enabled() && !runActive(); }
struct Scope {
  Lane lane; Phase previous;
  Scope(Lane l, Phase p) : lane(l), previous(currentPhase(l)) { phase(l, p); }
  ~Scope() { phase(lane, previous); }
};
}
