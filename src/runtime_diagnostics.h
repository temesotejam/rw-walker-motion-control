#pragma once
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
#define RUNTIME_DIAGNOSTIC_BUILD_ID "energy-pulse-lifecycle-v16"
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
bool enabled();
void setEnabled(bool enabled);
bool runActive();
void setRunActive(bool active);
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
void pollFallback(); // Only used if the independent observer task could not be created.
#else
inline bool& enabledState() { static bool value = false; return value; }
inline bool& runState() { static bool value = false; return value; }
inline bool enabled() { return enabledState(); }
inline void setEnabled(bool value) { enabledState() = value; }
inline bool runActive() { return runState(); }
inline void setRunActive(bool value) { runState() = value; }
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
