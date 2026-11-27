#include <Arduino.h>
#include <M5Unified.h>
#include "esp_timer.h"
#include "experiment_runner.h"
#include "imu_manager.h"
#include "periodic_foot_stream.h"
#include "roller485_manager.h"
#include "runtime_diagnostics.h"
#include "web_ui.h"
#if MOTORFREE_ARCHIVE
#include <WiFi.h>
#include <atomic>
#include "archive_layout.h"
extern bool replayArchiveSamples(PsramLogger&);
extern void replayMatchInternalHeap();
#if MOTORFREE_MEASURE
extern bool beginMeasuredReplay(PsramLogger&, uint16_t);
extern bool advanceMeasuredReplay(PsramLogger&, uint32_t&, bool&);
#endif
#endif

// The implementation files for the motor driver, walking runner and IMU manager are
// excluded from this link. Only these data-only metadata accessors are supplied.
Roller485Manager roller;
RollerTelemetry Roller485Manager::telemetrySnapshot() const { return {}; }
ExperimentRunner runner;
void ExperimentRunner::appendFootSupportMetadata(PsramString& json) const {
  json += ",\"communication_fixture\":{\"synthetic\":true,\"motor_driver_linked\":false,\"sample_count\":10000}";
}
ImuManager imu;
void ImuManager::appendAcquisitionDiagnostics(PsramString& json) const {
  json += "{\"synthetic\":true,\"sensor_acquisition\":false}";
}

PsramLogger logger;
RunControlWorker run_control;
FootObserver feet;
static OneShotCamera camera;
static BoundedWriteWebServer server(Config::HTTP_PORT);
static ImmutableExport sealed;
static WebUi web;
#if MOTORFREE_ARCHIVE
static constexpr uint16_t runId = replay_layout::kRunId;
static std::atomic<bool> pressureReady{false};
#else
static constexpr uint16_t runId = 61111;
#endif
static uint32_t cameraStarted;
static bool fixtureReady;

#if MOTORFREE_MEASURE
static enum class ReplayState { Ready, Recording, Finished, Failed } replayState = ReplayState::Ready;
static uint16_t measurementId = runId - 1;
static uint32_t measurementElapsedMs;
#endif

static bool seedFixture() {
#if MOTORFREE_ARCHIVE
  return replayArchiveSamples(logger);
#else
  logger.startRun(runId, 1000000, 0, 0, 0, false, 0, 0, 0, true);
  uint32_t noise = 511;
  for (uint32_t i = 0; i < 10000; ++i) {
    noise = 1664525u * noise + 1013904223u;
    LogSample sample{};
    sample.time_us = 1000000 + i * 3000;
    sample.t_test_ms = i * 3;
    sample.state_id = 3;
    sample.ax_mg = static_cast<int16_t>(noise >> 18) - 8192;
    sample.ay_mg = static_cast<int16_t>((noise >> 6) & 0x3fff) - 8192;
    sample.az_mg = static_cast<int16_t>(1000 + (i % 31));
    sample.physical_roll_abs_cdeg = static_cast<int16_t>(i % 1601) - 800;
    sample.gx_cdps = static_cast<int16_t>(noise >> 16);
    sample.roller_speed_sequence = i;
    sample.roller_speed_sample_time_us = sample.time_us - 73;
    sample.imu_update_dt_us = 3000;
    if (!logger.addSample(sample)) return false;
    if (i % 500 == 0) delay(1);
  }
  logger.seal();
  return logger.rwlogDownloadable();
#endif
}
static bool idleStep(void*) {
  run_control.takeStopRequest();
  const auto cmd = run_control.takeCommand();
#if MOTORFREE_MEASURE
  if (cmd == RunControlWorker::Command::Start && replayState == ReplayState::Ready) {
    const bool ok = measurementId < UINT16_MAX && beginMeasuredReplay(logger, ++measurementId);
    measurementElapsedMs = 0;
    replayState = ok ? ReplayState::Recording : ReplayState::Failed;
    run_control.completeCommand(ok, ok ? "simulated_recording" : "replay_init_failed");
  } else if (cmd == RunControlWorker::Command::Clear && replayState != ReplayState::Recording) {
    logger.clear();
    measurementElapsedMs = 0;
    replayState = ReplayState::Ready;
    run_control.completeCommand(true, "simulated_data_cleared");
  } else
#endif
  if (cmd != RunControlWorker::Command::None)
    run_control.completeCommand(false, "motor_free_transport_bench");
#if MOTORFREE_MEASURE
  if (replayState == ReplayState::Recording) {
    bool done = false;
    if (!advanceMeasuredReplay(logger, measurementElapsedMs, done)) replayState = ReplayState::Failed;
    else if (done) replayState = ReplayState::Finished;
    vTaskDelay(1);
  }
#endif
  const auto frame = feet.snapshot().latest;
  const auto now = esp_timer_get_time();
  periodicFeet.observe(frame, now);
  periodicFeet.service(now);
  return false;
}
static void stoppedState(void*, RunControlSnapshot& s) {
#if MOTORFREE_MEASURE
  const bool warm = pressureReady.load() && millis() - cameraStarted >= 4000;
  s.running = replayState == ReplayState::Recording;
  s.downloadable = fixtureReady && replayState == ReplayState::Finished;
  s.ready = fixtureReady && warm && replayState == ReplayState::Ready;
  s.state_id = s.running ? 3 : s.downloadable ? 4 : s.ready ? 2 : replayState == ReplayState::Failed ? 5 : 1;
  const char* names[] = {"", "WARMUP", "READY_TO_MEASURE", "RUNNING_BATCH_SWEEP", "FINISHED", "ESTOP"};
  snprintf(s.state_name, sizeof(s.state_name), "%s", names[s.state_id]);
  s.run_id = measurementId;
  s.sample_count = logger.sampleCount();
  s.measure_elapsed_ms = measurementElapsedMs;
  s.heartbeat_us = micros();
#else
  const bool finished = millis() - cameraStarted >= 4000;
  s.state_id = finished ? 4 : 1;
  snprintf(s.state_name, sizeof(s.state_name), "%s", finished ? "FINISHED" : "WARMUP");
  s.run_id = runId;
  s.sample_count = logger.sampleCount();
  s.heartbeat_us = micros();
  s.downloadable = finished && fixtureReady;
#if MOTORFREE_ARCHIVE
  s.downloadable = s.downloadable && pressureReady.load();
#endif
#endif
}
void setup() {
  vTaskPrioritySet(nullptr, 2);
  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(1);
  Serial.begin(Config::SERIAL_BAUD);
  RuntimeDiag::begin();
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;
  cfg.internal_imu = false;
  cfg.external_imu = false;
  M5.begin(cfg);
  RuntimeDiag::boot(RuntimeDiag::Stage::Logger);
  const bool logOk = logger.begin() && sealed.reserveMetadata();
  RuntimeDiag::result(logOk);
#if MOTORFREE_MEASURE
  fixtureReady = logOk;
#else
  fixtureReady = logOk && seedFixture();
#endif
  RuntimeDiag::result(fixtureReady);
  RuntimeDiag::boot(RuntimeDiag::Stage::Camera);
  RuntimeDiag::result(camera.begin());
  periodicFeet.begin();
  cameraStarted = millis();
  RuntimeDiag::boot(RuntimeDiag::Stage::Control);
  RuntimeDiag::result(run_control.begin(idleStep, stoppedState, nullptr));
  RuntimeDiag::boot(RuntimeDiag::Stage::Feet);
  RuntimeDiag::result(feet.begin(camera, run_control));
  RuntimeDiag::boot(RuntimeDiag::Stage::Export);
  RuntimeDiag::result(sealed.begin(logger));
  RuntimeDiag::boot(RuntimeDiag::Stage::Web);
#if MOTORFREE_MEASURE
  server.on("/bench/replay", HTTP_POST, []() {
    const bool ready = run_control.snapshot().ready && sealed.status().phase == ImmutableExport::Phase::Empty;
    const bool ok = ready && run_control.request(RunControlWorker::Command::Start, 8.f, 0.f, 300);
    server.send(ok ? 202 : 409, "text/plain", ok ? "simulated_recording_queued" : "simulator_not_ready");
  });
#endif
  web.begin(server, run_control, feet, sealed);
  RuntimeDiag::boot(RuntimeDiag::Stage::Ready);
}
void loop() {
#if MOTORFREE_ARCHIVE
  static uint32_t stationSince = 0, lastCheck = 0;
  if (!pressureReady.load() && millis() - lastCheck >= 250) {
    lastCheck = millis();
    if (!WiFi.softAPgetStationNum()) stationSince = 0;
    else if (!stationSince) stationSince = lastCheck;
    else if (lastCheck - stationSince >= 2000) {
      replayMatchInternalHeap();
      pressureReady.store(true);
    }
  }
#endif
  RuntimeDiag::pollFallback();
  if (RuntimeDiag::takeWifiRecoveryRequest()) web.requestWifiRecovery();
  web.update();
  delay(1);
}
