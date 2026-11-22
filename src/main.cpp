#include <Arduino.h>
#include <M5Unified.h>
#include "esp_timer.h"
#include "config.h"
#include "camera_coexistence.h"
#include "bounded_web_server.h"
#include "experiment_runner.h"
#include "imu_manager.h"
#include "psram_logger.h"
#include "roller485_manager.h"
#include "upright_pose_guide.h"
#include "control_latency.h"
#include "web_ui.h"
#include "run_control_worker.h"
#include "foot_observer.h"
#include "periodic_foot_stream.h"
#include "immutable_export.h"
#include "runtime_diagnostics.h"
#include "foot_support_start.h"

BoundedWriteWebServer server(Config::HTTP_PORT);
OneShotCamera camera_probe;
PsramLogger logger;
ImuManager imu;
Roller485Manager roller;
ExperimentRunner runner;
RunControlWorker run_control;
FootObserver feet;
ImmutableExport log_export;
WebUi web;

static uint32_t boot_ms = 0, upright_epoch = 0;
static uint32_t foot_observation_poll_ms = 0, foot_observation_sequence = 0;
static uint64_t upright_since_us = 0, log_epoch_us = 0, measurement_epoch_us = 0;
static bool upright_stable = false;
static UprightPoseGuide::CachedMetrics pose_metrics;
static bool reference_command_active = false;
static uint32_t reference_pose_accel_sequence = 0, reference_pose_revision = UINT32_MAX;
static float reference_pose_pitch_error_deg = 180;
static void updateReferencePoseMetrics(const ImuReading& r) {
  const uint32_t revision = runner.uprightReference().revision;
  if (r.accel_sequence != reference_pose_accel_sequence || revision != reference_pose_revision) {
    reference_pose_pitch_error_deg = runner.uprightPitchErrorDeg(r);
    reference_pose_accel_sequence = r.accel_sequence;
    reference_pose_revision = revision;
  }
}
static void updateAcquisitionContext() {
  imu.setAcquisitionContext(runner.running(),
      runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP,
      static_cast<uint8_t>(runner.status().state));
}
static void captureRunState(void*, RunControlSnapshot& out) {
  const auto& st = runner.status();
  const auto& r = imu.reading();
  out.running = runner.running(); out.state_id = static_cast<uint8_t>(st.state);
  out.run_id = st.run_id; out.energy_control_autonomous = runner.energyControlAutonomousMode();
  out.ready = st.state == ExperimentState::READY_TO_MEASURE;
  out.imu_ok = imu.acquisitionHealthy() && !imu.stale(millis()); out.roller_ok = roller.ok();
  out.downloadable = logger.rwlogDownloadable(); out.sample_count = logger.sampleCount();
  out.heartbeat_us = micros(); out.imu_sample_us = r.last_gyro_update_us;
  out.pulse_active = st.pulse_active; out.measure_elapsed_ms = st.measure_elapsed_ms;
  out.motor_cmd_mA = st.motor_cmd_mA; out.actual_current_mA = st.roller_actual_current_mA;
  out.remaining_ms = st.remaining_ms; out.battery_mV = st.roller_battery_mV;
  out.pitch_deg = st.pitch_mekf_deg; out.rate_dps = st.physical_roll_rate_dps;
  out.mekf_attitude = st.mekf_attitude;
  out.steering = st.steering;
#if WALKING_TURN_OBSERVER
  out.gait_turn = runner.gaitTurnStatus();
#endif
  out.target_deg = runner.energyControlAutonomousTargetPeakDeg();
  out.input_peak_percent = runner.energyControlAutonomousInputPeakPercent();
  out.input_current_mA = runner.energyControlAutonomousInputCurrentMa();
  out.brake_max_ms = runner.energyControlAutonomousBrakeMaxMs();
  out.negative_probe_mode = runner.negativeProbeMode();
  out.walking_profile = logger.walkingTrialProfile();
  out.peak_adaptation_enabled = runner.peakAdaptationEnabled();
  out.energy_bridge_enabled = runner.energyBridgeEnabled();
  out.upright_reference = runner.uprightReference();
  out.lateral_cg = runner.lateralCg();
  out.adaptive_cg = runner.adaptiveCg();
  out.equilibrium_capturing = runner.equilibriumCapturing();
  out.upright_reference_capturing = runner.uprightReferenceCapturing();
  out.upright_reference_capture_samples = runner.uprightReferenceCaptureSamples();
  if (upright_reference::valid(out.upright_reference))
    out.pitch_deg = upright_reference::relative(st.pitch_mekf_abs_deg, out.upright_reference);
  out.led_state = st.led_state; out.sync_event_id = st.sync_event_id;
  out.upright_stable = upright_stable; out.upright_epoch = upright_epoch;
  out.upright_since_us = upright_since_us;
  out.log_epoch_us = log_epoch_us; out.measurement_epoch_us = measurement_epoch_us;
  out.upright_error_deg = pose_metrics.direction_error_deg;
  updateReferencePoseMetrics(r);
  out.upright_pitch_error_deg = reference_pose_pitch_error_deg;
  out.accel_norm_g = pose_metrics.accel_norm_g; out.gyro_norm_dps = pose_metrics.gyro_norm_dps;
  strlcpy(out.state_name, runner.stateName(), sizeof(out.state_name));
  strlcpy(out.last_error, st.last_error ? st.last_error : "", sizeof(out.last_error));
}
static bool controlStep(void*) {
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlService);
  const uint32_t start = micros();
  const bool was_running = runner.running();
  bool run_started = false;
  const bool profile_measurement = runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP;
  const bool profile_pulse = runner.status().pulse_active;
  {
    control_work::Scope work(control_work::Stage::Service, profile_measurement, profile_pulse);
    if (run_control.takeStopRequest()) {
      runner.requestEmergencyStop("web_estop");
      reference_command_active = false;
    }
    updateAcquisitionContext();
    runner.serviceFast(); runner.updateImuDynamicBetaContext();
  }
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlImu);
  const uint32_t imu_start = micros();
  {
    control_work::Scope work(control_work::Stage::ImuDelivery, profile_measurement, profile_pulse);
    imu.update();
  }
  const uint32_t imu_us = micros() - imu_start;
  if (runner.running() && (!imu.acquisitionHealthy() || imu.stale(millis())))
    runner.requestEmergencyStop("imu_acquisition_overflow_backlog_or_stale");
  const auto& r = imu.reading();
  bool stable;
  {
    control_work::Scope work(control_work::Stage::PoseGuide, profile_measurement, profile_pulse);
    pose_metrics.update(r);
    updateReferencePoseMetrics(r);
    const float norm = pose_metrics.accel_norm_g;
    stable = millis() - boot_ms >= 10000 && imu.acquisitionHealthy() && r.last_gyro_update_us &&
        static_cast<uint32_t>(micros() - r.last_gyro_update_us) <= 10000 &&
        isfinite(norm) && fabsf(norm - 1.0f) <= appcfg::kAutoZeroAccelNormToleranceG &&
        r.az_g < 0.0f &&
        reference_pose_pitch_error_deg <= appcfg::kAutoZeroMaxPitchErrorDeg &&
        pose_metrics.gyro_norm_dps <= appcfg::kAutoZeroMaxGyroDps;
    if (stable != upright_stable) { ++upright_epoch; upright_since_us = stable ? esp_timer_get_time() : 0; }
    upright_stable = stable;
  }
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlCommand);
  float start_target_deg = 0.0f;
  float start_input_peak_percent = autonomous_input_percent::kDefaultPercent;
  int16_t start_input_current_mA = autonomous_input_current::kDefaultMa;
  uint16_t start_brake_max_ms = autonomous_brake_limit::kDefaultMs;
  bool start_peak_adaptation_enabled = true;
  bool start_energy_bridge_enabled = true;
  bool start_negative_probe_mode = false;
  const auto command = run_control.takeCommand(&start_target_deg, &start_input_peak_percent,
                                                &start_input_current_mA, &start_brake_max_ms,
                                                &start_peak_adaptation_enabled,
                                                &start_energy_bridge_enabled,
                                                &start_negative_probe_mode);
  if (command == RunControlWorker::Command::Start) {
    bool ok = false;
    const char* error = "clear_previous_run_first";
    if (runner.status().state == ExperimentState::READY_TO_MEASURE) {
      if (!log_export.ready()) error = "export_worker_unavailable";
      else if (!imu.acquisitionHealthy() || imu.stale(millis())) error = "imu_not_healthy";
#if defined(FOOT_SUPPORT_TRANSFER) && FOOT_SUPPORT_TRANSFER
      else if (start_target_deg==8.f && !foot_support_start::ready(feet.snapshot(),esp_timer_get_time()))
        error = "foot_zero_not_ready_or_observation_invalid";
#endif
      else {
        ok = runner.setEnergyControlAutonomousInputPeakPercent(start_input_peak_percent) &&
            runner.setEnergyControlAutonomousInputCurrentMa(start_input_current_mA) &&
            runner.setEnergyControlAutonomousBrakeMaxMs(start_brake_max_ms) &&
            runner.setPeakAdaptationEnabled(start_peak_adaptation_enabled) &&
            runner.setEnergyBridgeEnabled(start_energy_bridge_enabled) &&
            runner.setNegativeProbeMode(start_negative_probe_mode) &&
            runner.setEnergyControlAutonomousTarget(start_target_deg) &&
            runner.startEnergyControlAutonomousCapture();
        error = ok ? "started" : runner.status().last_error;
        if (ok) {
          run_started = true;
          log_epoch_us = esp_timer_get_time() - static_cast<uint32_t>(micros() - logger.runStartUs());
          measurement_epoch_us = 0;
          feet.beginRun(runner.status().run_id, log_epoch_us);
#if FOOT_PERIODIC_V38
          periodicFeet.startRun(runner.status().run_id);
          runner.observePeriodicFoot(periodic_foot::Packet{});
#if SUPPORT_TRANSFER_V39
          runner.startSupportTransferRun();
#endif
#endif
          run_control.beginRunAudit(start);
        }
      }
    }
    run_control.completeCommand(ok, error);
  } else if (command == RunControlWorker::Command::Clear) {
    runner.clearFinishedOrEstop(); feet.clearRun(); log_epoch_us = measurement_epoch_us = 0;
    run_control.completeCommand(true, "cleared");
  } else if (command == RunControlWorker::Command::CaptureUprightReference ||
             command == RunControlWorker::Command::CaptureEquilibrium) {
    reference_command_active = runner.beginUprightReferenceCapture(command == RunControlWorker::Command::CaptureEquilibrium);
    if (!reference_command_active) run_control.completeCommand(false, "upright_reference_requires_stopped");
  }
  if (command == RunControlWorker::Command::SetLateralCg) {
    const bool ok=runner.setLateralCgMm(start_target_deg);
    run_control.completeCommand(ok,ok ? "lateral_cg_saved" : "lateral_cg_save_failed");
  }
  const bool measurement = runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP;
#if FOOT_PERIODIC_V38
  periodicFeet.enter(measurement && measurement_epoch_us,measurement_epoch_us,esp_timer_get_time());
#endif
  control_latency::setActive(measurement);
  const bool fresh = r.gyro_fresh;
  const bool pulse_at_runner_entry = runner.status().pulse_active;
  const bool accel_fresh = r.accel_fresh;
  control_latency::profile.current.pulse_active_at_entry = pulse_at_runner_entry;
  control_latency::profile.current.accel_fresh_at_entry = accel_fresh;
  const uint32_t sample_us = r.last_gyro_update_us;
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlRunner);
  const uint32_t runner_start = micros();
  // v12's local evidence can be short-lived. Poll its already-computed
  // snapshot more promptly; image processing stays on Core0.
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  constexpr uint32_t foot_poll_interval_ms=foot_fast_policy::kControlPollMs;
#elif defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  constexpr uint32_t foot_poll_interval_ms=10;
#else
  constexpr uint32_t foot_poll_interval_ms=25;
#endif
#if (defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23) || (defined(FOOT_RECEIPT_AUDIT) && FOOT_RECEIPT_AUDIT)
  // Receipt audit only: do not feed the walking controller or start a run.
  if(!measurement && millis()-foot_observation_poll_ms>=foot_poll_interval_ms) {
    foot_observation_poll_ms=millis();
    const auto idle_foot=feet.snapshot();
    const uint64_t observed_us=esp_timer_get_time();
#if FOOT_PERIODIC_V38
    periodicFeet.observe(idle_foot.latest,observed_us);
#endif
    if(idle_foot.latest.timestamp_valid && !idle_foot.latest.control_observed_us)
      feet.recordControlObservation(idle_foot.latest.run_id,idle_foot.latest.sequence,observed_us);
  }
#endif
  if(measurement && measurement_epoch_us && millis()-foot_observation_poll_ms>=foot_poll_interval_ms) {
    foot_observation_poll_ms=millis();
    const auto fs=feet.snapshot();const uint64_t foot_observed_us=esp_timer_get_time();const auto& f=fs.latest;
#if FOOT_PERIODIC_V38
    periodicFeet.observe(f,foot_observed_us);
#endif
#if defined(WALKING_TRANSFER_V18) && WALKING_TRANSFER_V18
    // Raw identity/receipt survive an invalid local projection. A repeated
    // snapshot retains its first receipt; invalid timestamps cancel evidence.
    static uint32_t return_sequence=0;
    static uint16_t return_run=0;
    if(f.sequence!=return_sequence || f.run_id!=return_run || !f.timestamp_valid){
      return_sequence=f.sequence;return_run=f.run_id;
      return_observation_policy::FootFrame raw;
#if SUPPORT_TRANSFER_CARRY2
      raw.sequence=f.sequence;raw.image_us=f.frame_us;raw.received_us=foot_observed_us;
#endif
      raw.run_id=f.run_id;raw.timestamp_valid=f.timestamp_valid && f.frame_us>=measurement_epoch_us;
      raw.frame_ms=raw.timestamp_valid?static_cast<uint32_t>((f.frame_us-measurement_epoch_us)/1000):0;
      raw.ready_ms=static_cast<uint32_t>((foot_observed_us-measurement_epoch_us)/1000);
      raw.frame_valid=f.frame_valid;raw.zero_ready=f.zero_ready;
      raw.left_detected=f.left_valid;raw.right_detected=f.right_valid;
      raw.left_range_ok=foot_control_range::allows(f.left_in_range,f.left_x,false);
      raw.right_range_ok=foot_control_range::allows(f.right_in_range,f.right_x,true);
      raw.left_deg=f.left_deg;raw.right_deg=f.right_deg;
      runner.observeReturnFootFrame(raw);
    }
#endif
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
    // Even a timestamp-invalid new frame must cancel the previous forecast.
    runner.observeFootProjection(f.sequence,f.pose_projection);
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
    runner.observeLocalFootProjection(f.local_projection);
#endif
#endif
    if(f.sequence!=foot_observation_sequence && f.frame_us>=measurement_epoch_us && f.timestamp_valid) {
      feet.recordControlObservation(f.run_id,f.sequence,foot_observed_us);
      foot_observation_sequence=f.sequence;
      runner.observeFootFrame(f.run_id,static_cast<uint32_t>((f.frame_us-measurement_epoch_us)/1000),
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
          f.left_valid && foot_control_range::allows(f.left_in_range,f.left_x,false) && f.zero_ready && f.frame_valid,
          f.right_valid && foot_control_range::allows(f.right_in_range,f.right_x,true) && f.zero_ready && f.frame_valid,f.left_deg,f.right_deg,
#else
          f.left_valid,f.right_valid,f.left_deg,f.right_deg,
#endif
          (f.zero_ready && f.frame_valid && f.left_valid) ? fs.left_zero-f.left_x : NAN,
          (f.zero_ready && f.frame_valid && f.right_valid) ? fs.right_zero-f.right_x : NAN,
          f.frame_valid && f.state_id==3 && f.left_reason==MarkerDetectionReason::Detected,
          f.frame_valid && f.state_id==3 && f.right_reason==MarkerDetectionReason::Detected,f.left_x,f.right_x);
    }
  }
#if FOOT_PERIODIC_V38
  if (periodicFeet.service(esp_timer_get_time())) runner.observePeriodicFoot(periodicFeet.current());
#endif
  runner.update();
  uint32_t done;
  const auto done_activity = control_latency::activity(&done);
  const uint32_t runner_us = done - runner_start;
  control_latency::profile.finish(done, done_activity);
  control_latency::setActive(runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP);
  if (!measurement_epoch_us && runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP) {
    measurement_epoch_us = esp_timer_get_time() - static_cast<uint64_t>(runner.status().measure_elapsed_ms) * 1000;
  }
  if (was_running || runner.running()) {
    run_control.recordSampleCompletion(measurement, fresh, sample_us, done, runner_us,
        pulse_at_runner_entry, accel_fresh);
    runner.recordTimingProbeLoop(imu_us, runner_us, done - start);
    run_control.recordStep(start, imu_us, runner_us, done - start);
  }
  runner.setLoopDt(done - start);
  if (reference_command_active) {
    bool ok = false;
    const char* result = "";
    if (runner.pollUprightReferenceCapture(ok, result)) {
      reference_command_active = false;
      run_control.completeCommand(ok, result);
    }
  }
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlFinish);
  if ((was_running || run_started) && !runner.running()) { runner.sealCompletedLog(); feet.finishRun(); }
  // The existing LED sync pattern has exclusive authority during every run.
  if (!runner.running()) {
    RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlIdle);
    const uint32_t hold_ms = stable ? (esp_timer_get_time() - upright_since_us) / 1000 : 0;
    const bool upright_confirmed = hold_ms >= appcfg::kAutoZeroStableMs;
    const bool prompt = millis() - boot_ms >= 10000 && !upright_confirmed;
    digitalWrite(Config::SYNC_LED_PIN, prompt ? HIGH : LOW);
    imu.setStartupGuideState(upright_confirmed ? "upright_ready_camera_optional" :
        (stable ? "hold_upright" : "stand_upright"), upright_confirmed, hold_ms);
  }
  updateAcquisitionContext();
  return runner.running();
}
void setup() {
  vTaskPrioritySet(nullptr, 2);
  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(1);
  Serial.begin(Config::SERIAL_BAUD);
  RuntimeDiag::begin();
  const uint32_t usb_window_start = millis();
  while (millis() - usb_window_start < 8000) { RuntimeDiag::pollFallback(); delay(20); }
  boot_ms = millis(); // Keep the existing guide's ten seconds after this window.
  RuntimeDiag::boot(RuntimeDiag::Stage::M5);
  auto cfg = M5.config(); cfg.serial_baudrate = 0; cfg.internal_imu = false; M5.begin(cfg);
  RuntimeDiag::boot(RuntimeDiag::Stage::Logger); RuntimeDiag::result(logger.begin());
  // The export metadata buffer must have a contiguous block before camera
  // frame buffers and network allocations fragment the remaining PSRAM.
  const bool export_memory_ready = log_export.reserveMetadata();
  RuntimeDiag::boot(RuntimeDiag::Stage::Imu); RuntimeDiag::result(imu.begin());
  // Camera SCCB borrows I2C0 only during boot, before Roller485 owns this port.
  RuntimeDiag::boot(RuntimeDiag::Stage::Camera); RuntimeDiag::result(camera_probe.begin());
  RuntimeDiag::boot(RuntimeDiag::Stage::Roller);
  const bool roller_ok = roller.begin(); RuntimeDiag::result(roller_ok);
  if (roller_ok) RuntimeDiag::result(roller.startIoTask(Config::ROLLER_IO_TASK_CORE,
      Config::ROLLER_IO_TASK_PRIORITY, Config::ROLLER_IO_TASK_STACK_BYTES));
  RuntimeDiag::boot(RuntimeDiag::Stage::Runner);
  runner.begin(logger, imu, roller);
#if FOOT_PERIODIC_V38
  periodicFeet.begin();
#endif
  RuntimeDiag::boot(RuntimeDiag::Stage::Control);
  RuntimeDiag::result(run_control.begin(controlStep, captureRunState, nullptr));
  RuntimeDiag::boot(RuntimeDiag::Stage::Feet); RuntimeDiag::result(feet.begin(camera_probe, run_control));
  RuntimeDiag::boot(RuntimeDiag::Stage::Export);
  RuntimeDiag::result(export_memory_ready && log_export.begin(logger));
  RuntimeDiag::boot(RuntimeDiag::Stage::Web);
  web.begin(server, run_control, feet, log_export);
  RuntimeDiag::boot(RuntimeDiag::Stage::Ready);
}
void loop() {
  RuntimeDiag::pollFallback();
  if (RuntimeDiag::takeWifiRecoveryRequest()) web.requestWifiRecovery();
  web.update();
  delay(1);
}
