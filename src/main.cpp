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
#include "immutable_export.h"
#include "runtime_diagnostics.h"
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
#include "foot_geometry_async.h"
#include "early_geometry_diagnostics.h"
#include "early_clock.h"
#include "foot_calibration_table.h"
#endif

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

#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
// Provisional test only: frame driver timestamp +/-20 ms is an assumed
// exposure window; no optical timing validation is implied.
early_diag::Counters g_early_diag;
static foot_geometry_worker::Async early_worker;
static bool early_worker_ready=false;
static uint32_t early_generation=0,early_run=0,early_peak=0,early_sequence=0;
static uint32_t early_last_pose_us=0;
static float early_pitch_origin=NAN;
static void earlySyncIdentity() {
  const uint32_t run=runner.status().run_id,peak=runner.footEarlyPeakIdentity();
  if(run!=early_run || peak!=early_peak) {
    runner.invalidateFootEarlyCandidate();
    if(run!=early_run) {early_pitch_origin=NAN;early_last_pose_us=0;early_sequence=0;g_early_diag.reset(run);}
    early_run=run;early_peak=peak;
    if(++early_generation==0)++early_generation;
    early_worker.generation(run,peak,early_generation);
  }
}
static void __attribute__((noinline)) earlyRecordPose(uint64_t epoch) {
  if(!early_worker_ready || !epoch || !runner.status().mekf_attitude.valid)return;
  const auto& att=runner.status().mekf_attitude;
  const uint32_t age=static_cast<uint32_t>(micros()-att.sample_us);
  if(!att.sample_us || age>10000 || (early_last_pose_us &&
      static_cast<uint32_t>(att.sample_us-early_last_pose_us)<10000))return;
  const uint64_t now_us=esp_timer_get_time();
  if(now_us<age)return;
  const uint64_t sample_time=now_us-age;
  uint32_t sample_ms;
  if(!early_clock::relativeMs(sample_time,epoch,sample_ms))return;
  const auto& q=att.quaternion;
  const float pitch=atan2f(2.f*(q.w*q.x+q.y*q.z),1.f-2.f*(q.x*q.x+q.y*q.y))*57.2957795f;
  if(!isfinite(pitch))return;
  if(!isfinite(early_pitch_origin))early_pitch_origin=pitch;
  foot_geometry_worker::PoseMessage p;p.run_id=early_run;
  p.pose.ms=sample_ms;
  p.pose.fore_aft_deg=pitch-early_pitch_origin;
  if(early_worker.pose(p))early_last_pose_us=att.sample_us;
  else ++g_early_diag.pose_dropped;
}
static void __attribute__((noinline)) earlySubmitFrame(
    const FootSnapshot& fs,uint64_t observed_us,uint64_t epoch) {
  const FootFrame& f=fs.latest;
  if(!early_worker_ready || !epoch || !f.sequence || f.sequence==early_sequence)return;
  // Every new frame supersedes an armed result, including invalid frames.
  runner.invalidateFootEarlyCandidate();
  early_sequence=f.sequence;++g_early_diag.frames;if(++early_generation==0)++early_generation;
  early_worker.generation(early_run,early_peak,early_generation);
  if(!early_peak || f.run_id!=early_run || !f.timestamp_valid || !f.frame_valid ||
     !f.zero_ready || !f.left_valid || !f.right_valid || !f.left_in_range || !f.right_in_range ||
     f.frame_us<epoch || observed_us<f.frame_us || !isfinite(early_pitch_origin)) {++g_early_diag.invalid_frames;return;}
  uint32_t frame,observed;
  if(!early_clock::epochSupported(epoch) || !early_clock::relativeMs(f.frame_us,epoch,frame) ||
     !early_clock::relativeMs(observed_us,epoch,observed)) {++g_early_diag.invalid_frames;return;}
  // The earliest provisional exposure must be strictly after the current
  // confirmed peak, with 5 ms extra allowance for run-clock disagreement.
  // Subtractions are ordered to avoid unsigned under/overflow.
  if(frame<20 || frame-20<=early_peak || frame-20-early_peak<=5 ||
     observed<frame+20 || observed-frame>=120) {++g_early_diag.invalid_frames;return;}
  if(fs.left_zero<appcfg::kFootAngleBMinCalXPx || fs.left_zero>appcfg::kFootAngleBMaxCalXPx ||
     fs.right_zero<appcfg::kFootAngleAMinCalXPx || fs.right_zero>appcfg::kFootAngleAMaxCalXPx) {++g_early_diag.invalid_frames;return;}
  foot_geometry_worker::Request q;q.run_epoch_ms=static_cast<uint32_t>(epoch/1000);
  q.max_pose_gap_ms=25;auto& in=q.input;
  in.run_id=early_run;in.peak_id=early_peak;in.sequence=f.sequence;in.generation=early_generation;
  in.frame_ms=frame;in.observed_ms=observed;in.expiry_ms=frame+120;
  in.exposure=foot_geometry_worker::Input::Exposure::Provisional;
  in.exposure_earliest_ms=frame-20;in.exposure_latest_ms=frame+20;
  in.side=runner.footEarlyPeakSide();
  in.left_deg=foot_calibration::angleAt(foot_calibration::left,f.left_x);
  in.right_deg=foot_calibration::angleAt(foot_calibration::right,f.right_x);
  if(early_worker.submit(q))++g_early_diag.submitted;else ++g_early_diag.submit_failed;
}
static void __attribute__((noinline)) earlyReceive() {
  if(!early_worker_ready)return;
  foot_geometry_worker::Reply q;
  if(!early_worker.receive(q))return;
  // A replaced job can complete after a new run/peak/frame is current.
  // Never mix its reason or timings into the current run's diagnostics.
  if(q.diagnostic.run_id!=early_run || q.diagnostic.peak_id!=early_peak ||
     q.diagnostic.sequence!=early_sequence || q.diagnostic.generation!=early_generation) {
    ++g_early_diag.stale_replies;return;
  }
  ++g_early_diag.replies;
  if(q.compute_us>g_early_diag.max_compute_us.load())g_early_diag.max_compute_us.store(q.compute_us);
  if(q.max_step_us>g_early_diag.max_step_us.load())g_early_diag.max_step_us.store(q.max_step_us);
  if(q.stack_high_water_raw && (!g_early_diag.min_stack_raw.load() ||
      q.stack_high_water_raw<g_early_diag.min_stack_raw.load()))g_early_diag.min_stack_raw.store(q.stack_high_water_raw);
  const unsigned reason=static_cast<unsigned>(q.diagnostic.reason);
  const unsigned async_reason=static_cast<unsigned>(q.async_reason);
  if(reason<early_diag::kReasonCount)++g_early_diag.reason[reason];
  if(async_reason<early_diag::kAsyncReasonCount)++g_early_diag.async_reason[async_reason];
  if(!q.accepted)return;
  ++g_early_diag.valid;
  const auto& r=q.result;
  if(r.run_id!=early_run || r.peak_id!=early_peak || r.sequence!=early_sequence ||
     r.generation!=early_generation || r.exposure!=foot_geometry_worker::Input::Exposure::Provisional) {
    ++g_early_diag.stale_replies;return;
  }
  if(runner.offerFootEarlyCandidate(static_cast<uint16_t>(r.run_id),r.peak_id,r.frame_ms,r.ready_ms,r.threshold_deg))
    ++g_early_diag.armed;
}
#endif
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
  out.target_deg = runner.energyControlAutonomousTargetPeakDeg();
  out.input_peak_percent = runner.energyControlAutonomousInputPeakPercent();
  out.input_current_mA = runner.energyControlAutonomousInputCurrentMa();
  out.brake_max_ms = runner.energyControlAutonomousBrakeMaxMs();
  out.negative_probe_mode = runner.negativeProbeMode();
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
  control_latency::setActive(measurement);
  const bool fresh = r.gyro_fresh;
  const bool pulse_at_runner_entry = runner.status().pulse_active;
  const bool accel_fresh = r.accel_fresh;
  control_latency::profile.current.pulse_active_at_entry = pulse_at_runner_entry;
  control_latency::profile.current.accel_fresh_at_entry = accel_fresh;
  const uint32_t sample_us = r.last_gyro_update_us;
  RuntimeDiag::phase(RuntimeDiag::Lane::Control, RuntimeDiag::Phase::ControlRunner);
  const uint32_t runner_start = micros();
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
  if(measurement && measurement_epoch_us) {earlySyncIdentity();earlyReceive();}
#endif
  if(measurement && measurement_epoch_us && millis()-foot_observation_poll_ms>=25) {
    foot_observation_poll_ms=millis();
    const auto fs=feet.snapshot();const uint64_t foot_observed_us=esp_timer_get_time();const auto& f=fs.latest;
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
    earlySubmitFrame(fs,foot_observed_us,measurement_epoch_us);
#endif
    if(f.sequence!=foot_observation_sequence && f.frame_us>=measurement_epoch_us && f.timestamp_valid) {
      feet.recordControlObservation(f.run_id,f.sequence,foot_observed_us);
      foot_observation_sequence=f.sequence;
      runner.observeFootFrame(f.run_id,static_cast<uint32_t>((f.frame_us-measurement_epoch_us)/1000),
          f.left_valid,f.right_valid,f.left_deg,f.right_deg,
          (f.zero_ready && f.frame_valid && f.left_valid) ? fs.left_zero-f.left_x : NAN,
          (f.zero_ready && f.frame_valid && f.right_valid) ? fs.right_zero-f.right_x : NAN,
          f.frame_valid && f.state_id==3 && f.left_reason==MarkerDetectionReason::Detected,
          f.frame_valid && f.state_id==3 && f.right_reason==MarkerDetectionReason::Detected,f.left_x,f.right_x);
    }
  }
  runner.update();
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
  if(measurement && measurement_epoch_us) {earlySyncIdentity();earlyRecordPose(measurement_epoch_us);}
#endif
  uint32_t done;
  const auto done_activity = control_latency::activity(&done);
  const uint32_t runner_us = done - runner_start;
  control_latency::profile.finish(done, done_activity);
  control_latency::setActive(runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP);
  if (!measurement_epoch_us && runner.status().state == ExperimentState::RUNNING_BATCH_SWEEP) {
    measurement_epoch_us = esp_timer_get_time() - static_cast<uint64_t>(runner.status().measure_elapsed_ms) * 1000;
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
    runner.setFootEarlyEpoch(measurement_epoch_us);
#endif
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
  RuntimeDiag::boot(RuntimeDiag::Stage::Control);
  RuntimeDiag::result(run_control.begin(controlStep, captureRunState, nullptr));
  RuntimeDiag::boot(RuntimeDiag::Stage::Feet); RuntimeDiag::result(feet.begin(camera_probe, run_control));
  RuntimeDiag::boot(RuntimeDiag::Stage::Export);
  RuntimeDiag::result(export_memory_ready && log_export.begin(logger));
  RuntimeDiag::boot(RuntimeDiag::Stage::Web);
  web.begin(server, run_control, feet, log_export);
#if FOOT_EARLY_TRIAL && !ANGLE_EARLY_TRIAL
  early_worker_ready=early_worker.begin();
  g_early_diag.worker_ready.store(early_worker_ready);
#endif
  RuntimeDiag::boot(RuntimeDiag::Stage::Ready);
}
void loop() { RuntimeDiag::pollFallback(); web.update(); delay(1); }
