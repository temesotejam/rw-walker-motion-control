#include "work_target_policy.h"
#include "work_start_policy.h"
#include "web_ui.h"
#include "config.h"
#include "runtime_web.h"
#include "runtime_diagnostics.h"
#include "foot_range_diagnostics.h"
#include "foot_calibration_diagnostics.h"
#include "foot_support_start.h"
#include <WiFi.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <errno.h>

static_assert(ARDUINO_EVENT_RUNNING_CORE == 0, "Wi-Fi events must run outside the control core");

static String num(float x) { return isfinite(x) ? String(x, 4) : String("null"); }
static const char* phase(ImmutableExport::Phase p) {
  switch (p) {
    case ImmutableExport::Phase::Building: return "building";
    case ImmutableExport::Phase::Ready: return "ready";
    case ImmutableExport::Phase::Error: return "error";
    default: return "empty";
  }
}
void WebUi::begin(BoundedWriteWebServer& s, RunControlWorker& control, FootObserver& feet, ImmutableExport& exporter) {
  server_ = &s; control_ = &control; feet_ = &feet; export_ = &exporter;
  preview_buffer_ = static_cast<uint8_t*>(ps_malloc(FootObserver::kPreviewBytes));
  WiFi.onEvent([this](arduino_event_id_t event, arduino_event_info_t) {
    RuntimeDiag::wifiEvent(static_cast<uint32_t>(event),
        event == ARDUINO_EVENT_WIFI_AP_STACONNECTED ? 1 : event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED ? -1 : 0,
        event == ARDUINO_EVENT_WIFI_AP_START ? 1 : event == ARDUINO_EVENT_WIFI_AP_STOP ? 0 : -1);
  });
  RuntimeDiag::result(WiFi.mode(WIFI_AP));
  RuntimeDiag::result(WiFi.softAP(Config::AP_SSID, Config::AP_PASS, Config::AP_CHANNEL));
  RuntimeDiag::result(WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0)));
  boot_id_ = esp_random();
  s.on("/", HTTP_GET, [this]() {
    if (quietResponse()) return;
    RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpRoot);
    server_->sendHeader("Cache-Control", "no-store");
    server_->send_P(200, "text/html; charset=utf-8", RUNTIME_HTML);
  });
  s.on("/status.json", HTTP_GET, [this]() { status(); });
  s.on("/vision/capture", HTTP_POST, [this]() { previewCapture(); });
  s.on("/vision/chunk", HTTP_GET, [this]() { previewChunk(); });
  s.on("/start-energy-control-autonomous", HTTP_POST, [this]() { command(RunControlWorker::Command::Start); });
  s.on("/clear", HTTP_POST, [this]() { command(RunControlWorker::Command::Clear); });
  s.on("/lateral-cg/capture", HTTP_POST, [this]() { command(RunControlWorker::Command::CaptureEquilibrium); });
  s.on("/lateral-cg/set", HTTP_POST, [this]() { command(RunControlWorker::Command::SetLateralCg); });
  s.on("/upright-reference/capture", HTTP_POST, [this]() { command(RunControlWorker::Command::CaptureUprightReference); });
  s.on("/stop", HTTP_POST, [this]() {
    RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpCommand);
    const bool ok = control_->requestStop();
    offline_.cancel(millis());
    server_->send(ok ? 202 : 503, "text/plain", ok ? "stop_queued" : "controller_unavailable");
  });
  s.on("/export/prepare", HTTP_POST, [this]() {
    if (offline_.busy()) { server_->send(409, "text/plain", "run_preparing"); return; }
    RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpCommand);
    const auto st = control_->snapshot();
    const bool ok = !st.running && st.downloadable && !control_->commandState().pending && export_->prepare();
    server_->send(ok ? 202 : 409, "text/plain", ok ? "preparing" : "completed_run_required");
  });
  s.on("/export/manifest", HTTP_GET, [this]() { manifest(); });
  s.on("/export/chunk", HTTP_GET, [this]() { chunk(); });
  s.on("/diagnostics/usb", HTTP_POST, [this]() {
    if (offline_.busy()) { server_->send(409, "text/plain", "run_preparing"); return; }
    const String value = server_->arg("enabled");
    if (value != "1" && value != "0") { server_->send(400, "text/plain", "enabled_0_or_1_required"); return; }
    RuntimeDiag::setEnabled(value == "1");
    server_->send(200, "text/plain", RuntimeDiag::enabled() ? "diagnostics_on" : "diagnostics_off");
  });
  s.on("/download/rwlog", HTTP_GET, [this]() {
    server_->send(410, "text/plain", "Open http://192.168.4.1/ and use resumable RWLOG download.");
  });
  s.onNotFound([this]() { server_->send(404, "text/plain", "not_found"); });
  s.enableDelay(false); s.begin();
}
void WebUi::update() {
  if (offline_.busy()) {
    if (offline_.serving() && server_) server_->handleClient();
    const uint32_t now = millis();
    if (now - offline_poll_ms_ >= 20) { offline_poll_ms_ = now; offline_.update(now, *this); }
    return;
  }
  RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpPoll);
  if (server_) server_->handleClient();
  RuntimeDiag::sampleMemory();
  RuntimeDiag::beat(RuntimeDiag::Lane::Http);
  RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::Wait);
}
bool WebUi::stopServer() {
  server_->client().stop();
  server_->stop();
  return !server_->listening();
}
bool WebUi::queueStart() {
  start_command_id_ = control_->commandState().submitted + 1;
  start_submitted_ = control_->request(RunControlWorker::Command::Start, start_target_deg_,
                                       start_input_peak_percent_, start_input_current_mA_,
                                       start_brake_max_ms_, start_peak_adaptation_enabled_,
                                       start_energy_bridge_enabled_, start_negative_probe_mode_);
  return start_submitted_;
}
OfflineRunSession::StartResult WebUi::startResult() const {
  if (!start_submitted_) return OfflineRunSession::StartResult::Rejected;
  const auto c = control_->commandState();
  if (c.pending || c.completed != start_command_id_ || !control_->commandPublished(start_command_id_))
    return OfflineRunSession::StartResult::Pending;
  return c.ok ? OfflineRunSession::StartResult::Started : OfflineRunSession::StartResult::Rejected;
}
bool WebUi::runActive() const { return control_->active(); }
void WebUi::cancelStart() { if (!cancel_sent_) cancel_sent_ = control_->requestStop(); }
bool WebUi::restoreTransport() {
  // AP/netif/DHCP were never stopped. Rebind only port 80, preserving routes,
  // boot ID, buffers, sealed logs and ImmutableExport token/CRC/metadata.
  // Also close a client left by cancellation during the response-drain phase.
  server_->client().stop();
  server_->begin();
  return server_->listening();
}
void WebUi::transportReady() { RuntimeDiag::setRunActive(false); }
bool WebUi::quietResponse() {
  if (!offline_.busy()) return false;
  char json[160];
  snprintf(json, sizeof(json), "{\"offline_run\":true,\"phase\":\"%s\",\"wait_ms\":%lu}",
      offline_.name(), static_cast<unsigned long>(offline_.displayWaitMs(millis())));
  server_->sendHeader("Cache-Control", "no-store"); server_->send(200, "application/json", json);
  return true;
}
void WebUi::command(RunControlWorker::Command cmd) {
  if (offline_.busy()) { server_->send(409, "text/plain", "run_preparing"); return; }
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpCommand);
  if (server_->hasArg("timing_ms")) {
    server_->send(400, "text/plain", "fixed_3ms_reload_page"); return;
  }
  const auto st = control_->snapshot();
  if (st.running || control_->commandState().pending || export_->status().phase == ImmutableExport::Phase::Building) {
    server_->send(409, "text/plain", "busy"); return;
  }
  if (cmd == RunControlWorker::Command::Start &&
      (export_->status().phase != ImmutableExport::Phase::Empty || !st.ready)) {
    server_->send(409, "text/plain", "clear_previous_run_first"); return;
  }
  if (cmd == RunControlWorker::Command::Start && !work_start_policy::referenceAllowed(
      server_->hasArg("energy_model") && server_->arg("energy_model")=="work_control",
      upright_reference::valid(st.upright_reference))) {
    server_->send(409, "text/plain", "upright_reference_required"); return;
  }
  if ((cmd == RunControlWorker::Command::CaptureUprightReference ||
       cmd == RunControlWorker::Command::CaptureEquilibrium || cmd == RunControlWorker::Command::SetLateralCg) &&
      (!st.imu_ok || !st.heartbeat_us || static_cast<uint32_t>(micros() - st.heartbeat_us) >= 500000 ||
       st.pulse_active || st.motor_cmd_mA != 0 ||
       (String(st.state_name) != "READY_TO_MEASURE" && String(st.state_name) != "FINISHED" && String(st.state_name) != "ESTOP"))) {
    server_->send(409, "text/plain", "upright_reference_requires_stopped"); return;
  }
  if (cmd == RunControlWorker::Command::SetLateralCg) {
    const String text=server_->arg("shift_mm");char* end=nullptr;
    const float mm=strtof(text.c_str(),&end);
    if (text.length()==0 || end==text.c_str() || *end || !std::isfinite(mm) || std::fabs(mm)>lateral_cg::kMaxShiftMm) {
      server_->send(400,"text/plain","lateral_cg_shift_invalid");return;
    }
    const bool ok=control_->request(cmd,mm);
    server_->send(ok ? 202 : 409,"text/plain",ok ? "command_queued" : "command_busy");return;
  }
  if (cmd == RunControlWorker::Command::Clear && !export_->reset()) {
    server_->send(409, "text/plain", "export_preparing"); return;
  }
  if (cmd == RunControlWorker::Command::Start) {
    float target_deg;
    if (!server_->hasArg("target_deg") ||
        !autonomous_target::parse(server_->arg("target_deg").c_str(), target_deg)) {
      server_->send(400, "text/plain", "target_deg_8_10_12_required_reload_page"); return;
    }
    float input_peak_percent;
    if (!server_->hasArg("input_peak_percent") ||
        !autonomous_input_percent::parse(server_->arg("input_peak_percent").c_str(), input_peak_percent)) {
      server_->send(400, "text/plain", "input_peak_percent_must_be_zero_for_energy_control"); return;
    }
    int16_t input_current_mA;
    if (!server_->hasArg("input_current_mA") ||
        !autonomous_input_current::parse(server_->arg("input_current_mA").c_str(), input_current_mA)) {
      server_->send(400, "text/plain", "input_current_mA_100_to_1200_step_10_required_reload_page"); return;
    }
    uint16_t brake_max_ms;
    if (!server_->hasArg("brake_max_ms") ||
        !autonomous_brake_limit::parse(server_->arg("brake_max_ms").c_str(), brake_max_ms)) {
      server_->send(400, "text/plain", "negative_width_invalid_reload_page"); return;
    }
    if (!server_->hasArg("peak_adaptation") ||
        (server_->arg("peak_adaptation") != "0" && server_->arg("peak_adaptation") != "1")) {
      server_->send(400, "text/plain", "peak_adaptation_0_or_1_required_reload_page"); return;
    }
    const bool peak_adaptation_enabled = server_->arg("peak_adaptation") == "1";
    if (!server_->hasArg("energy_model") ||
        (server_->arg("energy_model") != "bridge" && server_->arg("energy_model") != "quadratic" &&
         server_->arg("energy_model") != "bridge_probe" &&
         server_->arg("energy_model") != "bridge_probe_batch" &&
         server_->arg("energy_model") != "bridge_probe_extended" &&
         server_->arg("energy_model") != "bridge_probe_positive_current" &&
         server_->arg("energy_model") != "bridge_inverse" &&
         server_->arg("energy_model") != "work_control")) {
      server_->send(400, "text/plain", "energy_model_invalid_reload_page"); return;
    }
    const bool negative_probe_batch_mode = server_->arg("energy_model") == "bridge_probe_batch";
    const bool negative_probe_extended_mode = server_->arg("energy_model") == "bridge_probe_extended";
    const bool positive_current_batch_mode = server_->arg("energy_model") == "bridge_probe_positive_current";
    const bool negative_probe_mode = server_->arg("energy_model") == "bridge_probe" ||
        negative_probe_batch_mode || negative_probe_extended_mode || positive_current_batch_mode;
    const bool energy_bridge_enabled = server_->arg("energy_model") != "quadratic";
    if ((negative_probe_mode && (!(positive_current_batch_mode ?
           autonomous_brake_limit::positiveCurrentBatch(brake_max_ms) : negative_probe_extended_mode ?
           autonomous_brake_limit::extendedBatch(brake_max_ms) : negative_probe_batch_mode ?
           autonomous_brake_limit::batch(brake_max_ms) : autonomous_brake_limit::probe(brake_max_ms)) ||
         target_deg != 8.0f || input_current_mA != 300 || peak_adaptation_enabled)) ||
        (!negative_probe_mode && energy_bridge_enabled && brake_max_ms != 0 &&
         !((server_->arg("energy_model") == "bridge_inverse" && brake_max_ms == 25 ||
            server_->arg("energy_model") == "work_control" && brake_max_ms == 40) &&
           work_target_policy::packageAllowed(target_deg,brake_max_ms,input_current_mA,peak_adaptation_enabled))) ||
        (!energy_bridge_enabled && !autonomous_brake_limit::quadratic(brake_max_ms))) {
      server_->send(400, "text/plain", "experiment_package_settings_invalid_reload_page"); return;
    }
    const bool ok = offline_.queue(millis());
    if (ok) {
      start_target_deg_ = target_deg;
      start_input_peak_percent_ = input_peak_percent;
      start_input_current_mA_ = input_current_mA;
      start_brake_max_ms_ = brake_max_ms;
      start_peak_adaptation_enabled_ = peak_adaptation_enabled;
      start_energy_bridge_enabled_ = energy_bridge_enabled;
      start_negative_probe_mode_ = negative_probe_mode;
      start_submitted_ = cancel_sent_ = false;
      RuntimeDiag::setRunActive(true);
    }
    server_->send(ok ? 202 : 409, "text/plain", ok ? "offline_run_queued" : "command_busy");
  } else {
    const bool ok = control_->request(cmd);
    server_->send(ok ? 202 : 409, "text/plain", ok ? "command_queued" : "command_busy");
  }
}
void WebUi::status() {
  if (quietResponse()) return;
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpStatus);
  const auto s = control_->snapshot(); const auto f = feet_->snapshot();
  const auto c = control_->commandState(); const auto e = export_->status();
  const auto h = control_->healthSnapshot();
  const auto camera = feet_->cameraSnapshot();
  const bool fresh = s.heartbeat_us && static_cast<uint32_t>(micros() - s.heartbeat_us) < 500000;
  RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpJson);
  String json; json.reserve(6100);
  json = "{\"revision\":\"" RUNTIME_VERSION "\",\"state\":\"" + String(s.state_name) + "\"";
#if defined(FOOT_EARLY_TRIAL) && FOOT_EARLY_TRIAL
  json += ",\"diagnostic_build_id\":\"" RUNTIME_DIAGNOSTIC_BUILD_ID "\"";
#endif
  json += ",\"boot_id\":" + String(boot_id_);
  json += ",\"network\":{\"phase\":\"" + String(offline_.name()) + "\",\"last_error\":\"" + String(offline_.error()) + "\"}";
  json += ",\"usb_diagnostics\":" + String(RuntimeDiag::enabled() ? "true" : "false");
  json += ",\"running\":" + String(s.running ? "true" : "false");
  json += ",\"ready\":" + String(work_start_policy::ready(s.ready,fresh,s.imu_ok,s.roller_ok,export_->ready()) ? "true" : "false");
  json += ",\"controller_fresh\":" + String(fresh ? "true" : "false");
  json += ",\"run_id\":" + String(s.run_id) + ",\"samples\":" + String(s.sample_count);
  json += ",\"remaining_ms\":" + String(s.remaining_ms) + ",\"elapsed_ms\":" + String(s.measure_elapsed_ms);
  json += ",\"pitch_deg\":" + num(s.pitch_deg) + ",\"rate_dps\":" + num(s.rate_dps);
  json += ",\"mekf\":" + mekfAttitudeJson(s.mekf_attitude, micros(), s.imu_ok);
  json += ",\"steering\":{\"profile\":\"disabled\",\"enabled\":false,\"feedback_enabled\":false,\"heading_enabled\":false,\"response_check_enabled\":false,\"delta_deg\":0}";
  json += ",\"target_deg\":" + num(s.target_deg);
  json += ",\"input_peak_percent\":" + num(s.input_peak_percent);
  json += ",\"input_current_mA\":" + String(s.input_current_mA);
  json += ",\"brake_max_ms\":" + String(s.brake_max_ms);
  json += ",\"peak_adaptation_enabled\":" + String(s.peak_adaptation_enabled ? "true" : "false");
  json += ",\"energy_model\":\"" + String(s.energy_bridge_enabled && !s.negative_probe_mode &&
      s.brake_max_ms == 40 ? "work_control" : s.energy_bridge_enabled && !s.negative_probe_mode &&
      s.brake_max_ms == 25 ? "bridge_inverse" : s.negative_probe_mode ?
      (autonomous_brake_limit::positiveCurrentBatch(s.brake_max_ms) ? "bridge_probe_positive_current" :
       autonomous_brake_limit::extendedBatch(s.brake_max_ms) ? "bridge_probe_extended" :
       autonomous_brake_limit::batch(s.brake_max_ms) ? "bridge_probe_batch" : "bridge_probe") :
      (s.energy_bridge_enabled ? "bridge" : "quadratic")) + "\"";
  json += ",\"upright_reference\":{\"valid\":" + String(upright_reference::valid(s.upright_reference) ? "true" : "false");
  json += ",\"abs_pitch_deg\":" + num(upright_reference::valid(s.upright_reference) ? s.upright_reference.pitch_deg : NAN);
  json += ",\"relative_pitch_deg\":" + num(upright_reference::relative(s.mekf_attitude.pitch_deg, s.upright_reference));
  json += ",\"revision\":" + String(s.upright_reference.revision) + ",\"samples\":" + String(s.upright_reference.samples);
  json += ",\"spread_deg\":" + num(s.upright_reference.spread_deg);
  json += ",\"capturing\":" + String(s.upright_reference_capturing ? "true" : "false");
  json += ",\"capture_samples\":" + String(s.upright_reference_capture_samples) + "}";
  const bool cg_valid=lateral_cg::matchesAbsolute(s.lateral_cg);
  const float cg_mm=s.adaptive_cg.enabled ? s.adaptive_cg.shift_mm : cg_valid ? s.lateral_cg.shift_mm : 0.f;
  const lateral_energy::Geometry cg_geometry;
  json += ",\"lateral_cg\":{\"valid\":" + String(cg_valid ? "true" : "false") +
      ",\"automatic\":true,\"pairs\":" + String(s.adaptive_cg.pairs) +
      ",\"updates\":" + String(s.adaptive_cg.updates) +
      ",\"common_bias_mj\":" + num(s.adaptive_cg.enabled ? s.adaptive_cg.common_bias_mj : 0.f) +
      ",\"shift_mm\":" + num(cg_mm) + ",\"source\":" + String(s.lateral_cg.source) +
      ",\"capturing\":" + String(s.equilibrium_capturing ? "true" : "false") +
      ",\"positive_target_mj\":" + num(1000.f*lateral_cg::potential(cg_geometry,s.target_deg,cg_mm)) +
      ",\"negative_target_mj\":" + num(1000.f*lateral_cg::potential(cg_geometry,-s.target_deg,cg_mm)) + "}";
  json += ",\"motor_mA\":" + String(s.motor_cmd_mA) + ",\"actual_mA\":" + String(s.actual_current_mA);
  json += ",\"battery_mV\":" + String(s.battery_mV);
  json += ",\"imu_ok\":" + String(s.imu_ok ? "true" : "false") + ",\"roller_ok\":" + String(s.roller_ok ? "true" : "false");
  json += ",\"led\":" + String(s.led_state) + ",\"sync_event_id\":" + String(s.sync_event_id);
  json += ",\"downloadable\":" + String(s.downloadable ? "true" : "false");
  json += ",\"last_error\":\"" + String(s.last_error) + "\"";
  json += ",\"command\":{\"submitted\":" + String(c.submitted) + ",\"completed\":" + String(c.completed);
  json += ",\"pending\":" + String(c.pending ? "true" : "false") + ",\"ok\":" + String(c.ok ? "true" : "false");
  json += ",\"result\":\"" + String(c.result) + "\"}";
  json += ",\"export_worker_ready\":" + String(export_->ready() ? "true" : "false");
  json += ",\"export_phase\":\"" + String(phase(e.phase)) + "\"";
#if defined(FOOT_SUPPORT_TRANSFER) && FOOT_SUPPORT_TRANSFER
  json += ",\"foot_control_required\":true,\"foot_control_ready\":" + String(foot_support_start::ready(f,esp_timer_get_time())?"true":"false");
#endif
  json += ",\"foot\":{\"available\":" + String(f.available ? "true" : "false");
  json += ",\"detector\":\"" + String(appcfg::kWhiteDetectorRevision) + "\"";
  json += ",\"range\":" + footRangeDiagnosticsJson();
  json += ",\"calibration\":" + footCalibrationDiagnosticsJson();
  json += ",\"zero_ready\":" + String(f.zero_ready ? "true" : "false") + ",\"zero_samples\":" + String(f.zero_samples);
  json += ",\"zero_reason\":\"" + String(footZeroReasonName(f.zero_reason)) + "\"";
  json += ",\"preview_available\":" + String(f.preview_available && preview_buffer_ ? "true" : "false");
  json += ",\"right_zero_x\":" + num(f.right_zero) + ",\"left_zero_x\":" + num(f.left_zero);
  json += ",\"right_x\":" + num(f.latest.right_x) + ",\"left_x\":" + num(f.latest.left_x);
  json += ",\"right_deg\":" + num(f.latest.right_deg) + ",\"left_deg\":" + num(f.latest.left_deg);
  json += ",\"right_scan_y\":" + num(f.latest.right_scan_y) + ",\"left_scan_y\":" + num(f.latest.left_scan_y);
  json += ",\"right_contrast\":" + num(f.latest.right_contrast) + ",\"left_contrast\":" + num(f.latest.left_contrast);
  json += ",\"right_weight\":" + num(f.latest.right_weight) + ",\"left_weight\":" + num(f.latest.left_weight);
  json += ",\"right_reason\":\"" + String(markerDetectionReasonName(f.latest.right_reason)) + "\"";
  json += ",\"left_reason\":\"" + String(markerDetectionReasonName(f.latest.left_reason)) + "\"";
  json += ",\"right_templates\":" + String(f.latest.right_templates) + ",\"left_templates\":" + String(f.latest.left_templates);
  json += ",\"right_candidates\":" + String(f.latest.right_candidates) + ",\"left_candidates\":" + String(f.latest.left_candidates);
  json += ",\"right_ambiguity\":" + num(f.latest.right_ambiguity) + ",\"left_ambiguity\":" + num(f.latest.left_ambiguity);
  json += ",\"right_valid\":" + String(f.latest.right_valid ? "true" : "false");
  json += ",\"left_valid\":" + String(f.latest.left_valid ? "true" : "false");
  json += ",\"right_in_range\":" + String(f.latest.right_in_range ? "true" : "false");
  json += ",\"left_in_range\":" + String(f.latest.left_in_range ? "true" : "false");
  json += ",\"age_ms\":" + String(f.latest.delivered_us ? (esp_timer_get_time() - f.latest.delivered_us) / 1000.0 : -1.0, 1);
  json += ",\"processing_us\":" + String(f.latest.processing_us);
  json += ",\"processing_max_us\":" + String(f.processing_max_us);
  json += ",\"sequence\":" + String(f.latest.sequence);
  json += ",\"frame_valid\":" + String(f.latest.frame_valid ? "true" : "false");
  json += ",\"frame_timestamp_valid\":" + String(f.latest.timestamp_valid ? "true" : "false");
  json += ",\"fps\":" + num(f.fps) + ",\"frames\":" + String(f.count);
  json += ",\"captured_frames\":" + String(f.captured_frames);
  json += ",\"right_marker_failures\":" + String(f.right_marker_failures);
  json += ",\"left_marker_failures\":" + String(f.left_marker_failures);
  json += ",\"failures\":" + String(f.frame_failures) + ",\"overflow\":" + String(f.overflow ? "true" : "false") + "}";
  json += ",\"upright\":{\"stable\":" + String(s.upright_stable ? "true" : "false");
  json += ",\"error_deg\":" + num(s.upright_error_deg) + ",\"accel_g\":" + num(s.accel_norm_g);
  json += ",\"pitch_error_deg\":" + num(s.upright_pitch_error_deg);
  json += ",\"gyro_dps\":" + num(s.gyro_norm_dps) + "}";
  json += ",\"camera\":{\"initialized\":" + String(camera.camera_ok ? "true" : "false");
  json += ",\"receiver_active\":" + String(camera.receiver_active ? "true" : "false");
  json += ",\"xclk_active\":" + String(camera.xclk_active ? "true" : "false");
  json += ",\"xclk_hz\":" + String(camera.xclk_hz);
  json += ",\"capture_us\":" + String(camera.last_capture_us) + ",\"max_capture_us\":" + String(camera.max_capture_us);
  json += ",\"driver_task_core\":" + String(camera.cam_task_core);
  json += ",\"driver_task_priority\":" + String(camera.cam_task_effective_priority) + "}";
  RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpMemory);
  const auto memory = RuntimeDiag::memorySnapshot();
  json += ",\"memory\":{\"sampled_ms\":" + String(memory.at_ms);
  json += ",\"sampled\":" + String(memory.at_ms ? "true" : "false");
  json += ",\"internal_free\":" + String(memory.internal_free);
  json += ",\"internal_min_free\":" + String(memory.internal_min);
  json += ",\"internal_largest\":" + String(memory.largest);
  json += ",\"dma_free\":" + String(memory.dma_free);
  json += ",\"psram_free\":" + String(memory.psram_free) + "}";
  RuntimeDiag::phase(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpJson);
  json += ",\"control\":{\"steps\":" + String(h.steps);
  json += ",\"sample_deadline_over\":" + String(h.sample_deadline_over);
  json += ",\"sample_deadline_max_us\":" + String(h.sample_deadline_max_us);
  json += ",\"runner_deadline_over\":" + String(h.runner_deadline_over) + "}}";
  server_->sendHeader("Cache-Control", "no-store"); server_->send(200, "application/json", json);
}
void WebUi::manifest() {
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpManifest);
  const auto s = export_->status();
  char body[420];
  snprintf(body, sizeof(body),
      "{\"phase\":\"%s\",\"token\":\"%s\",\"bytes\":%lu,\"hashed_bytes\":%lu,\"crc32\":%lu,"
      "\"chunk_bytes\":4096,\"filename\":\"%s\",\"error\":\"%s\"}",
      phase(s.phase), s.token, static_cast<unsigned long>(s.bytes), static_cast<unsigned long>(s.hashed_bytes),
      static_cast<unsigned long>(s.crc), s.filename, s.error);
  server_->sendHeader("Cache-Control", "no-store"); server_->send(200, "application/json", body);
}
static bool unsignedArg(const String& text, uint32_t& value) {
  if (!text.length() || text.length() > 10) return false;
  uint64_t n = 0;
  for (size_t i = 0; i < text.length(); ++i) {
    if (text[i] < '0' || text[i] > '9') return false;
    n = n * 10 + text[i] - '0'; if (n > UINT32_MAX) return false;
  }
  value = static_cast<uint32_t>(n); return true;
}
void WebUi::chunk() {
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpChunk);
  uint32_t offset = 0, length = 0;
  if (!unsignedArg(server_->arg("offset"), offset) || !unsignedArg(server_->arg("length"), length) ||
      !export_->chunk(server_->arg("token").c_str(), offset, length, chunk_buffer_ + 16)) {
    server_->send(409, "text/plain", "export_token_or_range_mismatch"); return;
  }
  const uint32_t header[] = {export_protocol::kChunkMagic, offset, length,
      export_protocol::crc32(0, chunk_buffer_ + 16, length)};
  memcpy(chunk_buffer_, header, 16);
  server_->sendHeader("Cache-Control", "no-store");
  server_->setContentLength(length + 16);
  server_->send(200, "application/octet-stream", "");
  server_->sendContent(reinterpret_cast<const char*>(chunk_buffer_), length + 16);
}

bool WebUi::previewAllowed() const {
  return !offline_.busy() && !control_->snapshot().running && !control_->commandState().pending &&
         export_->status().phase != ImmutableExport::Phase::Building;
}
static String previewMarkerJson(const WhiteMarkerObservation& m) {
  String s;
  s = "{\"valid\":" + String(m.valid ? "true" : "false");
  s += ",\"reason\":\"" + String(markerDetectionReasonName(m.reason)) + "\"";
  s += ",\"x\":" + num(m.center_x_px) + ",\"scan_y\":" + num(m.center_y_px);
  s += ",\"width\":" + String(m.bright_width_px) + ",\"contrast\":" + num(m.peak_contrast);
  s += ",\"weight\":" + num(m.weight_sum) + ",\"candidates\":" + String(m.candidate_count);
  s += ",\"ambiguity_ratio\":" + num(m.ambiguity_ratio);
  s += ",\"alternate_x\":" + num(m.alternate_x_px) + ",\"alternate_y\":" + num(m.alternate_y_px) + "}";
  return s;
}
void WebUi::previewCapture() {
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpManifest);
  if (!previewAllowed()) { server_->send(409, "text/plain", "preview_requires_idle"); return; }
  FootPreviewInfo p;
  if (!preview_buffer_ || !feet_->copyPreview(preview_buffer_, p)) {
    server_->send(503, "text/plain", "preview_frame_unavailable"); return;
  }
  // HTTP owns this frozen copy until the next capture request. Subsequent
  // camera frames cannot change bytes or metadata during chunked transfer.
  if (++preview_token_ == 0) ++preview_token_;
  const uint32_t crc = export_protocol::crc32(0, preview_buffer_, FootObserver::kPreviewBytes);
  String json; json.reserve(4100);
  json = "{\"revision\":\"" RUNTIME_VERSION "\",\"format\":\"gray8\",\"width\":160,\"height\":120,";
  json += "\"source_width\":320,\"source_height\":240,\"bytes\":" + String(FootObserver::kPreviewBytes);
  json += ",\"token\":" + String(preview_token_) + ",\"crc32\":" + String(crc);
  json += ",\"sequence\":" + String(p.frame.sequence) + ",\"run_id\":" + String(p.frame.run_id);
  json += ",\"boot_id\":" + String(boot_id_) + ",\"state_id\":" + String(p.frame.state_id);
  char timestamp[24];
  snprintf(timestamp, sizeof(timestamp), "%llu", static_cast<unsigned long long>(p.frame.frame_us));
  json += ",\"frame_us\":" + String(timestamp);
  snprintf(timestamp, sizeof(timestamp), "%llu", static_cast<unsigned long long>(p.frame.delivered_us));
  json += ",\"delivered_us\":" + String(timestamp);
  json += ",\"age_ms\":" + String((esp_timer_get_time() - p.frame.delivered_us) / 1000.0, 1);
  json += ",\"timestamp_valid\":" + String(p.frame.timestamp_valid ? "true" : "false");
  json += ",\"zero_ready\":" + String(p.frame.zero_ready ? "true" : "false");
  json += ",\"zero_reason\":\"" + String(footZeroReasonName(p.frame.zero_reason)) + "\"";
  json += ",\"zero_samples\":" + String(p.zero_samples);
  json += ",\"right_zero_x\":" + num(p.right_zero) + ",\"left_zero_x\":" + num(p.left_zero);
  json += ",\"right_deg\":" + num(p.frame.right_deg) + ",\"left_deg\":" + num(p.frame.left_deg);
  json += ",\"right_valid\":" + String(p.frame.right_valid ? "true" : "false");
  json += ",\"left_valid\":" + String(p.frame.left_valid ? "true" : "false");
  json += ",\"right_in_range\":" + String(p.frame.right_in_range ? "true" : "false");
  json += ",\"left_in_range\":" + String(p.frame.left_in_range ? "true" : "false");
  json += ",\"range\":" + footRangeDiagnosticsJson();
  json += ",\"calibration\":" + footCalibrationDiagnosticsJson();
  json += ",\"mekf\":" + mekfAttitudeJson(p.mekf_attitude, micros(), p.imu_ok);
  json += ",\"mekf_time_semantics\":\"control_snapshot_at_frame_delivery_not_exposure\"";
  json += ",\"right\":" + previewMarkerJson(p.right) + ",\"left\":" + previewMarkerJson(p.left) + "}";
  server_->sendHeader("Cache-Control", "no-store"); server_->send(200, "application/json", json);
}
void WebUi::previewChunk() {
  RuntimeDiag::Scope diagnostic(RuntimeDiag::Lane::Http, RuntimeDiag::Phase::HttpChunk);
  uint32_t token = 0, offset = 0, length = 0;
  if (!previewAllowed() || !preview_buffer_ || !preview_token_ ||
      !unsignedArg(server_->arg("token"), token) || token != preview_token_ ||
      !unsignedArg(server_->arg("offset"), offset) || !unsignedArg(server_->arg("length"), length) ||
      !export_protocol::validRange(FootObserver::kPreviewBytes, offset, length, export_protocol::kChunkBytes)) {
    server_->send(409, "text/plain", "preview_token_or_range_mismatch"); return;
  }
  memcpy(chunk_buffer_ + 16, preview_buffer_ + offset, length);
  const uint32_t header[] = {export_protocol::kChunkMagic, offset, length,
      export_protocol::crc32(0, chunk_buffer_ + 16, length)};
  memcpy(chunk_buffer_, header, 16);
  server_->sendHeader("Cache-Control", "no-store");
  server_->setContentLength(length + 16); server_->send(200, "application/octet-stream", "");
  server_->sendContent(reinterpret_cast<const char*>(chunk_buffer_), length + 16);
}
