#include "compact_json_table.h"
#include "foot_observer.h"
#include "foot_range_diagnostics.h"
#include "foot_calibration_diagnostics.h"
#include "runtime_diagnostics.h"
#include "foot_angle_estimator.h"
#include "esp_timer.h"
#include <new>

bool FootObserver::begin(OneShotCamera& camera, RunControlWorker& control) {
  camera_ = &camera; control_ = &control;
  frames_ = static_cast<FootFrame*>(ps_malloc(sizeof(FootFrame) * kCapacity));
  if (!frames_ || !camera.snapshot().camera_ok || !control.ready()) return false;
  preview_ = static_cast<uint8_t*>(ps_malloc(kPreviewBytes));
  preview_mutex_ = xSemaphoreCreateMutex();
  portENTER_CRITICAL(&mux_); status_.available = true; portEXIT_CRITICAL(&mux_);
  if (xTaskCreatePinnedToCore(entry, "foot_observer", 10240, this, 1, &task_, 0) != pdPASS) {
    portENTER_CRITICAL(&mux_); status_.available = false; portEXIT_CRITICAL(&mux_); return false;
  }
  return true;
}
bool FootObserver::copyPreview(uint8_t* out, FootPreviewInfo& info) const {
  if (!out || !preview_ || !preview_mutex_ || xSemaphoreTake(preview_mutex_, 0) != pdTRUE) return false;
  const bool valid = preview_info_.frame.frame_valid;
  if (valid) { memcpy(out, preview_, kPreviewBytes); info = preview_info_; }
  xSemaphoreGive(preview_mutex_);
  return valid;
}
void FootObserver::publishPreview(const uint8_t* gray, const FootPreviewInfo& info, bool run_active) {
  // HTTP already excludes previews during a run. Keep tracking/recording every
  // frame, but avoid the unused 19,200-byte thumbnail copy through both syncs
  // and measurement. The last idle preview keeps its original time/metadata.
  if (run_active) return;
  // Never hold an interrupt-disabling spinlock around PSRAM copies. The image
  // and its detection metadata share a mutex; a busy reader simply skips this
  // optional preview update. Network transmission never holds this mutex.
  if (!preview_ || !preview_mutex_ || xSemaphoreTake(preview_mutex_, 0) != pdTRUE) return;
  if (gray) for (uint32_t y = 0; y < kPreviewHeight; ++y)
    for (uint32_t x = 0; x < kPreviewWidth; ++x) preview_[y * kPreviewWidth + x] = gray[(y * 2) * 320 + x * 2];
  preview_info_ = info;
  xSemaphoreGive(preview_mutex_);
}
FootSnapshot FootObserver::snapshot() const {
  portENTER_CRITICAL(&mux_); const auto copy = status_; portEXIT_CRITICAL(&mux_);
  return copy;
}
void FootObserver::recordControlObservation(uint16_t run_id, uint32_t sequence, uint64_t observed_us) {
  // A later camera publication can replace the last row after snapshot().
  // Never attach an observation to a different sequence; count the miss.
  portENTER_CRITICAL(&mux_);
  if (status_.recording && run_id == recording_run_id_) {
    ++control_observation_attempts_;
    bool matched = false;
    if (status_.count && frames_) {
      auto& f = frames_[status_.count - 1];
      if (f.run_id == run_id && f.sequence == sequence && !f.control_observed_us &&
          observed_us >= f.delivered_us) { f.control_observed_us = observed_us; matched = true; }
    }
    if (!matched) ++control_observation_unmatched_;
  }
  portEXIT_CRITICAL(&mux_);
}
void FootObserver::beginRun(uint16_t id, uint64_t epoch) {
  portENTER_CRITICAL(&mux_);
  recording_run_id_ = id; log_epoch_us_ = epoch;
  control_observation_attempts_ = control_observation_unmatched_ = 0;
  status_.count = 0; status_.overflow = false; status_.recording = true;
  portEXIT_CRITICAL(&mux_);
}
void FootObserver::finishRun() {
  portENTER_CRITICAL(&mux_); status_.recording = false; portEXIT_CRITICAL(&mux_);
}
void FootObserver::clearRun() {
  portENTER_CRITICAL(&mux_);
  status_.recording = false; status_.count = 0; status_.overflow = false;
  portEXIT_CRITICAL(&mux_);
}
void FootObserver::loop() {
  uint32_t seq = 0;
  uint64_t last_valid_us = 0;
  for (;;) {
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Snapshot);
    const auto run_before = control_->snapshot();
    // Capture for boot zero/idle alignment and throughout START/MEASURE/END.
    // Once a run is complete the camera is idle while logs are prepared/sent.
    if (run_before.state_id == 4 || run_before.state_id == 5) {
      RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraStop);
      camera_->stopContinuous();
      RuntimeDiag::beat(RuntimeDiag::Lane::Camera, seq);
      RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Wait);
      vTaskDelay(pdMS_TO_TICKS(20)); continue;
    }
    const uint64_t begin = esp_timer_get_time();
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraCapture);
    camera_fb_t* fb = camera_->startContinuous() ? camera_->acquireContinuous(200) : nullptr;
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Snapshot);
    const auto run = control_->snapshot();
    FootFrame f{};
    f.sequence = ++seq; f.delivered_us = esp_timer_get_time();
    f.imu_sample_us = run.imu_sample_us; f.state_id = run.state_id;
    f.led_state = run.led_state; f.sync_event_id = run.sync_event_id;
    f.run_id = run.run_id;
    f.frame_valid = fb && fb->buf && fb->len == 320U * 240U &&
        fb->width == 320 && fb->height == 240 && fb->format == PIXFORMAT_GRAYSCALE;
    if (fb) {
      f.frame_us = static_cast<uint64_t>(fb->timestamp.tv_sec) * 1000000ULL + fb->timestamp.tv_usec;
      f.timestamp_valid = f.frame_us > 0 && f.frame_us <= f.delivered_us &&
          f.delivered_us - f.frame_us < 500000;
    }
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraProcess);
    const uint32_t processing_start = micros();
    const auto a = right_.process(f.frame_valid ? fb->buf : nullptr);
    const auto b = left_.process(f.frame_valid ? fb->buf : nullptr);
    // Check the IMU at BOTH sides of capture; any instability in between changes
    // the epoch. A stale queued image must not contribute to upright zeroing.
    zero_.observe(millis(), run.upright_epoch,
        run.upright_stable && run_before.upright_stable &&
        run.upright_epoch == run_before.upright_epoch && !run.running &&
        f.timestamp_valid && f.frame_us >= run.upright_since_us,
        a.valid && b.valid, a.center_x_px, b.center_x_px);
    const auto ar = estimateFootAngle(a, zero_.a_zero, zero_.ready && f.timestamp_valid);
    const auto bl = estimateFootAngle(b, zero_.b_zero, zero_.ready && f.timestamp_valid);
    f.zero_ready = zero_.ready; f.right_valid = ar.valid; f.left_valid = bl.valid;
    f.right_in_range = a.valid && ar.in_calibration_range;
    f.left_in_range = b.valid && bl.in_calibration_range;
    if (a.valid) { f.right_x = a.center_x_px; f.right_deg = ar.angle_deg; }
    if (b.valid) { f.left_x = b.center_x_px; f.left_deg = bl.angle_deg; }
    f.right_contrast = a.peak_contrast; f.left_contrast = b.peak_contrast;
    if (a.valid) f.right_scan_y = a.center_y_px;
    if (b.valid) f.left_scan_y = b.center_y_px;
    f.right_weight = a.weight_sum; f.left_weight = b.weight_sum;
    f.right_reason = a.reason; f.left_reason = b.reason;
    f.right_templates = a.templates_tested; f.left_templates = b.templates_tested;
    f.right_candidates = a.candidate_count; f.left_candidates = b.candidate_count;
    f.right_ambiguity = a.ambiguity_ratio; f.left_ambiguity = b.ambiguity_ratio;
    f.zero_reason = zero_.reason;
    f.processing_us = micros() - processing_start;
    FootPreviewInfo preview;
    preview.frame = f; preview.right = a; preview.left = b;
    preview.mekf_attitude = run.mekf_attitude; preview.imu_ok = run.imu_ok;
    preview.right_zero = zero_.a_zero; preview.left_zero = zero_.b_zero; preview.zero_samples = zero_.count;
    publishPreview(f.frame_valid ? fb->buf : nullptr, preview,
        run_before.running || run.running);
    f.processing_us = micros() - processing_start;
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraRelease);
    camera_->releaseContinuous(fb);
    float fps = 0;
    if (f.frame_valid) {
      if (last_valid_us && f.delivered_us > last_valid_us) fps = 1000000.0f / (f.delivered_us - last_valid_us);
      last_valid_us = f.delivered_us;
    }
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Publish);
    portENTER_CRITICAL(&mux_);
    if (!f.frame_valid) ++status_.frame_failures;
    else {
      ++status_.captured_frames;
      if (!a.valid) ++status_.right_marker_failures;
      if (!b.valid) ++status_.left_marker_failures;
    }
    if (f.processing_us > status_.processing_max_us) status_.processing_max_us = f.processing_us;
    if (status_.recording && f.run_id == recording_run_id_) {
      // Signed offsets retain frame-boundary cases; never label them as zero.
      f.log_time_us = f.timestamp_valid ? static_cast<int64_t>(f.frame_us) - log_epoch_us_ : -1;
      f.measurement_time_us = f.timestamp_valid && run.measurement_epoch_us
          ? static_cast<int64_t>(f.frame_us) - run.measurement_epoch_us : -1;
      if (status_.count < kCapacity) frames_[status_.count++] = f;
      else status_.overflow = true;
    }
    status_.latest = f; status_.zero_ready = zero_.ready;
    status_.right_zero = zero_.a_zero; status_.left_zero = zero_.b_zero;
    status_.zero_samples = zero_.count;
    status_.zero_reason = zero_.reason;
    status_.preview_available = preview_ && preview_mutex_;
    status_.fps = fps;
    portEXIT_CRITICAL(&mux_);
    RuntimeDiag::beat(RuntimeDiag::Lane::Camera, seq);
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Wait);
    const int64_t remaining = 66667 - (esp_timer_get_time() - begin);
    if (remaining > 0) vTaskDelay(pdMS_TO_TICKS((remaining + 999) / 1000));
    else vTaskDelay(1);
  }
}

static String number(float v) { return isfinite(v) ? String(v, 5) : String("null"); }
void FootObserver::appendMetadata(PsramString& json) const {
  const auto s = snapshot();
  json += "\"foot_observation\":{\"revision\":\"freefoot_runtime_v2_0477\",\"observation_only\":false,\"balance_control_scope\":\"work_mode_target8_calibrated_phase_pair_positive_energy_cap_v1\",";
  json += "\"detector\":\"" + String(appcfg::kWhiteDetectorRevision) + "\",";
  json += "\"weak_candidate_confirmation\":{\"frames\":" + String(appcfg::kWhiteReacquireFrames);
  json += ",\"min_x_step_px\":" + String(appcfg::kWhiteWeakXStepPx, 1);
  json += ",\"min_y_step_px\":" + String(appcfg::kWhiteWeakYStepPx, 1);
  json += ",\"contrast_ratio_below\":" + String(appcfg::kWhiteWeakContrastRatio, 2);
  json += ",\"weight_ratio_below\":" + String(appcfg::kWhiteWeakWeightRatio, 2);
  json += ",\"condition\":\"recent_track_and_xy_steps_and_both_quality_drops\"},";
  json += "\"scan_y_semantics\":\"selected_row_template_center_not_marker_centroid\",";
  json += "\"vertical_recovery_angle_accuracy_validated\":false,";
  json += "\"search_radius_y_px\":" + String(appcfg::kWhiteSearchRadiusYPx);
  json += ",\"search_step_y_px\":" + String(appcfg::kWhiteSearchStepYPx) + ",";
  json += "\"calibration_source_commit\":null,\"original_calibration_source_commit\":\"ac6df8caf59c93956b87cba57521903c25ff9f00\",";
  json += "\"calibration\":" + footCalibrationDiagnosticsJson() + ",";
  json += "\"mapping\":\"right=A upper lane;left=B lower lane\",\"positive_direction\":\"marker_x_decreases\",";
  json += "\"frame_timestamp_semantics\":\"camera_driver_frame_timestamp_not_verified_exposure_time\",";
  json += "\"control_context_semantics\":\"latest_control_snapshot_at_frame_delivery_not_exposure\",";
  json += "\"preview_generation\":\"idle_only_last_idle_frame_retained\",";
  json += "\"target_fps\":15,\"capacity\":" + String(kCapacity);
  json += ",\"count\":" + String(s.count) + ",\"overflow\":" + String(s.overflow ? "true" : "false");
  json += ",\"available\":" + String(s.available ? "true" : "false");
  json += ",\"zero_ready\":" + String(s.zero_ready ? "true" : "false");
  json += ",\"zero_reason\":\"" + String(footZeroReasonName(s.zero_reason)) + "\"";
  json += ",\"zero_max_nominal_offset_px\":" + number(appcfg::kAutoZeroMaxNominalOffsetXPx);
  json += ",\"zero_max_spread_px\":" + number(appcfg::kAutoZeroMaxSpreadXPx);
  json += ",\"zero_pitch_reference\":\"raw_imu_x_normalized_gravity_vs_historical_upright;roll_offset_allowed_if_stationary;upright_hemisphere_required\"";
  json += ",\"zero_max_pitch_error_deg\":" + number(appcfg::kAutoZeroMaxPitchErrorDeg);
  json += ",\"right_zero_x\":" + number(s.right_zero) + ",\"left_zero_x\":" + number(s.left_zero);
  json += ",\"right_deg_per_px\":" + String(appcfg::kFootAngleADegPerPx, 9);
  json += ",\"left_deg_per_px\":" + String(appcfg::kFootAngleBDegPerPx, 9) + ",";
  json += "\"right_support_x\":[" + number(appcfg::kFootAngleAMinCalXPx) + "," + number(appcfg::kFootAngleAMaxCalXPx) + "]";
  json += ",\"left_support_x\":[" + number(appcfg::kFootAngleBMinCalXPx) + "," + number(appcfg::kFootAngleBMaxCalXPx) + "]";
  json += ",\"range\":" + footRangeDiagnosticsJson() + "},\"foot_frames\":";
  CompactJsonTable<PsramString> table(json);
  for (uint32_t i = 0; i < s.count; ++i) {
    // Recording is sealed before export begins and clear/start are excluded.
    const auto& f = frames_[i];
    String row;
    char buf[480];
    snprintf(buf, sizeof(buf),
        "{\"sequence\":%lu,\"run_id\":%u,\"frame_us\":%llu,\"delivered_us\":%llu,"
        "\"log_time_us\":%lld,\"measurement_time_us\":%lld,\"imu_sample_us\":%lu,"
        "\"state_id\":%u,\"led_state\":%u,\"sync_event_id\":%u,\"processing_us\":%lu,"
        "\"timestamp_valid\":%s,\"frame_valid\":%s,\"zero_ready\":%s,"
        "\"right_valid\":%s,\"left_valid\":%s,\"right_in_range\":%s,\"left_in_range\":%s",
        static_cast<unsigned long>(f.sequence), f.run_id, f.frame_us, f.delivered_us,
        f.log_time_us, f.measurement_time_us, static_cast<unsigned long>(f.imu_sample_us),
        f.state_id, f.led_state, f.sync_event_id, static_cast<unsigned long>(f.processing_us),
        f.timestamp_valid ? "true" : "false", f.frame_valid ? "true" : "false", f.zero_ready ? "true" : "false",
        f.right_valid ? "true" : "false", f.left_valid ? "true" : "false",
        f.right_in_range ? "true" : "false", f.left_in_range ? "true" : "false");
    row += buf;
    row += ",\"right_x\":" + number(f.right_x) + ",\"left_x\":" + number(f.left_x);
    row += ",\"right_deg\":" + number(f.right_deg) + ",\"left_deg\":" + number(f.left_deg);
    row += ",\"right_contrast\":" + number(f.right_contrast) + ",\"left_contrast\":" + number(f.left_contrast);
    row += ",\"right_scan_y\":" + number(f.right_scan_y) + ",\"left_scan_y\":" + number(f.left_scan_y);
    row += ",\"right_weight\":" + number(f.right_weight) + ",\"left_weight\":" + number(f.left_weight);
    row += ",\"right_reason\":\"" + String(markerDetectionReasonName(f.right_reason)) + "\"";
    row += ",\"left_reason\":\"" + String(markerDetectionReasonName(f.left_reason)) + "\"";
    row += ",\"right_templates\":" + String(f.right_templates) + ",\"left_templates\":" + String(f.left_templates);
    row += ",\"right_candidates\":" + String(f.right_candidates) + ",\"left_candidates\":" + String(f.left_candidates);
    row += ",\"right_ambiguity\":" + number(f.right_ambiguity) + ",\"left_ambiguity\":" + number(f.left_ambiguity);
    row += ",\"zero_reason\":\"" + String(footZeroReasonName(f.zero_reason)) + "\"}";
    if (!table.append(row)) { json.fail(); return; }
  }
  if (!table.finish()) { json.fail(); return; }
  // Sparse supplemental table: [foot_frames row index, observed-delivered us].
  // Keep the existing 80% metadata headroom even in a maximum-size log.
  // A truncated table is explicit; omitted rows always mean unavailable.
  // Select ranks uniformly over the *whole run*, including first and last,
  // rather than recording only the first part of walking. Reserve a conservative
  // 32 bytes per JSON entry (index, full uint64 delta, delimiters).
  String observed_rows;
  uint32_t total_observed = 0, valid_observed = 0, stored_observed = 0;
  constexpr uint32_t kMaxStoredObservations = 64;
  constexpr size_t kSoftEnd = 640U * 1024U * 8U / 10U;
  constexpr size_t kTailReserve = 256U;
  constexpr size_t kMaxEntryBytes = 32U;
  for (uint32_t i = 0; i < s.count; ++i) {
    const auto& f = frames_[i];
    if (!f.control_observed_us) continue;
    ++total_observed;
    if (f.control_observed_us >= f.delivered_us) ++valid_observed;
  }
  size_t capacity = json.length() + kTailReserve < kSoftEnd
      ? (kSoftEnd - json.length() - kTailReserve - 1U) / kMaxEntryBytes : 0;
  if (capacity > kMaxStoredObservations) capacity = kMaxStoredObservations;
  if (capacity > valid_observed) capacity = valid_observed;
  uint32_t rank = 0;
  for (uint32_t i = 0; i < s.count && stored_observed < capacity; ++i) {
    const auto& f = frames_[i];
    if (!f.control_observed_us || f.control_observed_us < f.delivered_us) continue;
    const uint32_t target = capacity == 1 ? (valid_observed - 1U) / 2U :
        static_cast<uint32_t>((static_cast<uint64_t>(stored_observed) * (valid_observed - 1U)) / (capacity - 1U));
    if (rank++ != target) continue;
    char entry[64];
    snprintf(entry, sizeof(entry), "%s[%lu,%llu]", observed_rows.length() ? "," : "",
        static_cast<unsigned long>(i), f.control_observed_us - f.delivered_us);
    const size_t old_length = observed_rows.length();
    observed_rows += entry;
    if (observed_rows.length() != old_length + strlen(entry)) break;
    ++stored_observed;
  }
  const bool truncated = stored_observed < total_observed;
  {
    json += ",\"foot_control_observations\":{\"version\":1,\"truncated\":";
    json += truncated ? "true" : "false";
    json += ",\"total\":" + String(total_observed) + ",\"stored\":" + String(stored_observed);
    json += ",\"attempts\":" + String(control_observation_attempts_) + ",\"unmatched\":" + String(control_observation_unmatched_);
    json += ",\"semantics\":\"control_snapshot_return_before_runner_observe_v1\"";
    json += ",\"rows\":["; json += observed_rows; json += "]}";
  }
}
