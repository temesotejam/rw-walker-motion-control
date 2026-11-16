#include "compact_json_table.h"
#include "foot_observer.h"
#if defined(CAMERA_FRAME_READY_V21) && CAMERA_FRAME_READY_V21
#include "camera_frame_ready.h"
#endif
#include "foot_range_diagnostics.h"
#include "foot_control_range.h"
#include "foot_calibration_diagnostics.h"
#include "runtime_diagnostics.h"
#include "foot_angle_estimator.h"
#include "foot_fast_policy.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <new>
#if defined(CAD_SUPPORT_SHADOW) && CAD_SUPPORT_SHADOW
#include "cad_support.h"
#endif

bool FootObserver::begin(OneShotCamera& camera, RunControlWorker& control) {
  camera_ = &camera; control_ = &control;
  frames_ = static_cast<FootFrame*>(ps_malloc(sizeof(FootFrame) * kCapacity));
  if (!frames_ || !camera.snapshot().camera_ok || !control.ready()) return false;
  preview_ = static_cast<uint8_t*>(ps_malloc(kPreviewBytes));
  preview_mutex_ = xSemaphoreCreateMutex();
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  row_storage_=static_cast<uint8_t*>(heap_caps_malloc(foot_image_rows::kRequiredBytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
  preview_staging_=static_cast<uint8_t*>(ps_malloc(kPreviewBytes));
  if(!row_storage_ || !preview_staging_ || !preview_ || !preview_mutex_)return false;
  row_cache_=foot_image_rows::RowCache(row_storage_,foot_image_rows::kRequiredBytes);
#endif
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
  upper_storage_=static_cast<uint8_t*>(ps_malloc(foot_image_rows::kUpperBytes));
  if(!upper_storage_ || !camera_row_overlap::initialize(upper_storage_,foot_image_rows::kUpperBytes,prepareUpper,this))return false;
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  lower_storage_=static_cast<uint8_t*>(ps_malloc(foot_image_rows::kLowerBytes));
  if(!lower_storage_ || !camera_row_overlap::initializeLower(lower_storage_,foot_image_rows::kLowerBytes,prepareLower,this))return false;
#endif
  portENTER_CRITICAL(&mux_); status_.available = true;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  status_.row_cache_ready=true;
#endif
  portEXIT_CRITICAL(&mux_);
  constexpr uint32_t stack_bytes=
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
      12288; // Includes the acquisition callback's candidate-preparation stack.
#else
      10240;
#endif
  if (xTaskCreatePinnedToCore(entry, "foot_observer", stack_bytes, this, kTaskPriority, &task_, kTaskCore) != pdPASS) {
    portENTER_CRITICAL(&mux_); status_.available = false; portEXIT_CRITICAL(&mux_); return false;
  }
  return true;
}
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
void FootObserver::prepareUpper(const foot_image_rows::GrayRowView& rows,uint64_t stamp,uint32_t,void* context) {
  auto& self=*static_cast<FootObserver*>(context);
  const uint32_t begin=micros();
  // Only this task uses row_storage_, and acquisition precedes normal capture.
  // Reuse its internal RAM for computation; the HAL mailbox lives in PSRAM.
  self.row_cache_.invalidate();
  foot_image_rows::UpperRowCache scratch(self.row_storage_,foot_image_rows::kRequiredBytes);
  if(!scratch.capture(rows))return;
  self.upper_features_=self.right_.prepare(scratch.view()); // No tracking state mutation.
  self.upper_features_.processing_us=micros()-begin; // Includes mailbox-to-scratch copy.
  self.upper_frame_us_=stamp;
  ++self.upper_attempts_;
}
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
void FootObserver::prepareLower(const foot_image_rows::GrayRowView& rows,uint64_t stamp,uint32_t,int max_logical_y,void* context) {
  auto& self=*static_cast<FootObserver*>(context);
  const uint32_t begin=micros();
  ++self.lower_attempts_; // Callback attempts, not the number of admitted frames.
  if(!stamp){self.lower_frame_us_=0;self.lower_prepare_us_=0;return;}
  if(stamp!=self.lower_frame_us_) {
    self.lower_scan_=self.left_.beginScan();
    self.lower_frame_us_=stamp;
    self.lower_prepare_us_=0;
  }
  // All earlier required rows from this same immutable frame remain available.
  // Recopy the available set into internal RAM; neither scan nor features alias it.
  self.row_cache_.invalidate();
  foot_image_rows::LowerRowCache scratch(self.row_storage_,foot_image_rows::kRequiredBytes);
  if(!scratch.capture(rows,max_logical_y) ||
      !self.left_.appendAvailable(self.lower_scan_,scratch.view(),scratch.maxLogicalY())) {
    self.lower_frame_us_=0; // Failed speculative work cannot be admitted.
  }
  self.lower_prepare_us_+=micros()-begin;
}
#endif
bool FootObserver::copyPreview(uint8_t* out, FootPreviewInfo& info) const {
  if (!out || !preview_ || !preview_mutex_ || xSemaphoreTake(preview_mutex_, 0) != pdTRUE) return false;
  const bool valid = preview_info_.frame.frame_valid;
  if (valid) { memcpy(out, preview_, kPreviewBytes); info = preview_info_; }
  xSemaphoreGive(preview_mutex_);
  return valid;
}
void FootObserver::publishPreview(const uint8_t* gray, const FootPreviewInfo& info, bool run_active,bool thumbnail) {
  // HTTP already excludes previews during a run. Keep tracking/recording every
  // frame, but avoid the unused 19,200-byte thumbnail copy through both syncs
  // and measurement. The last idle preview keeps its original time/metadata.
  if (run_active) return;
  // Never hold an interrupt-disabling spinlock around PSRAM copies. The image
  // and its detection metadata share a mutex; a busy reader simply skips this
  // optional preview update. Network transmission never holds this mutex.
  if (!preview_ || !preview_mutex_ || xSemaphoreTake(preview_mutex_, 0) != pdTRUE) return;
  if(gray && thumbnail)memcpy(preview_,gray,kPreviewBytes);
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  else if(gray)foot_image_rows::copyNativeThumbnail(gray,foot_image_rows::kRawFrameBytes,preview_,kPreviewBytes);
#else
  else if (gray) for (uint32_t y = 0; y < kPreviewHeight; ++y)
    for (uint32_t x = 0; x < kPreviewWidth; ++x) preview_[y * kPreviewWidth + x] = gray[(y * 2) * 320 + x * 2];
#endif
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
#if (defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23) || (defined(FOOT_RECEIPT_AUDIT) && FOOT_RECEIPT_AUDIT)
  // Idle Core1 polling measures the same receipt boundary without starting a
  // run. Never attach a receipt to a newer frame that replaced the snapshot.
  auto& latest=status_.latest;
  if(latest.run_id==run_id && latest.sequence==sequence && latest.timestamp_valid &&
     !latest.control_observed_us && latest.ready_us && observed_us>=latest.ready_us)
    latest.control_observed_us=observed_us;
#endif
  if (status_.recording && run_id == recording_run_id_) {
    ++control_observation_attempts_;
    bool matched = false;
    if (status_.count && frames_) {
      auto& f = frames_[status_.count - 1];
      if (f.run_id == run_id && f.sequence == sequence && !f.control_observed_us &&
          observed_us >= f.delivered_us) { f.control_observed_us = observed_us; matched = true;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
        if(status_.latest.run_id==run_id && status_.latest.sequence==sequence)
          status_.latest.control_observed_us=observed_us;
#endif
      }
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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  uint64_t last_idle_preview_us=0;
  bool have_idle_preview=false;
  constexpr uint64_t idle_preview_period_us=200000;
#endif
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
  foot_phase_prediction::Predictor pose_predictor;
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  foot_local_prediction::Predictor local_predictor;
#endif
#endif
#if defined(CAD_SUPPORT_SHADOW) && CAD_SUPPORT_SHADOW
  // Single camera-task owner; no geometry work in the real-time control task.
  foot_pose_prediction::Predictor cad_predictor;
#endif
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
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    upper_frame_us_=0; // A candidate belongs only to this acquisition attempt.
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    lower_frame_us_=0;
    lower_prepare_us_=0;
#endif
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraCapture);
    camera_fb_t* fb = camera_->startContinuous() ? camera_->acquireContinuous(200) : nullptr;
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Snapshot);
    const auto run = control_->snapshot();
    FootFrame f{};
    f.sequence = ++seq; f.delivered_us = esp_timer_get_time();
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    f.capture_wait_us=static_cast<uint32_t>(f.delivered_us-begin);
#endif
    f.imu_sample_us = run.imu_sample_us; f.state_id = run.state_id;
    f.led_state = run.led_state; f.sync_event_id = run.sync_event_id;
    f.run_id = run.run_id;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    f.frame_valid = fb && fb->buf && fb->len == foot_image_rows::kRawFrameBytes &&
        fb->width == foot_image_rows::kRawFrameWidth && fb->height == foot_image_rows::kRawFrameHeight &&
        fb->format == PIXFORMAT_GRAYSCALE;
#else
    f.frame_valid = fb && fb->buf && fb->len == 320U * 240U &&
        fb->width == 320 && fb->height == 240 && fb->format == PIXFORMAT_GRAYSCALE;
#endif
    if (fb) {
      f.frame_us = static_cast<uint64_t>(fb->timestamp.tv_sec) * 1000000ULL + fb->timestamp.tv_usec;
      f.timestamp_valid = f.frame_us > 0 && f.frame_us <= f.delivered_us &&
          f.delivered_us - f.frame_us < 500000;
    }
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraProcess);
    const uint32_t processing_start = micros();
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    const bool preview_active=!(run_before.running || run.running);
    const uint32_t copy_start=micros();
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    const bool both_prepared=f.frame_valid && f.timestamp_valid &&
        upper_frame_us_ && upper_frame_us_==f.frame_us &&
        lower_frame_us_ && lower_frame_us_==f.frame_us && lower_scan_.complete();
    // Complete owned features need no second image-row copy. A mismatch on
    // either lane retains the normal full cache for that lane's fallback.
    if(both_prepared)row_cache_.invalidate();
    else {
      f.row_cache_used=f.frame_valid && row_cache_.capture(fb->buf,fb->len);
      if(!f.row_cache_used){row_cache_.invalidate();f.frame_valid=false;}
    }
#else
    f.row_cache_used=f.frame_valid && row_cache_.capture(fb->buf,fb->len);
    if(!f.row_cache_used){row_cache_.invalidate();f.frame_valid=false;}
#endif
    f.row_copy_us=micros()-copy_start;
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    const bool preview_due=preview_active && (!f.frame_valid || !have_idle_preview ||
        f.delivered_us-last_idle_preview_us>=idle_preview_period_us);
#endif
    if(f.frame_valid &&
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
        preview_due
#else
        preview_active
#endif
    ){
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
      foot_image_rows::copyNativeThumbnail(fb->buf,fb->len,preview_staging_,kPreviewBytes);
#else
      for(uint32_t y=0;y<kPreviewHeight;++y)for(uint32_t x=0;x<kPreviewWidth;++x)
        preview_staging_[y*kPreviewWidth+x]=fb->buf[(y*2)*320+x*2];
#endif
    }
    // No camera pixels may be accessed after this point. Tracker and preview
    // own independent copies, including on idle/zero-calibration frames.
    const bool had_frame=fb!=nullptr;
    camera_->releaseContinuous(fb);fb=nullptr;f.released_us=had_frame?esp_timer_get_time():0;
    const uint32_t recognition_start=micros();
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    f.upper_overlap_used=f.frame_valid && f.timestamp_valid && upper_frame_us_ && upper_frame_us_==f.frame_us;
    const auto a=f.upper_overlap_used?right_.processPrepared(upper_features_):right_.process(row_cache_.view());
    f.upper_prepare_us=f.upper_overlap_used?upper_features_.processing_us:0;
#else
    const auto a=right_.process(row_cache_.view());
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    const bool lower_candidate=f.frame_valid && f.timestamp_valid && lower_frame_us_ &&
        lower_frame_us_==f.frame_us && lower_scan_.valid();
    const uint8_t lower_templates_before_delivery=lower_candidate?lower_scan_.templatesTested():0;
    // The complete frame can arrive before the last mailbox callback. Finish
    // only this matching scan from owned full rows; never poll another frame's
    // mailbox after dequeue. This remaining work belongs to recognition_us.
    if(lower_candidate && !lower_scan_.complete())
      left_.appendAvailable(lower_scan_,row_cache_.view(),appcfg::kFrameHeight-1);
    const bool lower_overlap_used=lower_candidate && lower_scan_.complete();
    const auto b=lower_overlap_used?left_.processPrepared(left_.finishScan(lower_scan_)):
        left_.process(row_cache_.view());
#else
    const auto b=left_.process(row_cache_.view());
#endif
    f.recognition_us=micros()-recognition_start;
#else
    const auto a = right_.process(f.frame_valid ? fb->buf : nullptr);
    const auto b = left_.process(f.frame_valid ? fb->buf : nullptr);
#endif
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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    // Publish the staged image after ready/status, retaining its own matching
    // frame metadata. Undue frames do neither thumbnail copy and still update
    // all tracking, zeroing, predictions, status and archive observations.
#elif defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    publishPreview(f.frame_valid ? preview_staging_ : nullptr,preview,run_before.running || run.running,true);
#else
    publishPreview(f.frame_valid ? fb->buf : nullptr, preview,run_before.running || run.running);
#endif
    f.processing_us = micros() - processing_start;
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::CameraRelease);
#if !defined(FOOT_LOW_LATENCY_V20) || !FOOT_LOW_LATENCY_V20
    camera_->releaseContinuous(fb);
#endif
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
    const uint32_t pose_start=micros();
    const uint64_t pose_now=esp_timer_get_time();
    const bool pose_time=run.measurement_epoch_us && f.frame_us>=run.measurement_epoch_us && pose_now>=f.frame_us;
    const bool pose_valid=pose_time && f.timestamp_valid && f.frame_valid && f.zero_ready &&
        f.left_valid && f.right_valid &&
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
        foot_control_range::allows(f.left_in_range,f.left_x,false) && foot_control_range::allows(f.right_in_range,f.right_x,true) &&
#else
        f.left_in_range && f.right_in_range &&
#endif
        run_before.run_id==run.run_id && run_before.state_id==3 && run.state_id==3 && run.imu_ok;
    pose_predictor.observe(f.run_id,
        pose_time ? static_cast<uint32_t>((f.frame_us-run.measurement_epoch_us)/1000) : 0,
        pose_time ? static_cast<uint32_t>((pose_now-run.measurement_epoch_us)/1000) : 0,
        pose_valid,f.left_deg,f.right_deg);
    f.pose_projection=pose_predictor.projection();
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
    const uint32_t local_start=micros();
    local_predictor.observe(f.run_id,
        pose_time ? static_cast<uint32_t>((f.frame_us-run.measurement_epoch_us)/1000) : 0,
        pose_time ? static_cast<uint32_t>((pose_now-run.measurement_epoch_us)/1000) : 0,
        pose_valid,f.left_deg,f.right_deg);
    f.local_projection=local_predictor.projection();
    f.local_compute_us=micros()-local_start;
#endif
    // Availability includes history matching; a driver timestamp is not a
    // verified exposure timestamp. History matching stays in the foot worker,
    // below the control task's priority, outside the control task itself.
    if(pose_time)f.pose_projection.received_ms=static_cast<uint32_t>((esp_timer_get_time()-run.measurement_epoch_us)/1000);
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
    f.local_projection.received_ms=f.pose_projection.received_ms;
#endif
    f.pose_compute_us=micros()-pose_start;
#endif
#if defined(CAD_SUPPORT_SHADOW) && CAD_SUPPORT_SHADOW
    const uint32_t cad_start=micros();
    const uint64_t cad_now=esp_timer_get_time();
    const bool cad_time=run.measurement_epoch_us && f.frame_us>=run.measurement_epoch_us && cad_now>=f.frame_us;
    const bool cad_valid=cad_time && f.timestamp_valid && f.frame_valid && f.zero_ready &&
        f.left_valid && f.right_valid && f.left_in_range && f.right_in_range &&
        run_before.run_id==run.run_id && run_before.state_id==3 && run.state_id==3 && run.imu_ok;
    f.cad_prediction=cad_predictor.observe(f.run_id,
        cad_time ? static_cast<uint32_t>((f.frame_us-run.measurement_epoch_us)/1000) : 0,
        cad_time ? static_cast<uint32_t>((cad_now-run.measurement_epoch_us)/1000) : 0,
        cad_valid,f.left_deg,f.right_deg);
    if(run.mekf_attitude.valid) {
      const auto& q=run.mekf_attitude.quaternion;
      f.cad_body_fore_aft_raw=atan2f(2.f*(q.w*q.x+q.y*q.z),1.f-2.f*(q.x*q.x+q.y*q.y))*57.2957795f;
    }
    if(f.cad_prediction.ready()) {
      const auto& p=f.cad_prediction;
      const bool a=cad_support::switchAngle(p.left,p.right,0,f.cad_switch_positive);
      const bool b=cad_support::switchAngle(-p.left,-p.right,0,f.cad_switch_negative);
      f.cad_geometry_valid=a && b;
    }
    f.cad_compute_us=micros()-cad_start;
#endif
    float fps = 0;
    if (f.frame_valid) {
      if (last_valid_us && f.delivered_us > last_valid_us) fps = 1000000.0f / (f.delivered_us - last_valid_us);
      last_valid_us = f.delivered_us;
    }
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Publish);
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    f.ready_us=esp_timer_get_time(); // Recognition+projection done, before publish lock.
#endif
    portENTER_CRITICAL(&mux_);
    if (!f.frame_valid) ++status_.frame_failures;
    else {
      ++status_.captured_frames;
      if (!a.valid) ++status_.right_marker_failures;
      if (!b.valid) ++status_.left_marker_failures;
    }
    if (f.processing_us > status_.processing_max_us) status_.processing_max_us = f.processing_us;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    if(f.recognition_us>status_.recognition_max_us)status_.recognition_max_us=f.recognition_us;
    if(f.row_copy_us>status_.row_copy_max_us)status_.row_copy_max_us=f.row_copy_us;
#endif
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    status_.upper_attempts=upper_attempts_;
    if(f.upper_overlap_used)++status_.upper_hits;else ++status_.upper_misses;
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
    status_.lower_attempts=lower_attempts_;
    status_.lower_overlap_used=lower_overlap_used;
    status_.lower_prepare_us=lower_overlap_used?lower_prepare_us_:0;
    status_.lower_templates_before_delivery=lower_overlap_used?lower_templates_before_delivery:0;
    if(lower_overlap_used)++status_.lower_hits;else ++status_.lower_misses;
#endif
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
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
    if(preview_due) {
      preview.frame=f;
      publishPreview(f.frame_valid?preview_staging_:nullptr,preview,false,true);
      // An invalid idle frame clears availability immediately; the next valid
      // image is eligible without waiting for the previous preview interval.
      have_idle_preview=f.frame_valid;
      last_idle_preview_us=f.delivered_us;
    }
#endif
    RuntimeDiag::beat(RuntimeDiag::Lane::Camera, seq);
    RuntimeDiag::phase(RuntimeDiag::Lane::Camera, RuntimeDiag::Phase::Wait);
    const int64_t remaining =
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
        foot_fast_policy::kWorkerPeriodUs-
#else
        66667-
#endif
        (esp_timer_get_time() - begin);
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
  json += "\"worker_core\":"+String(kTaskCore)+",\"worker_priority\":"+String(kTaskPriority)+",";
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  json += "\"raw_sensor_geometry\":{\"width\":320,\"height\":236,\"bytes\":75520,\"logical_width\":320,\"logical_height\":240,\"raw_y_origin\":-2,\"row_mapping\":\"raw_y=logical_y-2;raw_x=logical_x\",\"preview_missing_rows\":\"zero_display_only\",\"dynamic_angle_mapping_validated\":false},";
  json += "\"idle_preview_period_us\":200000,\"preview_publication\":\"after_control_ready_and_status;staging_before_buffer_release_only_when_due\",\"preview_retention\":\"original_pixels_and_matching_metadata;invalid_idle_clears;first_valid_immediate\",";
#endif
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
#if defined(FOOT_SPEED_V22) && FOOT_SPEED_V22
  json+="\"target_fps\":30,\"capacity\":"+String(kCapacity);
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
#if defined(FOOT_CAPTURE_V23) && FOOT_CAPTURE_V23
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  json+=",\"sensor_speed_profile\":\"gc0308_native_skip_v25\",\"native_sensor_mode\":\"even_skip_p0_15_4a_p1_54_12\",\"sensor_profile_startup_readback_required\":true";
  json+=",\"native_sensor_orientation\":"+String(camera_native_skip_profile::kOrientation);
  json+=",\"sensor_period_estimate_ms\":null,\"sensor_xclk_nominal_hz\":22857142";
#else
  json+=",\"sensor_speed_profile\":\"gc0308_div7_hb128_sdk_aec_v23\",\"sensor_profile_startup_readback_required\":true";
  json+=",\"sensor_period_estimate_ms\":35.96,\"sensor_xclk_nominal_hz\":22857142";
#endif
  json+=",\"sensor_xclk_divider\":7,\"sensor_xclk_fraction_num\":0,\"sensor_xclk_fraction_den\":0";
  json+=",\"recognition_math\":\"integer_peak_and_bounded_int32_centroid_v23\"";
#else
  json+=",\"sensor_speed_profile\":\"gc0308_hb128_sdk_aec_v22_fix1\",\"sensor_profile_startup_readback_required\":true";
  json+=",\"sensor_period_estimate_ms\":41.1";
#endif
#if !defined(FOOT_NATIVE_SKIP_V25) || !FOOT_NATIVE_SKIP_V25
  json+=",\"sensor_hblank\":128";
#else
  json+=",\"sensor_hblank\":"+String(camera_native_skip_profile::kHorizontalBlank);
  json+=",\"queued_frame_discards\":"+String(rw_camera_native_stale_discard_count());
#endif
  json+=",\"sensor_vblank\":12,\"sensor_max_exposure_level_rows\":480,\"sensor_flicker_step_rows\":120";
  json+=",\"sensor_aec_modified\":false,\"sensor_readback_timeout_ms\":100";
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  const auto speed=cameraSnapshot().native_profile;
#else
  const auto speed=cameraSnapshot().speed_profile;
#endif
  json+=",\"sensor_profile_applied\":"+String(speed.applied?"true":"false");
  json+=",\"sensor_profile_stage\":\""+String(speed.stage)+"\",\"sensor_profile_settle_wait_ms\":"+String(speed.settle_wait_ms);
#else
  json+=",\"sensor_speed_profile\":\"gc0308_hb64_aec396_v22\",\"sensor_profile_startup_readback_required\":true";
  json+=",\"sensor_hblank\":64,\"sensor_vblank\":12,\"sensor_max_exposure_level_rows\":396,\"sensor_flicker_step_rows\":132";
  json+=",\"sensor_period_estimate_ms\":37.9";
#endif
  json+=",\"sensor_period_estimate_validated\":false,\"predictor_min_interval_ms\":25";
  json+=",\"contrast_reuse\":\"exact_integer_vertical_rolling_v22\"";
#else
  json+="\"target_fps\":25,\"capacity\":"+String(kCapacity);
#endif
  json+=",\"pipeline_revision\":\"sparse_cached_rows_double_buffer_v20\",\"camera_buffers\":2,\"grab_latest\":true,\"active_cam_take_zero_drain\":false";
  json+=",\"worker_min_period_us\":"+String(foot_fast_policy::kWorkerPeriodUs)+",\"control_poll_ms\":"+String(foot_fast_policy::kControlPollMs);
  json+=",\"row_cache_ready\":"+String(s.row_cache_ready?"true":"false")+",\"row_cache_internal_bytes\":"+String(foot_image_rows::kRequiredBytes);
  json+=",\"pixel_processing\":\"same_integer_contrast_and_full_search_row_major\",\"recognition_semantics_changed\":false";
  json+=",\"ready_us_semantics\":\"recognition_and_projection_done_before_publication_lock\",\"released_us_semantics\":\"driver_buffer_return_not_sensor_exposure_end\"";
  json+=",\"performance_validated_on_board\":false,\"all_frame_control_receipts_exported\":true";
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
  const auto overlap=rw_camera_overlap_stats();
  json+=",\"upper_overlap\":\"image_features_before_complete_frame_tracking_after_identity_validation\"";
  json+=",\"recognition_us_semantics\":\"post_delivery_only;upper_prepare_us_is_separate_overlapped_elapsed_time\"";
  json+=",\"upper_cache_bytes\":"+String(foot_image_rows::kUpperBytes);
  json+=",\"upper_attempts\":"+String(s.upper_attempts)+",\"upper_hits\":"+String(s.upper_hits)+",\"upper_misses\":"+String(s.upper_misses);
  json+=",\"upper_offered\":"+String(overlap.offered)+",\"upper_busy\":"+String(overlap.busy)+",\"upper_replaced\":"+String(overlap.replaced);
#endif
#if defined(FOOT_NATIVE_STREAM_V26) && FOOT_NATIVE_STREAM_V26
  json+=",\"lower_overlap\":\"incremental_image_features_before_complete_frame_tracking_after_identity_validation\"";
  json+=",\"lower_cache_bytes\":"+String(foot_image_rows::kLowerBytes)+",\"lower_scan_bytes\":"+String(sizeof(WhiteMarker1DTracker::PreparedScan));
  json+=",\"lower_attempts\":"+String(s.lower_attempts)+",\"lower_hits\":"+String(s.lower_hits)+",\"lower_misses\":"+String(s.lower_misses);
  json+=",\"lower_attempts_semantics\":\"callback_milestones_not_frames\",\"lower_timing_scope\":\"status_latest_only_not_frame_archive\"";
  json+=",\"lower_completion_policy\":\"matching_partial_scan_finishes_from_owned_full_rows_after_delivery;no_mailbox_poll_after_dequeue\"";
  json+=",\"lower_prepare_us_semantics\":\"callback_work_only;remaining_templates_in_recognition_us\",\"lower_templates_before_delivery_semantics\":\"status_latest_0_to_17_templates_reused_before_post_delivery_completion\"";
  json+=",\"full_row_cache_policy\":\"skip_only_when_both_owned_features_match_admitted_full_frame\"";
#endif
#if defined(CAMERA_FRAME_READY_V21) && CAMERA_FRAME_READY_V21
  const auto dma = rw_camera_frame_ready_stats();
#if defined(FOOT_NATIVE_SKIP_V25) && FOOT_NATIVE_SKIP_V25
  json+=",\"frame_publication\":\"complete_native_320x236_y8_dma_before_vsync_v25\"";
#else
  json+=",\"frame_publication\":\"complete_qvga_y8_dma_before_vsync_v21\"";
#endif
  json+=",\"dma_stats_scope\":\"boot_lifetime_independent_counters\"";
  json+=",\"dma_early_frames\":"+String(dma.early_frames)+",\"dma_vsync_frames\":"+String(dma.vsync_frames);
  json+=",\"dma_queue_drops\":"+String(dma.queue_drops)+",\"dma_last_full_age_us\":"+String(dma.last_full_age_us);
#endif
#else
  json += "\"target_fps\":15,\"capacity\":" + String(kCapacity);
#endif
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
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
    char timing[384];
    snprintf(timing,sizeof(timing),",\"ready_us\":%llu,\"released_us\":%llu,\"control_observed_us\":%llu,\"capture_wait_us\":%lu,\"row_copy_us\":%lu,\"recognition_us\":%lu,\"row_cache_used\":%s",
        static_cast<unsigned long long>(f.ready_us),static_cast<unsigned long long>(f.released_us),
        static_cast<unsigned long long>(f.control_observed_us),static_cast<unsigned long>(f.capture_wait_us),
        static_cast<unsigned long>(f.row_copy_us),static_cast<unsigned long>(f.recognition_us),f.row_cache_used?"true":"false");
    row+=timing;
#if defined(FOOT_ROW_OVERLAP_V24) && FOOT_ROW_OVERLAP_V24
    row+=",\"upper_prepare_us\":"+String(f.upper_prepare_us)+",\"upper_overlap_used\":"+String(f.upper_overlap_used?"true":"false");
#endif
#endif
    row += ",\"right_x\":" + number(f.right_x) + ",\"left_x\":" + number(f.left_x);
    row += ",\"right_deg\":" + number(f.right_deg) + ",\"left_deg\":" + number(f.left_deg);
#if defined(WALKING_RETURN_ENERGY) && WALKING_RETURN_ENERGY
    const auto right_range=foot_control_range::assess(f.right_x,true),left_range=foot_control_range::assess(f.left_x,false);
    row += ",\"right_control_range_ok\":" + String(f.right_valid && foot_control_range::allows(f.right_in_range,f.right_x,true)?"true":"false");
    row += ",\"left_control_range_ok\":" + String(f.left_valid && foot_control_range::allows(f.left_in_range,f.left_x,false)?"true":"false");
    row += ",\"right_extrapolation_deg\":" + number(right_range.extension_deg) + ",\"left_extrapolation_deg\":" + number(left_range.extension_deg);
#endif
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
#if defined(FOOT_SUPPORT_ASSIST) && FOOT_SUPPORT_ASSIST
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  constexpr size_t prediction_soft_end=foot_fast_policy::kOptionalPredictionEndBytes;
#else
  constexpr size_t prediction_soft_end=640U*1024U*8U/10U;
#endif
  json+=",\"foot_pose_prediction\":{\"revision\":\"causal_three_frame_history_v1\",\"timing\":\"driver_frame_timestamp;ready_after_camera_processing_and_prediction;exposure_unverified\",\"confidence_semantics\":\"history_match_score_not_probability\",\"uncertainty_semantics\":\"candidate_disagreement_margin_not_error_bound\",\"records\":";
  CompactJsonTable<PsramString> pose_table(json);uint32_t pose_stored=0;
  for(uint32_t i=0;i<s.count;++i){
    if(json.length()+16384U>=prediction_soft_end)break;
    const auto& f=frames_[i];const auto& p=f.pose_projection;
    const auto q=p.predict(p.received_ms,30);
    String row="{\"sequence\":"+String(f.sequence)+",\"ready_ms\":"+String(p.received_ms);
    row+=",\"reason\":"+String(static_cast<unsigned>(q.reason))+",\"confidence\":"+number(q.confidence);
    row+=",\"predicted_left_deg\":"+number(q.left)+",\"predicted_right_deg\":"+number(q.right);
    row+=",\"left_uncertainty_deg\":"+number(q.left_uncertainty)+",\"right_uncertainty_deg\":"+number(q.right_uncertainty);
    row+=",\"left_phase\":"+String(static_cast<unsigned>(q.phase_left))+",\"right_phase\":"+String(static_cast<unsigned>(q.phase_right));
    row+=",\"age_ms\":"+String(q.age_ms)+",\"horizon_ms\":"+String(q.horizon_ms);
    row+=",\"compute_us\":"+String(f.pose_compute_us)+"}";
    if(!pose_table.append(row)){json.fail();return;}++pose_stored;
  }
  if(!pose_table.finish()){json.fail();return;}
  json+=",\"stored\":"+String(pose_stored)+",\"total\":"+String(s.count)+",\"truncated\":"+String(pose_stored<s.count?"true":"false")+"}";
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
  json+=",\"foot_local_prediction\":{\"revision\":\"three_frame_local_returning_v12\",\"lead_ms\":0,\"uncertainty_semantics\":\"heuristic_local_margin_not_error_bound\",\"evidence_semantics\":\"fraction_of_feet_with_local_evidence_not_history_score_or_probability\",\"records\":";
  CompactJsonTable<PsramString> local_table(json);uint32_t local_stored=0;
  for(uint32_t i=0;i<s.count;++i){
    if(json.length()+16384U>=prediction_soft_end)break;
    const auto& f=frames_[i];const auto& p=f.local_projection;const auto q=p.predict(p.received_ms,0);
    String row="{\"sequence\":"+String(f.sequence)+",\"ready_ms\":"+String(p.received_ms);
    row+=",\"reason\":"+String(static_cast<unsigned>(q.reason))+",\"local_evidence\":"+number(q.confidence);
    row+=",\"predicted_left_deg\":"+number(q.left)+",\"predicted_right_deg\":"+number(q.right);
    row+=",\"left_uncertainty_deg\":"+number(q.left_uncertainty)+",\"right_uncertainty_deg\":"+number(q.right_uncertainty);
    row+=",\"left_phase\":"+String(static_cast<unsigned>(q.phase_left))+",\"right_phase\":"+String(static_cast<unsigned>(q.phase_right));
    row+=",\"age_ms\":"+String(q.age_ms)+",\"compute_us\":"+String(f.local_compute_us)+"}";
    if(!local_table.append(row)){json.fail();return;}++local_stored;
  }
  if(!local_table.finish()){json.fail();return;}
  json+=",\"stored\":"+String(local_stored)+",\"total\":"+String(s.count)+",\"truncated\":"+String(local_stored<s.count?"true":"false")+"}";
#endif
#endif
#if defined(CAD_SUPPORT_SHADOW) && CAD_SUPPORT_SHADOW
  // Separate compact table; binary sample format and existing foot fields stay compatible.
  json += ",\"cad_support_shadow\":{\"revision\":\"supplied_glb_pose_prediction_v1\",\"output_enabled\":false,\"mapping_calibrated\":false,";
  json += "\"cad_sha256\":\""+String(cad_support::sourceSha256)+"\",";
  json += "\"geometry\":\"upper_X_hinge_rigid_foot_mesh;nominal_body_pitch_zero;both_foot_sign_hypotheses\",\"camera_positive_direction\":\"forward_for_both_feet_user_confirmed_20261111\",\"lead_ms\":30,";
  json += "\"timing\":\"prediction_at_camera_processing_plus_lead;not_control_execution_time;frame_exposure_unverified\",\"angle_origin\":\"per_boot_camera_relative_not_CAD_absolute\",";
  json += "\"reason_names\":[\"ready\",\"warmup\",\"invalid\",\"timestamp\",\"gap\",\"jump\",\"stale\",\"horizon\"],\"records\":";
  CompactJsonTable<PsramString> cad_table(json);
  uint32_t cad_stored=0;
  for(uint32_t i=0;i<s.count;++i) {
    // Preserve room for the existing metadata tail even in a full log.
    if(json.length()+4096U>=
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
        foot_fast_policy::kOptionalPredictionEndBytes
#else
        640U*1024U*8U/10U
#endif
        )break;
    const auto& f=frames_[i];const auto& p=f.cad_prediction;
    String row="{\"sequence\":"+String(f.sequence)+",\"reason\":"+String(static_cast<unsigned>(p.reason));
    row+=",\"age_ms\":"+String(p.age_ms)+",\"horizon_ms\":"+String(p.horizon_ms);
    row+=",\"left_rate_dps\":"+number(p.left_rate)+",\"right_rate_dps\":"+number(p.right_rate);
    row+=",\"predicted_left_deg\":"+number(p.left)+",\"predicted_right_deg\":"+number(p.right);
    row+=",\"switch_sign_plus_deg\":"+number(f.cad_switch_positive)+",\"switch_sign_minus_deg\":"+number(f.cad_switch_negative);
    row+=",\"body_fore_aft_raw_deg\":"+number(f.cad_body_fore_aft_raw);
    row+=",\"geometry_valid\":"+String(f.cad_geometry_valid ? "true" : "false");
    row+=",\"compute_us\":"+String(f.cad_compute_us)+"}";
    if(!cad_table.append(row)){json.fail();return;}
    ++cad_stored;
  }
  if(!cad_table.finish()){json.fail();return;}
  json+=",\"stored\":"+String(cad_stored)+",\"total\":"+String(s.count);
  json+=",\"truncated\":"+String(cad_stored<s.count ? "true" : "false")+"}";
#endif
  // Sparse supplemental table: [foot_frames row index, observed-delivered us].
  // Keep the existing 80% metadata headroom even in a maximum-size log.
  // A truncated table is explicit; omitted rows always mean unavailable.
  // Select ranks uniformly over the *whole run*, including first and last,
  // rather than recording only the first part of walking. Reserve a conservative
  // 32 bytes per JSON entry (index, full uint64 delta, delimiters).
  String observed_rows;
  uint32_t total_observed = 0, valid_observed = 0, stored_observed = 0;
  constexpr uint32_t kMaxStoredObservations = 64;
#if defined(FOOT_LOW_LATENCY_V20) && FOOT_LOW_LATENCY_V20
  constexpr size_t kSoftEnd=foot_fast_policy::kOptionalPredictionEndBytes;
#else
  constexpr size_t kSoftEnd = 640U * 1024U * 8U / 10U;
#endif
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
