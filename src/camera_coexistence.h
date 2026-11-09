#pragma once

#include <Arduino.h>
#include "esp_camera.h"
#include "freertos/semphr.h"

struct CameraOneShotSnapshot {
  bool camera_ok = false;
  bool first_frame_seen = false;
  bool camera_driver_active = false;
  bool sensor_powered = false;
  bool one_shot_mode = false;
  bool receiver_active = false;
  bool xclk_active = false;
  bool camera_deinitialized = false;

  uint32_t frame_count = 0;
  uint32_t frame_failures = 0;
  uint32_t last_frame_bytes = 0;
  uint16_t last_width = 0;
  uint16_t last_height = 0;
  uint32_t last_capture_us = 0;
  uint32_t max_capture_us = 0;

  uint32_t xclk_hz = 16000000UL;
  uint16_t xclk_warmup_ms = 20;
  uint16_t minimum_idle_ms = 0;  // Serial debug build: no automatic capture schedule.
  int8_t consumer_core = -1;     // No background camera consumer task.
  uint8_t consumer_priority = 0;

  bool cam_task_priority_patch_observed = false;
  uint8_t cam_task_original_priority = 0;
  uint8_t cam_task_effective_priority = 0;
  int8_t cam_task_core = -1;

  uint32_t internal_free_before = 0;
  uint32_t internal_free_after = 0;
  uint32_t internal_largest_before = 0;
  uint32_t internal_largest_after = 0;
  uint32_t dma_free_before = 0;
  uint32_t dma_free_after = 0;
  uint32_t psram_free_before = 0;
  uint32_t psram_free_after = 0;
};

class OneShotCamera {
 public:
  bool begin();

  // Receiver and XCLK are normally OFF. acquire() enables them only long
  // enough to obtain one frame and disables both before it returns.
  camera_fb_t* acquire(uint32_t timeout_ms = 500);
  void release(camera_fb_t* fb);
  bool startContinuous();
  camera_fb_t* acquireContinuous(uint32_t timeout_ms = 200);
  void releaseContinuous(camera_fb_t* fb);
  void stopContinuous();

  // USB-serial diagnostic actions. No automatic capture is performed.
  bool debugCaptureOnce();
  void debugForceIdle();
  bool debugPowerSensorOff();
  bool debugDeinit();

  CameraOneShotSnapshot snapshot() const;
  const char* lastError() const { return last_error_; }

 private:
  bool beginOnOwnerCore();
  bool deinitOnOwnerCore();
  bool initCameraOnTemporaryI2c0();
  void setXclkEnabled(bool enabled);
  void flushQueuedFrames();
  void captureMemoryBefore();
  void captureMemoryAfter();
  void setError(const char* error);

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  SemaphoreHandle_t capture_mutex_ = nullptr;
  CameraOneShotSnapshot snapshot_;
  char last_error_[64] = "not_initialized";
};
