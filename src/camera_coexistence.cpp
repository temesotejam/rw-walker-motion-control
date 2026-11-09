#include "camera_coexistence.h"
#include "camera_task_priority_patch.h"
#include "camera_lifecycle.h"
#include "runtime_diagnostics.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "soc/gpio_sig_map.h"

extern "C" void cam_stop(void);
extern "C" void cam_start(void);
extern "C" camera_fb_t* cam_take(TickType_t timeout);
extern "C" void cam_give(camera_fb_t* fb);

namespace {

constexpr int PIN_CAM_POWER_N = 18;
constexpr int PIN_CAM_SDA = 12;
constexpr int PIN_CAM_SCL = 9;
constexpr int PIN_CAM_VSYNC = 10;
constexpr int PIN_CAM_HREF = 14;
constexpr int PIN_CAM_XCLK = 21;
constexpr int PIN_CAM_PCLK = 40;
constexpr int PIN_CAM_D0 = 3;
constexpr int PIN_CAM_D1 = 42;
constexpr int PIN_CAM_D2 = 46;
constexpr int PIN_CAM_D3 = 48;
constexpr int PIN_CAM_D4 = 4;
constexpr int PIN_CAM_D5 = 17;
constexpr int PIN_CAM_D6 = 11;
constexpr int PIN_CAM_D7 = 13;

constexpr uint32_t kCameraXclkHz = 20000000UL;
constexpr uint32_t kCaptureTimeoutMs = 500;
constexpr uint32_t kXclkWarmupMs = 20;

static_assert(CAM_CLK_IDX == 149, "Unexpected ESP32-S3 CAM clock matrix signal");
static_assert(SIG_GPIO_OUT_IDX == 256, "Unexpected ESP32-S3 GPIO output matrix signal");

}  // namespace

void OneShotCamera::setError(const char* error) {
  if (!error) error = "unknown";
  snprintf(last_error_, sizeof(last_error_), "%s", error);
}

void OneShotCamera::captureMemoryBefore() {
  snapshot_.internal_free_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  snapshot_.internal_largest_before = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  snapshot_.dma_free_before = heap_caps_get_free_size(MALLOC_CAP_DMA);
  snapshot_.psram_free_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

void OneShotCamera::captureMemoryAfter() {
  snapshot_.internal_free_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  snapshot_.internal_largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  snapshot_.dma_free_after = heap_caps_get_free_size(MALLOC_CAP_DMA);
  snapshot_.psram_free_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

void OneShotCamera::setXclkEnabled(bool enabled) {
  const gpio_num_t pin = static_cast<gpio_num_t>(PIN_CAM_XCLK);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);

  if (enabled) {
    gpio_matrix_out(PIN_CAM_XCLK, CAM_CLK_IDX, false, false);
  } else {
    gpio_matrix_out(PIN_CAM_XCLK, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_level(pin, 0);
  }

  portENTER_CRITICAL(&mux_);
  snapshot_.xclk_active = enabled;
  portEXIT_CRITICAL(&mux_);
}

void OneShotCamera::flushQueuedFrames() {
  for (;;) {
    camera_fb_t* stale = cam_take(0);
    if (!stale) break;
    cam_give(stale);
  }
}

bool OneShotCamera::begin() {
  const auto result = camera_lifecycle::run([](void* self) {
    return static_cast<OneShotCamera*>(self)->beginOnOwnerCore();
  }, this);
  if (!result.invoked) setError(result.error);
  return result.ok;
}

bool OneShotCamera::beginOnOwnerCore() {
  if (xPortGetCoreID() != 0) { setError("camera_init_wrong_core"); return false; }
  RuntimeDiag::cameraLifecycleCore(true, xPortGetCoreID());
  snapshot_ = CameraOneShotSnapshot{};
  snapshot_.xclk_hz = kCameraXclkHz;
  snapshot_.xclk_warmup_ms = kXclkWarmupMs;
  captureMemoryBefore();

  capture_mutex_ = xSemaphoreCreateMutex();
  if (!capture_mutex_) {
    setError("camera_capture_mutex_create_failed");
    captureMemoryAfter();
    return false;
  }

  if (!initCameraOnTemporaryI2c0()) {
    captureMemoryAfter();
    return false;
  }

  snapshot_.camera_driver_active = true;
  snapshot_.sensor_powered = true;
  snapshot_.camera_deinitialized = false;
  snapshot_.one_shot_mode = true;

  const CameraTaskPriorityPatchSnapshot task_patch = cameraTaskPriorityPatchSnapshot();
  snapshot_.cam_task_priority_patch_observed = task_patch.observed;
  snapshot_.cam_task_original_priority = task_patch.original_priority;
  snapshot_.cam_task_effective_priority = task_patch.effective_priority;
  snapshot_.cam_task_core = task_patch.core;

  // Complete exactly one boot frame so cam_task is known to be out of an
  // in-progress frame, then enter the idle state. There is NO background
  // camera task in this serial-debug build.
  camera_fb_t* boot_frame = cam_take(pdMS_TO_TICKS(kCaptureTimeoutMs));
  cameraTaskPriorityPatchSampleStack();
  if (!boot_frame) {
    cam_stop();
    setXclkEnabled(false);
    cameraTaskPriorityPatchForgetTask();
    esp_camera_deinit();
    digitalWrite(PIN_CAM_POWER_N, HIGH);
    snapshot_.camera_driver_active = false;
    snapshot_.sensor_powered = false;
    snapshot_.camera_deinitialized = true;
    setError("camera_boot_frame_failed");
    captureMemoryAfter();
    return false;
  }

  snapshot_.first_frame_seen = true;
  snapshot_.frame_count = 1;
  snapshot_.last_frame_bytes = boot_frame->len;
  snapshot_.last_width = 320;
  snapshot_.last_height = 240;

  cam_stop();
  setXclkEnabled(false);
  cam_give(boot_frame);
  flushQueuedFrames();

  snapshot_.camera_ok = true;
  setError("ok_serial_debug_idle");
  captureMemoryAfter();
  return true;
}

camera_fb_t* OneShotCamera::acquire(uint32_t timeout_ms) {
  const CameraOneShotSnapshot before = snapshot();
  if (!before.camera_ok || !before.camera_driver_active ||
      !before.sensor_powered || before.camera_deinitialized || !capture_mutex_) {
    setError("camera_capture_not_available");
    return nullptr;
  }

  if (xSemaphoreTake(capture_mutex_, pdMS_TO_TICKS(timeout_ms + 100)) != pdTRUE) {
    setError("camera_capture_mutex_timeout");
    return nullptr;
  }

  const uint32_t started_us = micros();

  setXclkEnabled(true);
  delay(kXclkWarmupMs);

  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = true;
  portEXIT_CRITICAL(&mux_);

  cam_start();
  camera_fb_t* fb = cam_take(pdMS_TO_TICKS(timeout_ms ? timeout_ms : kCaptureTimeoutMs));
  cameraTaskPriorityPatchSampleStack();

  // The key invariant: receiver and XCLK are OFF before control returns.
  cam_stop();
  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = false;
  portEXIT_CRITICAL(&mux_);
  setXclkEnabled(false);

  const uint32_t elapsed_us = static_cast<uint32_t>(micros() - started_us);

  if (!fb) {
    portENTER_CRITICAL(&mux_);
    ++snapshot_.frame_failures;
    snapshot_.last_capture_us = elapsed_us;
    if (elapsed_us > snapshot_.max_capture_us) snapshot_.max_capture_us = elapsed_us;
    portEXIT_CRITICAL(&mux_);
    setError("camera_oneshot_frame_timeout");
    xSemaphoreGive(capture_mutex_);
    return nullptr;
  }

  fb->width = 320;
  fb->height = 240;
  fb->format = PIXFORMAT_GRAYSCALE;

  portENTER_CRITICAL(&mux_);
  snapshot_.first_frame_seen = true;
  ++snapshot_.frame_count;
  snapshot_.last_frame_bytes = fb->len;
  snapshot_.last_width = static_cast<uint16_t>(fb->width);
  snapshot_.last_height = static_cast<uint16_t>(fb->height);
  snapshot_.last_capture_us = elapsed_us;
  if (elapsed_us > snapshot_.max_capture_us) snapshot_.max_capture_us = elapsed_us;
  portEXIT_CRITICAL(&mux_);

  setError("ok_serial_debug_capture");
  return fb;
}

void OneShotCamera::release(camera_fb_t* fb) {
  if (!capture_mutex_) return;
  if (fb) cam_give(fb);
  xSemaphoreGive(capture_mutex_);
}

bool OneShotCamera::debugCaptureOnce() {
  camera_fb_t* fb = acquire(kCaptureTimeoutMs);
  if (!fb) return false;
  release(fb);
  return true;
}

void OneShotCamera::debugForceIdle() {
  if (!capture_mutex_) return;
  if (xSemaphoreTake(capture_mutex_, pdMS_TO_TICKS(250)) != pdTRUE) {
    setError("camera_debug_idle_mutex_timeout");
    return;
  }
  cam_stop();
  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = false;
  portEXIT_CRITICAL(&mux_);
  setXclkEnabled(false);
  flushQueuedFrames();
  setError("debug_forced_idle");
  xSemaphoreGive(capture_mutex_);
}

bool OneShotCamera::debugPowerSensorOff() {
  if (!capture_mutex_) return false;
  if (xSemaphoreTake(capture_mutex_, pdMS_TO_TICKS(250)) != pdTRUE) {
    setError("camera_debug_poweroff_mutex_timeout");
    return false;
  }

  cam_stop();
  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = false;
  portEXIT_CRITICAL(&mux_);
  setXclkEnabled(false);
  digitalWrite(PIN_CAM_POWER_N, HIGH);

  portENTER_CRITICAL(&mux_);
  snapshot_.sensor_powered = false;
  snapshot_.one_shot_mode = false;
  portEXIT_CRITICAL(&mux_);

  setError("debug_sensor_power_off");
  xSemaphoreGive(capture_mutex_);
  return true;
}

bool OneShotCamera::debugDeinit() {
  const auto result = camera_lifecycle::run([](void* self) {
    return static_cast<OneShotCamera*>(self)->deinitOnOwnerCore();
  }, this);
  if (!result.invoked) setError(result.error);
  return result.ok;
}

bool OneShotCamera::deinitOnOwnerCore() {
  if (xPortGetCoreID() != 0) { setError("camera_deinit_wrong_core"); return false; }
  RuntimeDiag::cameraLifecycleCore(false, xPortGetCoreID());
  if (!capture_mutex_) return false;
  if (xSemaphoreTake(capture_mutex_, pdMS_TO_TICKS(250)) != pdTRUE) {
    setError("camera_debug_deinit_mutex_timeout");
    return false;
  }

  cam_stop();
  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = false;
  portEXIT_CRITICAL(&mux_);
  setXclkEnabled(false);

  cameraTaskPriorityPatchForgetTask();
  const esp_err_t err = esp_camera_deinit();
  digitalWrite(PIN_CAM_POWER_N, HIGH);

  portENTER_CRITICAL(&mux_);
  snapshot_.camera_driver_active = false;
  snapshot_.sensor_powered = false;
  snapshot_.one_shot_mode = false;
  snapshot_.camera_deinitialized = (err == ESP_OK);
  snapshot_.camera_ok = false;
  portEXIT_CRITICAL(&mux_);

  setError(err == ESP_OK ? "debug_camera_deinitialized" : "debug_camera_deinit_failed");
  xSemaphoreGive(capture_mutex_);
  return err == ESP_OK;
}

bool OneShotCamera::initCameraOnTemporaryI2c0() {
  pinMode(PIN_CAM_POWER_N, OUTPUT);
  digitalWrite(PIN_CAM_POWER_N, LOW);
  delay(500);

  (void)i2c_driver_delete(I2C_NUM_0);

  i2c_config_t sccb = {};
  sccb.mode = I2C_MODE_MASTER;
  sccb.sda_io_num = static_cast<gpio_num_t>(PIN_CAM_SDA);
  sccb.sda_pullup_en = GPIO_PULLUP_ENABLE;
  sccb.scl_io_num = static_cast<gpio_num_t>(PIN_CAM_SCL);
  sccb.scl_pullup_en = GPIO_PULLUP_ENABLE;
  sccb.master.clk_speed = 100000;

  esp_err_t err = i2c_param_config(I2C_NUM_0, &sccb);
  if (err != ESP_OK) {
    setError("camera_sccb_i2c0_param_config_failed");
    return false;
  }

  err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK) {
    setError("camera_sccb_i2c0_driver_install_failed");
    return false;
  }

  camera_config_t c = {};
  c.pin_pwdn = -1;
  c.pin_reset = -1;
  c.pin_xclk = PIN_CAM_XCLK;
  c.pin_sccb_sda = -1;
  c.pin_sccb_scl = -1;
  c.pin_d7 = PIN_CAM_D7;
  c.pin_d6 = PIN_CAM_D6;
  c.pin_d5 = PIN_CAM_D5;
  c.pin_d4 = PIN_CAM_D4;
  c.pin_d3 = PIN_CAM_D3;
  c.pin_d2 = PIN_CAM_D2;
  c.pin_d1 = PIN_CAM_D1;
  c.pin_d0 = PIN_CAM_D0;
  c.pin_vsync = PIN_CAM_VSYNC;
  c.pin_href = PIN_CAM_HREF;
  c.pin_pclk = PIN_CAM_PCLK;
  c.xclk_freq_hz = kCameraXclkHz;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  c.sccb_i2c_port = I2C_NUM_0;

  err = esp_camera_init(&c);
  if (err != ESP_OK) {
    // esp_camera_init can have already deleted its partially created task.
    cameraTaskPriorityPatchForgetTask();
    (void)i2c_driver_delete(I2C_NUM_0);
    setError("esp_camera_init_failed");
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) {
    cameraTaskPriorityPatchForgetTask();
    esp_camera_deinit();
    (void)i2c_driver_delete(I2C_NUM_0);
    setError("camera_sensor_missing");
    return false;
  }

  sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  sensor->set_vflip(sensor, 1);
  sensor->set_hmirror(sensor, 0);

  err = i2c_driver_delete(I2C_NUM_0);
  if (err != ESP_OK) {
    cameraTaskPriorityPatchForgetTask();
    esp_camera_deinit();
    setError("camera_sccb_i2c0_release_failed");
    return false;
  }

  return true;
}

CameraOneShotSnapshot OneShotCamera::snapshot() const {
  portENTER_CRITICAL(&mux_);
  const CameraOneShotSnapshot copy = snapshot_;
  portEXIT_CRITICAL(&mux_);
  return copy;
}

bool OneShotCamera::startContinuous() {
  const auto s = snapshot();
  if (!s.camera_ok || !s.camera_driver_active || !s.sensor_powered) return false;
  if (s.receiver_active) return true;
  setXclkEnabled(true); delay(kXclkWarmupMs);
  cam_start();
  portENTER_CRITICAL(&mux_);
  snapshot_.one_shot_mode = false;
  snapshot_.receiver_active = true;
  snapshot_.consumer_core = xPortGetCoreID();
  snapshot_.consumer_priority = uxTaskPriorityGet(nullptr);
  portEXIT_CRITICAL(&mux_);
  return true;
}
camera_fb_t* OneShotCamera::acquireContinuous(uint32_t timeout_ms) {
  if (!snapshot().receiver_active) return nullptr;
  const uint32_t start = micros();
  camera_fb_t* fb = cam_take(pdMS_TO_TICKS(timeout_ms));
  cameraTaskPriorityPatchSampleStack();
  // The low-level cam_take API does not populate the wrapper's format fields.
  // Length is checked by FootObserver before any image access.
  if (fb) { fb->width = 320; fb->height = 240; fb->format = PIXFORMAT_GRAYSCALE; }
  portENTER_CRITICAL(&mux_);
  snapshot_.last_capture_us = static_cast<uint32_t>(micros() - start);
  if (snapshot_.last_capture_us > snapshot_.max_capture_us) snapshot_.max_capture_us = snapshot_.last_capture_us;
  if (fb) { ++snapshot_.frame_count; snapshot_.last_frame_bytes = fb->len; }
  else ++snapshot_.frame_failures;
  portEXIT_CRITICAL(&mux_);
  return fb;
}
void OneShotCamera::releaseContinuous(camera_fb_t* fb) { if (fb) cam_give(fb); }
void OneShotCamera::stopContinuous() {
  if (!snapshot().receiver_active) return;
  cam_stop(); setXclkEnabled(false); flushQueuedFrames();
  portENTER_CRITICAL(&mux_);
  snapshot_.receiver_active = false;
  portEXIT_CRITICAL(&mux_);
}
