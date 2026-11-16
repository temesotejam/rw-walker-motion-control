#if defined(CAMERA_GEOMETRY_PROBE_V25) && CAMERA_GEOMETRY_PROBE_V25
// Isolated USB camera geometry bench. No motion, IMU, network or control code.
#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_struct.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <cstring>
#include <cstdlib>
#include "camera_capture_clock.h"
#include "camera_frame_ready.h"
#include "camera_speed_profile.h"

extern "C" void cam_stop(void);
extern "C" void cam_start(void);
extern "C" camera_fb_t* cam_take(TickType_t timeout);
extern "C" void cam_give(camera_fb_t* fb);
extern "C" BaseType_t __real_xTaskCreatePinnedToCore(TaskFunction_t, const char* const,
    const uint32_t, void* const, UBaseType_t, TaskHandle_t* const, const BaseType_t);
extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(TaskFunction_t function,
    const char* const name, const uint32_t stack, void* const parameter,
    UBaseType_t priority, TaskHandle_t* const task, const BaseType_t core) {
  const bool camera = name && std::strcmp(name, "cam_task") == 0;
  return __real_xTaskCreatePinnedToCore(function, name,
      camera && stack < 8192 ? 8192 : stack, parameter,
      camera ? 3 : priority, task, camera ? 0 : core);
}

namespace {
constexpr size_t kCapacity = 640U * 480U;
constexpr int kLegacyOrientation = 0x10;
constexpr int kPower = 18, kSda = 12, kScl = 9, kXclk = 21;
sensor_t* sensor = nullptr;
bool camera_ready = false, dump_requested = false;
int profile = -1;
uint64_t admit_after_us = 0, last_frame_us = 0;
uint32_t last_report_ms = 0, frames = 0, timeouts = 0, discarded = 0;
uint32_t dump_requested_ms = 0;
char command[64] = {};
size_t command_size = 0;
bool command_overflow = false;
const char* last_error = "initializing";
int baseline_aec = -1, baseline_exposure_h = -1, baseline_exposure_l = -1;
int manual_exposure_rows = 0;
int horizontal_blanking = -1, native_height = -1;
int boot_orientation_p0_14 = -1;
bool orientation_verified = false;
portMUX_TYPE physical_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t physical_lines = 0, physical_first_href_us = 0, physical_last_href_us = 0;
volatile uint32_t physical_vsync_us = 0, physical_vsync_count = 0;
volatile uint32_t physical_completed_lines = 0, physical_span_us = 0, physical_period_us = 0;
struct Physical { uint32_t lines, span_us, period_us, vsync_us, count; };
void IRAM_ATTR onHref() {
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&physical_mux);
  if (!physical_lines) physical_first_href_us = now;
  ++physical_lines;
  physical_last_href_us = now;
  portEXIT_CRITICAL_ISR(&physical_mux);
}
void IRAM_ATTR onVsync() {
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&physical_mux);
  physical_completed_lines = physical_lines;
  physical_span_us = physical_lines > 1 ? physical_last_href_us - physical_first_href_us : 0;
  physical_period_us = physical_vsync_us ? now - physical_vsync_us : 0;
  physical_vsync_us = now;
  ++physical_vsync_count;
  physical_lines = 0;
  portEXIT_CRITICAL_ISR(&physical_mux);
}
Physical physicalSnapshot() {
  portENTER_CRITICAL(&physical_mux);
  const Physical out{physical_completed_lines, physical_span_us, physical_period_us,
      physical_vsync_us, physical_vsync_count};
  portEXIT_CRITICAL(&physical_mux);
  return out;
}
void resumeFresh() {
  admit_after_us = esp_timer_get_time();
  last_frame_us = 0;
  cam_start();
}
void status() {
  Serial.printf("STATUS,identity=camera-skip-bench-v25,motor=absent,running=0,motor_mA=0,ready=%u,profile=%d,hb=%d,native_height=%d,exposure=%s,exposure_rows=%d,boot_p0_14=%d,orientation_verified=%u,error=%s\n",
      camera_ready, profile, horizontal_blanking, native_height,
      manual_exposure_rows ? "manual" : "auto", manual_exposure_rows,
      boot_orientation_p0_14, orientation_verified, last_error);
}

void xclkEnabled(bool enabled) {
  gpio_set_direction(static_cast<gpio_num_t>(kXclk), GPIO_MODE_OUTPUT);
  gpio_matrix_out(kXclk, enabled ? CAM_CLK_IDX : SIG_GPIO_OUT_IDX, false, false);
  if (!enabled) gpio_set_level(static_cast<gpio_num_t>(kXclk), 0);
}
bool writeReg(int reg, int value) {
  return sensor && sensor->set_reg && sensor->set_reg(sensor, reg, 0xff, value) == 0;
}
int readReg(int reg) {
  return sensor && sensor->get_reg ? sensor->get_reg(sensor, reg, 0xff) : -1;
}
bool verifyReg(int reg, int expected) {
  for (unsigned i = 0; i <= 20; ++i) {
    const int actual = readReg(reg);
    if (actual == expected) return true;
    if (actual < 0 || i == 20) return false;
    delay(5);
  }
  return false;
}
void registerReport() {
  if (!writeReg(0xfe, 0) || !verifyReg(0xfe, 0)) {
    Serial.println("REGS,error=page0_write_or_readback");
    return;
  }
  const int orientation = readReg(0x14), even = readReg(0x15), crop = readReg(0x46);
  const int exposure_h = readReg(0x03), exposure_l = readReg(0x04);
  const int aec = readReg(0xd2), flicker_h = readReg(0xe2), flicker_l = readReg(0xe3);
  const int hb = readReg(0x01), vb = readReg(0x02), blank_h = readReg(0x0f);
  const int height_h = readReg(0x09), height_l = readReg(0x0a);
  int subsample = -1;
  if (writeReg(0xfe, 1)) subsample = readReg(0x54);
  const bool page0 = writeReg(0xfe, 0);
  Serial.printf("REGS,profile=%d,p0_14=%d,p0_15=%d,p1_54=%d,p0_46=%d,exposure_03=%d,exposure_04=%d,aec_d2=%d,flicker_e2=%d,flicker_e3=%d,hb01=%d,vb02=%d,blank0f=%d,height09=%d,height0a=%d,page0_restored=%u\n",
      profile, orientation, even, subsample, crop, exposure_h, exposure_l, aec,
      flicker_h, flicker_l, hb, vb, blank_h, height_h, height_l, page0);
}
bool setHorizontalBlanking(int blanking) {
  if (!camera_ready || (blanking != 128 && blanking != 96 && blanking != 80 && blanking != 64))
    return false;
  cam_stop();
  int previous = -1;
  if (writeReg(0xfe, 0)) previous = readReg(0x0f);
  // GC0308 0x0f[3:0] extends HB, [7:4] extends VB. These bounded choices
  // need no HB high bits; preserve all existing VB high bits exactly.
  const int combined_high = previous & 0xf0;
  const bool ok = previous >= 0 && writeReg(0x0f, combined_high) &&
      verifyReg(0x0f, combined_high) && writeReg(0x01, blanking) && verifyReg(0x01, blanking);
  if (!ok) {
    camera_ready = false; last_error = "hb_readback_failed";
    Serial.println("HB,error=readback,receiver=stopped");
    return false;
  }
  horizontal_blanking = blanking;
  registerReport();
  delay(50);
  resumeFresh();
  Serial.printf("HB,value=%d,blank0f=%d,native_height=%d\n", horizontal_blanking, combined_high, native_height);
  return true;
}
bool setNativeHeight(int height) {
  if (!camera_ready || (height != 488 && height != 496)) return false;
  cam_stop();
  const bool ok = writeReg(0xfe, 0) && writeReg(0x09, height >> 8) &&
      verifyReg(0x09, height >> 8) && writeReg(0x0a, height & 255) && verifyReg(0x0a, height & 255);
  if (!ok) {
    camera_ready = false; last_error = "height_readback_failed";
    Serial.println("HEIGHT,error=readback,receiver=stopped");
    return false;
  }
  // This is sensor-native register height, not a claim of output row count.
  native_height = height;
  registerReport();
  delay(50);
  resumeFresh();
  Serial.printf("HEIGHT,native=%d,hb=%d,output_rows=unverified\n", native_height, horizontal_blanking);
  return true;
}
bool selectProfile(unsigned selected) {
  if (!camera_ready || selected > 4) return false;
  static constexpr uint8_t even[] = {0x0a, 0x4a, 0x4a, 0x4a, 0x4a};
  static constexpr uint8_t sub[] = {0x22, 0x22, 0x11, 0x12, 0x21};
  cam_stop();
  const bool ok = writeReg(0xfe, 0) && writeReg(0x15, even[selected]) &&
      verifyReg(0x15, even[selected]) && writeReg(0xfe, 1) &&
      writeReg(0x54, sub[selected]) && verifyReg(0x54, sub[selected]) &&
      writeReg(0xfe, 0);
  if (!ok) {
    (void)writeReg(0xfe, 0);
    camera_ready = false;
    last_error = "profile_register_readback_failed";
    Serial.printf("PROFILE,error=readback,requested=%u,receiver=stopped\n", selected);
    return false;
  }
  profile = static_cast<int>(selected);
  last_frame_us = 0;
  // SCCB remains owned by this bench. AEC/exposure registers are never changed.
  registerReport();
  delay(50);
  // Old queued frames are discarded by their acquisition timestamp. Never
  // call cam_take(0), which can reset this SDK's active DMA on an empty queue.
  resumeFresh();
  last_error = "ok";
  Serial.printf("PROFILE,selected=%d,running=false,motor_mA=0,capacity=%u\n",
      profile, static_cast<unsigned>(kCapacity));
  return true;
}
bool initializeCamera() {
  pinMode(kPower, OUTPUT);
  digitalWrite(kPower, LOW);
  delay(500);
  (void)i2c_driver_delete(I2C_NUM_0);
  i2c_config_t sccb = {};
  sccb.mode = I2C_MODE_MASTER;
  sccb.sda_io_num = static_cast<gpio_num_t>(kSda);
  sccb.scl_io_num = static_cast<gpio_num_t>(kScl);
  sccb.sda_pullup_en = GPIO_PULLUP_ENABLE;
  sccb.scl_pullup_en = GPIO_PULLUP_ENABLE;
  sccb.master.clk_speed = 100000;
  if (i2c_param_config(I2C_NUM_0, &sccb) != ESP_OK ||
      i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
    last_error = "sccb_init_failed";
    return false;
  }
  camera_config_t c = {};
  c.pin_pwdn = -1; c.pin_reset = -1; c.pin_xclk = kXclk;
  c.pin_sccb_sda = -1; c.pin_sccb_scl = -1;
  c.pin_d0 = 3; c.pin_d1 = 42; c.pin_d2 = 46; c.pin_d3 = 48;
  c.pin_d4 = 4; c.pin_d5 = 17; c.pin_d6 = 11; c.pin_d7 = 13;
  c.pin_vsync = 10; c.pin_href = 14; c.pin_pclk = 40;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size = FRAMESIZE_VGA; // HAL allocation capacity, not asserted output geometry.
  c.jpeg_quality = 12; c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
  c.sccb_i2c_port = I2C_NUM_0;
  if (esp_camera_init(&c) != ESP_OK) {
    last_error = "camera_init_failed";
    return false;
  }
  cam_stop();
  sensor = esp_camera_sensor_get();
  if (!sensor || sensor->id.PID != GC0308_PID) {
    last_error = "wrong_or_missing_sensor";
    return false;
  }
  xclkEnabled(false);
  const bool clock_ok = camera_capture_clock::apply(LCD_CAM.cam_ctrl);
  xclkEnabled(true);
  delay(20);
  if (!clock_ok) { last_error = "clock_readback_failed"; return false; }
  sensor->xclk_freq_hz = camera_capture_clock::kTargetHz;
  if (sensor->set_framesize(sensor, FRAMESIZE_QVGA) != 0 ||
      sensor->set_vflip(sensor, 1) != 0 || sensor->set_hmirror(sensor, 0) != 0) {
    last_error = "sensor_qvga_setup_failed";
    return false;
  }
  // Historical bench logs did not record P0:14. Separate SDK flip/mirror
  // setters could leave an unverified value after stale read/modify/write.
  // The legacy image comparison identifies 0x10, while 0x12 reverses lanes.
  // Establish the complete byte explicitly; each readback has a 100 ms bound.
  const bool orientation_page = writeReg(0xfe, 0) && verifyReg(0xfe, 0);
  const bool orientation_ok = orientation_page && writeReg(0x14, kLegacyOrientation) &&
      verifyReg(0x14, kLegacyOrientation);
  boot_orientation_p0_14 = orientation_page ? readReg(0x14) : -1;
  orientation_verified = orientation_ok && boot_orientation_p0_14 == kLegacyOrientation;
  Serial.printf("ORIENTATION,expected_p0_14=%d,actual_p0_14=%d,verified=%u,readback_limit_ms=100\n",
      kLegacyOrientation, boot_orientation_p0_14, orientation_verified);
  if (!orientation_verified) {
    last_error = "orientation_readback_failed";
    return false; // Reception remains stopped; no frame can be admitted.
  }
  sensor->status.vflip = 0;
  sensor->status.hmirror = 0;
  camera_speed_profile::Report timing;
  const bool timing_ok = camera_speed_profile::apply(readReg, writeReg,
      [](unsigned ms) { delay(ms); }, &timing);
  Serial.printf("TIMING,stage=%s,applied=%u,reg=%d,expected=%d,actual=%d,settle_ms=%u,xclk_hz=%u\n",
      timing.stage, timing.applied, timing.reg, timing.expected, timing.actual,
      timing.settle_wait_ms, camera_capture_clock::kTargetHz);
  if (!timing_ok) { last_error = "timing_readback_failed"; return false; }
  if (!writeReg(0xfe, 0)) { last_error = "exposure_baseline_page_failed"; return false; }
  baseline_aec = readReg(0xd2);
  baseline_exposure_h = readReg(0x03); baseline_exposure_l = readReg(0x04);
  const int hb_low = readReg(0x01), blank_high = readReg(0x0f);
  const int height_high = readReg(0x09), height_low = readReg(0x0a);
  if (baseline_aec < 0 || baseline_exposure_h < 0 || baseline_exposure_l < 0) {
    last_error = "exposure_baseline_read_failed";
    return false;
  }
  if (hb_low < 0 || blank_high < 0 || height_high < 0 || height_low < 0) {
    last_error = "geometry_baseline_read_failed";
    return false;
  }
  horizontal_blanking = ((blank_high & 0x0f) << 8) | hb_low;
  native_height = (height_high << 8) | height_low;
  // Physical GPIO edges are independent of LCD_CAM's synthetic matrix VSYNC.
  // These latest-completed physical-frame statistics are not asserted to be
  // the same frame as any separately queued DMA buffer.
  attachInterrupt(digitalPinToInterrupt(14), onHref, RISING);
  attachInterrupt(digitalPinToInterrupt(10), onVsync, FALLING);
  camera_ready = true;
  return selectProfile(0);
}

bool setExposure(int rows) {
  if (!camera_ready || rows < 0 || rows > 4095 || baseline_aec < 0) return false;
  cam_stop();
  const int h = rows ? rows >> 8 : baseline_exposure_h;
  const int l = rows ? rows & 255 : baseline_exposure_l;
  const int final_aec = rows ? baseline_aec & ~0x80 : baseline_aec;
  // Freeze first, restore the startup exposure exactly before AUTO resumes.
  // Sensor gain registers are deliberately left unchanged.
  const bool ok = writeReg(0xfe, 0) && writeReg(0xd2, baseline_aec & ~0x80) &&
      verifyReg(0xd2, baseline_aec & ~0x80) && writeReg(0x03, h) && writeReg(0x04, l) &&
      verifyReg(0x03, h) && verifyReg(0x04, l) && writeReg(0xd2, final_aec) && verifyReg(0xd2, final_aec);
  if (!ok) {
    camera_ready = false; last_error = "exposure_readback_failed";
    Serial.println("EXPOSURE,error=readback,receiver=stopped");
    return false;
  }
  manual_exposure_rows = rows;
  registerReport();
  delay(50);
  resumeFresh();
  Serial.printf("EXPOSURE,mode=%s,rows=%d,saved_d2=%d,saved03=%d,saved04=%d\n",
      rows ? "manual" : "auto", rows, baseline_aec, baseline_exposure_h, baseline_exposure_l);
  return true;
}

struct Pixels { uint32_t crc = 0; uint8_t min = 255, max = 0; double mean = 0; };
Pixels pixels(const camera_fb_t* fb) {
  Pixels out;
  uint32_t crc = 0xffffffffU, sum = 0;
  for (size_t i = 0; i < fb->len; ++i) {
    const uint8_t v = fb->buf[i];
    if (v < out.min) out.min = v;
    if (v > out.max) out.max = v;
    sum += v; crc ^= v;
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  out.crc = crc ^ 0xffffffffU;
  out.mean = fb->len ? static_cast<double>(sum) / fb->len : 0;
  return out;
}
bool usbWrite(const uint8_t* bytes, size_t length, uint32_t timeout_ms) {
  const uint32_t began = millis();
  size_t sent = 0;
  while (sent < length && static_cast<uint32_t>(millis() - began) < timeout_ms) {
    if (!Serial) return false;
    const int available = Serial.availableForWrite();
    if (available > 0) {
      const size_t chunk = min(length - sent, static_cast<size_t>(available));
      sent += Serial.write(bytes + sent, chunk);
    } else delay(1);
  }
  return sent == length;
}
void dumpFrame(camera_fb_t* fb, uint64_t stamp, uint64_t age, uint64_t period) {
  cam_stop();
  esp_log_level_set("*", ESP_LOG_NONE);
  const Physical physical = physicalSnapshot();
  const auto hal = rw_camera_geometry_probe_stats();
  // Let any previously queued HAL event/log finish before binary framing.
  delay(30);
  const Pixels p = pixels(fb);
  char header[512];
  const int n = snprintf(header, sizeof(header),
      "DUMP_BEGIN,profile=%d,hb=%d,native_height=%d,len=%u,crc32=%08lx,frame_us=%llu,driver_age_us=%llu,frame_period_us=%llu,meta_width=%u,meta_height=%u,gpio_latest_rows=%lu,gpio_span_us=%lu,gpio_period_us=%lu,gpio_vsync_us32=%lu,hal_capture_us=%llu,hal_match=%u,hal_eof_us=%llu,hal_end_vsync_us=%llu,hal_chunk=%lu\n",
      profile, horizontal_blanking, native_height, static_cast<unsigned>(fb->len), static_cast<unsigned long>(p.crc),
      static_cast<unsigned long long>(stamp), static_cast<unsigned long long>(age),
      static_cast<unsigned long long>(period), static_cast<unsigned>(fb->width),
      static_cast<unsigned>(fb->height), static_cast<unsigned long>(physical.lines),
      static_cast<unsigned long>(physical.span_us), static_cast<unsigned long>(physical.period_us),
      static_cast<unsigned long>(physical.vsync_us), static_cast<unsigned long long>(hal.last_capture_start_us),
      hal.last_capture_start_us == stamp, static_cast<unsigned long long>(hal.last_eof_event_us),
      static_cast<unsigned long long>(hal.last_end_vsync_event_us), static_cast<unsigned long>(hal.dma_chunk_bytes));
  const bool header_ok = n > 0 && static_cast<size_t>(n) < sizeof(header) &&
      usbWrite(reinterpret_cast<const uint8_t*>(header), static_cast<size_t>(n), 500);
  const bool ok = header_ok && usbWrite(fb->buf, fb->len, 8000);
  const char* trailer = ok ? "\nDUMP_END,ok=1\n" : "\nDUMP_END,ok=0,reason=usb_timeout\n";
  (void)usbWrite(reinterpret_cast<const uint8_t*>(trailer), strlen(trailer), 500);
  cam_give(fb);
  resumeFresh();
  esp_log_level_set("*", ESP_LOG_WARN);
}
void forceDownloadOnShutdown() {
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
}
void handleCommand(const char* line) {
  if (strcmp(line, "ENTER DOWNLOAD") == 0) {
    if (esp_register_shutdown_handler(forceDownloadOnShutdown) == ESP_OK) {
      Serial.print("USBDBG,download=starting\n");
      delay(150);
      esp_restart();
    } else Serial.println("USBDBG,download=failed");
  } else if (strcmp(line, "STATUS") == 0) {
    status();
  } else if (strncmp(line, "HB ", 3) == 0) {
    char* end = nullptr;
    const long blanking = strtol(line + 3, &end, 10);
    if (end == line + 3 || *end || (blanking != 128 && blanking != 96 && blanking != 80 && blanking != 64))
      Serial.println("HB,error=expected_128_96_80_or_64");
    else if (!setHorizontalBlanking(static_cast<int>(blanking))) Serial.printf("HB,error=%s\n", last_error);
  } else if (strncmp(line, "HEIGHT ", 7) == 0) {
    char* end = nullptr;
    const long height = strtol(line + 7, &end, 10);
    if (end == line + 7 || *end || (height != 488 && height != 496))
      Serial.println("HEIGHT,error=expected_488_or_496");
    else if (!setNativeHeight(static_cast<int>(height))) Serial.printf("HEIGHT,error=%s\n", last_error);
  } else if (strcmp(line, "EXPOSURE AUTO") == 0) {
    if (!setExposure(0)) Serial.printf("EXPOSURE,error=%s\n", last_error);
  } else if (strncmp(line, "EXPOSURE ", 9) == 0) {
    char* end = nullptr;
    const long rows = strtol(line + 9, &end, 10);
    if (end == line + 9 || *end || rows < 1 || rows > 4095)
      Serial.println("EXPOSURE,error=expected_1_to_4095_or_AUTO");
    else if (!setExposure(static_cast<int>(rows))) Serial.printf("EXPOSURE,error=%s\n", last_error);
  } else if (strcmp(line, "DUMP") == 0) {
    if (camera_ready) { dump_requested = true; dump_requested_ms = millis(); }
    else Serial.println("DUMP_ERROR,reason=camera_not_ready");
  } else if (strncmp(line, "PROFILE ", 8) == 0 && strlen(line) == 9 &&
             line[8] >= '0' && line[8] <= '4') {
    if (!selectProfile(static_cast<unsigned>(line[8] - '0')))
      Serial.printf("PROFILE,error=%s\n", last_error);
  } else if (*line) Serial.println("BENCH,command=unknown");
}
void readCommands() {
  for (unsigned i = 0; i < 64 && Serial.available(); ++i) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (!command_overflow && command_size) {
        command[command_size] = 0;
        handleCommand(command);
      }
      command_size = 0; command_overflow = false;
    } else if (!command_overflow) {
      if (command_size + 1 < sizeof(command)) command[command_size++] = ch;
      else command_overflow = true;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(20);
  Serial.setRxBufferSize(512);
  esp_log_level_set("*", ESP_LOG_WARN);
  delay(500);
  Serial.println("BENCH,identity=camera-skip-bench-v25,motor=absent,running=0,motor_mA=0,no_motor_code=1,no_imu=1,no_web=1,timestamp=driver_first_data,completion=hal_vsync_event,gpio_edges=independent_physical");
  if (!initializeCamera()) {
    camera_ready = false;
    Serial.printf("BENCH,error=%s,running=false,motor_mA=0\n", last_error);
  }
  status();
}
void loop() {
  readCommands();
  if (dump_requested && millis() - dump_requested_ms >= 2000) {
    dump_requested = false;
    Serial.println("DUMP_ERROR,reason=no_admissible_frame");
  }
  if (!camera_ready) {
    if (millis() - last_report_ms >= 1000) {
      last_report_ms = millis();
      Serial.printf("BENCH,identity=camera-skip-bench-v25,motor=absent,running=0,error=%s,motor_mA=0\n", last_error);
    }
    delay(10);
    return;
  }
  camera_fb_t* fb = cam_take(pdMS_TO_TICKS(100));
  const uint64_t delivered = esp_timer_get_time();
  if (!fb) {
    ++timeouts;
    if (dump_requested) { dump_requested = false; Serial.println("DUMP_ERROR,reason=frame_timeout"); }
    if (millis() - last_report_ms >= 1000) {
      last_report_ms = millis();
      Serial.printf("BENCH,identity=camera-skip-bench-v25,motor=absent,running=0,profile=%d,error=frame_timeout,timeouts=%lu,motor_mA=0\n", profile, static_cast<unsigned long>(timeouts));
    }
    delay(1);
    return;
  }
  const uint64_t stamp = uint64_t(fb->timestamp.tv_sec) * 1000000ULL + fb->timestamp.tv_usec;
  if (!fb->buf || !fb->len || fb->len > kCapacity || stamp < admit_after_us || stamp > delivered) {
    ++discarded;
    if (millis() - last_report_ms >= 1000) {
      last_report_ms = millis();
      Serial.printf("BENCH,identity=camera-skip-bench-v25,motor=absent,running=0,profile=%d,error=frame_rejected,len=%u,frame_us=%llu,admit_after_us=%llu,discarded=%lu,motor_mA=0\n",
          profile, static_cast<unsigned>(fb->len), static_cast<unsigned long long>(stamp),
          static_cast<unsigned long long>(admit_after_us), static_cast<unsigned long>(discarded));
    }
    cam_give(fb);
    delay(1);
    return;
  }
  ++frames;
  const uint64_t period = last_frame_us && stamp >= last_frame_us ? stamp - last_frame_us : 0;
  last_frame_us = stamp;
  if (dump_requested) {
    dump_requested = false;
    dumpFrame(fb, stamp, delivered - stamp, period);
  } else {
    if (millis() - last_report_ms >= 1000) {
      last_report_ms = millis();
      // Snapshot timing before the diagnostic CRC sweep adds CPU time. HAL
      // counters describe their latest boundary, so identity must match before
      // subtracting EOF/VSYNC event times from this frame's acquisition time.
      const Physical physical = physicalSnapshot();
      const auto hal = rw_camera_geometry_probe_stats();
      const Pixels p = pixels(fb);
      const int exposure_h = readReg(0x03), exposure_l = readReg(0x04), aec = readReg(0xd2);
      Serial.printf("BENCH,identity=camera-skip-bench-v25,motor=absent,running=0,profile=%d,hb=%d,native_height=%d,len=%u,frame_us=%llu,driver_age_us=%llu,frame_period_us=%llu,min=%u,max=%u,mean=%.3f,crc32=%08lx,frames=%lu,timeouts=%lu,discarded=%lu,meta_width=%u,meta_height=%u,internal_free=%u,psram_free=%u,motor_mA=0,gpio_latest_rows=%lu,gpio_span_us=%lu,gpio_period_us=%lu,gpio_vsync_us32=%lu,gpio_frames=%lu,exposure03=%d,exposure04=%d,aec_d2=%d,hal_enabled=%lu,hal_capture_us=%llu,hal_match=%u,hal_start_vsync_us=%llu,hal_eof_us=%llu,hal_end_vsync_us=%llu,hal_prefix=%lu,hal_chunk=%lu,hal_capacity=%lu,hal_boundaries=%lu,hal_published=%lu,hal_empty=%lu,hal_overflow=%lu,hal_event_drops=%lu\n",
          profile, horizontal_blanking, native_height, static_cast<unsigned>(fb->len), static_cast<unsigned long long>(stamp), static_cast<unsigned long long>(delivered - stamp),
          static_cast<unsigned long long>(period), p.min, p.max, p.mean, static_cast<unsigned long>(p.crc),
          static_cast<unsigned long>(frames), static_cast<unsigned long>(timeouts), static_cast<unsigned long>(discarded),
          static_cast<unsigned>(fb->width), static_cast<unsigned>(fb->height),
          static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
          static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
          static_cast<unsigned long>(physical.lines), static_cast<unsigned long>(physical.span_us),
          static_cast<unsigned long>(physical.period_us), static_cast<unsigned long>(physical.vsync_us),
          static_cast<unsigned long>(physical.count), exposure_h, exposure_l, aec,
          static_cast<unsigned long>(hal.enabled), static_cast<unsigned long long>(hal.last_capture_start_us),
          hal.last_capture_start_us == stamp, static_cast<unsigned long long>(hal.last_start_vsync_event_us),
          static_cast<unsigned long long>(hal.last_eof_event_us), static_cast<unsigned long long>(hal.last_end_vsync_event_us),
          static_cast<unsigned long>(hal.last_prefix_bytes), static_cast<unsigned long>(hal.dma_chunk_bytes),
          static_cast<unsigned long>(hal.frame_capacity_bytes), static_cast<unsigned long>(hal.boundaries),
          static_cast<unsigned long>(hal.published_frames), static_cast<unsigned long>(hal.empty_frames),
          static_cast<unsigned long>(hal.overflow_frames), static_cast<unsigned long>(hal.event_queue_drops));
    }
    cam_give(fb);
  }
  delay(1);
}
#endif
