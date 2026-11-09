#include "camera_serial_debug.h"

#include <Arduino.h>
#include <WiFi.h>
#include "esp_heap_caps.h"

namespace {

uint32_t g_last_heartbeat_ms = 0;

void printStatus(OneShotCamera& camera, const char* tag) {
  const CameraOneShotSnapshot c = camera.snapshot();
  Serial.printf(
      "CAMDBG,%s,ms=%lu,ap_clients=%u,camera_ok=%u,driver=%u,sensor_power=%u,"
      "oneshot=%u,receiver=%u,xclk=%u,deinit=%u,frames=%lu,failures=%lu,"
      "last_capture_us=%lu,max_capture_us=%lu,internal=%u,dma=%u,psram=%u,error=%s\n",
      tag ? tag : "status",
      static_cast<unsigned long>(millis()),
      static_cast<unsigned>(WiFi.softAPgetStationNum()),
      c.camera_ok ? 1U : 0U,
      c.camera_driver_active ? 1U : 0U,
      c.sensor_powered ? 1U : 0U,
      c.one_shot_mode ? 1U : 0U,
      c.receiver_active ? 1U : 0U,
      c.xclk_active ? 1U : 0U,
      c.camera_deinitialized ? 1U : 0U,
      static_cast<unsigned long>(c.frame_count),
      static_cast<unsigned long>(c.frame_failures),
      static_cast<unsigned long>(c.last_capture_us),
      static_cast<unsigned long>(c.max_capture_us),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
      static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
      camera.lastError());
}

void printHelp() {
  Serial.println("CAMDBG,commands: s=status c=capture_once i=force_idle p=sensor_power_off d=camera_deinit h=help");
  Serial.println("CAMDBG,note: p and d are one-way until reboot; camera actions are blocked during a control run");
}

}  // namespace

void cameraSerialDebugBegin(OneShotCamera& camera) {
  Serial.println("CAMDBG,USB serial camera debugger ready");
  printHelp();
  printStatus(camera, "boot");
  g_last_heartbeat_ms = millis();
}

void cameraSerialDebugUpdate(OneShotCamera& camera, bool allow_camera_actions) {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - g_last_heartbeat_ms) >= 2000UL) {
    g_last_heartbeat_ms = now;
    printStatus(camera, "heartbeat");
  }

  while (Serial.available() > 0) {
    const char raw = static_cast<char>(Serial.read());
    if (raw == '\r' || raw == '\n' || raw == ' ' || raw == '\t') continue;
    const char cmd = (raw >= 'A' && raw <= 'Z') ? static_cast<char>(raw - 'A' + 'a') : raw;

    if (cmd == 'h' || cmd == '?') {
      printHelp();
      continue;
    }
    if (cmd == 's') {
      printStatus(camera, "manual");
      continue;
    }

    if (!allow_camera_actions) {
      Serial.printf("CAMDBG,command=%c,result=DENIED_CONTROL_RUN_ACTIVE\n", cmd);
      continue;
    }

    if (cmd == 'c') {
      printStatus(camera, "before_capture");
      const uint32_t t0 = micros();
      const bool ok = camera.debugCaptureOnce();
      const uint32_t elapsed = static_cast<uint32_t>(micros() - t0);
      Serial.printf("CAMDBG,command=c,result=%s,total_us=%lu\n",
                    ok ? "OK" : "FAILED", static_cast<unsigned long>(elapsed));
      printStatus(camera, "after_capture");
    } else if (cmd == 'i') {
      camera.debugForceIdle();
      Serial.println("CAMDBG,command=i,result=FORCED_IDLE");
      printStatus(camera, "after_idle");
    } else if (cmd == 'p') {
      const bool ok = camera.debugPowerSensorOff();
      Serial.printf("CAMDBG,command=p,result=%s\n", ok ? "OK" : "FAILED");
      printStatus(camera, "after_poweroff");
    } else if (cmd == 'd') {
      const bool ok = camera.debugDeinit();
      Serial.printf("CAMDBG,command=d,result=%s\n", ok ? "OK" : "FAILED");
      printStatus(camera, "after_deinit");
    } else {
      Serial.printf("CAMDBG,unknown_command=%c\n", cmd);
      printHelp();
    }
  }
}
