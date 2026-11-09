from pathlib import Path
import hashlib

ROOT = Path(__file__).resolve().parents[1]

main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
webui = (ROOT / "src/web_ui.cpp").read_text(encoding="utf-8")
logger = (ROOT / "src/psram_logger.cpp").read_text(encoding="utf-8")
camera = (ROOT / "src/camera_coexistence.cpp").read_text(encoding="utf-8")
header = (ROOT / "src/camera_coexistence.h").read_text(encoding="utf-8")
serial = (ROOT / "src/camera_serial_debug.cpp").read_text(encoding="utf-8")
net = (ROOT / "src/tcp_transport_debug.cpp").read_text(encoding="utf-8")
bounded = (ROOT / "src/bounded_web_server.cpp").read_text(encoding="utf-8")
bounded_h = (ROOT / "src/bounded_web_server.h").read_text(encoding="utf-8")
patch = (ROOT / "src/camera_task_priority_patch.cpp").read_text(encoding="utf-8")
worker = (ROOT / "src/run_control_worker.h").read_text(encoding="utf-8")
pio = (ROOT / "platformio.ini").read_text(encoding="utf-8")

def git_blob_sha(path):
    data = (ROOT / path).read_bytes()
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

expected = {
    "src/experiment_runner.cpp": "58688977951627cdab038d73ca4d64b0c6b3d645",
    "src/psram_logger.h": "63a16781660142a5e3a82721f90cadd9cc2ce9b7",
    "src/imu_manager.cpp": "4842be2a4d91bcd8f2895cfe046489a47a3ce64c",
    "src/roller485_manager.cpp": "c891e50cc832654034d9a6bc64848f48713741c5",
}
for path, sha in expected.items():
    assert git_blob_sha(path) == sha, path

assert main.index("M5.begin(cfg);") < main.index("const bool psram_ok = logger.begin();")
assert main.index("const bool psram_ok = logger.begin();") < main.index("const bool imu_ok = imu.begin();")
assert main.index("const bool imu_ok = imu.begin();") < main.index("const bool camera_ok = camera_probe.begin();")
assert main.index("const bool camera_ok = camera_probe.begin();") < main.index("const bool roller_ok = roller.begin();")

# Phase 1N download recovery changes only transport/UI behavior; the detailed
# normalization guard lives in test_v46ak_download_freeze.py.
assert "if(downloading||refreshInFlight)return;" in webui
assert "},60000);" in webui
assert "STREAM_CHUNK_BYTES = 256" in logger
assert "STREAM_MIN_CHUNK_BYTES = 64" in logger
assert "STREAM_NO_PROGRESS_TIMEOUT_MS = 15000UL" in logger
assert 'server.sendHeader("Connection", "close");' in logger
assert 'server_->on("/rwlog-download-health", HTTP_GET' in webui
assert '#include "bounded_web_server.h"' in main
assert "BoundedWriteWebServer server(Config::HTTP_PORT);" in main
assert "class BoundedWriteWebServer : public WebServer" in bounded_h
assert "kMaxWriteBytes = 256" in bounded
assert "BoundedWriteWebServer::_currentClientWrite(" in bounded
assert "const char* buffer, size_t length" in bounded
assert "_currentClientWrite_P(" in bounded
assert "pacedWrite(" in bounded
assert 'source_tag' in bounded
assert "::send(" in bounded
assert "MSG_DONTWAIT" in bounded
assert "retryableSocketError" in bounded
assert "kSuccessPaceMs = 2" in bounded
assert "kMaxNoProgressMs = 3000" in bounded
assert "vTaskDelay(pdMS_TO_TICKS(kSuccessPaceMs))" in bounded
assert "vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs))" in bounded
assert "NETDBG,web_write_begin" in bounded
assert "NETDBG,web_write_end" in bounded

# Compact startup pose diagnostics are observation-only.
assert "POSEDBG,ms=%lu,reason=%s,fresh=%u,dir=%.2f,acc=%.3f,gyro=%.2f,hold=%lu" in main

# SCCB only at boot on I2C0; never touch BMI270 I2C1.
assert "I2C_NUM_1" not in camera
assert "i2c_param_config(I2C_NUM_0, &sccb)" in camera
assert "c.sccb_i2c_port = I2C_NUM_0;" in camera
assert "i2c_driver_delete(I2C_NUM_0)" in camera

# No background camera acquisition task exists in the serial debug build.
assert "xTaskCreatePinnedToCore(" not in camera
assert "taskLoop" not in camera

# Manual one-shot still obeys the strict XCLK/receiver gate.
capture = camera[camera.index("camera_fb_t* OneShotCamera::acquire") :
                 camera.index("void OneShotCamera::release")]
assert capture.index("setXclkEnabled(true);") < capture.index("cam_start();")
assert capture.index("cam_start();") < capture.index("cam_take(")
assert capture.index("cam_take(") < capture.index("cam_stop();")
assert capture.index("cam_stop();") < capture.index("setXclkEnabled(false);")
assert "esp_camera_fb_get()" not in camera

# USB CDC serial debugger exposes deterministic manual transitions.
for token in (
    "CAMDBG,USB serial camera debugger ready",
    "s=status c=capture_once i=force_idle p=sensor_power_off d=camera_deinit",
    "camera.debugCaptureOnce()",
    "camera.debugForceIdle()",
    "camera.debugPowerSensorOff()",
    "camera.debugDeinit()",
    "WiFi.softAPgetStationNum()",
    "MALLOC_CAP_INTERNAL",
    "MALLOC_CAP_DMA",
    "cameraSerialDebugUpdate(camera_probe, !runner.running());",
):
    assert token in serial + main, token

# Destructive debug actions cannot be triggered while the control run is active.
assert "DENIED_CONTROL_RUN_ACTIVE" in serial

# Phase 1N performs exactly one observation-only one-shot at t>=10 s of the
# Autonomous measurement. It runs on the priority-2 HTTP task and never calls
# the live ExperimentRunner from that task.
for token in (
    "PHASE1N_CAMERA_TRIGGER_MS = 10000UL",
    "phase1n_run_oneshot_coexistence_20260923",
    "servicePhase1nRunCameraValidation(run_control.snapshot())",
    "camera_probe.acquire(500)",
    "camera_probe.release(frame)",
    "imu.acquisitionHealthy()",
    "imu.stale(millis())",
    "run_control.healthSnapshot()",
    'server.on("/camera-run-validation", HTTP_GET',
    "final_rwlog_audit_required",
    "PHASE1N,run=%u",
):
    assert token in main, token
active_loop = main[main.index("if (run_control.active())"):]
assert active_loop.index("servicePhase1nRunCameraValidation") < active_loop.index("web.update()")
phase1n = main[main.index("static void servicePhase1nRunCameraValidation") :
               main.index("static void displayLine")]
assert "runner.status()" not in phase1n
assert "runner.update()" not in phase1n
assert "runner.serviceFast()" not in phase1n
for token in (
    "energy_control_autonomous",
    "pulse_active",
    "measure_elapsed_ms",
    "struct Health",
    "Health healthSnapshot() const",
    "sample_deadline_over",
    "runner_deadline_over",
):
    assert token in worker, token

# Port 81 is an independent raw-TCP probe; port 80 has a tiny WebServer probe.
for token in (
    "WiFiServer g_server(kDiagPort)",
    "constexpr uint16_t kDiagPort = 81",
    "raw tcp port 81 ok",
    "NETDBG,accept",
    "NETDBG,request",
    "NETDBG,response",
    "largest_internal",
    "largest_dma",
):
    assert token in net, token
assert 'server.on("/net-probe", HTTP_GET' in main
assert "tcpTransportDebugBegin();" in main
assert "tcpTransportDebugUpdate();" in main

# Still no image/foot-angle analysis.
for token in ("foot_angle", "right_foot", "left_foot", "centroid", "deg_per_px"):
    assert token not in camera.lower(), token

assert '--wrap=xTaskCreatePinnedToCore' in pio
assert 'strcmp(pcName, "cam_task") == 0' in patch
assert 'kCameraInternalTaskPriority = 3' in patch

print("PASS: Phase 1N run-time one-shot coexistence with preserved control ownership")
