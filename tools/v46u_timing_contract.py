"""Strict inverse of reviewed timing-only changes for retained baseline tests."""
from pathlib import Path
import json
from v46ac_delay_comp_contract import normalize_config as normalize_v46ac_config
from v46ab_no_prediction_contract import normalize_config as normalize_v46ab_config
from v46aa_control_zero_contract import normalize_log_types as normalize_v46aa_log_types
from v46z_comparison_zero_contract import normalize_log_types as normalize_v46z_log_types
from v46ak_observation_contract import normalize_file as normalize_v46ak_file
from v46al_control_contract import normalize_file as normalize_v46al_file
ROOT=Path(__file__).resolve().parents[1]

def normalize_phase1n_camera_validation(path, data):
    """Remove the approved Phase 1N observation layer before frozen timing hashes."""
    if path == 'src/main.cpp':
        start_marker='// Phase 1N: one observation-only camera acquisition during an Autonomous run.\n'
        end_marker='static void displayLine'
        if start_marker in data:
            start=data.index(start_marker)
            end=data.index(end_marker, start)
            data=data[:start]+data[end:]
        route='''  server.on("/camera-run-validation", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json; charset=utf-8", phase1nCameraValidationJson());
  });

'''
        data=data.replace(route,'')
        data=data.replace('''  out.energy_control_autonomous = runner.energyControlAutonomousMode();
  out.pulse_active = st.pulse_active;
  out.measure_elapsed_ms = st.measure_elapsed_ms;
''','')
        data=data.replace('''  if (run_control.active()) {
    // Phase 1N deliberately runs on this lower-priority task. The camera can
    // block this task while run control and BMI270 acquisition keep preempting it.
    servicePhase1nRunCameraValidation(run_control.snapshot());
    web.update();
    delay(1);
    return;
  }

  // Preserve the completed result for HTTP inspection, but arm a fresh Phase 1N
  // record when the next run transfers ownership to the worker.
  phase1n_camera.run_seen = false;

''','''  if (run_control.active()) {
    web.update();
    delay(1);
    return;
  }

''')
    elif path == 'src/run_control_worker.h':
        data=data.replace('''  bool energy_control_autonomous = false;
  bool pulse_active = false;
  uint32_t measure_elapsed_ms = 0;
''','')
        health_start='''  // Small lock-bounded view used by Phase 1N. It intentionally exposes only
  // deadline counters; no controller state is read from the HTTP task.
  struct Health {
'''
        if health_start in data:
            start=data.index(health_start)
            end=data.index('''  bool begin(Step step, Capture capture, void* context) {''', start)
            data=data[:start]+'\n'+data[end:]
        getter='''  Health healthSnapshot() const {
    Health h;
    portENTER_CRITICAL(&mux_);
    h.steps = audit_.steps;
    h.max_period_us = audit_.max_period_us;
    h.max_path_us = audit_.max_path_us;
    h.sample_deadline_over = audit_.sample_completion.over;
    h.sample_deadline_max_us = audit_.sample_completion.maximum;
    h.runner_deadline_over = audit_.runner_work.over;
    h.runner_deadline_max_us = audit_.runner_work.maximum;
    portEXIT_CRITICAL(&mux_);
    return h;
  }
'''
        data=data.replace(getter,'')
    return data

def normalize_camera_coexistence(path, data):
    if path != 'src/main.cpp':
        return data
    data=data.replace('#include "esp_heap_caps.h"\n','')
    data=data.replace('#include "camera_coexistence.h"\n','')
    data=data.replace('#include "camera_serial_debug.h"\n','')
    data=data.replace('#include "bounded_web_server.h"\n','')
    data=data.replace('#include "tcp_transport_debug.h"\n','')
    data=data.replace('BoundedWriteWebServer server(Config::HTTP_PORT);\n',
                      'WebServer server(Config::HTTP_PORT);\n')
    data=data.replace('OneShotCamera camera_probe;\n','')
    data=data.replace('static uint32_t startup_guide_last_diag_ms = 0;\n','')
    data=data.replace('  tcpTransportDebugBegin();\n','')
    data=data.replace('  cameraSerialDebugBegin(camera_probe);\n','')
    data=data.replace('  cameraSerialDebugUpdate(camera_probe, !runner.running());\n  tcpTransportDebugUpdate();\n\n','')
    data=data.replace('''  Serial.printf("Camera internal cam_task: patch=%s core=%d priority=%u->%u\\n",
                camera_boot.cam_task_priority_patch_observed ? "YES" : "NO",
                static_cast<int>(camera_boot.cam_task_core),
                static_cast<unsigned>(camera_boot.cam_task_original_priority),
                static_cast<unsigned>(camera_boot.cam_task_effective_priority));

''','')
    health_route='''  server.on("/camera-health", HTTP_GET, []() {
'''
    if health_route in data:
        route=data.index(health_route)
        comment=data.rfind('\n  //', 0, route)
        start=(comment + 1) if comment >= 0 else route
        end=data.index('''  web.begin(server, runner, imu, roller, logger);''', route)
        data=data[:start]+data[end:]
        data=data.replace(
            '                control_task_ok ? "OK" : "FAILED");\n\n  web.begin(server, runner, imu, roller, logger);',
            '                control_task_ok ? "OK" : "FAILED");\n  web.begin(server, runner, imu, roller, logger);')
    data=data.replace('''  if (static_cast<uint32_t>(now_ms - startup_guide_last_diag_ms) >= 1000UL) {
    startup_guide_last_diag_ms = now_ms;
    Serial.printf(
        "POSEDBG,ms=%lu,reason=%s,fresh=%u,dir=%.2f,acc=%.3f,gyro=%.2f,hold=%lu\\n",
        static_cast<unsigned long>(now_ms),
        reason,
        fresh ? 1U : 0U,
        UprightPoseGuide::directionErrorDeg(r),
        UprightPoseGuide::accelNormG(r),
        UprightPoseGuide::gyroNormDps(r),
        static_cast<unsigned long>(
            startup_upright_since_ms ? now_ms - startup_upright_since_ms : 0U));
  }

''','')
    data=data.replace(
        '  imu.setStartupGuideState(reason, false, 0);\n\n  if (!fresh || !UprightPoseGuide::isUprightStableSample(r)) {',
        '  imu.setStartupGuideState(reason, false, 0);\n  if (!fresh || !UprightPoseGuide::isUprightStableSample(r)) {')
    camera_start='''  // Camera one-shot integration proof only. No marker detection or foot angle.
'''
    if camera_start in data:
        start=data.index(camera_start)
        end=data.index('''  const bool roller_ok = roller.begin();''', start)
        data=data[:start]+data[end:]
    return data

def original_timing_file(path):
    data=normalize_phase1n_camera_validation(path,(ROOT/path).read_text())
    data=normalize_camera_coexistence(path,data)
    # V46al-R1 is the declared active-control delta; remove it before retained hashes.
    data=normalize_v46al_file(path, data)
    # V46ak is observation-only; remove it before checking the retained baseline.
    data=normalize_v46ak_file(path, data)
    if path == 'src/config.h':
        data=normalize_v46ac_config(data)
        data=normalize_v46ab_config(data)
    # V46aa makes the timing detector reference explicit without changing its
    # algebraic value. Reverse its identity/log extension first.
    if path == 'src/config.h':
        data=data.replace('v46aa_control_upright_zero_20260918','v46z_event_relative_angle_zero_20260918')
    if path == 'src/log_types.h':
        data=normalize_v46aa_log_types(data)
        data=normalize_v46z_log_types(data)
    # V46z changes comparison-zero observability only. Reverse its identity next.
    if path == 'src/config.h':
        data=data.replace('v46z_event_relative_angle_zero_20260918','v46y_frozen_imu_1ms_20260918')
    # V46y freezes the selected 1 ms polling specification. Reverse it to the
    # V46x comparison point first, then unwind the earlier timing-only releases.
    if path == 'src/config.h':
        data=data.replace('v46y_frozen_imu_1ms_20260918','v46x_imu_poll_500us_20260918')
        data=data.replace('IMU_POLL_PERIOD_US = 1000UL;  // V46y frozen IMU host polling specification.',
                          'IMU_POLL_PERIOD_US = 500UL;  // V46x: poll at 2 kHz to reduce data-ready discovery latency.')
    # V46x changes only the polling experiment identity/period. Reverse it to
    # V46w first, then unwind the earlier timing-only releases.
    if path == 'src/config.h':
        data=data.replace('v46x_imu_poll_500us_20260918','v46w_imu_poll_2500us_20260918')
        data=data.replace('IMU_POLL_PERIOD_US = 500UL;  // V46x: poll at 2 kHz to reduce data-ready discovery latency.',
                          'IMU_POLL_PERIOD_US = 2500UL;  // V46w: poll once per nominal 400 Hz gyro period.')
    # V46w changes only the polling experiment identity/period. Reverse these
    # before reversing V46v/V46u so retained protected-source hashes still certify
    # the unchanged controller and estimators.
    if path == 'src/config.h':
        data=data.replace('v46w_imu_poll_2500us_20260918','v46v_deadline_tightening_20260918')
        data=data.replace('IMU_POLL_PERIOD_US = 2500UL;  // V46w: poll once per nominal 400 Hz gyro period.',
                          'IMU_POLL_PERIOD_US = 1000UL;')
    # V46v timing-only changes are reversed first so retained V46u/V46s
    # protected-source hashes still certify the unchanged controller/estimators.
    if path == 'src/config.h':
        data=data.replace('v46v_deadline_tightening_20260918','v46u_timing_reader_20260915')
        data=data.replace('static constexpr uint32_t BMI270_I2C_HZ = 1000000UL;  // BMI270 Fast-mode Plus maximum.\n','')
        data=data.replace('CURRENT_AUDIT_FAST_READ_PERIOD_US = 1000UL;  // Try every 1 ms to keep valid samples within the 2 ms audit budget.',
                          'CURRENT_AUDIT_FAST_READ_PERIOD_US = 2000UL;')
    elif path == 'src/imu_manager.cpp':
        data=data.replace('  // V46v timing-only change: BMI270 supports Fast-mode Plus up to 1 MHz.\n  // Keep the same internal bus, axes, ODR and estimator path; only shorten transfers.\n  M5.Imu.setClock(Config::BMI270_I2C_HZ);\n','')
    elif path == 'src/roller485_manager.cpp':
        data=data.replace('  // V46v: try the observational current audit every 1 ms while a pulse is active.\n  // This does not alter pulse timing or current command; it only reduces sample-age slack.\n  bool current_already_fresh = false;\n',
                          '  // V46i keeps the full 2 ms current audit, but it now runs only on Core 0.\n')
        data=data.replace('    current_already_fresh = readCurrentFresh(true);','    readCurrentFresh(true);')
        data=data.replace('  // If the fast audit already obtained a valid current in this same loop,\n  // reuse it instead of immediately reading CURRENT_READBACK a second time.\n  bool ok = current_already_fresh || readCurrentFresh(command_mA_ != 0);',
                          '  bool ok = readCurrentFresh(command_mA_ != 0);')
    edits=json.loads((ROOT/'tools/v46u_timing_delta.json').read_text())
    for edit in reversed(edits):
        if edit['path'] != path:continue
        if data.count(edit['new']) != 1:raise ValueError('Timing delta changed: '+path)
        data=data.replace(edit['new'],edit['old'],1)
    return data.encode()
