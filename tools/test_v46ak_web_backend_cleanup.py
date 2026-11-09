#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
web = (ROOT / "src/web_ui.cpp").read_text(encoding="utf-8")
header = (ROOT / "src/web_ui.h").read_text(encoding="utf-8")

begin = web[web.index("void WebUi::begin("):web.index("void WebUi::update()")]
status = web[web.index("String WebUi::statusJson() const"):]

# Only routes used by the current simplified device UI remain.
required_routes = (
    'server_->on("/", HTTP_GET',
    'server_->on("/status.json", HTTP_GET',
    'server_->on("/start-energy-control-autonomous", HTTP_POST',
    'server_->on("/stop", HTTP_POST',
    'server_->on("/clear", HTTP_POST',
    'server_->on("/download/rwlog", HTTP_GET',
)
for token in required_routes:
    assert token in begin, token

removed_routes = (
    "/imu-acquisition.json",
    "/start-passive",
    "/start-energy-control-v0",
    "/energy-control-autonomous/target",
    "/settings",
    "/current-roll/zero",
    "/current-roll/target",
    "/q1-shadow/target",
)
for route in removed_routes:
    assert route not in begin, route

removed_handlers = (
    "handleStartPassive",
    "handleStartEnergyControlV0",
    "handleSetEnergyControlAutonomousTarget",
    "handleCurrentRollZero",
    "handleSetCurrentRollTarget",
    "handleSetQ1ShadowTargetPeakAbs",
    "handleSettings",
)
for name in removed_handlers:
    assert f"void WebUi::{name}" not in web, name
    assert f"void {name}" not in header, name

# The idle status document contains exactly the data families rendered by the new UI.
required_status_keys = (
    "running", "downloading", "state", "ready",
    "energy_control_autonomous_target_peak_deg",
    "pitch_mekf_control_deg", "physical_roll_rate_dps",
    "motor_cmd_mA", "remaining_s",
    "rwlog_downloadable", "download_filename",
    "imu_ok", "roller_ok", "roller_actual_current_mA",
    "battery_mV", "last_error",
)
for key in required_status_keys:
    assert key in status, key

# Historical experiment/debug payloads are no longer generated every status poll.
removed_status_tokens = (
    "startupDiagnosticsJson()",
    "zero_cross_mode",
    "identification_mode",
    "q_run_mode",
    "passive_capture_mode",
    "q_ident_mode",
    "energy_control_v0_mode",
    "q_probe_schedule_id",
    "pitch_beta_series_",
    "beta_applied_series_",
    "pitch_madgwick_dynamic_abs_deg",
    "q1_shadow_target_peak_abs_deg",
    "roller_io_task_running",
    "roller_io_task_ready",
    "roller_io_recovery_count",
    "roller_command_latency_max_us",
    "psram_usage_percent",
    "run_start_us",
)
for token in removed_status_tokens:
    assert token not in status, token

assert "json.reserve(768)" in status
assert "json.reserve(3200)" not in status
assert "appendJsonUint64" not in web + header

# The bounded active-run heartbeat remains independent of the idle status JSON.
handler = web[web.index("void WebUi::handleStatus()"):
              web.index("void WebUi::handleStartEnergyControlAutonomous()")]
assert "run_control.snapshot()" in handler
assert "char body[192]" in handler
assert handler.index("return;") < handler.index("statusJson()")

print("V46ak Web backend cleanup PASS: obsolete routes removed, status JSON slimmed, run heartbeat retained")
