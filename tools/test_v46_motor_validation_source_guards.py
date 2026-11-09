#!/usr/bin/env python3
"""Static guards for the motor-driven V46l attitude-validation path."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def main() -> None:
    config = (SRC / "config.h").read_text(encoding="utf-8")
    runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
    web = (SRC / "web_ui.cpp").read_text(encoding="utf-8")
    roller = (SRC / "roller485_manager.cpp").read_text(encoding="utf-8")
    main_cpp = (SRC / "main.cpp").read_text(encoding="utf-8")
    upright = (SRC / "upright_pose_guide.h").read_text(encoding="utf-8")

    for token in (
        '"energy_control_autonomous_v7_side_response_correction_rwlog30s"',
        "ENERGY_CONTROL_AUTONOMOUS_DURATION_MS = 30000UL",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 1000",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
        "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000",
        "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
    ):
        assert token in config, token

    for token in (
        'server_->on("/start-energy-control-autonomous"',
        "runner_->startEnergyControlAutonomousCapture()",
        "8°測定を開始",
        'server_->on("/stop"',
        'runner_->requestEmergencyStop("web_estop")',
        "displayFrozen",
        "refreshInFlight",
        "if(refreshInFlight)return;",
        "if(lastStatus.running){displayFrozen=true;applyFrozenState();return;}",
        "setInterval(refresh,1000);refresh();",
        "async function postStop(){displayFrozen=false;await post('/stop');}",
        "if(displayFrozen)displayFrozen=false;apply(lastStatus);",
        'json.replace(":nan", ":null")',
        'json.replace(":NaN", ":null")',
        'json.replace(":inf", ":null")',
    ):
        assert token in web, token

    # V46o keeps the UI frozen but receives a small heartbeat to show ESTOP.
    # Test the replacement behavior, not the obsolete 41-second polling pause.
    status_region = web[web.index("void WebUi::handleStatus()"):web.index("void WebUi::handleStartEnergyControlAutonomous()")]
    assert "char body[192]" in status_region
    assert status_region.index("return;") < status_region.index("statusJson()")
    assert "41000" not in web

    for token in (
        "energy_control_autonomous_pulse_live",
        "energy_control_autonomous_pulse_authorized_",
        "beginEnergyControlAutonomousStartKickPulse",
        "beginEnergyControlAutonomousPulse",
        "updateEnergyControlAutonomousPulse",
    ):
        assert token in runner, token

    # V46l preserves fail-closed motor authority across the Core-1 -> Core-0 queue.
    for token in (
        "xQueueReset(command_queue_)",
        "requested_current_mA_ = 0",
        "if (!applyCurrentMa(cmd))",
        "applyCurrentMa(zero)",
        "requested_current_mA_ == 0 && (command_mA_ != 0 || telemetry_.output_raw != 0)",
        "telemetry_.output_raw = cmd.current_mA == 0 ? 0 : 1",
    ):
        assert token in roller, token

    for token in (
        "GUIDE_LED_ON_AFTER_BOOT_MS = 10000UL",
        "REF_AX = 0.021626f",
        "REF_AY = 0.033568f",
        "REF_AZ = -0.999202f",
        "UPRIGHT_STABLE_HOLD_MS = 400UL",
        "MEKF_REINIT_AVERAGE_MS = 800UL",
        "MEKF_REINIT_MIN_SAMPLES = 60UL",
    ):
        assert token in upright, token
    assert "if (startup_upright_confirmed || runner.running()) return;" in main_cpp
    assert "digitalWrite(Config::SYNC_LED_PIN, HIGH);" in main_cpp
    assert "startup_upright_confirmed = true;" in main_cpp
    assert "digitalWrite(Config::SYNC_LED_PIN, LOW);" in main_cpp

    assert "return {-r.ax_g, r.ay_g, -r.az_g};" in runner
    assert "r.gy_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "by_dps * Config::MEKF_GYRO_Y_SCALE" in runner
    assert "-mean_accel_raw.x, mean_accel_raw.y, -mean_accel_raw.z" in runner

    assert "MEKF_VIDEO_OUTPUT_SCALE" not in config
    assert "MEKF_VIDEO_OUTPUT_SIGN" not in config
    assert "status_.pitch_mekf_abs_deg = raw_mekf_pitch_abs_deg_;" in runner
    assert "status_.pitch_mekf_deg = raw_mekf_predicted_abs_deg_ - offset_mekf_pitch_deg_" in runner
    assert "MEKF_GYRO_Y_SCALE = 0.908911f" in config
    assert "mekf_bias_y_dps = mekf6::radToDeg(b.y) / Config::MEKF_GYRO_Y_SCALE" in runner

    assert "rate_sign == -energy_control_autonomous_candidate_detector_side_" in runner
    assert "event.physical_next_peak_side = rate_dps >= 0.0f ? 1 : -1;" in runner
    assert "event.q_command_direction = event.physical_next_peak_side;" in runner

    for token in (
        'status_.last_error = "upright_pose_required_before_start"',
        "g_v46_mekf_run_reinit.active = true",
        "sync_step_ == 0",
        "UprightPoseGuide::MEKF_REINIT_AVERAGE_MS",
        "mekf_.reset();",
        "mekf_.initializeFromAccel(mean_accel)",
        "mekf_.setGyroBiasRadS(mekfStartupBiasFromRaw(",
        'requestEmergencyStop("mekf_reinit_upright_not_stable")',
        'Serial.printf("MEKF run reinit:',
    ):
        assert token in runner, token

    assert "const bool v46_mekf_dynamic_compare = energy_control_autonomous_mode_" in runner
    assert "pitch_madgwick_dynamic_abs_deg" in runner
    assert "v46l_fast_solver_shadow_20260914" in config
    assert "AtomS3R V46l MEKF dual-core motor validation" in main_cpp
    assert "DUAL-CORE V7" in main_cpp
    assert "motor output OFF" not in main_cpp

    print("V46l dual-core physical-frame motor validation source guards passed")


if __name__ == "__main__":
    main()
