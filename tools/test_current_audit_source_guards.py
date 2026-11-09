#!/usr/bin/env python3
"""Static guard checks for the v46 MEKF/current-audit instrumentation.

This is intentionally a source-level test: Q_meas must remain observational and
the pre-existing V7 safety/status reads must remain in the firmware.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def main() -> None:
    config = (SRC / "config.h").read_text(encoding="utf-8")
    roller = (SRC / "roller485_manager.cpp").read_text(encoding="utf-8")
    runner = (SRC / "experiment_runner.cpp").read_text(encoding="utf-8")
    logger = (SRC / "psram_logger.cpp").read_text(encoding="utf-8")
    imu_manager = (SRC / "imu_manager.cpp").read_text(encoding="utf-8")
    log_types = (SRC / "log_types.h").read_text(encoding="utf-8")

    assert "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000" in config
    assert "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100" in config
    assert "CURRENT_AUDIT_FAST_READ_PERIOD_US = 1000UL" in config
    assert "CURRENT_AUDIT_LOG_PERIOD_US = 2000UL" in config

    # The fast path augments but does not replace the ordinary full safety/status
    # snapshot. Verify the behavior inside update() itself rather than depending
    # on a particular explanatory comment being present.
    update_body = roller.split("void Roller485Manager::update()", 1)[1].split(
        "bool Roller485Manager::setCurrentMa", 1
    )[0]
    for token in (
        "readCurrentFresh(command_mA_ != 0)",
        "readI32(REG_VIN",
        "readU8(REG_MODE",
        "readU8(REG_OUTPUT",
        "readU8(REG_SYS_STATUS",
        "readU8(REG_ERROR_CODE",
        "recordIo(ok)",
    ):
        assert token in update_body, token
    assert "readI32(REG_CURRENT_READBACK" in roller
    assert "readCurrentFresh(true)" in roller

    # Q_meas is emitted only as a telemetry field. No runner controller method
    # is allowed to use it for pulse selection or stopping.
    assert "roller_q_meas_observed_mAms" in runner
    pre_log_controller = runner.split("void ExperimentRunner::logSampleNow()", 1)[0]
    assert "q_meas_observed" not in pre_log_controller

    # V46 attitude adoption: MEKF is the only control/detector attitude in the
    # autonomous path; the adopted hold-073 dynamic-beta Madgwick remains online
    # only as a synchronized comparison signal.
    for token in (
        "q1_shadow_angle_zero_deg_ = status_.pitch_mekf_deg",
        "const float detector_relative_angle_deg = status_.pitch_mekf_detector_relative_deg",
        "const float angle_deg = status_.pitch_mekf_deg",
        "e.theta0_cdeg = centi(status_.pitch_mekf_deg)",
        "identification_peak_angle_deg_ = status_.pitch_mekf_deg",
        "pitch_madgwick_dynamic_abs_deg",
        "v46_mekf_dynamic_compare",
        "i != Config::FILTER_ADOPTED_INDEX",
    ):
        assert token in runner, token
    # The only remaining direct use of the old adopted dynamic angle is the
    # legacy/comparison status field; it must not feed a controller.
    assert runner.count("pitch_dynamic_beta_deg[Config::FILTER_ADOPTED_INDEX]") == 1
    assert "Adafruit_Madgwick" not in imu_manager
    assert "RWLOG v46: adopted MEKF" in log_types
    assert "sizeof(LogSample) == 258" in log_types

    # Video synchronization is preserved byte-for-byte at the protocol level.
    for token in (
        'LED_SYNC_PATTERN_ID[] = "LED_SYNC_PATTERN_V2_LOGGED_ANCHORS"',
        "MID_SYNC_FIRST_MS = 2500UL",
        "MID_SYNC_INTERVAL_MS = 5000UL",
        "SYNC_LED_PIN = 38",
    ):
        assert token in config, token
    assert "row.led_state = status_.led_state ? 1 : 0" in runner
    assert "row.sync_event_id = status_.sync_event_id" in runner

    assert "RWLOG_FORMAT_VERSION = 51" in logger
    assert "actual_current_audit_policy" in logger
    print("V46ac MEKF/current-audit source guard checks passed")


if __name__ == "__main__":
    main()
