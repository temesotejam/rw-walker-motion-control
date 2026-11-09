#!/usr/bin/env python3
"""Static and arithmetic checks for the Q1 direct motor-OFF shadow path."""
import csv
import importlib.util
import math
import tempfile
from pathlib import Path

B0 = 0.01304
RATE = 0.08812
SIDE = 0.11858
G_PLUS = 0.29032
G_MINUS = 0.25455
Q_MIN = 0.454
Q_MAX = 1.197
RATE_MIN = 1.68
RATE_MAX = 27.80


def q1_shadow(rate_dps: float, side: int, target_abs_deg: float):
    if not math.isfinite(rate_dps) or side not in (-1, 1) or not math.isfinite(target_abs_deg):
        return None, None, "INVALID_NONFINITE_STATE"
    abs_rate = abs(rate_dps)
    base = B0 + RATE * abs_rate + SIDE * side
    if abs_rate < RATE_MIN:
        return base, None, "INVALID_RATE_BELOW_SUPPORT"
    if abs_rate > RATE_MAX:
        return base, None, "INVALID_RATE_ABOVE_SUPPORT"
    delta = target_abs_deg - base
    if delta <= 0:
        return base, None, "INVALID_BRAKING_NOT_IDENTIFIED"
    q_req = delta / (G_PLUS if side > 0 else G_MINUS)
    if q_req < Q_MIN:
        return base, q_req, "INVALID_Q_BELOW_SUPPORT"
    if q_req > Q_MAX:
        return base, q_req, "INVALID_Q_ABOVE_SUPPORT"
    return base, q_req, "VALID"


def q1_zero_cross_sign_gate(previous_detector_deg: float, detector_deg: float,
                            physical_gy_dps: float) -> tuple[bool, int]:
    """Mirror the detector-sign acceptance rule; +1 means detector - to +."""
    detector_neg_to_pos = previous_detector_deg < 0.0 <= detector_deg
    detector_pos_to_neg = previous_detector_deg > 0.0 >= detector_deg
    accepted = ((detector_neg_to_pos and physical_gy_dps <= -0.25) or
                (detector_pos_to_neg and physical_gy_dps >= 0.25))
    direction = 1 if detector_neg_to_pos else -1 if detector_pos_to_neg else 0
    return accepted, direction


def main() -> None:
    project = Path(__file__).resolve().parents[1]
    config_text = (project / "src" / "config.h").read_text(encoding="utf-8")
    runner_text = (project / "src" / "experiment_runner.cpp").read_text(encoding="utf-8")
    logger_text = (project / "src" / "psram_logger.cpp").read_text(encoding="utf-8")

    for fragment in (
        'Q1_SHADOW_MOTOR_OFF_ONLY = true',
        'Q1_SHADOW_MODEL_NAME[] = "direct_video_q1_20260829"',
        'Q1_SHADOW_Q_MODEL_AXIS_TYPE[] = "q_target"',
        'Q1_SHADOW_Q_MODEL_AXIS_FIELD[] = "q_target_mA_s"',
        'Q1_SHADOW_INTERCEPT_DEG = 0.01304f',
        'Q1_SHADOW_RATE_GAIN_DEG_PER_DPS = 0.08812f',
        'Q1_SHADOW_SIDE_TERM_DEG = 0.11858f',
        'Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS = 0.29032f',
        'Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS = 0.25455f',
        'Q1_SHADOW_Q_SUPPORT_MIN_MAS = 0.454f',
        'Q1_SHADOW_Q_SUPPORT_MAX_MAS = 1.197f',
    ):
        assert fragment in config_text, fragment
    assert "0.3353708841" not in config_text
    assert "0.2479096027" not in config_text
    assert "updateQ1ShadowAtZeroCross(now_ms);" in runner_text
    assert "updateE2ShadowPeakTracker(now_ms);" not in runner_text
    assert "logger_->addQ1ShadowEvent(event);" in runner_text
    # Q1 itself remains motor-off.  The separate Q_IDENT actual-output route is
    # allowed to call setCurrentMa only through beginQIdentPulse().
    q1_start = runner_text.index("void ExperimentRunner::updateQ1ShadowAtZeroCross")
    q1_end = runner_text.index("void ExperimentRunner::resetQIdentTracker", q1_start)
    q1_body = runner_text[q1_start:q1_end]
    assert "roller_->setCurrentMa" not in q1_body
    assert "beginQIdentPulse" not in q1_body
    assert "bool ExperimentRunner::beginQIdentPulse" in runner_text
    assert "energy_control_v0_pulse_live = energy_control_v0_mode_" in runner_text
    assert "detector_neg_to_pos && rate_dps <= -Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS" in runner_text
    assert "detector_pos_to_neg && rate_dps >= Config::Q1_SHADOW_ZERO_CROSS_MIN_ABS_RATE_DPS" in runner_text
    assert "q1_motor_connected" in logger_text
    for field in (
        "q1_shadow_event_index", "zero_cross_time_ms", "zero_cross_rate_dps",
        "zero_cross_abs_rate_dps", "physical_next_peak_side", "q1_intercept_deg",
        "detector_crossing_direction", "detector_angle_before_deg",
        "detector_angle_after_deg", "crossing_interpolation_alpha",
        "interpolated_zero_cross_time_ms", "physical_roll_rate_before_dps",
        "physical_roll_rate_after_dps", "interpolated_physical_roll_rate_dps",
        "sign_gate_passed",
        "q1_rate_term_deg", "q1_side_term_deg", "q1_baseline_next_peak_abs_deg",
        "target_next_peak_abs_deg", "delta_peak_required_deg",
        "q1_gain_deg_per_mAs", "q_req_shadow_mA_s", "q1_shadow_valid",
        "q1_shadow_invalid_reason", "q_model_axis_mA_s",
    ):
        assert field in logger_text, field

    base, q_req, reason = q1_shadow(5.0, 1, 0.80)
    assert math.isclose(base, B0 + RATE * 5.0 + SIDE, abs_tol=1e-12)
    assert reason == "VALID" and Q_MIN <= q_req <= Q_MAX
    assert q1_shadow(5.0, 1, base)[2] == "INVALID_BRAKING_NOT_IDENTIFIED"
    assert q1_shadow(5.0, 1, base + 0.1 * G_PLUS)[2] == "INVALID_Q_BELOW_SUPPORT"
    assert q1_shadow(5.0, -1, q1_shadow(5.0, -1, 0.0)[0] + 1.3 * G_MINUS)[2] == "INVALID_Q_ABOVE_SUPPORT"
    assert q1_shadow(1.0, 1, 1.0)[2] == "INVALID_RATE_BELOW_SUPPORT"
    assert q1_shadow(30.0, -1, 4.0)[2] == "INVALID_RATE_ABOVE_SUPPORT"
    assert q1_zero_cross_sign_gate(-0.02, 0.01, -4.0) == (True, 1)
    assert q1_zero_cross_sign_gate(0.02, -0.01, 4.0) == (True, -1)
    assert q1_zero_cross_sign_gate(-0.02, 0.01, 4.0) == (False, 1)
    assert q1_zero_cross_sign_gate(0.02, -0.01, -4.0) == (False, -1)

    module_path = Path(__file__).with_name("convert_rwlog_to_csv.py")
    spec = importlib.util.spec_from_file_location("rwlog_converter", module_path)
    converter = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(converter)
    event = {
        "q1_shadow_event_index": 1,
        "zero_cross_time_ms": 1234,
        "zero_cross_rate_dps": 5.0,
        "zero_cross_abs_rate_dps": 5.0,
        "physical_next_peak_side": 1,
        "detector_crossing_direction": -1,
        "detector_angle_before_deg": 0.02,
        "detector_angle_after_deg": -0.01,
        "crossing_interpolation_alpha": 2.0 / 3.0,
        "interpolated_zero_cross_time_ms": 1233,
        "physical_roll_rate_before_dps": 5.2,
        "physical_roll_rate_after_dps": 5.0,
        "interpolated_physical_roll_rate_dps": 5.07,
        "sign_gate_passed": True,
        "q1_intercept_deg": B0,
        "q1_rate_term_deg": RATE * 5.0,
        "q1_side_term_deg": SIDE,
        "q1_baseline_next_peak_abs_deg": base,
        "target_next_peak_abs_deg": 0.80,
        "delta_peak_required_deg": 0.80 - base,
        "q1_gain_deg_per_mAs": G_PLUS,
        "q_model_axis_mA_s": q_req,
        "q_req_shadow_mA_s": q_req,
        "q1_shadow_valid": True,
        "q1_shadow_invalid_reason": "VALID",
        "q1_shadow_invalid_reason_code": 0,
    }
    with tempfile.TemporaryDirectory() as temp:
        out = Path(temp)
        assert converter.write_q1_shadow_events({"q1_shadow_events": [event]}, out) == 1
        with (out / "q1_shadow_events.csv").open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 1
        assert rows[0]["q_model_axis_mA_s"] == str(q_req)
        assert rows[0]["q1_shadow_invalid_reason"] == "VALID"
        assert rows[0]["detector_crossing_direction"] == "-1"
        assert rows[0]["sign_gate_passed"] == "True"
    print("Q1 direct motor-OFF shadow and CSV extraction checks passed")


if __name__ == "__main__":
    main()
