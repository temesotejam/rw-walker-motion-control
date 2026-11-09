#!/usr/bin/env python3
"""V0.2 output-gate and metadata-capacity tests (no MCU I/O)."""
from __future__ import annotations

import csv
import json
import tempfile
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
THRESHOLD_DEG = 0.7
MAX_EVENTS = 128
METADATA_RESERVE_BYTES = 1024 * 1024
FIXED_METADATA_WORST_CASE_BYTES = 64 * 1024

WAIT_INITIAL_EXCURSION = 0
ARMED_FOR_ZERO_CROSS = 1
WAIT_OPPOSITE_EXCURSION = 2


@dataclass
class GateResult:
    authorized: bool
    reason: str


class V01OutputGate:
    """Mirror the firmware's output-only gate in detector coordinates."""

    def __init__(self) -> None:
        self.state = WAIT_INITIAL_EXCURSION
        self.previous_side = 0
        self.rearm_seen = False
        self.max_abs_excursion = 0.0

    def update_detector(self, detector_deg: float) -> None:
        self.max_abs_excursion = max(self.max_abs_excursion, abs(detector_deg))
        if self.state == WAIT_INITIAL_EXCURSION:
            if abs(detector_deg) >= THRESHOLD_DEG:
                self.rearm_seen = True
                self.state = ARMED_FOR_ZERO_CROSS
            return
        if self.state != WAIT_OPPOSITE_EXCURSION:
            return
        # detector derivative is anti-correlated with physical +gy.  After a
        # next-peak side s, physical travel to -s has detector sign -s.
        if self.previous_side and detector_deg * self.previous_side <= -THRESHOLD_DEG:
            self.rearm_seen = True
            self.state = ARMED_FOR_ZERO_CROSS

    def candidate(self, side: int, *, v0_capacity=False, q1_capacity=False) -> GateResult:
        assert side in (-1, 1)
        if v0_capacity or q1_capacity:
            return GateResult(False, "INVALID_EVENT_LOG_OVERFLOW")
        if self.state == WAIT_INITIAL_EXCURSION:
            return GateResult(False, "WAIT_INITIAL_EXCURSION")
        if self.state == WAIT_OPPOSITE_EXCURSION:
            return GateResult(False, "WAIT_REARM_EXCURSION")
        if self.previous_side and side != -self.previous_side:
            return GateResult(False, "NONALTERNATING_SIDE")
        self.previous_side = side
        self.rearm_seen = False
        self.max_abs_excursion = 0.0
        self.state = WAIT_OPPOSITE_EXCURSION
        return GateResult(True, "VALID")


def make_q1_event(index: int) -> dict[str, object]:
    return {
        "q1_shadow_event_index": index,
        "zero_cross_time_ms": 59999,
        "zero_cross_rate_dps": -27.8000,
        "zero_cross_abs_rate_dps": 27.8000,
        "physical_next_peak_side": -1,
        "detector_crossing_direction": 1,
        "detector_angle_before_deg": -1.23456,
        "detector_angle_after_deg": 1.23456,
        "crossing_interpolation_alpha": 0.999999,
        "interpolated_zero_cross_time_ms": 59999,
        "physical_roll_rate_before_dps": -27.8,
        "physical_roll_rate_after_dps": -27.8,
        "interpolated_physical_roll_rate_dps": -27.8,
        "sign_gate_passed": True,
        "q1_intercept_deg": 0.01304,
        "q1_rate_term_deg": 2.45,
        "q1_side_term_deg": -0.11858,
        "q1_baseline_next_peak_abs_deg": 2.34,
        "target_next_peak_abs_deg": 2.0,
        "delta_peak_required_deg": -0.34,
        "q1_gain_deg_per_mAs": 0.25455,
        "q_model_axis_mA_s": 0.9,
        "q_req_shadow_mA_s": 0.9,
        "q1_shadow_valid": True,
        "q1_shadow_invalid_reason": "VALID",
        "q1_shadow_invalid_reason_code": 0,
    }


def make_v0_event(index: int) -> dict[str, object]:
    return {
        "event_index": index,
        "zero_cross_time_ms": 59999,
        "zero_cross_rate_dps": -27.8,
        "zero_cross_abs_rate_dps": 27.8,
        "physical_next_peak_side": -1,
        "output_gate_state": ARMED_FOR_ZERO_CROSS,
        "previous_accepted_physical_next_peak_side": 1,
        "candidate_physical_next_peak_side": -1,
        "rearm_excursion_seen": True,
        "maximum_excursion_since_previous_cross_deg": 12.34567,
        "rearm_threshold_deg": 0.7,
        "side_alternation_passed": True,
        "output_authorized": True,
        "output_blocked_reason": "VALID",
        "output_blocked_reason_code": 0,
        "q_command_direction": 1,
        "passive_next_peak_abs_deg": 2.34567,
        "passive_energy_j": 0.00012345,
        "target_peak_abs_deg": 2.0,
        "target_energy_j": 0.00019893,
        "delta_energy_required_j": 0.00007548,
        "q1_gain_deg_per_mA_s": 0.25455,
        "q_selected_mA_s": 0.90000,
        "q_effective_pred_mA_s": 0.90000,
        "predicted_next_peak_abs_deg": 2.57477,
        "predicted_next_energy_j": 0.00019800,
        "q_saturated_at_max": True,
        "rate_support_status": 1,
        "q_support_status": 2,
        "vbat_mV": 8100,
        "i0_estimated_mA": -299.9999,
        "solver_required_width_ms": 24.9999,
        "solver_selected_integer_width_ms": 25,
        "command_current_mA": 300,
        "pulse_width_ms": 25,
        "pulse_start_ms": 59999,
        "pulse_end_ms": 60024,
        "output_executed": True,
        "valid": True,
        "reason": "VALID",
        "reason_code": 0,
    }


def test_gate_state_machine() -> None:
    gate = V01OutputGate()
    # A quiet 0.2--0.6 deg static trace never produces an authorization.
    for angle, side in ((0.2, 1), (-0.6, -1), (0.4, 1), (-0.5, -1)) * 50:
        gate.update_detector(angle)
        assert not gate.candidate(side).authorized
    assert gate.state == WAIT_INITIAL_EXCURSION

    gate.update_detector(0.699)
    assert gate.state == WAIT_INITIAL_EXCURSION
    gate.update_detector(0.700)
    assert gate.state == ARMED_FOR_ZERO_CROSS
    first = gate.candidate(+1)
    assert first.authorized and gate.state == WAIT_OPPOSITE_EXCURSION

    # No further output is possible while the post-cross trace remains static.
    for angle, side in ((0.2, +1), (-0.6, -1), (0.4, +1), (-0.5, -1)) * 50:
        gate.update_detector(angle)
        blocked = gate.candidate(side)
        assert not blocked.authorized and blocked.reason == "WAIT_REARM_EXCURSION"

    # Re-arm only after the opposite physical excursion and require alternation.
    gate.update_detector(-0.7)  # previous + side -> detector - means physical - side.
    assert gate.state == ARMED_FOR_ZERO_CROSS
    assert gate.candidate(+1).reason == "NONALTERNATING_SIDE"
    second = gate.candidate(-1)
    assert second.authorized and gate.state == WAIT_OPPOSITE_EXCURSION
    gate.update_detector(+0.7)
    assert gate.candidate(+1).authorized


def test_v02_run1_rearm_window() -> None:
    # The 0.2--0.6 deg static trace remains below the new 0.7 deg gate.
    gate = V01OutputGate()
    for angle, side in ((0.2, +1), (-0.6, -1), (0.4, +1), (-0.5, -1)) * 50:
        gate.update_detector(angle)
        assert gate.candidate(side).reason == "WAIT_INITIAL_EXCURSION"

    # V0.1 Run 1 event 10 accepted physical +. The subsequent detector
    # event 15 had detector excursion -0.81965 deg; it must rearm on physical -.
    gate.update_detector(+0.7)
    assert gate.candidate(+1).authorized
    gate.update_detector(-0.81965)
    event11 = gate.candidate(-1)
    assert event11.authorized and event11.reason == "VALID"

def test_capacity_is_fail_closed() -> None:
    gate = V01OutputGate()
    gate.update_detector(1.1)
    assert gate.candidate(+1, v0_capacity=True).reason == "INVALID_EVENT_LOG_OVERFLOW"
    assert gate.state == ARMED_FOR_ZERO_CROSS
    assert not gate.candidate(+1, q1_capacity=True).authorized


def test_metadata_worst_case_and_csv_round_trip() -> None:
    metadata = {
        "format": "rwlog_energy_control_v0",
        "firmware_revision": "energy_control_v0_2_rearm0700_alternation_metadata_20260903",
        "measurement_mode": "energy_control_v0_p1_step_q1",
        "energy_control_v0_revision": "V0.2",
        "energy_control_v0_event_overflow": False,
        "q1_shadow_event_overflow": False,
        "q1_shadow_events": [make_q1_event(i + 1) for i in range(MAX_EVENTS)],
        "energy_control_v0_events": [make_v0_event(i + 1) for i in range(MAX_EVENTS)],
    }
    payload = json.dumps(metadata, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
    # 64 KiB conservatively covers the fixed metadata sections emitted outside
    # these two V0-active arrays.  The observed base metadata is smaller.
    worst_case_bytes = len(payload) + FIXED_METADATA_WORST_CASE_BYTES
    assert worst_case_bytes * 100 <= METADATA_RESERVE_BYTES * 80, (
        worst_case_bytes, METADATA_RESERVE_BYTES
    )
    reparsed = json.loads(payload.decode("utf-8"))
    assert len(reparsed["q1_shadow_events"]) == MAX_EVENTS
    assert len(reparsed["energy_control_v0_events"]) == MAX_EVENTS
    required = set(make_v0_event(1))
    assert required <= set(reparsed["energy_control_v0_events"][0])

    import importlib.util
    converter_path = ROOT / "tools" / "convert_rwlog_to_csv.py"
    spec = importlib.util.spec_from_file_location("rwlog_converter", converter_path)
    converter = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(converter)
    with tempfile.TemporaryDirectory() as temp:
        out = Path(temp)
        assert converter.write_energy_control_v0_events(metadata, out) == MAX_EVENTS
        with (out / "energy_control_v0_events.csv").open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == MAX_EVENTS
        assert rows[0]["output_authorized"] == "True"
        assert rows[0]["rearm_threshold_deg"] == "0.7"


def test_source_contract() -> None:
    config = (ROOT / "src" / "config.h").read_text(encoding="utf-8")
    runner = (ROOT / "src" / "experiment_runner.cpp").read_text(encoding="utf-8")
    logger_h = (ROOT / "src" / "psram_logger.h").read_text(encoding="utf-8")
    logger_cpp = (ROOT / "src" / "psram_logger.cpp").read_text(encoding="utf-8")
    assert 'ENERGY_CONTROL_V0_REARM_EXCURSION_DEG = 0.7f' in config
    for name in ("WAIT_INITIAL_EXCURSION", "ARMED_FOR_ZERO_CROSS", "WAIT_OPPOSITE_EXCURSION"):
        assert name in runner
    for name in (
        "INVALID_WAIT_INITIAL_EXCURSION", "INVALID_WAIT_REARM_EXCURSION",
        "INVALID_NONALTERNATING_SIDE", "INVALID_EVENT_LOG_OVERFLOW",
    ):
        assert name in config and name in runner and name in logger_cpp
    assert "updateEnergyControlV0OutputGate(detector_angle_deg);" in runner
    assert "disarmEnergyControlV0AfterAcceptedCross(event.physical_next_peak_side);" in runner
    assert "energyControlV0EventCapacityReached" in runner
    assert "q1ShadowEventCapacityReached" in runner
    assert "kMetadataJsonReserveBytes = 1024U * 1024U" in logger_cpp
    for field in (
        "output_gate_state", "previous_accepted_physical_next_peak_side",
        "candidate_physical_next_peak_side", "rearm_excursion_seen",
        "maximum_excursion_since_previous_cross_deg", "rearm_threshold_deg",
        "side_alternation_passed", "output_authorized", "output_blocked_reason",
    ):
        assert field in logger_h and field in logger_cpp
    # Frozen model/energy parameters remain intact.
    for frozen in (
        "ENERGY_CONTROL_V0_TARGET_PEAK_DEG = 2.0f",
        "ENERGY_CONTROL_V0_Q_MAX_MAS = 0.900f",
        "Q1_SHADOW_INTERCEPT_DEG = 0.01304f",
        "Q1_SHADOW_RATE_GAIN_DEG_PER_DPS = 0.08812f",
        "Q_IDENT_MIN_PULSE_MS = 5", "Q_IDENT_MAX_PULSE_MS = 25",
    ):
        assert frozen in config


def main() -> None:
    test_gate_state_machine()
    test_v02_run1_rearm_window()
    test_capacity_is_fail_closed()
    test_metadata_worst_case_and_csv_round_trip()
    test_source_contract()
    print("Energy Control V0.2 rearm/alternation/metadata tests: PASS")


if __name__ == "__main__":
    main()
