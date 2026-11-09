#!/usr/bin/env python3
"""Static/protocol tests for the v3 fixed-Q Q_IDENT path.

They exercise the schedule and fail-closed rules without an MCU and inspect
source boundaries so Q_IDENT remains the sole physical-output route.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "src" / "config.h").read_text(encoding="utf-8")
RUNNER = (ROOT / "src" / "experiment_runner.cpp").read_text(encoding="utf-8")
LOGGER_H = (ROOT / "src" / "psram_logger.h").read_text(encoding="utf-8")
LOGGER_CPP = (ROOT / "src" / "psram_logger.cpp").read_text(encoding="utf-8")
CONVERTER = (ROOT / "tools" / "convert_rwlog_to_csv.py").read_text(encoding="utf-8")

EXPECTED = (
    ((0.0, 0.786, 0.900, 0.454, 0.0, 0.900), (0.900, 0.454, 0.0, 0.786, 0.900, 0.0)),
    ((0.454, 0.900, 0.786, 0.0, 0.454, 0.786), (0.786, 0.0, 0.454, 0.900, 0.786, 0.454)),
    ((0.786, 0.0, 0.454, 0.900, 0.786, 0.454), (0.0, 0.900, 0.786, 0.454, 0.0, 0.900)),
    ((0.900, 0.454, 0.0, 0.786, 0.900, 0.0), (0.454, 0.786, 0.900, 0.0, 0.454, 0.786)),
)
LEVELS = {0.0, 0.454, 0.786, 0.900}


def parse_schedule() -> tuple:
    match = re.search(r"Q_IDENT_SCHEDULE_Q_MAS\[.*?= \{(.*?)\n\};", CONFIG, flags=re.S)
    assert match, "Q_IDENT schedule missing"
    values = [float(v) for v in re.findall(r"([0-9]+\.[0-9]+)f", match.group(1))]
    assert len(values) == 48, values
    return tuple(tuple(tuple(values[(run * 12 + side * 6 + occurrence)] for occurrence in range(6))
                       for side in range(2)) for run in range(4))


@dataclass
class Event:
    armed: bool
    occurrence: int
    q: float | None
    command_direction: int
    pulse: bool
    valid: bool
    reason: str


class Protocol:
    def __init__(self, schedule_id: int):
        self.schedule_id = schedule_id
        self.armed = False
        self.streak = 0
        self.last_arm_side = 0
        self.counts = [0, 0]  # physical +, physical -

    def cross(self, rate: float, *, roller=True, battery=True, solver=True, estop=False) -> Event:
        side = 1 if rate >= 0 else -1
        abs_rate = abs(rate)
        if not self.armed:
            qualifies = abs_rate >= 30.0 and (self.last_arm_side == 0 or side != self.last_arm_side)
            if qualifies:
                self.streak += 1
                self.last_arm_side = side
                if self.streak >= 2:
                    self.armed = True
                    return Event(True, 0, None, 0, False, False, "ARMED_EVENT_NO_OUTPUT")
                return Event(False, 0, None, 0, False, False, "ARM_WAITING")
            self.streak = 0
            self.last_arm_side = 0
            return Event(False, 0, None, 0, False, False, "ARM_WAITING")
        if abs_rate < 1.68:
            return Event(True, 0, None, 0, False, False, "INVALID_RATE_BELOW_SUPPORT")
        if abs_rate > 27.80:
            return Event(True, 0, None, 0, False, False, "INVALID_RATE_ABOVE_SUPPORT")
        idx = 0 if side > 0 else 1
        if self.counts[idx] >= 6:
            return Event(True, 0, None, 0, False, False, "INVALID_SCHEDULE_EXHAUSTED")
        self.counts[idx] += 1
        occurrence = self.counts[idx]
        q = EXPECTED[self.schedule_id][idx][occurrence - 1]
        direction = -side
        if q == 0.0:
            return Event(True, occurrence, q, direction, False, True, "VALID_Q_ZERO_NO_PULSE")
        if estop:
            return Event(True, occurrence, q, direction, False, False, "INVALID_ESTOP_OR_STATE")
        if not roller:
            return Event(True, occurrence, q, direction, False, False, "INVALID_ROLLER_NOT_READY")
        if not battery:
            return Event(True, occurrence, q, direction, False, False, "INVALID_BATTERY_GUARD")
        if not solver:
            return Event(True, occurrence, q, direction, False, False, "INVALID_PULSE_WIDTH_GUARD")
        return Event(True, occurrence, q, direction, True, True, "VALID")


def arm(protocol: Protocol) -> None:
    first = protocol.cross(-30.0)
    second = protocol.cross(+30.0)
    assert first.reason == "ARM_WAITING" and not first.pulse
    assert second.reason == "ARMED_EVENT_NO_OUTPUT" and second.armed and not second.pulse


def test_schedule() -> None:
    parsed = parse_schedule()
    assert parsed == EXPECTED
    for side in range(2):
        for level in LEVELS:
            assert sum(1 for run in range(4) for q in parsed[run][side][:5] if q == level) == 5
    assert all(q in LEVELS for run in parsed for side in run for q in side)
    assert 1.197 not in {q for run in parsed for side in run for q in side}


def test_arm_support_direction_and_zero() -> None:
    p = Protocol(0)
    arm(p)
    event = p.cross(+1.68)
    assert event.occurrence == 1 and event.q == 0.0 and event.command_direction == -1
    assert event.valid and not event.pulse and event.reason == "VALID_Q_ZERO_NO_PULSE"
    event = p.cross(-27.80)
    assert event.occurrence == 1 and event.q == 0.900 and event.command_direction == +1
    assert event.valid and event.pulse
    assert not p.cross(+1.679).pulse
    assert not p.cross(-27.801).pulse


def test_invalid_consumes_without_substitution_or_carryover() -> None:
    p = Protocol(1)
    arm(p)
    failed = p.cross(+10.0, battery=False)
    assert failed.q == 0.454 and failed.occurrence == 1 and not failed.pulse
    next_event = p.cross(+10.0)
    assert next_event.q == 0.900 and next_event.occurrence == 2 and next_event.pulse
    assert not p.cross(-10.0, solver=False).pulse
    assert not p.cross(-10.0, roller=False).pulse
    assert not p.cross(-10.0, estop=True).pulse


def test_schedule_end_and_reset() -> None:
    p = Protocol(2)
    arm(p)
    for _ in range(6):
        p.cross(+10.0)
    assert p.cross(+10.0).reason == "INVALID_SCHEDULE_EXHAUSTED"
    replacement = Protocol(2)
    assert replacement.counts == [0, 0] and not replacement.armed


def current_goal(vbat_v: float, command_mA: float = 300.0) -> float:
    i_sat = 329.547119 + 46.253815 * (vbat_v - 7.50)
    ratio = command_mA / i_sat
    return command_mA / (1.0 + ratio**3.86) ** (1.0 / 3.86)


def tau_s(command_mA: float = 300.0) -> float:
    ratio = command_mA / 372.0
    return (31.4 + (72.8 - 31.4) / (1.0 + ratio**4.35)) / 1000.0


def charge(vbat_v: float, width_ms: float, signed_i0_mA: float = 0.0) -> float:
    target = current_goal(vbat_v)
    tau = tau_s()
    t = width_ms / 1000.0
    return target * t + (signed_i0_mA - target) * tau * (1.0 - math.exp(-t / tau))


def required_width(vbat_v: float, q_target: float) -> float:
    lo, hi = 0.0, 100.0
    assert charge(vbat_v, hi) >= q_target
    for _ in range(40):
        mid = (lo + hi) / 2.0
        if charge(vbat_v, mid) < q_target:
            lo = mid
        else:
            hi = mid
    return hi


def test_qhigh_at_guard_minimum() -> None:
    assert "Q_IDENT_Q_HIGH_MAS = 0.900f" in CONFIG
    assert "Q_IDENT_QHIGH_SELECTION_VBAT_MV = 6180" in CONFIG
    assert "Q_IDENT_QHIGH_SELECTION_I0_MA = 0.0f" in CONFIG
    assert "Q_IDENT_MAX_PULSE_MS = 25" in CONFIG
    assert "Q_IDENT_BATTERY_MIN_MV = 6180" in CONFIG
    assert "Q_IDENT_BATTERY_MAX_MV = 8100" in CONFIG
    assert "energy_control_autonomous_v7_side_response_correction_20260904" in CONFIG
    required = required_width(6.180, 0.900)
    selected = math.ceil(required - 1e-4)
    assert 22.9 < required < 23.0 and selected == 23
    assert charge(6.180, selected) >= 0.900
    assert 25 - selected == 2
    assert math.ceil(required_width(6.180, 1.197) - 1e-4) == 27
    assert math.ceil(required_width(6.180, 1.100) - 1e-4) == 26


def test_observed_full_charge_range_stays_inside_pulse_guard() -> None:
    # 8.100 V is the automatic upper guard.  This verifies that every
    # nonzero scheduled Q is still selected inside the immutable 5--25 ms
    # pulse guard; no manual voltage check is an operator prerequisite.
    for q_target, expected_width_ms in ((0.454, 15), (0.786, 20), (0.900, 22)):
        required = required_width(8.100, q_target)
        selected = math.ceil(required - 1e-4)
        assert selected == expected_width_ms
        assert 5 <= selected <= 25
        assert charge(8.100, selected) >= q_target


def test_source_isolation_and_diagnostics() -> None:
    # The historical schedule remains in config for provenance but cannot start.
    start = RUNNER.index("bool ExperimentRunner::startQIdentCapture")
    frozen_return = RUNNER.index("return false;", start)
    legacy_enable = RUNNER.index("q_ident_mode_ = true;", start)
    assert frozen_return < legacy_enable
    assert '"/start-q-ident"' not in (ROOT / "src" / "web_ui.cpp").read_text(encoding="utf-8")
    assert '"/start-energy-control-v0"' in (ROOT / "src" / "web_ui.cpp").read_text(encoding="utf-8")
    assert "energy_control_v0_pulse_live = energy_control_v0_mode_" in RUNNER
    assert "bool ExperimentRunner::beginEnergyControlV0Pulse" in RUNNER
    assert "void ExperimentRunner::beginPulse" in RUNNER and 'status_.last_error = "motor_off_only";' in RUNNER
    assert "ENERGY_CONTROL_V0_TARGET_PEAK_DEG = 2.0f" in CONFIG
    assert "ENERGY_CONTROL_V0_Q_MAX_MAS = 0.900f" in CONFIG
    assert "ENERGY_CONTROL_V0_Q_SEARCH_STEP_MAS = 0.001f" in CONFIG
    assert "Q_IDENT_MIN_PULSE_MS = 5" in CONFIG and "Q_IDENT_MAX_PULSE_MS = 25" in CONFIG
    assert "Q_IDENT_BATTERY_MIN_MV = 6180" in CONFIG and "Q_IDENT_BATTERY_MAX_MV = 8100" in CONFIG
    for name in (
        "energy_control_v0_events", "passive_energy_j", "target_energy_j",
        "delta_energy_required_j", "q_selected_mA_s", "q_effective_pred_mA_s",
        "q_saturated_at_max", "rate_support_status", "q_support_status",
        "solver_required_width_ms", "output_executed",
    ):
        assert name in LOGGER_H and name in LOGGER_CPP and name in CONVERTER
    assert "reason_code" in LOGGER_CPP and "reason_code" in CONVERTER

def main() -> int:
    test_schedule()
    test_arm_support_direction_and_zero()
    test_invalid_consumes_without_substitution_or_carryover()
    test_schedule_end_and_reset()
    test_qhigh_at_guard_minimum()
    test_observed_full_charge_range_stays_inside_pulse_guard()
    test_source_isolation_and_diagnostics()
    print("Q_IDENT frozen-provenance and Energy Control V0 guard tests PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

