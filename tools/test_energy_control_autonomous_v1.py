#!/usr/bin/env python3
"""V7 Autonomous Energy Control direction and V5 half-cycle regressions (no MCU I/O)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "src" / "config.h").read_text(encoding="utf-8")
RUNNER = (ROOT / "src" / "experiment_runner.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "src" / "experiment_runner.h").read_text(encoding="utf-8")
LOGGER = (ROOT / "src" / "psram_logger.cpp").read_text(encoding="utf-8")

WAIT_PEAK = "WAIT_PEAK"
WAIT_ZERO_CROSS = "WAIT_ZERO_CROSS"
PULSE_ACTIVE = "PULSE_ACTIVE"


@dataclass
class V7HalfCycle:
    phase: str = "IDLE"
    state: str = WAIT_PEAK
    last_zero_ms: int | None = None
    pulse_count: int = 0
    peak_count: int = 0
    tracker_reset_count: int = 0

    def start_kick(self) -> tuple[int, int, int]:
        assert self.phase == "IDLE"
        self.phase = "STRONG_START_KICK"
        return (-1, 1000, 100)

    def finish_kick(self) -> None:
        assert self.phase == "STRONG_START_KICK"
        self.phase = "WAIT_FIRST_PEAK"
        self.state = WAIT_PEAK
        self.tracker_reset_count += 1

    def peak(self, time_ms: int) -> bool:
        if self.state != WAIT_PEAK:
            return False
        first = self.phase == "WAIT_FIRST_PEAK"
        if not first and self.last_zero_ms is not None and time_ms - self.last_zero_ms < 125:
            self.tracker_reset_count += 1
            return False
        self.peak_count += 1
        self.phase = "ENERGY_CONTROL"
        self.state = WAIT_ZERO_CROSS
        return True

    def zero_cross(self, time_ms: int, width_ms: int) -> bool:
        if self.state != WAIT_ZERO_CROSS:
            return False
        if self.last_zero_ms is not None and time_ms - self.last_zero_ms < 250:
            return False
        assert 0 <= width_ms <= 100
        self.last_zero_ms = time_ms
        if width_ms == 0:
            self.state = WAIT_PEAK
            self.tracker_reset_count += 1
            return True
        self.state = PULSE_ACTIVE
        self.pulse_count += 1
        return True

    def pulse_active_peak_or_zero_candidate(self) -> bool:
        return False

    def finish_pulse(self) -> None:
        assert self.state == PULSE_ACTIVE
        self.state = WAIT_PEAK
        self.tracker_reset_count += 1


def autonomous_region() -> str:
    return RUNNER[
        RUNNER.index("bool ExperimentRunner::beginEnergyControlAutonomousPulse"):
        RUNNER.index("void ExperimentRunner::captureAngleOffsets")
    ]


def test_duration_and_frozen_values() -> None:
    assert "ENERGY_CONTROL_AUTONOMOUS_DURATION_MS = 30000UL" in CONFIG
    assert "energy_control_autonomous_v7_side_response_correction_20260904" in CONFIG
    for frozen in (
        "ENERGY_CONTROL_AUTONOMOUS_DEFAULT_TARGET_PEAK_DEG = 8.0f",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_CURRENT_MA = 1000",
        "ENERGY_CONTROL_AUTONOMOUS_START_KICK_PULSE_MS = 100",
        "ENERGY_CONTROL_AUTONOMOUS_CURRENT_MA = 1000",
        "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS = 100",
        "ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_ALPHA = 0.8706716644111074f",
        "ENERGY_CONTROL_AUTONOMOUS_P1_FREE_DECAY_EC_J = 0.0f",
        "ENERGY_CONTROL_AUTONOMOUS_INTEGRAL_KI_MAS_PER_DEG = 0.10f",
        "Q_IDENT_Q_HIGH_MAS = 0.900f",
        "Q_IDENT_MAX_PULSE_MS = 25",
        "FILTER_ADOPTED_INDEX = FILTER_DYNAMIC_HOLD_073_INDEX",
    ):
        assert frozen in CONFIG


def test_one_peak_zero_pulse_cycle_and_pulse_suppression() -> None:
    cycle = V7HalfCycle()
    assert cycle.start_kick() == (-1, 1000, 100)
    cycle.finish_kick()
    # First peak is deliberately exempt from normal zero-to-peak timing.
    assert cycle.peak(110)
    assert cycle.zero_cross(320, 100)
    assert cycle.state == PULSE_ACTIVE
    assert not cycle.pulse_active_peak_or_zero_candidate()
    assert cycle.pulse_count == 1
    cycle.finish_pulse()
    assert cycle.state == WAIT_PEAK
    assert cycle.tracker_reset_count >= 2
    assert not cycle.zero_cross(400, 80)  # no second pulse for that peak
    assert not cycle.peak(410)            # too soon after accepted zero
    assert cycle.peak(470)
    assert cycle.zero_cross(580, 0)
    assert cycle.state == WAIT_PEAK
    assert cycle.pulse_count == 1


def test_minimum_zero_and_peak_timing() -> None:
    cycle = V7HalfCycle()
    cycle.start_kick(); cycle.finish_kick()
    assert cycle.peak(115)
    assert cycle.zero_cross(400, 50)
    cycle.finish_pulse()
    assert not cycle.peak(524)  # strictly less than 125 ms
    assert cycle.peak(525)
    assert not cycle.zero_cross(649, 50)  # strictly less than 250 ms
    assert cycle.zero_cross(650, 50)


def test_source_contract_for_physical_event_state() -> None:
    region = autonomous_region()
    for name in (WAIT_PEAK, WAIT_ZERO_CROSS, PULSE_ACTIVE):
        assert name in HEADER and name in region
    assert "ENERGY_CONTROL_AUTONOMOUS_MIN_HALF_CYCLE_MS = 250UL" in CONFIG
    assert "ENERGY_CONTROL_AUTONOMOUS_MIN_ZERO_TO_PEAK_MS = 125UL" in CONFIG
    assert "status_.pulse_active ||" in region
    assert "resetEnergyControlAutonomousPeakTracker(true);" in region
    assert "energy_control_autonomous_last_accepted_zero_cross_ms_" in region
    assert "energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::PULSE_ACTIVE" in region
    assert "energy_control_autonomous_half_cycle_state_ = EnergyControlAutonomousHalfCycleState::WAIT_PEAK" in region
    assert "pulse transients" in (ROOT / "src" / "web_ui.cpp").read_text(encoding="utf-8")
    assert 'energy_control_autonomous_revision' in LOGGER and 'V7' in LOGGER
    assert 'normal_excitation_direction_policy' in LOGGER
    assert 'normal_direction_sign_fix' in LOGGER
    assert 'start_kick_direction_changed' in LOGGER
    assert 'events_accepted_during_pulse' in LOGGER and 'false' in LOGGER


def test_v7_normal_excitation_direction() -> None:
    def normal_direction(zero_cross_rate_dps: float) -> int:
        side = 1 if zero_cross_rate_dps >= 0.0 else -1
        return side

    # Test 1 and Test 2: normal command is now the same sign as zero-cross motion.
    assert normal_direction(+20.0) == +1
    assert normal_direction(-20.0) == -1
    region = autonomous_region()
    assert "event.q_command_direction = event.physical_next_peak_side;" in region
    assert "event.q_command_direction = -event.physical_next_peak_side;" not in region
    assert "command_matches_zero_cross_motion" in region
    # Test 4: start kick stays independent and unchanged.
    assert "ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION = -1" in CONFIG
    assert "direction != Config::ENERGY_CONTROL_AUTONOMOUS_START_KICK_DIRECTION" in region
    # Test 5: gain-side selection remains a physical-side selection.
    assert "energyControlAutonomousGainForSide(event.physical_next_peak_side)" in region
    assert "energyControlAutonomousGainForSide(event.q_command_direction)" not in region
    assert "Q1_SHADOW_GAIN_PHYSICAL_PLUS_DEG_PER_MAS = 0.29032f" in CONFIG
    assert "Q1_SHADOW_GAIN_PHYSICAL_MINUS_DEG_PER_MAS = 0.25455f" in CONFIG


def test_q_ident_remains_isolated() -> None:
    region = autonomous_region()
    assert "Q_IDENT_" not in region
    assert "ENERGY_CONTROL_AUTONOMOUS_MAX_PULSE_MS" in region
    assert "ENERGY_CONTROL_AUTONOMOUS_Q_MAX_MAS" not in CONFIG


def main() -> None:
    test_duration_and_frozen_values()
    test_one_peak_zero_pulse_cycle_and_pulse_suppression()
    test_minimum_zero_and_peak_timing()
    test_source_contract_for_physical_event_state()
    test_v7_normal_excitation_direction()
    test_q_ident_remains_isolated()
    print("PASS: Autonomous Energy Control V7 direction, V5 half-cycle, and frozen-model regressions")


if __name__ == "__main__":
    main()
