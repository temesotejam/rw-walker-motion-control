#!/usr/bin/env python3
"""Offline replay/safety tests for the frozen Energy Control V0 equations.

The arithmetic mirrors the firmware deliberately: Q1 Q=0 baseline, P1 STEP
potential and a bounded 1-D Q search.  It contains no motor I/O.
"""
from __future__ import annotations

from math import asin, cos, fabs, isfinite, sin, sqrt
from pathlib import Path

RADIUS_M = 0.150
INNER_EDGE_M = 0.005
OUTER_EDGE_M = 0.045
CG_HEIGHT_M = 0.120
MASS_KG = 0.1997
G = 9.80665
TARGET_DEG = 2.0
Q_MAX = 0.900
Q_STEP = 0.001
Q1_INTERCEPT = 0.01304
Q1_RATE_GAIN = 0.08812
Q1_SIDE_TERM = 0.11858
GAIN = {1: 0.29032, -1: 0.25455}


def potential_j(amplitude_deg: float) -> float:
    theta = abs(amplitude_deg) * 0.01745329251994329577
    inner = asin(INNER_EDGE_M / RADIUS_M)
    outer = asin(OUTER_EDGE_M / RADIUS_M)
    if not isfinite(theta) or theta > outer:
        return float("nan")
    center = sqrt(RADIUS_M * RADIUS_M - INNER_EDGE_M * INNER_EDGE_M)
    height = (
        CG_HEIGHT_M * cos(theta) + INNER_EDGE_M * sin(theta)
        if theta <= inner
        else RADIUS_M + (CG_HEIGHT_M - center) * cos(theta)
    )
    return MASS_KG * G * (height - CG_HEIGHT_M)


def decide(rate_dps: float, side: int, vbat_mv: int = 7500) -> dict[str, float | int | bool | str]:
    assert side in (-1, 1)
    baseline = Q1_INTERCEPT + Q1_RATE_GAIN * abs(rate_dps) + Q1_SIDE_TERM * side
    free_energy = potential_j(baseline)
    target_energy = potential_j(TARGET_DEG)
    if not (isfinite(free_energy) and isfinite(target_energy)):
        return {"reason": "INVALID_POTENTIAL_DOMAIN", "output": False}
    delta = target_energy - free_energy
    result: dict[str, float | int | bool | str] = {
        "baseline": baseline,
        "free_energy": free_energy,
        "target_energy": target_energy,
        "delta": delta,
        "direction": -side,
        "rate_support": 0 if abs(rate_dps) < 1.68 else 1 if abs(rate_dps) <= 27.80 else 2,
    }
    if delta <= 0:
        result.update(q=0.0, output=False, reason="VALID_PASSIVE_NO_OUTPUT")
        return result
    best_q = 0.0
    best_peak = baseline
    best_energy = free_energy
    best_error = abs(target_energy - best_energy)
    for index in range(1, round(Q_MAX / Q_STEP) + 1):
        q = index * Q_STEP
        peak = baseline + GAIN[side] * q
        energy = potential_j(peak)
        if not isfinite(energy):
            break
        error = abs(target_energy - energy)
        if error < best_error:
            best_q, best_peak, best_energy, best_error = q, peak, energy, error
    saturated = best_q >= Q_MAX - 0.5 * Q_STEP and best_energy < target_energy
    result.update(q=best_q, predicted_peak=best_peak, predicted_energy=best_energy,
                  saturated=saturated, q_support=0 if best_q == 0 else 1 if best_q < 0.454 else 2)
    if not 6180 <= vbat_mv <= 8100:
        result.update(output=False, reason="INVALID_BATTERY_GUARD")
    else:
        result.update(output=best_q > 0, reason="VALID")
    return result


def source_safety_checks() -> None:
    root = Path(__file__).resolve().parents[1]
    runner = (root / "src" / "experiment_runner.cpp").read_text(encoding="utf-8")
    web = (root / "src" / "web_ui.cpp").read_text(encoding="utf-8")
    logger = (root / "src" / "psram_logger.cpp").read_text(encoding="utf-8")
    start = runner.index("bool ExperimentRunner::startQIdentCapture")
    legacy_return = runner.index('return false;', start)
    legacy_enable = runner.index('q_ident_mode_ = true;', start)
    assert legacy_return < legacy_enable, "Q_IDENT must fail before it can enable its old path"
    assert '"/start-q-ident"' not in web, "the frozen Q_IDENT route must not be registered"
    assert '"/start-energy-control-v0"' in web
    assert "energy_control_v0_pulse_live" in runner
    assert "void ExperimentRunner::beginPulse" in runner and 'status_.last_error = "motor_off_only";' in runner
    assert "energy_control_v0_events" in logger


def main() -> None:
    # P1 geometry is finite at 0, 2 and its CAD outer boundary, but not beyond.
    assert abs(potential_j(0.0)) < 1e-8
    assert potential_j(2.0) > 0.0
    assert not isfinite(potential_j(18.0))

    # High free prediction needs no braking/output; direction never flips from side.
    coast = decide(25.0, 1)
    assert coast["reason"] == "VALID_PASSIVE_NO_OUTPUT" and coast["q"] == 0.0
    assert coast["direction"] == -1

    # A small required Q is retained even though it is below Q1's identification
    # support; the support class is diagnostic, not an output gate.
    augmentation = decide(18.0, 1)
    assert augmentation["reason"] == "VALID" and 0.0 < augmentation["q"] <= Q_MAX
    assert augmentation["direction"] == -1
    assert augmentation["q_support"] in (1, 2)

    # Unsatisfied large energy demand saturates only at the explicit 0.900 bound.
    saturated = decide(4.0, -1)
    assert saturated["q"] == Q_MAX and saturated["saturated"] is True
    assert saturated["direction"] == 1

    # Automatic battery guard remains fail-closed; no manual checking is assumed.
    battery = decide(18.0, 1, 6100)
    assert battery["reason"] == "INVALID_BATTERY_GUARD" and battery["output"] is False
    source_safety_checks()
    print("Energy Control V0 offline replay/safety tests: PASS")


if __name__ == "__main__":
    main()
