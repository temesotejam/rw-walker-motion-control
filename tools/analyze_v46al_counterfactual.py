#!/usr/bin/env python3
"""Replay the historical V46al previous-peak correction against a stable V46ak capture manifest.

This is deliberately OFFLINE ONLY. It never changes firmware output, Q, pulse width,
RWLOG format, or download transport.
"""
import argparse
import json
import math
from pathlib import Path

COEFFICIENTS = {
    1: {
        "c": 0.591392151,
        "k": -0.442636343,
        "support_min": 7.19424,
        "support_max": 9.34474,
        "gain": 0.29032,
    },
    -1: {
        "c": -0.157912422,
        "k": 0.585367534,
        "support_min": 6.95706,
        "support_max": 8.75588,
        "gain": 0.25455,
    },
}
ENABLE_AFTER_MS = 10000
MAX_ABS_CORRECTION_DEG = 0.70


def stats(values):
    values = list(values)
    if not values:
        return None
    return {
        "n": len(values),
        "mean": sum(values) / len(values),
        "mae": sum(abs(v) for v in values) / len(values),
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
        "min": min(values),
        "max": max(values),
    }


def fmt(s):
    if not s:
        return "n=0"
    return (
        f"n={s['n']} mean={s['mean']:+.4f} "
        f"MAE={s['mae']:.4f} RMSE={s['rmse']:.4f} "
        f"range=[{s['min']:+.4f},{s['max']:+.4f}]"
    )


def load_metadata(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    try:
        return data["rwlog"]["inspection"]["metadata"]
    except KeyError as exc:
        raise SystemExit("manifest does not contain rwlog.inspection.metadata") from exc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest", type=Path)
    args = ap.parse_args()

    m = load_metadata(args.manifest)
    zero = [
        e for e in m.get("energy_control_autonomous_zero_cross_events", [])
        if e.get("event_kind") == "energy_control_zero_cross"
    ]
    peaks = m.get("energy_control_autonomous_peak_events", [])

    rows = []
    for event in zero:
        t_ms = event.get("zero_cross_time_ms")
        side = event.get("physical_next_peak_side")
        prev = event.get("previous_peak_amplitude_deg")
        predicted = event.get("predicted_next_peak_amplitude_deg")
        if t_ms is None or side not in COEFFICIENTS or predicted is None:
            continue

        next_peak = next(
            (
                p for p in peaks
                if p.get("peak_time_ms", -1) > t_ms
                and p.get("physical_peak_side") == side
            ),
            None,
        )
        if not next_peak:
            continue

        actual = next_peak.get("peak_amplitude_deg")
        if actual is None:
            continue

        cfg = COEFFICIENTS[side]
        applies = (
            t_ms >= ENABLE_AFTER_MS
            and prev is not None
            and math.isfinite(prev)
            and cfg["support_min"] <= prev <= cfg["support_max"]
        )

        correction = 0.0
        if applies:
            correction = cfg["c"] + cfg["k"] * (prev - 8.0)
            correction = max(-MAX_ABS_CORRECTION_DEG,
                             min(MAX_ABS_CORRECTION_DEG, correction))

        residual = actual - predicted
        corrected_residual_same_q = actual - (predicted + correction)
        gain = event.get("q1_gain_deg_per_mA_s") or cfg["gain"]
        implied_delta_q = (-correction / gain) if applies and gain else 0.0

        rows.append({
            "t_ms": t_ms,
            "side": side,
            "prev": prev,
            "actual": actual,
            "predicted": predicted,
            "residual": residual,
            "applies": applies,
            "correction": correction,
            "corrected_residual_same_q": corrected_residual_same_q,
            "implied_delta_q": implied_delta_q,
            "integral_side_mA_s": event.get("integral_side_mA_s"),
            "q_command_mA_s": event.get("q_command_mA_s"),
            "pulse_width_ms": event.get("pulse_width_ms"),
        })

    eval_rows = [r for r in rows if r["t_ms"] >= ENABLE_AFTER_MS]
    applied = [r for r in rows if r["applies"]]

    print("V46al previous-peak counterfactual audit")
    print(f"manifest: {args.manifest}")
    print(f"10-30 s events: {len(eval_rows)}")
    print(f"V46al support/applied events: {len(applied)}")
    print("stable residual:          ", fmt(stats(r["residual"] for r in eval_rows)))
    print("applied-only residual:    ", fmt(stats(r["residual"] for r in applied)))
    print("same-Q corrected residual:", fmt(stats(r["corrected_residual_same_q"] for r in applied)))
    print("correction deg:           ", fmt(stats(r["correction"] for r in applied)))
    print("implied delta-Q mA*s:     ", fmt(stats(r["implied_delta_q"] for r in applied)))

    for side in (1, -1):
        sr = [r for r in applied if r["side"] == side]
        print(f"\nside {side:+d}")
        print("  stable residual:   ", fmt(stats(r["residual"] for r in sr)))
        print("  correction deg:    ", fmt(stats(r["correction"] for r in sr)))
        print("  corrected residual:", fmt(stats(r["corrected_residual_same_q"] for r in sr)))
        print("  implied delta-Q:   ", fmt(stats(r["implied_delta_q"] for r in sr)))
        ints = [r["integral_side_mA_s"] for r in sr if r["integral_side_mA_s"] is not None]
        if ints:
            print("  existing integral:", fmt(stats(ints)))

    print("\nLargest implied Q changes")
    for r in sorted(applied, key=lambda x: abs(x["implied_delta_q"]), reverse=True)[:12]:
        print(
            f"  t={r['t_ms']:5d} ms side={r['side']:+d} "
            f"Aprev={r['prev']:.3f} corr={r['correction']:+.3f} deg "
            f"dQ={r['implied_delta_q']:+.3f} mA*s "
            f"Qstable={r['q_command_mA_s']:.3f} pulse={r['pulse_width_ms']} ms"
        )


if __name__ == "__main__":
    main()
