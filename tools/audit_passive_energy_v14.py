"""Fit the v14 low-rate passive-arrival surrogate without using held-out trials.

This is an audit of conditional arrival energy U(next peak), not identification
of physical body inertia. It deliberately uses only no-new-command coast rows.
Run from the repository root: python tools/audit_passive_energy_v14.py
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import Counter
from pathlib import Path

TRAIN = ("137267573", "267557373", "141282685", "611622951")
HELD = ("97855607", "241508524")
ROOT = Path(__file__).resolve().parents[1]


def potential_j(angle_deg: float) -> float:
    """Same nominal, zero-CG-shift rocker reference as lateral_energy::potential."""
    mass, gravity, height, radius, inner_x = .1997, 9.80665, .120, .150, .005
    a = abs(angle_deg) * math.pi / 180
    center = math.sqrt(radius * radius - inner_x * inner_x)
    if a <= math.asin(inner_x / radius):
        dh = -2 * height * math.sin(a / 2) ** 2 + inner_x * math.sin(a)
    else:
        dh = (radius - center) + 2 * (center - height) * math.sin(a / 2) ** 2
    return mass * gravity * dh


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def rows_for_trial(trial: str) -> list[dict]:
    matches = list((ROOT / "analysis").glob(f"*-walking/{trial}/energy_control_autonomous_zero_cross_events.csv"))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one event CSV for {trial}, found {len(matches)}")
    events = read_csv(matches[0])
    peaks = read_csv(matches[0].with_name("energy_control_autonomous_peak_events.csv"))
    peaks.sort(key=lambda p: float(p["peak_time_ms"]))
    times = [float(p["peak_time_ms"]) for p in peaks]
    pulses = sorted(float(e["pulse_start_ms"]) for e in events
                    if e["output_executed"] == "True" and e["pulse_start_ms"])
    selected = []
    for e in events:
        if not (e["work_control_branch"] == "15" and e["output_executed"] == "False"
                and e["valid"] == "True" and e["event_kind"] == "energy_control_zero_cross"):
            continue
        t = float(e["zero_cross_time_ms"])
        side = int(e["physical_next_peak_side"])
        i = bisect.bisect_right(times, t)
        if i == len(peaks):
            continue
        p = peaks[i]  # First accepted chronological peak, not next matching-side peak.
        peak_t = times[i]
        if int(p["physical_peak_side"]) != side or not 125 <= peak_t - t <= 600:
            continue
        pulse_i = bisect.bisect_left(pulses, t)
        if pulse_i < len(pulses) and pulses[pulse_i] < peak_t:
            continue
        rate = float(e["zero_cross_abs_rate_dps"])
        peak = float(p["peak_amplitude_deg"])
        selected.append(dict(trial=trial, split="train" if trial in TRAIN else "held",
                             event_index=int(e["event_index"]), side=side, time_ms=t,
                             rate_dps=rate, omega2=(rate * math.pi / 180) ** 2,
                             next_peak_ms=peak_t, next_peak_deg=peak,
                             observed_mj=1000 * potential_j(peak)))
    return selected


def fit(rows: list[dict]) -> dict:
    n = len(rows)
    if n < 2:
        raise RuntimeError("At least two training rows required")
    sx = sum(r["omega2"] for r in rows)
    sy = sum(r["observed_mj"] for r in rows)
    sxx = sum(r["omega2"] ** 2 for r in rows)
    sxy = sum(r["omega2"] * r["observed_mj"] for r in rows)
    b = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    return dict(offset_mj=(sy - b * sx) / n, omega2_mj=b)


def score(rows: list[dict], model: dict) -> dict:
    errors = [r["observed_mj"] - model["offset_mj"] - model["omega2_mj"] * r["omega2"] for r in rows]
    if not errors:
        return dict(n=0)
    return dict(n=len(rows), rate_min_dps=min(r["rate_dps"] for r in rows),
                rate_max_dps=max(r["rate_dps"] for r in rows),
                peak_min_deg=min(r["next_peak_deg"] for r in rows),
                peak_max_deg=max(r["next_peak_deg"] for r in rows),
                rmse_mj=math.sqrt(sum(e * e for e in errors) / len(errors)),
                maximum_abs_error_mj=max(abs(e) for e in errors),
                bias_actual_minus_predicted_mj=sum(errors) / len(errors))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "analysis/2026-11-13-walking/v14_passive_energy_audit")
    args = p.parse_args()
    rows = [row for trial in TRAIN + HELD for row in rows_for_trial(trial)]
    report = dict(training_trials=TRAIN, heldout_trials=HELD,
                  heldout_scope="excluded from final coefficient fit; all six trials had already been explored, so this is not untouched or blinded model-selection validation",
                  selection="branch15 valid normal ZERO coast; first chronological accepted same-side peak 125..600ms later; no executed pulse from coast through next peak",
                  equation="E_arrival[mJ] = offset_mj + omega2_mj * omega[rad/s]^2",
                  observed="nominal zero-CG-shift U(next peak), not total system energy",
                  trial_counts=dict(Counter(r["trial"] for r in rows)), models={})
    for side in (1, -1):
        subset = [r for r in rows if r["side"] == side]
        training = [r for r in subset if r["split"] == "train"]
        model = fit(training)
        report["models"][str(side)] = dict(**model, training=score(training, model),
            heldout=score([r for r in subset if r["split"] == "held"], model),
            heldout_by_trial={trial: score([r for r in subset if r["trial"] == trial], model) for trial in HELD})
        for r in subset:
            r["predicted_mj"] = model["offset_mj"] + model["omega2_mj"] * r["omega2"]
            r["residual_mj"] = r["observed_mj"] - r["predicted_mj"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    with args.output.with_suffix(".csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# v14 passive-arrival energy audit", "", "Coefficient fitting uses only the first four trials; the two latest trials are excluded from that fit. All six trials were already inspected in an exploratory audit, so this comparison is not untouched or blinded model-selection validation.", "",
             "`E_arrival[mJ] = offset + coefficient * omega[rad/s]^2`. Observed energy is the nominal, zero-CG-shift rocker `U(next peak)`.", "",
             "| Side | Offset mJ | Coefficient mJ/(rad/s)^2 | Train n / rate dps | Train RMSE mJ | Held n / rate dps | Held RMSE mJ | Held max error mJ |",
             "|---|---:|---:|---|---:|---|---:|---:|"]
    for side in (1, -1):
        m = report["models"][str(side)]
        tr, he = m["training"], m["heldout"]
        lines.append(f"| {side:+d} | {m['offset_mj']:.12f} | {m['omega2_mj']:.12f} | {tr['n']} / {tr['rate_min_dps']:.4f}..{tr['rate_max_dps']:.4f} | {tr['rmse_mj']:.8f} | {he['n']} / {he['rate_min_dps']:.4f}..{he['rate_max_dps']:.4f} | {he['rmse_mj']:.8f} | {he['maximum_abs_error_mj']:.8f} |")
    lines += ["", "Training trials: " + ", ".join(TRAIN) + ". Held-out trials: " + ", ".join(HELD) + ".", "",
              "Selection: " + report["selection"] + ".", "",
              "This fit is conditional on the old controller selecting coast. No new command does not eliminate stored wheel/foot energy exchange. Its positive intercept is not static kinetic energy or independently measured body inertia. The passive-arrival fit already includes losses; do not subtract the same loss again. Only arrival prediction is checked here, not actuator work calibration or new closed-loop walking.", "",
              "The low-rate data do not identify behavior near zero or at high rates. Any connector to a legacy model needs continuous, monotone energy versus rate and explicit extrapolation labels. A naive 35..45 dps blend is nonmonotone for these fits. Training/held-out membership must remain fixed in future replays.", "",
              "Reproduce: `python tools/audit_passive_energy_v14.py`. Detailed model metrics and the 65 selected rows are saved beside this report as JSON/CSV.", ""]
    args.output.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
