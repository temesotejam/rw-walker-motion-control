#!/usr/bin/env python3
"""Analyze V46 MEKF vs dynamic-beta Madgwick during motor-driven motion.

Without video this reports estimator disagreement, pulse/non-pulse behavior,
MEKF accel-rejection behavior, and timing health.

With a video-tracking CSV (time_s, angle_deg by default), two LED anchor pairs
map video time to RWLOG time and the script reports MEKF/Madgwick error metrics
against video plus a best-fit estimator delay.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median
from typing import Iterable


def finite_float(value: object) -> float | None:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    p = (len(xs) - 1) * q
    lo = int(math.floor(p))
    hi = int(math.ceil(p))
    if lo == hi:
        return xs[lo]
    a = p - lo
    return xs[lo] * (1.0 - a) + xs[hi] * a


def basic_stats(values: Iterable[float]) -> dict[str, float | int | None]:
    xs = [x for x in values if math.isfinite(x)]
    if not xs:
        return {"n": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "n": len(xs),
        "mean": sum(xs) / len(xs),
        "median": median(xs),
        "p95": percentile(xs, 0.95),
        "max": max(xs),
    }


def error_metrics(errors: Iterable[float]) -> dict[str, float | int | None]:
    e = [x for x in errors if math.isfinite(x)]
    if not e:
        return {
            "n": 0,
            "bias_deg": None,
            "mae_deg": None,
            "rmse_deg": None,
            "max_abs_deg": None,
        }
    return {
        "n": len(e),
        "bias_deg": sum(e) / len(e),
        "mae_deg": sum(abs(x) for x in e) / len(e),
        "rmse_deg": math.sqrt(sum(x * x for x in e) / len(e)),
        "max_abs_deg": max(abs(x) for x in e),
    }


def load_rwlog_csv(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        required = {
            "time_s",
            "pulse_active",
            "pitch_mekf_abs_deg",
            "pitch_madgwick_dynamic_abs_deg",
            "mekf_accel_confidence",
            "mekf_accel_used",
            "imu_update_dt_us",
            "imu_sample_age_us",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"missing V46 columns: {', '.join(sorted(missing))}")
        for r in reader:
            t = finite_float(r.get("time_s"))
            mekf = finite_float(r.get("pitch_mekf_abs_deg"))
            madg = finite_float(r.get("pitch_madgwick_dynamic_abs_deg"))
            if t is None or mekf is None or madg is None:
                continue
            rows.append({
                "t": t,
                "mekf": mekf,
                "madgwick": madg,
                "pulse": str(r.get("pulse_active", "0")).strip().lower() in {"1", "true", "yes"},
                "acc_conf": finite_float(r.get("mekf_accel_confidence")),
                "acc_used": str(r.get("mekf_accel_used", "0")).strip().lower() in {"1", "true", "yes"},
                "imu_dt_us": finite_float(r.get("imu_update_dt_us")),
                "imu_age_us": finite_float(r.get("imu_sample_age_us")),
                "motor_cmd_mA": finite_float(r.get("motor_cmd_mA")),
                "actual_current_mA": finite_float(r.get("roller_actual_current_mA")),
            })
    if not rows:
        raise SystemExit("no usable V46 samples found")
    return rows


def load_video_csv(path: Path, time_col: str, angle_col: str) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    with path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if time_col not in (reader.fieldnames or []) or angle_col not in (reader.fieldnames or []):
            raise SystemExit(f"video CSV must contain {time_col!r} and {angle_col!r}")
        for r in reader:
            t = finite_float(r.get(time_col))
            a = finite_float(r.get(angle_col))
            if t is not None and a is not None:
                out.append((t, a))
    out.sort()
    if len(out) < 2:
        raise SystemExit("video CSV needs at least two finite samples")
    return out


def interp(samples: list[tuple[float, float]], t: float) -> float | None:
    if t < samples[0][0] or t > samples[-1][0]:
        return None
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, y0 = samples[lo]
    t1, y1 = samples[hi]
    if t1 == t0:
        return y0
    alpha = (t - t0) / (t1 - t0)
    return y0 + alpha * (y1 - y0)


def compare_to_video(
    rows: list[dict[str, object]],
    video: list[tuple[float, float]],
    video_anchor1: float,
    rw_anchor1: float,
    video_anchor2: float,
    rw_anchor2: float,
    lag_window_ms: float,
    lag_step_ms: float,
) -> dict[str, object]:
    if video_anchor2 == video_anchor1:
        raise SystemExit("video anchors must have different times")
    scale = (rw_anchor2 - rw_anchor1) / (video_anchor2 - video_anchor1)
    if scale <= 0:
        raise SystemExit("anchor mapping must have positive time scale")
    offset = rw_anchor1 - scale * video_anchor1

    # Convert video samples into RWLOG time so interpolation is direct.
    video_rw = [(scale * t + offset, a) for t, a in video]

    def metrics_for(estimator: str, lag_s: float = 0.0, subset=None):
        errors: list[float] = []
        for r in rows:
            if subset is not None and not subset(r):
                continue
            t = float(r["t"])
            # Positive lag means the estimator is delayed: estimator(t) is
            # compared with the physical/video state at t-lag.
            ref = interp(video_rw, t - lag_s)
            if ref is None:
                continue
            errors.append(float(r[estimator]) - ref)
        return error_metrics(errors)

    lag_candidates: list[float] = []
    if lag_step_ms <= 0:
        lag_step_ms = 5.0
    kmax = int(round(lag_window_ms / lag_step_ms))
    for k in range(-kmax, kmax + 1):
        lag_candidates.append(k * lag_step_ms / 1000.0)

    result: dict[str, object] = {
        "time_mapping": {"rw_time_equals_scale_times_video_plus_offset": [scale, offset]},
        "estimators": {},
    }
    for est in ("mekf", "madgwick"):
        scans: list[tuple[float, float]] = []
        for lag in lag_candidates:
            m = metrics_for(est, lag)
            rmse = m["rmse_deg"]
            if isinstance(rmse, (int, float)):
                scans.append((float(rmse), lag))
        best_rmse, best_lag = min(scans) if scans else (math.nan, 0.0)
        est_result = {
            "zero_lag": metrics_for(est, 0.0),
            "best_fit_lag_ms": best_lag * 1000.0 if scans else None,
            "best_fit_lag_metrics": metrics_for(est, best_lag) if scans else error_metrics([]),
            "pulse_active": metrics_for(est, best_lag, lambda r: bool(r["pulse"])),
            "pulse_inactive": metrics_for(est, best_lag, lambda r: not bool(r["pulse"])),
            "accel_used": metrics_for(est, best_lag, lambda r: bool(r["acc_used"])),
            "accel_rejected": metrics_for(est, best_lag, lambda r: not bool(r["acc_used"])),
        }
        result["estimators"][est] = est_result
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("timeseries_csv", type=Path, help="converted V46 timeseries.csv")
    ap.add_argument("--video-csv", type=Path, help="tracked video angle CSV")
    ap.add_argument("--video-time-col", default="time_s")
    ap.add_argument("--video-angle-col", default="angle_deg")
    ap.add_argument("--video-anchor1", type=float)
    ap.add_argument("--rw-anchor1", type=float)
    ap.add_argument("--video-anchor2", type=float)
    ap.add_argument("--rw-anchor2", type=float)
    ap.add_argument("--lag-window-ms", type=float, default=200.0)
    ap.add_argument("--lag-step-ms", type=float, default=5.0)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    rows = load_rwlog_csv(args.timeseries_csv)
    disagreement = [float(r["mekf"]) - float(r["madgwick"]) for r in rows]
    pulse_rows = [r for r in rows if r["pulse"]]
    nonpulse_rows = [r for r in rows if not r["pulse"]]
    used_rows = [r for r in rows if r["acc_used"]]
    rejected_rows = [r for r in rows if not r["acc_used"]]

    result: dict[str, object] = {
        "source": str(args.timeseries_csv),
        "samples": len(rows),
        "duration_s": float(rows[-1]["t"]) - float(rows[0]["t"]),
        "mekf_minus_madgwick": {
            "all": error_metrics(disagreement),
            "pulse_active": error_metrics(float(r["mekf"]) - float(r["madgwick"]) for r in pulse_rows),
            "pulse_inactive": error_metrics(float(r["mekf"]) - float(r["madgwick"]) for r in nonpulse_rows),
            "accel_used": error_metrics(float(r["mekf"]) - float(r["madgwick"]) for r in used_rows),
            "accel_rejected": error_metrics(float(r["mekf"]) - float(r["madgwick"]) for r in rejected_rows),
        },
        "mekf_accel": {
            "used_samples": len(used_rows),
            "rejected_samples": len(rejected_rows),
            "rejected_fraction": len(rejected_rows) / len(rows),
            "confidence": basic_stats(float(r["acc_conf"]) for r in rows if r["acc_conf"] is not None),
        },
        "timing": {
            "imu_update_dt_us": basic_stats(float(r["imu_dt_us"]) for r in rows if r["imu_dt_us"] is not None),
            "imu_sample_age_us": basic_stats(float(r["imu_age_us"]) for r in rows if r["imu_age_us"] is not None),
        },
        "motor": {
            "pulse_active_samples": len(pulse_rows),
            "pulse_active_fraction": len(pulse_rows) / len(rows),
            "command_current_mA": basic_stats(abs(float(r["motor_cmd_mA"])) for r in rows if r["motor_cmd_mA"] is not None),
            "actual_current_mA": basic_stats(abs(float(r["actual_current_mA"])) for r in rows if r["actual_current_mA"] is not None),
        },
    }

    if args.video_csv:
        anchors = [args.video_anchor1, args.rw_anchor1, args.video_anchor2, args.rw_anchor2]
        if any(v is None for v in anchors):
            raise SystemExit("--video-csv requires --video-anchor1 --rw-anchor1 --video-anchor2 --rw-anchor2")
        video = load_video_csv(args.video_csv, args.video_time_col, args.video_angle_col)
        result["video_ground_truth"] = compare_to_video(
            rows,
            video,
            float(args.video_anchor1),
            float(args.rw_anchor1),
            float(args.video_anchor2),
            float(args.rw_anchor2),
            args.lag_window_ms,
            args.lag_step_ms,
        )

    out = args.out or args.timeseries_csv.with_name("v46_attitude_validation_summary.json")
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
