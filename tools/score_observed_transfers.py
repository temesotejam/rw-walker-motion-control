"""Compare v39 predictions with independently timed support transfers in a converted trial."""
import argparse
import csv
import json
import math
from bisect import bisect_left, bisect_right
from pathlib import Path


def numeric(value):
    if value in (None, ""):
        return None
    n = float(value)
    if not math.isfinite(n):
        raise ValueError("A recorded value is non-finite")
    return n


def measurement_epoch(metadata):
    origins = {r["frame_us"] - r["measurement_time_us"] for r in metadata["foot_frames"]
               if r.get("state_id") == 3 and r.get("timestamp_valid") and r.get("frame_us", 0) > 0}
    return origins.pop() if len(origins) == 1 else None


def gyro_samples(metadata, samples, epoch):
    # t_test_ms describes a control tick; the gyro sample has its own clock.
    if epoch is None:
        return [], [], []
    times, rates, angles = [], [], []
    for row in samples:
        if int(row["state_id"]) != 3 or not row["body_rate_sample_time_us"]:
            continue
        stamp = int(row["body_rate_sample_time_us"])
        if not stamp:
            continue
        elapsed = (stamp - epoch) & 0xffffffff
        if elapsed > 600_000_000:
            continue
        t = elapsed / 1000
        gyro, bias = numeric(row["gy_dps"]), numeric(row["mekf_bias_y_dps"])
        if gyro is not None and bias is not None and (not times or t > times[-1]):
            times.append(t)
            rates.append((gyro-bias)*metadata["mekf_gyro_y_scale"])
            angles.append(numeric(row["pitch_mekf_detector_relative_deg"]))
    return times, rates, angles


FIELDS = ("start_ms", "end_ms", "receiving_foot", "body_direction", "opposite_foot_swung",
          "forward_mm", "peak_ms", "decision_ms", "reason", "direction_agrees", "receiving_foot_agrees",
          "predicted_transfer_ms", "timing_error_to_interval_ms", "coast_margin_j",
          "candidate_margin_j", "candidate_delay_ms", "candidate_width_ms", "rate_sample_ms",
          "observed_directional_rate_dps", "exit_observation", "exit_sample_ms", "exit_rate_dps",
          "exit_carry_5dps", "status")


def recorded_exit(times, rates, angles, g, end_ms, direction):
    # The reference is the first observed pose after the annotation window.
    # Censor gaps and reversals; do not score a later attempt as this transfer.
    if angles[g] is None:
        return "missing_angle", None
    for j in range(g + 1, len(times)):
        if times[j] >= end_ms:
            return "next_peak", None
        if times[j] - times[j-1] > 10 or angles[j] is None:
            return "observation_gap", None
        if direction*(angles[j] - angles[g]) >= 1:
            return "reached", j
        if (direction*rates[j] < 0 and direction*rates[j-1] < 0):
            return "reversed", None
    return "end_of_data", None


def score(metadata, plans, samples, labels):
    peak_times = sorted({p["peak_time_ms"] for p in metadata["energy_control_autonomous_peak_events"]})
    epoch = measurement_epoch(metadata)
    times, rates, angles = gyro_samples(metadata, samples, epoch)
    snapshots = []
    if epoch is not None:
        for p in plans:
            # ESP32 micros() is the low 32 bits of the same microsecond clock.
            # Unsigned subtraction also covers trials crossing a micros() wrap.
            us = (int(p["capture_us32"]) - epoch) & 0xffffffff
            if us > 600_000_000:
                raise ValueError("Decision timestamp outside the measurement clock")
            snapshots.append((us, p))
    previous_end = -1
    for label in labels:
        start, end = numeric(label["start_ms"]), numeric(label["end_ms"])
        if start is None or end is None or not 0 <= start <= end or start <= previous_end:
            raise ValueError("Label ordered, non-overlapping transfer intervals in measurement milliseconds")
        previous_end = end
        foot, swung = label["receiving_foot"], label.get("opposite_foot_swung", "")
        direction = int(label["body_direction"])
        if foot not in ("L", "R") or direction not in (-1, 1) or swung not in ("", "0", "1"):
            raise ValueError("Use L/R, physical direction -1/+1, and swing 0/1 or blank")
        out = dict(start_ms=start, end_ms=end, receiving_foot=foot, body_direction=direction,
                   opposite_foot_swung=swung, forward_mm=numeric(label.get("forward_mm")))
        if epoch is None:
            yield dict(out, status="unknown_measurement_clock")
            continue
        # First independent, fresh gyro sample at or after the latest possible transfer.
        g = bisect_left(times, end)
        if g < len(times) and times[g] <= end + 10:
            out.update(rate_sample_ms=times[g], observed_directional_rate_dps=rates[g]*direction)
        i = bisect_right(peak_times, start)
        if not i or (i < len(peak_times) and peak_times[i] <= end):
            yield dict(out, status="ambiguous_half_cycle")
            continue
        origin = peak_times[i-1]
        out["peak_ms"] = origin
        if "rate_sample_ms" in out:
            until = peak_times[i] if i < len(peak_times) else math.inf
            if times[g] < until:
                out["exit_observation"], j = recorded_exit(times, rates, angles, g, until, direction)
                if j is not None:
                    out.update(exit_sample_ms=times[j], exit_rate_dps=rates[j]*direction,
                               exit_carry_5dps=rates[j]*direction >= 5)
        # Keep the latest snapshot, even when its answer invalidates an older
        # forecast. Worker completion must precede the entire label window.
        before = [(us, p) for us, p in snapshots if int(p["peak_ms"]) == origin and
                  us + int(p["ready_after_us"]) < start * 1000]
        if not before:
            yield dict(out, status="no_prior_recorded_decision")
            continue
        us, p = max(before, key=lambda item: (item[0], int(item[1]["packet"])))
        t = us / 1000
        side = int(p["side"])
        out.update(decision_ms=t, reason=p["reason"], direction_agrees=side == direction,
                   receiving_foot_agrees=(side == 1 and foot == "R") or (side == -1 and foot == "L"),
                   coast_margin_j=numeric(p["coast_margin_j"]), candidate_margin_j=numeric(p["margin_j"]),
                   candidate_delay_ms=p["delay_ms"], candidate_width_ms=p["width_ms"])
        dt = numeric(p["transfer_ms"])
        if dt is None or dt <= 0:
            yield dict(out, status="no_forecast")
            continue
        predicted = t + dt
        error = predicted - min(end, max(start, predicted))
        status = "matched"
        if not out["receiving_foot_agrees"]:
            status = "wrong_foot"
        if not out["direction_agrees"]:
            status = "wrong_direction"
        yield dict(out, status=status, predicted_transfer_ms=predicted, timing_error_to_interval_ms=error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, help="Directory produced by convert_rwlog_to_csv.py")
    parser.add_argument("labels", type=Path, help="Independent synchronized annotations, CSV")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    meta = json.loads((args.trial / "metadata.json").read_text(encoding="utf8"))
    if "support_transfer_shadow" not in meta:
        parser.error("This trial has no v39 support-transfer prediction archive")
    with (args.trial / "support_transfer_shadow.csv").open(encoding="utf-8-sig", newline="") as pf, \
            (args.trial / "timeseries.csv").open(encoding="utf-8-sig", newline="") as sf, \
            args.labels.open(encoding="utf-8-sig", newline="") as lf:
        rows = list(score(meta, list(csv.DictReader(pf)), csv.DictReader(sf), csv.DictReader(lf)))
    with args.out.open("w", encoding="utf8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} labeled transfers; {sum(r['status'] == 'matched' for r in rows)} with a timed forecast.")


if __name__ == "__main__":
    main()
