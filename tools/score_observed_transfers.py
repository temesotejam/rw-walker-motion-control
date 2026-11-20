"""Compare recorded support forecasts with annotated transfers or measured body motion."""
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


def capture_us(p, epoch):
    elapsed = (int(p["capture_us32"]) - epoch) & 0xffffffff
    if elapsed > 600_000_000:
        raise ValueError("Decision timestamp outside the measurement clock")
    return elapsed


FIELDS = ("start_ms", "end_ms", "receiving_foot", "body_direction", "opposite_foot_swung",
          "forward_mm", "peak_ms", "decision_ms", "reason", "direction_agrees", "receiving_foot_agrees",
          "predicted_transfer_ms", "timing_error_to_interval_ms", "coast_margin_j",
          "candidate_margin_j", "candidate_delay_ms", "candidate_width_ms", "rate_sample_ms",
          "observed_directional_rate_dps", "exit_observation", "exit_sample_ms", "exit_rate_dps",
          "exit_carry_5dps", "status")

TERMINAL_FIELDS = ("peak_ms", "packet", "side", "anchor", "reason", "coast_frames", "decision_ms", "ready_ms",
                   "terminal_deg", "predicted_terminal_ms", "status", "observation_end_ms",
                   "observed_rate_dps", "carry_5dps", "recorded_command")


def terminal_observation(times, rates, angles, start, ready, until, direction, target):
    """Follow one absolute target until the next peak or the first reversal."""
    if until <= start:
        return "next_peak", start, None
    j = bisect_right(times, start) - 1
    if j < 0 or start - times[j] > 10:
        j += 1
        if j == len(times) or times[j] > start + 10 or times[j] >= until:
            return "missing_initial_observation", start, None
    if angles[j] is None:
        return "missing_angle", start, None
    if direction * (angles[j] - target) >= 0:
        return "already_at_first_observation", max(start, times[j]), None
    for k in range(j + 1, len(times)):
        if times[k] - times[k-1] > 10 or angles[k] is None:
            return "observation_gap", max(start, times[k-1]), None
        if direction * (angles[k] - target) >= 0:
            # Interpolate the first crossing bracket, on the IMU clock.
            f = (target - angles[k-1]) / (angles[k] - angles[k-1])
            crossed = times[k-1] + f * (times[k] - times[k-1])
            if crossed < until:
                if crossed <= ready:
                    return "already_reached_at_ready", max(start, crossed), None
                rate = direction * (rates[k-1] + f * (rates[k] - rates[k-1]))
                return "reached", crossed, rate
        if times[k] >= until:
            return "next_peak", until, None
        if times[k-1] >= start and direction*rates[k-1] < 0 and direction*rates[k] < 0:
            return ("reversed_at_ready" if times[k] <= ready else "reversed"), times[k], None
    return "end_of_data", max(start, times[-1]), None


def command_overlap(archive, start, end):
    # The walking branch records software command edges with millisecond resolution.
    # A missing command is informative only with a complete archive.
    records = archive.get("records", [])
    uncertain = False
    for command in records:
        if not command["executed"]:
            continue
        on, off = command["output_start_ms"], command["output_end_ms"]
        if on > end+1 or (off and off < start-1):
            continue
        if start+1 < on < end-1 or (on < end-1 and off and off > start+1):
            return "overlap"
        uncertain = True
    if uncertain:
        return "uncertain_edge"
    if (archive.get("archive_available") is True and archive.get("truncated") is False
            and archive.get("total") == len(records)):
        return "no_recorded_command"
    return "unknown_archive"


def score_terminals(metadata, plans, samples):
    # This endpoint belongs to a decision, even when no contact was labeled.
    epoch = measurement_epoch(metadata)
    times, rates, angles = gyro_samples(metadata, samples, epoch)
    peaks = sorted({r["peak_time_ms"] for r in metadata["energy_control_autonomous_peak_events"]})
    for p in plans:
        if str(p.get("phase", "")) != "2":
            continue
        origin, side = int(p["peak_ms"]), int(p["side"])
        target = numeric(p["terminal_deg"])
        out = dict(peak_ms=origin, packet=p["packet"], side=side, anchor=p["anchor"],
                   reason=p["reason"], coast_frames=p.get("coast_frames"), terminal_deg=target)
        if epoch is None:
            yield dict(out, status="unknown_measurement_clock")
            continue
        start = capture_us(p, epoch) / 1000
        ready = start + int(p["ready_after_us"]) / 1000
        out.update(decision_ms=start, ready_ms=ready)
        nominal = numeric(p["terminal_ms"])
        if nominal is not None:
            out["predicted_terminal_ms"] = start + nominal
        i = bisect_right(peaks, origin)
        if not i or peaks[i-1] != origin or start < origin:
            yield dict(out, status="unknown_source_peak")
            continue
        if target is None or side not in (-1, 1):
            yield dict(out, status="no_terminal_forecast")
            continue
        until = peaks[i] if i < len(peaks) else math.inf
        status, end, rate = terminal_observation(times, rates, angles, start, ready, until, side, target)
        out.update(status=status, observation_end_ms=end, observed_rate_dps=rate,
                   recorded_command=command_overlap(metadata.get("walking_energy", {}), start, end))
        if rate is not None or status == "reversed":
            out["carry_5dps"] = rate is not None and rate >= 5
        yield out


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
            snapshots.append((capture_us(p, epoch), p))
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
            # An already-lower receiving sole does not date past contact.
            status = "no_prior_contact_forecast" if str(p.get("phase", "")) in ("2", "3") else "no_forecast"
            yield dict(out, status=status)
            continue
        predicted = t + dt
        error = predicted - min(end, max(start, predicted))
        status = "matched"
        if not out["receiving_foot_agrees"]:
            status = "wrong_foot"
        if not out["direction_agrees"]:
            status = "wrong_direction"
        yield dict(out, status=status, predicted_transfer_ms=predicted, timing_error_to_interval_ms=error)


def write_rows(path, fields, rows):
    with path.open("w", encoding="utf8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, help="Directory produced by convert_rwlog_to_csv.py")
    parser.add_argument("labels", type=Path, nargs="?", help="Independent synchronized annotations, CSV")
    parser.add_argument("--out", type=Path, help="Score annotated contacts")
    parser.add_argument("--terminal-out", type=Path, help="Score each receiving-side target against the IMU")
    args = parser.parse_args()
    if bool(args.labels) != bool(args.out) or not (args.out or args.terminal_out):
        parser.error("Specify labels with --out, or use --terminal-out without contact labels")
    if args.out and args.terminal_out and args.out.resolve() == args.terminal_out.resolve():
        parser.error("Choose different output files for contacts and receiving-side targets")
    meta = json.loads((args.trial / "metadata.json").read_text(encoding="utf8"))
    if "support_transfer_shadow" not in meta:
        parser.error("This trial has no v39 support-transfer prediction archive")
    if args.terminal_out and meta["support_transfer_shadow"].get("revision", 1) < 2:
        parser.error("This recording predates receiving-side terminal forecasts (revision 2)")
    with (args.trial / "support_transfer_shadow.csv").open(encoding="utf-8-sig", newline="") as pf, \
            (args.trial / "timeseries.csv").open(encoding="utf-8-sig", newline="") as sf:
        plans, samples = list(csv.DictReader(pf)), list(csv.DictReader(sf))
    if args.labels:
        with args.labels.open(encoding="utf-8-sig", newline="") as lf:
            rows = list(score(meta, plans, samples, csv.DictReader(lf)))
        write_rows(args.out, FIELDS, rows)
        print(f"{len(rows)} labeled transfers; {sum(r['status'] == 'matched' for r in rows)} with a timed forecast.")
    if args.terminal_out:
        rows = list(score_terminals(meta, plans, samples))
        write_rows(args.terminal_out, TERMINAL_FIELDS, rows)
        print(f"{len(rows)} receiving-side decisions; {sum(r['status'] == 'reached' for r in rows)} reached after readiness.")


if __name__ == "__main__":
    main()
