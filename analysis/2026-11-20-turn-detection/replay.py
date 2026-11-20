"""Replay confirmed-peak gait decisions with only camera observations already delivered."""
import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
import csv
import json
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from score_observed_transfers import measurement_epoch

FIELDS = ("start_ms mid_ms end_ms samples left_deg right_deg left_swing_deg right_swing_deg "
          "yaw_delta_deg pair_yaw_delta_deg weak_side turn_sign streak reason").split()


def trial_id(folder):
    return str(json.loads((folder / "header.json").read_text(encoding="utf8"))["run_start_us"])


def unique_times(rows, key):
    return sorted({int(row[key]) for row in rows if row[key] and int(row[key]) > 0})


def measured_swing_phase(meta):
    # Retrospective check of which foot advances after a ZERO. This is an
    # outcome measurement and is never supplied to the online observer.
    peaks = meta["energy_control_autonomous_peak_events"]
    zeros = {z["previous_peak_time_ms"]: z["zero_cross_time_ms"] for z in meta["energy_control_autonomous_zero_cross_events"]
             if z["event_kind"] == "energy_control_zero_cross"}
    feet = [r for r in meta["foot_frames"] if r["state_id"] == 3 and
            all(r[k] for k in ("timestamp_valid", "frame_valid", "zero_ready", "left_valid", "right_valid"))]
    totals, aligned = Counter(), Counter()
    for a, b in zip(peaks, peaks[1:]):
        if a["peak_time_ms"] not in zeros or b["physical_peak_side"] == a["physical_peak_side"]:
            continue
        begin, end = zeros[a["peak_time_ms"]], b["peak_time_ms"] + 25
        data = [r for r in feet if begin * 1000 <= r["measurement_time_us"] <= end * 1000]
        if len(data) < 3:
            continue
        def advance(key):
            values = [r[key] for r in data]
            return statistics.quantiles(values, n=10, method="inclusive")[-1] - values[0]
        side = b["physical_peak_side"]
        totals[side] += 1
        if side * (advance("left_deg") - advance("right_deg")) > 0:
            aligned[side] += 1
    return dict(totals), dict(aligned)


def peak_stamp(peak, start_ms, speed, current):
    clock_bin = (start_ms + peak["peak_telemetry_capture_time_ms"]) * 1000
    candidates = []
    for key, clocks in (("speed", speed), ("current", current)):
        if not peak["peak_" + key + "_valid"]:
            continue
        age = peak["peak_" + key + "_age_us"]
        # micros() precedes the millis() read during the telemetry snapshot.
        first = bisect_left(clocks, clock_bin - 20 - age)
        last = bisect_left(clocks, clock_bin + 1000 - age)
        candidates.append({c + age for c in clocks[first:last]})
    if candidates and len(shared := set.intersection(*candidates)) == 1:
        return shared.pop(), True
    return clock_bin, False


def observations(folder):
    def load(name):
        return json.loads((folder / name).read_text(encoding="utf-8"))

    meta, header = load("metadata.json"), load("header.json")
    with (folder / "timeseries.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["state_id"] == "3" and r["sync_event_id"] != "2"]
    log_clock = [header["run_start_us"] + round(float(r["log_time_s"]) * 1000000) for r in rows]
    if sorted(log_clock) != log_clock:
        raise ValueError("Logged samples are out of order")
    origins = Counter(t // 1000 - int(r["t_test_ms"]) for t, r in zip(log_clock, rows))
    start_ms, count = origins.most_common(1)[0]
    if count < 0.9 * len(rows):
        raise ValueError("Controller millisecond epoch ambiguous")
    epoch = measurement_epoch(meta)
    periodic = meta["foot_periodic"]
    if epoch is None or not periodic["samples"] or len(periodic["samples"]) != periodic["count"]:
        raise ValueError("Incomplete foot record")
    frames = {f["sequence"]: f for f in meta["foot_frames"] if f["state_id"] == 3}
    events = []
    for n, (_, publish, lseq, rseq, lest, rest) in enumerate(periodic["samples"], 1):
        timestamp = epoch + publish
        fields = ["P", n, epoch, timestamp]
        for side, seq, estimate in (("left", lseq, lest), ("right", rseq, rest)):
            if not seq:
                if estimate is not None:
                    raise ValueError("An estimate has no source image")
                fields.extend((0, 0, "nan", "nan"))
                continue
            frame = frames[seq]
            if not (frame["timestamp_valid"] and frame[side + "_valid"] and frame[side + "_control_range_ok"]
                    and 0 < frame["control_observed_us"] <= timestamp):
                raise ValueError("A published packet cannot be traced to a received valid measurement")
            fields.extend((seq, frame["frame_us"], frame[side + "_deg"], estimate if estimate is not None else "nan"))
        events.append((timestamp, 0, " ".join(map(str, fields))))
    speed = unique_times(rows, "roller_speed_sample_time_us")
    current = unique_times(rows, "roller_current_sample_time_us")
    peaks = []
    for p in meta["energy_control_autonomous_peak_events"]:
        clock, exact = peak_stamp(p, start_ms, speed, current)
        peaks.append((p, clock, exact))
    return events, rows, log_clock, peaks, measured_swing_phase(meta)


def replay(exe, inputs, *, upper=False, next_yaw=False):
    packets, rows, log_clock, peaks, _ = inputs
    events = list(packets)
    lag = []
    for p, clock, exact in peaks:
        if not exact and upper:
            clock += 999
        # The onboard heading is read before this tick's gyro integration. Use
        # the last row already logged for the causal result. The optional next
        # row is a sensitivity check only and never used for the main result.
        index = bisect_right(log_clock, clock) - 1
        if index < 0 or index + int(next_yaw) >= len(rows):
            raise ValueError("No logged heading near accepted peak")
        index += int(next_yaw)
        row = rows[index]
        heading = row["gyro_heading_deg"] if row["gyro_heading_valid"] == "1" else "nan"
        lag.append(abs(clock - log_clock[index]))
        known = p["peak_telemetry_capture_time_ms"]
        events.append((clock, 1, f"K {p['physical_peak_side']} {known} {heading}"))
    result = subprocess.check_output([str(exe)], text=True,
                                     input="".join(line + "\n" for _, _, line in sorted(events)))
    cycles = []
    for line in result.splitlines():
        values = line.split()
        if len(values) != len(FIELDS):
            raise ValueError(f"Malformed decision: {line}")
        cycles.append(dict(zip(FIELDS, values)))
    return cycles, lag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted", type=Path, nargs="+", help="Directories from convert_rwlog_to_csv.py")
    parser.add_argument("--out", type=Path, help="Store one row per closed cycle")
    args = parser.parse_args()
    out = []
    with tempfile.TemporaryDirectory() as work:
        exe = Path(work) / "turn-replay.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(ROOT / "src"), str(HERE / "replay.cpp"),
                        "-o", str(exe)], check=True)
        for folder in args.converted:
            inputs = observations(folder)
            cycles, lag = replay(exe, inputs)
            nominal = [(v["end_ms"], v["reason"], v["weak_side"], v["turn_sign"]) for v in cycles]
            differences = []
            for opts in ({"upper": True}, {"next_yaw": True}):
                alt, _ = replay(exe, inputs, **opts)
                choices = [(v["end_ms"], v["reason"], v["weak_side"], v["turn_sign"]) for v in alt]
                if len(choices) != len(nominal):
                    raise ValueError("Sensitivity replay changed cycle count")
                differences.append(sum(a != b for a, b in zip(nominal, choices)))
            peaks = inputs[3]
            offsets = [p["peak_telemetry_capture_time_ms"] - p["peak_time_ms"] for p, _, _ in peaks]
            label = trial_id(folder)
            print(label, "count", len(cycles), "reasons", dict(Counter(v["reason"] for v in cycles)),
                  "confirmed", [(v["end_ms"], v["weak_side"], v["turn_sign"]) for v in cycles if v["reason"] == "6"],
                  "capture_us_resolved", sum(exact for _, _, exact in peaks), "/", len(peaks),
                  "confirm_delay_ms", (min(offsets), statistics.median(offsets), max(offsets)),
                  "max_previous_heading_age_us", max(lag), "different_results_bin_heading", differences,
                  "measured_swing_phase_total_matched", inputs[4])
            out.extend(dict(trial=label, **v) for v in cycles)
    if args.out:
        with args.out.open("w", encoding="utf8", newline="") as f:
            writer = csv.DictWriter(f, ["trial", *FIELDS])
            writer.writeheader()
            writer.writerows(out)


if __name__ == "__main__":
    main()
