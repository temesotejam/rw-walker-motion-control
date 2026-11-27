"""Measure lateral body rates on the two sides of independently labelled support transfers."""
import argparse
import csv
import json
import math
import sys
from bisect import bisect_left, bisect_right
from itertools import combinations
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "support"))
from event_foot_features import CausalFootFeatures, MAX_FRAME_AGE_US
from score_observed_transfers import command_overlap, gyro_samples, measurement_epoch, numeric, write_rows


LABEL_FIELDS = ("run_start_us", "start_ms", "end_ms", "receiving_foot", "body_direction", "source")
FIELDS = (*LABEL_FIELDS, "event_kind", "note", "video_lo_ms", "video_hi_ms", "label_span_ms", "imu_status",
          "pre_ms", "post_ms", "pre_gap_ms", "post_gap_ms", "max_imu_gap_ms",
          "pre_body_deg", "post_body_deg", "pre_gyro_dps", "post_gyro_dps",
          "pre_directional_dps", "post_directional_dps", "delta_directional_dps",
          "foot_status", "foot_sequence", "foot_capture_ms", "foot_receipt_ms", "foot_age_ms",
          "left_angle_deg", "right_angle_deg", "left_rate_dps", "right_rate_dps",
          "command_overlap", "ms_since_recorded_command_end", "max_abs_measured_current_ma",
          "current_samples", "max_current_gap_ms")
MAX_IMU_GAP_MS = 10


def checked_labels(labels, run_start):
    result = []
    for index, label in enumerate(labels, 2):
        try:
            if any(label.get(key) in (None, "") for key in LABEL_FIELDS):
                raise ValueError("missing required independent label field")
            if str(label["run_start_us"]).strip() != str(run_start):
                raise ValueError("run_start_us does not match header.json")
            start, end = float(label["start_ms"]), float(label["end_ms"])
            direction = int(str(label["body_direction"]).strip())
            foot = label["receiving_foot"].strip().upper()
            source = label["source"].strip()
            if not (math.isfinite(start) and math.isfinite(end) and 0 <= start <= end):
                raise ValueError("require finite 0 <= start_ms <= end_ms")
            if foot not in ("L", "R") or direction not in (-1, 1):
                raise ValueError("receiving_foot must be L/R and body_direction must be -1/+1")
            if source not in ("load_sensor", "video_proxy"):
                raise ValueError("source must be load_sensor or video_proxy")
            result.append(dict(run_start_us=run_start, start_ms=start, end_ms=end,
                               receiving_foot=foot, body_direction=direction, source=source,
                               event_kind=(label.get("event_kind") or "unspecified").strip(),
                               note=label.get("note", ""), label_span_ms=end-start,
                               video_lo_ms=label.get("video_lo_ms"), video_hi_ms=label.get("video_hi_ms")))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"Labels row {index}: {error}") from error
    return result


def pre_foot(camera, cutoff, row):
    feet = camera.foot_at_us(camera.epoch_us + cutoff * 1000)
    receipt = feet["foot_receipt_us"]
    if receipt is None:
        return "no_received_frame"
    row.update(foot_sequence=feet["foot_sequence"], foot_receipt_ms=(receipt-camera.epoch_us)/1000)
    age = feet["foot_age_ms"]
    if age is None:
        return "invalid_frame_clock"
    row.update(foot_capture_ms=(feet["foot_frame_us"]-camera.epoch_us)/1000, foot_age_ms=age)
    if age > MAX_FRAME_AGE_US/1000:
        return "stale"
    for side in ("left", "right"):
        row[side+"_angle_deg"] = feet[side+"_deg"]
        row[side+"_rate_dps"] = feet[side+"_rate_dps"]
    if any(row.get(side+"_angle_deg") is None for side in ("left", "right")):
        return "missing_angle"
    if any(row.get(side+"_rate_dps") is None for side in ("left", "right")):
        return "missing_rate"
    return "ok"


def command_archive(meta, applied):
    # The launch kick and queued restarts can bypass walking_energy.records.
    walking = meta.get("walking_energy", {})
    records = list(walking.get("records", []))
    complete = (walking.get("archive_available") is True and walking.get("truncated") is False
                and walking.get("total") == len(records))
    events = meta.get("energy_control_autonomous_zero_cross_events", [])
    complete &= (meta.get("energy_control_autonomous_event_overflow") is False
                 and meta.get("energy_control_autonomous_zero_cross_event_total_count") == len(events))
    for event in events:
        if event.get("event_kind") == "strong_start_kick" and event.get("output_executed"):
            # The kick's event stores its planned end. Find the first applied stop
            # following the launch command. Unobserved stops keep an open edge.
            on = event["pulse_start_ms"]
            off = 0
            for i, (stamp, current) in enumerate(applied[:-1]):
                if on-1 <= stamp <= event["pulse_end_ms"] and current:
                    for next_stamp, next_current in applied[i+1:]:
                        if next_current != current:
                            if next_current == 0:
                                off = next_stamp
                            break
                    break
            records.append(dict(executed=True, output_start_ms=on, output_end_ms=off))
    restart = meta.get("walking_restart", {})
    columns = restart.get("columns", [])
    if {"start_ms", "queued", "elapsed_ms"}.issubset(columns) and "rows" in restart:
        for values in restart["rows"]:
            r = dict(zip(columns, values))
            if r.get("queued") and r.get("start_ms") is not None and r.get("elapsed_ms") is not None:
                records.append(dict(executed=True, output_start_ms=r["start_ms"],
                                    output_end_ms=r["start_ms"]+r["elapsed_ms"] if r["elapsed_ms"] else 0))
            else:
                complete = False  # Failed command writes have uncertain hardware acknowledgement.
    else:
        complete = False
    return dict(records=records, archive_available=complete, truncated=not complete, total=len(records))


def input_evidence(samples, epoch):
    active, currents, applied = [], [], {}
    for sample in samples:
        if int(sample["state_id"]) != 3:
            continue
        tick = numeric(sample.get("t_test_ms"))
        if tick is None:
            continue
        cmd = [numeric(sample.get(k)) for k in ("pulse_active", "motor_cmd_mA", "roller_applied_current_mA")]
        if any(value is not None and value != 0 for value in cmd):
            active.append(tick)
        ack = sample.get("roller_applied_command_time_us")
        if ack and int(ack) and cmd[-1] is not None:
            # Launch and camera-derived measurement origins can straddle zero
            # within the first millisecond. Preserve that signed offset across wrap.
            ack_ms = ((int(ack)-epoch+2**31) % 2**32 - 2**31)/1000
            if -1 <= ack_ms <= tick+1:
                applied[ack_ms] = cmd[-1]
        stamp = sample.get("roller_current_sample_time_us")
        if stamp and int(stamp) and str(sample.get("roller_current_valid")).lower() in ("1", "true"):
            time = ((int(stamp)-epoch) & 0xffffffff)/1000
            current = numeric(sample.get("roller_actual_current_mA"))
            if current is not None and 0 <= time <= tick+1 and (not currents or time > currents[-1][0]):
                currents.append((time, abs(current)))
    return active, currents, sorted(applied.items())


def measure(header, meta, samples, labels):
    rows = checked_labels(labels, int(header["run_start_us"]))
    if not rows:
        return []
    frames = meta.get("foot_frames", [])
    run_ids = {frame.get("run_id") for frame in frames if frame.get("state_id") == 3}
    if run_ids and run_ids != {header["run_id"]}:
        raise ValueError("Camera run_id does not match this RWLOG")
    epoch = measurement_epoch(meta) if frames else None
    samples = list(samples)
    scale = numeric(meta.get("mekf_gyro_y_scale"))
    times, rates, angles = (gyro_samples(meta, samples, epoch) if scale is not None
                            else ([], [], []))
    camera = None
    if epoch is not None:
        camera = CausalFootFeatures(meta)
    active, currents, applied = input_evidence(samples, epoch) if epoch is not None else ([], [], [])
    archive = command_archive(meta, applied)
    for row in rows:
        start, end, direction = row["start_ms"], row["end_ms"], row["body_direction"]
        row.update(imu_status="unknown_measurement_clock", foot_status="unknown_measurement_clock",
                   command_overlap="unknown_window")
        if epoch is None:
            continue
        i, j = bisect_left(times, start)-1, bisect_right(times, end)
        cutoff = times[i] if i >= 0 and start-times[i] <= MAX_IMU_GAP_MS else start
        row["foot_status"] = pre_foot(camera, cutoff, row)
        if scale is None:
            row["imu_status"] = "unknown_gyro_scale"
            continue
        row["imu_status"] = "missing_pre_imu" if i < 0 else "missing_post_imu"
        for name, idx, boundary in (("pre", i, start), ("post", j, end)):
            if 0 <= idx < len(times):
                gap = abs(times[idx]-boundary)
                row.update({name+"_ms": times[idx], name+"_gap_ms": gap})
                if gap <= MAX_IMU_GAP_MS:
                    row.update({name+"_gyro_dps": rates[idx], name+"_directional_dps": direction*rates[idx],
                                name+"_body_deg": angles[idx]})
        if i < 0 or j >= len(times):
            continue
        pre, post = times[i], times[j]
        max_gap = max(times[k+1]-times[k] for k in range(i, j))
        row["max_imu_gap_ms"] = max_gap
        row["imu_status"] = "ok" if max_gap <= MAX_IMU_GAP_MS else "imu_gap"
        if row["imu_status"] == "ok":
            row["delta_directional_dps"] = direction*(rates[j]-rates[i])
        overlap = command_overlap(archive, pre, post)
        if any(pre <= tick <= post for tick in active):
            overlap = "overlap"
        row["command_overlap"] = overlap
        previous = [r["output_end_ms"] for r in archive["records"] if r["executed"]
                    and r["output_end_ms"] and r["output_end_ms"] <= pre]
        if previous:
            row["ms_since_recorded_command_end"] = pre-max(previous)
        measured = [(t, current) for t, current in currents if pre <= t <= post]
        row["current_samples"] = len(measured)
        if measured:
            row["max_abs_measured_current_ma"] = max(current for _, current in measured)
            stamps = [pre, *(t for t, _ in measured), post]
            row["max_current_gap_ms"] = max(b-a for a, b in zip(stamps, stamps[1:]))
    return rows


def led_edges(header, meta, samples):
    """Bracket each LED change in measurement milliseconds, including the start/end patterns."""
    epoch = measurement_epoch(meta)
    if epoch is None:
        raise ValueError("Missing unique measurement clock")
    origin_ms = (int(header["run_start_us"]) - epoch) / 1000
    edges, previous = [], None
    for sample in samples:
        log_seconds = float(sample["log_time_s"])
        # The experiment-start marker can carry a zero archive timestamp mid-file.
        # The adjacent ordinary samples still bracket any real LED edge.
        if log_seconds == 0 and str(sample.get("sync_event_id")) == "2" and str(sample.get("state_id")) == "3":
            continue
        stamp = round(log_seconds * 1000 + origin_ms, 3)
        state = int(sample["led_state"])
        if state not in (0, 1) or not math.isfinite(stamp) or (previous and stamp < previous[0]):
            raise ValueError("Invalid LED timeseries")
        if previous and state != previous[1]:
            edges.append(dict(edge=len(edges)+1, state=state, log_lo_ms=previous[0], log_hi_ms=stamp))
        previous = stamp, state
    return edges


def video_clock(run_start, edges, anchors):
    """Find affine clocks consistent with every paired video frame and recorded LED edge."""
    bands = []
    for row in anchors:
        edge = int(row["edge"])
        if int(row["run_start_us"]) != run_start or not 1 <= edge <= len(edges):
            raise ValueError("LED anchor does not belong to this run/edge")
        entry = edges[edge-1]
        lo, hi = float(row["video_lo_ms"]), float(row["video_hi_ms"])
        if not (0 <= lo < hi < math.inf) or int(row["state"]) != entry["state"]:
            raise ValueError("Invalid video LED frame bounds or edge polarity")
        if bands and (lo <= bands[-1][1] or entry["log_lo_ms"] <= bands[-1][3]):
            raise ValueError("Pair distinct LED edges in chronological order")
        bands.append((lo, hi, entry["log_lo_ms"], entry["log_hi_ms"]))
    if len(bands) < 3:
        raise ValueError("Pair at least three LED transitions spanning the observed events")
    # With increasing log = a*video+b each band gives two linear constraints.
    # Extremes of a mapped timestamp occur at vertices of this bounded polygon.
    boundaries = [(hi, log_lo) for _, hi, log_lo, _ in bands]
    boundaries += [(lo, log_hi) for lo, _, _, log_hi in bands]
    clocks = []
    for (x1, y1), (x2, y2) in combinations(boundaries, 2):
        if x1 == x2:
            continue
        slope = (y2-y1)/(x2-x1)
        intercept = y1-slope*x1
        if slope > 0 and all(slope*hi+intercept >= log_lo-1e-7 and
                             slope*lo+intercept <= log_hi+1e-7 for lo, hi, log_lo, log_hi in bands):
            clocks.append((slope, intercept))
    if not clocks:
        raise ValueError("No clock fits the matched LED edges; check pairing, frame bounds or recording cuts")
    return bands[0][1], bands[-1][0], clocks


def align_video_labels(labels, clock):
    earliest, latest, clocks = clock
    for label in labels:
        lo, hi = float(label["video_lo_ms"]), float(label["video_hi_ms"])
        if label["source"] != "video_proxy" or not earliest <= lo < hi <= latest:
            raise ValueError("Video label must lie between matched LED anchors with source=video_proxy")
        if label.get("event_kind") not in ("touchdown_to_liftoff", "receiving_touchdown", "old_foot_liftoff"):
            raise ValueError("Identify the observed touchdown, liftoff, or the full transfer interval")
        yield dict(label, start_ms=min(a*lo+b for a, b in clocks),
                   end_ms=max(a*hi+b for a, b in clocks))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, help="Directory from tools/convert_rwlog_to_csv.py")
    parser.add_argument("labels", nargs="?", type=Path, help="Independently observed event intervals (CSV)")
    parser.add_argument("--video-anchors", type=Path, help="Match video PTS to recorded LED edges (CSV)")
    parser.add_argument("--list-led", action="store_true", help="List recorded LED edge numbers and times")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.list_led and (args.labels or args.video_anchors or args.out):
        parser.error("Use --list-led separately")
    if not args.list_led and not (args.labels and args.out):
        parser.error("Supply labels and --out or use --list-led")
    header = json.loads((args.trial/"header.json").read_text(encoding="utf8"))
    meta = json.loads((args.trial/"metadata.json").read_text(encoding="utf8"))
    with (args.trial/"timeseries.csv").open(encoding="utf-8-sig", newline="") as times:
        samples = list(csv.DictReader(times))
    if args.list_led or args.video_anchors:
        edges = led_edges(header, meta, samples)
    if args.list_led:
        writer = csv.DictWriter(sys.stdout, fieldnames=("edge", "state", "log_lo_ms", "log_hi_ms"),
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(edges)
        return
    with args.labels.open(encoding="utf-8-sig", newline="") as label_file:
        reader = csv.DictReader(label_file)
        required = set(LABEL_FIELDS)
        if args.video_anchors:
            required = required.difference(("start_ms", "end_ms")) | {"video_lo_ms", "video_hi_ms", "event_kind"}
        if not required.issubset(reader.fieldnames or ()):
            parser.error("Required label columns: " + ", ".join(sorted(required)))
        labels = list(reader)
    if args.video_anchors:
        with args.video_anchors.open(encoding="utf-8-sig", newline="") as anchor_file:
            clock = video_clock(int(header["run_start_us"]), edges, csv.DictReader(anchor_file))
        labels = align_video_labels(labels, clock)
    result = measure(header, meta, samples, labels)
    inputs = [args.labels, args.video_anchors, *(args.trial/p for p in ("header.json", "metadata.json", "timeseries.csv"))]
    if args.out.resolve() in {p.resolve() for p in inputs if p}:
        parser.error("Output must differ from inputs")
    write_rows(args.out, FIELDS, result)
    print(f"Extracted {len(result)} independent label(s) to {args.out}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError) as error:
        sys.exit(f"Measurement error: {error}")
