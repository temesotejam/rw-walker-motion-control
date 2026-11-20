"""Replay event-horizon plans on CRC-checked v37 logs and score geometric transfer proxies."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import csv
import hashlib
import json
from math import copysign, cos, isfinite, radians, sin
from pathlib import Path
import statistics
import subprocess
import tempfile

from convert_rwlog_to_csv import convert
from event_foot_features import CausalFootFeatures

ROOT = Path(__file__).resolve().parents[1]
REASONS = ("invalid", "outside", "coast", "wait", "drive", "insufficient")
PERIOD_US = 25000
BODY_KEYS = ("pitch_mekf_detector_relative_deg", "pitch_mekf_abs_deg", "roll_mekf_abs_deg", "gx_dps", "gy_dps",
             "gz_dps", "mekf_bias_x_dps", "mekf_bias_y_dps", "mekf_bias_z_dps",
             "roller_actual_current_mA", "roller_battery_mV")


def current_model(meta):
    """Use the 1200 mA calibration stored as run configuration in the source log."""
    command = 1200
    tau = meta["model_tau_rise_min_ms"] + (meta["model_tau_rise_max_ms"] - meta["model_tau_rise_min_ms"]) / (
        1 + (command / meta["model_tau_rise_u_mA"]) ** meta["model_tau_rise_exponent"])

    def goal(mv):
        voltage = min(meta["model_vbat_max_v"],max(meta["model_vbat_min_v"],mv * .001))
        sat = (meta["model_i_sat_at_reference_mA"] + meta["model_i_sat_slope_mA_per_v"]
               * (voltage - meta["model_vbat_reference_v"]))
        power = meta["model_i_sat_exponent"]
        return command / (1 + (command / sat) ** power) ** (1 / power)
    return tau * .001, goal


def samples(folder, header, meta):
    owner = []
    with (folder / "timeseries.csv").open(encoding="utf8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["state_id"] != "3" or float(row["log_time_s"]) <= 0:
                continue  # the synthetic transition row has no owner timestamp
            r = {k: float(row[k]) for k in BODY_KEYS}
            r.update(now=header["run_start_us"] + round(1e6 * float(row["log_time_s"])),
                     imu=int(row["body_rate_sample_time_us"]), current_us=int(row["roller_current_sample_time_us"]),
                     current_valid=row["roller_current_valid"] == "1", pulse=row["pulse_active"] == "1",
                     motor_cmd=int(row["motor_cmd_mA"]),
                     speed_valid=row["roller_speed_valid"] == "1", speed_end=int(row["roller_speed_read_end_us"]))
            r["rate"] = (r["gy_dps"] - r["mekf_bias_y_dps"]) * meta["mekf_gyro_y_scale"]
            pitch, roll = radians(r["pitch_mekf_abs_deg"]), radians(r["roll_mekf_abs_deg"])
            gx, gy, gz = -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)
            yz = gy*gy + gz*gz
            wx = -(r["gx_dps"]-r["mekf_bias_x_dps"])
            wz = -(r["gz_dps"]-r["mekf_bias_z_dps"])
            r["fore_rate"] = wx-gx*(gz*wz+gy*r["rate"])/yz if yz >= .04 else None
            owner.append(r)
    if not owner or any(b["now"] < a["now"] for a, b in zip(owner, owner[1:])):
        raise ValueError("Missing or unordered active owner log")
    return owner


def last_at_or_before(times, cutoff):
    index = bisect_right(times, cutoff) - 1
    return None if index < 0 else index


def budget_at(records, t_ms, ceiling):
    """Return only entry/completion effects known by this tick; record ms are rounded."""
    remaining, used = ceiling, False
    for event in records:
        if not event["executed"] or t_ms < event["output_start_ms"] + 1:
            continue
        used = True
        if not event["completion_valid"] or t_ms < event["output_end_ms"] + 1:
            return None, used
        remaining = max(0, remaining-event["actual_elapsed_ms"])
    return remaining, used


def planning_queries(meta, owner, feet):
    """Only the most recently *published* sample/receipt and confirmed peak enter a plan."""
    epoch = feet.epoch_us
    tau, goal = current_model(meta)
    commands = []
    for frame in feet.frames:
        clock = frame["timestamp_valid"] and frame["zero_ready"]
        values = [frame["control_observed_us"], frame["sequence"], frame["frame_us"], int(clock)]
        for side in ("left", "right"):
            values.extend((int(frame[side + "_valid"]), int(frame[side + "_control_range_ok"]), frame[side + "_deg"] or 0))
        commands.append((values[0], 0, "F " + " ".join(map(str, values))))
    energy = meta["walking_energy"]
    ref = energy["fore_aft_motion_guard"]
    peaks = meta["energy_control_autonomous_peak_events"]
    # The detected maximum predates confirmation; allow the full reported millisecond to elapse.
    confirmations = [epoch + (p["peak_telemetry_capture_time_ms"] + 1) * 1000 for p in peaks]
    if any(b <= a for a, b in zip(confirmations, confirmations[1:])):
        raise ValueError("Peak admission timestamps missing or unordered")
    recorded = defaultdict(list)
    for event in energy["records"]:
        recorded[event["peak_ms"]].append(event)
    owner_times = [r["now"] for r in owner]
    queries, previous = [], None
    cap = energy["combined_half_cycle_envelope_ms"]
    polarity = meta["motor_driver_polarity"] * meta["normal_autonomous_motor_output_sign"]
    if meta["motor_driver_polarity"] not in (-1, 1) or meta["normal_autonomous_motor_output_sign"] not in (-1, 1):
        raise ValueError("Unknown driver-to-logical current polarity")
    for tick in range(epoch + PERIOD_US, owner_times[-1] + 1, PERIOD_US):
        commands.append((tick, 1, f"T {tick}"))
        index = last_at_or_before(owner_times, tick)
        r = owner[index] if index is not None else None
        prev, previous = previous, r
        p = last_at_or_before(confirmations, tick)
        if p is None or not prev or not r:
            continue
        dt = r["imu"] - prev["imu"]
        if (r["pulse"] or not 15000 <= dt <= 50000 or not 0 <= tick - r["imu"] <= 10000
                or not 0 <= tick - PERIOD_US - prev["imu"] <= 10000
                or not r["current_valid"] or not 0 <= tick - r["current_us"] <= 50000
                or not r["speed_valid"] or not 0 <= tick - r["speed_end"] <= 50000
                or r["roller_battery_mV"] < 1000 or tick < ref["start_sample_us"]):
            continue
        peak = peaks[p]
        side = -peak["physical_peak_side"]
        t_ms = (tick-epoch)/1000
        if side*r["rate"] <= 0 or t_ms-peak["peak_time_ms"] > 1500:
            continue
        remaining, used = budget_at(recorded[peak["peak_time_ms"]], t_ms, cap)
        if remaining is None:
            continue
        fore = r["roll_mekf_abs_deg"] - ref["start_roll_deg"]
        if r["fore_rate"] is None:
            continue
        outward = copysign(1.,fore)*r["fore_rate"]
        if (abs(fore) >= ref["candidate_angle_deg"] and outward >= ref["minimum_outward_rate_dps"] and
                1000*max(0,ref["predicted_boundary_deg"]-abs(fore))/outward <= ref["maximum_time_to_boundary_ms"]):
            continue
        vals = (side, r["pitch_mekf_detector_relative_deg"], r["rate"],
                (r["rate"] - prev["rate"]) * 1e6 / dt, fore, side * polarity * r["roller_actual_current_mA"],
                goal(r["roller_battery_mV"]), remaining)
        commands.append((tick, 2, "D " + " ".join(map(str, vals))))
        queries.append(dict(peak=peak["peak_time_ms"], time_ms=t_ms, sway_deg=vals[1], prior_pulse=used,
                            rate_dps=vals[2], accel_dps2=vals[3], owner_age_ms=(tick-r["now"])/1000))
    lines = [f"{epoch} {tau}", *(c for _, _, c in sorted(commands))]
    return lines, queries


def hindsight_frames(meta, owner, epoch):
    """Use measured feet at exposure and independently bracketed IMU, strictly for scoring."""
    imu = {}
    for r in owner:
        if r["imu"] and 0 <= r["now"] - r["imu"] <= 10000:
            imu.setdefault(r["imu"], r)
    times = sorted(imu)
    reference = meta["walking_energy"]["fore_aft_motion_guard"]["start_roll_deg"]
    frames, commands = [], []
    for f in meta["foot_frames"]:
        if f["state_id"] != 3 or not (f["timestamp_valid"] and f["frame_valid"] and f["zero_ready"]):
            continue
        if not all(f[side + "_valid"] and f[side + "_control_range_ok"] for side in ("left", "right")):
            continue
        t = f["frame_us"]
        j = bisect_right(times, t)
        if j == 0 or j == len(times) or times[j] - times[j-1] > 10000:
            continue
        a, b = imu[times[j-1]], imu[times[j]]
        alpha = (t - times[j-1]) / (times[j] - times[j-1])

        def interpolate(key):
            return a[key] + alpha * (b[key] - a[key])

        fore = interpolate("roll_mekf_abs_deg") - reference
        l, r = f["left_deg"], f["right_deg"]
        if abs(fore) > 10 or max(abs(l), abs(r)) > 20:
            continue
        frames.append(dict(ms=(t - epoch) / 1000, seq=f["sequence"], left=l, right=r,
                           sway=interpolate("pitch_mekf_detector_relative_deg"), rate=interpolate("rate")))
        commands.append(f"G {l} {r} {fore} {interpolate('pitch_mekf_detector_relative_deg')}")
    return commands, frames


def parse_decision(row, line):
    fields = line.split()
    if fields[0] != "D" or len(fields) != 12:
        raise ValueError("Unexpected planner response")
    row.update(reason=REASONS[int(fields[1])], delay_ms=int(fields[2]), width_ms=int(fields[3]),
               transfer_ms=float(fields[4]), transfer_deg=float(fields[5]), coast_margin_j=float(fields[6]),
               margin_j=float(fields[7]), work_j=float(fields[8]), work_supported=bool(int(fields[9])),
               left_sequence=int(fields[10]), right_sequence=int(fields[11]))


def score_observed_exit(a, b, span, side, actual):
    # Score the planner's terminal event using later synchronized body/foot measurements.
    # Only interpolate between contiguous samples where the receiving sole stays lower.
    fraction = -a["gap"] / (b["gap"] - a["gap"])
    cross_sway = a["sway"] + fraction * (b["sway"] - a["sway"])
    end_angle = cross_sway + side
    prev = dict(ms=a["ms"] + fraction * (b["ms"] - a["ms"]), sway=cross_sway,
                rate=a["rate"] + fraction * (b["rate"] - a["rate"]))
    for frame in span:
        if frame["gap"] * side < 0:
            break
        distance = side * (frame["sway"] - prev["sway"])
        if distance > 0 and side * (frame["sway"] - end_angle) >= 0:
            alpha = (end_angle-prev["sway"]) / (frame["sway"]-prev["sway"])
            t = prev["ms"] + alpha * (frame["ms"]-prev["ms"])
            rate = side * (prev["rate"] + alpha * (frame["rate"]-prev["rate"]))
            return dict(exit_proxy_ms=t, exit_direction_rate_dps=rate, exit_carry_5dps=rate >= 5,
                        legacy_pulse_before_exit=bool(actual and actual["output_start_ms"] < t))
        prev = frame
    return {}


def recorded_coast(start, end, epoch, owner_times, owner, pulses):
    """Agreement of pulse intervals and continuously sampled output commands."""
    if start is None or end is None or end <= start:
        return None
    if any(a < end and b > start for a, b in pulses):
        return False
    # Marginal join near a rounded pulse boundary is inconclusive.
    if any(a-2 < end and b+2 > start for a, b in pulses):
        return None
    first = last_at_or_before(owner_times, epoch+start*1000)
    last = bisect_right(owner_times, epoch+end*1000)
    if first is None or last == len(owner):
        return None
    portion = owner[first:last+1]
    if any(b["now"]-a["now"] > 10000 for a, b in zip(portion, portion[1:])):
        return None
    return not any(s["pulse"] or s["motor_cmd"] for s in portion)


def score_cycle(peak, end, side, ticks, frames, baseline):
    first = next((q for q in ticks if q["reason"] not in ("invalid", "outside")), {})
    drive = next((q for q in ticks if q["reason"] == "drive"), {})
    wait = next((q for q in ticks if q["reason"] == "wait"), {})
    count = Counter(q["reason"] for q in ticks)
    pulses = [p for p in baseline if p["executed"] and p["peak_ms"] == peak and p["side"] == side]
    actual = min(pulses, key=lambda p: p["output_start_ms"]) if pulses else {}
    here = [f for f in frames if peak < f["ms"] <= end]
    cross = [(a, b) for a, b in zip(here, here[1:]) if b["seq"] == a["seq"] + 1
             and 0 < b["ms"] - a["ms"] <= 50 and side*a["gap"] < 0 <= side*b["gap"]]
    obs = dict(observed_gap_crossings=len(cross))
    if len(cross) == 1:
        a, b = cross[0]
        center = a["ms"] - a["gap"] * (b["ms"] - a["ms"]) / (b["gap"] - a["gap"])
        obs.update(proxy_lower_ms=a["ms"], proxy_upper_ms=b["ms"], proxy_interpolated_ms=center,
                   direction_rate_at_upper_dps=side*b["rate"])
        if drive:
            t = drive["time_ms"]
            obs.update(drive_relative_to_proxy=("before_bracket" if t < a["ms"] else
                       "after_bracket" if t > b["ms"] else "inside_bracket"),
                       proxy_minus_drive_ms=center-t)
        swing = "left" if side > 0 else "right"
        span = [b]
        for f in here:
            if f["ms"] <= b["ms"]:
                continue
            if f["seq"] != span[-1]["seq"] + 1 or f["ms"] - span[-1]["ms"] > 50 or f["gap"]*side < 0:
                break
            span.append(f)
        obs.update(score_observed_exit(a, b, span, side, actual))
        if len(span) >= 2:
            obs.update(opposite_foot=swing, opposite_forward_excursion_deg=max(f[swing] for f in span)-b[swing],
                       opposite_observation_ms=span[-1]["ms"]-b["ms"])
        if first and first["time_ms"] < a["ms"] and isfinite(first["transfer_ms"]):
            obs["forecast_minus_proxy_ms"] = first["time_ms"] + first["transfer_ms"] - center
            obs["legacy_pulse_before_proxy"] = bool(actual and actual["output_start_ms"] < center)
    return dict(peak_ms=peak, next_side=side, **{s+"_ticks":count[s] for s in REASONS},
                first_reason=first.get("reason"), first_ms=first.get("time_ms"), first_after_legacy=first.get("prior_pulse"),
                first_forecast_lead_ms=first.get("transfer_ms"), first_coast_margin_j=first.get("coast_margin_j"),
                first_forecast_sway_at_transfer_deg=first.get("transfer_deg"), first_sway_deg=first.get("sway_deg"),
                first_wait_ms=wait.get("time_ms"), first_wait_delay_ms=wait.get("delay_ms"),
                first_drive_ms=drive.get("time_ms"), first_drive_width_ms=drive.get("width_ms"),
                first_drive_after_legacy=drive.get("prior_pulse"),
                first_drive_sway_deg=drive.get("sway_deg"), first_drive_rate_dps=drive.get("rate_dps"),
                first_drive_coast_margin_j=drive.get("coast_margin_j"),
                first_drive_work_j=drive.get("work_j"), first_drive_work_supported=drive.get("work_supported"),
                supported_drive_ticks=sum(q["reason"]=="drive" and q["work_supported"] for q in ticks),
                first_drive_remaining_margin_j=drive.get("margin_j"),
                legacy_start_ms=actual.get("output_start_ms"), legacy_width_ms=actual.get("guarded_ms"),
                legacy_start_minus_drive_ms=actual["output_start_ms"]-drive["time_ms"] if actual and drive else None,
                **obs)


def replay(raw, converted, binary):
    convert(raw, converted)
    header = json.loads((converted/"header.json").read_text(encoding="utf8"))
    meta = json.loads((converted/"metadata.json").read_text(encoding="utf8"))
    energy = meta["walking_energy"]
    if (header["format_version"] != 56 or meta["diagnostic_build_id"] != "walking-foot-cadence20-v37" or
            energy["revision"] != "walking_energy_admission_v19" or
            energy["truncated"] or len(energy["records"]) != energy["total"] or
            meta["metadata_event_detail_truncated"] or meta["energy_control_autonomous_event_overflow"] or
            not meta["work_absolute_reference_enabled"] or meta["control_angle_origin_abs_deg"] != 0):
        raise ValueError("This replay requires complete v37 walking records with the mechanical zero reference.")
    feet = CausalFootFeatures(meta)
    owner = samples(converted, header, meta)
    cmds, queries = planning_queries(meta, owner, feet)
    offline, frames = hindsight_frames(meta, owner, feet.epoch_us)
    result = subprocess.run([binary], input="\n".join(cmds+offline)+"\n", text=True,
                            capture_output=True, check=True).stdout.splitlines()
    if len(result) != len(queries) + len(frames):
        raise ValueError("Incomplete planner output")
    by_peak = defaultdict(list)
    for q, line in zip(queries, result):
        parse_decision(q, line)
        by_peak[q["peak"]].append(q)
    for f, line in zip(frames, result[len(queries):]):
        op, gap = line.split()
        if op != "G":
            raise ValueError("Missing independent geometry result")
        f["gap"] = float(gap)
    peaks = meta["energy_control_autonomous_peak_events"]
    zeros = meta["energy_control_autonomous_zero_cross_events"]
    pulses = [(z["pulse_start_ms"],z["pulse_end_ms"]) for z in zeros if z["output_executed"]]
    # Include startup, and use the actual stop in the walking audit when the
    # control task completed a pulse later than the event's logged stop time.
    pulses.extend((r["output_start_ms"],r["output_end_ms"]) for r in energy["records"] if r["executed"])
    if any(b < a for a, b in pulses) or any(r["executed"] and not r["completion_valid"] for r in energy["records"]):
        raise ValueError("Incomplete pulse interval in source log")
    owner_times = [r["now"] for r in owner]
    rows = []
    for i,p in enumerate(peaks):
        next_peak = peaks[i+1] if i+1 < len(peaks) else None
        end = next_peak["peak_time_ms"] if next_peak else (owner[-1]["now"]-feet.epoch_us)/1000
        side = -p["physical_peak_side"]
        row = score_cycle(p["peak_time_ms"], end, side, by_peak[p["peak_time_ms"]], frames, energy["records"])
        mechanical = [z["zero_cross_time_ms"] for z in zeros if z["event_kind"]=="energy_control_zero_cross"
                      and z["previous_peak_time_ms"]==p["peak_time_ms"] and z["physical_next_peak_side"]==side
                      and p["peak_time_ms"] < z["zero_cross_time_ms"] < end
                      and side*z["detector_angle_before_deg"]<=0<=side*z["detector_angle_after_deg"]
                      and z["detector_angle_before_deg"] != z["detector_angle_after_deg"]]
        if len(mechanical) == 1:
            row["mechanical_zero_ms"] = mechanical[0]
            if "proxy_interpolated_ms" in row:
                row["proxy_minus_mechanical_zero_ms"] = row["proxy_interpolated_ms"]-mechanical[0]
        row["coast_first_forecast_to_exit"] = recorded_coast(row["first_ms"], row.get("exit_proxy_ms"),
                    feet.epoch_us, owner_times, owner, pulses) if "forecast_minus_proxy_ms" in row else None
        rows.append(dict(run_start_us=header["run_start_us"], opposite_peak=next_peak["physical_peak_side"]==side
                         if next_peak else None, **row))
    errors = [r["forecast_minus_proxy_ms"] for r in rows if "forecast_minus_proxy_ms" in r]
    lead = [r["legacy_start_minus_drive_ms"] for r in rows if r["legacy_start_minus_drive_ms"] is not None]
    info = dict(source=raw.name, sha256=hashlib.sha256(raw.read_bytes()).hexdigest(), crc_ok=True,
                build_id=meta["diagnostic_build_id"], half_cycles=len(rows), evaluated_ticks=len(queries),
                planner_ticks=dict(Counter(q["reason"] for q in queries)),
                first_drive_cycles=sum(r["first_drive_ms"] is not None for r in rows),
                first_drive_before_any_source_pulse=sum(r["first_drive_after_legacy"] is False for r in rows),
                first_drive_supported_cycles=sum(r["first_drive_work_supported"] is True for r in rows),
                any_supported_drive_cycles=sum(r["supported_drive_ticks"] > 0 for r in rows),
                supported_drive_ticks=sum(r["supported_drive_ticks"] for r in rows),
                first_drive_before_proxy_bracket=sum(r.get("drive_relative_to_proxy")=="before_bracket" for r in rows),
                first_drive_with_proxy=sum("drive_relative_to_proxy" in r for r in rows),
                matched_legacy_drive=len(lead), median_legacy_start_minus_drive_ms=statistics.median(lead) if lead else None,
                unique_proxy_cycles=sum(r["observed_gap_crossings"]==1 for r in rows),
                directional_exit_speed_pairs=sum("exit_direction_rate_dps" in r for r in rows),
                confirmed_coast_forecast_to_exit=dict(Counter(r["first_reason"] for r in rows
                    if r["coast_first_forecast_to_exit"] is True and r["exit_carry_5dps"])),
                forecast_proxy_pairs=len(errors), median_abs_forecast_error_ms=statistics.median(map(abs,errors)) if errors else None)
    return info, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rwlog", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    runs, rows = [], []
    with tempfile.TemporaryDirectory(prefix="support-replay-") as tmp:
        temp = Path(tmp)
        binary = temp / "replay.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-DSUPPORT_TRANSFER_V39=1", "-I", ROOT/"src", ROOT/"tools/support_transfer_replay.cpp",
                        "-o", binary], check=True)
        for i, raw in enumerate(args.rwlog):
            info, records = replay(raw, temp/str(i), binary)
            runs.append(info)
            rows.extend(records)
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out/"half_cycles.csv").open("w", encoding="utf8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        writer.writeheader()
        writer.writerows(rows)
    (args.out/"summary.json").write_text(json.dumps(runs, indent=2, allow_nan=False)+"\n", encoding="utf8")
    print(json.dumps(runs, indent=2))


if __name__ == "__main__":
    main()
