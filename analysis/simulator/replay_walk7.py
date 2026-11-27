"""Replay post-release walking interventions on saved RWLOG trajectories.

Each half-cycle starts from its original logged inputs. Once an intervention
is proposed its subsequent motion is unknown. By default, reproduce the
walk6 recheck and independent from-rest calculation. --after-first uses the
earlier horizon and --event-allocation enables the walk7 actuator response
and split between the inward aid and the centre-crossing event.
"""
import argparse
from bisect import bisect_right
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "analysis/support"))
from convert_rwlog_to_csv import convert
from score_observed_transfers import measurement_epoch


def rows(folder, name):
    with (folder / (name + ".csv")).open(newline="", encoding="utf8") as file:
        return list(csv.DictReader(file))


def wire(*values):
    return " ".join("nan" if v is None or v == "" else str(int(v)) if isinstance(v, bool) else str(v)
                    for v in values)


def run(exe, lines):
    return subprocess.run([exe], input="\n".join(lines) + "\n", text=True,
                          capture_output=True, check=True).stdout.splitlines()


def restart_replay(exe, meta, samples, epoch, packets, frames, peaks, decisions):
    fore = meta["walking_energy"]["fore_aft_motion_guard"]
    # The guard has no per-sample allow bit in RWLOG v56. Accept only a run
    # with a valid reference, no guard veto episode and no terminal fault.
    if not fore["initialized"] or fore["candidate_entries"] or fore["stop"]:
        return {"unreplayable": "fore/aft guard or its per-sample history unavailable"}
    start_roll = fore["start_roll_deg"]
    published = [epoch + int(p["publish_us"]) for p in packets]
    had_peak_at = min(p["peak_telemetry_capture_time_ms"] for p in peaks)
    rise_time = next(r["start_tau_s"] for r in decisions if r["executed"] and r["start_tau_s"] > 0)
    lines = []
    previous_angle = previous_stamp = None
    for s in samples:
        if int(s["state_id"]) != 3 or not s["body_rate_sample_time_us"]:
            continue
        ms = int(s["t_test_ms"])
        stamp = int(s["body_rate_sample_time_us"])
        if stamp == previous_stamp or not s["imu_sample_age_us"]:
            continue
        previous_stamp = stamp
        age = int(s["imu_sample_age_us"])
        at = epoch + ((stamp - epoch) & 0xffffffff) + age
        if at > epoch + 31_000_000:
            continue
        angle = float(s["pitch_mekf_detector_relative_deg"])
        rate = (float(s["gy_dps"]) - float(s["mekf_bias_y_dps"])) * meta["mekf_gyro_y_scale"]
        crossed = previous_angle is not None and (
            previous_angle > 0 >= angle and rate < 0 or previous_angle < 0 <= angle and rate > 0)
        previous_angle = angle
        n = bisect_right(published, at) - 1
        fresh = n >= 0 and 0 <= at - published[n] <= 50_000
        feet = []
        for side in ("left","right"):
            packet = packets[n] if n >= 0 else {}
            image = frames.get(int(packet.get(side + "_sequence") or 0))
            fresh = bool(fresh and image and image["timestamp_valid"] and image["frame_valid"] and
                         image["zero_ready"] and image[side+"_valid"] and image[side+"_control_range_ok"] and
                         epoch <= image["frame_us"] <= image["control_observed_us"] <= published[n] and
                         at - image["frame_us"] <= 75_000 and packet[side+"_est_deg"] != "")
            feet.append(packet.get(side+"_observed_deg"))
        # In the upright branch the normalized YZ gravity angle equals Euler
        # roll. Derive its angular rate using the same body-axis mapping.
        roll, pitch = map(math.radians, map(float, (s["roll_mekf_abs_deg"], s["pitch_mekf_abs_deg"])))
        wx = -float(s["gx_dps"]) + float(s["mekf_bias_x_dps"])
        wz = -float(s["gz_dps"]) + float(s["mekf_bias_z_dps"])
        fore_rate = wx + math.tan(pitch)*(wz*math.cos(roll) + rate*math.sin(roll))
        current_age = (int(at)-int(s["roller_current_sample_time_us"] or 0)) & 0xffffffff
        speed_age = (int(at)-int(s["roller_speed_sample_time_us"] or 0)) & 0xffffffff
        voltage = float(s["roller_battery_mV"])*.001
        hardware = (current_age <= 50000 and speed_age <= 50000 and s["roller_current_valid"] == "1" and
                    s["roller_speed_valid"] == "1" and abs(float(s["roller_speed_rpm"])) <= 1000 and
                    6.18 <= voltage <= 8.5 and
                    s["roller_applied_current_mA"] == "0" and abs(float(s["roller_actual_current_mA"])) <= 40 and
                    s["motor_cmd_mA"] == "0" and ms + 80 < meta["autonomous_duration_ms"])
        sat = meta["model_i_sat_at_reference_mA"] + meta["model_i_sat_slope_mA_per_v"]*(voltage-meta["model_vbat_reference_v"])
        power = meta["model_i_sat_exponent"]
        goal = 1200/(1+(1200/sat)**power)**(1/power)
        lines.append(wire("D", ms, stamp, ms >= had_peak_at, age<=10000, fresh, s["pulse_active"]=="1",
                          ms >= had_peak_at, crossed, angle, rate, math.degrees(roll)-start_roll,
                          fore_rate, *feet, hardware, s["roller_actual_current_mA"], goal, rise_time))
    reply = run(exe, lines)
    if not reply:
        return None
    _, ms, direction, angle, rate, left, right, ref, width, demand, predicted = reply[0].split()
    return dict(time_ms=int(ms),side=int(direction),angle_deg=float(angle),rate_dps=float(rate),
                left_deg=float(left),right_deg=float(right),reference_dps=float(ref),
                model_width_ms=int(width),request_uJ=round(float(demand)*1e6,3),
                predicted_work_uJ=round(float(predicted)*1e6,3))


def replay(folder, exe, after_first=False, event_allocation=False):
    meta = json.loads((folder / "metadata.json").read_bytes())
    epoch = measurement_epoch(meta)
    if epoch is None:
        raise ValueError("Missing common IMU/camera measurement epoch")
    walking = meta["walking_energy"]
    decisions = walking["records"]
    peaks = meta["energy_control_autonomous_peak_events"]
    if walking["stored"] != walking["total"] or len(decisions) != walking["stored"]:
        raise ValueError("Incomplete walking decision archive")
    frames = sorted((f for f in meta["foot_frames"] if f["state_id"] == 3 and f["control_observed_us"]),
                    key=lambda f: f["control_observed_us"])
    frames_by_id = {f["sequence"]: f for f in frames}
    if len(frames_by_id) != len(frames):
        raise ValueError("Ambiguous source images")
    frame_times = [f["control_observed_us"] for f in frames]
    packets = rows(folder, "foot_periodic")
    timeseries = rows(folder, "timeseries")
    snapshots = rows(folder, "support_transfer_shadow")
    imu = {}
    current_samples = defaultdict(set)
    wheel_speeds = {}
    for s in timeseries:
        if s["state_id"] == "3" and s["roller_speed_valid"] == "1" and s["roller_speed_read_end_us"]:
            at = epoch + ((int(s["roller_speed_read_end_us"])-epoch) & 0xffffffff)
            wheel_speeds[at] = (int(s["roller_speed_sample_time_us"]),s["roller_speed_rpm"])
        if (s["state_id"] == "3" and s["roller_current_valid"] == "1" and s["roller_applied_current_mA"] == "0"
                and s["roller_current_sample_time_us"] and s["roller_applied_command_time_us"]):
            stamp = int(s["roller_current_sample_time_us"])
            after_ack = (stamp-int(s["roller_applied_command_time_us"])) & 0xffffffff
            if after_ack < 0x80000000:
                current_samples[-float(s["roller_actual_current_mA"]),after_ack//1000].add(stamp)
        if s["state_id"] == "3" and s["body_rate_sample_time_us"]:
            at = epoch + ((int(s["body_rate_sample_time_us"]) - epoch) & 0xffffffff)
            imu[at] = (float(s["pitch_mekf_detector_relative_deg"]),
                       (float(s["gy_dps"])-float(s["mekf_bias_y_dps"]))*meta["mekf_gyro_y_scale"])
    imu_stamps = sorted(imu)
    wheel_times = sorted(wheel_speeds)
    # Ordinary ZERO is interpolated before its decision. An exact match on
    # milliseconds would discard real passages and the last wheel observation.
    trigger_events = {}
    for decision in decisions:
        if not decision["executed"] or decision["return_event"] or not decision["guarded_ms"]:
            continue
        matches = [event for event in meta["energy_control_autonomous_zero_cross_events"]
                   if event["previous_peak_time_ms"] == decision["peak_ms"]
                   and event["physical_next_peak_side"] == decision["side"] and event["output_executed"]
                   and event["pulse_width_ms"] == decision["guarded_ms"]
                   and abs(event["zero_cross_abs_rate_dps"]-decision["start_rate_dps"]) < .001
                   and 0 <= decision["time_ms"]-event["zero_cross_time_ms"] <= 10]
        if len(matches) == 1:
            trigger_events[decision["time_ms"]] = matches[0]
    passages = []
    for p in peaks:
        if p["physical_peak_side"] != -1 or not p["pending_command_matched"]:
            continue
        previous = next((r for r in reversed(decisions) if r["time_ms"] < p["peak_time_ms"]
                         and (r["executed"] or (r["valid"] and r["coast"] and not r["return_event"]))), None)
        if not previous or previous["time_ms"] not in trigger_events or previous["side"] != -1 or any(
                previous[k] for k in ("optional","rescue","return_event")):
            continue
        if not (previous["completion_reason"] == 1 and all(previous[k] for k in
                ("completion_valid","completion_work_valid","planned_duration_completed",
                 "charge_supported","completion_width_supported"))):
            continue
        trigger = trigger_events[previous["time_ms"]]["zero_cross_time_ms"]
        passages.append((p["peak_telemetry_capture_time_ms"],wire("H",trigger,p["peak_time_ms"],
            p["peak_amplitude_deg"],previous["start_rate_dps"],previous["target_j"],
            previous["actual_elapsed_ms"],previous["elapsed_work_pred_j"])))
    lines, cases = [wire("E", event_allocation)], []
    original_post = [r for r in decisions if r["executed"] and r["return_event"] and r["side"] == -1 and
                     (r["return_evidence"] not in (7,8,9) if after_first else r["return_evidence"] == 8)]
    archived = {int(r["time_ms"]): r for r in rows(folder, "walking_energy")}
    for post in original_post:
        peak, end = post["peak_ms"], post["output_end_ms"]
        first = post if after_first else next(r for r in decisions if r["peak_ms"] == peak and
            r["output_start_ms"] < post["output_start_ms"] and r["executed"] and r["return_event"]
            and r["return_evidence"] not in (7,8,9))
        if not all(r["completion_reason"] == 1 and r["planned_duration_completed"] and r["charge_supported"]
                   and r["completion_work_valid"] for r in (first, post)):
            continue
        following = [r for r in decisions if r["peak_ms"] == peak and r["time_ms"] > end and
                     (r["executed"] or (not r["optional"] and not r["return_event"]))]
        cutoff = min([end + 151] + [r["time_ms"] for r in following] +
                     [p["peak_telemetry_capture_time_ms"] for p in peaks if p["peak_time_ms"] > peak])
        # A snapshot used to decide an already recorded pulse still belongs to
        # the measured pre-command trajectory. Include that exact packet only.
        decision_packets = {r["periodic_foot_packet"] for r in following if r["time_ms"] == cutoff
                            and r["return_event"] and r["executed"]}
        sample_rows = [s for s in snapshots if int(s["peak_ms"]) == peak and
                       (end <= int(s["time_ms"]) < cutoff or
                        (int(s["time_ms"]) == cutoff and int(s["packet"]) in decision_packets))]
        remaining = post["envelope_after_ms"]
        next_zero = next((e["zero_cross_time_ms"] for e in meta["energy_control_autonomous_zero_cross_events"]
                          if e["event_kind"] == "energy_control_zero_cross"
                          and e["zero_cross_time_ms"] > end and e["previous_peak_time_ms"] == peak), None)
        case = dict(peak_ms=peak, post_end_ms=end, remaining_ms=remaining,
                    recorded_next_zero_ms=next_zero, recheck=None, archived_slots=len(sample_rows))
        cases.append(case)
        if remaining <= 0:
            continue
        capture = next((s for s in snapshots if int(s["packet"]) == post["periodic_foot_packet"]
                        and int(s["time_ms"]) == post["time_ms"] and s.get("current_after_zero_ms","") != ""
                        and int(s["gate_mask"]) == 0), None)
        # At equal duration the stored selected-width work is calculated with
        # the packet's observed current, even when the CAD snapshot is absent.
        at_nominal = post["output_work_pred_j"] if post["actual_elapsed_ms"] == post["guarded_ms"] else None
        if not after_first and not capture and at_nominal is None:
            case["unreplayable"] = "continuation initial current provenance missing"
            continue
        edge = next(e for e in meta["energy_control_autonomous_zero_cross_events"]
                    if e["previous_peak_time_ms"] == peak and e["event_kind"] == "inward_stall_rescue")
        same_side = [p for p in peaks if p["physical_peak_side"] < 0 and
                     end-3000 <= p["peak_time_ms"] and p["peak_telemetry_capture_time_ms"] <= end]
        init = ["F",peak,end,epoch,first["elapsed_work_pred_j"],first["start_rate_dps"]] if after_first else [
            "A",peak,end,epoch,first["elapsed_work_pred_j"],post["actual_elapsed_ms"],
            capture["i0_ma"] if capture else None, post["start_goal_ma"], post["start_rate_dps"], at_nominal,
            archived[post["time_ms"]]["arc_post_release_decel_dps2"]]
        lines.append(wire(*init,post["target_j"],post["start_tau_s"],edge["startup_target_deg"],
                          edge["previous_peak_amplitude_deg"],same_side[-1]["peak_amplitude_deg"] if same_side else None,
                          remaining))
        lines.extend(line for capture_ms,line in passages if capture_ms <= end)
        # Ordinary powered commands update the wheel guard. A return aid only updates
        # its half-cycle reservation. Replay the last four matched peaks for its trim.
        latest_main = next((r for r in reversed(decisions) if r["time_ms"] <= end and r["executed"]
                            and not r["return_event"] and r["guarded_ms"]), None)
        if latest_main and latest_main["time_ms"] in trigger_events:
            zero = trigger_events[latest_main["time_ms"]]
            lines.append(wire("W",latest_main["side"],latest_main["time_ms"],zero["pre_input_wheel_speed_rpm"]))
        for p in [p for p in peaks if p["peak_telemetry_capture_time_ms"] <= end][-4:]:
            lines.append(wire("P",p["physical_peak_side"],p["peak_time_ms"],p["peak_amplitude_deg"],
                              bool(p["pending_command_matched"] and (p["pending_q_command_mA_s"] or 0)>0)))
        candidates = []
        for r in sample_rows:
            at = epoch + ((int(r["capture_us32"]) - epoch) & 0xffffffff)
            number = int(r["packet"])
            p = packets[number - 1]
            published = epoch + int(p["publish_us"])
            if published > at:
                raise ValueError("Support snapshot predates its foot packet")
            flags = int(r["gate_mask"])
            frame_index = bisect_right(frame_times, at) - 1
            f = frames[frame_index] if frame_index >= 0 else None
            flags_order = ("timestamp_valid", "frame_valid", "zero_ready", "left_valid", "right_valid",
                           "left_control_range_ok", "right_control_range_ok")
            raw = [sum(1 << i for i,k in enumerate(flags_order) if f[k]),
                   max(0, f["measurement_time_us"] // 1000), (f["control_observed_us"] - epoch) // 1000,
                   f["left_deg"], f["right_deg"]] if f else [0, 0, 0, None, None]
            sides = []
            for side in ("left", "right"):
                seq = int(p[side + "_sequence"])
                source = frames_by_id.get(seq)
                image = source["frame_us"] if source and source["control_observed_us"] <= published else 0
                sides += [seq, image, p[side + "_observed_deg"], r[side + "_deg"]]
            aligned = r["i0_ma"] if flags == 0 and r.get("current_after_zero_ms", "") != "" else None
            # Identify the wheel observation in use by value and capture time relative
            # to the acknowledged zero command. Use the oldest plausible match or the
            # firmware freshness bound when this wheel sample is absent from the archive.
            current_age = 50000
            if aligned is not None:
                stamps = current_samples[float(aligned),int(r["current_after_zero_ms"])]
                ages = [(at-stamp) & 0xffffffff for stamp in stamps]
                current_age = max((age for age in ages if age <= 50000), default=50000)
            active = not (flags & (16 | 512)) and not (int(r["baseline_flags"]) & (1 | 4))
            wheel_index = bisect_right(wheel_times,at)-1
            stamp,rpm = wheel_speeds[wheel_times[wheel_index]] if wheel_index >= 0 else (0,None)
            if ((at-stamp) & 0xffffffff) > 50000:
                rpm = None
            # A CAD snapshot stores its capture clock and freshness gate, not
            # its gyro clock. Match the rounded observed angle and rate to the
            # logged IMU sample. Otherwise use the worst age allowed by that
            # gate, particularly for the first few milliseconds after release.
            stamps = imu_stamps[bisect_right(imu_stamps,at-10001):bisect_right(imu_stamps,at)]
            matching = [stamp for stamp in stamps if abs(imu[stamp][0]-float(r["lateral_deg"]))<=.0051 and
                        abs(imu[stamp][1]-float(r["rate_dps"]))<=.011]
            age = 10001 if flags & 1 else (at-min(matching) if matching else 10000)
            candidates.append((at, 0, wire("S", at, r["time_ms"], (at-age)&0xffffffff, age,
                active, number, epoch + int(p["slot_us"]), published, *sides, *raw, r["lateral_deg"],
                r["rate_dps"], r["fore_aft_deg"], r["accel_dps2"], aligned, r["goal_ma"], current_age,
                min(remaining,int(r["cap_ms"])),rpm)))
        # Preserve cancellations visible on independent IMU samples between the
        # 40 Hz publications, including an observed zero or reversal.
        for s in timeseries:
            now = int(s["t_test_ms"])
            if s["state_id"] != "3" or not end < now < cutoff or not s["body_rate_sample_time_us"]:
                continue
            stamp = int(s["body_rate_sample_time_us"])
            age = int(s["imu_sample_age_us"])
            at = epoch + ((stamp - epoch) & 0xffffffff) + age
            rate = (float(s["gy_dps"]) - float(s["mekf_bias_y_dps"])) * meta["mekf_gyro_y_scale"]
            candidates.append((at, 1, wire("I", at, now, stamp, age, s["pitch_mekf_detector_relative_deg"], rate)))
        lines.extend(line for _, _, line in sorted(candidates))

    for reply in run(exe, lines):
        tag,peak,ms,work,cap,angle,speed,pre_width,width,extra,reserve,ref_zero,ref_peak,matched = reply.split()
        if tag != "R":
            raise ValueError(reply)
        case, = [c for c in cases if c["peak_ms"] == int(peak)]
        if case["recheck"] is not None:
            raise ValueError("Multiple offers from one completed pair")
        case["recheck"] = dict(time_ms=int(ms),request_uJ=round(float(work)*1e6,3),
                               loss_cap_ms=int(cap), angle_deg=float(angle), speed_dps=float(speed),
                               guarded_before_allocation_ms=int(pre_width),width_at_recorded_cap_ms=int(width),
                               predicted_increment_uJ=round(float(extra)*1e6,3), reserved_ms=int(reserve),
                               passage_zero_ms=int(ref_zero),passage_peak_ms=int(ref_peak),wheel_cap_matched=bool(int(matched)))
    if after_first or event_allocation:
        return dict(stage="first" if after_first else "second", event_allocation=event_allocation, cases=cases)
    return dict(rechecks=cases, stationary=restart_replay(exe, meta, timeseries, epoch, packets, frames_by_id,
                                                          peaks, decisions))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path, help="original RWLOG or converted folder")
    parser.add_argument("--after-first", action="store_true", help="start at the end of each first return aid")
    parser.add_argument("--event-allocation", action="store_true", help="use the updated event allocation policy")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        exe = tmp / "walk6-replay.exe"
        subprocess.run(["g++","-std=c++17","-O2","-Wall","-Wextra","-Werror","-I",ROOT/"src",
                        HERE/"walk7_detectors.cpp","-o",exe], check=True)
        for source in args.inputs:
            folder = source
            if source.is_file():
                folder = tmp / source.stem
                convert(source, folder)
            result = replay(folder, exe, args.after_first, args.event_allocation)
            result["run_start_us"] = json.loads((folder/"header.json").read_bytes())["run_start_us"]
            print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    main()
