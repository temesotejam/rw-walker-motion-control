"""Replay the small-angle-difference rescue on saved control snapshots and ordinary pulse history."""
import argparse
from bisect import bisect_right
import csv
import json
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent


def csv_rows(folder, name):
    with (folder / (name + ".csv")).open(encoding="utf8", newline="") as f:
        return list(csv.DictReader(f))


def replay(folder, exe):
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf8"))
    raw = sorted((f for f in meta["foot_frames"] if f["state_id"] == 3 and f["control_observed_us"]),
                 key=lambda f: f["control_observed_us"])
    epoch, = {f["frame_us"] - f["measurement_time_us"] for f in raw if f["timestamp_valid"]}
    seen = [f["control_observed_us"] for f in raw]
    by_sequence = {f["sequence"]: f for f in raw}
    periodic = csv_rows(folder, "foot_periodic")
    audit = csv_rows(folder, "support_transfer_shadow")
    samples = csv_rows(folder, "timeseries")
    rpm_by_stamp = {int(r["roller_speed_sample_time_us"]): float(r["roller_speed_rpm"]) for r in samples
                    if r["roller_speed_valid"] == "1" and r["roller_speed_sample_time_us"] and r["roller_speed_rpm"]}
    rpm_stamps = sorted(rpm_by_stamp)
    peaks = meta["energy_control_autonomous_peak_events"]
    peak_map = {k["peak_time_ms"]: k for k in peaks}
    zeros = meta["energy_control_autonomous_zero_cross_events"]
    walking = meta["walking_energy"]["records"]
    events = []

    def record(stamp, kind, *values):
        events.append((stamp, kind, kind + " " + " ".join("nan" if v in ("", None) else str(v) for v in values)))

    def camera_approach(frame):
        keys = ("timestamp_valid","frame_valid","zero_ready","left_valid","right_valid",
                "left_control_range_ok","right_control_range_ok")
        return all(frame[k] for k in keys) and 0 < frame["left_deg"]-frame["right_deg"] < 3

    # Reproduce invalidation between decisions wherever a delivered frame or
    # logged motion sample is available. The firmware sees more IMU ticks.
    sample_ages = [(int(r["body_rate_sample_time_us"]), r) for r in samples
                   if r["state_id"]=="3" and r["body_rate_sample_time_us"] and r["gy_dps"]
                   and r["mekf_bias_y_dps"] and r["pitch_mekf_detector_relative_deg"]]
    sample_ages.sort(key=lambda pair: pair[0])
    imu_times = [t for t,_ in sample_ages]
    previous_stamp = previous_peak = None
    for k in peaks:
        record(epoch + 1000*k["peak_telemetry_capture_time_ms"], "K", k["physical_peak_side"], k["peak_time_ms"],
               int(k["pending_command_matched"] and (k["pending_q_command_mA_s"] or 0)>0), k["peak_amplitude_deg"])
    for w in walking:
        if w["return_event"]:
            continue
        matches = [z for z in zeros if z["previous_peak_time_ms"] == w["peak_ms"]
                   and 0 <= w["time_ms"]-z["zero_cross_time_ms"] <= 5 and z["output_executed"] == w["executed"]]
        if not matches:
            assert not w["executed"]
            continue
        z, = matches
        if w["executed"] or (w["coast"] and z["valid"] and not w["optional"]):
            record(z["pre_input_capture_time_us"], "M", w["time_ms"], w["side"], w["requested_ms"], z["pre_input_wheel_speed_rpm"])
        if w["executed"] and w["completion_reason"] == 1 and w["completion_valid"]:
            record(epoch + w["output_end_ms"]*1000, "R", w["side"], int(w["assisted_cycle"]), w["actual_elapsed_ms"])

    for s in audit:
        t, peak, count = (int(s[k]) for k in ("time_ms", "peak_ms", "packet"))
        p = periodic[count-1]
        stamp = epoch + ((int(s["capture_us32"])-epoch) & 0xffffffff)
        fi = bisect_right(seen, stamp)-1
        if fi < 0 or peak not in peak_map:
            continue
        if previous_peak==peak and previous_stamp is not None:
            for frame in raw[bisect_right(seen,previous_stamp):bisect_right(seen,stamp)]:
                if not camera_approach(frame):
                    record(frame["control_observed_us"],"B",peak)
            for sample_t, row in sample_ages[bisect_right(imu_times,previous_stamp):bisect_right(imu_times,stamp)]:
                speed = -(float(row["gy_dps"])-float(row["mekf_bias_y_dps"]))*meta["mekf_gyro_y_scale"]
                body = float(row["pitch_mekf_detector_relative_deg"])
                if not (.35 <= body <= 12 and peak_map[peak]["peak_amplitude_deg"]-body >= .5 and 3 <= speed <= 35):
                    record(sample_t,"B",peak)
        previous_stamp, previous_peak = stamp, peak
        f = raw[fi]
        valid = all(f[k] for k in ("timestamp_valid","frame_valid","zero_ready","left_valid","right_valid",
                                   "left_control_range_ok","right_control_range_ok"))
        sides = [int(p[k+"_sequence"]) for k in ("left","right")]
        images = [by_sequence.get(seq, {}).get("frame_us", 0) for seq in sides]
        # The audit's InvalidFoot additionally requires two finite *slopes*, which
        # the new measured-angle observer does not require. It checks the raw pair.
        active = not int(s["baseline_flags"]) and not (int(s["gate_mask"]) & ~4)
        # Consume known ZERO/aid edges independently of snapshot timing.
        if any(w["peak_ms"] == peak and w["time_ms"] <= t and w["executed"] for w in walking):
            active = False
        if any(z["previous_peak_time_ms"] == peak and z["event_kind"]=="energy_control_zero_cross"
               and z["zero_cross_time_ms"] <= t for z in zeros):
            active = False
        zi = next((z for z in reversed(zeros) if z["zero_cross_time_ms"] < t and z["startup_target_deg"]), None)
        target = zi["startup_target_deg"] if zi else 4.5
        ri = bisect_right(rpm_stamps,stamp)-1
        rpm = rpm_by_stamp[rpm_stamps[ri]] if ri >= 0 and stamp-rpm_stamps[ri] <= 50000 else None
        previous = next((w for w in reversed(walking) if w["time_ms"] < t and w["start_tau_s"]), None)
        tau = previous["start_tau_s"] if previous else None
        record(stamp,"P",t,peak,-int(s["side"]),int(active),stamp & 0xffffffff,count,epoch,
               epoch+int(p["slot_us"]),epoch+int(p["publish_us"]),*sides,*images,
               p["left_observed_deg"],p["right_observed_deg"],f["sequence"],f["frame_us"],
               max(0,f["measurement_time_us"]//1000), max(0,(f["control_observed_us"]-epoch)//1000),int(valid),
               f["left_deg"],f["right_deg"],peak_map[peak]["peak_amplitude_deg"],s["lateral_deg"],s["rate_dps"],
               s["accel_dps2"],s["fore_aft_deg"],target,s["i0_ma"],s["goal_ma"],rpm,tau,int(s["cap_ms"]))
    history = "\n".join(e[2] for e in sorted(events))+"\n"
    result = subprocess.run([exe], input=history, capture_output=True, text=True, check=True)
    print(folder.name, len(audit), len(periodic), result.stdout.strip() or "no new command")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folders", type=Path, nargs="+")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        binary = Path(temp)/"small_arc.exe"
        subprocess.run(["g++","-std=c++17","-O2","-I",HERE.parents[1]/"src",HERE/"replay_small_arc.cpp","-o",binary],check=True)
        for directory in args.folders:
            replay(directory,binary)
