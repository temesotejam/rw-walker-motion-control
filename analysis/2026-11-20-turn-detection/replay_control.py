"""Compare logged and proposed ordinary ZERO pulses against the same real pre-command state."""
import argparse
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import subprocess
import tempfile

from replay import HERE, ROOT, observations, trial_id


def control_inputs(folder, cycles):
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf8"))
    run = json.loads((folder / "header.json").read_text(encoding="utf8"))["run_id"]
    if meta["energy_control_autonomous_target_peak_deg"] != 8 or meta["autonomous_input_peak_percent"] != 0:
        raise ValueError("Replay is for the 8 degree, zero-crossing walking trials")
    _, timeseries, log_clock, peaks, _ = observations(folder)
    turns = {int(c["end_ms"]): c for c in cycles}
    events, ground = [], {}

    def add(clock, order, event, *fields):
        tokens = [str(int(x)) if isinstance(x, bool) else str(x) if x is not None else "nan" for x in fields]
        events.append((clock, order, event + " " + " ".join(tokens)))

    for p, clock, _ in peaks:
        matched = p["pending_command_matched"] and (p["pending_q_command_mA_s"] or 0) > 0
        add(clock, 0, "K", p["physical_peak_side"], p["peak_time_ms"], matched, p["peak_amplitude_deg"])
        c = turns.get(p["peak_telemetry_capture_time_ms"])
        if c:
            add(clock, 1, "C", *(c[k] for k in ("start_ms", "end_ms", "reason", "weak_side", "turn_sign",
                                                "left_swing_deg", "right_swing_deg")))
    frames = sorted((f for f in meta["foot_frames"] if f.get("control_observed_us") and f["state_id"] == 3),
                    key=lambda f: f["control_observed_us"])
    frame_clock = [f["control_observed_us"] for f in frames]
    last_raw = ("timestamp_valid", "frame_valid", "zero_ready", "left_valid", "right_valid",
                "left_control_range_ok", "right_control_range_ok")
    for r in meta["walking_energy"]["records"]:
        if r["return_event"] or not r["valid"]:
            continue
        t, side = r["time_ms"], r["side"]
        kind = "angle_rate_early_trial" if r["optional"] else "energy_control_zero_cross"
        matches = [z for z in meta["energy_control_autonomous_zero_cross_events"]
                   if z["event_kind"] == kind and z["previous_peak_time_ms"] == r["peak_ms"]
                   and 0 <= t - z["zero_cross_time_ms"] <= 5 and z["output_executed"] == r["executed"]]
        if len(matches) != 1:
            if r["executed"]:
                raise ValueError(f"Cannot match an executed pulse at {t}")
            continue
        e, = matches
        if not e["valid"]:  # Optional candidates often fail after the work calculation.
            continue
        now, rpm = e["pre_input_capture_time_us"], e["pre_input_wheel_speed_rpm"]
        if not r["optional"] and not r["coast"] and r["executed"]:
            latest = [p for p, clock, _ in peaks if clock <= now and p["physical_peak_side"] == side
                      and t - p["peak_time_ms"] <= 3000]
            same_side = latest[-1]["peak_amplitude_deg"] if latest else None
            fi, hi = bisect_right(frame_clock, now) - 1, bisect_right(log_clock, now) - 1
            raw = frames[fi] if fi >= 0 else {}
            fresh = (bool(raw) and all(raw[k] for k in last_raw)
                     and math.isfinite(raw["left_deg"]) and math.isfinite(raw["right_deg"])
                     and 0 < raw["frame_us"] <= raw["control_observed_us"] <= now
                     and now - raw["frame_us"] <= 75000 and raw["run_id"] == run)
            eligible = fresh and hi >= 0 and timeseries[hi]["gyro_heading_valid"] == "1" and not r["assisted_cycle"]
            add(now, 2, "M", side, t, eligible, e["work_brake_cap_ms"], r["start_rate_dps"],
                e["detector_angle_after_deg"], e["startup_target_deg"], e["previous_peak_amplitude_deg"],
                same_side, e["aligned_initial_current_mA"], e["predicted_current_goal_mA"], r["arrival_bias_j"],
                r["start_tau_s"], rpm)
            ground[t] = (r, eligible)
        if r["executed"]:
            add(now, 3, "E", side, t, r["requested_ms"], rpm)
    return "".join(line + "\n" for _, _, line in sorted(events)), ground


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("converted", type=Path, nargs="+")
    parser.add_argument("--cycles", type=Path, default=HERE / "cycles.csv")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    with args.cycles.open(newline="", encoding="utf8") as f:
        cycles = list(csv.DictReader(f))
    out = []
    with tempfile.TemporaryDirectory() as work:
        exe = Path(work) / "control-replay.exe"
        subprocess.run(["g++", "-std=c++17", "-O2", "-I", str(ROOT / "src"), str(HERE / "replay_control.cpp"),
                        "-o", str(exe)], check=True)
        for folder in args.converted:
            label = trial_id(folder)
            history = [r for r in cycles if r["trial"] == label]
            if not history:
                raise ValueError(f"No cycle decisions for {label}")
            payload, ground = control_inputs(folder, history)
            result = subprocess.check_output([str(exe)], input=payload, text=True)
            choices = []
            for line in result.splitlines():
                t, requested, old, request_j, base_work, delta, candidate, unmet, candidate_work, new, work = line.split()
                r, eligible = ground[int(t)]
                if (int(requested), int(old)) != (r["requested_ms"], r["guarded_ms"]):
                    raise ValueError(f"Baseline inverse/guard mismatch {label} {t}: {line} vs {r['requested_ms']}/{r['guarded_ms']}")
                if abs(float(request_j) - r["request_j"]) > 2e-7 or abs(float(base_work) - r["predicted_work_j"]) > 2e-7:
                    raise ValueError(f"Baseline energy mismatch {label} {t}: {line}")
                if int(old) != int(new):
                    choices.append((t, old, new))
                out.append(dict(trial=label, time_ms=t, side=r["side"], input_gates_ok=int(eligible),
                                base_requested_ms=requested, base_ms=old, base_work_j=base_work,
                                adjustment_j=delta, candidate_ms=candidate, candidate_unmet_j=unmet,
                                candidate_work_j=candidate_work, new_ms=new, new_work_j=work))
            if len(result.splitlines()) != len(ground):
                raise ValueError("An ordinary ZERO was omitted by the host adapter")
            print(label, "ordinary_driven", len(ground), "changed", choices)
    if args.out:
        with args.out.open("w", encoding="utf8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=out[0])
            writer.writeheader()
            writer.writerows(out)


if __name__ == "__main__":
    main()
