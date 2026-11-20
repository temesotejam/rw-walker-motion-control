"""Replay one proposed extension at each actual rescue, conditional on the recorded history."""
import argparse
import csv
import json
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FIELDS = ("time_ms packet_age_us source left_est right_est left_rate right_rate shape_reason extra_j timing_cap "
          "old_raw old_guarded candidate_raw candidate_guarded reserve_ms new_ms").split()


def replay(folder, exe, *, compare_pivot=False):
    m = json.loads((folder/"metadata.json").read_text(encoding="utf8"))
    active = [f for f in m["foot_frames"] if f["state_id"] == 3 and f["control_observed_us"]]
    epoch, = {f["frame_us"]-f["measurement_time_us"] for f in active if f["timestamp_valid"]}
    plan = m["walking_energy"]
    assert plan["archive_available"] and not plan["truncated"] and len(plan["records"]) == plan["total"]
    walking = plan["records"]
    arc_rows = {row["row_index"]: row for row in plan.get("arc_aids", [])}
    zeros = m["energy_control_autonomous_zero_cross_events"]
    periodic = m["foot_periodic"]["samples"]
    assert len({f["run_id"] for f in active}) == 1
    assert len(periodic) == m["foot_periodic"]["count"]
    events, ground = [], {}

    def add(time, kind, *fields):
        values = ["nan" if v is None else str(int(v)) if isinstance(v, bool) else str(v) for v in fields]
        events.append((time, kind, kind+" "+" ".join(values)))

    for f in active:
        add(f["control_observed_us"], "F", f["sequence"], f["frame_us"], f["control_observed_us"],
            f["timestamp_valid"] and f["zero_ready"], f["left_valid"], f["left_control_range_ok"], f["left_deg"],
            f["right_valid"], f["right_control_range_ok"], f["right_deg"])
    for p in periodic:
        add(epoch+p[1], "P", epoch+p[1])
    for p in m["energy_control_autonomous_peak_events"]:
        matched = p["pending_command_matched"] and (p["pending_q_command_mA_s"] or 0)>0
        add(epoch+p["peak_telemetry_capture_time_ms"]*1000, "K", p["physical_peak_side"], p["peak_time_ms"],
            matched, p["peak_amplitude_deg"])
    for index, r in enumerate(walking):
        t, side, peak = r["time_ms"], r["side"], r["peak_ms"]
        # ZERO is interpolated between samples. The walking record timestamps the
        # control tick that acted on that detection, up to one sample later.
        matches = [e for e in zeros if e["previous_peak_time_ms"] == peak
                   and 0 <= t-e["zero_cross_time_ms"] <= 5 and e["output_executed"] == r["executed"]]
        if not matches:
            if r["executed"]:
                raise ValueError("Missing executed event")
            continue
        e, = matches
        if not r["return_event"]:
            if r["executed"] or (r["coast"] and e["valid"] and not r["optional"]):
                add(e["pre_input_capture_time_us"], "M", side, t, r["executed"], r["requested_ms"],
                    e["pre_input_wheel_speed_rpm"], e["target_peak_deg"])
            if r["executed"] and r["completion_reason"] == 1 and r["completion_valid"]:
                add(epoch+r["output_end_ms"]*1000, "E", side, r["assisted_cycle"], r["actual_elapsed_ms"])
            continue
        if not (r["rescue"] and r["executed"]):
            continue
        now = e["pre_input_capture_time_us"]
        # The raw frame identity is uniquely determined by the stored integer frame age.
        raw, = [f for f in active if f["timestamp_valid"] and f["measurement_time_us"]//1000 == t-r["return_foot_age_ms"]
                and f["control_observed_us"] <= now]
        pkt = r["periodic_foot_packet"]
        p = periodic[pkt-1]
        fresh = r["return_evidence"] == 1 and r["return_foot_age_ms"] <= 75
        add(now, "R", t, now, pkt, p[2], p[3], p[4], p[5], raw["sequence"], raw["frame_us"],
            raw["control_observed_us"], r["envelope_before_ms"], fresh, side, e["detector_angle_after_deg"],
            r["start_rate_dps"], r["return_deceleration_dps2"], r["fore_aft_angle_deg"], raw["left_deg"],
            raw["right_deg"], r["return_work_j"] if compare_pivot else r["request_j"], r["target_j"],
            e["aligned_initial_current_mA"] if compare_pivot else r["start_i0_ma"],
            e["predicted_current_goal_mA"] if compare_pivot else r["start_goal_ma"],
            r["start_tau_s"], e["pre_input_wheel_speed_rpm"])
        main = [c for c in walking if c["peak_ms"]==peak and c["executed"] and not c["return_event"] and c["time_ms"]>t]
        crossed = any(z["previous_peak_time_ms"]==peak and z["event_kind"]=="energy_control_zero_cross"
                      and z["zero_cross_time_ms"]>t for z in zeros)
        ground[t] = (r, crossed, sum(c["actual_elapsed_ms"] for c in main), arc_rows.get(index, {}))
    payload = "\n".join([str(epoch)] + [line for _,_,line in sorted(events)])
    result = subprocess.check_output([exe], text=True, input=payload+"\n")
    rows = []
    for line in result.splitlines():
        vals = line.split()
        fields = FIELDS + (["old_pivot", "old_choice", "tail_proof", "start_clearance_mm",
                            "tail_clearance_mm", "end_clearance_mm", "horizon_s", "new_work_j"] if compare_pivot else [])
        if len(vals) != len(fields):
            raise ValueError(line)
        row = dict(zip(fields, vals))
        r, crossed, later, arc = ground[int(row["time_ms"])]
        if compare_pivot:
            expected = arc.get("arc_base_guarded_ms", r["guarded_ms"])
            if int(row["old_guarded"]) != expected or (arc and int(row["old_choice"]) != r["guarded_ms"]):
                raise ValueError(f"Wheel-limited base or original arc choice differs: {folder.name} {row}")
        elif (int(row["old_raw"]), int(row["old_guarded"])) != (r["requested_ms"], r["guarded_ms"]):
            raise ValueError(f"Baseline width mismatch: {folder.name} {row}")
        row.update(trial=folder.name, observed_zero=int(crossed), later_main_ms=later,
                   remaining_ms=r["envelope_before_ms"]-int(row["new_ms"]))
        rows.append(row)
    assert len(rows) == len(ground)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("folders", type=Path, nargs="+", help="Directories produced by convert_rwlog_to_csv.py")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        exe = Path(temp)/"replay_aid.exe"
        subprocess.run(["g++","-std=c++17","-O2","-I",ROOT/"src",HERE/"replay_aid.cpp","-o",exe],check=True)
        rows = [r for folder in args.folders for r in replay(folder,exe)]
    print("trial          time_ms  source  old_guard  candidate  reserve  selected  remaining  next_main")
    for row in rows:
        print(row["trial"].removeprefix("control_trial_").rjust(9),
              *(str(row[key]).rjust(8) for key in ("time_ms", "source", "old_guarded", "candidate_guarded",
                                               "reserve_ms", "new_ms", "remaining_ms", "later_main_ms")))
    if args.out:
        with args.out.open("w",encoding="utf8",newline="") as f:
            w=csv.DictWriter(f, fieldnames=["trial",*FIELDS,"observed_zero","later_main_ms","remaining_ms"])
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
