"""Reproduce the failure review's history distances and ordinary-width shadow.

This preserves the analysis harness used for failure_review.json. It imports
the existing replay tool and compiles the real portable C++ headers. Source
hashes must match the reviewed baseline unless --allow-changed-source is given.
No firmware, CSV, or existing report is modified. --out writes a NEW JSON only.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import importlib.util
import inspect
import json
import math
import subprocess
import tempfile

OUT = Path(__file__).resolve().parent
ROOT = Path(__file__).resolve().parents[3]


def read(path):
    with path.open(encoding="utf8", newline="") as stream:
        return list(csv.DictReader(stream))


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, list):
        return [clean(v) for v in value]
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    return value


def history_distances():
    """The same Python reconstruction of the reviewed C++ match formula."""
    feet = read(OUT / "foot_angles.csv")
    saved = {r["sequence"]: r for r in read(OUT / "foot_pose_prediction.csv")}
    samples = [r for r in read(OUT / "timeseries.csv") if r["state_id"] == "3"]
    history, explained = [], []

    def good(row):
        return row["state_id"] == "3" and int(row["measurement_time_us"]) >= 0 and all(
            row[k] == "True" for k in ["timestamp_valid", "frame_valid", "zero_ready",
                                      "left_valid", "right_valid", "left_in_range", "right_in_range"])

    def future_ok(k, target):
        for j in range(k + 1, len(history)):
            a, b = history[j - 1], history[j]
            if not b["good"] or b["t"] <= a["t"] or b["t"] - a["t"] > 200:
                return False
            if target <= b["t"]:
                return True
        return False

    for row in feet:
        if int(row["measurement_time_us"]) < 0:
            continue
        current = {"t": int(row["measurement_time_us"]) // 1000,
                   "l": float(row["left_deg"] or "nan"), "r": float(row["right_deg"] or "nan"),
                   "good": good(row)}
        history.append(current)
        history = history[-48:]
        if not 27450 < current["t"] < 28160:
            continue
        options, excluded = [], {}
        for k in range(2, len(history) - 1):
            b = history[k]
            lag = current["t"] - b["t"]
            reason = None
            if not 800 <= lag <= 4000:
                reason = "lag_outside_800_4000ms"
            elif not all(a["good"] and old["good"] and
                         abs((current["t"] - a["t"]) - (b["t"] - old["t"])) <= 35
                         for a, old in zip(history[-3:], history[k - 2:k + 1])):
                reason = "triplet_invalid_or_relative_timing"
            elif not future_ok(k, b["t"] + 200):
                reason = "historical_future_200ms_not_valid"
            if reason:
                excluded[reason] = excluded.get(reason, 0) + 1
                continue
            distance = sum((a[side] - old[side]) ** 2
                           for a, old in zip(history[-3:], history[k - 2:k + 1])
                           for side in ["l", "r"]) / 6
            options.append({"candidate_frame_ms": b["t"],
                            "mean_squared_triplet_distance_deg2": distance,
                            "triplet_ms_left_right": [[v["t"], v["l"], v["r"]] for v in history[k - 2:k + 1]]})
        options.sort(key=lambda v: v["mean_squared_triplet_distance_deg2"])
        saved_row = saved[row["sequence"]]
        slopes = {side: [(history[j][side] - history[j - 1][side]) * 1000 /
                         (history[j]["t"] - history[j - 1]["t"])
                         for j in [len(history) - 2, len(history) - 1]] for side in ["l", "r"]}
        body = min(samples, key=lambda s: abs(float(s["t_test_ms"]) - float(saved_row["ready_ms"])))
        explained.append({"frame_ms_floor": current["t"], "frame_measurement_s": int(row["measurement_time_us"]) / 1e6,
                          "sequence": int(row["sequence"]), "both_feet_valid_inrange": current["good"],
                          "ready_ms": int(saved_row["ready_ms"]), "reason": int(saved_row["reason"]),
                          "confidence": float(saved_row["confidence"]), "right_phase": int(saved_row["right_phase"]),
                          "right_uncertainty": float(saved_row["right_uncertainty_deg"] or "nan"),
                          "current_triplet_ms_left_right": [[v["t"], v["l"], v["r"]] for v in history[-3:]],
                          "local_interval_slopes_dps": slopes, "nearest_historical_candidates": options[:3],
                          "accepted_candidates_mse_lt_half": sum(v["mean_squared_triplet_distance_deg2"] < .5 for v in options),
                          "excluded_counts": excluded,
                          "body_at_availability": {k: body[k] for k in ["t_test_ms", "pitch_mekf_detector_relative_deg",
                                                                        "physical_roll_rate_dps", "pulse_active"]}})
    return explained


def controller_shadow():
    specification = importlib.util.spec_from_file_location("failure_replay", ROOT / "tools/replay_foot_support_assist.py")
    replay = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(replay)
    # These are the exact two replay-adapter changes used in the review:
    # saved availability timestamps instead of estimated latency, and lead 0.
    source = inspect.getsource(replay.replay).replace(
        "feet=read(path/'foot_angles.csv');samples=read(path/'timeseries.csv')",
        "feet=read(path/'foot_angles.csv');samples=read(path/'timeseries.csv');saved_pose={int(r['sequence']):r for r in read(path/'foot_pose_prediction.csv')}")
    old_ready = "received=(int(f['delivered_us'])+int(f['processing_us'])+cad.get(int(f['sequence']),0)-epoch+999)//1000"
    assert old_ready in source
    source = source.replace(old_ready, "received=int(saved_pose[int(f['sequence'])]['ready_ms'])")
    exec(source, replay.__dict__)
    cpp_source = replay.CPP.replace("pose_lead=30", "pose_lead=0").replace(
        "/*ANTICIPATED_ASSIGN*/", "in.anticipated_shape_need=anticipated;")
    original_output = "<<in.normal_width_ms<<'\\n';"
    reviewed_output = "<<in.normal_width_ms<<' '<<controller.widthLimit(-peak_side)<<' '<<controller.ordinaryRiseCap(-peak_side,now,150)<<'\\n';"
    assert original_output in cpp_source
    cpp_source = cpp_source.replace(original_output, reviewed_output)
    with tempfile.TemporaryDirectory(prefix="failure_tilt_") as temporary:
        temporary = Path(temporary)
        cpp, exe = temporary / "r.cpp", temporary / "r.exe"
        cpp.write_text(cpp_source, encoding="utf8")
        subprocess.run(["g++", "-std=c++11", "-O2", "-I" + str(ROOT / "src"), str(cpp), "-o", str(exe)], check=True)
        original_run, captured = replay.subprocess.run, []

        def capture(*args, **kwargs):
            result = original_run(*args, **kwargs)
            if "input" in kwargs:
                captured.append((kwargs["input"], result.stdout))
            return result

        try:
            replay.subprocess.run = capture
            result = replay.replay(exe, OUT, 0, 5)
        finally:
            replay.subprocess.run = original_run
    queries = {int(line.split()[1]): line.split() for line in captured[0][0].splitlines() if line.startswith("2 ")}
    rows = []
    for line in captured[0][1].splitlines():
        values = line.split()
        query = queries[int(values[0])]
        now = int(query[3])
        if values[1] != "0" or not 27622 < now < 27958:
            continue
        distance, speed = -float(query[4]), float(query[5])
        lead = distance * 1000 / speed if speed > 0 else None
        rows.append({"time_ms": now, "body_angle_deg": float(query[4]), "inward_rate_dps": speed,
                     "first_selector_reason": replay.REASONS[int(values[2])], "confidence": float(values[7]),
                     "frame_age_ms": int(values[10]), "left_deg": float(values[5]), "right_deg": float(values[6]),
                     "ordinary_proposal_ms": int(values[22]), "ordinary_side_width_limit_ms": int(values[23]),
                     "ordinary_rise_cap_ms": int(values[24]), "ordinary_coast": values[16] == "1",
                     "target_deg": float(values[17]), "decelerating": values[18] == "1",
                     "constant_rate_zero_lead_ms": lead,
                     "independent_nonpose_gates": {"normal_width_le_60": int(values[22]) <= 60,
                                                  "speed_6_45": 6 <= speed <= 45,
                                                  "estimated_zero_lead_le250": lead is not None and lead <= 250,
                                                  "estimated_zero_lead_ge_width_plus20": lead is not None and lead >= int(values[22]) + 20}})
    return {"saved_pose_ready_ms_used": True, "peak_target_checks": result["recorded_peak_target_checks"],
            "target_mismatches": result["shadow_target_mismatches"], "window_before_actual_rescue": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-changed-source", action="store_true", help="Explicitly evaluate current sources after the reviewed baseline changed.")
    parser.add_argument("--out", type=Path, help="Write a new JSON artifact; existing files are refused.")
    args = parser.parse_args()
    baseline = json.loads((OUT / "failure_review.json").read_text(encoding="utf8"))
    current_hashes = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in baseline["source_sha256"]}
    changed = [name for name, digest in current_hashes.items() if digest != baseline["source_sha256"][name]]
    if changed and not args.allow_changed_source:
        parser.error("Reviewed sources changed: " + ", ".join(changed) + ". Use --allow-changed-source for an explicit new-source comparison.")
    data = clean({"source_sha256": current_hashes, "changed_source_files": changed,
                  "history_candidate_reconstruction": history_distances(), "ordinary_controller_shadow": controller_shadow()})
    if args.out:
        with args.out.open("x", encoding="utf8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
    shadow = data["ordinary_controller_shadow"]
    rows = shadow["window_before_actual_rescue"]
    print("peak target checks/mismatches:", shadow["peak_target_checks"], shadow["target_mismatches"])
    print("ordinary proposal / side limit / rise cap:", sorted(set((r["ordinary_proposal_ms"], r["ordinary_side_width_limit_ms"], r["ordinary_rise_cap_ms"]) for r in rows)))
    print("reviewed query window:", rows[0]["time_ms"], rows[-1]["time_ms"], "count", len(rows))
    print("first constant-rate lead >250ms:", next((r["time_ms"] for r in rows if r["constant_rate_zero_lead_ms"] > 250), None))
    for frame in data["history_candidate_reconstruction"]:
        best = frame["nearest_historical_candidates"]
        print("frame/ready/confidence/best-distance:", frame["frame_ms_floor"], frame["ready_ms"], frame["confidence"], best[0]["mean_squared_triplet_distance_deg2"] if best else None)


if __name__ == "__main__":
    main()
