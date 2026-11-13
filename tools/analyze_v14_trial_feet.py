"""Independent foot/body timing review; no firmware or CAD-coordinate inference.

Usage: python tools/analyze_v14_trial_feet.py [TRIAL ...]
Defaults to the first two v14 trials. Generated per-trial JSON is local evidence.
"""
import argparse
import bisect
import csv
import json
import math
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "analysis/2026-11-13-walking"


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def read(folder, name):
    with (folder / name).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def quantile(values, fraction):
    values = sorted(v for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    i = (len(values) - 1) * fraction
    lo = int(i)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (i - lo)


def stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return dict(n=len(values), minimum=min(values) if values else None,
                p05=quantile(values, .05), median=quantile(values, .5), p95=quantile(values, .95),
                maximum=max(values) if values else None,
                p95_minus_p05=None if not values else quantile(values, .95) - quantile(values, .05))


def nominal_potential_mj(angle):
    a = abs(angle) * math.pi / 180
    center = math.sqrt(.150 ** 2 - .005 ** 2)
    dh = (-2 * .120 * math.sin(a / 2) ** 2 + .005 * math.sin(a)) if a <= math.asin(.005 / .150) else (.150 - center) + 2 * (center - .120) * math.sin(a / 2) ** 2
    return 1000 * .1997 * 9.80665 * dh


def analyze(trial):
    folder = BASE / trial
    m = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    feet = read(folder, "foot_angles.csv")
    peaks = read(folder, "energy_control_autonomous_peak_events.csv")
    events = read(folder, "energy_control_autonomous_zero_cross_events.csv")
    energy = read(folder, "walking_energy.csv") if (folder / "walking_energy.csv").exists() else []
    samples = read(folder, "timeseries.csv")
    predictions = read(folder, "foot_pose_prediction.csv")
    local = read(folder, "foot_local_prediction.csv")
    ready = {r["sequence"]: r for r in predictions}
    active_feet = [r for r in feet if num(r["measurement_time_us"]) is not None and 0 <= float(r["measurement_time_us"]) <= 30000000]
    for r in active_feet:
        r["frame_ms"] = float(r["measurement_time_us"]) / 1000
        epoch = float(r["frame_us"]) - float(r["measurement_time_us"])
        r["delivery_ms"] = (float(r["delivered_us"]) - epoch) / 1000
        r["ready_ms"] = num(ready.get(r["sequence"], {}).get("ready_ms"))
        r["control_ms"] = (float(r["control_observed_us"]) - epoch) / 1000 if r["control_observed_us"] else None
    active_samples = [r for r in samples if num(r["t_test_ms"]) is not None and 0 <= float(r["t_test_ms"]) <= 30000 and r["state_id"] == "3"]
    active_peaks = [r for r in peaks if 0 <= float(r["peak_time_ms"]) <= 30000]
    rows = []
    for low, high in ((0, 2000), (2000, 4000), (4000, 6000), (6000, 10000), (10000, 15000), (15000, 20000), (20000, 25000), (25000, 30001)):
        ff = [r for r in active_feet if low <= r["frame_ms"] < high]
        ss = [r for r in active_samples if low <= float(r["t_test_ms"]) < high]
        pp = [r for r in active_peaks if low <= float(r["peak_time_ms"]) < high]
        ee = [r for r in energy if low <= float(r["time_ms"]) < high]
        rows.append(dict(start_ms=low, end_ms=high, frames=len(ff),
                         left_valid=sum(r["left_valid"] == "True" for r in ff), right_valid=sum(r["right_valid"] == "True" for r in ff),
                         left_deg=stats([num(r["left_deg"]) for r in ff if r["left_valid"] == "True"]),
                         right_deg=stats([num(r["right_deg"]) for r in ff if r["right_valid"] == "True"]),
                         body_angle_deg=stats([num(r["pitch_mekf_detector_relative_deg"]) for r in ss]),
                         body_rate_dps=stats([num(r["physical_roll_rate_dps"]) for r in ss]),
                         peak_plus_deg=stats([float(r["peak_amplitude_deg"]) for r in pp if r["physical_peak_side"] == "1"]),
                         peak_minus_deg=stats([float(r["peak_amplitude_deg"]) for r in pp if r["physical_peak_side"] == "-1"]),
                         energy_decisions=len(ee), executed=sum(r["executed"] == "True" for r in ee),
                         width_counts=dict(Counter(r["guarded_ms"] for r in ee))))
    latencies = dict(frame_to_delivery_ms=stats([(float(r["delivered_us"]) - float(r["frame_us"])) / 1000 for r in active_feet]),
                     recognition_processing_ms=stats([float(r["processing_us"]) / 1000 for r in active_feet]),
                     frame_to_prediction_ready_ms=stats([r["ready_ms"] - r["frame_ms"] for r in active_feet if r["ready_ms"] is not None]),
                     frame_to_control_observation_ms=stats([r["control_ms"] - r["frame_ms"] for r in active_feet if r["control_ms"] is not None]))
    interesting = sorted(active_peaks, key=lambda r: float(r["peak_amplitude_deg"]))[:10]
    joins = []
    for peak in interesting:
        t = float(peak["peak_time_ms"])
        frame = min(active_feet, key=lambda r: abs(r["frame_ms"] - t))
        causal = [r for r in active_feet if r["ready_ms"] is not None and r["ready_ms"] <= t]
        last = max(causal, key=lambda r: r["ready_ms"]) if causal else None
        joins.append(dict(peak_ms=t, side=int(peak["physical_peak_side"]), amplitude_deg=float(peak["peak_amplitude_deg"]),
                          nearest_frame_ms=frame["frame_ms"], nearest_left_deg=num(frame["left_deg"]), nearest_right_deg=num(frame["right_deg"]),
                          latest_ready_frame_ms=last["frame_ms"] if last else None,
                          latest_ready_left_deg=num(last["left_deg"]) if last else None,
                          latest_ready_right_deg=num(last["right_deg"]) if last else None))
    forward_extremes = {}
    for side in ("left", "right"):
        valid = [r for r in active_feet if r[f"{side}_valid"] == "True"]
        ranked = sorted(valid, key=lambda r: float(r[f"{side}_deg"]), reverse=True)[:8]
        forward_extremes[side] = [dict(frame_ms=r["frame_ms"], angle_deg=float(r[f"{side}_deg"]), ready_ms=r["ready_ms"], sequence=r["sequence"]) for r in ranked]
    foot_metadata = m.get("foot_support_assist", {})
    counts = foot_metadata.get("candidate_check_counts", [])
    names = foot_metadata.get("reason_names", [])
    active_predictions = [r for r in predictions if num(r["ready_ms"]) is not None and 0 < float(r["ready_ms"]) <= 30000]
    active_local = [r for r in local if num(r["ready_ms"]) is not None and 0 < float(r["ready_ms"]) <= 30000]
    comparisons = {}
    for label, low, high in (("early_swing_2_to6s", 2000, 6000), ("late_10_to30s", 10000, 30001)):
        ff = [r for r in active_feet if low <= r["frame_ms"] < high]
        pp = [r for r in active_peaks if low <= float(r["peak_time_ms"]) < high]
        comparisons[label] = dict(left_deg=stats([num(r["left_deg"]) for r in ff if r["left_valid"] == "True"]),
                                 right_deg=stats([num(r["right_deg"]) for r in ff if r["right_valid"] == "True"]),
                                 plus_peak_deg=stats([float(r["peak_amplitude_deg"]) for r in pp if r["physical_peak_side"] == "1"]),
                                 minus_peak_deg=stats([float(r["peak_amplitude_deg"]) for r in pp if r["physical_peak_side"] == "-1"]))
    threshold_last = {}
    for side in ("left", "right"):
        forward = [r for r in active_feet if r[f"{side}_valid"] == "True" and float(r[f"{side}_deg"]) >= 1.5]
        threshold_last[side] = dict(count=len(forward), last_frame_ms=forward[-1]["frame_ms"] if forward else None)
    peak_gaps = [float(b["peak_time_ms"]) - float(a["peak_time_ms"]) for a, b in zip(active_peaks, active_peaks[1:])]
    decision_ages = []
    for event in energy:
        t = float(event["time_ms"])
        candidates = [r for r in active_feet if r["ready_ms"] is not None and r["ready_ms"] <= t]
        if candidates:
            last = max(candidates, key=lambda r: r["ready_ms"])
            decision_ages.append(t - last["frame_ms"])
    timing_examples = []
    example_sequences = {"172594684": ("1304", "1325", "1340"), "294107213": ("1940",)}
    for seq in example_sequences.get(trial, ()):
        matches = [r for r in active_feet if r["sequence"] == seq]
        if not matches:
            continue
        f = matches[0]
        t = f["ready_ms"]
        s = min(active_samples, key=lambda r: abs(float(r["t_test_ms"]) - t))
        preceding = [r for r in events if float(r["zero_cross_time_ms"]) <= t]
        last_event = max(preceding, key=lambda r: float(r["zero_cross_time_ms"])) if preceding else None
        timing_examples.append(dict(sequence=seq, frame_ms=f["frame_ms"], ready_ms=t, left_deg=num(f["left_deg"]), right_deg=num(f["right_deg"]),
                                    body_sample_ms=float(s["t_test_ms"]), body_angle_deg=float(s["pitch_mekf_detector_relative_deg"]),
                                    body_rate_dps=float(s["physical_roll_rate_dps"]),
                                    preceding_event_ms=float(last_event["zero_cross_time_ms"]) if last_event else None,
                                    preceding_event_kind=last_event["event_kind"] if last_event else None,
                                    preceding_executed=last_event["output_executed"] if last_event else None))
    result = dict(trial=trial, firmware=m.get("diagnostic_build_id"), total_foot_frames=len(feet), active_foot_frames=len(active_feet),
                  duration_ms=30000, valid_left=sum(r["left_valid"] == "True" for r in active_feet),
                  valid_right=sum(r["right_valid"] == "True" for r in active_feet),
                  zero_ready_count=sum(r["zero_ready"] == "True" for r in active_feet),
                  foot_prediction_ready_reason_counts=dict(Counter(r["reason"] for r in active_predictions)),
                  local_prediction_reason_counts=dict(Counter(r["reason"] for r in active_local)),
                  latency=latencies, windows=rows, weakest_peak_foot_joins=joins, forward_extremes=forward_extremes,
                  early_late_comparison=comparisons, forward_above_1p5_deg=threshold_last,
                  latest_completed_frame_age_at_energy_decisions_ms=stats(decision_ages), delayed_foot_timing_examples=timing_examples,
                  nominal_potential_of_late_median_peak_mj={side: nominal_potential_mj(comparisons['late_10_to30s'][side+'_peak_deg']['median']) for side in ('plus','minus')},
                  nominal_target8_potential_mj=nominal_potential_mj(8),
                  accepted_peak_sequence=dict(n=len(active_peaks), gap_ms=stats(peak_gaps),
                                              repeated_side_pairs=sum(a["physical_peak_side"] == b["physical_peak_side"] for a, b in zip(active_peaks, active_peaks[1:]))),
                  foot_assist=dict(stored=foot_metadata.get("stored"), total=foot_metadata.get("total"),
                                   reason_counts={names[i] if i < len(names) else str(i): n for i, n in enumerate(counts) if n}),
                  energy_counts=dict(records=len(energy), optional=sum(r["optional"] == "True" for r in energy),
                                     rescue=sum(r["rescue"] == "True" for r in energy),
                                     executed=sum(r["executed"] == "True" for r in energy),
                                     coast=sum(r["coast"] == "True" for r in energy),
                                     saturated=sum(r["saturated"] == "True" for r in energy),
                                     widths=dict(Counter(r["guarded_ms"] for r in energy)),
                                     kinds=dict(Counter(r["event_kind"] for r in events))),
                  neutral_interpretation="Foot-relative angular excursions can be compared in time, but there is no calibrated camera/CAD absolute zero or CAD-roll/MEKF mapping. No absolute theta_E, barrier work, contact or forward displacement is identified by these rows. A near-constant relative foot angle does not establish zero energy-neutral offset.",
                  observed_motion_interpretation="Body peaks remain alternating while recorded foot excursions shrink. This establishes continuing lateral rocking, not continuing walking. After the brief initial large foot-angle episodes, no foot-timing assist is selected. External synchronized video is needed for alternating ground steps and forward progress.",
                  latency_interpretation="Use prediction ready_ms and control_observed_us for causal availability. delivered_us precedes recognition processing, so frame-to-delivery alone understates foot-angle age.")
    (folder / "foot_energy_review.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(dict(trial=trial, foot_assist=result["foot_assist"], energy_counts=result["energy_counts"], latency=latencies,
                          windows=[{k:v for k,v in r.items() if k in ("start_ms","end_ms","left_deg","right_deg","peak_plus_deg","peak_minus_deg","width_counts")} for r in rows]), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", nargs="*", default=["172594684", "294107213"])
    for trial in parser.parse_args().trials:
        analyze(trial)
