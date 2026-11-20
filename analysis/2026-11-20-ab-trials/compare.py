"""Compare recorded startup predictions, peaks and the return from each high positive peak."""
import argparse
import csv
import json
from pathlib import Path
from statistics import median


def read_rows(folder, name):
    with (folder / (name + ".csv")).open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def right_startup_trials(metadata):
    peaks = metadata["energy_control_autonomous_peak_events"]
    for event in metadata["energy_control_autonomous_zero_cross_events"]:
        if (event["event_kind"] != "energy_control_zero_cross" or not event["output_executed"]
                or event["physical_next_peak_side"] != 1 or event["startup_target_deg"] != 4.5
                or event["zero_cross_time_ms"] >= 6000):
            continue
        peak = next((p for p in peaks if p["peak_time_ms"] > event["pulse_start_ms"]), None)
        prediction = event["predicted_next_peak_amplitude_deg"]
        if peak and peak["physical_peak_side"] == 1 and prediction is not None:
            yield (event["pulse_start_ms"], event["pulse_width_ms"], prediction,
                   peak["peak_amplitude_deg"], peak["peak_amplitude_deg"] - prediction)


def arrival1_outcomes(metadata):
    peaks = metadata["energy_control_autonomous_peak_events"]
    events = metadata["energy_control_autonomous_zero_cross_events"]
    for row in metadata["walking_energy"]["records"]:
        if not row.get("right_arrival_checked"):
            continue
        zero = row["time_ms"]
        right_index = next((i for i, p in enumerate(peaks) if p["peak_time_ms"] > zero), None)
        right = peaks[right_index] if right_index is not None else None
        if right and (right["physical_peak_side"] != 1 or right["peak_time_ms"] - zero > 750):
            right = None
        if right and any(e["output_executed"] and e["previous_peak_time_ms"] != row["peak_ms"]
                         and zero < e["zero_cross_time_ms"] < right["peak_time_ms"] for e in events):
            right = None
        left = peaks[right_index + 1] if right and right_index + 1 < len(peaks) else None
        final_j = row.get("right_arrival_final_j")
        print("right arrival: ms, base/limit/final ms, executed, reason, selected arrival mJ, observed right/next left deg",
              zero, row["right_arrival_base_ms"], row["right_arrival_limit_ms"], row["guarded_ms"],
              row["executed"], row["right_arrival_reason"], round(1000 * final_j, 4) if final_j is not None else None,
              right["peak_amplitude_deg"] if right else None,
              -left["peak_amplitude_deg"] if left and left["physical_peak_side"] == -1 else None)


def return_motion(folder, metadata):
    imu = [r for r in read_rows(folder, "timeseries") if r["state_id"] == "3"]
    foot = read_rows(folder, "foot_periodic")
    peaks = metadata["energy_control_autonomous_peak_events"]
    for i, peak in enumerate(peaks):
        if peak["physical_peak_side"] != 1 or peak["peak_amplitude_deg"] < 7:
            continue
        start = peak["peak_time_ms"]
        # Include the first inward swing, ending at the next signed peak when recorded.
        end = min(start + 650, peaks[i + 1]["peak_time_ms"] if i + 1 < len(peaks) else start + 650)
        window = [r for r in imu if start <= int(r["t_test_ms"]) <= end]
        if not window:
            continue
        angle = "pitch_mekf_detector_relative_deg"
        low = min(window, key=lambda r: float(r[angle]))
        probes = [(str(limit), next((r for r in window if float(r[angle]) <= limit), None))
                  for limit in (2.8, 1.7)]
        if float(low[angle]) > 0:
            probes.append(("first inward minimum", low))
        for label, row in probes:
            if row is None:
                continue
            t = int(row["t_test_ms"])
            if not foot:
                continue
            slot = min(foot, key=lambda f: abs(int(f["slot_us"]) - t * 1000))
            print(start, label, t, row[angle], row["physical_roll_rate_dps"],
                  slot["left_observed_deg"], slot["right_observed_deg"])
    tail = [r for r in foot if 25_000_000 <= int(r["slot_us"]) < 30_000_000]
    medians = []
    for key in ("left_observed_deg", "right_observed_deg"):
        observed = [float(r[key]) for r in tail if r[key]]
        medians.append(median(observed) if observed else None)
    print("25-30 s left/right foot median (deg)", *medians)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folders", type=Path, nargs="+", help="Converted RWLOG folders")
    parser.add_argument("--returns", action="store_true", help="Inspect returns in these recordings")
    args = parser.parse_args()
    errors = []
    for folder in args.folders:
        with (folder / "metadata.json").open(encoding="utf-8") as file:
            metadata = json.load(file)
        print(folder.name, metadata["diagnostic_build_id"], metadata.get("walking_profile"))
        if metadata.get("walking_profile") == "right_arrival_v1":
            arrival1_outcomes(metadata)
        for row in right_startup_trials(metadata):
            print("positive ZERO: ms, width, predicted, observed, error", *(round(x, 4) for x in row))
            errors.append(row[-1])
        if args.returns:
            return_motion(folder, metadata)
    if errors:
        print("positive ZERO count, min/median/max error", len(errors),
              *(round(x, 4) for x in (min(errors), median(errors), max(errors))))
