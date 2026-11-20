"""Compare receiving-foot forecasts using only completed earlier half-cycles of the same run."""
import argparse
import csv
from bisect import bisect_left
from pathlib import Path
from statistics import mean, median

HORIZONS = (150, 250)


def rows(path):
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


class Observations:
    def __init__(self, frames, foot):
        self.data = [(int(r["measurement_time_us"]) / 1000, float(r[foot + "_deg"]),
                      (int(r["measurement_time_us"]) + int(r["control_observed_us"]) - int(r["frame_us"])) / 1000)
                     for r in frames if r["state_id"] == "3" and r["timestamp_valid"] == "True"
                     and r["frame_valid"] == "True" and r["zero_ready"] == "True" and r[foot + "_valid"] == "True"
                     and r[foot + "_control_range_ok"] == "True" and int(r["control_observed_us"]) > 0]
        self.times = [r[0] for r in self.data]
        if self.times != sorted(set(self.times)):
            raise ValueError("Expected distinct chronological camera timestamps")

    def at(self, t, start=-float("inf"), end=float("inf"), available=float("inf")):
        i = bisect_left(self.times, t)
        if i < len(self.times) and self.times[i] == t:
            x, v, received = self.data[i]
            return v if start <= x <= end and received <= available else None
        if not 0 < i < len(self.times):
            return None
        a, b = self.data[i-1:i+1]
        if a[0] < start or b[0] > end or max(a[2], b[2]) > available or b[0] - a[0] > 50:
            return None
        return a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])


def compare(trial, snapshots):
    peaks = [(int(p["peak_time_ms"]), -int(p["physical_peak_side"]))
             for p in rows(trial / "energy_control_autonomous_peak_events.csv")]
    # A later recorded peak closes a historical half-cycle, regardless of whether ZERO was crossed.
    history = [(a, b, side) for (a, side), (b, _) in zip(peaks, peaks[1:])]
    frames = rows(trial / "foot_angles.csv")
    obs = {foot: Observations(frames, foot) for foot in ("left", "right")}
    pulses = [(int(r["output_start_ms"]), int(r["output_end_ms"]))
              for r in rows(trial / "walking_energy.csv") if r["executed"] == "True"]
    stored = rows(trial / "support_transfer_shadow.csv")
    replay = rows(snapshots)
    if [r["capture_us32"] for r in replay] != [r["capture_us32"] for r in stored]:
        raise ValueError("Replay and trial contain different decision timestamps")
    results = []
    for p in replay:
        if p["phase"] != "2":
            continue
        t, peak, side = float(p["capture_ms"]), int(p["peak_ms"]), int(p["side"])
        foot = "left" if side < 0 else "right"
        angle = float(p[foot + "_deg"])
        rate = float(p[foot + "_rate_dps"])
        local = 150.
        for leg in ("left", "right"):
            velocity = float(p[leg + "_rate_dps"])
            if velocity:
                bound = 20 if velocity > 0 else -20
                local = min(local, 1000*(bound - float(p[leg + "_deg"])) / velocity)
        for horizon in HORIZONS:
            future = obs[foot].at(t + horizon)
            old = angle + min(local, horizon)*rate/1000
            deltas = []
            for a, b, historical_side in history:
                if historical_side != side or b > peak:
                    continue
                # Both historical frames must belong to the closed cycle and have reached control.
                same_phase = a + t - peak
                former = obs[foot].at(same_phase, a, b, t)
                later = obs[foot].at(same_phase + horizon, a, b, t)
                if former is not None and later is not None:
                    deltas.append(later - former)
            estimate = angle + median(deltas) if deltas else None
            command = any(on <= t+horizon+1 and (off == 0 or off >= t-1) for on,off in pulses)
            results.append(dict(time_ms=int(p["time_ms"]), peak=peak, horizon=horizon, old=old,
                                estimate=estimate, actual=future, examples=len(deltas), command=command))
    return results


def errors(results):
    return tuple(mean(abs(r[k]-r["actual"]) for r in results) for k in ("old", "estimate"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, help="Directory from convert_rwlog_to_csv.py")
    parser.add_argument("snapshots", type=Path, help="Snapshots CSV from replay_audit.py --out")
    args = parser.parse_args()
    results = compare(args.trial, args.snapshots)
    for horizon in HORIZONS:
        all_rows = [r for r in results if r["horizon"] == horizon]
        measurable = [r for r in all_rows if r["actual"] is not None]
        shared = [r for r in measurable if r["estimate"] is not None]
        print(f"{horizon} ms eligible={len(all_rows)}, camera={len(measurable)}, shared={len(shared)},",
              "median past examples", median(r["examples"] for r in shared) if shared else None)
        if not shared:
            continue
        peaks = sorted({r["peak"] for r in shared})
        per_peak = [errors([r for r in shared if r["peak"] == peak]) for peak in peaks]
        print("  MAE current/history", errors(shared), "peaks", len(peaks))
        print("  MAE equally weighting those peaks", tuple(map(mean, zip(*per_peak))))
        for cmd in (False, True):
            group = [r for r in shared if r["command"] == cmd]
            if group:
                print("  command", cmd, len(group), errors(group))
        if measurable:
            fallback = [dict(r, estimate=r["old"] if r["estimate"] is None else r["estimate"]) for r in measurable]
            print("  MAE with current fallback", errors(fallback))
    for r in results:
        if r["time_ms"] in (5251, 10828, 14427):
            print("  selected",r)


if __name__ == "__main__":
    main()
