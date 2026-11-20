"""Compare differences in ordinary pulse duration and subsequent passive foot motion."""
import argparse
import csv
from itertools import combinations
from pathlib import Path

import numpy as np


def rows(folder, name):
    with (folder / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def intervals(folder):
    zeros = [r for r in rows(folder, "energy_control_autonomous_zero_cross_events.csv")
             if r["event_kind"] == "energy_control_zero_cross"]
    pulses = [r for r in rows(folder, "walking_energy.csv") if r["executed"] == "True"]
    imu = [r for r in rows(folder, "timeseries.csv") if r["state_id"] == "3"]
    gyro_t = np.array([int(r["t_test_ms"]) for r in imu])
    yaw = np.array([float(r["gyro_heading_deg"]) if r["gyro_heading_valid"] == "1" else np.nan for r in imu])
    feet = [r for r in rows(folder, "foot_angles.csv") if r["state_id"] == "3" and
            all(r[k] == "True" for k in ("frame_valid", "timestamp_valid", "zero_ready", "left_valid", "right_valid"))]
    times, left, right = np.array([(int(r["measurement_time_us"]) / 1000, float(r["left_deg"]),
                                    float(r["right_deg"])) for r in feet]).T
    candidates = []
    for start, end in zip(zeros, zeros[1:]):
        t, u = int(start["zero_cross_time_ms"]), int(end["zero_cross_time_ms"])
        side = int(start["physical_next_peak_side"])
        if side != -int(end["physical_next_peak_side"]) or not 150 <= u - t <= 800:
            continue
        peak = int(start["previous_peak_time_ms"])
        inputs = [r for r in pulses if peak <= int(r["time_ms"]) < u]
        if len(inputs) != 1 or any(inputs[0][k] != "False" for k in ("optional", "rescue")):
            continue
        pulse = inputs[0]
        if (int(pulse["peak_ms"]) != peak or not 0 <= int(pulse["time_ms"]) - t <= 5
                or int(pulse["side"]) != side or pulse["completion_reason"] != "1"):
            continue
        # This is retrospective identification. Use a pre-ZERO image only for
        # matching and subsequent images only for the measured swing outcome.
        i = np.searchsorted(times, t, side="right") - 1
        j = np.searchsorted(times, u)
        if i < 0 or j - i < 5 or max(np.diff(np.r_[times[i:j], u])) > 75:
            continue
        stop = int(pulse["output_end_ms"])
        if stop >= u or t - times[i] > 50:
            continue
        # Start posture and speed are confounders for the closed-loop pulse.
        features = np.array([float(start["zero_cross_abs_rate_dps"]), float(start["previous_peak_amplitude_deg"]),
                             left[i], right[i], float(start["pre_input_wheel_speed_rpm"]),
                             float(pulse["start_i0_ma"])])
        # Forward excursion from each leg's pre-pulse position.
        reach = np.percentile(right[i+1:j], 95) - right[i] - (np.percentile(left[i+1:j], 95) - left[i])
        first, last = np.searchsorted(gyro_t, [t, u], side="right") - 1
        if first < 0 or last <= first or not np.all(np.isfinite(yaw[first:last + 1])):
            continue
        candidates.append((side, features, int(pulse["actual_elapsed_ms"]), reach, yaw[last] - yaw[first]))
    return candidates


def matched_count(events, motor_matched):
    # Within a trial and direction, retain disjoint nearest-condition pairs.
    limits = np.array([5., 1., 1.5, 1.5, 200., 50.]) if motor_matched else np.array([5., 1., 1.5, 1.5])
    pairs = []
    for (i, a), (j, b) in combinations(enumerate(events), 2):
        distance = np.abs((a[1] - b[1])[:len(limits)] / limits)
        if a[0] == b[0] and max(distance) <= 1 and abs(a[2] - b[2]) >= 5:
            pairs.append((np.linalg.norm(distance), i, j))
    used = set()
    result = {-1: [], 1: []}
    for _, i, j in sorted(pairs):
        if i in used or j in used:
            continue
        used.update((i, j))
        a, b = sorted((events[i], events[j]), key=lambda e: e[2])
        result[a[0]].append((b[3] - a[3], b[4] - a[4]))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", type=Path, nargs="+", help="RWLOG converter output directories")
    for directory in parser.parse_args().trials:
        events = intervals(directory)
        for motors in (False, True):
            paired = matched_count(events, motors)
            print(directory.name, len(events), "posture+motor" if motors else "posture",
                  {s: np.round(v, 2).tolist() for s, v in paired.items()})
