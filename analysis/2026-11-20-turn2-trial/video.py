"""Measure the November 20 turn2 walks in the same floor coordinates as turn1."""

import argparse
import csv
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import cv2
import numpy as np

prior_spec = spec_from_file_location("turn1_video", Path(__file__).resolve().parents[1] /
                                    "2026-11-20-turn1-trial/video.py")
prior = module_from_spec(prior_spec)
prior_spec.loader.exec_module(prior)

# Translation relative to turn1 (median of eight fixed grid templates).
REGISTRATION = {"45820669": [-2, -3], "239670034": [-1.5, -2]}


def rows(folder, name):
    with (folder / (name + ".csv")).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def estimate_offset(stamps, points, folder):
    axis = points[:, 1] - points[:, 0]
    optical = np.degrees(np.arctan2(axis[:, 1], axis[:, 0]))
    records = [r for r in rows(folder, "timeseries") if r["state_id"] == "3"]
    times = np.array([int(r["t_test_ms"]) * .001 for r in records])
    body = np.array([float(r["pitch_mekf_detector_relative_deg"]) for r in records])
    offsets = np.arange(3, 12, .001)
    for lo, hi in ((0, 4), (.5, 5)):
        mask = (times >= lo) & (times < hi)
        correlations = [np.corrcoef(np.interp(times[mask] + t, stamps, optical),
                                    body[mask])[0, 1] for t in offsets]
        best = np.nanargmin(correlations)
        print("sync", (lo, hi), offsets[best], round(correlations[best], 4))
    return offsets[best]


def stationary(video, offset, markers):
    # Both feet remain visible; verify their positions throughout test 12–30 s.
    cam = cv2.VideoCapture(str(video))
    cam.set(cv2.CAP_PROP_POS_MSEC, 1000 * (offset + 12))
    ok, reference = cam.read()
    if not ok:
        raise ValueError("Missing rest reference")
    reference = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    result = []
    while True:
        ok, frame = cam.read()
        if not ok or cam.get(cv2.CAP_PROP_POS_MSEC) > 1000 * (offset + 30):
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        matches = []
        for x, y in np.rint(markers).astype(int):
            template = reference[y-12:y+13, x-12:x+13]
            search = gray[y-25:y+26, x-25:x+26]
            _, score, _, (u, v) = cv2.minMaxLoc(cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED))
            matches.append((u, v, score))
        result.append(matches)
    cam.release()
    a = np.asarray(result)
    print("rest samples, ranges(px), minimum correlation", len(a), np.ptp(a[:, :, :2], axis=0).tolist(),
          a[:, :, 2].min(axis=0).round(3).tolist())


def main(video, folder, trial):
    stamps, points = prior.tracks(video)
    offset = estimate_offset(stamps, points, folder)
    t = stamps - offset
    grid_shift = np.array(REGISTRATION[trial])
    feet = np.array([[prior.floor_point(p - grid_shift) for p in pair] for pair in points[:, 2:]])
    print("tracked frames", len(t), np.isfinite(points).all(axis=-1).sum(axis=0))

    def pos(at, halfwidth):
        return np.mean(feet[np.abs(t-at) < halfwidth], axis=0)

    start = feet[(t > -.8) & (t < 0)].mean(axis=0)
    for a, b in ((0, 4), (4, 8), (8, 12), (0, 12)):
        delta = pos(b, .4) - (pos(a, .4) if a else start)
        lateral, forward = delta.mean(axis=0)
        # At rest, projecting subpixel jitter to a path angle is meaningless.
        direction = round(np.degrees(np.arctan2(lateral, forward)), 2) if np.hypot(lateral, forward) > .1 else None
        print("test", a, b, "right/left forward; lateral; path",
              delta[:, 1].round(3).tolist(), round(lateral, 3), direction)

    peaks = [(int(r["peak_time_ms"])*.001, int(r["physical_peak_side"]))
             for r in rows(folder, "energy_control_autonomous_peak_events")]
    zeros = [r for r in rows(folder, "energy_control_autonomous_zero_cross_events")
             if r["event_kind"] == "energy_control_zero_cross"]
    for halfwidth in (.05, .1):
        match = {-1: [], 1: []}
        for z in zeros:
            crossing, side = int(z["zero_cross_time_ms"]) * .001, int(z["physical_next_peak_side"])
            peak = next((p for p in peaks if p[0] > crossing), None)
            if peak and peak[1] == side:
                advance = pos(peak[0]+.025, halfwidth) - pos(crossing, halfwidth)
                if np.isfinite(advance).all():
                    match[side].append(side * (advance[1, 1] - advance[0, 1]) > 0)
        print("halfwidth, relative forward right/left matched/total", halfwidth,
              [(sum(values), len(values)) for values in match.values()])
    for a in np.arange(4, 10, .5):
        y = feet[(t >= a) & (t < a+.5), :, 1]
        if len(y):
            print("test half-second", a, "front/back ranges", np.ptp(y, axis=0).round(3).tolist())
    stationary(video, offset, points[-1, 2:])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", choices=REGISTRATION)
    parser.add_argument("video", type=Path)
    parser.add_argument("converted_rwlog", type=Path)
    args = parser.parse_args()
    main(args.video, args.converted_rwlog, args.trial)
