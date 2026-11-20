"""Reproduce optical measurements for the 2026-11-20 18:36 trial (OpenCV, NumPy)."""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def tracks(video):
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise OSError(video)
    position = np.array([[527., 252.], [739., 252.], [611., 391.], [651., 391.]])
    velocity = np.zeros_like(position)
    stamps, samples = [], []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        stamp = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
        if stamp > 20:  # Both foot marks become intermittently occluded later.
            break
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        points = np.full_like(position, np.nan)
        for i, previous in enumerate(position):
            body = i < 2
            predicted = previous + .65 * velocity[i]
            x, y = predicted.astype(int) - 35
            binary = cv2.inRange(gray[y:y+71, x:x+71], 178 if body else 185, 255)
            _, _, stats, centers = cv2.connectedComponentsWithStats(binary)
            choices = []
            for (_, _, w, h, area), center in zip(stats[1:], centers[1:]):
                if body:
                    fits = 55 <= area <= 600 and 4 <= w <= 30 and 9 <= h <= 33 and area / (w*h) > .57
                else:
                    fits = 20 <= area <= 280 and 4 <= w <= 34 and 3 <= h <= 24 and area / (w*h) > .48
                point = center + [x, y]
                distance = np.linalg.norm(point - predicted)
                if fits and distance < 28:
                    cost = distance + 1.7 * abs(np.log(area / (260 if body else 100)))
                    choices.append((cost, point))
            if choices:
                points[i] = min(choices, key=lambda candidate: candidate[0])[1]
                velocity[i] = points[i] - previous
                position[i] = points[i]
            else:
                velocity[i] = 0.
        stamps.append(stamp)
        samples.append(points)
    capture.release()
    return np.asarray(stamps), np.asarray(samples)


def interp(value, grid, levels):
    index = np.clip(np.searchsorted(grid, value) - 1, 0, len(grid) - 2)
    fraction = (value - grid[index]) / (grid[index+1] - grid[index])
    return levels[index] + fraction * (levels[index+1] - levels[index])


def floor_point(point):
    if not np.isfinite(point).all():
        return np.full(2, np.nan)
    x, y = point - [3, 8]  # Register with the grid measured in trial 69728952.
    left = np.array([381,391,401,411,422,433,445,457,470,483,497,512,527,543,560])
    right = np.array([380,389,399,409,420,431,443,455,467,481,495,509,524,540,557])
    lines = left + (right-left) * (x-455) / 305
    depth = interp(y, lines, np.arange(len(lines)))
    rows = [400, 449, 550]
    lo = interp(y, rows, [487, 474, 448])
    hi = interp(y, rows, [747, 761, 789])
    return (12 * (x - (lo+hi)/2) / (hi-lo), depth)


def stationary_links(video, offset):
    capture = cv2.VideoCapture(str(video))
    capture.set(cv2.CAP_PROP_POS_MSEC, 25000)
    ok, image = capture.read()
    if not ok:
        raise ValueError('Missing video frame at 25 s')
    ref = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fixtures = [(419, 452), (487, 570)]  # Far and near foot-link screws at rest.
    capture.set(cv2.CAP_PROP_POS_MSEC, (offset+18)*1000)
    measurements = []
    while True:
        ok, image = capture.read()
        if not ok or capture.get(cv2.CAP_PROP_POS_MSEC) > (offset+30)*1000:
            break
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        values = []
        for x, y in fixtures:
            patch = ref[y-12:y+13, x-12:x+13]
            search = gray[y-27:y+28, x-27:x+28]
            _, score, _, (u, v) = cv2.minMaxLoc(cv2.matchTemplate(search, patch, cv2.TM_CCOEFF_NORMED))
            values.append([u, v, score])
        measurements.append(values)
    capture.release()
    m = np.asarray(measurements)
    print('Foot screw ranges in pixels, test 18-30s:', np.ptp(m[:, :, :2], axis=0).tolist(),
          'minimum template correlation', m[:, :, 2].min(axis=0).round(2).tolist())


def main(video, folder):
    stamps, points = tracks(video)
    body = points[:, 1, :] - points[:, 0, :]
    angle = np.degrees(np.arctan2(body[:, 1], body[:, 0]))
    names = ('t_test_ms', 'state_id', 'pitch_mekf_detector_relative_deg')
    imu = np.genfromtxt(folder / 'timeseries.csv', delimiter=',', names=True, usecols=names)
    valid = (imu['state_id'] == 3) & np.isfinite(imu[names[2]])
    time = imu['t_test_ms'][valid] / 1000
    active = (time >= .5) & (time < 5)
    offsets = np.arange(5., 8., .001)
    correlations = [np.corrcoef(np.interp(time[active]+z, stamps, angle),
                                imu[names[2]][valid][active])[0, 1] for z in offsets]
    best = np.nanargmin(correlations)
    print(f'Video time at test zero {offsets[best]:.3f} s, correlation {correlations[best]:.4f}')
    t = stamps - offsets[best]
    feet = np.array([[floor_point(p) for p in pair] for pair in points[:, 2:]])

    def position(at, halfwidth):
        return np.mean(feet[np.abs(t-at) < halfwidth], axis=0)

    for a, b in [(0, 4), (4, 8), (8, 12), (5.05, 5.62)]:
        halfwidth = .06 if a == 5.05 else .4
        base = position(a, halfwidth) if a else np.mean(feet[(-.8 < t) & (t < 0)], axis=0)
        delta = position(b, halfwidth) - base
        direction = np.degrees(np.arctan2(*np.mean(delta, axis=0)))
        print(f'{a}-{b} s; projected right and left (lateral, forward)', delta.round(3).tolist(),
              f'path direction {direction:+.1f} deg')

    with (folder / 'energy_control_autonomous_peak_events.csv').open(encoding='utf-8-sig') as stream:
        peaks = [(int(r['peak_time_ms'])/1000, int(r['physical_peak_side'])) for r in csv.DictReader(stream)]
    with (folder / 'energy_control_autonomous_zero_cross_events.csv').open(encoding='utf-8-sig') as stream:
        zero = [r for r in csv.DictReader(stream) if r['event_kind'] == 'energy_control_zero_cross']
    for halfwidth in [.05, .1]:
        aligned = {s: [] for s in (-1, 1)}
        for r in zero:
            at = int(r['zero_cross_time_ms'])/1000
            side = int(r['physical_next_peak_side'])
            next_peak = next((p for p in peaks if p[0] > at), None)
            if not next_peak or next_peak[0]+.025+halfwidth > 12 or next_peak[1] != side:
                continue
            step = position(next_peak[0]+.025, halfwidth) - position(at, halfwidth)
            if np.isfinite(step).all():
                aligned[side].append(side * (step[1, 1] - step[0, 1]) > 0)
        print(f'Projected swing matches phase ({halfwidth:.2f}s halfwidth)',
              [(side, int(sum(values)), len(values)) for side, values in aligned.items()])
    stationary_links(video, offsets[best])


if __name__ == '__main__':
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('video', type=Path)
    args.add_argument('converted_rwlog', type=Path)
    inputs = args.parse_args()
    main(inputs.video, inputs.converted_rwlog)
