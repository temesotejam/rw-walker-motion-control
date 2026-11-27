"""Track the two feet in clips recorded with the reference floor-grid calibration."""
import cv2
import numpy as np


REGISTRATION = {"45820669": [-2, -3], "239670034": [-1.5, -2]}


def tracks(video, initial_markers=((527., 252.), (739., 252.), (611., 391.), (651., 391.)), end_s=20):
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise OSError(video)
    position = np.array(initial_markers, dtype=float)
    velocity = np.zeros_like(position)
    stamps, samples = [], []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        stamp = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000
        if stamp > end_s:  # In the turn1 recording the feet become intermittently occluded after 20 s.
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
