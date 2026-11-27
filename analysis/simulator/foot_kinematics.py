"""Causal per-foot angle and angular rate from the published periodic camera stream."""
from __future__ import annotations

from bisect import bisect_right
from collections import deque
from dataclasses import dataclass, replace
import math
from pathlib import Path

from return_model import read_rows


MAX_AGE_US = 75_000
HISTORY_US = 75_000
MIN_SPAN_US = 15_000


@dataclass(frozen=True)
class FootMotion:
    angle_deg: float
    rate_dps: float | None
    measurement_ms: float
    age_ms: float
    span_ms: float
    frames: int
    published_est_deg: float | None


@dataclass(frozen=True)
class FootPair:
    left: FootMotion | None
    right: FootMotion | None


def enabled(value):
    return str(value).lower() in ('true', '1')


class SideHistory:
    def __init__(self, side):
        self.side = side
        self.last_sequence = None
        self.samples = deque()

    def update(self, frame, publication, estimate=None):
        side = self.side
        if not frame or not all(enabled(frame.get(key)) for key in (
                'timestamp_valid', 'frame_valid', 'zero_ready', side + '_valid')):
            self.samples.clear()
            self.last_sequence = None
            return None
        # Firmware accepts modest calibrated endpoint extrapolation. Old data with
        # no runtime range flag have evidence only for the strict calibration range.
        accepted = frame.get(side + '_control_range_ok')
        if accepted in (None, ''):
            accepted = frame.get(side + '_in_range')
        seq = int(frame['sequence'])
        measured = frame.get('measurement_time_us')
        stamp = int(measured) if measured not in ('', None) else -1
        raw = frame.get(side + '_deg')
        angle = float(raw) if raw not in ('', None) else math.nan
        if (seq <= 0 or not enabled(accepted) or not math.isfinite(angle) or stamp < 0
                or not 0 <= publication - stamp <= MAX_AGE_US):
            self.samples.clear()
            self.last_sequence = None
            return None
        if self.samples and stamp < self.samples[-1][0]:
            # An out-of-order source cannot enter the causal derivative.
            self.samples.clear()
            self.last_sequence = None
            return None
        if seq != self.last_sequence and (not self.samples or stamp > self.samples[-1][0]):
            self.samples.append((stamp, angle))
            self.last_sequence = seq
        while self.samples and stamp - self.samples[0][0] > HISTORY_US:
            self.samples.popleft()
        # One slope per distinct measured image. Re-publishing a stale sequence
        # should age that measurement without adding a flat segment to the fit.
        span = self.samples[-1][0] - self.samples[0][0]
        rate = None
        if span >= MIN_SPAN_US:
            times = [(t - stamp) * 1e-6 for t, _ in self.samples]
            center = sum(times) / len(times)
            dt = [t - center for t in times]
            rate = sum(d * (a - angle) for d, (_, a) in zip(dt, self.samples)) / sum(d * d for d in dt)
        estimate = float(estimate) if estimate not in ('', None) else None
        if estimate is not None and not math.isfinite(estimate):
            estimate = None
        return FootMotion(angle, rate, stamp / 1000, (publication - stamp) / 1000,
                          span / 1000, len(self.samples), estimate)


class FootHistory:
    def __init__(self, packets, frames):
        by_sequence = {int(f['sequence']): f for f in frames}
        left, right = SideHistory('left'), SideHistory('right')
        self.times = []
        self.pairs = []
        for p in sorted(packets, key=lambda r: int(r['publish_us'])):
            stamp = int(p['publish_us'])
            self.times.append(stamp / 1000)
            self.pairs.append(FootPair(
                left.update(by_sequence.get(int(p['left_sequence'] or 0)), stamp, p.get('left_est_deg')),
                right.update(by_sequence.get(int(p['right_sequence'] or 0)), stamp, p.get('right_est_deg'))))

    @classmethod
    def from_folder(cls, folder):
        folder = Path(folder)
        return cls(read_rows(folder, 'foot_periodic'), read_rows(folder, 'foot_angles'))

    def at(self, time_ms):
        """Use the last packet published by time_ms; positive rates swing forward."""
        idx = bisect_right(self.times, time_ms) - 1
        if idx < 0:
            return FootPair(None, None)

        def current(side):
            if side is None or time_ms - side.measurement_ms > MAX_AGE_US / 1000:
                return None
            return replace(side, age_ms=time_ms - side.measurement_ms)

        pair = self.pairs[idx]
        return FootPair(current(pair.left), current(pair.right))
