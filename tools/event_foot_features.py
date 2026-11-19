"""Reconstruct camera evidence available at an archived v37 energy decision.

Only the control owner's *first receipt* admits a frame. The frame timestamp
sets its age and the intervals used to classify the recent body-relative foot
motion. No extrapolation to the decision, contact inference or world position
is implied. The classifier thresholds are provisional replay parameters.
"""

from bisect import bisect_right
from collections import defaultdict
from itertools import combinations
from math import isfinite
from statistics import median


MAX_FRAME_AGE_US = 75_000
MAX_HISTORY_GAP_US = 50_000
MAX_HISTORY_SPAN_US = 120_000
MAX_STEP_DEG = 8.0
HOLD_BAND_DEG = 0.20
MOVE_DELTA_DEG = 0.25
MOVE_RATE_DPS = 3.0
STEP_NOISE_DEG = 0.05


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and isfinite(value)


def _kind(event):
    if event.get("rescue"):
        return "inward_stall_rescue"
    if event.get("optional"):
        return "angle_rate_early_trial"
    return "energy_control_zero_cross"


def _angle(row, side):
    value = row.get(side + "_deg")
    if (row.get("frame_valid") and row.get("timestamp_valid") and row.get("zero_ready")
            and row.get(side + "_valid") and row.get(side + "_control_range_ok")
            and _finite(value)):
        return value
    return None


def _direction(first, last):
    delta = last[1] - first[1]
    if abs(delta) < MOVE_DELTA_DEG or abs(delta) * 1e6 < MOVE_RATE_DPS * (last[0] - first[0]):
        return 0
    return 1 if delta > 0 else -1


def _phase(history):
    # Requiring a span comprising at least three intervals rejects a single
    # noisy derivative. Check the *latest* movement separately so a fitted
    # rate dominated by the earlier stroke cannot hide a pause or reversal.
    if len(history) < 4:
        return "unknown", None
    rate = median((b[1] - a[1]) * 1e6 / (b[0] - a[0]) for a, b in combinations(history, 2))
    values = [angle for _, angle in history]
    if max(values) - min(values) <= HOLD_BAND_DEG:
        return "holding", rate
    mid = history[-3]
    previous = _direction(history[0], mid)
    recent = _direction(mid, history[-1])
    if previous * recent < 0:
        # A single sharp excursion can mimic a reversal. Require an interior
        # point on each side of the turn for directional corroboration.
        if (len(history) == 5 and previous * (history[1][1] - history[0][1]) > STEP_NOISE_DEG
                and recent * (history[-1][1] - history[-2][1]) > STEP_NOISE_DEG):
            return "turning", rate
        return "unknown", rate
    corroboration = sum(recent * (b - a) > STEP_NOISE_DEG for a, b in zip(values, values[1:]))
    if (recent and recent * rate >= MOVE_RATE_DPS and corroboration >= 2
            and recent * (values[-1] - values[-2]) > STEP_NOISE_DEG):
        return ("outgoing" if recent > 0 else "returning"), rate
    return "unknown", rate


class CausalFootFeatures:
    """Index a single run's converted metadata; query in any event order."""

    def __init__(self, meta):
        active = [row for row in meta["foot_frames"] if row.get("state_id") == 3]
        if len({row["run_id"] for row in active}) != 1:
            raise ValueError("Expected camera frames from one running measurement")
        origins = {row["frame_us"] - row["measurement_time_us"] for row in active
                   if row.get("timestamp_valid") and row.get("frame_us", 0) > 0}
        if len(origins) != 1:
            raise ValueError("Missing or inconsistent camera measurement epoch")
        self.epoch_us = origins.pop()
        self.frames = [row for row in active if (row.get("control_observed_us") or 0) > 0]
        self.receipts = [row["control_observed_us"] for row in self.frames]
        if any(a >= b for a, b in zip(self.receipts, self.receipts[1:])):
            raise ValueError("First control receipts must be strictly increasing")
        self.crossings = defaultdict(list)
        for row in meta["energy_control_autonomous_zero_cross_events"]:
            if row.get("work_control_branch") == 24:
                key = (row["event_kind"], row["previous_peak_time_ms"], row["physical_next_peak_side"])
                self.crossings[key].append(row)

    def _cutoff(self, event):
        # Physical ZERO is interpolated, so its rounded crossing time can be
        # slightly earlier than the owning control tick. Synthetic aid events
        # use the tick time. Require an unambiguous matching event and use its
        # *pre-solver* hardware timestamp, never a later output timestamp.
        key = (_kind(event), event["peak_ms"], event["side"])
        ms = event["time_ms"]
        matches = [row for row in self.crossings.get(key, ())
                   if 0 <= ms - row["zero_cross_time_ms"] <= 5 and row.get("pre_input_capture_time_us")]
        if len(matches) != 1:
            return None
        # micros() is uint32, frame and receipt timestamps use esp_timer uint64.
        approx = self.epoch_us + ms * 1000
        since = (matches[0]["pre_input_capture_time_us"] - approx + 2**31) % 2**32 - 2**31
        if abs(since) > 10_000:
            return None
        return approx + since

    def _side_history(self, index, side):
        history = []
        newest_us = self.frames[index]["frame_us"]
        for pos in range(index, max(index - 5, -1), -1):
            row = self.frames[pos]
            angle = _angle(row, side)
            when = row["frame_us"]
            if angle is None or when < self.epoch_us or when > self.receipts[pos]:
                break
            if history:
                next_us, next_deg = history[-1]
                if (row["sequence"] + 1 != self.frames[pos + 1]["sequence"]
                        or not 0 < next_us - when <= MAX_HISTORY_GAP_US
                        or newest_us - when > MAX_HISTORY_SPAN_US
                        or abs(next_deg - angle) > MAX_STEP_DEG):
                    break
            history.append((when, angle))
        return history[::-1]

    def at(self, event):
        cutoff = self._cutoff(event)
        # tryWalkingEnergy() copies the owner's guard result *before* solving.
        # Its allowed flag includes the 10 ms IMU sample-age limit. Numeric
        # angle/rate are measured motion proxies, not a contact or fall proof.
        has_posture = (event.get("fore_aft_allowed") is True and event.get("fore_aft_reason") in (0, 1)
                       and _finite(event.get("fore_aft_angle_deg")) and _finite(event.get("fore_aft_rate_dps")))
        out = {"decision_capture_us": cutoff, "foot_frame_us": None, "foot_receipt_us": None,
               "foot_sequence": None, "foot_age_ms": None,
               "fore_aft_angle_deg": event["fore_aft_angle_deg"] if has_posture else None,
               "fore_aft_rate_dps": event["fore_aft_rate_dps"] if has_posture else None,
               "fore_aft_source": "pre_solve_owner_guard" if has_posture else "unavailable",
               "fore_aft_age_bound_ms": 10 if has_posture else None}
        for side in ("left", "right"):
            out.update({side + "_valid": False, side + "_deg": None, side + "_phase": "unknown",
                        side + "_rate_dps": None, side + "_history_count": 0})
        if cutoff is None:
            return out
        index = bisect_right(self.receipts, cutoff) - 1
        if index < 0:
            return out
        row = self.frames[index]
        age = cutoff - row["frame_us"]
        out.update(foot_sequence=row["sequence"], foot_frame_us=row["frame_us"],
                   foot_receipt_us=self.receipts[index])
        if not row.get("timestamp_valid") or not self.epoch_us <= row["frame_us"] <= self.receipts[index]:
            return out
        out["foot_age_ms"] = age / 1000
        if age > MAX_FRAME_AGE_US:
            return out
        for side in ("left", "right"):
            history = self._side_history(index, side)
            if history:
                phase, rate = _phase(history)
                out.update({side + "_deg": history[-1][1], side + "_valid": True,
                            side + "_phase": phase, side + "_rate_dps": rate,
                            side + "_history_count": len(history)})
        return out
