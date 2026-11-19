"""Causality, phase transitions and v37 archive coverage for event replay."""

import json
from pathlib import Path
import unittest

from event_foot_features import CausalFootFeatures


def fixture(left=(0, 1, 2, 3, 4), right=None, tick_ms=110, epoch=2_000_000):
    if right is None:
        right = [0.0] * len(left)
    frames = []
    for i, (l_deg, r_deg) in enumerate(zip(left, right)):
        capture = (i + 1) * 20_000
        frames.append(dict(run_id=7, state_id=3, sequence=20 + i,
                           frame_us=epoch + capture, measurement_time_us=capture,
                           control_observed_us=epoch + capture + 9_000,
                           delivered_us=epoch + capture + 5_000, ready_us=epoch + capture + 6_000,
                           timestamp_valid=True, frame_valid=True, zero_ready=True,
                           left_valid=True, left_control_range_ok=True, left_deg=l_deg,
                           right_valid=True, right_control_range_ok=True, right_deg=r_deg))
    event = dict(time_ms=tick_ms, peak_ms=20, side=-1, rescue=True,
                 fore_aft_allowed=True, fore_aft_reason=0,
                 fore_aft_angle_deg=0.5, fore_aft_rate_dps=-3)
    crossing = dict(event_kind="inward_stall_rescue", previous_peak_time_ms=20,
                    physical_next_peak_side=-1, zero_cross_time_ms=tick_ms, work_control_branch=24,
                    pre_input_capture_time_us=(epoch + tick_ms * 1000) % 2**32)
    return dict(foot_frames=frames, energy_control_autonomous_zero_cross_events=[crossing]), event


class CausalFeaturesTest(unittest.TestCase):
    def test_axes_rates_and_independent_phases(self):
        for left, expected in (
            ([0, 1, 2, 3, 4], "outgoing"), ([6, 5, 4, 3, 2], "returning"),
            ([1, 1.02, 0.99, 1.04, 1.02], "holding"), ([0, 1, 2, 1, 0], "turning"),
            ([0, 0, 2, 0, 0], "unknown"), ([0, 0, 0, 1, 1], "unknown"),
        ):
            with self.subTest(left=left):
                meta, event = fixture(left, right=[0, 1, 2, 3, 4])
                result = CausalFootFeatures(meta).at(event)
                self.assertEqual(result["left_phase"], expected)
                self.assertEqual(result["right_phase"], "outgoing")
                self.assertEqual(result["right_rate_dps"], 50)

    def test_capture_and_delivery_are_insufficient_until_actual_receipt(self):
        meta, event = fixture()
        frames = meta["foot_frames"]
        frames[-1]["control_observed_us"] += 20_000
        result = CausalFootFeatures(meta).at(event)
        # Frame 24 is already delivered and recognized, but not yet read by control.
        self.assertEqual(result["foot_sequence"], 23)
        self.assertEqual(result["left_deg"], 3)
        frames[-1]["control_observed_us"] = 0
        frames[-1]["left_deg"] = 50
        self.assertEqual(CausalFootFeatures(meta).at(event), result)
        frames.pop()
        self.assertEqual(CausalFootFeatures(meta).at(event), result)

    def test_exact_clock_boundary_precedes_rounded_tick(self):
        meta, event = fixture(epoch=2**32 - 50_000)
        cross = meta["energy_control_autonomous_zero_cross_events"][0]
        cross["pre_input_capture_time_us"] -= 1_001
        result = CausalFootFeatures(meta).at(event)
        self.assertEqual(result["foot_sequence"], 23)
        self.assertEqual(result["decision_capture_us"], 2**32 + 58_999)
        cross["pre_input_capture_time_us"] += 1
        self.assertEqual(CausalFootFeatures(meta).at(event)["foot_sequence"], 24)

    def test_invalid_side_does_not_hold_previous_angle_or_poison_other_side(self):
        meta, event = fixture(right=[6, 5, 4, 3, 2])
        for flag in ["left_valid", "left_control_range_ok", "frame_valid", "zero_ready"]:
            with self.subTest(flag=flag):
                meta["foot_frames"][-1][flag] = False
                result = CausalFootFeatures(meta).at(event)
                self.assertFalse(result["left_valid"])
                self.assertIsNone(result["left_deg"])
                self.assertEqual(result["left_phase"], "unknown")
                if flag.startswith("left"):
                    self.assertEqual(result["right_phase"], "returning")
                meta["foot_frames"][-1][flag] = True
        meta["foot_frames"][-2]["left_valid"] = False
        result = CausalFootFeatures(meta).at(event)
        self.assertTrue(result["left_valid"])
        self.assertEqual(result["left_history_count"], 1)
        self.assertEqual(result["left_phase"], "unknown")
        self.assertEqual(result["right_phase"], "returning")

    def test_stale_discontinuous_and_unmatched_fail_closed(self):
        meta, event = fixture(tick_ms=200)
        result = CausalFootFeatures(meta).at(event)
        self.assertEqual(result["foot_age_ms"], 100)
        self.assertFalse(result["left_valid"])
        meta, event = fixture()
        meta["foot_frames"][-1]["sequence"] += 1
        result = CausalFootFeatures(meta).at(event)
        self.assertEqual(result["left_history_count"], 1)
        self.assertIsNone(result["left_rate_dps"])
        event["fore_aft_allowed"] = False
        event["peak_ms"] += 1
        result = CausalFootFeatures(meta).at(event)
        self.assertIsNone(result["decision_capture_us"])
        self.assertIsNone(result["fore_aft_angle_deg"])
        self.assertFalse(result["left_valid"])

    def test_no_outcome_fields_are_inputs(self):
        meta, event = fixture()
        features = CausalFootFeatures(meta)
        before = features.at(event)
        event.update(executed=True, output_end_ms=160, actual_elapsed_ms=50,
                     elapsed_work_pred_j=123, unmet_after_command_j=-99)
        self.assertEqual(features.at(event), before)

    def test_available_v37_archives(self):
        root = Path(__file__).resolve().parents[1] / "analysis" / "2026-11-17-walking"
        cases = {"46492793": (51, 2), "201405726": (86, 8)}
        for case, (decisions, assists) in cases.items():
            path = root / case / "metadata.json"
            if not path.exists():
                self.skipTest("Local v37 trial conversion unavailable")
            meta = json.loads(path.read_text(encoding="utf8"))
            features = CausalFootFeatures(meta)
            events = meta["walking_energy"]["records"]
            self.assertEqual(len(events), decisions)
            self.assertEqual(sum(e["return_event"] for e in events), assists)
            for event in events:
                row = features.at(event)
                self.assertIsNotNone(row["decision_capture_us"])
                self.assertLessEqual(row["foot_frame_us"], row["foot_receipt_us"])
                self.assertLessEqual(row["foot_receipt_us"], row["decision_capture_us"])
                if event["return_event"]:
                    # Firmware logged integer time_ms minus the frame's
                    # epoch-relative integer milliseconds, independently
                    # corroborating the selected camera observation.
                    self.assertEqual(event["return_foot_age_ms"], event["time_ms"]
                                     - (row["foot_frame_us"] - features.epoch_us) // 1000)


if __name__ == "__main__":
    unittest.main()
