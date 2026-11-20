"""Regression tests for the event joins used in the offline aid replay."""

import unittest

import _project  # Add the project tools directory for direct test runs.
from event_foot_outcomes import RecordedOutcomes


class RecordedOutcomesTests(unittest.TestCase):
    def setUp(self):
        self.aid = {"time_ms": 120, "peak_ms": 100, "side": -1,
                    "executed": True, "completion_valid": True, "actual_elapsed_ms": 14}
        self.zero = {
            "event_kind": "energy_control_zero_cross", "previous_peak_time_ms": 100,
            "physical_next_peak_side": -1, "zero_cross_time_ms": 155,
            "detector_angle_before_deg": 0.15, "detector_angle_after_deg": -0.03,
        }
        self.peak = {"peak_time_ms": 200, "physical_peak_side": -1, "peak_amplitude_deg": 3.0,
                     "target_peak_deg": 6.5, "target_energy_j": 0.0005}
        self.main = {"time_ms": 156, "peak_ms": 100, "side": -1, "rescue": False,
                     "optional": False, "executed": True, "valid": True, "request_j": 0.0002,
                     "completion_valid": True, "guarded_ms": 19, "actual_elapsed_ms": 20}
        self.meta = {
            "walking_energy": {"mechanical_reference_deg": 0, "records": [self.main]},
            "work_absolute_reference_enabled": True, "control_angle_origin_abs_deg": 0,
            "autonomous_input_peak_percent": 0, "autonomous_total_timing_projection_us": 0,
            "energy_control_autonomous_zero_cross_events": [self.zero],
            "energy_control_autonomous_peak_events": [self.peak],
            "metadata_event_detail_truncated": False, "energy_control_autonomous_event_overflow": False,
            "terminal_state": {"last_error": "fore_aft_outward_divergence"},
        }

    def outcome(self):
        return RecordedOutcomes(None, self.meta).for_aid(self.aid)

    def test_verified_cross_and_first_opposite_peak_with_main(self):
        self.meta["walking_energy"]["records"].extend([
            dict(self.main, time_ms=155, optional=True), dict(self.main, time_ms=155, rescue=True),
            dict(self.main, peak_ms=105), dict(self.main, time_ms=201),
        ])
        row = self.outcome()
        self.assertEqual((row["outcome_status"], row["zero_ms"], row["peak_ms"]),
                         ("zero_and_opposite_peak", 155, 200))
        self.assertEqual((row["main_elapsed_ms"], row["main_request_j"], row["peak_target_deg"]),
                         (20, 0.0002, 6.5))

    def test_invalid_command_does_not_invalidate_measured_crossing(self):
        self.zero.update(valid=False, output_executed=False)
        self.main.update(valid=False, executed=False, request_j=0, actual_elapsed_ms=0,
                         energy_unsupported_mask=0)
        row = self.outcome()
        self.assertTrue(row["zero_observed"])
        self.assertTrue(row["peak_observed"])
        self.assertIs(row["main_executed"], False)
        for field in ("main_request_j", "main_model_unsupported_mask", "main_elapsed_ms"):
            self.assertIsNone(row[field])

    def test_pseudo_cross_and_different_origin_are_excluded(self):
        self.meta["energy_control_autonomous_zero_cross_events"] = [
            dict(self.zero, event_kind="inward_stall_rescue"),
            dict(self.zero, event_kind="angle_rate_early_trial"),
            dict(self.zero, previous_peak_time_ms=90),
        ]
        row = self.outcome()
        self.assertIsNone(row["zero_observed"])
        self.assertIsNone(row["main_executed"])
        self.assertEqual(row["outcome_status"], "opposite_peak_without_verified_zero")

    def test_later_opposite_peak_cannot_replace_first_same_side_peak(self):
        self.meta["energy_control_autonomous_peak_events"].append(
            dict(self.peak, peak_time_ms=140, physical_peak_side=1))
        row = self.outcome()
        self.assertEqual(row["outcome_status"], "same_side_next_peak")
        self.assertIs(row["peak_observed"], False)
        self.assertIsNone(row["zero_ms"])
        self.assertIsNone(row["peak_ms"])

    def test_reference_and_signed_bracket_are_checked_separately(self):
        self.meta["autonomous_input_peak_percent"] = 25
        self.assertEqual(self.outcome()["outcome_status"], "unsupported_mechanical_reference")
        self.assertIsNone(self.outcome()["zero_ms"])
        self.meta["autonomous_input_peak_percent"] = 0
        self.zero["detector_angle_after_deg"] = 0.02
        self.assertEqual(self.outcome()["outcome_status"], "unverified_zero")
        self.assertIsNone(self.outcome()["zero_ms"])

    def test_estop_right_censors_unobserved_endpoint(self):
        self.meta["energy_control_autonomous_peak_events"] = []
        self.meta["energy_control_autonomous_zero_cross_events"] = []
        row = self.outcome()
        self.assertEqual(row["outcome_status"], "censored_before_zero")
        self.assertIsNone(row["peak_observed"])
        self.assertEqual(row["terminal_reason"], "fore_aft_outward_divergence")


if __name__ == "__main__":
    unittest.main()
