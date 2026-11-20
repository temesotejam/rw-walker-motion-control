import unittest

import _project
from score_observed_transfers import score


class ObservedTransfers(unittest.TestCase):
    def setUp(self):
        self.metadata = {
            "mekf_gyro_y_scale": .8,
            "foot_frames": [{"state_id": 3, "timestamp_valid": True, "frame_us": 1_900_000,
                             "measurement_time_us": 900_000}],
            "energy_control_autonomous_peak_events": [{"peak_time_ms": 300}, {"peak_time_ms": 800}],
        }
        self.label = {"start_ms": 500, "end_ms": 510, "receiving_foot": "L", "body_direction": "-1",
                      "opposite_foot_swung": "", "forward_mm": ""}
        self.plan = dict(time_ms=460, peak_ms=300, side=-1, packet=8, reason=3, delay_ms=5, width_ms=20,
                         transfer_ms=56, coast_margin_j=-.001, margin_j=.001, ready_after_us=500,
                         capture_us32=1_460_000)
        self.gyro = [dict(state_id=3, body_rate_sample_time_us=1_511_250, gy_dps="-16", mekf_bias_y_dps=".5",
                          pitch_mekf_detector_relative_deg="0")]

    def result(self, plans):
        return next(score(self.metadata, plans, self.gyro, [self.label]))

    def test_only_completed_decisions_before_independent_event(self):
        late = dict(self.plan, time_ms=495, packet=9, capture_us32=1_495_000, ready_after_us=8000)
        foreign = dict(self.plan, time_ms=488, peak_ms=100)
        r = self.result([self.plan, late, foreign])
        self.assertEqual(r["decision_ms"], 460)
        self.assertEqual(r["timing_error_to_interval_ms"], 6)
        self.assertAlmostEqual(r["observed_directional_rate_dps"], 13.2)
        self.assertIsNone(r["forward_mm"])
        self.assertEqual(r["opposite_foot_swung"], "")

    def test_unusable_latest_plan_and_missing_gyro(self):
        self.gyro = []
        r = self.result([self.plan, dict(self.plan, time_ms=480, capture_us32=1_480_000, transfer_ms="", reason=0)])
        self.assertEqual(r["status"], "no_forecast")
        self.assertNotIn("observed_directional_rate_dps", r)
        self.assertEqual(self.result([])["status"], "no_prior_recorded_decision")

    def test_capture_wrap_and_missing_measurement_clock(self):
        epoch = 2**32 - 100_000
        self.metadata["foot_frames"][0]["frame_us"] = epoch + 900_000
        self.plan["capture_us32"] = (epoch + 460_000) % 2**32
        self.gyro[0]["body_rate_sample_time_us"] = (epoch + 511_250) % 2**32
        self.assertAlmostEqual(self.result([self.plan])["observed_directional_rate_dps"], 13.2)
        self.assertEqual(self.result([self.plan])["decision_ms"], 460)
        self.metadata["foot_frames"] = []
        self.assertEqual(self.result([self.plan])["status"], "unknown_measurement_clock")

    def test_measured_exit_or_censored_gap(self):
        self.gyro.append(dict(self.gyro[0], body_rate_sample_time_us=1_517_000, pitch_mekf_detector_relative_deg=-1.01))
        self.assertTrue(self.result([])["exit_carry_5dps"])
        self.gyro[1]["body_rate_sample_time_us"] = 1_525_000
        self.assertEqual(self.result([])["exit_observation"], "observation_gap")

    def test_window_straddles_peak_and_direction_disagrees(self):
        r = self.result([dict(self.plan, side=1, transfer_ms=45)])
        self.assertFalse(r["direction_agrees"])
        self.assertEqual(r["status"], "wrong_direction")
        self.assertEqual(r["timing_error_to_interval_ms"], 0)
        self.label["receiving_foot"] = "R"
        self.assertEqual(self.result([self.plan])["status"], "wrong_foot")
        self.label.update(start_ms=797, end_ms=802)
        self.assertEqual(self.result([self.plan])["status"], "ambiguous_half_cycle")


if __name__ == "__main__":
    unittest.main()
