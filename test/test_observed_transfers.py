import unittest

import _project
from score_observed_transfers import score, score_terminals


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

    def receiving(self, **changes):
        plan = dict(self.plan, phase=2, anchor=1, transfer_ms="", terminal_deg=-1.9, terminal_ms=6)
        plan.update(changes)
        return plan

    def terminal(self, points, plan=None):
        samples = [dict(self.gyro[0], body_rate_sample_time_us=1_000_000 + ms * 1000,
                        gy_dps=rate / .8 + .5, pitch_mekf_detector_relative_deg=angle)
                   for ms, angle, rate in points]
        return next(score_terminals(self.metadata, [plan or self.receiving()], samples))

    def test_receiving_prediction_does_not_date_a_past_contact(self):
        out = self.result([self.receiving()])
        self.assertEqual(out["status"], "no_prior_contact_forecast")
        self.assertNotIn("predicted_transfer_ms", out)
        self.assertEqual(list(score_terminals(self.metadata, [self.plan], self.gyro)), [])

    def test_absolute_terminal_and_interpolated_directional_rate(self):
        # The fixed target is beyond the nominal arrival. The event horizon
        # still closes when that target is reached without an intervening peak.
        points = [(458, -1.55, -20), (465, -1.7, -14), (475, -2.1, -6)]
        self.metadata["walking_energy"] = dict(archive_available=True, records=[], total=0, truncated=False)
        r = self.terminal(points)
        self.assertEqual(r["status"], "reached")
        self.assertEqual(r["predicted_terminal_ms"], 466)
        self.assertAlmostEqual(r["observation_end_ms"], 470)
        self.assertAlmostEqual(r["observed_rate_dps"], 10)
        self.assertTrue(r["carry_5dps"])
        self.assertEqual(r["recorded_command"], "no_recorded_command")
        self.metadata["walking_energy"].update(records=[dict(executed=True, output_start_ms=462,
                                                            output_end_ms=473)], total=1)
        self.assertEqual(self.terminal(points)["recorded_command"], "overlap")
        self.metadata["walking_energy"]["records"][0].update(output_start_ms=470)
        self.assertEqual(self.terminal(points)["recorded_command"], "uncertain_edge")
        r = self.terminal(points, self.receiving(ready_after_us=12_000))
        self.assertEqual(r["status"], "already_reached_at_ready")
        self.assertNotIn("carry_5dps", r)

    def test_reversal_and_gaps_censor_later_crossing(self):
        points = [(460, -.5, -10), (467, -.6, 2), (473, -.6, 4), (482, -2, -30)]
        r = self.terminal(points)
        self.assertEqual(r["status"], "reversed")
        self.assertFalse(r["carry_5dps"])
        self.assertEqual(r["recorded_command"], "unknown_archive")
        self.assertEqual(self.terminal(points[::2])["status"], "observation_gap")
        self.assertEqual(self.terminal([])["status"], "missing_initial_observation")
        self.metadata["energy_control_autonomous_peak_events"][1]["peak_time_ms"] = 472
        self.assertEqual(self.terminal(points)["status"], "next_peak")


if __name__ == "__main__":
    unittest.main()
