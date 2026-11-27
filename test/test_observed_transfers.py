import unittest

import _project
from score_observed_transfers import score, score_terminals
from measure_transfer_speeds import align_video_labels, command_archive, input_evidence, led_edges, measure, video_clock


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


class TransferMeasurements(unittest.TestCase):
    def setUp(self):
        self.header = dict(run_start_us=1_000_000, run_id=4)
        self.meta = dict(mekf_gyro_y_scale=.8, energy_control_autonomous_zero_cross_events=[],
                         foot_frames=[dict(run_id=4, state_id=3, frame_us=1_200_000,
                                          timestamp_valid=True, measurement_time_us=100_000)],
                         walking_energy=dict(archive_available=True, truncated=False, total=0, records=[]),
                         energy_control_autonomous_event_overflow=False,
                         energy_control_autonomous_zero_cross_event_total_count=0,
                         walking_restart=dict(columns=["start_ms", "queued", "elapsed_ms"], rows=[]))
        self.rows = [dict(state_id=3, t_test_ms=t, gy_dps=str(t-102), mekf_bias_y_dps=1,
                          pitch_mekf_detector_relative_deg=0, body_rate_sample_time_us=1_100_000+t*1000)
                     for t in (96, 100, 104, 104, 108, 112, 116)]
        self.label = dict(run_start_us=1_000_000, start_ms=100, end_ms=112,
                          receiving_foot="L", body_direction=-1, source="video_proxy", event_kind="old_foot_liftoff")

    def test_samples_strictly_outside_both_edges_and_reversal_keeps_sign(self):
        left, right = measure(self.header, self.meta, self.rows,
                              [self.label, dict(self.label, receiving_foot="R", body_direction=1)])
        self.assertEqual((left["pre_ms"], left["post_ms"]), (96, 116))
        self.assertEqual((left["imu_status"], left["foot_status"]), ("ok", "no_received_frame"))
        self.assertAlmostEqual(left["pre_directional_dps"], 5.6)
        self.assertAlmostEqual(left["post_directional_dps"], -10.4)
        self.assertEqual(right["post_directional_dps"], -left["post_directional_dps"])
        self.assertEqual(left["command_overlap"], "no_recorded_command")
        self.assertEqual(left["current_samples"], 0)
        self.assertNotIn("max_abs_measured_current_ma", left)

    def test_missing_data_and_wrong_run(self):
        gap = measure(self.header, self.meta, self.rows[:1]+self.rows[-1:], [self.label])[0]
        self.assertEqual(gap["imu_status"], "imu_gap")
        self.assertNotIn("delta_directional_dps", gap)
        with self.assertRaises(ValueError):
            measure(self.header, self.meta, [], [dict(self.label, run_start_us=123)])
        self.rows[2]["motor_cmd_mA"] = 50
        self.assertEqual(measure(self.header, self.meta, self.rows, [self.label])[0]["command_overlap"], "overlap")

    def test_video_brackets_include_drift_and_sync_width(self):
        samples = [dict(log_time_s=t/1000, led_state=s) for t,s in
                   ((10,0), (14,1), (111,1), (115,0), (212,0), (216,1))]
        samples.insert(2, dict(log_time_s=0, led_state=0, state_id=3, sync_event_id=2))
        edges = led_edges(self.header, self.meta, samples)
        self.assertEqual([e["state"] for e in edges], [1,0,1])
        self.assertAlmostEqual(edges[0]["log_lo_ms"], -90)
        anchors = [dict(run_start_us=1_000_000, edge=i+1, state=e["state"],
                        video_lo_ms=1000+100*i, video_hi_ms=1004+100*i) for i,e in enumerate(edges)]
        clock = video_clock(1_000_000, edges, anchors)
        label = dict(self.label, video_lo_ms=1150, video_hi_ms=1155)
        row = next(align_video_labels([label], clock))
        # One feasible rate/offset maps 1000 video ms to -90 measurement ms.
        self.assertLessEqual(row["start_ms"], -90+150*1.01)
        self.assertGreaterEqual(row["end_ms"], -90+155*1.01)
        self.assertGreater(row["end_ms"]-row["start_ms"], 5)
        with self.assertRaises(ValueError):
            list(align_video_labels([dict(label,video_hi_ms=1205)], clock))
        with self.assertRaises(ValueError):
            video_clock(1_000_000, edges, [anchors[0], dict(anchors[1], video_lo_ms=1120, video_hi_ms=1124), anchors[2]])
        with self.assertRaises(ValueError):
            video_clock(1_000_000, edges, [anchors[0], dict(anchors[1], state=1), anchors[2]])

    def test_launch_before_rounded_zero_and_repeated_nonzero_commands(self):
        ticks = [dict(state_id=3, t_test_ms=t, roller_applied_command_time_us=1_100_000+us,
                      roller_applied_current_mA=cmd) for t,us,cmd in
                 ((0,-19,300), (25,24_999,300), (100,99_766,0))]
        self.meta["energy_control_autonomous_zero_cross_events"] = [
            dict(event_kind="strong_start_kick", output_executed=True, pulse_start_ms=0, pulse_end_ms=100)]
        _, _, applied = input_evidence(ticks, 1_100_000)
        self.assertAlmostEqual(command_archive(self.meta, applied)["records"][0]["output_end_ms"], 99.766)
        self.assertEqual(command_archive(self.meta, applied[:-1])["records"][0]["output_end_ms"], 0)


if __name__ == "__main__":
    unittest.main()
