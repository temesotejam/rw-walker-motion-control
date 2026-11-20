"""Time-admission and outcome-label regressions for v37 support-transfer replay."""
from copy import deepcopy
from types import SimpleNamespace
import unittest

import _project
from replay_support_transfer import planning_queries, recorded_coast, score_cycle


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.epoch = 100000
        self.feet = SimpleNamespace(epoch_us=self.epoch, frames=[])
        self.meta = dict(
            model_tau_rise_min_ms=31.4, model_tau_rise_max_ms=72.8, model_tau_rise_u_mA=372,
            model_tau_rise_exponent=4.35, model_i_sat_at_reference_mA=329.55,
            model_i_sat_slope_mA_per_v=46.25, model_vbat_reference_v=7.5, model_i_sat_exponent=3.86,
            model_vbat_min_v=6.18, model_vbat_max_v=8.02,
            motor_driver_polarity=-1, normal_autonomous_motor_output_sign=-1, energy_control_autonomous_zero_cross_events=[],
            walking_energy=dict(combined_half_cycle_envelope_ms=80, records=[],
                                fore_aft_motion_guard=dict(start_roll_deg=1, start_sample_us=self.epoch,
                                candidate_angle_deg=6, minimum_outward_rate_dps=20, predicted_boundary_deg=20,
                                maximum_time_to_boundary_ms=350)),
            energy_control_autonomous_peak_events=[dict(peak_time_ms=10, peak_telemetry_capture_time_ms=49,
                                                       physical_peak_side=-1)])
        self.owner = [dict(now=self.epoch+t-100, imu=self.epoch+t-1100, rate=20, pulse=False,
                           current_valid=True, current_us=self.epoch+t-2000, roller_actual_current_mA=-17,
                           speed_valid=True, speed_end=self.epoch+t-3000, roller_battery_mV=7950,
                           roll_mekf_abs_deg=2, pitch_mekf_detector_relative_deg=-3, fore_rate=0)
                      for t in (25000,50000,75000,100000,125000,150000)]

    def commands(self, metadata=None):
        return planning_queries(metadata or self.meta, self.owner, self.feet)

    def test_admission_and_polarity_at_tick(self):
        # A snapshot from after 50 ms must never leak into the plan at 50 ms.
        late = {**self.owner[1], "now":self.epoch+50001, "pitch_mekf_detector_relative_deg":77}
        self.owner.insert(2, late)
        commands, rows = self.commands()
        self.assertEqual(rows[0]["time_ms"], 50)
        self.assertEqual(rows[0]["sway_deg"], -3)
        decision = next(line.split() for line in commands if line.startswith("D "))
        self.assertEqual(float(decision[6]), -17)  # v19 normal sign and driver polarity cancel

    def test_no_baseline_event_lookahead(self):
        pulse = dict(peak_ms=10, executed=True, completion_valid=True, output_start_ms=81,
                     output_end_ms=102, actual_elapsed_ms=21)
        self.meta["walking_energy"]["records"] = [pulse]
        original, closed = self.commands()
        later = deepcopy(self.meta)
        later["walking_energy"]["records"][0].update(output_start_ms=101, output_end_ms=122)
        _, open_longer = self.commands(later)
        self.assertEqual(closed[:2], open_longer[:2])
        self.assertEqual([r["time_ms"] for r in closed], [50,75,125])
        self.assertTrue(closed[-1]["prior_pulse"])
        self.assertEqual([s.split()[-1] for s in original if s.startswith("D ")], ["80","80","59"])

    def test_coast_label_checks_other_cycle_and_log_gaps(self):
        times = [1000*i for i in range(11)]
        samples = [dict(now=t, pulse=False, motor_cmd=0) for t in times]
        self.assertTrue(recorded_coast(2,8,0,times,samples,[]))
        self.assertFalse(recorded_coast(2,8,0,times,samples,[(0,3)]))
        samples[4]["motor_cmd"] = -1200
        self.assertFalse(recorded_coast(2,8,0,times,samples,[]))
        self.assertIsNone(recorded_coast(2,11,0,times,samples,[]))

    def test_directed_proxy_requires_contiguous_measured_pair(self):
        a = dict(ms=20, seq=1, gap=-1., rate=10, left=1, right=2, sway=-.5)
        b = dict(ms=40, seq=2, gap=1., rate=12, left=1.5, right=2, sway=.5)
        c = dict(ms=60, seq=3, gap=2., rate=14, left=2, right=2, sway=1.5)
        r = score_cycle(1, 100, 1, [], [a,b,c], [])
        self.assertEqual(r["proxy_interpolated_ms"], 30)
        self.assertEqual(r["exit_proxy_ms"], 50)
        self.assertEqual(r["opposite_forward_excursion_deg"], .5)
        b["seq"] = 11
        self.assertEqual(score_cycle(1,100,1,[],[a,b,c],[])["observed_gap_crossings"], 0)
        self.assertEqual(score_cycle(1,100,-1,[],[a,c],[])["observed_gap_crossings"], 0)


if __name__ == "__main__":
    unittest.main()
