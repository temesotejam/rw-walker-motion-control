"""Coverage of the staged shadow selector. Run with python test/test_event_foot_energy_candidate.py."""
import unittest

import _project  # Add the project tools directory for direct test runs.
from event_foot_energy_candidate import evaluate_candidate, reachability_first


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.event = dict(return_event=True, event_kind='inward_stall_rescue', time_ms=2000, peak_ms=1800,
                          side=-1, valid=True, energy_admission_allowed=True, fore_aft_allowed=True,
                          fore_aft_reason=0, return_evidence=1, request_j=.000300, target_j=.0006,
                          envelope_before_ms=80, guarded_ms=70, decision_i0_ma=-2., decision_goal_ma=350.,
                          decision_rate_dps=22., decision_tau_s=.032)
        self.context = dict(left_deg=17., right_deg=0., left_phase='returning', right_phase='holding',
                            left_valid=True, right_valid=True, foot_age_ms=45, fore_aft_angle_deg=-1.,
                            fore_aft_rate_dps=-25.)
        self.past = dict(self.event, time_ms=1030, peak_ms=860, output_end_ms=1098,
                         event_kind='energy_control_zero_cross', return_event=False,
                         assisted_cycle=False, executed=True, completion_valid=True, coast=False,
                         request_j=.0005, guarded_ms=68, decision_rate_dps=40.)

    def choose(self, **context):
        return evaluate_candidate(self.event, dict(self.context, **context), [self.past])

    def test_two_stage_model_tradeoff(self):
        c = self.choose()
        self.assertEqual(c['status'], 'shadow')
        self.assertEqual(c['candidate_aid_ms'] + c['reserve_ms'], 80)
        self.assertLess(c['candidate_aid_ms'], c['baseline_aid_ms'])
        self.assertGreater(c['candidate_aid_unmet_j'], c['baseline_aid_unmet_j'])
        self.assertLess(c['worst_main_proxy_unmet_j'], c['baseline_worst_main_proxy_unmet_j'])
        self.assertFalse(c['candidate_aid_work_rate_supported'])
        outward = self.choose(left_phase='outgoing')
        self.assertEqual(outward['candidate_aid_ms'], c['reserve_priority_aid_ms'])
        self.assertLess(outward['candidate_aid_ms'], c['candidate_aid_ms'])
        self.assertEqual(self.choose(left_phase='outgoing', fore_aft_rate_dps=25.)['candidate_aid_ms'],
                         c['balanced_aid_ms'])

    def test_unknown_phase_keeps_conditional_alternatives(self):
        c = self.choose(left_phase='unknown')
        self.assertIsNone(c['candidate_aid_ms'])
        self.assertEqual(c['reason'], 'unknown_leading_foot_phase')
        self.assertLess(c['reserve_priority_aid_ms'], c['balanced_aid_ms'])
        for context in ({'left_valid': False}, {'foot_age_ms': 76}, {'left_deg': float('nan')},
                        {'fore_aft_angle_deg': None}):
            self.assertIsNone(self.choose(**context)['candidate_aid_ms'])

    def test_history_must_precede_and_match_event(self):
        for change in ({'output_end_ms': 2000}, {'peak_ms': 1800}, {'assisted_cycle': True},
                       {'target_j': .00059}, {'side': 1}, {'event_kind': 'angle_rate_early_trial'}):
            with self.subTest(change=change):
                history = [dict(self.past, **change)]
                c = evaluate_candidate(self.event, self.context, history)
                self.assertEqual(c['main_proxy_count'], 0)
                self.assertIsNone(c['candidate_aid_ms'])
        coast = dict(self.past, executed=False, completion_valid=False, coast=True, request_j=0.,
                     guarded_ms=0, decision_rate_dps=None)
        c = evaluate_candidate(self.event, self.context, [coast])
        self.assertEqual(c['main_proxy_count'], 1)
        self.assertEqual(c['worst_main_proxy_unmet_j'], 0.)

    def test_crossing_first_never_invents_extra_local_work(self):
        preserved = reachability_first(self.event)
        self.assertEqual(preserved['reachability_first_aid_ms'], self.event['guarded_ms'])
        self.assertGreater(preserved['reachability_first_unmet_j'], 0.)
        small = reachability_first(dict(self.event, request_j=.000010))
        self.assertLess(small['reachability_first_aid_ms'], self.event['guarded_ms'])
        self.assertEqual(small['reachability_first_unmet_j'], 0.)
        vetoed = dict(self.event, fore_aft_allowed=False)
        self.assertIsNone(reachability_first(vetoed)['reachability_first_aid_ms'])
        self.assertIsNone(evaluate_candidate(vetoed, self.context, [self.past])['candidate_aid_ms'])


if __name__ == '__main__':
    unittest.main()
