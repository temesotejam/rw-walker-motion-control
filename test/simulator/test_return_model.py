"""Temporal alignment, exposure and run independence for the return model."""

import math
import unittest

from return_model import Observation, State, evaluate_episodes, first_outcome, foot_at, predict


def example(run, section=3., outcome='reversed', intervening=(), speed=10., peak_ms=100):
    t = peak_ms + (100 if section == 3. else 130)
    state = State(run, peak_ms, section, t, section, speed, -20., 4., 1., 25., t - 1.)
    observed_speed = 12. if outcome == 'crossed' else 0.
    return Observation(state, outcome, peak_ms + 200, observed_speed, bool(intervening), intervening, 60)


class ReturnModelTests(unittest.TestCase):
    def test_only_published_foot_values_are_visible(self):
        old = dict(left_sequence=1, right_sequence=2, left_observed_deg=3., right_observed_deg=1.,
                   left_age_us=10000, right_age_us=20000, slot_us=15000)
        new = dict(old, left_observed_deg=99.)
        self.assertEqual(foot_at([old, new], [21000, 26001], 26), (3., 1., 25., 21.))
        self.assertTrue(math.isnan(foot_at([old, new], [21000, 26001], 20)[0]))
        self.assertTrue(math.isnan(foot_at([old], [21000], 77)[0]))

    def test_reversal_crossing_and_observation_gap(self):
        t = [0, 10, 20, 30, 40]
        # An arrival stops at reversal, even if a later sample crosses zero.
        result = first_outcome(t, [3., 2., 1.5, 1.8, -.1], [12., 8., .1, -4., 4.], 0, 5, None)
        self.assertEqual(result, ('reversed', 2, 20, 0., 30))
        result = first_outcome(t, [3., 2., 1., -.5, -1.], [10.]*5, 0, 5, None)
        self.assertEqual(result[0], 'crossed')
        self.assertAlmostEqual(result[2], 20 + 10/1.5)
        self.assertEqual(first_outcome([0, 10, 45], [3., 2., -1.], [10.]*3, 0, 3, None)[0], 'censored')

    def test_unique_analogs_exclude_same_run_and_intervention(self):
        train = [example(10), example(10, peak_ms=200), example(11), example(12, outcome='crossed'),
                 example(13, intervening=(230,)), example(99, outcome='crossed')]
        pred = predict(train, example(99).state)
        self.assertEqual({run for run, _ in pred['analogs']}, {10, 11, 12})
        self.assertAlmostEqual(pred['cross_fraction'], 1/3)
        self.assertAlmostEqual(pred['zero_speed'], 4.)
        energy = pred['energy_per_inertia']
        self.assertEqual(energy['below_carry_count'], 2)
        self.assertEqual(energy['median'], 0.)
        self.assertAlmostEqual(energy['mean'], 0.5 * math.radians(12.)**2 / 3)
        self.assertAlmostEqual(energy['mean_positive_deficit'], math.radians(5.)**2 / 3)
        self.assertIsNone(predict(train[:2], example(99).state))

    def test_first_warning_and_intervening_pulse_exclusion(self):
        train = [example(run, a) for run in (1, 2, 3) for a in (3., 2.)]
        clean = [example(4, 3.), example(4, 2.)]
        aided = [example(5, 3., intervening=(215,)), example(5, 2.)]
        outside = [example(6, 3., speed=200.)]
        report = evaluate_episodes(train, clean + aided + outside)
        self.assertEqual(report['counts'], {'scored':1, 'intervening_pulse':1, 'outside_reference':1})
        self.assertEqual(report['episodes'][0]['warning_ms'], 200)
        self.assertEqual(report['metrics']['warning']['detected_reversals'], 1)
        self.assertEqual(report['metrics']['warning']['detected_reversal_leads_ms'], [100])
        self.assertIsNone(report['episodes'][2]['warning_ms'])


if __name__ == '__main__':
    unittest.main()
