import unittest

from post_zero_carry import score_prediction


def row(run, time=300, speed=12., arrival=16., **extra):
    values = dict(run=run, zero_ms=time, status='crossed', zero_speed_dps=speed,
                  arrival_dps=arrival, powered=True, assisted=False, terminal_deg=-1.9,
                  charge_mas=6., width_ms=35., foot_gap_deg=1.)
    return {**values, **extra}


class PostZeroCarryTests(unittest.TestCase):
    def test_same_run_or_different_action_never_supports_query(self):
        query = row(4)
        refs = [row(4), row(1), row(1, time=350), row(2, powered=False), row(3, assisted=True),
                row(5, status='another_pulse'), row(6, terminal_deg=-2.5)]
        p = score_prediction(refs, query)
        self.assertEqual(p['status'], 'insufficient_similar_runs')
        self.assertEqual(p['donors'], [[1, 300]])

    def test_predicted_energy_ignores_query_outcome(self):
        query = row(4, arrival=0.)
        refs = [row(1, arrival=18.), row(2, arrival=14.)]
        p = score_prediction(refs, query)
        self.assertEqual(p, score_prediction(refs, dict(query, status='censored', arrival_dps=None)))
        self.assertAlmostEqual(p['predicted_dps']**2, (18.**2 + 14.**2)/2)


if __name__ == '__main__':
    unittest.main()
