import unittest

from add_early_watch import compare_episodes, summarize


def episode(peak, outcome, score_status, states):
    return {
        'run': 5, 'peak_ms': peak, 'end_ms': 200, 'end_angle_deg': 0.,
        'outcome': outcome, 'score_status': score_status,
        'observations': [
            {'section_deg': angle, 'time_ms': stamp, 'warning': alarm, 'supported': alarm is not None,
             'interfering_pulse_starts_ms': [126] if score_status == 'intervening_pulse' else []}
            for angle, stamp, alarm in states
        ],
    }


class EarlyWatchTest(unittest.TestCase):
    def test_shared_denominator_includes_geometric_misses_and_abstentions(self):
        episodes = [
            episode(1, 'reversed', 'scored', [(4.5, 105, True)]),
            episode(2, 'reversed', 'outside_reference', [(4.5, 106, None)]),
            episode(3, 'reversed', 'scored', [(4.5, 105, True), (3, 160, True)]),
            episode(4, 'crossed', 'scored', [(4.5, 115, True), (3, 155, False)]),
            episode(5, 'reversed', 'intervening_pulse', [(4.5, 110, False), (3, 155, True)]),
            episode(6, 'censored', 'no_usable_section', []),
        ]
        rows = compare_episodes(episodes)
        self.assertEqual(rows[0]['early_warning_ms'], 105)
        result = summarize(rows)
        self.assertEqual(result['scored'], 4)
        self.assertEqual(result['reversed'], 3)
        self.assertEqual(result['two']['detected_reversals'], 1)
        self.assertEqual(result['three']['detected_reversals'], 2)
        self.assertEqual(result['additional_false_warnings'], 1)
        self.assertEqual(result['earlier_by_ms_on_common_true_alarms'], [55])


if __name__ == '__main__':
    unittest.main()
