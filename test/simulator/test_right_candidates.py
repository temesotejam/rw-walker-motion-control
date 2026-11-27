import json
import tempfile
import unittest
from pathlib import Path

import right_candidates as rc


class RightCandidateTests(unittest.TestCase):
    def test_prefers_first_two_distinct_runs_and_respects_guard(self):
        row = {'original_ms': 40, 'i0_ma': 0, 'goal_ma': 300, 'tau_s': .03}
        donor = {'run': 1, 'original_ms': 24, 'q_mas': rc.command_charge(row, 24)}
        second = {'run': 2, 'original_ms': 32, 'q_mas': rc.command_charge(row, 32)}
        matches = [donor, dict(donor), second]
        result, neighbors = rc.less_severe_cut(row, 18, matches)
        self.assertGreater(result, 18)
        self.assertEqual({r['run'] for r in neighbors}, {1, 2})
        for width in range(18, result):
            self.assertLess(len({r['run'] for r in rc.matching_charge(matches, row, width)}), 2)
        narrowed = dict(row, original_ms=result - 1)
        short, neighbors = rc.less_severe_cut(narrowed, 18, matches)
        self.assertLessEqual(short, narrowed['original_ms'])
        self.assertEqual({r['run'] for r in neighbors}, {1})

    def test_coast_does_not_support_power_and_empty_pool_has_no_proposal(self):
        row = {'original_ms': 10, 'i0_ma': 0, 'goal_ma': 300, 'tau_s': .03}
        coast = {'original_ms': 0, 'q_mas': 0., 'run': 1}
        self.assertLess(rc.command_charge(row, 1), rc.CHARGE_WINDOW_MAS)
        self.assertEqual(rc.matching_charge([coast], row, 1), [])
        self.assertEqual(rc.matching_charge([coast], row, 0), [coast])
        self.assertEqual(rc.less_severe_cut(row, 1, [coast]), (None, []))

    def test_followup_censoring_and_subsequent_commands(self):
        right = {'peak_time_ms': 200, 'physical_peak_side': 1, 'peak_amplitude_deg': 4.5}
        left = {'peak_time_ms': 400, 'physical_peak_side': -1, 'peak_amplitude_deg': 3.}
        def event(t, side):
            return {'zero_cross_time_ms': t, 'event_kind': 'energy_control_zero_cross',
                    'physical_next_peak_side': side, 'output_executed': True}
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            for peaks, end, expected in (([right], 500, None), ([right], 1250, False), ([right, left], 500, True)):
                with self.subTest(expected=expected):
                    data = {'energy_control_autonomous_peak_events': peaks,
                            'energy_control_autonomous_zero_cross_events': [event(100, 1), event(300, -1)]}
                    (folder / 'metadata.json').write_text(json.dumps(data))
                    (folder / 'timeseries.csv').write_text(f't_test_ms,state_id\n0,3\n{end},3\n2000,5\n')
                    rows = [{'run': 1234, 'zero_ms': 100, 'next_peak_deg': 4.5}]
                    observed, total = rc.return_observations({1234: folder}, rows)
                    self.assertEqual(total, 1)
                    self.assertIs(observed[1234, 100]['next_left'], expected)
                    self.assertTrue(observed[1234, 100]['further_command'])


if __name__ == '__main__':
    unittest.main()
