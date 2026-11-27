import unittest

from receiving_carry import CarryReplay


class CarryTest(unittest.TestCase):
    def replay(self):
        replay = CarryReplay.__new__(CarryReplay)
        replay.reasons = ['test']
        replay.peaks = [0, 120]
        replay.confirmations = [(1, 0, 1), (122, 120, -1)]
        replay.archive = {'records': [], 'archive_available': True, 'truncated': False, 'total': 0}
        row = {'phase': '2', 'side': '-1', 'anchor': '1', 'gate_mask': '0', 'reason': '0',
               'cap_ms': '40', 'baseline_flags': '0', 'packet': '2', 'terminal_deg': '-1.9',
               'coast_margin_j': '-0.001'}
        replay.by_peak = {0: [(3., 6., row), (10., 20., dict(row, phase='1', packet='3', terminal_deg='5'))]}
        return replay

    def test_latest_completed_at_time_only(self):
        replay = self.replay()
        self.assertEqual(replay.at(6)['status'], 'no_completed_geometry')
        self.assertEqual(replay.at(12)['terminal_deg'], -1.9)
        self.assertIsNone(replay.at(22)['terminal_deg'])
        self.assertIsNone(replay.at(122)['terminal_deg'])

    def test_two_reverse_gyro_samples_before_true_arrival(self):
        replay = self.replay()
        replay.gyro = (list(range(0, 66, 5)), [-20]*5 + [1, 2] + [-25]*7,
                       [1.5, 1.4, 1.3, 1.1, .9, .8, .81, .6, .1, -.8, -1.5, -1.8, -2.0, -2.1])
        result = replay.follow(replay.at(12))
        self.assertEqual(result['status'], 'crossed')
        self.assertTrue(result['coast_scored'])
        self.assertTrue(result['at_least_5_dps'])

    def test_pulse_before_reversal_confirmation_censors_coast(self):
        replay = self.replay()
        replay.gyro = (list(range(0, 46, 5)), [-15]*6 + [2]*4,
                       [1.5, 1.4, 1.3, 1.2, .8, .4, .42, .47, .54, .67])
        replay.archive.update(total=1, records=[{'executed': True, 'output_start_ms': 32, 'output_end_ms': 38}])
        result = replay.follow(replay.at(12))
        self.assertEqual((result['end_ms'], result['confirmed_ms']), (25, 45))
        self.assertFalse(result['coast_scored'])
        self.assertEqual(result['recorded_command'], 'overlap')
