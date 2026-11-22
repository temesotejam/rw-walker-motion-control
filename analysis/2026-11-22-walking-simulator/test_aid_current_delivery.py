import unittest
from types import SimpleNamespace

import aid_current_delivery as delivery
import receiving_carry


class DeliveryTest(unittest.TestCase):
    def test_interpolated_signed_charge_and_missing_interval(self):
        points = [(5., 5.), (15., -15.), (40., -15.), (70., 0.)]
        clocks = [t for t, _ in points]
        self.assertAlmostEqual(delivery.bracketed_charge(points, clocks, 10., 30.), -.275)
        self.assertAlmostEqual(delivery.bracketed_charge(points, clocks, 15., 40.), -.375)
        self.assertIsNone(delivery.bracketed_charge(points, clocks, 10., 30., next_command_ms=35.))
        for start, end in ((4., 10.), (40., 50.), (70., 70.), (70., 71.)):
            self.assertIsNone(delivery.bracketed_charge(points, clocks, start, end))

    def test_driver_edges_use_first_change_and_exclude_intervening_commands(self):
        e = {'output_start_ms': 19, 'output_end_ms': 49}
        commands = [(10, 0), (20, -300), (30, -300), (50, 0), (55, 0), (60, -300)]
        self.assertEqual(delivery.command_window(commands, e), (20, 50, 60))
        self.assertIsNone(delivery.command_window([(20, -300), (35, 300), (50, 0)], e))
        self.assertIsNone(delivery.command_window([(20, -300), (60, 0)], e))

    def test_first_receiving_only_uses_same_direction_and_real_anchor_before_event(self):
        def frame(t, side, phase, anchor):
            return (t, t+3, dict(side=side, phase=phase, anchor=anchor))
        rows = [frame(80, '-1', '2', '2'), frame(30, '1', '2', '1'), frame(43, '-1', '2', '0'),
                frame(55, '-1', '2', '1')]
        self.assertEqual(delivery.first_receiving_capture(rows, 60), (55, 58, 1))
        self.assertIsNone(delivery.first_receiving_capture(rows, 50))

    def test_endpoint_tracking_starts_at_first_completed_receiving_solution(self):
        anchor = dict(side='-1', phase='2', anchor='1', terminal_deg='-1.91')
        dataset = SimpleNamespace(confirmations=[(100, 90, 1), (200, 185, -1), (280, 260, 1)],
                                  by_peak={90: [(110, 145, anchor), (120, 130, anchor)]},
                                  follow=lambda plan: dict(status='crossed', inward_speed_dps=24.))
        results = receiving_carry.first_receiving_outcomes(dataset)
        self.assertEqual([(r['peak_ms'], r['status']) for r in results], [(90, 'crossed'), (260, 'no_receiving_anchor')])
        self.assertEqual(results[0]['decision_ms'], 130)


if __name__ == '__main__':
    unittest.main()
