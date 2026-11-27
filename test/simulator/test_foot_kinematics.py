import unittest

from foot_kinematics import FootHistory


def frame(seq, time_ms, left, right):
    return dict(sequence=seq, measurement_time_us=time_ms * 1000, left_deg=left, right_deg=right,
                timestamp_valid=True, zero_ready=True, frame_valid=True, left_valid=True,
                right_valid=True, left_control_range_ok=True, right_control_range_ok=True)


def packet(time_ms, left, right):
    return dict(publish_us=time_ms * 1000, left_sequence=left, right_sequence=right)


class FootHistoryTests(unittest.TestCase):
    def test_velocity_uses_unique_measurement_times_not_publication_or_holds(self):
        frames = [frame(1, 10, 1, 4), frame(2, 30, 2, 2), frame(3, 50, 3, 0)]
        hist = FootHistory([packet(20, 1, 1), packet(55, 2, 1), packet(64, 2, 2),
                            packet(90, 3, 3)], frames)
        self.assertIsNone(hist.at(19).left)
        self.assertIsNone(hist.at(54).left.rate_dps)
        self.assertAlmostEqual(hist.at(55).left.rate_dps, 50)
        self.assertIsNone(hist.at(63).right.rate_dps)
        self.assertAlmostEqual(hist.at(64).right.rate_dps, -100)
        self.assertEqual(hist.at(89).left.frames, 2)
        self.assertEqual(hist.at(89).left.age_ms, 59)
        self.assertEqual(hist.at(90).left.frames, 3)
        self.assertIsNotNone(hist.at(125).left)
        self.assertIsNone(hist.at(125.01).left)

    def test_invalidation_is_local_and_discards_old_slope(self):
        fs = [frame(1, 10, 1, 4), frame(2, 30, 2, 2), frame(3, 50, 3, 0)]
        fs[1]['left_control_range_ok'] = False
        fs[1]['left_in_range'] = True
        h = FootHistory([packet(20, 1, 1), packet(40, 2, 2), packet(60, 3, 3)], fs)
        self.assertIsNone(h.at(40).left)
        self.assertAlmostEqual(h.at(40).right.rate_dps, -100)
        self.assertIsNone(h.at(60).left.rate_dps)
        fs[0]['zero_ready'] = False
        self.assertIsNone(FootHistory([packet(20, 1, 1)], fs).at(20).right)

    def test_unavailable_and_future_dated_sources_and_long_gap(self):
        fs = [frame(1, 10, 1, 4), frame(2, 60, 2, 2), frame(3, 160, 3, 0)]
        h = FootHistory([packet(20, 1, 1), packet(55, 2, 99), packet(180, 3, 3)], fs)
        self.assertIsNone(h.at(55).left)
        self.assertIsNone(h.at(55).right)
        self.assertIsNone(h.at(180).left.rate_dps)


if __name__ == '__main__':
    unittest.main()
