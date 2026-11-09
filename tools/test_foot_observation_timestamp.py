import unittest
import convert_rwlog_to_csv as converter

class FootObservationTimestampTests(unittest.TestCase):
    def test_sparse_table_reconstructs_and_leaves_missing_unknown(self):
        data = {"foot_frames": [{"delivered_us": 123}, {"delivered_us": 1000}],
                "foot_control_observations": {"version": 1, "total": 2, "stored": 1,
                                              "truncated": True, "rows": [[1, 0]]}}
        result = converter.expand_tables(data)
        self.assertNotIn("control_observed_us", result["foot_frames"][0])
        self.assertEqual(result["foot_frames"][1]["control_observed_us"], 1000)
        self.assertTrue(result["foot_control_observations"]["truncated"])

    def test_invalid_indices_deltas_and_duplicates_are_rejected(self):
        for rows in ([[0, -1]], [[1, 0]], [[True, 1]], [[0, 1], [0, 2]]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                converter.expand_tables({"foot_frames": [{"delivered_us": 1}],
                   "foot_control_observations": {"version": 1, "rows": rows}})

    def test_legacy_without_table_still_decodes(self):
        frames = [{"delivered_us": 100}]
        self.assertEqual(converter.expand_tables({"foot_frames": frames})["foot_frames"], frames)

if __name__ == "__main__":
    unittest.main()
