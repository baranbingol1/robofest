import unittest

from ders_cizim.controllers.rgb_rl_controller.hardware_probe import summarize_profile_records


class HardwareProbeTests(unittest.TestCase):
    def test_summarize_profile_records_reports_camera_calibration_ranges(self):
        records = [
            {"visible": True, "line_width_ratio": 0.12, "confidence": 0.9},
            {"visible": True, "line_width_ratio": 0.18, "confidence": 1.0},
            {"visible": False, "line_width_ratio": 0.0, "confidence": 0.0},
        ]
        summary = summarize_profile_records(records)
        self.assertEqual(summary["frames"], 3)
        self.assertAlmostEqual(summary["visible_ratio"], 2 / 3)
        self.assertEqual(summary["min_line_width_ratio"], 0.0)
        self.assertEqual(summary["max_line_width_ratio"], 0.18)
        self.assertFalse(summary["camera_ready"])

    def test_summarize_profile_records_marks_ready_for_good_camera_signal(self):
        records = [
            {"visible": True, "line_width_ratio": 0.13, "confidence": 0.9},
            {"visible": True, "line_width_ratio": 0.16, "confidence": 0.95},
            {"visible": True, "line_width_ratio": 0.18, "confidence": 0.92},
        ]
        summary = summarize_profile_records(records)
        self.assertTrue(summary["camera_ready"])


if __name__ == "__main__":
    unittest.main()
