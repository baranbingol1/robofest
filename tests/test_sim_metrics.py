import unittest

from ders_cizim.controllers.rgb_rl_controller.sim_metrics import RunSummary, summarize_records


class SimMetricsTests(unittest.TestCase):
    def test_summarize_records_reports_visibility_target_lock_and_distance(self):
        records = [
            {
                "step": 1,
                "visible": True,
                "matched_target": False,
                "target_seen": False,
                "translation": [0.0, 0.0, 0.1],
            },
            {
                "step": 2,
                "visible": True,
                "matched_target": True,
                "target_seen": False,
                "translation": [0.3, 0.0, 0.1],
            },
            {
                "step": 3,
                "visible": True,
                "matched_target": True,
                "target_seen": True,
                "translation": [0.3, 0.4, 0.1],
                "goal_center": [0.3, 0.5],
                "goal_radius": 0.2,
                "terminal_reason": "reached_goal",
            },
        ]
        summary = summarize_records(records)
        self.assertIsInstance(summary, RunSummary)
        self.assertEqual(summary.steps, 3)
        self.assertEqual(summary.first_target_seen_step, 3)
        self.assertAlmostEqual(summary.visible_ratio, 1.0)
        self.assertAlmostEqual(summary.matched_target_ratio, 2 / 3)
        self.assertAlmostEqual(summary.travel_distance, 0.7)
        self.assertAlmostEqual(summary.final_goal_distance, 0.1)
        self.assertAlmostEqual(summary.final_goal_margin, 0.1)
        self.assertFalse(summary.off_board)
        self.assertTrue(summary.reached_goal)
        self.assertEqual(summary.terminal_reason, "reached_goal")
        self.assertTrue(summary.passed_smoke_gate(require_goal_reached=True))

    def test_goal_margin_gate_fails_near_boundary_successes(self):
        summary = summarize_records(
            [
                {
                    "step": 1,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.0, 0.0, 0.1],
                },
                {
                    "step": 2,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.19, 0.0, 0.1],
                    "goal_center": [0.0, 0.0],
                    "goal_radius": 0.2,
                    "terminal_reason": "reached_goal",
                },
            ]
        )
        self.assertAlmostEqual(summary.final_goal_margin, 0.01)
        self.assertFalse(summary.passed_smoke_gate(require_goal_reached=True, min_goal_margin=0.02))

    def test_summarize_records_flags_off_board_pose(self):
        summary = summarize_records(
            [
                {
                    "step": 1,
                    "visible": True,
                    "matched_target": False,
                    "target_seen": False,
                    "translation": [1.2, 0.0, 0.1],
                }
            ]
        )
        self.assertTrue(summary.off_board)
        self.assertFalse(summary.passed_smoke_gate())

    def test_passed_smoke_gate_can_require_goal_reached(self):
        summary = summarize_records(
            [
                {
                    "step": 1,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.0, 0.0, 0.1],
                },
                {
                    "step": 2,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.3, 0.0, 0.1],
                },
            ]
        )
        self.assertTrue(summary.passed_smoke_gate())
        self.assertFalse(summary.passed_smoke_gate(require_goal_reached=True))

    def test_lost_line_terminal_fails_gate(self):
        summary = summarize_records(
            [
                {
                    "step": 1,
                    "visible": False,
                    "matched_target": False,
                    "target_seen": True,
                    "translation": [0.0, 0.0, 0.1],
                    "terminal_reason": "lost_line",
                }
            ]
        )
        self.assertTrue(summary.lost_line)
        self.assertFalse(summary.passed_smoke_gate(require_target_lock=False))

    def test_timeout_terminal_fails_gate(self):
        summary = summarize_records(
            [
                {
                    "step": 1,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.0, 0.0, 0.1],
                },
                {
                    "step": 2,
                    "visible": True,
                    "matched_target": True,
                    "target_seen": True,
                    "translation": [0.3, 0.0, 0.1],
                    "terminal_reason": "timeout",
                },
            ]
        )
        self.assertTrue(summary.timed_out)
        self.assertFalse(summary.passed_smoke_gate())


if __name__ == "__main__":
    unittest.main()
