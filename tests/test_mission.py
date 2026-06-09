import random
import unittest

from ders_cizim.controllers.rgb_rl_controller.mission import (
    DEFAULT_GOAL_ZONES,
    GoalZone,
    evaluate_terminal_reason,
    mission_config_from_env,
    parse_goal_zones,
    randomized_start_pose,
)


class MissionTests(unittest.TestCase):
    def test_goal_zone_contains_points_inside_radius(self):
        zone = GoalZone("red", (0.8, -0.55), 0.2)
        self.assertTrue(zone.contains([0.9, -0.55, 0.1]))
        self.assertFalse(zone.contains([0.5, -0.55, 0.1]))

    def test_parse_goal_zones_overrides_known_colors_only(self):
        zones = parse_goal_zones("red=0.7:-0.4:0.12,orange=0:0:1,blue=bad")
        self.assertEqual(zones["red"].center, (0.7, -0.4))
        self.assertAlmostEqual(zones["red"].radius, 0.12)
        self.assertEqual(zones["blue"], DEFAULT_GOAL_ZONES["blue"])

    def test_mission_config_ignores_invalid_board_extent(self):
        config = mission_config_from_env({"MONSTERBORG_RL_BOARD_HALF_EXTENT": "wide"})
        self.assertAlmostEqual(config.board_half_extent, 1.0)

    def test_evaluate_terminal_reason_requires_goal_clearance(self):
        config = mission_config_from_env({})
        zone = config.zones["red"]
        near_boundary = [
            zone.center[0] + zone.radius - 0.005,
            zone.center[1],
            0.1,
        ]
        self.assertIsNone(
            evaluate_terminal_reason(
                target_color="red",
                translation=near_boundary,
                target_seen=True,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=100,
                max_steps=900,
                config=config,
            )
        )
        clear_inside = [
            zone.center[0] + zone.radius - config.goal_reach_clearance - 0.005,
            zone.center[1],
            0.1,
        ]
        self.assertEqual(
            evaluate_terminal_reason(
                target_color="red",
                translation=clear_inside,
                target_seen=True,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=100,
                max_steps=900,
                config=config,
            ),
            "reached_goal",
        )

    def test_goal_clearance_can_be_overridden_and_invalid_values_are_ignored(self):
        self.assertAlmostEqual(
            mission_config_from_env({"MONSTERBORG_RL_GOAL_REACH_CLEARANCE": "0.03"}).goal_reach_clearance,
            0.03,
        )
        self.assertAlmostEqual(
            mission_config_from_env({"MONSTERBORG_RL_GOAL_REACH_CLEARANCE": "-0.01"}).goal_reach_clearance,
            mission_config_from_env({}).goal_reach_clearance,
        )

    def test_evaluate_terminal_reason_requires_target_lock_for_goal_by_default(self):
        config = mission_config_from_env({})
        translation = [0.8, -0.55, 0.1]
        self.assertIsNone(
            evaluate_terminal_reason(
                target_color="red",
                translation=translation,
                target_seen=False,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=100,
                max_steps=900,
                config=config,
            )
        )
        self.assertEqual(
            evaluate_terminal_reason(
                target_color="red",
                translation=translation,
                target_seen=True,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=100,
                max_steps=900,
                config=config,
            ),
            "reached_goal",
        )

    def test_evaluate_terminal_reason_reports_failures_and_timeout(self):
        config = mission_config_from_env({})
        self.assertEqual(
            evaluate_terminal_reason(
                target_color="red",
                translation=[1.1, 0.0, 0.1],
                target_seen=True,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=1,
                max_steps=900,
                config=config,
            ),
            "off_board",
        )
        self.assertEqual(
            evaluate_terminal_reason(
                target_color="red",
                translation=[0.0, 0.0, 0.1],
                target_seen=True,
                lost_steps=18,
                lost_reset_steps=18,
                episode_step=1,
                max_steps=900,
                config=config,
            ),
            "lost_line",
        )
        self.assertEqual(
            evaluate_terminal_reason(
                target_color="red",
                translation=[0.0, 0.0, 0.1],
                target_seen=True,
                lost_steps=0,
                lost_reset_steps=18,
                episode_step=900,
                max_steps=900,
                config=config,
            ),
            "timeout",
        )

    def test_randomized_start_pose_is_seeded_and_bounded(self):
        pose = randomized_start_pose(
            random.Random(5),
            lateral_jitter=0.02,
            longitudinal_jitter=0.03,
            heading_jitter=0.04,
        )
        self.assertLessEqual(abs(pose.lateral_offset), 0.02)
        self.assertLessEqual(abs(pose.translation[0] + 0.42), 0.03)
        self.assertLessEqual(abs(pose.heading_offset), 0.04)
        self.assertEqual(pose, randomized_start_pose(
            random.Random(5),
            lateral_jitter=0.02,
            longitudinal_jitter=0.03,
            heading_jitter=0.04,
        ))


if __name__ == "__main__":
    unittest.main()
