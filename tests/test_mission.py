import random
import unittest

from ders_cizim.controllers.rgb_rl_controller.mission import (
    DEFAULT_COLOR_SEQUENCE,
    DEFAULT_BRANCH_WAYPOINTS,
    DEFAULT_FORK_ZONE,
    DEFAULT_GOAL_ZONES,
    GoalZone,
    SequenceProgress,
    evaluate_terminal_reason,
    mission_config_from_env,
    parse_color_sequence,
    parse_branch_waypoints,
    parse_goal_zones,
    parse_zone,
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

    def test_parse_goal_zones_allows_black_run_only_goal(self):
        zones = parse_goal_zones("black=0.1:0.2:0.3")
        self.assertEqual(zones["black"], GoalZone("black", (0.1, 0.2), 0.3))

    def test_parse_color_sequence_filters_invalid_and_duplicate_colors(self):
        self.assertEqual(parse_color_sequence("green,orange,red,green,blue"), ("red", "blue"))
        self.assertEqual(parse_color_sequence("orange"), tuple(DEFAULT_COLOR_SEQUENCE))

    def test_parse_zone_accepts_configurable_fork_zone(self):
        zone = parse_zone("0.4:-0.72:0.16", DEFAULT_FORK_ZONE)
        self.assertEqual(zone.color, "fork")
        self.assertEqual(zone.center, (0.4, -0.72))
        self.assertAlmostEqual(zone.radius, 0.16)
        self.assertEqual(parse_zone("bad", DEFAULT_FORK_ZONE), DEFAULT_FORK_ZONE)

    def test_parse_branch_waypoints_overrides_known_colors(self):
        waypoints = parse_branch_waypoints("green=0.2:-0.5,orange=1:1,blue=bad")
        self.assertNotIn("green", waypoints)
        self.assertEqual(waypoints["blue"], DEFAULT_BRANCH_WAYPOINTS["blue"])

    def test_mission_config_ignores_invalid_board_extent(self):
        config = mission_config_from_env({"MONSTERBORG_RL_BOARD_HALF_EXTENT": "wide"})
        self.assertAlmostEqual(config.board_half_extent, 1.0)

    def test_mission_config_parses_sequence_options(self):
        config = mission_config_from_env(
            {
                "MONSTERBORG_RL_MISSION_MODE": "all",
                "MONSTERBORG_RL_COLOR_SEQUENCE": "blue,red",
                "MONSTERBORG_RL_FORK_ZONE": "0.41 -0.69 0.11",
                "MONSTERBORG_RL_BRANCH_WAYPOINTS": "blue=0.44 -0.48",
                "MONSTERBORG_RL_START_RETURN_RADIUS": "0.09",
            }
        )
        self.assertEqual(config.mission_mode, "sequence")
        self.assertEqual(config.color_sequence, ("blue", "red"))
        self.assertEqual(config.fork_zone.center, (0.41, -0.69))
        self.assertEqual(config.branch_waypoints["blue"], (0.44, -0.48))
        self.assertAlmostEqual(config.start_return_radius, 0.09)

    def test_sequence_progress_visits_each_color_then_returns_home(self):
        progress = SequenceProgress(("red", "blue"))
        self.assertEqual(progress.active_color, "red")
        self.assertTrue(progress.mark_color_goal_reached())
        self.assertEqual(progress.stage, "return_fork")
        self.assertEqual(progress.visited_colors, ("red",))
        self.assertTrue(progress.mark_returned_to_fork())
        self.assertEqual(progress.active_color, "blue")
        self.assertEqual(progress.stage, "seek_color")
        self.assertTrue(progress.mark_color_goal_reached())
        self.assertTrue(progress.mark_returned_to_fork())
        self.assertEqual(progress.stage, "return_start")
        self.assertIsNone(progress.active_color)
        self.assertTrue(progress.mark_returned_start())
        self.assertTrue(progress.complete)
        self.assertEqual(progress.visited_colors, ("red", "blue"))

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
        zone = config.zones["red"]
        translation = [zone.center[0], zone.center[1], 0.1]
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
