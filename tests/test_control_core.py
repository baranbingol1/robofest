import unittest

from ders_cizim.controllers.rgb_rl_controller.control_core import (
    ACTION_NAMES,
    DifferentialDriveCommand,
    apply_drive_realism,
    action_to_command,
    clamp,
    heuristic_action,
    line_follow_command,
    line_follower_gains_from_env,
    parse_target_search_actions,
    target_search_action,
)
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS, DriveRealism


class _Profile:
    def __init__(self, visible=True, center_error=0.0, confidence=1.0):
        self.visible = visible
        self.center_error = center_error
        self.confidence = confidence


class ControlCoreTests(unittest.TestCase):
    def test_clamp_limits_values(self):
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)
        self.assertEqual(clamp(0.25, -1.0, 1.0), 0.25)

    def test_drive_command_clips_for_webots(self):
        command = DifferentialDriveCommand(left=9.0, right=-9.0)
        clipped = command.clipped(3.0)
        self.assertEqual(clipped.left, 3.0)
        self.assertEqual(clipped.right, -3.0)

    def test_drive_command_normalizes_for_hardware_output(self):
        command = DifferentialDriveCommand(left=3.0, right=-1.5)
        normalized = command.normalized(DEFAULT_SAFETY_LIMITS.webots_max_speed)
        self.assertAlmostEqual(normalized.left, 1.0)
        self.assertAlmostEqual(normalized.right, -0.5)

    def test_action_to_command_preserves_existing_straight_action(self):
        action_index = ACTION_NAMES.index("straight")
        command = action_to_command(action_index, DEFAULT_SAFETY_LIMITS)
        self.assertAlmostEqual(command.left, DEFAULT_SAFETY_LIMITS.webots_base_speed)
        self.assertAlmostEqual(command.right, DEFAULT_SAFETY_LIMITS.webots_base_speed)

    def test_heuristic_turns_toward_line_error(self):
        self.assertEqual(ACTION_NAMES[heuristic_action(_Profile(center_error=-0.7))], "hard_left")
        self.assertEqual(ACTION_NAMES[heuristic_action(_Profile(center_error=0.7))], "hard_right")
        self.assertEqual(ACTION_NAMES[heuristic_action(_Profile(center_error=0.0))], "straight")

    def test_line_follow_command_uses_pd_error_feedback(self):
        increasing_error = line_follow_command(
            _Profile(center_error=0.20),
            previous_error=0.05,
            limits=DEFAULT_SAFETY_LIMITS,
        )
        steady_error = line_follow_command(
            _Profile(center_error=0.20),
            previous_error=0.20,
            limits=DEFAULT_SAFETY_LIMITS,
        )

        self.assertLess(increasing_error.left, increasing_error.right)
        self.assertLess(increasing_error.left, steady_error.left)
        self.assertGreater(increasing_error.right, steady_error.right)

    def test_line_follow_command_slows_when_requested(self):
        command = line_follow_command(
            _Profile(center_error=0.0),
            previous_error=0.0,
            limits=DEFAULT_SAFETY_LIMITS,
            speed_scale=0.62,
        )

        self.assertAlmostEqual(command.left, DEFAULT_SAFETY_LIMITS.webots_base_speed * 0.62)
        self.assertAlmostEqual(command.right, DEFAULT_SAFETY_LIMITS.webots_base_speed * 0.62)

    def test_line_follower_gains_can_be_tuned_from_environment(self):
        gains = line_follower_gains_from_env(
            {
                "MONSTERBORG_RL_LINE_KP": "2.25",
                "MONSTERBORG_RL_LINE_KD": "0.30",
            }
        )

        self.assertAlmostEqual(gains.kp, 2.25)
        self.assertAlmostEqual(gains.kd, 0.30)

    def test_line_follower_gains_reject_negative_environment_values(self):
        gains = line_follower_gains_from_env(
            {
                "MONSTERBORG_RL_LINE_KP": "-1.0",
                "MONSTERBORG_RL_LINE_KD": "-0.5",
            }
        )

        self.assertGreaterEqual(gains.kp, 0.0)
        self.assertGreaterEqual(gains.kd, 0.0)

    def test_target_search_action_uses_default_red_blue_branch_biases(self):
        self.assertEqual(ACTION_NAMES[target_search_action("red")], "straight")
        self.assertEqual(ACTION_NAMES[target_search_action("blue")], "left")

    def test_parse_target_search_actions_allows_environment_override(self):
        mapping = parse_target_search_actions("red=soft_right, green=left, blue=straight")
        self.assertEqual(mapping["red"], "soft_right")
        self.assertNotIn("green", mapping)
        self.assertEqual(mapping["blue"], "straight")

    def test_apply_drive_realism_zeroes_small_deadband_commands(self):
        command = DifferentialDriveCommand(left=0.04, right=-0.11)
        realism = DriveRealism(motor_deadband=0.05, speed_noise_std=0.0, command_latency_steps=0)
        self.assertEqual(
            apply_drive_realism(command, realism),
            DifferentialDriveCommand(left=0.0, right=-0.11),
        )


if __name__ == "__main__":
    unittest.main()
