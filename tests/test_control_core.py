import unittest

from ders_cizim.controllers.rgb_rl_controller.control_core import (
    ACTION_NAMES,
    DifferentialDriveCommand,
    apply_drive_realism,
    action_to_command,
    clamp,
    heuristic_action,
    parse_target_search_actions,
    target_search_action,
)
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS, DriveRealism


class _Profile:
    def __init__(self, visible=True, center_error=0.0):
        self.visible = visible
        self.center_error = center_error


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

    def test_target_search_action_uses_default_rgb_branch_biases(self):
        self.assertEqual(ACTION_NAMES[target_search_action("red")], "straight")
        self.assertEqual(ACTION_NAMES[target_search_action("green")], "hard_left")
        self.assertEqual(ACTION_NAMES[target_search_action("blue")], "left")

    def test_parse_target_search_actions_allows_environment_override(self):
        mapping = parse_target_search_actions("red=soft_right, green=left, blue=straight")
        self.assertEqual(mapping["red"], "soft_right")
        self.assertEqual(mapping["green"], "left")
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
