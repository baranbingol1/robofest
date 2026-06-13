import unittest
from pathlib import Path
from types import SimpleNamespace

from ders_cizim.controllers.rgb_rl_controller import control_core
from ders_cizim.controllers.rgb_rl_controller import rgb_rl_controller
from ders_cizim.controllers.rgb_rl_controller.control_core import DifferentialDriveCommand
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS, DriveRealism


class FakeMotor:
    def __init__(self):
        self.velocity = None

    def setVelocity(self, value):
        self.velocity = value


class RgbRlControllerContractTests(unittest.TestCase):
    def test_controller_reuses_shared_action_definitions(self):
        self.assertIs(rgb_rl_controller.ACTION_NAMES, control_core.ACTION_NAMES)
        self.assertIs(rgb_rl_controller.ACTION_TURNS, control_core.ACTION_TURNS)

    def test_controller_exposes_shared_safety_limits(self):
        self.assertEqual(rgb_rl_controller.SAFETY_LIMITS, DEFAULT_SAFETY_LIMITS)

    def test_dim_saturated_red_is_classified_as_red_for_variant_textures(self):
        self.assertEqual(rgb_rl_controller.detect_color_name((43.6, 14.5, 15.6)), "red")

    def test_dim_low_saturation_pixels_still_classify_as_black(self):
        self.assertEqual(rgb_rl_controller.detect_color_name((18.0, 19.0, 24.0)), "black")

    def test_speed_scaling_is_applied_before_motor_commands(self):
        old_left = rgb_rl_controller.LEFT_SPEED_SCALE
        old_right = rgb_rl_controller.RIGHT_SPEED_SCALE
        try:
            rgb_rl_controller.LEFT_SPEED_SCALE = 0.8
            rgb_rl_controller.RIGHT_SPEED_SCALE = 1.1
            left = [FakeMotor()]
            right = [FakeMotor()]
            applied = rgb_rl_controller.set_left_right_speed(left, right, 1.0, 1.0)
            self.assertEqual(applied, (0.8, 1.1))
            self.assertEqual(left[0].velocity, 0.8)
            self.assertEqual(right[0].velocity, 1.1)
        finally:
            rgb_rl_controller.LEFT_SPEED_SCALE = old_left
            rgb_rl_controller.RIGHT_SPEED_SCALE = old_right

    def test_delayed_drive_command_holds_then_replays_commands(self):
        queue = []
        realism = DriveRealism(motor_deadband=0.0, speed_noise_std=0.0, command_latency_steps=2)
        first = DifferentialDriveCommand(1.0, 1.0)
        second = DifferentialDriveCommand(2.0, 2.0)
        third = DifferentialDriveCommand(3.0, 3.0)
        self.assertEqual(
            rgb_rl_controller.delayed_drive_command(first, queue, realism),
            DifferentialDriveCommand(0.0, 0.0),
        )
        self.assertEqual(
            rgb_rl_controller.delayed_drive_command(second, queue, realism),
            DifferentialDriveCommand(0.0, 0.0),
        )
        self.assertEqual(
            rgb_rl_controller.delayed_drive_command(third, queue, realism),
            first,
        )

    def test_pose_guided_return_drives_toward_target_pose(self):
        command = rgb_rl_controller.pose_guided_return_command(
            current_translation=[0.0, 0.0, 0.1],
            current_rotation=[0.0, 0.0, 1.0, 0.0],
            target_translation=[-1.0, 0.0, 0.1],
            limits=DEFAULT_SAFETY_LIMITS,
        )
        self.assertGreater(command.left, 0.0)
        self.assertAlmostEqual(command.left, command.right)

        turn_command = rgb_rl_controller.pose_guided_return_command(
            current_translation=[0.0, 0.0, 0.1],
            current_rotation=[0.0, 0.0, 1.0, 0.0],
            target_translation=[0.0, 1.0, 0.1],
            limits=DEFAULT_SAFETY_LIMITS,
        )
        self.assertGreater(turn_command.left, turn_command.right)

    def test_pose_guided_return_can_reverse_when_target_is_behind(self):
        command = rgb_rl_controller.pose_guided_return_command(
            current_translation=[0.0, 0.0, 0.1],
            current_rotation=[0.0, 0.0, 1.0, 0.0],
            previous_translation=[0.01, 0.0, 0.1],
            target_translation=[1.0, 0.0, 0.1],
            limits=DEFAULT_SAFETY_LIMITS,
            allow_reverse=True,
        )
        self.assertLess(command.left, 0.0)
        self.assertLess(command.right, 0.0)

    def test_q_policy_initializes_unseen_states_with_action_values(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=-0.55,
            confidence=0.8,
            color_name="red",
            target_color="red",
            matched_target=True,
            line_width_ratio=0.12,
            rgb_balance=(120.0, 20.0, 20.0),
            threshold=30.0,
        )
        policy = rgb_rl_controller.QPolicy(Path("unused.json"))
        key = rgb_rl_controller.state_key(profile, previous_error=-0.20)

        action = policy.choose_action(key, profile, epsilon=0.0)
        values = policy.values_for(key)

        self.assertEqual(action, values.index(max(values)))
        self.assertNotEqual(values, [0.0 for _ in values])

    def test_q_policy_replaces_legacy_zero_rows_instead_of_using_heuristic_fallback(self):
        profile = SimpleNamespace(
            visible=True,
            center_error=0.48,
            confidence=0.9,
            color_name="red",
            target_color="red",
            matched_target=True,
            line_width_ratio=0.13,
        )
        policy = rgb_rl_controller.QPolicy(Path("unused.json"))
        policy.table["legacy"] = [0.0 for _ in rgb_rl_controller.ACTION_NAMES]

        policy.choose_action("legacy", profile, epsilon=0.0)

        self.assertNotEqual(policy.table["legacy"], [0.0 for _ in rgb_rl_controller.ACTION_NAMES])


if __name__ == "__main__":
    unittest.main()
