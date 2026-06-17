import unittest
import tempfile
from pathlib import Path

from ders_cizim.controllers.rgb_rl_controller import control_core
from ders_cizim.controllers.rgb_rl_controller import rgb_rl_controller
from ders_cizim.controllers.rgb_rl_controller.control_core import DifferentialDriveCommand
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS, DriveRealism
from ders_cizim.controllers.rgb_rl_controller.vision_adapter import RgbArrayCamera, RgbArrayCameraApi


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

    def test_black_target_is_run_only(self):
        self.assertEqual(rgb_rl_controller.normalize_target_color("black", train_mode=False), "black")
        self.assertEqual(rgb_rl_controller.normalize_target_color("black", train_mode=True), "red")

    def test_black_camera_target_matches_thin_black_line(self):
        from PIL import Image, ImageDraw

        image = Image.new("RGB", (96, 96), "white")
        draw = ImageDraw.Draw(image)
        draw.rectangle((43, 0, 48, 70), fill=(20, 20, 20))

        profile = rgb_rl_controller.analyze_rgb_camera(
            RgbArrayCamera(image),
            RgbArrayCameraApi,
            "black",
            previous_error=0.0,
        )

        self.assertTrue(profile.visible)
        self.assertEqual(profile.color_name, "black")
        self.assertTrue(profile.matched_target)
        self.assertGreater(profile.line_width_ratio, 0.0)

    def test_controller_args_can_set_black_target_and_goal_zone(self):
        overrides = rgb_rl_controller.controller_arg_env_overrides(
            [
                "rgb_rl_controller.py",
                "--target=black",
                "--goal-zones=black=0.80:0.36:0.18",
            ]
        )

        self.assertEqual(overrides["MONSTERBORG_RL_START_COLOR"], "black")
        self.assertEqual(overrides["MONSTERBORG_RL_GOAL_ZONES"], "black=0.80:0.36:0.18")

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

    def test_option_policy_fallback_searches_when_target_branch_is_due(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.1,
            confidence=1.0,
            color_name="black",
            target_color="blue",
            matched_target=False,
            line_width_ratio=0.1,
            rgb_balance=(10.0, 10.0, 10.0),
            threshold=24.0,
        )

        option = rgb_rl_controller.fallback_option(profile, should_search_target=True)
        action, speed_scale = rgb_rl_controller.option_to_action(
            option,
            profile,
            {"blue": "left"},
        )

        self.assertEqual(rgb_rl_controller.OPTION_NAMES[option], "search_target")
        self.assertEqual(control_core.ACTION_NAMES[action], "left")
        self.assertLess(speed_scale, 1.0)

    def test_option_follow_line_uses_pd_command_not_discrete_action_command(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.20,
            confidence=1.0,
            color_name="black",
            target_color="red",
            matched_target=False,
            line_width_ratio=0.1,
            rgb_balance=(10.0, 10.0, 10.0),
            threshold=24.0,
        )

        command = rgb_rl_controller.option_to_command(
            rgb_rl_controller.OPTION_FOLLOW_LINE,
            profile,
            previous_error=0.05,
            target_search_actions={},
            limits=DEFAULT_SAFETY_LIMITS,
        )
        discrete_action, _ = rgb_rl_controller.option_to_action(
            rgb_rl_controller.OPTION_FOLLOW_LINE,
            profile,
            {},
        )
        discrete_command = control_core.action_to_command(discrete_action, DEFAULT_SAFETY_LIMITS)

        self.assertLess(command.left, command.right)
        self.assertNotEqual(command, discrete_command)

    def test_option_slow_follow_reuses_pd_command_at_lower_speed(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.0,
            confidence=0.4,
            color_name="black",
            target_color="red",
            matched_target=False,
            line_width_ratio=0.02,
            rgb_balance=(10.0, 10.0, 10.0),
            threshold=24.0,
        )

        command = rgb_rl_controller.option_to_command(
            rgb_rl_controller.OPTION_SLOW_FOLLOW,
            profile,
            previous_error=0.0,
            target_search_actions={},
            limits=DEFAULT_SAFETY_LIMITS,
        )

        self.assertAlmostEqual(command.left, DEFAULT_SAFETY_LIMITS.webots_base_speed * 0.62)
        self.assertAlmostEqual(command.right, DEFAULT_SAFETY_LIMITS.webots_base_speed * 0.62)

    def test_target_search_resumes_after_lock_when_wrong_branch_color_is_visible(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.1,
            confidence=1.0,
            color_name="red",
            target_color="blue",
            matched_target=False,
            line_width_ratio=0.08,
            rgb_balance=(180.0, 30.0, 30.0),
            threshold=24.0,
        )

        self.assertTrue(
            rgb_rl_controller.should_search_for_target_branch(
                profile,
                target_seen=True,
                target_handoff_open=True,
                current_translation=[0.40, -0.60, 0.10],
                min_x=0.30,
            )
        )
        self.assertEqual(
            rgb_rl_controller.allowed_option_indexes(
                profile,
                target_seen=True,
                should_search_target=True,
            ),
            (rgb_rl_controller.OPTION_SEARCH_TARGET,),
        )

    def test_option_state_key_marks_search_stage(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.2,
            confidence=0.8,
            color_name="black",
            target_color="red",
            matched_target=False,
            line_width_ratio=0.08,
            rgb_balance=(20.0, 20.0, 20.0),
            threshold=24.0,
        )

        key = rgb_rl_controller.option_state_key(
            profile,
            previous_error=0.0,
            target_seen=False,
            should_search_target=True,
            lost_steps=0,
        )

        self.assertTrue(key.startswith("stagesearch|"))

    def test_option_policy_skips_direct_q_table_states(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "q_table.json"
            path.write_text(
                '{"training_steps": 5, "episodes": 1, "q_table": {"state": [0, 0, 0, 0, 0, 0, 0]}}',
                encoding="utf-8",
            )

            policy = rgb_rl_controller.QPolicy(path, action_names=rgb_rl_controller.OPTION_NAMES)
            policy.load()

        self.assertEqual(policy.table, {})
        self.assertEqual(policy.skipped_states, 1)

    def test_stable_target_line_only_allows_follow_line_option(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=-0.34,
            confidence=1.0,
            color_name="red",
            target_color="red",
            matched_target=True,
            line_width_ratio=0.03,
            rgb_balance=(220.0, 20.0, 20.0),
            threshold=24.0,
        )

        allowed = rgb_rl_controller.allowed_option_indexes(
            profile,
            target_seen=True,
            should_search_target=False,
        )

        self.assertEqual(allowed, (rgb_rl_controller.OPTION_FOLLOW_LINE,))

    def test_large_line_error_only_allows_slow_follow_option(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=0.64,
            confidence=1.0,
            color_name="red",
            target_color="red",
            matched_target=True,
            line_width_ratio=0.03,
            rgb_balance=(220.0, 20.0, 20.0),
            threshold=24.0,
        )

        allowed = rgb_rl_controller.allowed_option_indexes(
            profile,
            target_seen=True,
            should_search_target=False,
        )

        self.assertEqual(allowed, (rgb_rl_controller.OPTION_SLOW_FOLLOW,))

    def test_thin_target_trace_uses_slow_follow_and_stronger_turn(self):
        profile = rgb_rl_controller.RgbProfile(
            visible=True,
            center_error=-0.34,
            confidence=0.67,
            color_name="red",
            target_color="red",
            matched_target=True,
            line_width_ratio=0.008,
            rgb_balance=(84.0, 30.0, 29.0),
            threshold=173.0,
        )

        fallback = rgb_rl_controller.fallback_option(profile, should_search_target=False)
        action, speed_scale = rgb_rl_controller.option_to_action(fallback, profile, {})

        self.assertEqual(fallback, rgb_rl_controller.OPTION_SLOW_FOLLOW)
        self.assertEqual(control_core.ACTION_NAMES[action], "hard_left")
        self.assertLess(speed_scale, 1.0)

    def test_branch_biased_target_segments_prefers_red_right_branch(self):
        segments = [
            (80.0, 31.0, 300.0, "red", 8, 80, 20, 20),
            (50.0, 82.0, 250.0, "red", 8, 180, 30, 30),
            (40.0, 86.0, 200.0, "red", 7, 190, 35, 35),
        ]

        biased = rgb_rl_controller.branch_biased_target_segments(segments, 96, "red")

        self.assertEqual(biased, segments[1:])

    def test_option_policy_respects_allowed_actions_over_higher_q_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "q_table.json"
            policy = rgb_rl_controller.QPolicy(path, action_names=rgb_rl_controller.OPTION_NAMES)
            key = "stagetarget|e-1|c2|w0|rgb2|target2|match1|lost0|t0"
            policy.table[key] = [1.0, 0.0, 5.0]

            action = policy.choose_action(
                key,
                rgb_rl_controller.OPTION_FOLLOW_LINE,
                0.0,
                allowed_actions=(rgb_rl_controller.OPTION_FOLLOW_LINE,),
            )

        self.assertEqual(action, rgb_rl_controller.OPTION_FOLLOW_LINE)

    def test_option_policy_exploration_respects_allowed_actions(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "q_table.json"
            policy = rgb_rl_controller.QPolicy(path, action_names=rgb_rl_controller.OPTION_NAMES)
            key = "stagecommon|e0|c2|w1|rgb1|target2|match0|lost0|t0"

            choices = {
                policy.choose_action(
                    key,
                    rgb_rl_controller.OPTION_FOLLOW_LINE,
                    1.0,
                    allowed_actions=(rgb_rl_controller.OPTION_FOLLOW_LINE,),
                )
                for _ in range(20)
            }

        self.assertEqual(choices, {rgb_rl_controller.OPTION_FOLLOW_LINE})


if __name__ == "__main__":
    unittest.main()
