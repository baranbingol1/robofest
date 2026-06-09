import unittest

from ders_cizim.controllers.rgb_rl_controller import control_core
from ders_cizim.controllers.rgb_rl_controller import rgb_rl_controller
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS


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


if __name__ == "__main__":
    unittest.main()
