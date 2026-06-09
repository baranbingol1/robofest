import re
import unittest
from pathlib import Path

from ders_cizim.controllers.rgb_rl_controller.robot_config import (
    DEFAULT_CAMERA_SENSOR,
    DEFAULT_CAMERA_POSE,
    DEFAULT_SAFETY_LIMITS,
    CameraPose,
    camera_pose_from_env,
    env_float,
    parse_float_tuple,
)


ROOT = Path(__file__).resolve().parents[1]
RGB_WORLD = ROOT / "ders_cizim" / "worlds" / "monsterborg_rgb_rl.wbt"


def _parse_first_camera_block(world_text: str) -> dict[str, list[float]]:
    match = re.search(r"Camera\s*\{(?P<body>.*?)\n\s*\}", world_text, re.DOTALL)
    if match is None:
        raise AssertionError("No Camera node found in RGB Webots world")
    body = match.group("body")
    translation_match = re.search(r"translation\s+([^\n]+)", body)
    rotation_match = re.search(r"rotation\s+([^\n]+)", body)
    fov_match = re.search(r"fieldOfView\s+([^\n]+)", body)
    width_match = re.search(r"width\s+([^\n]+)", body)
    height_match = re.search(r"height\s+([^\n]+)", body)
    near_match = re.search(r"near\s+([^\n]+)", body)
    if None in {translation_match, rotation_match, fov_match, width_match, height_match, near_match}:
        raise AssertionError("Camera node is missing a required pose or sensor field")
    return {
        "translation": [float(value) for value in translation_match.group(1).split()],
        "rotation": [float(value) for value in rotation_match.group(1).split()],
        "fieldOfView": [float(fov_match.group(1))],
        "width": [float(width_match.group(1))],
        "height": [float(height_match.group(1))],
        "near": [float(near_match.group(1))],
    }


class RobotConfigTests(unittest.TestCase):
    def assertSequenceAlmostEqual(self, actual, expected, places=6):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=places)

    def test_default_camera_pose_is_front_mounted_and_down_forward(self):
        pose = DEFAULT_CAMERA_POSE
        self.assertIsInstance(pose, CameraPose)
        self.assertLessEqual(pose.translation[0], -0.12)
        self.assertEqual(pose.translation[1], 0.0)
        self.assertGreaterEqual(pose.translation[2], -0.005)
        self.assertLessEqual(pose.translation[2], 0.01)
        self.assertSequenceAlmostEqual(pose.rotation, (0.0, 1.0, 0.0, -1.2), places=4)

    def test_rgb_world_camera_matches_default_contract(self):
        world_text = RGB_WORLD.read_text(encoding="utf-8")
        self.assertIn("DEF RGB_CAMERA Camera", world_text)
        camera = _parse_first_camera_block(world_text)
        self.assertSequenceAlmostEqual(camera["translation"], DEFAULT_CAMERA_POSE.translation)
        self.assertSequenceAlmostEqual(camera["rotation"], DEFAULT_CAMERA_POSE.rotation, places=4)
        self.assertAlmostEqual(camera["fieldOfView"][0], DEFAULT_CAMERA_SENSOR.field_of_view, places=6)
        self.assertEqual(int(camera["width"][0]), DEFAULT_CAMERA_SENSOR.width)
        self.assertEqual(int(camera["height"][0]), DEFAULT_CAMERA_SENSOR.height)
        self.assertAlmostEqual(camera["near"][0], DEFAULT_CAMERA_SENSOR.near, places=6)

    def test_safety_limits_keep_webots_and_hardware_scales_separate(self):
        self.assertGreater(DEFAULT_SAFETY_LIMITS.webots_max_speed, DEFAULT_SAFETY_LIMITS.webots_base_speed)
        self.assertLessEqual(DEFAULT_SAFETY_LIMITS.hardware_output_limit, 1.0)
        self.assertGreater(DEFAULT_SAFETY_LIMITS.hardware_output_limit, 0.0)

    def test_env_float_uses_default_for_invalid_values(self):
        self.assertEqual(env_float("MONSTERBORG_TEST_MISSING", 1.25), 1.25)

    def test_parse_float_tuple_accepts_commas_or_spaces(self):
        self.assertEqual(parse_float_tuple("1, 2,3", 3), (1.0, 2.0, 3.0))
        self.assertEqual(parse_float_tuple("1 2 3", 3), (1.0, 2.0, 3.0))

    def test_camera_pose_from_env_uses_overrides(self):
        env = {
            "MONSTERBORG_CAMERA_TRANSLATION": "-0.1 0 0.02",
            "MONSTERBORG_CAMERA_ROTATION": "0 1 0 -1.2",
        }
        pose = camera_pose_from_env(env)
        self.assertEqual(pose.translation, (-0.1, 0.0, 0.02))
        self.assertEqual(pose.rotation, (0.0, 1.0, 0.0, -1.2))


if __name__ == "__main__":
    unittest.main()
