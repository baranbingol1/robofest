import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from ders_cizim.controllers.rgb_rl_controller.sim_metrics import RunSummary
from ders_cizim.controllers.rgb_rl_controller.smoke_matrix import (
    SmokeCase,
    build_smoke_cases,
    env_for_case,
    parse_camera_pose_options,
    replace_track_texture,
    run_case,
    write_variant_world,
)


class SmokeMatrixTests(unittest.TestCase):
    def test_replace_track_texture_changes_only_track_url(self):
        world_text = '''
ImageTexture {
  url [
    "textures/rgb_training_tracks.png"
  ]
}
Camera {
  name "camera"
}
'''
        replaced = replace_track_texture(world_text, "textures/variants/rgb_training_tracks_variant_00.png")
        self.assertIn('"textures/variants/rgb_training_tracks_variant_00.png"', replaced)
        self.assertIn('name "camera"', replaced)
        self.assertNotIn('"textures/rgb_training_tracks.png"', replaced)

    def test_write_variant_world_writes_next_to_base_world(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "monsterborg_rgb_rl.wbt"
            base.write_text('"textures/rgb_training_tracks.png"', encoding="utf-8")
            output = write_variant_world(
                base,
                texture_url="textures/variants/rgb_training_tracks_variant_00.png",
                variant_name="variant_00",
            )
            self.assertEqual(output.parent, base.parent)
            self.assertTrue(output.name.startswith("_generated_variant_00"))
            self.assertIn("rgb_training_tracks_variant_00.png", output.read_text(encoding="utf-8"))

    def test_build_smoke_cases_is_deterministic(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base_world = root / "monsterborg_rgb_rl.wbt"
            base_world.write_text('"textures/rgb_training_tracks.png"', encoding="utf-8")
            out_dir = root / "artifacts"
            textures = [
                root / "textures" / "variants" / "rgb_training_tracks_variant_00.png",
                root / "textures" / "variants" / "rgb_training_tracks_variant_01.png",
            ]
            cases = build_smoke_cases(
                base_world=base_world,
                texture_paths=textures,
                target_colors=("red", "green"),
                out_dir=out_dir,
                steps=300,
            )
            self.assertEqual([case.name for case in cases], [
                "variant_00_red_300",
                "variant_00_green_300",
                "variant_01_red_300",
                "variant_01_green_300",
            ])
            self.assertTrue(all(case.log_path.parent == out_dir for case in cases))
            self.assertTrue(all(case.steps == 300 for case in cases))

    def test_build_smoke_cases_can_include_camera_pose_variants(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base_world = root / "monsterborg_rgb_rl.wbt"
            base_world.write_text('"textures/rgb_training_tracks.png"', encoding="utf-8")
            cases = build_smoke_cases(
                base_world=base_world,
                texture_paths=[root / "textures" / "variants" / "rgb_training_tracks_variant_00.png"],
                target_colors=("red",),
                out_dir=root / "artifacts",
                steps=200,
                camera_poses=parse_camera_pose_options([
                    "nominal:-0.13 0 0|0 1 0 -1.2",
                    "tilt_low:-0.13 0 0|0 1 0 -1.12",
                ]),
            )
            self.assertEqual([case.name for case in cases], [
                "variant_00_nominal_red_200",
                "variant_00_tilt_low_red_200",
            ])
            env = env_for_case(cases[1])
            self.assertEqual(env["MONSTERBORG_CAMERA_TRANSLATION"], "-0.13 0 0")
            self.assertEqual(env["MONSTERBORG_CAMERA_ROTATION"], "0 1 0 -1.12")

    def test_env_for_case_uses_absolute_log_paths_for_webots_controller(self):
        case = build_smoke_cases(
            base_world=Path.cwd() / "ders_cizim" / "worlds" / "monsterborg_rgb_rl.wbt",
            texture_paths=[Path.cwd() / "ders_cizim" / "worlds" / "textures" / "variants" / "rgb_training_tracks_variant_00.png"],
            target_colors=("red",),
            out_dir=Path("ders_cizim") / "artifacts" / "rgb_rl" / "matrix_test",
            steps=250,
        )[0]
        env = env_for_case(case)
        self.assertTrue(Path(env["MONSTERBORG_RL_STEP_LOG_PATH"]).is_absolute())
        self.assertTrue(Path(env["MONSTERBORG_RL_SUMMARY_PATH"]).is_absolute())
        self.assertEqual(env["MONSTERBORG_RL_MAX_STEPS"], "250")
        self.assertEqual(env["MONSTERBORG_RL_QUIT_ON_MISSION_DONE"], "1")

    def test_build_smoke_cases_can_expand_seeded_randomized_episodes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base_world = root / "monsterborg_rgb_rl.wbt"
            base_world.write_text('"textures/rgb_training_tracks.png"', encoding="utf-8")
            cases = build_smoke_cases(
                base_world=base_world,
                texture_paths=[root / "textures" / "variants" / "rgb_training_tracks_variant_00.png"],
                target_colors=("blue",),
                out_dir=root / "artifacts",
                steps=500,
                episodes=3,
                seed=123,
                start_lateral_jitter=0.02,
                start_longitudinal_jitter=0.03,
                start_heading_jitter=0.04,
                speed_scale_jitter=0.05,
                camera_noise=0.01,
            )
            self.assertEqual([case.name for case in cases], [
                "variant_00_blue_500_ep00",
                "variant_00_blue_500_ep01",
                "variant_00_blue_500_ep02",
            ])
            self.assertEqual([case.seed for case in cases], [
                build_smoke_cases(
                    base_world=base_world,
                    texture_paths=[root / "textures" / "variants" / "rgb_training_tracks_variant_00.png"],
                    target_colors=("blue",),
                    out_dir=root / "artifacts2",
                    steps=500,
                    episodes=3,
                    seed=123,
                )[index].seed
                for index in range(3)
            ])
            env = env_for_case(cases[0])
            self.assertEqual(env["MONSTERBORG_RL_START_LATERAL_JITTER"], "0.02")
            self.assertEqual(env["MONSTERBORG_RL_START_LONGITUDINAL_JITTER"], "0.03")
            self.assertEqual(env["MONSTERBORG_RL_START_HEADING_JITTER"], "0.04")
            self.assertEqual(env["MONSTERBORG_RL_START_SEED"], str(cases[0].seed))
            self.assertNotEqual(env["MONSTERBORG_RL_LEFT_SPEED_SCALE"], "1.000000")
            self.assertNotEqual(env["MONSTERBORG_RL_RIGHT_SPEED_SCALE"], "1.000000")
            self.assertEqual(env["MONSTERBORG_CAMERA_NOISE"], "0.01")

    def test_run_case_requires_goal_margin_for_smoke_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case = SmokeCase(
                name="variant_00_red_300",
                variant_name="variant_00",
                target_color="red",
                world_path=root / "world.wbt",
                log_path=root / "run.json",
                summary_path=root / "summary.json",
                steps=300,
            )
            summary = RunSummary(
                steps=200,
                visible_ratio=1.0,
                matched_target_ratio=0.2,
                first_target_seen_step=120,
                final_translation=(0.19, 0.0, 0.1),
                final_goal_distance=0.19,
                final_goal_margin=0.01,
                travel_distance=0.7,
                off_board=False,
                terminal_reason="reached_goal",
                reached_goal=True,
                timed_out=False,
                lost_line=False,
                success=True,
            )
            with patch("ders_cizim.controllers.rgb_rl_controller.smoke_matrix.subprocess.run") as run_mock:
                run_mock.return_value = SimpleNamespace(returncode=0, stdout="", stderr="")
                with patch("ders_cizim.controllers.rgb_rl_controller.smoke_matrix.summarize_file", return_value=summary):
                    result = run_case(Path("webots"), case)
            self.assertFalse(result["passed_smoke_gate"])


if __name__ == "__main__":
    unittest.main()
