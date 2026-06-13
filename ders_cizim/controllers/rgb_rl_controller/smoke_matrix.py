"""Run Webots smoke episodes across texture variants and target colors."""

from __future__ import annotations

import argparse
import json
import os
import re
import random
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

try:
    from .sim_metrics import summarize_file
except ImportError:  # Allows direct script execution from the controller directory.
    from sim_metrics import summarize_file


TRACK_TEXTURE_PATTERN = re.compile(r'"textures/(?:variants/)?rgb_training_tracks[^"]*\.png"')
MIN_GOAL_MARGIN = 0.015


@dataclass(frozen=True, slots=True)
class SmokeCase:
    name: str
    variant_name: str
    target_color: str
    world_path: Path
    log_path: Path
    summary_path: Path
    steps: int
    camera_pose_name: str = "nominal"
    camera_translation: str | None = None
    camera_rotation: str | None = None
    episode_index: int = 0
    seed: int = 7
    start_lateral_jitter: float = 0.0
    start_longitudinal_jitter: float = 0.0
    start_heading_jitter: float = 0.0
    left_speed_scale: float = 1.0
    right_speed_scale: float = 1.0
    camera_noise: float = 0.0
    mission_mode: str = "single"
    color_sequence: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class CameraPoseOption:
    name: str
    translation: str | None = None
    rotation: str | None = None


def replace_track_texture(world_text: str, texture_url: str) -> str:
    replacement = f'"{texture_url}"'
    replaced, count = TRACK_TEXTURE_PATTERN.subn(replacement, world_text, count=1)
    if count != 1:
        raise ValueError("expected exactly one RGB training track texture URL in world")
    return replaced


def texture_url_for_world(base_world: Path, texture_path: Path) -> str:
    try:
        relative = texture_path.resolve().relative_to(base_world.parent.resolve())
        return relative.as_posix()
    except ValueError:
        return texture_path.resolve().as_posix()


def variant_name_from_texture(texture_path: Path) -> str:
    stem = texture_path.stem
    prefix = "rgb_training_tracks_"
    return stem[len(prefix):] if stem.startswith(prefix) else stem


def write_variant_world(base_world: Path, *, texture_url: str, variant_name: str) -> Path:
    world_text = base_world.read_text(encoding="utf-8")
    output = base_world.with_name(f"_generated_{variant_name}_{base_world.name}")
    output.write_text(replace_track_texture(world_text, texture_url), encoding="utf-8")
    return output


def parse_camera_pose_options(values: Sequence[str] | None) -> list[CameraPoseOption]:
    if not values:
        return [CameraPoseOption(name="nominal")]
    options: list[CameraPoseOption] = []
    for value in values:
        name, _, rest = value.partition(":")
        name = name.strip()
        if not name:
            raise ValueError("camera pose name cannot be empty")
        translation = None
        rotation = None
        if rest:
            translation_part, sep, rotation_part = rest.partition("|")
            if not sep:
                raise ValueError("camera pose must use name:translation|rotation")
            translation = translation_part.strip()
            rotation = rotation_part.strip()
            if not translation or not rotation:
                raise ValueError("camera pose translation and rotation are required")
        options.append(CameraPoseOption(name=name, translation=translation, rotation=rotation))
    return options


def build_smoke_cases(
    *,
    base_world: Path,
    texture_paths: Sequence[Path],
    target_colors: Sequence[str],
    out_dir: Path,
    steps: int,
    camera_poses: Sequence[CameraPoseOption] | None = None,
    episodes: int = 1,
    seed: int = 7,
    start_lateral_jitter: float = 0.0,
    start_longitudinal_jitter: float = 0.0,
    start_heading_jitter: float = 0.0,
    speed_scale_jitter: float = 0.0,
    camera_noise: float = 0.0,
    mission_mode: str = "single",
    color_sequence: Sequence[str] = ("red", "blue"),
) -> list[SmokeCase]:
    cases: list[SmokeCase] = []
    out_dir.mkdir(parents=True, exist_ok=True)
    pose_options = parse_camera_pose_options(None) if camera_poses is None else list(camera_poses)
    case_rng = random.Random(seed)
    normalized_mode = "sequence" if mission_mode.strip().lower() in {"sequence", "multi", "all"} else "single"
    sequence_colors = tuple(color.strip().lower() for color in color_sequence if color.strip())
    if not sequence_colors:
        sequence_colors = ("red", "blue")
    for texture_path in texture_paths:
        variant_name = variant_name_from_texture(texture_path)
        texture_url = texture_url_for_world(base_world, texture_path)
        world_path = write_variant_world(base_world, texture_url=texture_url, variant_name=variant_name)
        for pose in pose_options:
            case_target_colors = (sequence_colors[0],) if normalized_mode == "sequence" else tuple(target_colors)
            for target_color in case_target_colors:
                for episode_index in range(max(1, episodes)):
                    episode_seed = case_rng.randrange(1, 2_000_000_000)
                    speed_jitter = abs(speed_scale_jitter)
                    left_speed_scale = 1.0 + case_rng.uniform(-speed_jitter, speed_jitter)
                    right_speed_scale = 1.0 + case_rng.uniform(-speed_jitter, speed_jitter)
                    target_label = (
                        f"sequence_{'-'.join(sequence_colors)}"
                        if normalized_mode == "sequence"
                        else target_color
                    )
                    if pose.name == "nominal" and len(pose_options) == 1:
                        name = f"{variant_name}_{target_label}_{steps}"
                    else:
                        name = f"{variant_name}_{pose.name}_{target_label}_{steps}"
                    if episodes > 1:
                        name = f"{name}_ep{episode_index:02d}"
                    cases.append(
                        SmokeCase(
                            name=name,
                            variant_name=variant_name,
                            target_color=target_color,
                            world_path=world_path,
                            log_path=out_dir / f"run_{name}.json",
                            summary_path=out_dir / f"run_{name}_summary.json",
                            steps=steps,
                            camera_pose_name=pose.name,
                            camera_translation=pose.translation,
                            camera_rotation=pose.rotation,
                            episode_index=episode_index,
                            seed=episode_seed,
                            start_lateral_jitter=start_lateral_jitter,
                            start_longitudinal_jitter=start_longitudinal_jitter,
                            start_heading_jitter=start_heading_jitter,
                            left_speed_scale=left_speed_scale,
                            right_speed_scale=right_speed_scale,
                            camera_noise=max(0.0, camera_noise),
                            mission_mode=normalized_mode,
                            color_sequence=sequence_colors if normalized_mode == "sequence" else (),
                        )
                    )
    return cases


def env_for_case(case: SmokeCase, *, base_env: dict[str, str] | None = None) -> dict[str, str]:
    env = os.environ.copy() if base_env is None else dict(base_env)
    env.update(
        {
            "MONSTERBORG_RL_START_COLOR": case.target_color,
            "MONSTERBORG_RL_MAX_STEPS": str(case.steps),
            "MONSTERBORG_RL_STEP_LOG_PATH": str(case.log_path.resolve()),
            "MONSTERBORG_RL_SUMMARY_PATH": str(case.summary_path.resolve()),
            "MONSTERBORG_RL_SEED": str(case.seed),
            "MONSTERBORG_RL_START_SEED": str(case.seed),
            "MONSTERBORG_RL_START_LATERAL_JITTER": str(case.start_lateral_jitter),
            "MONSTERBORG_RL_START_LONGITUDINAL_JITTER": str(case.start_longitudinal_jitter),
            "MONSTERBORG_RL_START_HEADING_JITTER": str(case.start_heading_jitter),
            "MONSTERBORG_RL_LEFT_SPEED_SCALE": f"{case.left_speed_scale:.6f}",
            "MONSTERBORG_RL_RIGHT_SPEED_SCALE": f"{case.right_speed_scale:.6f}",
            "MONSTERBORG_RL_QUIT_ON_MISSION_DONE": "1",
            "MONSTERBORG_RL_MISSION_MODE": case.mission_mode,
        }
    )
    if case.color_sequence:
        env["MONSTERBORG_RL_COLOR_SEQUENCE"] = ",".join(case.color_sequence)
    if case.camera_noise > 0:
        env["MONSTERBORG_CAMERA_NOISE"] = str(case.camera_noise)
    if case.camera_translation is not None:
        env["MONSTERBORG_CAMERA_TRANSLATION"] = case.camera_translation
    if case.camera_rotation is not None:
        env["MONSTERBORG_CAMERA_ROTATION"] = case.camera_rotation
    return env


def find_default_webots() -> Path:
    candidates = [
        Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"),
        Path(r"C:\Program Files\Webots\webots.exe"),
        Path("/Applications/Webots.app/Contents/MacOS/webots"),
        Path("webots"),
    ]
    for candidate in candidates:
        if candidate.exists() or candidate.name == "webots":
            return candidate
    raise FileNotFoundError("Webots executable was not found")


def run_case(
    webots: Path,
    case: SmokeCase,
    *,
    extra_env: dict[str, str] | None = None,
    webots_mode: str = "fast",
    minimize: bool = True,
) -> dict[str, object]:
    env = env_for_case(case)
    if extra_env:
        env.update(extra_env)
    command = [str(webots), f"--mode={webots_mode}", "--stdout", "--stderr"]
    if minimize:
        command.append("--minimize")
    command.append(str(case.world_path.resolve()))
    completed = subprocess.run(
        command,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )
    summary = summarize_file(case.log_path)
    if case.mission_mode == "sequence":
        passed_smoke_gate = summary.passed_smoke_gate(
            require_target_lock=True,
            require_returned_start=True,
            require_sequence_complete=True,
            required_sequence_colors=case.color_sequence,
            min_goal_margin=MIN_GOAL_MARGIN,
            min_travel_distance=1.2,
        )
    else:
        passed_smoke_gate = summary.passed_smoke_gate(
            require_goal_reached=True,
            min_goal_margin=MIN_GOAL_MARGIN,
        )
    return {
        "case": asdict(case),
        "returncode": completed.returncode,
        "stdout_tail": completed.stdout[-2000:],
        "stderr_tail": completed.stderr[-2000:],
        "summary": asdict(summary),
        "passed_smoke_gate": passed_smoke_gate,
    }


def run_cases(
    webots: Path,
    cases: Iterable[SmokeCase],
    *,
    webots_mode: str = "fast",
    minimize: bool = True,
) -> list[dict[str, object]]:
    return [run_case(webots, case, webots_mode=webots_mode, minimize=minimize) for case in cases]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", type=Path, default=find_default_webots())
    parser.add_argument("--world", type=Path, default=Path("ders_cizim/worlds/monsterborg_rgb_rl.wbt"))
    parser.add_argument(
        "--textures",
        type=Path,
        nargs="+",
        default=sorted(Path("ders_cizim/worlds/textures/variants").glob("rgb_training_tracks_variant_*.png")),
    )
    parser.add_argument("--colors", nargs="+", default=["red", "blue"])
    parser.add_argument("--mission-mode", choices=["single", "sequence"], default="single")
    parser.add_argument("--sequence", nargs="+", default=["red", "blue"])
    parser.add_argument("--steps", type=int, default=4600)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--start-lateral-jitter", type=float, default=0.0)
    parser.add_argument("--start-longitudinal-jitter", type=float, default=0.0)
    parser.add_argument("--start-heading-jitter", type=float, default=0.0)
    parser.add_argument("--speed-scale-jitter", type=float, default=0.0)
    parser.add_argument("--camera-noise", type=float, default=0.0)
    parser.add_argument("--out-dir", type=Path, default=Path("ders_cizim/artifacts/rgb_rl/variant_matrix"))
    parser.add_argument("--results-json", type=Path, default=None)
    parser.add_argument("--webots-mode", choices=["fast", "realtime", "pause"], default="fast")
    parser.add_argument("--visible", action="store_true", help="Open Webots visibly instead of minimized")
    parser.add_argument(
        "--camera-poses",
        nargs="*",
        default=None,
        help="Optional pose specs like nominal:-0.13 0 0|0 1 0 -1.2",
    )
    args = parser.parse_args()

    cases = build_smoke_cases(
        base_world=args.world,
        texture_paths=args.textures,
        target_colors=args.colors,
        out_dir=args.out_dir,
        steps=args.steps,
        camera_poses=parse_camera_pose_options(args.camera_poses),
        episodes=args.episodes,
        seed=args.seed,
        start_lateral_jitter=args.start_lateral_jitter,
        start_longitudinal_jitter=args.start_longitudinal_jitter,
        start_heading_jitter=args.start_heading_jitter,
        speed_scale_jitter=args.speed_scale_jitter,
        camera_noise=args.camera_noise,
        mission_mode=args.mission_mode,
        color_sequence=args.sequence,
    )
    results = run_cases(args.webots, cases, webots_mode=args.webots_mode, minimize=not args.visible)
    for result in results:
        case = result["case"]
        summary = result["summary"]
        print(
            case["name"],
            f"return={result['returncode']}",
            f"visible={summary['visible_ratio']:.3f}",
            f"matched={summary['matched_target_ratio']:.3f}",
            f"first_lock={summary['first_target_seen_step']}",
            f"goal_margin={summary['final_goal_margin']:.3f}" if summary["final_goal_margin"] is not None else "goal_margin=none",
            f"distance={summary['travel_distance']:.3f}",
            f"off_board={int(summary['off_board'])}",
            f"terminal={summary['terminal_reason'] or 'none'}",
            f"returned_start={int(summary.get('returned_start', False))}",
            f"visited={','.join(summary.get('sequence_visited_colors', [])) or 'none'}",
            f"success={int(summary['success'])}",
            f"pass={int(result['passed_smoke_gate'])}",
        )

    output_path = args.results_json or args.out_dir / "matrix_results.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(results, indent=2, sort_keys=True, default=str), encoding="utf-8")
    if not all(result["passed_smoke_gate"] and result["returncode"] == 0 for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
