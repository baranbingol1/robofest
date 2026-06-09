"""Safe Raspberry Pi closed-loop runner for the physical MonsterBorg."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Protocol

try:
    from .control_core import (
        ACTION_NAMES,
        TARGET_COLORS,
        action_to_command,
        heuristic_action,
        parse_target_search_actions,
        target_search_action,
    )
    from .hardware_pi import NullMotorSink, PiCameraFrameSource, ThunderBorgMotorSink
    from .hardware_probe import analyze_frame, summarize_profile_records
    from .robot_config import safety_limits_from_env
except ImportError:
    from control_core import (
        ACTION_NAMES,
        TARGET_COLORS,
        action_to_command,
        heuristic_action,
        parse_target_search_actions,
        target_search_action,
    )
    from hardware_pi import NullMotorSink, PiCameraFrameSource, ThunderBorgMotorSink
    from hardware_probe import analyze_frame, summarize_profile_records
    from robot_config import safety_limits_from_env


class FrameSource(Protocol):
    def read_rgb_array(self):
        ...

    def close(self) -> None:
        ...


class MotorSink(Protocol):
    def set_drive(self, command) -> None:
        ...

    def stop(self) -> None:
        ...


Analyzer = Callable[[object, str, float], tuple[dict[str, object], float]]


@dataclass(frozen=True, slots=True)
class HardwareRunConfig:
    target_color: str = "red"
    max_frames: int = 600
    max_seconds: float = 20.0
    interval: float = 0.05
    lost_stop_frames: int = 8
    branch_search_after_frames: int = 0
    target_lock_frames: int = 2
    camera_ready_warmup_frames: int = 20
    require_camera_ready: bool = True
    min_visible_ratio: float = 0.90
    min_mean_confidence: float = 0.70
    min_line_width_ratio: float = 0.08
    max_line_width_ratio: float = 0.25
    stop_file: Path | None = None
    output_path: Path = Path("hardware_run.json")


@dataclass(frozen=True, slots=True)
class HardwareRunSummary:
    frames: int
    stopped_reason: str
    target_color: str
    target_seen: bool
    commands_sent: int
    visible_ratio: float
    mean_confidence: float
    min_line_width_ratio: float
    max_line_width_ratio: float
    camera_ready: bool


class StaticImageFrameSource:
    def __init__(self, frame) -> None:
        self.frame = frame

    def read_rgb_array(self):
        return self.frame

    def close(self) -> None:
        return None


def camera_ready(summary: dict[str, object], config: HardwareRunConfig) -> bool:
    return (
        float(summary.get("visible_ratio", 0.0)) >= config.min_visible_ratio
        and float(summary.get("mean_confidence", 0.0)) >= config.min_mean_confidence
        and float(summary.get("min_line_width_ratio", 0.0)) >= config.min_line_width_ratio
        and float(summary.get("max_line_width_ratio", 0.0)) <= config.max_line_width_ratio
    )


def profile_view(record: dict[str, object], target_color: str):
    return SimpleNamespace(
        visible=bool(record.get("visible")),
        center_error=float(record.get("center_error", 0.0)),
        confidence=float(record.get("confidence", 0.0)),
        color_name=str(record.get("color_name", "none")),
        target_color=target_color,
    )


def choose_action(
    record: dict[str, object],
    *,
    target_color: str,
    frame_index: int,
    target_seen: bool,
    target_search_actions: dict[str, str],
    branch_search_after_frames: int,
) -> int:
    branch_gate_open = branch_search_after_frames > 0 and frame_index >= branch_search_after_frames
    if (
        branch_gate_open
        and not target_seen
        and not bool(record.get("matched_target"))
        and target_color in target_search_actions
    ):
        return target_search_action(target_color, target_search_actions)
    return heuristic_action(profile_view(record, target_color))


def build_summary(
    records: list[dict[str, object]],
    *,
    config: HardwareRunConfig,
    stopped_reason: str,
    target_seen: bool,
    commands_sent: int,
) -> HardwareRunSummary:
    profile_summary = summarize_profile_records(records)
    ready = camera_ready(profile_summary, config)
    return HardwareRunSummary(
        frames=len(records),
        stopped_reason=stopped_reason,
        target_color=config.target_color,
        target_seen=target_seen,
        commands_sent=commands_sent,
        visible_ratio=float(profile_summary.get("visible_ratio", 0.0)),
        mean_confidence=float(profile_summary.get("mean_confidence", 0.0)),
        min_line_width_ratio=float(profile_summary.get("min_line_width_ratio", 0.0)),
        max_line_width_ratio=float(profile_summary.get("max_line_width_ratio", 0.0)),
        camera_ready=ready,
    )


def write_output(config: HardwareRunConfig, summary: HardwareRunSummary, records: list[dict[str, object]]) -> None:
    payload = {
        "summary": asdict(summary),
        "records": records,
    }
    config.output_path.parent.mkdir(parents=True, exist_ok=True)
    config.output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def run_hardware_loop(
    config: HardwareRunConfig,
    frame_source: FrameSource,
    motor_sink: MotorSink,
    *,
    analyzer: Analyzer = analyze_frame,
    sleeper: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    target_search_actions: dict[str, str] | None = None,
) -> HardwareRunSummary:
    if config.target_color not in TARGET_COLORS:
        raise ValueError(f"unsupported target color: {config.target_color}")
    target_actions = parse_target_search_actions(None) if target_search_actions is None else target_search_actions
    records: list[dict[str, object]] = []
    previous_error = 0.0
    lost_frames = 0
    target_lock_candidates = 0
    target_seen = False
    commands_sent = 0
    stopped_reason = "max_frames"
    start_time = clock()
    warmed_up = config.camera_ready_warmup_frames <= 0
    try:
        for frame_index in range(max(0, config.max_frames)):
            if config.stop_file is not None and config.stop_file.exists():
                stopped_reason = "operator_stop_file"
                break
            if config.max_seconds > 0 and clock() - start_time >= config.max_seconds:
                stopped_reason = "max_seconds"
                break

            frame = frame_source.read_rgb_array()
            record, previous_error = analyzer(frame, config.target_color, previous_error)
            record["frame"] = frame_index + 1
            records.append(record)

            if bool(record.get("matched_target")) and float(record.get("confidence", 0.0)) >= 0.24:
                target_lock_candidates += 1
            else:
                target_lock_candidates = 0
            if target_lock_candidates >= config.target_lock_frames:
                target_seen = True

            if not warmed_up and len(records) >= config.camera_ready_warmup_frames:
                warmup_summary = summarize_profile_records(records)
                warmed_up = True
                if config.require_camera_ready and not camera_ready(warmup_summary, config):
                    stopped_reason = "camera_not_ready"
                    break

            if bool(record.get("visible")):
                lost_frames = 0
            else:
                lost_frames += 1
                record["action"] = "line_lost_stop"
                record["command_left"] = 0.0
                record["command_right"] = 0.0
                motor_sink.stop()
                if lost_frames >= config.lost_stop_frames:
                    stopped_reason = "lost_line"
                    break
                if config.interval > 0:
                    sleeper(config.interval)
                continue

            if warmed_up:
                action_index = choose_action(
                    record,
                    target_color=config.target_color,
                    frame_index=frame_index + 1,
                    target_seen=target_seen,
                    target_search_actions=target_actions,
                    branch_search_after_frames=config.branch_search_after_frames,
                )
                command = action_to_command(action_index, safety_limits_from_env())
                motor_sink.set_drive(command)
                commands_sent += 1
                record["action"] = ACTION_NAMES[action_index]
                record["command_left"] = command.left
                record["command_right"] = command.right
            else:
                record["action"] = "warmup_stop"
                record["command_left"] = 0.0
                record["command_right"] = 0.0

            if config.interval > 0:
                sleeper(config.interval)
        else:
            stopped_reason = "max_frames"
    except KeyboardInterrupt:
        stopped_reason = "keyboard_interrupt"
    finally:
        motor_sink.stop()
        frame_source.close()

    summary = build_summary(
        records,
        config=config,
        stopped_reason=stopped_reason,
        target_seen=target_seen,
        commands_sent=commands_sent,
    )
    write_output(config, summary, records)
    return summary


def load_image(path: Path):
    from PIL import Image

    return Image.open(path).convert("RGB")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="red", choices=list(TARGET_COLORS))
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--image", type=Path, default=None, help="Replay one saved image instead of using Pi camera")
    parser.add_argument("--output", type=Path, default=Path("hardware_run.json"))
    parser.add_argument("--stop-file", type=Path, default=None)
    parser.add_argument("--lost-stop-frames", type=int, default=8)
    parser.add_argument("--branch-search-after-frames", type=int, default=0)
    parser.add_argument("--camera-ready-warmup-frames", type=int, default=20)
    parser.add_argument("--allow-uncalibrated-camera", action="store_true")
    parser.add_argument("--hardware-output-limit", type=float, default=None)
    parser.add_argument("--armed", action="store_true", help="Actually command ThunderBorg motors")
    args = parser.parse_args()

    limits = safety_limits_from_env()
    if args.hardware_output_limit is not None:
        limits = replace(limits, hardware_output_limit=max(0.0, min(1.0, args.hardware_output_limit)))

    config = HardwareRunConfig(
        target_color=args.target,
        max_frames=args.frames,
        max_seconds=args.max_seconds,
        interval=args.interval,
        lost_stop_frames=args.lost_stop_frames,
        branch_search_after_frames=args.branch_search_after_frames,
        camera_ready_warmup_frames=args.camera_ready_warmup_frames,
        require_camera_ready=not args.allow_uncalibrated_camera,
        stop_file=args.stop_file,
        output_path=args.output,
    )
    if args.image is not None:
        frame_source = StaticImageFrameSource(load_image(args.image))
    else:
        frame_source = PiCameraFrameSource()
    motor_sink = ThunderBorgMotorSink(limits) if args.armed else NullMotorSink()
    summary = run_hardware_loop(config, frame_source, motor_sink)
    print(json.dumps(asdict(summary), sort_keys=True))


if __name__ == "__main__":
    main()
