"""Camera-only calibration probe for Raspberry Pi and offline frame replay."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Iterable

try:
    from .hardware_pi import PiCameraFrameSource, V4L2FrameSource
    from .rgb_rl_controller import analyze_rgb_camera
    from .vision_adapter import RgbArrayCamera, RgbArrayCameraApi
except ImportError:  # Allows direct execution from the controller directory.
    from hardware_pi import PiCameraFrameSource, V4L2FrameSource
    from rgb_rl_controller import analyze_rgb_camera
    from vision_adapter import RgbArrayCamera, RgbArrayCameraApi

HARDWARE_CAMERA_TARGETS = ("red", "blue", "black")
DEFAULT_MIN_LINE_WIDTH_RATIO = 0.08
DEFAULT_BLACK_MIN_LINE_WIDTH_RATIO = 0.05
DEFAULT_MAX_LINE_WIDTH_RATIO = 0.25


def default_min_line_width_ratio(target_color: str) -> float:
    return DEFAULT_BLACK_MIN_LINE_WIDTH_RATIO if target_color == "black" else DEFAULT_MIN_LINE_WIDTH_RATIO


def profile_to_record(profile) -> dict[str, object]:
    return {
        "visible": profile.visible,
        "center_error": profile.center_error,
        "confidence": profile.confidence,
        "color_name": profile.color_name,
        "target_color": profile.target_color,
        "matched_target": profile.matched_target,
        "line_width_ratio": profile.line_width_ratio,
        "rgb_balance": list(profile.rgb_balance),
        "threshold": profile.threshold,
    }


def summarize_profile_records(
    records: Iterable[dict[str, object]],
    *,
    min_line_width_ratio: float = DEFAULT_MIN_LINE_WIDTH_RATIO,
    max_line_width_ratio: float = DEFAULT_MAX_LINE_WIDTH_RATIO,
) -> dict[str, object]:
    rows = list(records)
    if not rows:
        return {
            "frames": 0,
            "visible_ratio": 0.0,
            "min_line_width_ratio": 0.0,
            "max_line_width_ratio": 0.0,
            "mean_confidence": 0.0,
            "camera_ready": False,
        }
    visible_ratio = sum(1 for row in rows if bool(row.get("visible"))) / len(rows)
    widths = [float(row.get("line_width_ratio", 0.0)) for row in rows]
    confidences = [float(row.get("confidence", 0.0)) for row in rows]
    min_width = min(widths)
    max_width = max(widths)
    mean_confidence = sum(confidences) / len(confidences)
    camera_ready = (
        visible_ratio >= 0.90
        and mean_confidence >= 0.70
        and min_width >= min_line_width_ratio
        and max_width <= max_line_width_ratio
    )
    return {
        "frames": len(rows),
        "visible_ratio": visible_ratio,
        "min_line_width_ratio": min_width,
        "max_line_width_ratio": max_width,
        "mean_confidence": mean_confidence,
        "camera_ready": camera_ready,
    }


def analyze_frame(frame, target_color: str, previous_error: float) -> tuple[dict[str, object], float]:
    profile = analyze_rgb_camera(
        RgbArrayCamera(frame),
        RgbArrayCameraApi,
        target_color,
        previous_error,
    )
    return profile_to_record(profile), profile.center_error


def load_image(path: Path):
    from PIL import Image

    return Image.open(path).convert("RGB")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="red", choices=HARDWARE_CAMERA_TARGETS)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--image", type=Path, default=None, help="Replay one saved RGB image instead of using Pi camera")
    parser.add_argument("--camera-backend", default="picamera2", choices=["picamera2", "v4l2"])
    parser.add_argument("--camera-width", type=int, default=96)
    parser.add_argument("--camera-height", type=int, default=96)
    parser.add_argument("--video-device", default="/dev/video0")
    parser.add_argument("--v4l2-input-width", type=int, default=640)
    parser.add_argument("--v4l2-input-height", type=int, default=480)
    parser.add_argument("--v4l2-framerate", type=int, default=30)
    parser.add_argument("--min-line-width-ratio", type=float, default=None)
    parser.add_argument("--max-line-width-ratio", type=float, default=DEFAULT_MAX_LINE_WIDTH_RATIO)
    parser.add_argument("--output", type=Path, default=Path("hardware_probe.json"))
    args = parser.parse_args()

    records: list[dict[str, object]] = []
    previous_error = 0.0
    source = None
    try:
        if args.image is not None:
            frame = load_image(args.image)
            for _ in range(args.frames):
                record, previous_error = analyze_frame(frame, args.target, previous_error)
                records.append(record)
        elif args.camera_backend == "v4l2":
            source = V4L2FrameSource(
                device=args.video_device,
                width=args.camera_width,
                height=args.camera_height,
                input_width=args.v4l2_input_width,
                input_height=args.v4l2_input_height,
                framerate=args.v4l2_framerate,
            )
            for _ in range(args.frames):
                frame = source.read_rgb_array()
                record, previous_error = analyze_frame(frame, args.target, previous_error)
                records.append(record)
                time.sleep(args.interval)
        else:
            source = PiCameraFrameSource(width=args.camera_width, height=args.camera_height)
            for _ in range(args.frames):
                frame = source.read_rgb_array()
                record, previous_error = analyze_frame(frame, args.target, previous_error)
                records.append(record)
                time.sleep(args.interval)
    finally:
        if source is not None:
            source.close()

    payload = {
        "target": args.target,
        "summary": summarize_profile_records(
            records,
            min_line_width_ratio=(
                default_min_line_width_ratio(args.target)
                if args.min_line_width_ratio is None
                else max(0.0, args.min_line_width_ratio)
            ),
            max_line_width_ratio=max(0.0, args.max_line_width_ratio),
        ),
        "records": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload["summary"], sort_keys=True))


if __name__ == "__main__":
    main()
