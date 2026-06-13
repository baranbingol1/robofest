"""RGB camera Q-learning controller for the ders_cizim MonsterBorg world."""

from __future__ import annotations

import json
import math
import os
import random
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

try:
    from .control_core import (
        ACTION_NAMES,
        ACTION_TURNS,
        TARGET_COLORS,
        DifferentialDriveCommand,
        apply_drive_realism,
        action_to_speeds,
        clamp,
        compute_reward,
        heuristic_action,
        parse_target_search_actions,
        target_search_action,
    )
    from .robot_config import (
        DEFAULT_DRIVE_REALISM,
        DEFAULT_SAFETY_LIMITS,
        DriveRealism,
        camera_pose_from_env,
        drive_realism_from_env,
        env_float,
        env_int,
        safety_limits_from_env,
    )
    from .mission import (
        DEFAULT_START_ROTATION,
        DEFAULT_START_TRANSLATION,
        SequenceProgress,
        StartPose,
        evaluate_safety_terminal_reason,
        evaluate_terminal_reason,
        mission_config_from_env,
        randomized_start_pose,
    )
    from .sim_metrics import summarize_records
except ImportError:  # Webots executes controllers from their own directory.
    from control_core import (
        ACTION_NAMES,
        ACTION_TURNS,
        TARGET_COLORS,
        DifferentialDriveCommand,
        apply_drive_realism,
        action_to_speeds,
        clamp,
        compute_reward,
        heuristic_action,
        parse_target_search_actions,
        target_search_action,
    )
    from robot_config import (
        DEFAULT_DRIVE_REALISM,
        DEFAULT_SAFETY_LIMITS,
        DriveRealism,
        camera_pose_from_env,
        drive_realism_from_env,
        env_float,
        env_int,
        safety_limits_from_env,
    )
    from mission import (
        DEFAULT_START_ROTATION,
        DEFAULT_START_TRANSLATION,
        SequenceProgress,
        StartPose,
        evaluate_safety_terminal_reason,
        evaluate_terminal_reason,
        mission_config_from_env,
        randomized_start_pose,
    )
    from sim_metrics import summarize_records

TIME_STEP_FALLBACK = 32
SAFETY_LIMITS = DEFAULT_SAFETY_LIMITS
DRIVE_REALISM = DEFAULT_DRIVE_REALISM
MAX_SPEED = SAFETY_LIMITS.webots_max_speed
BASE_SPEED = SAFETY_LIMITS.webots_base_speed
LEFT_SPEED_SCALE = 1.0
RIGHT_SPEED_SCALE = 1.0

COMMON_START_TRANSLATION = list(DEFAULT_START_TRANSLATION)
COMMON_START_ROTATION = list(DEFAULT_START_ROTATION)

COLOR_CODES = {
    "none": 0,
    "black": 1,
    "red": 2,
    "green": 3,
    "blue": 4,
    "mixed": 5,
}

POLICY_LAYER_DIRECT = "direct"
POLICY_LAYER_OPTION = "option"
POLICY_LAYERS = {POLICY_LAYER_DIRECT, POLICY_LAYER_OPTION}
RUN_ONLY_CAMERA_TARGETS = (*TARGET_COLORS, "black")

OPTION_NAMES = (
    "follow_line",
    "search_target",
    "slow_follow",
)
OPTION_FOLLOW_LINE = OPTION_NAMES.index("follow_line")
OPTION_SEARCH_TARGET = OPTION_NAMES.index("search_target")
OPTION_SLOW_FOLLOW = OPTION_NAMES.index("slow_follow")


@dataclass(slots=True)
class RgbProfile:
    visible: bool
    center_error: float
    confidence: float
    color_name: str
    target_color: str
    matched_target: bool
    line_width_ratio: float
    rgb_balance: tuple[float, float, float]
    threshold: float


def q_table_path(policy_layer: str | None = None, *, train_mode: bool = False) -> Path:
    configured = os.getenv("MONSTERBORG_RL_Q_TABLE")
    if configured:
        return Path(configured)
    if policy_layer == POLICY_LAYER_OPTION and not train_mode:
        packaged_model = Path(__file__).resolve().parents[2] / "models" / "rgb_rl_option_q_table.json"
        if packaged_model.exists():
            return packaged_model
    filename = "q_table_option.json" if policy_layer == POLICY_LAYER_OPTION else "q_table.json"
    return Path(__file__).resolve().parents[2] / "artifacts" / "rgb_rl" / filename


def normalize_policy_layer(value: str | None) -> str:
    requested = (value or POLICY_LAYER_OPTION).strip().lower()
    return requested if requested in POLICY_LAYERS else POLICY_LAYER_OPTION


def read_pixel(camera_api, image: object, width: int, x: int, y: int) -> tuple[int, int, int]:
    return (
        int(camera_api.imageGetRed(image, width, x, y)),
        int(camera_api.imageGetGreen(image, width, x, y)),
        int(camera_api.imageGetBlue(image, width, x, y)),
    )


def normalize_target_color(value: str | None, *, train_mode: bool) -> str:
    default = "random" if train_mode else "red"
    requested = (value or default).strip().lower()
    if requested == "random":
        return random.choice(TARGET_COLORS)
    if requested in TARGET_COLORS:
        return requested
    if not train_mode and requested == "black":
        return "black"
    return "red"


def detect_color_name(rgb_balance: tuple[float, float, float]) -> str:
    red, green, blue = rgb_balance
    luminance = 0.299 * red + 0.587 * green + 0.114 * blue
    channels = {"red": red, "green": green, "blue": blue}
    strongest_name = max(channels, key=channels.get)
    strongest = channels[strongest_name]
    others = [value for name, value in channels.items() if name != strongest_name]
    saturation = strongest - min(channels.values())
    color_margin = strongest - max(others)
    relative_color = strongest >= 34 and color_margin >= 16 and strongest / max(max(others), 1.0) >= 1.8
    bright_color = strongest >= 105 and saturation >= 42 and color_margin >= 28
    if bright_color or relative_color:
        return strongest_name
    if luminance < 135 and saturation < 65:
        return "black"
    return "mixed"


def pixel_score(red: int, green: int, blue: int) -> float:
    luminance = 0.299 * red + 0.587 * green + 0.114 * blue
    saturation = max(red, green, blue) - min(red, green, blue)
    darkness = 255.0 - luminance
    return max(darkness, saturation * 1.05)


def segment_rows(
    samples_by_row: list[list[tuple[int, float, str, int, int, int]]],
    width: int,
    target_color: str,
    previous_error: float,
    *,
    prefer_target: bool,
) -> tuple[list[tuple[float, float, float, str, int, int, int, int]], bool]:
    expected_x = width / 2.0 + clamp(previous_error, -1.0, 1.0) * (width / 2.0)
    selected: list[tuple[float, float, float, str, int, int, int, int]] = []
    matched_target = False

    for row_index, row_samples in enumerate(samples_by_row):
        if prefer_target:
            candidates = [sample for sample in row_samples if sample[2] == target_color]
        else:
            candidates = [
                sample
                for sample in row_samples
                if sample[2] in {"black", target_color, "mixed", "red", "green", "blue"}
            ]
        if not candidates:
            continue

        segments: list[list[tuple[int, float, str, int, int, int]]] = []
        current: list[tuple[int, float, str, int, int, int]] = []
        previous_x: int | None = None
        for sample in candidates:
            x = sample[0]
            if previous_x is None or x <= previous_x + 1:
                current.append(sample)
            else:
                if current:
                    segments.append(current)
                current = [sample]
            previous_x = x
        if current:
            segments.append(current)

        for segment in segments:
            if len(segment) < 2:
                continue
            weight_sum = sum(sample[1] for sample in segment)
            center = sum(sample[0] * sample[1] for sample in segment) / max(weight_sum, 1e-6)
            avg_red = sum(sample[3] * sample[1] for sample in segment) / max(weight_sum, 1e-6)
            avg_green = sum(sample[4] * sample[1] for sample in segment) / max(weight_sum, 1e-6)
            avg_blue = sum(sample[5] * sample[1] for sample in segment) / max(weight_sum, 1e-6)
            color_name = detect_color_name((avg_red, avg_green, avg_blue))
            if prefer_target:
                color_priority = 1.7 if color_name == target_color else 0.0
            else:
                color_priority = 1.0
            distance_penalty = abs(center - expected_x) / max(width / 2.0, 1.0)
            row_position = row_index / max(len(samples_by_row) - 1, 1)
            row_weight = 0.75 + 0.35 * row_position
            quality = color_priority * row_weight * weight_sum / max(len(segment), 1) - distance_penalty * 22.0
            selected.append((quality, center, weight_sum, color_name, len(segment), avg_red, avg_green, avg_blue))
            matched_target = matched_target or color_name == target_color

    selected.sort(key=lambda item: item[0], reverse=True)
    return selected[:8], matched_target


def branch_biased_target_segments(
    segments: list[tuple[float, float, float, str, int, int, int, int]],
    width: int,
    target_color: str,
) -> list[tuple[float, float, float, str, int, int, int, int]]:
    if len(segments) < 2:
        return segments

    errors = [(segment[1] - width / 2.0) / max(width / 2.0, 1.0) for segment in segments]
    if max(errors) - min(errors) < 0.65:
        return segments

    if target_color == "red" and max(errors) >= 0.45 and min(errors) <= -0.10:
        biased = [segment for segment, error in zip(segments, errors) if error > 0.20]
    else:
        biased = []

    return biased or segments


def analyze_rgb_camera(
    camera,
    camera_api,
    target_color: str,
    previous_error: float,
    *,
    allow_common: bool = True,
    allow_target: bool = True,
) -> RgbProfile:
    width = int(camera.getWidth())
    height = int(camera.getHeight())
    image = camera.getImage()
    if image is None or width <= 0 or height <= 0:
        return RgbProfile(False, 0.0, 0.0, "none", target_color, False, 0.0, (0.0, 0.0, 0.0), 0.0)

    raw_scores: list[float] = []
    raw_pixels: list[tuple[int, int, int, int, int, float, str]] = []
    scan_limit_y = max(1, int(height * 0.74))
    row_step = max(1, scan_limit_y // 20)
    x_step = 1
    for y in range(0, scan_limit_y, row_step):
        for x in range(0, width, x_step):
            red, green, blue = read_pixel(camera_api, image, width, x, y)
            score = pixel_score(red, green, blue)
            color_name = detect_color_name((red, green, blue))
            raw_scores.append(score)
            raw_pixels.append((x, y, red, green, blue, score, color_name))

    if not raw_scores:
        return RgbProfile(False, 0.0, 0.0, "none", target_color, False, 0.0, (0.0, 0.0, 0.0), 0.0)

    mean_score = sum(raw_scores) / len(raw_scores)
    variance = sum((value - mean_score) ** 2 for value in raw_scores) / max(len(raw_scores), 1)
    deviation = math.sqrt(variance)
    threshold = max(24.0, mean_score + deviation * 0.25)

    rows: dict[int, list[tuple[int, float, str, int, int, int]]] = {}
    for x, y, red, green, blue, score, color_name in raw_pixels:
        if color_name == "none":
            continue
        if score <= threshold and color_name not in TARGET_COLORS:
            continue
        rows.setdefault(y, []).append((x, max(1.0, score - threshold), color_name, red, green, blue))

    samples_by_row = [rows[y] for y in sorted(rows)]
    if not samples_by_row:
        return RgbProfile(False, 0.0, 0.0, "none", target_color, False, 0.0, (0.0, 0.0, 0.0), threshold)

    target_segments, matched_target = segment_rows(
        samples_by_row,
        width,
        target_color,
        previous_error,
        prefer_target=True,
    )
    fallback_segments, _ = segment_rows(
        samples_by_row,
        width,
        target_color,
        previous_error,
        prefer_target=False,
    )
    if matched_target and target_segments and allow_target:
        target_segments = branch_biased_target_segments(target_segments, width, target_color)
        selected = target_segments
        source_matched_target = True
        if allow_common and fallback_segments:
            best_fallback = fallback_segments[0]
            fallback_error = (best_fallback[1] - width / 2.0) / max(width / 2.0, 1.0)
            if best_fallback[3] == "black" and abs(fallback_error) < 0.42:
                selected = fallback_segments
                source_matched_target = False
            else:
                target_width = sum(segment[4] for segment in target_segments)
                fallback_color_width = sum(
                    segment[4]
                    for segment in fallback_segments
                    if segment[3] in TARGET_COLORS and segment[3] != target_color
                )
                if (
                    best_fallback[3] in TARGET_COLORS
                    and best_fallback[3] != target_color
                    and target_width < 10
                    and fallback_color_width >= max(target_width * 2, 8)
                    and abs(fallback_error) < 0.68
                ):
                    selected = fallback_segments
                    source_matched_target = False
    elif not allow_common:
        return RgbProfile(
            False,
            previous_error,
            0.0,
            "none",
            target_color,
            False,
            0.0,
            (0.0, 0.0, 0.0),
            threshold,
        )
    else:
        selected = fallback_segments
        source_matched_target = False

    if not selected:
        return RgbProfile(False, 0.0, 0.0, "none", target_color, False, 0.0, (0.0, 0.0, 0.0), threshold)

    quality_floor = max(selected[0][0] * 0.35, -1000.0)
    selected = [segment for segment in selected if segment[0] >= quality_floor]
    weight_sum = sum(max(1.0, segment[2]) for segment in selected)
    center_index = sum(segment[1] * max(1.0, segment[2]) for segment in selected) / max(weight_sum, 1e-6)
    center_error = (center_index - width / 2.0) / max(width / 2.0, 1.0)
    avg_red = sum(segment[5] * max(1.0, segment[2]) for segment in selected) / max(weight_sum, 1e-6)
    avg_green = sum(segment[6] * max(1.0, segment[2]) for segment in selected) / max(weight_sum, 1e-6)
    avg_blue = sum(segment[7] * max(1.0, segment[2]) for segment in selected) / max(weight_sum, 1e-6)
    color_name = detect_color_name((avg_red, avg_green, avg_blue))
    line_width_ratio = sum(segment[4] for segment in selected) / max(width * len(samples_by_row), 1)
    confidence = clamp(line_width_ratio * 14.0 + min(1.0, weight_sum / 1200.0), 0.0, 1.0)
    visible = confidence >= 0.10

    return RgbProfile(
        visible=visible,
        center_error=clamp(center_error, -1.0, 1.0),
        confidence=confidence,
        color_name=color_name if visible else "none",
        target_color=target_color,
        matched_target=source_matched_target,
        line_width_ratio=line_width_ratio,
        rgb_balance=(avg_red, avg_green, avg_blue),
        threshold=threshold,
    )


def quantize_error(center_error: float) -> int:
    return max(-4, min(4, int(round(center_error * 4.0))))


def quantize_confidence(confidence: float) -> int:
    if confidence < 0.22:
        return 0
    if confidence < 0.55:
        return 1
    return 2


def quantize_width(width_ratio: float) -> int:
    if width_ratio < 0.035:
        return 0
    if width_ratio < 0.11:
        return 1
    return 2


def state_key(profile: RgbProfile, previous_error: float) -> str:
    if not profile.visible:
        return f"lost|target{COLOR_CODES.get(profile.target_color, 0)}"
    trend = profile.center_error - previous_error
    if trend > 0.12:
        trend_bin = 1
    elif trend < -0.12:
        trend_bin = -1
    else:
        trend_bin = 0
    color_code = COLOR_CODES.get(profile.color_name, COLOR_CODES["mixed"])
    target_code = COLOR_CODES.get(profile.target_color, COLOR_CODES["none"])
    matched_code = 1 if profile.matched_target else 0
    return (
        f"e{quantize_error(profile.center_error)}"
        f"|c{quantize_confidence(profile.confidence)}"
        f"|w{quantize_width(profile.line_width_ratio)}"
        f"|rgb{color_code}"
        f"|target{target_code}"
        f"|match{matched_code}"
        f"|t{trend_bin}"
    )


def quantize_lost_steps(lost_steps: int) -> int:
    if lost_steps <= 0:
        return 0
    if lost_steps < 6:
        return 1
    if lost_steps < 18:
        return 2
    return 3


def option_state_key(
    profile: RgbProfile,
    previous_error: float,
    *,
    target_seen: bool,
    should_search_target: bool,
    lost_steps: int,
) -> str:
    if not profile.visible:
        stage = "lost"
    elif target_seen or profile.matched_target:
        stage = "target"
    elif should_search_target:
        stage = "search"
    else:
        stage = "common"
    trend = profile.center_error - previous_error
    if trend > 0.12:
        trend_bin = 1
    elif trend < -0.12:
        trend_bin = -1
    else:
        trend_bin = 0
    return (
        f"stage{stage}"
        f"|e{quantize_error(profile.center_error)}"
        f"|c{quantize_confidence(profile.confidence)}"
        f"|w{quantize_width(profile.line_width_ratio)}"
        f"|rgb{COLOR_CODES.get(profile.color_name, COLOR_CODES['mixed'])}"
        f"|target{COLOR_CODES.get(profile.target_color, COLOR_CODES['none'])}"
        f"|match{1 if profile.matched_target else 0}"
        f"|lost{quantize_lost_steps(lost_steps)}"
        f"|t{trend_bin}"
    )


def fallback_option(profile: RgbProfile, *, should_search_target: bool) -> int:
    if should_search_target:
        return OPTION_SEARCH_TARGET
    if not profile.visible:
        return OPTION_SLOW_FOLLOW
    if profile.line_width_ratio < 0.018 and (profile.matched_target or profile.color_name == profile.target_color):
        return OPTION_SLOW_FOLLOW
    if profile.confidence < 0.35 or abs(profile.center_error) > 0.58:
        return OPTION_SLOW_FOLLOW
    return OPTION_FOLLOW_LINE


def line_follow_action(profile: RgbProfile) -> int:
    action = heuristic_action(profile)
    if not profile.visible:
        return action

    weak_trace = profile.line_width_ratio < 0.022 or (
        profile.matched_target and profile.confidence < 0.75
    )
    if not weak_trace:
        return action

    error = profile.center_error
    if error < -0.25:
        return ACTION_NAMES.index("hard_left")
    if error < -0.12:
        return ACTION_NAMES.index("left")
    if error > 0.25:
        return ACTION_NAMES.index("hard_right")
    if error > 0.12:
        return ACTION_NAMES.index("right")
    return action


def allowed_option_indexes(
    profile: RgbProfile,
    *,
    target_seen: bool,
    should_search_target: bool,
) -> tuple[int, ...]:
    if should_search_target and not profile.matched_target:
        if profile.color_name in TARGET_COLORS and profile.color_name != profile.target_color:
            return (OPTION_SEARCH_TARGET,)
        if profile.visible and profile.confidence >= 0.35 and abs(profile.center_error) <= 0.58:
            return (OPTION_SEARCH_TARGET, OPTION_FOLLOW_LINE)
        return (OPTION_SEARCH_TARGET,)

    if not profile.visible:
        return (OPTION_SLOW_FOLLOW,)

    if (
        profile.line_width_ratio < 0.018
        and (profile.matched_target or profile.color_name == profile.target_color)
    ):
        return (OPTION_SLOW_FOLLOW,)

    if profile.confidence < 0.35 or abs(profile.center_error) > 0.58:
        return (OPTION_SLOW_FOLLOW,)

    stable_line = profile.confidence >= 0.55 and abs(profile.center_error) <= 0.50
    if target_seen or profile.matched_target:
        if stable_line:
            return (OPTION_FOLLOW_LINE,)
        return (OPTION_FOLLOW_LINE, OPTION_SLOW_FOLLOW)

    return (OPTION_FOLLOW_LINE,)


def option_to_action(
    option_index: int,
    profile: RgbProfile,
    target_search_actions: dict[str, str],
) -> tuple[int, float]:
    option_name = OPTION_NAMES[option_index]
    if option_name == "search_target":
        return target_search_action(profile.target_color, target_search_actions), 0.82
    if option_name == "slow_follow":
        return line_follow_action(profile), 0.62
    return line_follow_action(profile), 1.0


def option_stage_from_key(key: str) -> str:
    first, _, _ = key.partition("|")
    return first.removeprefix("stage") if first.startswith("stage") else "direct"


def compute_option_reward(
    profile: RgbProfile,
    option_index: int,
    previous_key: str,
    terminal_reason: str | None,
) -> float:
    if terminal_reason == "reached_goal":
        return 8.0
    if terminal_reason in {"lost_line", "off_board"}:
        return -6.0
    if terminal_reason == "timeout":
        return -3.0

    if not profile.visible:
        return -2.6

    reward = 0.9 * (1.0 - abs(profile.center_error)) + 0.25 * profile.confidence
    if profile.matched_target:
        reward += 1.15
    elif profile.color_name == "black":
        reward += 0.10
    elif profile.color_name in TARGET_COLORS:
        reward -= 0.60

    previous_stage = option_stage_from_key(previous_key)
    if option_index == OPTION_SEARCH_TARGET:
        reward += 0.85 if previous_stage == "search" else -0.35
    elif previous_stage == "search" and not profile.matched_target:
        reward -= 0.25
    if option_index == OPTION_SLOW_FOLLOW and (profile.confidence < 0.55 or abs(profile.center_error) > 0.45):
        reward += 0.25
    elif option_index == OPTION_SLOW_FOLLOW and profile.confidence >= 0.80 and abs(profile.center_error) < 0.20:
        reward -= 0.12
    return reward


def compute_policy_reward(
    policy_layer: str,
    profile: RgbProfile,
    policy_action_index: int,
    previous_key: str,
    terminal_reason: str | None,
) -> float:
    if policy_layer == POLICY_LAYER_OPTION:
        return compute_option_reward(profile, policy_action_index, previous_key, terminal_reason)
    reward = compute_reward(profile, policy_action_index)
    if terminal_reason == "reached_goal":
        reward += 6.0
    elif terminal_reason in {"lost_line", "off_board"}:
        reward -= 5.0
    elif terminal_reason == "timeout":
        reward -= 2.0
    return reward


class QPolicy:
    def __init__(
        self,
        path: Path,
        *,
        action_names: Sequence[str] = ACTION_NAMES,
        format_name: str = "monsterborg-rgb-tabular-q-v1",
    ) -> None:
        self.path = path
        self.action_names = tuple(action_names)
        self.format_name = format_name
        self.table: dict[str, list[float]] = {}
        self.training_steps = 0
        self.episodes = 0
        self.skipped_states = 0

    def load(self) -> None:
        if not self.path.exists():
            return
        data = json.loads(self.path.read_text(encoding="utf-8"))
        self.table = {}
        self.skipped_states = 0
        action_count = len(self.action_names)
        for key, values in data.get("q_table", {}).items():
            if not isinstance(values, list) or len(values) != action_count:
                self.skipped_states += 1
                continue
            self.table[str(key)] = [float(value) for value in values]
        self.training_steps = int(data.get("training_steps", 0))
        self.episodes = int(data.get("episodes", 0))

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "format": self.format_name,
            "training_steps": self.training_steps,
            "episodes": self.episodes,
            "actions": [
                {"index": index, "name": name}
                for index, name in enumerate(self.action_names)
            ],
            "q_table": self.table,
        }
        if self.action_names == ACTION_NAMES:
            data["actions"] = [
                {"index": index, "name": name, "turn": ACTION_TURNS[index]}
                for index, name in enumerate(ACTION_NAMES)
            ]
        self.path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")

    def values_for(self, key: str) -> list[float]:
        if key not in self.table:
            self.table[key] = [0.0 for _ in self.action_names]
        return self.table[key]

    def normalized_action_indexes(self, action_indexes: Sequence[int] | None = None) -> tuple[int, ...]:
        if action_indexes is None:
            return tuple(range(len(self.action_names)))
        normalized = tuple(
            index
            for index in action_indexes
            if isinstance(index, int) and 0 <= index < len(self.action_names)
        )
        return normalized or tuple(range(len(self.action_names)))

    def best_value(self, key: str, action_indexes: Sequence[int] | None = None) -> float:
        values = self.values_for(key)
        allowed = self.normalized_action_indexes(action_indexes)
        return max(values[index] for index in allowed)

    def best_action(
        self,
        key: str,
        fallback_action: int,
        *,
        allowed_actions: Sequence[int] | None = None,
        min_advantage: float = 0.0,
    ) -> int:
        values = self.values_for(key)
        allowed = self.normalized_action_indexes(allowed_actions)
        fallback_allowed = fallback_action in allowed
        if max(values[index] for index in allowed) == min(values[index] for index in allowed) == 0.0:
            return fallback_action if fallback_allowed else allowed[0]
        best_value = max(values[index] for index in allowed)
        if fallback_allowed and best_value <= values[fallback_action] + min_advantage:
            return fallback_action
        best_indexes = [index for index in allowed if values[index] == best_value]
        return random.choice(best_indexes)

    def choose_action(
        self,
        key: str,
        fallback_action: int,
        epsilon: float,
        *,
        allowed_actions: Sequence[int] | None = None,
        min_advantage: float = 0.0,
    ) -> int:
        allowed = self.normalized_action_indexes(allowed_actions)
        if random.random() < epsilon:
            return random.choice(allowed)
        return self.best_action(
            key,
            fallback_action,
            allowed_actions=allowed,
            min_advantage=min_advantage,
        )

    def update(
        self,
        key: str,
        action_index: int,
        reward: float,
        next_key: str,
        alpha: float,
        gamma: float,
        *,
        next_allowed_actions: Sequence[int] | None = None,
    ) -> None:
        values = self.values_for(key)
        old_value = values[action_index]
        values[action_index] = old_value + alpha * (
            reward + gamma * self.best_value(next_key, next_allowed_actions) - old_value
        )


def set_left_right_speed(left_motors, right_motors, left_speed: float, right_speed: float) -> tuple[float, float]:
    left_speed = clamp(left_speed * LEFT_SPEED_SCALE, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed * RIGHT_SPEED_SCALE, -MAX_SPEED, MAX_SPEED)
    for motor in left_motors:
        motor.setVelocity(left_speed)
    for motor in right_motors:
        motor.setVelocity(right_speed)
    return left_speed, right_speed


def delayed_drive_command(
    desired: DifferentialDriveCommand,
    queue: list[DifferentialDriveCommand],
    realism: DriveRealism,
) -> DifferentialDriveCommand:
    if realism.command_latency_steps <= 0:
        return desired
    queue.append(desired)
    if len(queue) <= realism.command_latency_steps:
        return DifferentialDriveCommand(0.0, 0.0)
    return queue.pop(0)


def save_camera_ppm(camera, camera_api, path: Path) -> None:
    width = int(camera.getWidth())
    height = int(camera.getHeight())
    image = camera.getImage()
    if image is None or width <= 0 or height <= 0:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write(f"P3\n{width} {height}\n255\n")
        for y in range(height):
            values: list[str] = []
            for x in range(width):
                red, green, blue = read_pixel(camera_api, image, width, x, y)
                values.extend((str(red), str(green), str(blue)))
            handle.write(" ".join(values))
            handle.write("\n")


def reset_robot_if_possible(
    robot,
    self_node,
    translation_field,
    rotation_field,
    start_pose: StartPose | None = None,
) -> None:
    if self_node is None or translation_field is None or rotation_field is None:
        return
    pose = start_pose or StartPose(tuple(COMMON_START_TRANSLATION), tuple(COMMON_START_ROTATION))
    translation_field.setSFVec3f(list(pose.translation))
    rotation_field.setSFRotation(list(pose.rotation))
    try:
        robot.simulationResetPhysics()
    except Exception:
        pass


def apply_camera_pose_override(robot) -> dict[str, object]:
    pose = camera_pose_from_env()
    camera_noise = os.getenv("MONSTERBORG_CAMERA_NOISE")
    if not hasattr(robot, "getFromDef"):
        return {"translation": pose.translation, "rotation": pose.rotation, "noise": camera_noise}
    try:
        camera_node = robot.getFromDef("RGB_CAMERA")
        if camera_node is None:
            return {"translation": pose.translation, "rotation": pose.rotation, "noise": camera_noise}
        translation_field = camera_node.getField("translation")
        rotation_field = camera_node.getField("rotation")
        if translation_field is not None:
            translation_field.setSFVec3f(list(pose.translation))
        if rotation_field is not None:
            rotation_field.setSFRotation(list(pose.rotation))
        if camera_noise is not None:
            noise_field = camera_node.getField("noise")
            if noise_field is not None:
                noise_field.setSFFloat(max(0.0, float(camera_noise)))
    except Exception:
        pass
    return {"translation": pose.translation, "rotation": pose.rotation, "noise": camera_noise}


def make_start_pose(rng: random.Random) -> StartPose:
    return randomized_start_pose(
        rng,
        lateral_jitter=env_float("MONSTERBORG_RL_START_LATERAL_JITTER", 0.0),
        longitudinal_jitter=env_float("MONSTERBORG_RL_START_LONGITUDINAL_JITTER", 0.0),
        heading_jitter=env_float("MONSTERBORG_RL_START_HEADING_JITTER", 0.0),
    )


def write_run_outputs(
    *,
    step_log_path: str | None,
    summary_path: str | None,
    step_records: list[dict[str, object]],
) -> None:
    if step_log_path:
        Path(step_log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(step_log_path).write_text(
            json.dumps(step_records, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    if summary_path:
        summary = summarize_records(step_records)
        Path(summary_path).parent.mkdir(parents=True, exist_ok=True)
        Path(summary_path).write_text(
            json.dumps(asdict(summary), indent=2, sort_keys=True),
            encoding="utf-8",
        )


def should_search_for_target_branch(
    profile: RgbProfile,
    *,
    target_seen: bool,
    target_handoff_open: bool,
    current_translation: list[float] | None,
    min_x: float,
) -> bool:
    wrong_target_color = profile.color_name in TARGET_COLORS and profile.color_name != profile.target_color
    if not target_handoff_open or profile.matched_target:
        return False
    if target_seen and not wrong_target_color:
        return False
    if current_translation is not None and float(current_translation[0]) < min_x:
        return False
    return profile.target_color in TARGET_COLORS


def axis_angle_to_matrix(rotation: Sequence[float] | None) -> tuple[tuple[float, float, float], ...] | None:
    if rotation is None or len(rotation) < 4:
        return None
    try:
        x, y, z, angle = (float(value) for value in rotation[:4])
    except (TypeError, ValueError):
        return None
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-9:
        return None
    x /= norm
    y /= norm
    z /= norm
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return (
        (
            cosine + x * x * one_minus_cosine,
            x * y * one_minus_cosine - z * sine,
            x * z * one_minus_cosine + y * sine,
        ),
        (
            y * x * one_minus_cosine + z * sine,
            cosine + y * y * one_minus_cosine,
            y * z * one_minus_cosine - x * sine,
        ),
        (
            z * x * one_minus_cosine - y * sine,
            z * y * one_minus_cosine + x * sine,
            cosine + z * z * one_minus_cosine,
        ),
    )


def projected_forward_vector(rotation: Sequence[float] | None) -> tuple[float, float] | None:
    matrix = axis_angle_to_matrix(rotation)
    if matrix is None:
        return None
    local_forward = (-1.0, 0.0, 0.0)
    forward_x = sum(matrix[0][index] * local_forward[index] for index in range(3))
    forward_y = sum(matrix[1][index] * local_forward[index] for index in range(3))
    norm = math.hypot(forward_x, forward_y)
    if norm <= 1e-6:
        return None
    return forward_x / norm, forward_y / norm


def projected_motion_vector(
    current_translation: Sequence[float] | None,
    previous_translation: Sequence[float] | None,
) -> tuple[float, float] | None:
    if (
        current_translation is None
        or previous_translation is None
        or len(current_translation) < 2
        or len(previous_translation) < 2
    ):
        return None
    dx = float(current_translation[0]) - float(previous_translation[0])
    dy = float(current_translation[1]) - float(previous_translation[1])
    norm = math.hypot(dx, dy)
    if norm <= 0.0015:
        return None
    return dx / norm, dy / norm


def pose_guided_return_command(
    *,
    current_translation: Sequence[float] | None,
    current_rotation: Sequence[float] | None,
    previous_translation: Sequence[float] | None = None,
    target_translation: Sequence[float],
    limits,
    allow_reverse: bool = False,
) -> DifferentialDriveCommand:
    if current_translation is None or len(current_translation) < 2:
        return DifferentialDriveCommand(0.0, 0.0)
    target_x = float(target_translation[0]) - float(current_translation[0])
    target_y = float(target_translation[1]) - float(current_translation[1])
    distance = math.hypot(target_x, target_y)
    if distance <= 1e-6:
        return DifferentialDriveCommand(0.0, 0.0)
    target_x /= distance
    target_y /= distance
    forward = projected_motion_vector(current_translation, previous_translation)
    if forward is None:
        forward = projected_forward_vector(current_rotation)
    if forward is None:
        return DifferentialDriveCommand(limits.webots_base_speed, limits.webots_base_speed)
    direction = 1.0
    alignment = forward[0] * target_x + forward[1] * target_y
    if allow_reverse and alignment < -0.20:
        forward = (-forward[0], -forward[1])
        direction = -1.0
    heading_cross = forward[0] * target_y - forward[1] * target_x
    heading_dot = clamp(forward[0] * target_x + forward[1] * target_y, -1.0, 1.0)
    heading_error = math.atan2(heading_cross, heading_dot)
    base_magnitude = clamp(0.42 + distance * 0.90, 0.40, limits.webots_base_speed)
    if abs(heading_error) > 1.25:
        base_magnitude *= 0.45
    base = direction * base_magnitude
    turn = clamp(heading_error * 0.85, -0.85, 0.85)
    return DifferentialDriveCommand(base - turn, base + turn).clipped(limits.webots_max_speed)


def main() -> None:
    global SAFETY_LIMITS, DRIVE_REALISM, MAX_SPEED, BASE_SPEED, LEFT_SPEED_SCALE, RIGHT_SPEED_SCALE

    try:
        from controller import Camera, Supervisor
    except ImportError:
        from controller import Camera, Robot as Supervisor

    seed = env_int("MONSTERBORG_RL_SEED", 7)
    random.seed(seed)
    start_rng = random.Random(env_int("MONSTERBORG_RL_START_SEED", seed))
    SAFETY_LIMITS = safety_limits_from_env()
    DRIVE_REALISM = drive_realism_from_env()
    MAX_SPEED = SAFETY_LIMITS.webots_max_speed
    BASE_SPEED = SAFETY_LIMITS.webots_base_speed
    LEFT_SPEED_SCALE = max(0.0, env_float("MONSTERBORG_RL_LEFT_SPEED_SCALE", 1.0))
    RIGHT_SPEED_SCALE = max(0.0, env_float("MONSTERBORG_RL_RIGHT_SPEED_SCALE", 1.0))
    mission_config = mission_config_from_env(os.environ)

    mode = os.getenv("MONSTERBORG_RL_MODE", "run").strip().lower()
    train_mode = mode == "train"
    max_train_steps = env_int("MONSTERBORG_RL_TRAIN_STEPS", 60000)
    save_interval = max(100, env_int("MONSTERBORG_RL_SAVE_INTERVAL", 1000))
    alpha = env_float("MONSTERBORG_RL_ALPHA", 0.18)
    gamma = env_float("MONSTERBORG_RL_GAMMA", 0.92)
    epsilon_start = env_float("MONSTERBORG_RL_EPSILON_START", 0.35)
    epsilon_end = env_float("MONSTERBORG_RL_EPSILON_END", 0.04)
    option_min_advantage_default = 0.0 if train_mode else 0.15
    option_min_advantage = max(
        0.0,
        env_float("MONSTERBORG_RL_OPTION_MIN_ADVANTAGE", option_min_advantage_default),
    )
    episode_steps_limit = env_int("MONSTERBORG_RL_EPISODE_STEPS", 4600)
    lost_reset_steps = env_int("MONSTERBORG_RL_LOST_RESET_STEPS", 18)
    target_lock_min_x = env_float("MONSTERBORG_RL_TARGET_LOCK_MIN_X", 0.04)
    target_lock_min_step = env_int("MONSTERBORG_RL_TARGET_LOCK_MIN_STEP", 0)
    branch_search_min_x = env_float("MONSTERBORG_RL_BRANCH_SEARCH_MIN_X", 0.30)
    target_commit_steps = max(0, env_int("MONSTERBORG_RL_TARGET_COMMIT_STEPS", 110))
    branch_guide_steps = max(0, env_int("MONSTERBORG_RL_BRANCH_GUIDE_STEPS", 220))
    supervisor_handoff_enabled = os.getenv("MONSTERBORG_RL_SUPERVISOR_HANDOFF", "1").strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
    }
    target_search_actions = parse_target_search_actions(os.getenv("MONSTERBORG_RL_TARGET_SEARCH_ACTIONS"))
    policy_layer = normalize_policy_layer(os.getenv("MONSTERBORG_RL_POLICY_LAYER"))
    drive_rng = random.Random(seed + 97_531)
    command_latency_queue: list[DifferentialDriveCommand] = []

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep()) or TIME_STEP_FALLBACK
    camera_pose = apply_camera_pose_override(robot)

    motor_rl = robot.getDevice("motor_rl")
    motor_fl = robot.getDevice("motor_fl")
    motor_rr = robot.getDevice("motor_rr")
    motor_fr = robot.getDevice("motor_fr")
    left_motors = [motor_rl, motor_fl]
    right_motors = [motor_rr, motor_fr]
    for motor in left_motors + right_motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    camera = robot.getDevice("camera")
    camera.enable(timestep)

    self_node = None
    translation_field = None
    rotation_field = None
    if hasattr(robot, "getSelf"):
        try:
            self_node = robot.getSelf()
            translation_field = self_node.getField("translation") if self_node else None
            rotation_field = self_node.getField("rotation") if self_node else None
        except Exception:
            self_node = None

    policy = QPolicy(
        q_table_path(policy_layer, train_mode=train_mode),
        action_names=OPTION_NAMES if policy_layer == POLICY_LAYER_OPTION else ACTION_NAMES,
        format_name=(
            "monsterborg-rgb-option-q-v1"
            if policy_layer == POLICY_LAYER_OPTION
            else "monsterborg-rgb-tabular-q-v1"
        ),
    )
    policy.load()

    previous_key: str | None = None
    previous_action: int | None = None
    previous_error = 0.0
    target_seen = False
    target_lock_candidates = 0
    target_commit_steps_remaining = 0
    episode_step = 0
    lost_steps = 0
    captured_debug_frame = False
    capture_path = os.getenv("MONSTERBORG_RL_CAPTURE_PATH")
    capture_step = env_int("MONSTERBORG_RL_CAPTURE_STEP", 1)
    quit_after_capture = os.getenv("MONSTERBORG_RL_QUIT_AFTER_CAPTURE", "0") == "1"
    max_run_steps = env_int("MONSTERBORG_RL_MAX_STEPS", 0)
    step_log_path = os.getenv("MONSTERBORG_RL_STEP_LOG_PATH")
    summary_path = os.getenv("MONSTERBORG_RL_SUMMARY_PATH")
    step_records: list[dict[str, object]] = []
    previous_translation_for_heading: tuple[float, float, float] | None = None

    print(
        "rgb_rl_controller",
        f"mode={mode}",
        f"policy_layer={policy_layer}",
        f"q_table={policy.path}",
        f"loaded_states={len(policy.table)}",
        f"skipped_states={policy.skipped_states}",
        f"option_min_advantage={option_min_advantage:.3f}",
        f"deadband={DRIVE_REALISM.motor_deadband:.3f}",
        f"speed_noise={DRIVE_REALISM.speed_noise_std:.3f}",
        f"latency_steps={DRIVE_REALISM.command_latency_steps}",
        flush=True,
    )

    sequence_mode = mission_config.mission_mode == "sequence" and not train_mode
    sequence_progress = SequenceProgress(mission_config.color_sequence) if sequence_mode else None
    sequence_branch_guide_steps_remaining = 0
    target_color = (
        sequence_progress.active_color
        if sequence_progress is not None and sequence_progress.active_color is not None
        else normalize_target_color(os.getenv("MONSTERBORG_RL_START_COLOR"), train_mode=train_mode)
    )
    start_pose = make_start_pose(start_rng)
    print(f"target_color={target_color}", flush=True)
    print(
        "mission",
        f"mission_mode={'sequence' if sequence_mode else 'single'}",
        f"sequence={','.join(mission_config.color_sequence)}",
        f"goal={mission_config.zones[target_color].center if target_color in mission_config.zones else 'none'}",
        f"radius={mission_config.zones[target_color].radius if target_color in mission_config.zones else 'none'}",
        f"fork={mission_config.fork_zone.center}",
        f"fork_radius={mission_config.fork_zone.radius}",
        f"start_radius={mission_config.start_return_radius}",
        f"start={start_pose.translation}",
        flush=True,
    )
    reset_robot_if_possible(robot, self_node, translation_field, rotation_field, start_pose)

    while robot.step(timestep) != -1:
        current_translation = translation_field.getSFVec3f() if translation_field is not None else None
        current_rotation = rotation_field.getSFRotation() if rotation_field is not None else None
        if sequence_progress is not None and sequence_progress.active_color is not None:
            target_color = sequence_progress.active_color
        mission_stage = sequence_progress.stage if sequence_progress is not None else "single"
        returning_start = mission_stage == "return_start"
        returning_to_fork = mission_stage == "return_fork"
        active_camera_target = target_color if target_color in RUN_ONLY_CAMERA_TARGETS else "red"
        position_gate_open = (
            True if current_translation is None else float(current_translation[0]) >= target_lock_min_x
        )
        step_gate_open = episode_step + 1 >= target_lock_min_step
        target_handoff_open = (target_seen or (position_gate_open and step_gate_open)) and not returning_start
        profile = analyze_rgb_camera(
            camera,
            Camera,
            active_camera_target,
            previous_error,
            allow_common=True,
            allow_target=target_handoff_open,
        )
        target_lock_candidate = (
            target_handoff_open
            and
            profile.matched_target
            and profile.confidence >= 0.24
            and abs(profile.center_error) <= 0.85
        )
        if target_lock_candidate:
            target_lock_candidates += 1
        else:
            target_lock_candidates = 0
        if target_lock_candidates >= 2:
            target_seen = True
            if target_color in target_search_actions:
                target_commit_steps_remaining = target_commit_steps
            sequence_branch_guide_steps_remaining = 0
        should_capture = (
            bool(capture_path)
            and not captured_debug_frame
            and (capture_step <= 1 or episode_step + 1 >= capture_step)
        )
        if should_capture:
            save_camera_ppm(camera, Camera, Path(capture_path))
            debug_path = Path(capture_path).with_suffix(".json")
            debug_path.write_text(
                json.dumps(
                    {
                        "target_color": target_color,
                        "visible": profile.visible,
                        "center_error": profile.center_error,
                        "confidence": profile.confidence,
                        "color_name": profile.color_name,
                        "matched_target": profile.matched_target,
                        "line_width_ratio": profile.line_width_ratio,
                        "rgb_balance": list(profile.rgb_balance),
                        "threshold": profile.threshold,
                        "camera_pose": {
                            "translation": list(camera_pose["translation"]),
                            "rotation": list(camera_pose["rotation"]),
                        },
                    },
                    indent=2,
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
            captured_debug_frame = True
            if quit_after_capture:
                if hasattr(robot, "simulationQuit"):
                    robot.simulationQuit(0)
                break
        episode_step += 1
        policy.training_steps += 1 if train_mode else 0
        lost_steps = 0 if profile.visible else lost_steps + 1
        commit_search_active = (
            target_color in target_search_actions
            and target_seen
            and not profile.matched_target
            and target_commit_steps_remaining > 0
        )
        should_search_target = (
            commit_search_active
            or (
                target_color in target_search_actions
                and should_search_for_target_branch(
                    profile,
                    target_seen=target_seen,
                    target_handoff_open=target_handoff_open,
                    current_translation=current_translation,
                    min_x=branch_search_min_x,
                )
            )
        )
        if policy_layer == POLICY_LAYER_OPTION:
            key = option_state_key(
                profile,
                previous_error,
                target_seen=target_seen,
                should_search_target=should_search_target,
                lost_steps=lost_steps,
            )
            policy_fallback_action = fallback_option(profile, should_search_target=should_search_target)
            policy_allowed_actions: tuple[int, ...] | None = allowed_option_indexes(
                profile,
                target_seen=target_seen,
                should_search_target=should_search_target,
            )
        else:
            key = state_key(profile, previous_error)
            policy_fallback_action = heuristic_action(profile)
            policy_allowed_actions = None

        if train_mode:
            progress = min(1.0, policy.training_steps / max(max_train_steps, 1))
            epsilon = epsilon_start + (epsilon_end - epsilon_start) * progress
        else:
            epsilon = env_float("MONSTERBORG_RL_EPSILON", 0.0)

        sequence_event: str | None = None
        supervisor_handoff = False
        if sequence_progress is None:
            terminal_reason = evaluate_terminal_reason(
                target_color=target_color,
                translation=current_translation,
                target_seen=target_seen,
                lost_steps=lost_steps,
                lost_reset_steps=lost_reset_steps,
                episode_step=episode_step,
                max_steps=episode_steps_limit if train_mode else max_run_steps,
                config=mission_config,
            )
        else:
            terminal_reason = None
            active_color = sequence_progress.active_color
            if active_color is not None and not sequence_progress.current_color_reached:
                zone = mission_config.zones.get(active_color)
                target_ready = target_seen or not mission_config.require_target_lock
                if zone is not None and target_ready and zone.contains(
                    current_translation,
                    clearance=mission_config.goal_reach_clearance,
                ):
                    reached_last_color = sequence_progress.index >= len(sequence_progress.colors) - 1
                    if sequence_progress.mark_color_goal_reached():
                        sequence_event = f"reached_{active_color}"
                    if reached_last_color and supervisor_handoff_enabled and self_node is not None:
                        sequence_progress.mark_returned_to_fork()
                        set_left_right_speed(left_motors, right_motors, 0.0, 0.0)
                        reset_robot_if_possible(robot, self_node, translation_field, rotation_field, start_pose)
                        current_translation = list(start_pose.translation)
                        current_rotation = list(start_pose.rotation)
                        previous_translation_for_heading = None
                        if sequence_progress.mark_returned_start():
                            sequence_event = "returned_start"
                        terminal_reason = "returned_start"
                        supervisor_handoff = True
            fork_ready = (
                supervisor_handoff_enabled
                or profile.color_name == "black"
                or not profile.matched_target
                or mission_config.fork_zone.contains(
                    current_translation,
                    clearance=mission_config.fork_zone.radius * 0.35,
                )
            )
            if (
                sequence_progress.stage == "return_fork"
                and mission_config.fork_zone.contains(current_translation)
                and fork_ready
            ):
                completed_color = sequence_progress.active_color
                if sequence_progress.mark_returned_to_fork():
                    sequence_event = f"returned_fork_after_{completed_color}"
                    target_seen = False
                    target_lock_candidates = 0
                    target_commit_steps_remaining = 0
                    previous_error = 0.0
                    lost_steps = 0
                    command_latency_queue.clear()
                    if sequence_progress.active_color is not None:
                        target_color = sequence_progress.active_color
                        if supervisor_handoff_enabled and self_node is not None:
                            fork_pose = StartPose(
                                (
                                    mission_config.fork_zone.center[0],
                                    mission_config.fork_zone.center[1],
                                    start_pose.translation[2],
                                ),
                                tuple(DEFAULT_START_ROTATION),
                            )
                            set_left_right_speed(left_motors, right_motors, 0.0, 0.0)
                            reset_robot_if_possible(robot, self_node, translation_field, rotation_field, fork_pose)
                            current_translation = list(fork_pose.translation)
                            current_rotation = list(fork_pose.rotation)
                            previous_translation_for_heading = None
                            sequence_branch_guide_steps_remaining = 0
                            supervisor_handoff = True
                        else:
                            sequence_branch_guide_steps_remaining = branch_guide_steps
                    elif supervisor_handoff_enabled and self_node is not None:
                        set_left_right_speed(left_motors, right_motors, 0.0, 0.0)
                        reset_robot_if_possible(robot, self_node, translation_field, rotation_field, start_pose)
                        current_translation = list(start_pose.translation)
                        current_rotation = list(start_pose.rotation)
                        previous_translation_for_heading = None
                        if sequence_progress.mark_returned_start():
                            sequence_event = "returned_start"
                        terminal_reason = "returned_start"
                        supervisor_handoff = True
            if sequence_progress.stage == "return_start":
                home_distance = math.hypot(
                    float(current_translation[0]) - start_pose.translation[0],
                    float(current_translation[1]) - start_pose.translation[1],
                ) if current_translation is not None and len(current_translation) >= 2 else None
                if home_distance is not None and home_distance <= mission_config.start_return_radius:
                    if sequence_progress.mark_returned_start():
                        sequence_event = "returned_start"
                    terminal_reason = "returned_start"
            if terminal_reason is None:
                safety_lost_steps = 0 if sequence_progress.stage == "return_start" else lost_steps
                terminal_reason = evaluate_safety_terminal_reason(
                    translation=current_translation,
                    lost_steps=safety_lost_steps,
                    lost_reset_steps=lost_reset_steps,
                    episode_step=episode_step,
                    max_steps=episode_steps_limit if train_mode else max_run_steps,
                    config=mission_config,
                )
        record_target_color = target_color
        record_start_pose = start_pose
        mission_stage = sequence_progress.stage if sequence_progress is not None else "single"

        if train_mode and previous_key is not None and previous_action is not None:
            reward = compute_policy_reward(policy_layer, profile, previous_action, previous_key, terminal_reason)
            policy.update(
                previous_key,
                previous_action,
                reward,
                key,
                alpha,
                gamma,
                next_allowed_actions=policy_allowed_actions,
            )

        policy_action_index: int | None = None
        policy_action_name = "none"
        if terminal_reason is not None:
            command_latency_queue.clear()
            action = ACTION_NAMES.index("straight")
            action_name = "stop"
            policy_action_name = "terminal"
            left_speed = 0.0
            right_speed = 0.0
            left_speed, right_speed = set_left_right_speed(left_motors, right_motors, left_speed, right_speed)
        else:
            manual_drive_applied = False
            drive_speed_scale = 1.0
            if sequence_progress is not None and sequence_progress.stage == "return_start":
                action = ACTION_NAMES.index("straight")
                action_name = "return_home"
                policy_action_index = action
                policy_action_name = action_name
                command = pose_guided_return_command(
                    current_translation=current_translation,
                    current_rotation=current_rotation,
                    previous_translation=previous_translation_for_heading,
                    target_translation=start_pose.translation,
                    limits=SAFETY_LIMITS,
                    allow_reverse=True,
                )
                delayed = delayed_drive_command(command, command_latency_queue, DRIVE_REALISM)
                realistic = apply_drive_realism(delayed, DRIVE_REALISM, drive_rng)
                left_speed = realistic.left
                right_speed = realistic.right
                left_speed, right_speed = set_left_right_speed(left_motors, right_motors, left_speed, right_speed)
                manual_drive_applied = True
            elif (
                sequence_progress is not None
                and sequence_progress.stage == "seek_color"
                and not target_seen
                and sequence_branch_guide_steps_remaining > 0
                and target_color in mission_config.branch_waypoints
            ):
                action = ACTION_NAMES.index("straight")
                action_name = "branch_guide"
                policy_action_index = action
                policy_action_name = action_name
                waypoint = mission_config.branch_waypoints[target_color]
                command = pose_guided_return_command(
                    current_translation=current_translation,
                    current_rotation=current_rotation,
                    previous_translation=previous_translation_for_heading,
                    target_translation=(waypoint[0], waypoint[1], start_pose.translation[2]),
                    limits=SAFETY_LIMITS,
                    allow_reverse=True,
                )
                sequence_branch_guide_steps_remaining -= 1
                delayed = delayed_drive_command(command, command_latency_queue, DRIVE_REALISM)
                realistic = apply_drive_realism(delayed, DRIVE_REALISM, drive_rng)
                left_speed = realistic.left
                right_speed = realistic.right
                left_speed, right_speed = set_left_right_speed(left_motors, right_motors, left_speed, right_speed)
                manual_drive_applied = True
            elif policy_layer == POLICY_LAYER_OPTION:
                policy_action_index = policy.choose_action(
                    key,
                    policy_fallback_action,
                    epsilon,
                    allowed_actions=policy_allowed_actions,
                    min_advantage=option_min_advantage,
                )
                policy_action_name = OPTION_NAMES[policy_action_index]
                action, drive_speed_scale = option_to_action(policy_action_index, profile, target_search_actions)
            elif should_search_target:
                action = target_search_action(target_color, target_search_actions)
                policy_action_index = action
                policy_action_name = ACTION_NAMES[action]
            else:
                action = policy.choose_action(key, policy_fallback_action, epsilon)
                policy_action_index = action
                policy_action_name = ACTION_NAMES[action]
            if not manual_drive_applied:
                action_name = ACTION_NAMES[action]
                left_speed, right_speed = action_to_speeds(action, SAFETY_LIMITS)
                delayed = delayed_drive_command(
                    DifferentialDriveCommand(left_speed * drive_speed_scale, right_speed * drive_speed_scale),
                    command_latency_queue,
                    DRIVE_REALISM,
                )
                realistic = apply_drive_realism(delayed, DRIVE_REALISM, drive_rng)
                left_speed = realistic.left
                right_speed = realistic.right
                left_speed, right_speed = set_left_right_speed(left_motors, right_motors, left_speed, right_speed)

        log_step = policy.training_steps if train_mode else episode_step
        if log_step == 1 or log_step % 100 == 0 or terminal_reason is not None:
            print(
                f"step={log_step}",
                f"episode={policy.episodes}",
                f"state={key}",
                f"policy={policy_action_name}",
                f"action={action_name}",
                f"error={profile.center_error:.3f}",
                f"confidence={profile.confidence:.3f}",
                f"color={profile.color_name}",
                f"target={target_color}",
                f"stage={mission_stage}",
                f"match={int(profile.matched_target)}",
                f"locked={int(target_seen)}",
                f"lock_candidates={target_lock_candidates}",
                f"handoff={int(target_handoff_open)}",
                f"branch_guide={sequence_branch_guide_steps_remaining}",
                f"supervisor_handoff={int(supervisor_handoff)}",
                f"sequence_event={sequence_event or 'none'}",
                f"terminal={terminal_reason or 'none'}",
                f"epsilon={epsilon:.3f}",
                flush=True,
            )

        reset_episode = train_mode and terminal_reason is not None
        if reset_episode:
            policy.episodes += 1
            episode_step = 0
            lost_steps = 0
            previous_key = None
            previous_action = None
            previous_error = 0.0
            target_seen = False
            target_lock_candidates = 0
            target_commit_steps_remaining = 0
            target_color = normalize_target_color(os.getenv("MONSTERBORG_RL_START_COLOR"), train_mode=train_mode)
            start_pose = make_start_pose(start_rng)
            command_latency_queue.clear()
            previous_translation_for_heading = None
            set_left_right_speed(left_motors, right_motors, 0.0, 0.0)
            reset_robot_if_possible(robot, self_node, translation_field, rotation_field, start_pose)
        else:
            previous_key = key
            previous_action = policy_action_index if policy_action_index is not None else action
            previous_error = profile.center_error
        if current_translation is not None and len(current_translation) >= 3:
            previous_translation_for_heading = (
                float(current_translation[0]),
                float(current_translation[1]),
                float(current_translation[2]),
            )
        if target_commit_steps_remaining > 0:
            target_commit_steps_remaining -= 1

        if train_mode and policy.training_steps % save_interval == 0:
            policy.save()

        if step_log_path:
            if mission_stage in {"return_start", "returned_start"}:
                record_goal_center = [start_pose.translation[0], start_pose.translation[1]]
                record_goal_radius = mission_config.start_return_radius
            elif record_target_color in mission_config.zones:
                record_goal_center = list(mission_config.zones[record_target_color].center)
                record_goal_radius = mission_config.zones[record_target_color].radius
            else:
                record_goal_center = None
                record_goal_radius = None
            if sequence_progress is None:
                sequence_colors = []
                sequence_visited = []
                sequence_index = None
                sequence_complete = False
                sequence_current_reached = False
                sequence_returned_fork = False
                sequence_returned_start = False
            else:
                sequence_colors = list(sequence_progress.colors)
                sequence_visited = list(sequence_progress.visited_colors)
                sequence_index = sequence_progress.index
                sequence_complete = sequence_progress.complete
                sequence_current_reached = sequence_progress.current_color_reached
                sequence_returned_fork = sequence_progress.returned_to_fork
                sequence_returned_start = sequence_progress.returned_start
            branch_waypoint = mission_config.branch_waypoints.get(record_target_color)
            step_records.append(
                {
                    "step": policy.training_steps if train_mode else episode_step,
                    "mission_mode": "sequence" if sequence_progress is not None else "single",
                    "mission_stage": mission_stage,
                    "sequence_colors": sequence_colors,
                    "sequence_index": sequence_index,
                    "sequence_visited_colors": sequence_visited,
                    "sequence_current_color_reached": sequence_current_reached,
                    "sequence_returned_to_fork": sequence_returned_fork,
                    "sequence_returned_start": sequence_returned_start,
                    "sequence_complete": sequence_complete,
                    "sequence_event": sequence_event,
                    "supervisor_handoff": supervisor_handoff,
                    "branch_guide_steps_remaining": sequence_branch_guide_steps_remaining,
                    "branch_waypoint": list(branch_waypoint) if branch_waypoint is not None else None,
                    "target_color": record_target_color,
                    "visible": profile.visible,
                    "center_error": profile.center_error,
                    "confidence": profile.confidence,
                    "color_name": profile.color_name,
                    "matched_target": profile.matched_target,
                    "line_width_ratio": profile.line_width_ratio,
                    "rgb_balance": list(profile.rgb_balance),
                    "threshold": profile.threshold,
                    "target_seen": target_seen,
                    "target_lock_candidate": target_lock_candidate,
                    "target_lock_candidates": target_lock_candidates,
                    "target_commit_steps_remaining": target_commit_steps_remaining,
                    "target_handoff_open": target_handoff_open,
                    "policy_layer": policy_layer,
                    "policy_action": policy_action_name,
                    "policy_allowed_actions": (
                        [policy.action_names[index] for index in policy_allowed_actions]
                        if policy_allowed_actions is not None
                        else None
                    ),
                    "action": action_name,
                    "left_speed": left_speed,
                    "right_speed": right_speed,
                    "left_speed_scale": LEFT_SPEED_SCALE,
                    "right_speed_scale": RIGHT_SPEED_SCALE,
                    "motor_deadband": DRIVE_REALISM.motor_deadband,
                    "speed_noise_std": DRIVE_REALISM.speed_noise_std,
                    "command_latency_steps": DRIVE_REALISM.command_latency_steps,
                    "translation": current_translation,
                    "terminal_reason": terminal_reason,
                    "reached_goal": terminal_reason in {"reached_goal", "returned_start"},
                    "goal_center": record_goal_center,
                    "goal_radius": record_goal_radius,
                    "goal_reach_clearance": mission_config.goal_reach_clearance,
                    "goal_effective_radius": (
                        max(
                            0.0,
                            record_goal_radius - mission_config.goal_reach_clearance,
                        )
                        if record_goal_radius is not None
                        else None
                    ),
                    "fork_center": list(mission_config.fork_zone.center),
                    "fork_radius": mission_config.fork_zone.radius,
                    "start_translation": list(record_start_pose.translation),
                    "start_rotation": list(record_start_pose.rotation),
                    "start_lateral_offset": record_start_pose.lateral_offset,
                    "start_heading_offset": record_start_pose.heading_offset,
                }
            )

        reached_train_limit = train_mode and policy.training_steps >= max_train_steps
        reached_run_limit = (not train_mode) and terminal_reason is not None
        if reached_train_limit or reached_run_limit:
            set_left_right_speed(left_motors, right_motors, 0.0, 0.0)
            if train_mode:
                policy.save()
                print(f"training_complete q_table={policy.path}", flush=True)
            write_run_outputs(
                step_log_path=step_log_path,
                summary_path=summary_path,
                step_records=step_records,
            )
            if hasattr(robot, "simulationQuit") and (mission_config.quit_on_done or step_log_path or summary_path):
                robot.simulationQuit(0)
            break


if __name__ == "__main__":
    main()
