"""Reusable policy and differential-drive helpers for MonsterBorg controllers."""

from __future__ import annotations

from dataclasses import dataclass
import random
from typing import Protocol

try:
    from .robot_config import DEFAULT_DRIVE_REALISM, DEFAULT_SAFETY_LIMITS, DriveRealism, SafetyLimits
except ImportError:  # Webots executes controllers from their own directory.
    from robot_config import DEFAULT_DRIVE_REALISM, DEFAULT_SAFETY_LIMITS, DriveRealism, SafetyLimits


ACTION_TURNS = (-1.4, -0.95, -0.48, 0.0, 0.48, 0.95, 1.4)
ACTION_NAMES = (
    "hard_left",
    "left",
    "soft_left",
    "straight",
    "soft_right",
    "right",
    "hard_right",
)
TARGET_COLORS = ("red", "blue")
DEFAULT_TARGET_SEARCH_ACTIONS = {
    "blue": "left",
}


class LineProfileLike(Protocol):
    visible: bool
    center_error: float
    confidence: float
    color_name: str
    target_color: str


@dataclass(frozen=True, slots=True)
class DifferentialDriveCommand:
    left: float
    right: float

    def clipped(self, max_abs: float) -> "DifferentialDriveCommand":
        return DifferentialDriveCommand(
            left=clamp(self.left, -max_abs, max_abs),
            right=clamp(self.right, -max_abs, max_abs),
        )

    def normalized(self, max_abs: float) -> "DifferentialDriveCommand":
        if max_abs <= 0:
            return DifferentialDriveCommand(0.0, 0.0)
        clipped = self.clipped(max_abs)
        return DifferentialDriveCommand(clipped.left / max_abs, clipped.right / max_abs)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def action_to_command(
    action_index: int,
    limits: SafetyLimits = DEFAULT_SAFETY_LIMITS,
) -> DifferentialDriveCommand:
    turn = ACTION_TURNS[action_index]
    max_turn = max(abs(value) for value in ACTION_TURNS) or 1.0
    turn_ratio = abs(turn) / max_turn
    base = limits.webots_base_speed * (1.0 - 0.38 * turn_ratio)
    command = DifferentialDriveCommand(left=base - turn, right=base + turn)
    return command.clipped(limits.webots_max_speed)


def action_to_speeds(
    action_index: int,
    limits: SafetyLimits = DEFAULT_SAFETY_LIMITS,
) -> tuple[float, float]:
    command = action_to_command(action_index, limits)
    return command.left, command.right


def apply_drive_realism(
    command: DifferentialDriveCommand,
    realism: DriveRealism = DEFAULT_DRIVE_REALISM,
    rng: random.Random | None = None,
) -> DifferentialDriveCommand:
    left = command.left
    right = command.right
    if realism.speed_noise_std > 0.0:
        source = rng or random
        left *= max(0.0, 1.0 + source.gauss(0.0, realism.speed_noise_std))
        right *= max(0.0, 1.0 + source.gauss(0.0, realism.speed_noise_std))
    if realism.motor_deadband > 0.0:
        left = 0.0 if abs(left) < realism.motor_deadband else left
        right = 0.0 if abs(right) < realism.motor_deadband else right
    return DifferentialDriveCommand(left, right)


def parse_target_search_actions(value: str | None) -> dict[str, str]:
    mapping = dict(DEFAULT_TARGET_SEARCH_ACTIONS)
    if not value:
        return mapping
    for item in value.split(","):
        if not item.strip():
            continue
        if "=" in item:
            color, action_name = item.split("=", 1)
        elif ":" in item:
            color, action_name = item.split(":", 1)
        else:
            continue
        color = color.strip().lower()
        action_name = action_name.strip().lower()
        if color in TARGET_COLORS and action_name in ACTION_NAMES:
            mapping[color] = action_name
    return mapping


def target_search_action(target_color: str, mapping: dict[str, str] | None = None) -> int:
    actions = DEFAULT_TARGET_SEARCH_ACTIONS if mapping is None else mapping
    action_name = actions.get(target_color, "straight")
    if action_name not in ACTION_NAMES:
        action_name = "straight"
    return ACTION_NAMES.index(action_name)


def heuristic_action(profile: LineProfileLike) -> int:
    if not profile.visible:
        if profile.center_error < -0.16:
            return 2
        if profile.center_error > 0.16:
            return 4
        return 3
    error = profile.center_error
    if error < -0.62:
        return 0
    if error < -0.35:
        return 1
    if error < -0.12:
        return 2
    if error > 0.62:
        return 6
    if error > 0.35:
        return 5
    if error > 0.12:
        return 4
    return 3


def compute_reward(profile: LineProfileLike, action_index: int) -> float:
    if not profile.visible:
        return -3.0
    line_reward = 1.4 * (1.0 - abs(profile.center_error))
    confidence_reward = 0.35 * profile.confidence
    turn_penalty = 0.05 * abs(ACTION_TURNS[action_index])
    center_bonus = 0.35 if abs(profile.center_error) < 0.14 else 0.0
    if profile.color_name == profile.target_color:
        color_reward = 0.35
    elif profile.color_name == "black":
        color_reward = 0.08
    elif profile.color_name in TARGET_COLORS:
        color_reward = -0.55
    else:
        color_reward = -0.12
    return line_reward + confidence_reward + center_bonus + color_reward - turn_penalty
