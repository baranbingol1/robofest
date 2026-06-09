"""Mission-level episode logic for the RGB MonsterBorg track."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Mapping, Sequence

try:
    from .control_core import TARGET_COLORS
except ImportError:  # Webots executes controllers from their own directory.
    from control_core import TARGET_COLORS


DEFAULT_START_TRANSLATION = (-0.42, -0.70, 0.0993865)
DEFAULT_START_ROTATION = (
    0.007308191135707109,
    0.9999732948135465,
    -1.43672787700961e-06,
    3.1412758139848846,
)


@dataclass(frozen=True, slots=True)
class GoalZone:
    color: str
    center: tuple[float, float]
    radius: float

    def distance_to(self, translation: Sequence[float] | None) -> float | None:
        if translation is None or len(translation) < 2:
            return None
        return math.hypot(float(translation[0]) - self.center[0], float(translation[1]) - self.center[1])

    def contains(self, translation: Sequence[float] | None, *, clearance: float = 0.0) -> bool:
        distance = self.distance_to(translation)
        if distance is None:
            return False
        effective_radius = max(0.0, self.radius - max(0.0, clearance))
        return distance <= effective_radius


@dataclass(frozen=True, slots=True)
class MissionConfig:
    zones: Mapping[str, GoalZone]
    board_half_extent: float
    goal_reach_clearance: float
    require_target_lock: bool
    stop_on_goal: bool
    quit_on_done: bool


@dataclass(frozen=True, slots=True)
class StartPose:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    lateral_offset: float = 0.0
    heading_offset: float = 0.0


DEFAULT_GOAL_ZONES = {
    "red": GoalZone("red", (0.80, -0.55), 0.18),
    "green": GoalZone("green", (0.10, -0.50), 0.20),
    "blue": GoalZone("blue", (0.32, -0.28), 0.20),
}
DEFAULT_GOAL_REACH_CLEARANCE = 0.02


def parse_bool(value: str | None, *, default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def parse_float(value: str | None, default: float) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_nonnegative_float(value: str | None, default: float) -> float:
    parsed = parse_float(value, default)
    if parsed < 0.0:
        return default
    return parsed


def parse_goal_zones(value: str | None) -> dict[str, GoalZone]:
    zones = dict(DEFAULT_GOAL_ZONES)
    if not value:
        return zones
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        color, sep, spec = item.partition("=")
        if not sep:
            continue
        color = color.strip().lower()
        if color not in TARGET_COLORS:
            continue
        parts = spec.replace(":", " ").split()
        if len(parts) != 3:
            continue
        try:
            x, y, radius = (float(part) for part in parts)
        except ValueError:
            continue
        if radius <= 0:
            continue
        zones[color] = GoalZone(color, (x, y), radius)
    return zones


def mission_config_from_env(environ: Mapping[str, str]) -> MissionConfig:
    return MissionConfig(
        zones=parse_goal_zones(environ.get("MONSTERBORG_RL_GOAL_ZONES")),
        board_half_extent=parse_float(environ.get("MONSTERBORG_RL_BOARD_HALF_EXTENT"), 1.0),
        goal_reach_clearance=parse_nonnegative_float(
            environ.get("MONSTERBORG_RL_GOAL_REACH_CLEARANCE"),
            DEFAULT_GOAL_REACH_CLEARANCE,
        ),
        require_target_lock=parse_bool(environ.get("MONSTERBORG_RL_GOAL_REQUIRES_TARGET_LOCK"), default=True),
        stop_on_goal=parse_bool(environ.get("MONSTERBORG_RL_STOP_ON_GOAL"), default=True),
        quit_on_done=parse_bool(environ.get("MONSTERBORG_RL_QUIT_ON_MISSION_DONE"), default=False),
    )


def evaluate_terminal_reason(
    *,
    target_color: str,
    translation: Sequence[float] | None,
    target_seen: bool,
    lost_steps: int,
    lost_reset_steps: int,
    episode_step: int,
    max_steps: int,
    config: MissionConfig,
) -> str | None:
    if translation is not None and len(translation) >= 2:
        if abs(float(translation[0])) > config.board_half_extent or abs(float(translation[1])) > config.board_half_extent:
            return "off_board"
    if lost_reset_steps > 0 and lost_steps >= lost_reset_steps:
        return "lost_line"
    zone = config.zones.get(target_color)
    if zone is not None and zone.contains(translation, clearance=config.goal_reach_clearance):
        if not config.require_target_lock or target_seen:
            return "reached_goal"
    if max_steps > 0 and episode_step >= max_steps:
        return "timeout"
    return None


def randomized_start_pose(
    rng: random.Random,
    *,
    lateral_jitter: float = 0.0,
    longitudinal_jitter: float = 0.0,
    heading_jitter: float = 0.0,
    base_translation: Sequence[float] = DEFAULT_START_TRANSLATION,
    base_rotation: Sequence[float] = DEFAULT_START_ROTATION,
) -> StartPose:
    lateral_offset = rng.uniform(-abs(lateral_jitter), abs(lateral_jitter)) if lateral_jitter else 0.0
    longitudinal_offset = rng.uniform(-abs(longitudinal_jitter), abs(longitudinal_jitter)) if longitudinal_jitter else 0.0
    heading_offset = rng.uniform(-abs(heading_jitter), abs(heading_jitter)) if heading_jitter else 0.0
    translation = (
        float(base_translation[0]) + longitudinal_offset,
        float(base_translation[1]) + lateral_offset,
        float(base_translation[2]),
    )
    rotation = (
        float(base_rotation[0]),
        float(base_rotation[1]),
        float(base_rotation[2]),
        float(base_rotation[3]) + heading_offset,
    )
    return StartPose(
        translation=translation,
        rotation=rotation,
        lateral_offset=lateral_offset,
        heading_offset=heading_offset,
    )
