"""Shared MonsterBorg configuration for simulation and Raspberry Pi runs."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True, slots=True)
class CameraPose:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class CameraSensor:
    field_of_view: float
    near: float
    width: int
    height: int


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    webots_base_speed: float
    webots_max_speed: float
    hardware_output_limit: float


@dataclass(frozen=True, slots=True)
class DriveRealism:
    motor_deadband: float
    speed_noise_std: float
    command_latency_steps: int


DEFAULT_CAMERA_POSE = CameraPose(
    translation=(-0.13, 0.0, 0.0),
    rotation=(0.0, 1.0, 0.0, -1.2),
)

DEFAULT_CAMERA_SENSOR = CameraSensor(
    field_of_view=1.25,
    near=0.02,
    width=96,
    height=96,
)

DEFAULT_SAFETY_LIMITS = SafetyLimits(
    webots_base_speed=1.05,
    webots_max_speed=3.0,
    hardware_output_limit=0.65,
)

DEFAULT_DRIVE_REALISM = DriveRealism(
    motor_deadband=0.0,
    speed_noise_std=0.0,
    command_latency_steps=0,
)


def env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        return default


def parse_float_tuple(value: str, expected_length: int) -> tuple[float, ...]:
    normalized = value.replace(",", " ")
    values = tuple(float(part) for part in normalized.split())
    if len(values) != expected_length:
        raise ValueError(f"expected {expected_length} floats, got {len(values)}")
    return values


def camera_pose_from_env(environ: Mapping[str, str] | None = None) -> CameraPose:
    source = os.environ if environ is None else environ
    translation = DEFAULT_CAMERA_POSE.translation
    rotation = DEFAULT_CAMERA_POSE.rotation
    if source.get("MONSTERBORG_CAMERA_TRANSLATION"):
        try:
            translation = parse_float_tuple(source["MONSTERBORG_CAMERA_TRANSLATION"], 3)  # type: ignore[assignment]
        except ValueError:
            translation = DEFAULT_CAMERA_POSE.translation
    if source.get("MONSTERBORG_CAMERA_ROTATION"):
        try:
            rotation = parse_float_tuple(source["MONSTERBORG_CAMERA_ROTATION"], 4)  # type: ignore[assignment]
        except ValueError:
            rotation = DEFAULT_CAMERA_POSE.rotation
    return CameraPose(translation=translation, rotation=rotation)


def safety_limits_from_env() -> SafetyLimits:
    return SafetyLimits(
        webots_base_speed=env_float("MONSTERBORG_RL_BASE_SPEED", DEFAULT_SAFETY_LIMITS.webots_base_speed),
        webots_max_speed=env_float("MONSTERBORG_RL_MAX_SPEED", DEFAULT_SAFETY_LIMITS.webots_max_speed),
        hardware_output_limit=env_float(
            "MONSTERBORG_HARDWARE_OUTPUT_LIMIT",
            DEFAULT_SAFETY_LIMITS.hardware_output_limit,
        ),
    )


def drive_realism_from_env() -> DriveRealism:
    return DriveRealism(
        motor_deadband=max(
            0.0,
            env_float("MONSTERBORG_RL_MOTOR_DEADBAND", DEFAULT_DRIVE_REALISM.motor_deadband),
        ),
        speed_noise_std=max(
            0.0,
            env_float("MONSTERBORG_RL_SPEED_NOISE_STD", DEFAULT_DRIVE_REALISM.speed_noise_std),
        ),
        command_latency_steps=max(
            0,
            env_int("MONSTERBORG_RL_COMMAND_LATENCY_STEPS", DEFAULT_DRIVE_REALISM.command_latency_steps),
        ),
    )
