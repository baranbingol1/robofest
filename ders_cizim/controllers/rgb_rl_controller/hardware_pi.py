"""Lazy Raspberry Pi hardware adapters for the RGB MonsterBorg controller."""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field

try:
    from .control_core import DifferentialDriveCommand
    from .robot_config import DEFAULT_SAFETY_LIMITS, SafetyLimits
except ImportError:  # Webots executes controllers from their own directory.
    from control_core import DifferentialDriveCommand
    from robot_config import DEFAULT_SAFETY_LIMITS, SafetyLimits


HARDWARE_AVAILABLE = (
    importlib.util.find_spec("picamera2") is not None
    and (
        importlib.util.find_spec("ThunderBorg") is not None
        or importlib.util.find_spec("tborg") is not None
    )
)


@dataclass(slots=True)
class NullMotorSink:
    commands: list[DifferentialDriveCommand] = field(default_factory=list)

    def set_drive(self, command: DifferentialDriveCommand) -> None:
        self.commands.append(command)

    def stop(self) -> None:
        self.set_drive(DifferentialDriveCommand(0.0, 0.0))


class ThunderBorgMotorSink:
    """Motor sink for original PiBorg and python-thunderborg APIs."""

    def __init__(self, limits: SafetyLimits = DEFAULT_SAFETY_LIMITS) -> None:
        self.limits = limits
        self._mode = "original"
        try:
            import ThunderBorg  # type: ignore

            board = ThunderBorg.ThunderBorg()
            board.Init()
            self._board = board
        except ImportError:
            from tborg import ThunderBorg  # type: ignore

            self._mode = "python-thunderborg"
            self._board = ThunderBorg()

    def set_drive(self, command: DifferentialDriveCommand) -> None:
        normalized = command.normalized(self.limits.webots_max_speed)
        limited = normalized.clipped(self.limits.hardware_output_limit)
        if self._mode == "original":
            self._board.SetMotor1(limited.left)
            self._board.SetMotor2(limited.right)
        else:
            self._board.set_motor_one(limited.left)
            self._board.set_motor_two(limited.right)

    def stop(self) -> None:
        if self._mode == "original":
            self._board.MotorsOff()
        else:
            self._board.halt_motors()


class PiCameraFrameSource:
    def __init__(self, width: int = 96, height: int = 96) -> None:
        from picamera2 import Picamera2  # type: ignore

        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"format": "RGB888", "size": (width, height)}
        )
        self.camera.configure(config)
        self.camera.start()

    def read_rgb_array(self):
        return self.camera.capture_array()

    def close(self) -> None:
        self.camera.stop()
