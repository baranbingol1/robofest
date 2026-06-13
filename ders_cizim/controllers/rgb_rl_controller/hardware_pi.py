"""Lazy Raspberry Pi hardware adapters for the RGB MonsterBorg controller."""

from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field
from typing import Any

try:
    from .control_core import DifferentialDriveCommand
    from .robot_config import DEFAULT_SAFETY_LIMITS, SafetyLimits
except ImportError:  # Webots executes controllers from their own directory.
    from control_core import DifferentialDriveCommand
    from robot_config import DEFAULT_SAFETY_LIMITS, SafetyLimits


def _module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ModuleNotFoundError, ValueError):
        return False


CAMERA_AVAILABLE = _module_available("picamera2")
TB6612_GPIO_AVAILABLE = _module_available("RPi.GPIO")
THUNDERBORG_AVAILABLE = _module_available("ThunderBorg") or _module_available("tborg")
HARDWARE_AVAILABLE = CAMERA_AVAILABLE and TB6612_GPIO_AVAILABLE


@dataclass(frozen=True, slots=True)
class TB6612MotorPins:
    pwm: int
    in1: int
    in2: int


@dataclass(frozen=True, slots=True)
class TB6612Pinout:
    right_stby: int
    left_stby: int
    right_rear: TB6612MotorPins
    right_front: TB6612MotorPins
    left_front: TB6612MotorPins
    left_rear: TB6612MotorPins

    def motor_channels(self) -> tuple[TB6612MotorPins, ...]:
        return (self.right_rear, self.right_front, self.left_front, self.left_rear)

    def output_pins(self) -> tuple[int, ...]:
        pins = [self.right_stby, self.left_stby]
        for channel in self.motor_channels():
            pins.extend((channel.pwm, channel.in1, channel.in2))
        return tuple(dict.fromkeys(pins))


@dataclass(frozen=True, slots=True)
class TB6612MotorSigns:
    right_rear: int = 1
    right_front: int = 1
    left_front: int = 1
    left_rear: int = 1


DEFAULT_TB6612_PINOUT = TB6612Pinout(
    right_stby=21,
    left_stby=27,
    right_rear=TB6612MotorPins(pwm=12, in1=5, in2=6),
    right_front=TB6612MotorPins(pwm=13, in1=16, in2=20),
    left_front=TB6612MotorPins(pwm=18, in1=23, in2=24),
    left_rear=TB6612MotorPins(pwm=19, in1=25, in2=26),
)


DEFAULT_TB6612_MOTOR_SIGNS = TB6612MotorSigns()


def _signed(value: float, sign: int) -> float:
    return value if sign >= 0 else -value


@dataclass(slots=True)
class NullMotorSink:
    commands: list[DifferentialDriveCommand] = field(default_factory=list)

    def set_drive(self, command: DifferentialDriveCommand) -> None:
        self.commands.append(command)

    def stop(self) -> None:
        self.set_drive(DifferentialDriveCommand(0.0, 0.0))


class TB6612GPIOMotorSink:
    """Motor sink for two TB6612 drivers wired directly to Raspberry Pi GPIO pins."""

    def __init__(
        self,
        limits: SafetyLimits = DEFAULT_SAFETY_LIMITS,
        pinout: TB6612Pinout = DEFAULT_TB6612_PINOUT,
        motor_signs: TB6612MotorSigns = DEFAULT_TB6612_MOTOR_SIGNS,
        *,
        gpio: Any | None = None,
        pwm_frequency_hz: int = 1000,
    ) -> None:
        self.limits = limits
        self.pinout = pinout
        self.motor_signs = motor_signs
        self._gpio = self._load_gpio() if gpio is None else gpio
        self._closed = False
        self._pwms: dict[int, Any] = {}

        self._gpio.setwarnings(False)
        self._gpio.setmode(self._gpio.BCM)
        for pin in self.pinout.output_pins():
            self._gpio.setup(pin, self._gpio.OUT, initial=self._gpio.LOW)
        for channel in self.pinout.motor_channels():
            pwm = self._gpio.PWM(channel.pwm, pwm_frequency_hz)
            pwm.start(0.0)
            self._pwms[channel.pwm] = pwm
        self.stop()

    @staticmethod
    def _load_gpio() -> Any:
        try:
            import RPi.GPIO as GPIO  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "RPi.GPIO is required for TB6612 motor control. "
                "Install it on Raspberry Pi OS before running with --armed."
            ) from exc
        return GPIO

    def set_drive(self, command: DifferentialDriveCommand) -> None:
        if self._closed:
            raise RuntimeError("cannot drive a closed TB6612GPIOMotorSink")
        normalized = command.normalized(self.limits.webots_max_speed)
        limited = normalized.clipped(self.limits.hardware_output_limit)
        self._gpio.output(self.pinout.right_stby, self._gpio.HIGH)
        self._gpio.output(self.pinout.left_stby, self._gpio.HIGH)
        self._set_channel(self.pinout.right_rear, _signed(limited.right, self.motor_signs.right_rear))
        self._set_channel(self.pinout.right_front, _signed(limited.right, self.motor_signs.right_front))
        self._set_channel(self.pinout.left_front, _signed(limited.left, self.motor_signs.left_front))
        self._set_channel(self.pinout.left_rear, _signed(limited.left, self.motor_signs.left_rear))

    def stop(self) -> None:
        for channel in self.pinout.motor_channels():
            pwm = self._pwms.get(channel.pwm)
            if pwm is not None:
                pwm.ChangeDutyCycle(0.0)
            self._gpio.output(channel.in1, self._gpio.LOW)
            self._gpio.output(channel.in2, self._gpio.LOW)
        self._gpio.output(self.pinout.right_stby, self._gpio.LOW)
        self._gpio.output(self.pinout.left_stby, self._gpio.LOW)

    def close(self) -> None:
        if self._closed:
            return
        self.stop()
        for pwm in self._pwms.values():
            pwm.stop()
        self._gpio.cleanup(self.pinout.output_pins())
        self._closed = True

    def _set_channel(self, channel: TB6612MotorPins, value: float) -> None:
        duty_cycle = min(100.0, max(0.0, abs(value) * 100.0))
        if duty_cycle <= 0.0:
            self._pwms[channel.pwm].ChangeDutyCycle(0.0)
            self._gpio.output(channel.in1, self._gpio.LOW)
            self._gpio.output(channel.in2, self._gpio.LOW)
            return
        if value > 0.0:
            self._gpio.output(channel.in1, self._gpio.HIGH)
            self._gpio.output(channel.in2, self._gpio.LOW)
        else:
            self._gpio.output(channel.in1, self._gpio.LOW)
            self._gpio.output(channel.in2, self._gpio.HIGH)
        self._pwms[channel.pwm].ChangeDutyCycle(duty_cycle)


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
