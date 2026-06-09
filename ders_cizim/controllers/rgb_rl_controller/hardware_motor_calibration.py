"""Low-power motor-sign calibration for the physical MonsterBorg."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

try:
    from .control_core import DifferentialDriveCommand, clamp
    from .hardware_pi import NullMotorSink, ThunderBorgMotorSink
except ImportError:
    from control_core import DifferentialDriveCommand, clamp
    from hardware_pi import NullMotorSink, ThunderBorgMotorSink


MAX_CALIBRATION_POWER = 0.25


@dataclass(frozen=True, slots=True)
class CalibrationStep:
    label: str
    command: DifferentialDriveCommand
    seconds: float


def build_calibration_sequence(power: float = 0.12, pulse_seconds: float = 0.35) -> list[CalibrationStep]:
    safe_power = clamp(power, 0.0, MAX_CALIBRATION_POWER)
    pause = max(0.0, min(pulse_seconds, 1.0))
    stop = DifferentialDriveCommand(0.0, 0.0)
    return [
        CalibrationStep("stop_before", stop, 0.0),
        CalibrationStep("both_forward", DifferentialDriveCommand(safe_power, safe_power), pause),
        CalibrationStep("stop_after_forward", stop, 0.15),
        CalibrationStep("left_forward_only", DifferentialDriveCommand(safe_power, 0.0), pause),
        CalibrationStep("stop_after_left", stop, 0.15),
        CalibrationStep("right_forward_only", DifferentialDriveCommand(0.0, safe_power), pause),
        CalibrationStep("stop_after_right", stop, 0.15),
        CalibrationStep("final_stop", stop, 0.0),
    ]


def run_calibration_sequence(sink, sequence: list[CalibrationStep]) -> None:
    try:
        for step in sequence:
            print(f"{step.label}: left={step.command.left:.3f} right={step.command.right:.3f} seconds={step.seconds:.2f}")
            sink.set_drive(step.command)
            if step.seconds > 0:
                time.sleep(step.seconds)
    finally:
        sink.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power", type=float, default=0.12)
    parser.add_argument("--pulse-seconds", type=float, default=0.35)
    parser.add_argument("--armed", action="store_true", help="Actually command ThunderBorg motors")
    args = parser.parse_args()

    sequence = build_calibration_sequence(power=args.power, pulse_seconds=args.pulse_seconds)
    sink = ThunderBorgMotorSink() if args.armed else NullMotorSink()
    run_calibration_sequence(sink, sequence)
    if not args.armed:
        print("dry_run=1 commands_recorded=", len(sink.commands))


if __name__ == "__main__":
    main()
