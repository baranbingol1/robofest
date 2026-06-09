import unittest

from ders_cizim.controllers.rgb_rl_controller.control_core import DifferentialDriveCommand
from ders_cizim.controllers.rgb_rl_controller.hardware_motor_calibration import (
    build_calibration_sequence,
    run_calibration_sequence,
)
from ders_cizim.controllers.rgb_rl_controller.hardware_pi import NullMotorSink


class MotorCalibrationTests(unittest.TestCase):
    def test_build_calibration_sequence_clamps_power_and_includes_stops(self):
        sequence = build_calibration_sequence(power=0.9)
        self.assertEqual(sequence[0].command, DifferentialDriveCommand(0.0, 0.0))
        self.assertEqual(sequence[-1].command, DifferentialDriveCommand(0.0, 0.0))
        self.assertTrue(all(abs(step.command.left) <= 0.25 for step in sequence))
        self.assertTrue(all(abs(step.command.right) <= 0.25 for step in sequence))

    def test_run_calibration_sequence_records_commands_on_null_sink(self):
        sink = NullMotorSink()
        sequence = build_calibration_sequence(power=0.1, pulse_seconds=0.0)
        run_calibration_sequence(sink, sequence)
        self.assertEqual(sink.commands[0], DifferentialDriveCommand(0.0, 0.0))
        self.assertEqual(sink.commands[-1], DifferentialDriveCommand(0.0, 0.0))
        self.assertIn(DifferentialDriveCommand(0.1, 0.1), sink.commands)


if __name__ == "__main__":
    unittest.main()
