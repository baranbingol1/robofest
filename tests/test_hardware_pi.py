import unittest

from ders_cizim.controllers.rgb_rl_controller import hardware_pi
from ders_cizim.controllers.rgb_rl_controller.control_core import DifferentialDriveCommand


class HardwarePiTests(unittest.TestCase):
    def test_hardware_module_imports_without_pi_dependencies(self):
        self.assertIsInstance(hardware_pi.HARDWARE_AVAILABLE, bool)

    def test_null_motor_sink_records_normalized_commands(self):
        sink = hardware_pi.NullMotorSink()
        sink.set_drive(DifferentialDriveCommand(left=0.25, right=-0.5))
        self.assertEqual(sink.commands[-1], DifferentialDriveCommand(left=0.25, right=-0.5))


if __name__ == "__main__":
    unittest.main()
