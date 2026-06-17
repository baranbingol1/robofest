import io
import unittest

from ders_cizim.controllers.rgb_rl_controller import hardware_pi
from ders_cizim.controllers.rgb_rl_controller.control_core import DifferentialDriveCommand
from ders_cizim.controllers.rgb_rl_controller.robot_config import SafetyLimits


class FakePwm:
    def __init__(self, pin, frequency, gpio):
        self.pin = pin
        self.frequency = frequency
        self.gpio = gpio
        self.started = []
        self.duty_cycles = []
        self.stopped = False

    def start(self, duty_cycle):
        self.started.append(duty_cycle)
        self.duty_cycles.append(duty_cycle)

    def ChangeDutyCycle(self, duty_cycle):
        self.duty_cycles.append(duty_cycle)

    def stop(self):
        self.stopped = True


class FakeGPIO:
    BCM = "BCM"
    OUT = "OUT"
    LOW = 0
    HIGH = 1

    def __init__(self):
        self.mode = None
        self.warnings = None
        self.setup_calls = []
        self.output_states = {}
        self.pwms = {}
        self.cleaned = None

    def setwarnings(self, value):
        self.warnings = value

    def setmode(self, mode):
        self.mode = mode

    def setup(self, pin, mode, initial=None):
        self.setup_calls.append((pin, mode, initial))
        self.output_states[pin] = initial

    def PWM(self, pin, frequency):
        pwm = FakePwm(pin, frequency, self)
        self.pwms[pin] = pwm
        return pwm

    def output(self, pin, value):
        self.output_states[pin] = value

    def cleanup(self, pins):
        self.cleaned = pins


class HardwarePiTests(unittest.TestCase):
    def test_hardware_module_imports_without_pi_dependencies(self):
        self.assertIsInstance(hardware_pi.HARDWARE_AVAILABLE, bool)

    def test_null_motor_sink_records_normalized_commands(self):
        sink = hardware_pi.NullMotorSink()
        sink.set_drive(DifferentialDriveCommand(left=0.25, right=-0.5))
        self.assertEqual(sink.commands[-1], DifferentialDriveCommand(left=0.25, right=-0.5))

    def test_default_tb6612_pinout_matches_robot_wiring(self):
        pinout = hardware_pi.DEFAULT_TB6612_PINOUT
        self.assertEqual(pinout.right_stby, 21)
        self.assertEqual(pinout.left_stby, 27)
        self.assertEqual(pinout.right_rear, hardware_pi.TB6612MotorPins(pwm=12, in1=5, in2=6))
        self.assertEqual(pinout.right_front, hardware_pi.TB6612MotorPins(pwm=13, in1=16, in2=20))
        self.assertEqual(pinout.left_front, hardware_pi.TB6612MotorPins(pwm=18, in1=23, in2=24))
        self.assertEqual(pinout.left_rear, hardware_pi.TB6612MotorPins(pwm=19, in1=25, in2=26))

    def test_tb6612_sink_maps_left_and_right_commands_to_four_motors(self):
        gpio = FakeGPIO()
        limits = SafetyLimits(webots_base_speed=1.0, webots_max_speed=2.0, hardware_output_limit=0.5)
        sink = hardware_pi.TB6612GPIOMotorSink(limits=limits, gpio=gpio)

        sink.set_drive(DifferentialDriveCommand(left=1.0, right=-2.0))

        self.assertEqual(gpio.mode, gpio.BCM)
        self.assertEqual(gpio.output_states[21], gpio.HIGH)
        self.assertEqual(gpio.output_states[27], gpio.HIGH)
        self.assertEqual(gpio.output_states[5], gpio.LOW)
        self.assertEqual(gpio.output_states[6], gpio.HIGH)
        self.assertEqual(gpio.output_states[16], gpio.LOW)
        self.assertEqual(gpio.output_states[20], gpio.HIGH)
        self.assertEqual(gpio.output_states[23], gpio.HIGH)
        self.assertEqual(gpio.output_states[24], gpio.LOW)
        self.assertEqual(gpio.output_states[25], gpio.HIGH)
        self.assertEqual(gpio.output_states[26], gpio.LOW)
        self.assertEqual(gpio.pwms[12].duty_cycles[-1], 50.0)
        self.assertEqual(gpio.pwms[13].duty_cycles[-1], 50.0)
        self.assertEqual(gpio.pwms[18].duty_cycles[-1], 50.0)
        self.assertEqual(gpio.pwms[19].duty_cycles[-1], 50.0)

        sink.stop()

        self.assertEqual(gpio.output_states[21], gpio.LOW)
        self.assertEqual(gpio.output_states[27], gpio.LOW)
        for pin in (12, 13, 18, 19):
            self.assertEqual(gpio.pwms[pin].duty_cycles[-1], 0.0)

    def test_tb6612_motor_sign_can_reverse_one_channel_in_software(self):
        gpio = FakeGPIO()
        signs = hardware_pi.TB6612MotorSigns(left_front=-1)
        sink = hardware_pi.TB6612GPIOMotorSink(gpio=gpio, motor_signs=signs)

        sink.set_drive(DifferentialDriveCommand(left=1.0, right=1.0))

        self.assertEqual(gpio.output_states[23], gpio.LOW)
        self.assertEqual(gpio.output_states[24], gpio.HIGH)
        self.assertEqual(gpio.output_states[25], gpio.HIGH)
        self.assertEqual(gpio.output_states[26], gpio.LOW)

    def test_motor_signs_can_be_built_from_cli_values(self):
        signs = hardware_pi.tb6612_motor_signs_from_values(
            right_rear=-1,
            right_front=1,
            left_front="-1",
            left_rear="bad",
        )

        self.assertEqual(signs.right_rear, -1)
        self.assertEqual(signs.right_front, 1)
        self.assertEqual(signs.left_front, -1)
        self.assertEqual(signs.left_rear, 1)

    def test_right_rear_motor_sign_can_be_reversed_without_reversing_right_front(self):
        gpio = FakeGPIO()
        signs = hardware_pi.TB6612MotorSigns(right_rear=-1)
        sink = hardware_pi.TB6612GPIOMotorSink(gpio=gpio, motor_signs=signs)

        sink.set_drive(DifferentialDriveCommand(left=0.0, right=1.0))

        self.assertEqual(gpio.output_states[5], gpio.LOW)
        self.assertEqual(gpio.output_states[6], gpio.HIGH)
        self.assertEqual(gpio.output_states[16], gpio.HIGH)
        self.assertEqual(gpio.output_states[20], gpio.LOW)

    def test_v4l2_frame_source_reads_rgb_frame_from_ffmpeg_pipe(self):
        captured = {}
        raw_frame = bytes(
            [
                255,
                0,
                0,
                0,
                255,
                0,
            ]
        )

        class FakeProc:
            def __init__(self):
                self.stdout = io.BytesIO(raw_frame)
                self.stderr = io.BytesIO(b"")

            def poll(self):
                return 0

        def fake_popen(cmd, **kwargs):
            captured["cmd"] = cmd
            captured["kwargs"] = kwargs
            return FakeProc()

        source = hardware_pi.V4L2FrameSource(
            device="/dev/video-test",
            width=2,
            height=1,
            input_width=640,
            input_height=480,
            framerate=15,
            ffmpeg="fake-ffmpeg",
            popen_factory=fake_popen,
        )

        frame = source.read_rgb_array()
        source.close()

        self.assertEqual(frame.size, (2, 1))
        self.assertEqual(frame.getpixel((0, 0)), (255, 0, 0))
        self.assertEqual(frame.getpixel((1, 0)), (0, 255, 0))
        self.assertEqual(captured["cmd"][0], "fake-ffmpeg")
        self.assertIn("/dev/video-test", captured["cmd"])
        self.assertIn("640x480", captured["cmd"])
        self.assertIn("scale=2:1", captured["cmd"])
        self.assertEqual(captured["kwargs"]["stdout"], hardware_pi.subprocess.PIPE)


if __name__ == "__main__":
    unittest.main()
