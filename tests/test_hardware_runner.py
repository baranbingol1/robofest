import tempfile
import unittest
from pathlib import Path

from ders_cizim.controllers.rgb_rl_controller.hardware_pi import NullMotorSink
from ders_cizim.controllers.rgb_rl_controller.hardware_runner import (
    HardwareRunConfig,
    camera_ready,
    run_hardware_loop,
)
from ders_cizim.controllers.rgb_rl_controller.control_core import line_follow_command
from ders_cizim.controllers.rgb_rl_controller.robot_config import DEFAULT_SAFETY_LIMITS


class FakeFrameSource:
    def __init__(self, frames):
        self.frames = list(frames)
        self.closed = False

    def read_rgb_array(self):
        return self.frames.pop(0) if self.frames else object()

    def close(self):
        self.closed = True


def analyzer_from_records(records):
    rows = list(records)

    def analyze(_frame, target_color, previous_error):
        row = dict(rows.pop(0))
        row.setdefault("target_color", target_color)
        row.setdefault("center_error", previous_error)
        row.setdefault("confidence", 0.0)
        row.setdefault("color_name", "none")
        row.setdefault("matched_target", False)
        row.setdefault("line_width_ratio", 0.0)
        row.setdefault("rgb_balance", [0.0, 0.0, 0.0])
        row.setdefault("threshold", 0.0)
        return row, float(row["center_error"])

    return analyze


class HardwareRunnerTests(unittest.TestCase):
    def test_camera_ready_uses_configured_thresholds(self):
        config = HardwareRunConfig()
        self.assertTrue(
            camera_ready(
                {
                    "visible_ratio": 1.0,
                    "mean_confidence": 0.9,
                    "min_line_width_ratio": 0.12,
                    "max_line_width_ratio": 0.18,
                },
                config,
            )
        )
        self.assertFalse(
            camera_ready(
                {
                    "visible_ratio": 1.0,
                    "mean_confidence": 0.9,
                    "min_line_width_ratio": 0.03,
                    "max_line_width_ratio": 0.18,
                },
                config,
            )
        )

    def test_hardware_loop_refuses_to_drive_when_camera_is_not_ready(self):
        with tempfile.TemporaryDirectory() as tmp:
            records = [
                {"visible": False, "center_error": 0.0, "confidence": 0.0, "line_width_ratio": 0.0},
                {"visible": False, "center_error": 0.0, "confidence": 0.0, "line_width_ratio": 0.0},
            ]
            source = FakeFrameSource([object(), object()])
            sink = NullMotorSink()
            config = HardwareRunConfig(
                max_frames=5,
                camera_ready_warmup_frames=2,
                lost_stop_frames=10,
                output_path=Path(tmp) / "run.json",
            )
            summary = run_hardware_loop(
                config,
                source,
                sink,
                analyzer=analyzer_from_records(records),
                sleeper=lambda _seconds: None,
                clock=lambda: 0.0,
            )
        self.assertEqual(summary.stopped_reason, "camera_not_ready")
        self.assertEqual(summary.commands_sent, 0)
        self.assertTrue(source.closed)
        self.assertGreaterEqual(len(sink.commands), 1)
        self.assertTrue(all(command.left == 0.0 and command.right == 0.0 for command in sink.commands))

    def test_hardware_loop_drives_after_ready_warmup_and_stops_on_lost_line(self):
        with tempfile.TemporaryDirectory() as tmp:
            records = [
                {"visible": True, "center_error": 0.0, "confidence": 0.9, "line_width_ratio": 0.12},
                {"visible": True, "center_error": 0.1, "confidence": 0.9, "line_width_ratio": 0.12},
                {"visible": False, "center_error": 0.1, "confidence": 0.0, "line_width_ratio": 0.0},
                {"visible": False, "center_error": 0.1, "confidence": 0.0, "line_width_ratio": 0.0},
            ]
            source = FakeFrameSource([object(), object(), object(), object()])
            sink = NullMotorSink()
            config = HardwareRunConfig(
                max_frames=4,
                camera_ready_warmup_frames=1,
                lost_stop_frames=2,
                output_path=Path(tmp) / "run.json",
            )
            summary = run_hardware_loop(
                config,
                source,
                sink,
                analyzer=analyzer_from_records(records),
                sleeper=lambda _seconds: None,
                clock=lambda: 0.0,
            )
        self.assertEqual(summary.stopped_reason, "lost_line")
        self.assertEqual(summary.commands_sent, 2)
        self.assertGreater(len(sink.commands), 1)

    def test_hardware_loop_uses_shared_pd_line_follow_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            records = [
                {"visible": True, "center_error": 0.0, "confidence": 0.9, "line_width_ratio": 0.12},
                {"visible": True, "center_error": 0.2, "confidence": 0.9, "line_width_ratio": 0.12},
            ]
            source = FakeFrameSource([object(), object()])
            sink = NullMotorSink()
            config = HardwareRunConfig(
                max_frames=2,
                camera_ready_warmup_frames=1,
                lost_stop_frames=3,
                output_path=Path(tmp) / "run.json",
            )
            run_hardware_loop(
                config,
                source,
                sink,
                analyzer=analyzer_from_records(records),
                sleeper=lambda _seconds: None,
                clock=lambda: 0.0,
            )

        expected = line_follow_command(
            type("Profile", (), {"visible": True, "center_error": 0.2, "confidence": 0.9})(),
            previous_error=0.0,
            limits=DEFAULT_SAFETY_LIMITS,
        )
        self.assertGreaterEqual(len(sink.commands), 2)
        self.assertAlmostEqual(sink.commands[1].left, expected.left)
        self.assertAlmostEqual(sink.commands[1].right, expected.right)


if __name__ == "__main__":
    unittest.main()
