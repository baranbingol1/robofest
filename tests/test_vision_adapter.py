import unittest

from PIL import Image

from ders_cizim.controllers.rgb_rl_controller.vision_adapter import RgbArrayCamera, RgbArrayCameraApi


class VisionAdapterTests(unittest.TestCase):
    def test_rgb_array_camera_exposes_webots_like_shape(self):
        frame = [
            [[10, 20, 30], [40, 50, 60]],
            [[70, 80, 90], [100, 110, 120]],
        ]
        camera = RgbArrayCamera(frame)
        self.assertEqual(camera.getWidth(), 2)
        self.assertEqual(camera.getHeight(), 2)
        self.assertIs(camera.getImage(), frame)

    def test_rgb_array_camera_api_reads_rgb_channels(self):
        frame = [
            [[10, 20, 30], [40, 50, 60]],
            [[70, 80, 90], [100, 110, 120]],
        ]
        self.assertEqual(RgbArrayCameraApi.imageGetRed(frame, 2, 1, 0), 40)
        self.assertEqual(RgbArrayCameraApi.imageGetGreen(frame, 2, 1, 0), 50)
        self.assertEqual(RgbArrayCameraApi.imageGetBlue(frame, 2, 1, 0), 60)

    def test_pil_image_frames_are_supported_for_offline_capture_replay(self):
        frame = Image.new("RGB", (2, 1))
        frame.putpixel((0, 0), (11, 22, 33))
        frame.putpixel((1, 0), (44, 55, 66))
        camera = RgbArrayCamera(frame)
        self.assertEqual(camera.getWidth(), 2)
        self.assertEqual(camera.getHeight(), 1)
        self.assertEqual(RgbArrayCameraApi.imageGetRed(frame, 2, 1, 0), 44)
        self.assertEqual(RgbArrayCameraApi.imageGetGreen(frame, 2, 1, 0), 55)
        self.assertEqual(RgbArrayCameraApi.imageGetBlue(frame, 2, 1, 0), 66)


if __name__ == "__main__":
    unittest.main()
