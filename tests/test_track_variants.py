import unittest

from ders_cizim.worlds.textures.generate_rgb_training_tracks import (
    SIZE,
    TrackVariant,
    build_track_image,
    generate_stress_variant_specs,
    generate_variant_specs,
)


class TrackVariantTests(unittest.TestCase):
    def test_generate_variant_specs_is_deterministic(self):
        first = generate_variant_specs(4, seed=23)
        second = generate_variant_specs(4, seed=23)
        self.assertEqual(first, second)

    def test_generate_variant_specs_stays_in_safe_ranges(self):
        specs = generate_variant_specs(12, seed=5)
        self.assertEqual(len(specs), 12)
        for spec in specs:
            self.assertGreaterEqual(spec.line_width, 34)
            self.assertLessEqual(spec.line_width, 50)
            self.assertGreaterEqual(spec.brightness, 0.88)
            self.assertLessEqual(spec.brightness, 1.10)
            self.assertGreaterEqual(spec.color_scale, 0.88)
            self.assertLessEqual(spec.color_scale, 1.10)

    def test_generate_stress_variant_specs_is_deterministic(self):
        first = generate_stress_variant_specs(4, seed=19)
        second = generate_stress_variant_specs(4, seed=19)
        self.assertEqual(first, second)
        self.assertTrue(all(spec.name.startswith("stress_") for spec in first))

    def test_generate_stress_variant_specs_stays_in_safe_ranges(self):
        specs = generate_stress_variant_specs(12, seed=17)
        self.assertEqual(len(specs), 12)
        for spec in specs:
            self.assertGreaterEqual(spec.line_width, 36)
            self.assertLessEqual(spec.line_width, 48)
            self.assertGreaterEqual(spec.path_jitter, 0.0)
            self.assertLessEqual(spec.path_jitter, 0.025)
            self.assertGreaterEqual(spec.gap_count, 1)
            self.assertLessEqual(spec.gap_count, 3)
            self.assertGreaterEqual(spec.speckle_count, 140)
            self.assertLessEqual(spec.speckle_count, 320)

    def test_stress_variant_image_is_rgb_and_deterministic(self):
        spec = TrackVariant(
            name="stress_test",
            line_width=42,
            brightness=1.0,
            color_scale=1.0,
            path_jitter=0.02,
            gap_count=2,
            gap_radius=8,
            dirt_count=4,
            dirt_radius=6,
            speckle_count=20,
            seed=123,
        )
        first = build_track_image(spec)
        second = build_track_image(spec)
        baseline = build_track_image()
        self.assertEqual(first.mode, "RGB")
        self.assertEqual(first.size, (SIZE, SIZE))
        self.assertEqual(first.tobytes(), second.tobytes())
        self.assertNotEqual(first.tobytes(), baseline.tobytes())


if __name__ == "__main__":
    unittest.main()
