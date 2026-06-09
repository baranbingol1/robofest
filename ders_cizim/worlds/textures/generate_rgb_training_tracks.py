from __future__ import annotations

import argparse
import random
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw


SIZE = 1024
SCALE = 3
CANVAS = SIZE * SCALE
BACKGROUND = (247, 247, 244, 255)
SHADOW = (210, 210, 205, 210)
BLACK = (18, 18, 18, 255)
RED = (238, 28, 42, 255)
GREEN = (20, 178, 80, 255)
BLUE = (38, 94, 232, 255)


@dataclass(frozen=True, slots=True)
class TrackVariant:
    name: str
    line_width: int
    brightness: float
    color_scale: float
    path_jitter: float = 0.0
    gap_count: int = 0
    gap_radius: int = 0
    dirt_count: int = 0
    dirt_radius: int = 0
    speckle_count: int = 0
    seed: int = 0


def generate_variant_specs(count: int, seed: int = 7) -> list[TrackVariant]:
    rng = random.Random(seed)
    specs: list[TrackVariant] = []
    for index in range(max(0, count)):
        specs.append(
            TrackVariant(
                name=f"variant_{index:02d}",
                line_width=rng.randint(34, 50),
                brightness=round(rng.uniform(0.88, 1.10), 3),
                color_scale=round(rng.uniform(0.88, 1.10), 3),
            )
        )
    return specs


def generate_stress_variant_specs(count: int, seed: int = 41) -> list[TrackVariant]:
    rng = random.Random(seed)
    specs: list[TrackVariant] = []
    for index in range(max(0, count)):
        specs.append(
            TrackVariant(
                name=f"stress_{index:02d}",
                line_width=rng.randint(36, 48),
                brightness=round(rng.uniform(0.90, 1.08), 3),
                color_scale=round(rng.uniform(0.90, 1.08), 3),
                path_jitter=round(rng.uniform(0.0, 0.025), 4),
                gap_count=rng.randint(1, 3),
                gap_radius=rng.randint(7, 13),
                dirt_count=rng.randint(8, 18),
                dirt_radius=rng.randint(5, 14),
                speckle_count=rng.randint(140, 320),
                seed=rng.randrange(1, 2_000_000_000),
            )
        )
    return specs


def scale_color(color: tuple[int, int, int, int], scale: float) -> tuple[int, int, int, int]:
    red, green, blue, alpha = color
    return (
        max(0, min(255, round(red * scale))),
        max(0, min(255, round(green * scale))),
        max(0, min(255, round(blue * scale))),
        alpha,
    )


def world_to_px(point: tuple[float, float]) -> tuple[int, int]:
    x, y = point
    return (round((x + 1.0) * 0.5 * CANVAS), round((1.0 - (y + 1.0) * 0.5) * CANVAS))


def catmull_rom(points: list[tuple[float, float]], samples: int = 28) -> list[tuple[int, int]]:
    if len(points) < 2:
        return [world_to_px(point) for point in points]

    padded = [points[0], *points, points[-1]]
    output: list[tuple[int, int]] = []
    for index in range(1, len(padded) - 2):
        p0 = padded[index - 1]
        p1 = padded[index]
        p2 = padded[index + 1]
        p3 = padded[index + 2]
        for sample in range(samples):
            t = sample / samples
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * (
                (2 * p1[0])
                + (-p0[0] + p2[0]) * t
                + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
            )
            y = 0.5 * (
                (2 * p1[1])
                + (-p0[1] + p2[1]) * t
                + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
            )
            output.append(world_to_px((x, y)))
    output.append(world_to_px(points[-1]))
    return output


def jitter_points(
    points: list[tuple[float, float]],
    rng: random.Random,
    amount: float,
    *,
    preserve_ends: bool = True,
) -> list[tuple[float, float]]:
    if amount <= 0:
        return list(points)
    jittered: list[tuple[float, float]] = []
    for index, (x, y) in enumerate(points):
        if preserve_ends and index in {0, len(points) - 1}:
            jittered.append((x, y))
            continue
        jittered.append(
            (
                max(-0.92, min(0.92, x + rng.uniform(-amount, amount))),
                max(-0.92, min(0.92, y + rng.uniform(-amount, amount))),
            )
        )
    return jittered


def draw_track(
    draw: ImageDraw.ImageDraw,
    points: list[tuple[float, float]],
    color: tuple[int, int, int, int],
    width: int,
    shadow: tuple[int, int, int, int] = SHADOW,
) -> None:
    px_points = catmull_rom(points)
    scaled_width = width * SCALE
    shadow_width = (width + 16) * SCALE
    draw.line(px_points, fill=shadow, width=shadow_width, joint="curve")
    radius = shadow_width // 2
    for x, y in px_points[:: max(1, len(px_points) // 30)]:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=shadow)
    draw.line(px_points, fill=color, width=scaled_width, joint="curve")
    radius = scaled_width // 2
    for x, y in px_points[:: max(1, len(px_points) // 30)]:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
    for x, y in (px_points[0], px_points[-1]):
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)


def draw_floor_artifacts(
    draw: ImageDraw.ImageDraw,
    rng: random.Random,
    *,
    background: tuple[int, int, int, int],
    dirt_count: int,
    dirt_radius: int,
    speckle_count: int,
) -> None:
    for _ in range(max(0, dirt_count)):
        x = rng.randint(35 * SCALE, CANVAS - 35 * SCALE)
        y = rng.randint(35 * SCALE, CANVAS - 35 * SCALE)
        radius = rng.randint(max(2, dirt_radius // 2), max(3, dirt_radius)) * SCALE
        delta = rng.randint(-28, 18)
        color = tuple(max(0, min(255, channel + delta)) for channel in background[:3]) + (rng.randint(80, 150),)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)

    for _ in range(max(0, speckle_count)):
        x = rng.randint(0, CANVAS - 1)
        y = rng.randint(0, CANVAS - 1)
        radius = rng.randint(1, 2) * SCALE
        delta = rng.randint(-35, 25)
        color = tuple(max(0, min(255, channel + delta)) for channel in background[:3]) + (rng.randint(100, 200),)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)


def draw_line_gaps(
    draw: ImageDraw.ImageDraw,
    rng: random.Random,
    paths: list[list[tuple[float, float]]],
    *,
    background: tuple[int, int, int, int],
    gap_count: int,
    gap_radius: int,
) -> None:
    if gap_count <= 0 or gap_radius <= 0:
        return
    candidate_points: list[tuple[int, int]] = []
    for path in paths:
        px_points = catmull_rom(path)
        if len(px_points) < 8:
            continue
        trim = max(4, len(px_points) // 12)
        candidate_points.extend(px_points[trim:-trim])
    if not candidate_points:
        return
    for _ in range(gap_count):
        x, y = rng.choice(candidate_points)
        radius_x = rng.randint(gap_radius, gap_radius + 7) * SCALE
        radius_y = rng.randint(max(4, gap_radius - 3), gap_radius + 4) * SCALE
        draw.ellipse((x - radius_x, y - radius_y, x + radius_x, y + radius_y), fill=background)


def build_track_image(variant: TrackVariant | None = None) -> Image.Image:
    line_width = 42 if variant is None else variant.line_width
    common_width = max(30, line_width - 4)
    brightness = 1.0 if variant is None else variant.brightness
    color_scale = 1.0 if variant is None else variant.color_scale
    background = scale_color(BACKGROUND, brightness)
    shadow = scale_color(SHADOW, brightness)
    rng = random.Random(0 if variant is None else variant.seed)
    image = Image.new("RGBA", (CANVAS, CANVAS), background)
    draw = ImageDraw.Draw(image)
    if variant is not None:
        draw_floor_artifacts(
            draw,
            rng,
            background=background,
            dirt_count=variant.dirt_count,
            dirt_radius=variant.dirt_radius,
            speckle_count=variant.speckle_count,
        )

    common = [(-0.74, -0.70), (-0.42, -0.70), (-0.06, -0.70), (0.38, -0.70)]
    fork = (0.38, -0.70)
    red = [
        fork,
        (0.50, -0.70),
        (0.65, -0.62),
        (0.80, -0.60),
        (0.84, -0.48),
        (0.73, -0.37),
        (0.57, -0.42),
        (0.48, -0.55),
        (0.42, -0.66),
        fork,
    ]
    green = [
        fork,
        (0.24, -0.58),
        (0.02, -0.47),
        (-0.52, -0.36),
        (-0.82, -0.12),
        (-0.80, 0.23),
        (-0.53, 0.43),
        (-0.22, 0.36),
        (-0.06, 0.25),
        (0.04, 0.03),
        (0.00, -0.28),
        (0.22, -0.56),
        fork,
    ]
    blue = [
        fork,
        (0.39, -0.49),
        (0.44, -0.21),
        (0.47, 0.08),
        (0.62, 0.36),
        (0.58, 0.64),
        (0.35, 0.79),
        (0.10, 0.68),
        (-0.13, 0.46),
        (0.05, 0.28),
        (0.24, 0.06),
        (0.32, -0.32),
        fork,
    ]
    path_jitter = 0.0 if variant is None else variant.path_jitter
    common = jitter_points(common, rng, path_jitter * 0.4, preserve_ends=True)
    red = jitter_points(red, rng, path_jitter, preserve_ends=True)
    green = jitter_points(green, rng, path_jitter, preserve_ends=True)
    blue = jitter_points(blue, rng, path_jitter, preserve_ends=True)

    draw_track(draw, red, scale_color(RED, color_scale), width=line_width, shadow=shadow)
    draw_track(draw, green, scale_color(GREEN, color_scale), width=line_width, shadow=shadow)
    draw_track(draw, blue, scale_color(BLUE, color_scale), width=line_width, shadow=shadow)
    draw_track(draw, common, BLACK, width=common_width, shadow=shadow)
    if variant is not None:
        draw_line_gaps(
            draw,
            rng,
            [red, green, blue, common],
            background=background,
            gap_count=variant.gap_count,
            gap_radius=variant.gap_radius,
        )

    fork_x, fork_y = world_to_px(fork)
    fork_radius = 26 * SCALE
    draw.ellipse(
        (fork_x - fork_radius, fork_y - fork_radius, fork_x + fork_radius, fork_y + fork_radius),
        fill=BLACK,
    )

    border_width = 4 * SCALE
    draw.rectangle(
        (border_width, border_width, CANVAS - border_width, CANVAS - border_width),
        outline=(205, 205, 200, 255),
        width=border_width,
    )

    image = image.resize((SIZE, SIZE), Image.Resampling.LANCZOS).convert("RGB")
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variants", type=int, default=0, help="Generate this many deterministic variants")
    parser.add_argument("--stress-variants", type=int, default=0, help="Generate topology/appearance stress variants")
    parser.add_argument("--seed", type=int, default=7, help="Seed for deterministic variants")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).parent,
        help="Directory for generated textures",
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    variants: list[TrackVariant] = []
    if args.variants > 0:
        variants.extend(generate_variant_specs(args.variants, seed=args.seed))
    if args.stress_variants > 0:
        variants.extend(generate_stress_variant_specs(args.stress_variants, seed=args.seed))
    if variants:
        for variant in variants:
            output = args.output_dir / f"rgb_training_tracks_{variant.name}.png"
            build_track_image(variant).save(output)
            print(output)
        return

    image = build_track_image()
    output = Path(__file__).with_name("rgb_training_tracks.png")
    image.save(output)
    print(output)


if __name__ == "__main__":
    main()
