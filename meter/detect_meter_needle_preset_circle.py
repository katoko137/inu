from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Tuple

import cv2

from detect_meter_needle import (
    Circle,
    Needle,
    choose_needle_line,
    cartesian_angle_from_image_angle,
    draw_detection,
    image_angle_from_points,
    refine_needle_tip,
)


BASE_DIR = Path(__file__).resolve().parent
IMAGE_PATH = BASE_DIR / "meter_img" / "test2.jpg"
DEFAULT_OUTPUT_PATH = BASE_DIR / "output" / "test2_needle_detected_preset_circle.jpg"

DEFAULT_CENTER_X = 770
DEFAULT_CENTER_Y = 469
DEFAULT_RADIUS = 154


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect the analog meter needle in meter_img/test2.jpg using a "
            "predefined meter circle."
        )
    )
    parser.add_argument(
        "--center-x",
        type=int,
        default=DEFAULT_CENTER_X,
        help=f"Preset meter circle center x. Default: {DEFAULT_CENTER_X}",
    )
    parser.add_argument(
        "--center-y",
        type=int,
        default=DEFAULT_CENTER_Y,
        help=f"Preset meter circle center y. Default: {DEFAULT_CENTER_Y}",
    )
    parser.add_argument(
        "--radius",
        type=int,
        default=DEFAULT_RADIUS,
        help=f"Preset meter circle radius. Default: {DEFAULT_RADIUS}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help="Path for the annotated detection image.",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Print the result without writing the annotated image.",
    )
    return parser.parse_args()


def validate_circle(image_shape: Tuple[int, int, int], circle: Circle) -> None:
    height, width = image_shape[:2]
    if circle.radius <= 0:
        raise ValueError("Meter radius must be greater than 0.")
    if not (0 <= circle.x < width and 0 <= circle.y < height):
        raise ValueError(
            f"Meter center ({circle.x}, {circle.y}) is outside the image "
            f"size ({width}, {height})."
        )
    if circle.x - circle.radius < 0 or circle.y - circle.radius < 0:
        raise ValueError("Preset meter circle extends outside the image.")
    if circle.x + circle.radius >= width or circle.y + circle.radius >= height:
        raise ValueError("Preset meter circle extends outside the image.")


def detect_needle_with_preset_circle(image, circle: Circle) -> Needle:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    segment, line_score, threshold = choose_needle_line(gray, circle)
    x1, y1, x2, y2 = segment

    d1 = math.hypot(x1 - circle.x, y1 - circle.y)
    d2 = math.hypot(x2 - circle.x, y2 - circle.y)
    rough_tip = (x1, y1) if d1 > d2 else (x2, y2)

    tip, _image_angle = refine_needle_tip(gray, circle, rough_tip, threshold)
    image_angle = image_angle_from_points((circle.x, circle.y), tip)
    cartesian_angle = cartesian_angle_from_image_angle(image_angle)

    return Needle(
        center=(circle.x, circle.y),
        tip=tip,
        image_angle_deg=image_angle,
        cartesian_angle_deg=cartesian_angle,
        line_score=line_score,
        line_threshold=threshold,
    )


def main() -> int:
    args = parse_args()
    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

    circle = Circle(args.center_x, args.center_y, args.radius, score=0.0)
    validate_circle(image.shape, circle)
    needle = detect_needle_with_preset_circle(image, circle)

    print(f"image: {IMAGE_PATH}")
    print(
        "preset_meter_circle: "
        f"center=({circle.x}, {circle.y}), radius={circle.radius}"
    )
    print(
        "needle: "
        f"center={needle.center}, tip={needle.tip}, "
        f"image_angle={needle.image_angle_deg:.2f} deg, "
        f"cartesian_angle={needle.cartesian_angle_deg:.2f} deg"
    )

    if not args.no_save:
        output_path = args.output
        if not output_path.is_absolute():
            output_path = BASE_DIR / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)

        annotated = draw_detection(image, circle, needle)
        if not cv2.imwrite(str(output_path), annotated):
            raise RuntimeError(f"Could not write output image: {output_path}")
        print(f"output: {output_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
