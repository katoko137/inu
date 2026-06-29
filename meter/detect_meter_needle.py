from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np


BASE_DIR = Path(__file__).resolve().parent
IMAGE_PATH = BASE_DIR / "meter_img" / "test2.jpg"
DEFAULT_OUTPUT_PATH = BASE_DIR / "output" / "test2_needle_detected.jpg"
DEFAULT_DRAW_LINE_THICKNESS = 1
DEFAULT_DRAW_NEEDLE_TIP_POINT = False


@dataclass(frozen=True)
class Circle:
    x: int
    y: int
    radius: int
    score: float


@dataclass(frozen=True)
class Needle:
    center: Tuple[int, int]
    tip: Tuple[int, int]
    image_angle_deg: float
    cartesian_angle_deg: float
    line_score: float
    line_threshold: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect the analog meter needle in meter_img/test2.jpg."
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


def normalize_angle(angle_deg: float) -> float:
    return angle_deg % 360.0


def image_angle_from_points(
    center: Tuple[int, int], point: Tuple[int, int]
) -> float:
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    return normalize_angle(math.degrees(math.atan2(dy, dx)))


def cartesian_angle_from_image_angle(image_angle_deg: float) -> float:
    return normalize_angle(360.0 - image_angle_deg)


def unique_circles(
    candidates: Iterable[Tuple[float, float, float]], distance_tol: float = 6.0
) -> list[Tuple[float, float, float]]:
    unique: list[Tuple[float, float, float]] = []
    for circle in candidates:
        cx, cy, radius = circle
        already_seen = False
        for ux, uy, ur in unique:
            same_center = math.hypot(cx - ux, cy - uy) < distance_tol
            same_radius = abs(radius - ur) < distance_tol
            if same_center and same_radius:
                already_seen = True
                break
        if not already_seen:
            unique.append(circle)
    return unique


def score_circle(gray: np.ndarray, cx: float, cy: float, radius: float) -> float:
    height, width = gray.shape
    if cx - radius < 0 or cy - radius < 0:
        return -1_000_000.0
    if cx + radius >= width or cy + radius >= height:
        return -1_000_000.0

    center = (round(cx), round(cy))
    inner_mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(inner_mask, center, round(radius * 0.72), 255, -1)

    ring_mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(ring_mask, center, round(radius * 1.05), 255, -1)
    cv2.circle(ring_mask, center, round(radius * 0.88), 0, -1)

    inner_values = gray[inner_mask > 0]
    ring_values = gray[ring_mask > 0]
    if inner_values.size == 0 or ring_values.size == 0:
        return -1_000_000.0

    # The target meter has a bright face and a dark outer bezel.
    return float(inner_values.mean()) - float(ring_values.mean()) + float(
        inner_values.mean()
    ) * 0.05


def find_meter_circle(gray: np.ndarray) -> Circle:
    blur = cv2.medianBlur(gray, 5)
    min_dim = min(gray.shape[:2])
    min_radius = max(60, int(min_dim * 0.11))
    max_radius = int(min_dim * 0.32)

    raw_candidates: list[Tuple[float, float, float]] = []
    for accumulator_threshold in range(70, 19, -5):
        circles = cv2.HoughCircles(
            blur,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=100,
            param1=100,
            param2=accumulator_threshold,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is not None:
            raw_candidates.extend(tuple(circle) for circle in circles[0])

    if not raw_candidates:
        raise RuntimeError("No meter circle was detected.")

    best: Optional[Circle] = None
    for cx, cy, radius in unique_circles(raw_candidates):
        score = score_circle(gray, cx, cy, radius)
        circle = Circle(round(cx), round(cy), round(radius), score)
        if best is None or circle.score > best.score:
            best = circle

    if best is None:
        raise RuntimeError("No valid meter circle candidate remained.")
    return best


def distance_point_to_segment(
    point: Tuple[int, int], segment: Tuple[int, int, int, int]
) -> float:
    px, py = point
    x1, y1, x2, y2 = segment
    vx = x2 - x1
    vy = y2 - y1
    length_sq = vx * vx + vy * vy
    if length_sq == 0:
        return math.hypot(px - x1, py - y1)

    t = ((px - x1) * vx + (py - y1) * vy) / length_sq
    t = max(0.0, min(1.0, t))
    nearest_x = x1 + vx * t
    nearest_y = y1 + vy * t
    return math.hypot(px - nearest_x, py - nearest_y)


def dark_ratio_on_segment(
    gray: np.ndarray, segment: Tuple[int, int, int, int], threshold: int
) -> float:
    x1, y1, x2, y2 = segment
    samples = 60
    hits = 0
    for index in range(samples):
        t = index / (samples - 1)
        px = round(x1 + (x2 - x1) * t)
        py = round(y1 + (y2 - y1) * t)
        if 0 <= px < gray.shape[1] and 0 <= py < gray.shape[0]:
            if int(gray[py, px]) <= threshold + 10:
                hits += 1
    return hits / samples


def choose_needle_line(
    gray: np.ndarray, circle: Circle
) -> Tuple[Tuple[int, int, int, int], float, int]:
    center = (circle.x, circle.y)
    mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.circle(mask, center, round(circle.radius * 0.82), 255, -1)
    cv2.circle(mask, center, round(circle.radius * 0.06), 0, -1)

    best_segment: Optional[Tuple[int, int, int, int]] = None
    best_score = -1_000_000.0
    best_threshold = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    for threshold in range(50, 126, 5):
        dark = cv2.inRange(gray, 0, threshold)
        dark = cv2.bitwise_and(dark, dark, mask=mask)
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel, iterations=1)
        edges = cv2.Canny(dark, 50, 150)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=18,
            minLineLength=round(circle.radius * 0.18),
            maxLineGap=round(circle.radius * 0.08),
        )
        if lines is None:
            continue

        for raw_line in lines[:, 0, :]:
            segment = tuple(int(value) for value in raw_line)
            x1, y1, x2, y2 = segment
            length = math.hypot(x2 - x1, y2 - y1)
            if length < circle.radius * 0.22:
                continue

            center_distance = distance_point_to_segment(center, segment)
            if center_distance > max(12.0, circle.radius * 0.12):
                continue

            d1 = math.hypot(x1 - circle.x, y1 - circle.y)
            d2 = math.hypot(x2 - circle.x, y2 - circle.y)
            far_distance = max(d1, d2)
            near_distance = min(d1, d2)
            if far_distance < circle.radius * 0.35:
                continue
            if far_distance > circle.radius * 0.95:
                continue

            dark_ratio = dark_ratio_on_segment(gray, segment, threshold)
            if dark_ratio < 0.35:
                continue

            score = (
                length
                + far_distance * 0.85
                - center_distance * 4.0
                + dark_ratio * 25.0
                - max(0.0, near_distance - circle.radius * 0.30) * 0.5
            )
            if score > best_score:
                best_segment = segment
                best_score = score
                best_threshold = threshold

    if best_segment is None:
        raise RuntimeError("No needle line was detected.")

    return best_segment, best_score, best_threshold


def refine_needle_tip(
    gray: np.ndarray,
    circle: Circle,
    rough_tip: Tuple[int, int],
    threshold: int,
) -> Tuple[Tuple[int, int], float]:
    rough_angle = image_angle_from_points((circle.x, circle.y), rough_tip)
    scan_threshold = min(90, max(70, threshold))
    dark = gray <= scan_threshold

    best_score = -1_000_000.0
    best_angle = rough_angle
    best_tip_radius = circle.radius * 0.65
    width = max(3, round(circle.radius * 0.025))
    radii = np.linspace(circle.radius * 0.08, circle.radius * 0.84, 140)

    for angle in np.linspace(rough_angle - 18.0, rough_angle + 18.0, 145):
        angle_rad = math.radians(angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        weighted_hits = 0.0
        connected_hits = 0
        max_connected_hits = 0
        near_hits = 0
        last_hit_radius = 0.0

        for radius in radii:
            hit = False
            for offset in range(-width, width + 1):
                px = round(circle.x + radius * cos_a - offset * sin_a)
                py = round(circle.y + radius * sin_a + offset * cos_a)
                if 0 <= px < gray.shape[1] and 0 <= py < gray.shape[0]:
                    if bool(dark[py, px]):
                        hit = True
                        break

            if hit:
                weighted_hits += radius / circle.radius
                connected_hits += 1
                last_hit_radius = radius
                if radius < circle.radius * 0.25:
                    near_hits += 1
            else:
                connected_hits = max(0, connected_hits - 2)

            max_connected_hits = max(max_connected_hits, connected_hits)

        score = (
            weighted_hits
            + max_connected_hits * 0.35
            + (last_hit_radius / circle.radius) * 10.0
            + min(near_hits, 20) * 0.3
        )
        if score > best_score:
            best_score = score
            best_angle = normalize_angle(angle)
            best_tip_radius = last_hit_radius

    angle_rad = math.radians(best_angle)
    tip = (
        round(circle.x + best_tip_radius * math.cos(angle_rad)),
        round(circle.y + best_tip_radius * math.sin(angle_rad)),
    )
    return tip, best_angle


def detect_needle(image: np.ndarray) -> Tuple[Circle, Needle]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    circle = find_meter_circle(gray)
    segment, line_score, threshold = choose_needle_line(gray, circle)
    x1, y1, x2, y2 = segment

    d1 = math.hypot(x1 - circle.x, y1 - circle.y)
    d2 = math.hypot(x2 - circle.x, y2 - circle.y)
    rough_tip = (x1, y1) if d1 > d2 else (x2, y2)
    tip, image_angle = refine_needle_tip(gray, circle, rough_tip, threshold)
    cartesian_angle = cartesian_angle_from_image_angle(image_angle)

    needle = Needle(
        center=(circle.x, circle.y),
        tip=tip,
        image_angle_deg=image_angle,
        cartesian_angle_deg=cartesian_angle,
        line_score=line_score,
        line_threshold=threshold,
    )
    return circle, needle


def draw_detection(
    image: np.ndarray,
    circle: Circle,
    needle: Needle,
    draw_text: bool = True,
    line_thickness: int = DEFAULT_DRAW_LINE_THICKNESS,
    draw_tip_point: bool = DEFAULT_DRAW_NEEDLE_TIP_POINT,
) -> np.ndarray:
    annotated = image.copy()
    center = needle.center
    tip = needle.tip
    thickness = max(1, int(line_thickness))

    cv2.circle(annotated, center, circle.radius, (0, 180, 0), thickness)
    cv2.circle(annotated, center, 5, (255, 0, 0), -1)
    cv2.line(annotated, center, tip, (0, 0, 255), thickness, cv2.LINE_AA)
    if draw_tip_point:
        cv2.circle(annotated, tip, 7, (0, 0, 255), -1)

    if draw_text:
        label = f"needle {needle.image_angle_deg:.1f} deg"
        cv2.putText(
            annotated,
            label,
            (center[0] - 90, center[1] - circle.radius - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            thickness,
            cv2.LINE_AA,
        )
    return annotated


def main() -> int:
    args = parse_args()
    image = cv2.imread(str(IMAGE_PATH))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

    circle, needle = detect_needle(image)

    print(f"image: {IMAGE_PATH}")
    print(
        "meter_circle: "
        f"center=({circle.x}, {circle.y}), radius={circle.radius}, "
        f"score={circle.score:.2f}"
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
