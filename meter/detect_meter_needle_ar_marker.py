from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from detect_meter_needle import Circle, Needle, draw_detection, find_meter_circle
from detect_meter_needle_preset_circle import (
    detect_needle_with_preset_circle,
    validate_circle,
)


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_IMAGE_PATH = BASE_DIR / "meter_img" / "test2_with_ar.png"
DEFAULT_CONFIG_PATH = BASE_DIR / "ar_marker_meter_config.json"
DEFAULT_OUTPUT_DIR = BASE_DIR / "output"
DEFAULT_DRAW_LINE_THICKNESS = 1
DEFAULT_DRAW_POINT_RADIUS = 8
DEFAULT_DRAW_NEEDLE_TIP_POINT = False
MARKER_PLANE_CORNERS = np.array(
    [
        [-0.5, -0.5],
        [0.5, -0.5],
        [0.5, 0.5],
        [-0.5, 0.5],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    corners: np.ndarray
    center: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    side_length: float
    marker_to_image_homography: np.ndarray
    image_to_marker_homography: np.ndarray


@dataclass(frozen=True)
class MarkerMeterRegistration:
    dictionary: str
    marker_id: int
    center_rel_x: float
    center_rel_y: float
    radius_rel: float
    max_width: int
    registration_image: str
    registration_circle_x: int
    registration_circle_y: int
    registration_circle_radius: int
    registration_work_width: Optional[int] = None
    registration_work_height: Optional[int] = None
    scale_min_value: Optional[float] = None
    scale_max_value: Optional[float] = None
    scale_min_rel_x: Optional[float] = None
    scale_min_rel_y: Optional[float] = None
    scale_max_rel_x: Optional[float] = None
    scale_max_rel_y: Optional[float] = None
    scale_direction: str = "clockwise"
    registration_scale_min_x: Optional[int] = None
    registration_scale_min_y: Optional[int] = None
    registration_scale_max_x: Optional[int] = None
    registration_scale_max_y: Optional[int] = None


@dataclass(frozen=True)
class MeterRectification:
    image: np.ndarray
    circle: Circle
    image_to_rect_homography: np.ndarray
    rect_to_image_homography: np.ndarray
    marker_to_rect_homography: np.ndarray
    rect_to_marker_homography: np.ndarray


def read_image(path: Path):
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def resize_for_processing(image, max_width: int) -> Tuple[np.ndarray, float]:
    if max_width <= 0 or image.shape[1] <= max_width:
        return image.copy(), 1.0

    scale = max_width / image.shape[1]
    height = round(image.shape[0] * scale)
    resized = cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)
    return resized, scale


def transform_points(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    points_array = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    transformed = cv2.perspectiveTransform(points_array, homography)
    return transformed.reshape(-1, 2)


def marker_relative_to_image_float(
    marker: MarkerObservation,
    point: Tuple[float, float],
) -> Tuple[float, float]:
    transformed = transform_points(marker.marker_to_image_homography, np.array([point]))[0]
    return float(transformed[0]), float(transformed[1])


def image_point_to_marker_relative_float(
    marker: MarkerObservation,
    point: Tuple[float, float],
) -> Tuple[float, float]:
    transformed = transform_points(marker.image_to_marker_homography, np.array([point]))[0]
    return float(transformed[0]), float(transformed[1])


def get_aruco_dictionary(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is not available. Install opencv-contrib-python.")

    aruco = cv2.aruco
    if not hasattr(aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
    return aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))


def detect_markers(image, dictionary_name: str):
    aruco = cv2.aruco
    dictionary = get_aruco_dictionary(dictionary_name)

    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return detector.detectMarkers(image)[:2]

    parameters = (
        aruco.DetectorParameters_create()
        if hasattr(aruco, "DetectorParameters_create")
        else aruco.DetectorParameters()
    )
    corners, ids, _rejected = aruco.detectMarkers(image, dictionary, parameters=parameters)
    return corners, ids


def find_marker(
    image,
    dictionary_name: str,
    marker_id: Optional[int],
) -> MarkerObservation:
    corners_list, ids = detect_markers(image, dictionary_name)
    if ids is None or len(corners_list) == 0:
        raise RuntimeError(f"No ArUco marker was detected with {dictionary_name}.")

    flat_ids = ids.flatten().astype(int)
    selected_index = None
    if marker_id is not None:
        for index, detected_id in enumerate(flat_ids):
            if int(detected_id) == marker_id:
                selected_index = index
                break
        if selected_index is None:
            raise RuntimeError(
                f"Marker ID {marker_id} was not detected. Detected IDs: {flat_ids.tolist()}"
            )
    else:
        areas = [cv2.contourArea(corners[0].astype(np.float32)) for corners in corners_list]
        selected_index = int(np.argmax(areas))

    corners = corners_list[selected_index][0].astype(np.float64)
    top_left, top_right, _bottom_right, bottom_left = corners
    center = corners.mean(axis=0)
    x_axis = top_right - top_left
    y_axis = bottom_left - top_left
    side_length = (np.linalg.norm(x_axis) + np.linalg.norm(y_axis)) / 2.0
    if side_length <= 0:
        raise RuntimeError("Detected marker has invalid side length.")

    marker_to_image_homography = cv2.getPerspectiveTransform(
        MARKER_PLANE_CORNERS.astype(np.float32),
        corners.astype(np.float32),
    ).astype(np.float64)
    image_to_marker_homography = cv2.getPerspectiveTransform(
        corners.astype(np.float32),
        MARKER_PLANE_CORNERS.astype(np.float32),
    ).astype(np.float64)

    return MarkerObservation(
        marker_id=int(flat_ids[selected_index]),
        corners=corners,
        center=center,
        x_axis=x_axis,
        y_axis=y_axis,
        side_length=float(side_length),
        marker_to_image_homography=marker_to_image_homography,
        image_to_marker_homography=image_to_marker_homography,
    )


def register_meter_from_marker(
    marker: MarkerObservation,
    meter_circle: Circle,
    dictionary_name: str,
    max_width: int,
    registration_image: Path,
    work_shape: Optional[Tuple[int, int]] = None,
) -> MarkerMeterRegistration:
    center_rel_x, center_rel_y = point_to_marker_relative(
        marker,
        (meter_circle.x, meter_circle.y),
    )
    radius_rel = marker_plane_radius_from_circle(marker, meter_circle)

    work_height = work_shape[0] if work_shape is not None else None
    work_width = work_shape[1] if work_shape is not None else None

    return MarkerMeterRegistration(
        dictionary=dictionary_name,
        marker_id=marker.marker_id,
        center_rel_x=float(center_rel_x),
        center_rel_y=float(center_rel_y),
        radius_rel=float(radius_rel),
        max_width=max_width,
        registration_image=str(registration_image),
        registration_circle_x=meter_circle.x,
        registration_circle_y=meter_circle.y,
        registration_circle_radius=meter_circle.radius,
        registration_work_width=work_width,
        registration_work_height=work_height,
    )


def estimate_meter_circle(
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
) -> Circle:
    center_rel = (registration.center_rel_x, registration.center_rel_y)
    center = np.array(marker_relative_to_image_float(marker, center_rel), dtype=np.float64)
    radius_rel = registration.radius_rel
    radius_points = np.array(
        [
            [registration.center_rel_x + radius_rel, registration.center_rel_y],
            [registration.center_rel_x - radius_rel, registration.center_rel_y],
            [registration.center_rel_x, registration.center_rel_y + radius_rel],
            [registration.center_rel_x, registration.center_rel_y - radius_rel],
        ],
        dtype=np.float64,
    )
    projected_radius_points = transform_points(marker.marker_to_image_homography, radius_points)
    distances = np.linalg.norm(projected_radius_points - center, axis=1)
    radius = float(np.mean(distances))
    return Circle(round(float(center[0])), round(float(center[1])), round(radius), 0.0)


def fallback_scale_from_registration(
    registration: MarkerMeterRegistration,
    work_shape: Tuple[int, int],
) -> float:
    base_width = registration.registration_work_width or registration.max_width
    if base_width <= 0:
        return 1.0
    return work_shape[1] / base_width


def fallback_meter_circle_from_registration(
    registration: MarkerMeterRegistration,
    work_shape: Tuple[int, int],
) -> Circle:
    scale = fallback_scale_from_registration(registration, work_shape)
    return Circle(
        round(registration.registration_circle_x * scale),
        round(registration.registration_circle_y * scale),
        max(1, round(registration.registration_circle_radius * scale)),
        0.0,
    )


def validate_projected_meter_circle(image_shape: Tuple[int, int, int], circle: Circle) -> None:
    height, width = image_shape[:2]
    if circle.radius <= 0:
        raise ValueError("Projected meter radius must be greater than 0.")
    if not (0 <= circle.x < width and 0 <= circle.y < height):
        raise ValueError(
            f"Projected meter center ({circle.x}, {circle.y}) is outside the image "
            f"size ({width}, {height})."
        )


def point_to_marker_relative(marker: MarkerObservation, point: Tuple[int, int]) -> Tuple[float, float]:
    return image_point_to_marker_relative_float(marker, point)


def marker_relative_to_point(
    marker: MarkerObservation,
    rel_x: Optional[float],
    rel_y: Optional[float],
) -> Optional[Tuple[int, int]]:
    if rel_x is None or rel_y is None:
        return None
    point = marker_relative_to_image_float(marker, (rel_x, rel_y))
    return round(point[0]), round(point[1])


def marker_plane_radius_from_circle(marker: MarkerObservation, circle: Circle) -> float:
    center = np.array(point_to_marker_relative(marker, (circle.x, circle.y)), dtype=np.float64)
    edge_points = [
        (circle.x + circle.radius, circle.y),
        (circle.x - circle.radius, circle.y),
        (circle.x, circle.y + circle.radius),
        (circle.x, circle.y - circle.radius),
    ]
    edge_rel_points = np.array(
        [point_to_marker_relative(marker, point) for point in edge_points],
        dtype=np.float64,
    )
    distances = np.linalg.norm(edge_rel_points - center, axis=1)
    return float(np.mean(distances))


def create_meter_rectification(
    image,
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
    margin: float = 1.25,
) -> MeterRectification:
    rect_radius = max(90, round(registration.radius_rel * marker.side_length))
    rect_center = max(rect_radius + 16, round(rect_radius * margin))
    size = rect_center * 2 + 1
    plane_scale = rect_radius / registration.radius_rel

    rect_to_marker = np.array(
        [
            [1.0 / plane_scale, 0.0, registration.center_rel_x - rect_center / plane_scale],
            [0.0, 1.0 / plane_scale, registration.center_rel_y - rect_center / plane_scale],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    marker_to_rect = np.linalg.inv(rect_to_marker)
    rect_to_image = marker.marker_to_image_homography @ rect_to_marker
    image_to_rect = np.linalg.inv(rect_to_image)
    rectified = cv2.warpPerspective(
        image,
        image_to_rect,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    return MeterRectification(
        image=rectified,
        circle=Circle(rect_center, rect_center, rect_radius, 0.0),
        image_to_rect_homography=image_to_rect,
        rect_to_image_homography=rect_to_image,
        marker_to_rect_homography=marker_to_rect,
        rect_to_marker_homography=rect_to_marker,
    )


def marker_relative_to_rectified_point(
    rectification: MeterRectification,
    rel_x: Optional[float],
    rel_y: Optional[float],
) -> Optional[Tuple[int, int]]:
    if rel_x is None or rel_y is None:
        return None
    point = transform_points(rectification.marker_to_rect_homography, np.array([[rel_x, rel_y]]))[0]
    return round(float(point[0])), round(float(point[1]))


def detect_needle_with_homography(
    image,
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
) -> Tuple[Needle, MeterRectification, Needle]:
    rectification = create_meter_rectification(image, marker, registration)
    validate_circle(rectification.image.shape, rectification.circle)
    rectified_needle = detect_needle_with_preset_circle(rectification.image, rectification.circle)
    mapped_points = transform_points(
        rectification.rect_to_image_homography,
        np.array([rectified_needle.center, rectified_needle.tip], dtype=np.float64),
    )
    mapped_center = (round(float(mapped_points[0][0])), round(float(mapped_points[0][1])))
    mapped_tip = (round(float(mapped_points[1][0])), round(float(mapped_points[1][1])))
    mapped_image_angle = image_angle_deg(mapped_center, mapped_tip)
    mapped_needle = Needle(
        center=mapped_center,
        tip=mapped_tip,
        image_angle_deg=mapped_image_angle,
        cartesian_angle_deg=(360.0 - mapped_image_angle) % 360.0,
        line_score=rectified_needle.line_score,
        line_threshold=rectified_needle.line_threshold,
    )
    return mapped_needle, rectification, rectified_needle


def image_angle_deg(center: Tuple[int, int], point: Tuple[int, int]) -> float:
    dx = point[0] - center[0]
    dy = point[1] - center[1]
    return float(np.degrees(np.arctan2(dy, dx)) % 360.0)


def interpolate_needle_value(
    center: Tuple[int, int],
    needle_tip: Tuple[int, int],
    min_point: Tuple[int, int],
    max_point: Tuple[int, int],
    min_value: float,
    max_value: float,
    direction: str,
) -> Tuple[float, float]:
    min_angle = image_angle_deg(center, min_point)
    max_angle = image_angle_deg(center, max_point)
    needle_angle = image_angle_deg(center, needle_tip)

    if direction == "counterclockwise":
        sweep = (min_angle - max_angle) % 360.0
        position = (min_angle - needle_angle) % 360.0
    else:
        sweep = (max_angle - min_angle) % 360.0
        position = (needle_angle - min_angle) % 360.0

    if sweep <= 0:
        raise ValueError("Scale sweep angle must be greater than 0.")

    fraction = max(0.0, min(1.0, position / sweep))
    value = min_value + (max_value - min_value) * fraction
    return value, fraction


def estimate_scale_points(
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]]]:
    min_point = marker_relative_to_point(
        marker,
        registration.scale_min_rel_x,
        registration.scale_min_rel_y,
    )
    max_point = marker_relative_to_point(
        marker,
        registration.scale_max_rel_x,
        registration.scale_max_rel_y,
    )
    return min_point, max_point


def fallback_scale_points_from_registration(
    registration: MarkerMeterRegistration,
    work_shape: Tuple[int, int],
) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]]]:
    scale = fallback_scale_from_registration(registration, work_shape)
    min_point = None
    max_point = None

    if (
        registration.registration_scale_min_x is not None
        and registration.registration_scale_min_y is not None
    ):
        min_point = (
            round(registration.registration_scale_min_x * scale),
            round(registration.registration_scale_min_y * scale),
        )

    if (
        registration.registration_scale_max_x is not None
        and registration.registration_scale_max_y is not None
    ):
        max_point = (
            round(registration.registration_scale_max_x * scale),
            round(registration.registration_scale_max_y * scale),
        )

    return min_point, max_point


def auto_scale_points_from_circle(circle: Circle) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    # The target gauge has the minimum tick at lower-left and maximum tick at lower-right.
    tick_radius = circle.radius
    min_angle = np.deg2rad(135.0)
    max_angle = np.deg2rad(45.0)
    min_point = (
        round(circle.x + tick_radius * np.cos(min_angle)),
        round(circle.y + tick_radius * np.sin(min_angle)),
    )
    max_point = (
        round(circle.x + tick_radius * np.cos(max_angle)),
        round(circle.y + tick_radius * np.sin(max_angle)),
    )
    return min_point, max_point


def project_point_to_circle_boundary(
    circle: Circle,
    point: Optional[Tuple[int, int]],
) -> Optional[Tuple[int, int]]:
    if point is None:
        return None
    dx = float(point[0] - circle.x)
    dy = float(point[1] - circle.y)
    distance = float(np.hypot(dx, dy))
    radius = max(1.0, float(circle.radius))
    if distance <= 1e-9:
        return round(circle.x + radius), round(circle.y)
    return (
        round(circle.x + dx * radius / distance),
        round(circle.y + dy * radius / distance),
    )


def marker_scale_point_on_meter_boundary(
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
    rel_x: Optional[float],
    rel_y: Optional[float],
) -> Optional[Tuple[int, int]]:
    if rel_x is None or rel_y is None:
        return None
    dx = float(rel_x - registration.center_rel_x)
    dy = float(rel_y - registration.center_rel_y)
    distance = float(np.hypot(dx, dy))
    radius = max(1e-9, float(registration.radius_rel))
    if distance <= 1e-9:
        boundary_rel_x = registration.center_rel_x + radius
        boundary_rel_y = registration.center_rel_y
    else:
        boundary_rel_x = registration.center_rel_x + dx * radius / distance
        boundary_rel_y = registration.center_rel_y + dy * radius / distance
    return marker_relative_to_point(marker, boundary_rel_x, boundary_rel_y)


def clamp_text_origin(view, origin: Tuple[int, int], text: str, font_scale: float, thickness: int) -> Tuple[int, int]:
    height, width = view.shape[:2]
    (text_width, text_height), baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        thickness,
    )
    x = max(0, min(width - text_width - 1, origin[0]))
    y = max(text_height + baseline, min(height - baseline - 1, origin[1]))
    return x, y


def text_origin_near_point(
    view,
    text: str,
    point: Tuple[int, int],
    position: str,
    font_scale: float,
    thickness: int,
) -> Tuple[int, int]:
    (text_width, _text_height), _baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        thickness,
    )
    offset = 10
    if position == "lower_left":
        origin = (point[0] - text_width - offset, point[1] + 22)
    elif position == "lower_right":
        origin = (point[0] + offset, point[1] + 22)
    else:
        origin = (point[0] + offset, point[1] + 22)
    return clamp_text_origin(view, origin, text, font_scale, thickness)


def draw_scale_points(
    view,
    min_point: Optional[Tuple[int, int]],
    max_point: Optional[Tuple[int, int]],
    draw_text: bool = True,
    line_thickness: int = DEFAULT_DRAW_LINE_THICKNESS,
    point_radius: int = DEFAULT_DRAW_POINT_RADIUS,
) -> None:
    thickness = max(1, int(line_thickness))
    radius = max(1, int(point_radius))
    if min_point is not None:
        cv2.circle(view, min_point, radius, (0, 255, 255), -1, cv2.LINE_AA)
        if draw_text:
            cv2.putText(
                view,
                "min",
                text_origin_near_point(view, "min", min_point, "lower_left", 0.6, thickness),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 180, 255),
                thickness,
                cv2.LINE_AA,
            )
    if max_point is not None:
        cv2.circle(view, max_point, radius, (0, 165, 255), -1, cv2.LINE_AA)
        if draw_text:
            cv2.putText(
                view,
                "max",
                text_origin_near_point(view, "max", max_point, "lower_right", 0.6, thickness),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 120, 255),
                thickness,
                cv2.LINE_AA,
            )


def has_scale_registration(registration: MarkerMeterRegistration) -> bool:
    return all(
        value is not None
        for value in [
            registration.scale_min_value,
            registration.scale_max_value,
            registration.scale_min_rel_x,
            registration.scale_min_rel_y,
            registration.scale_max_rel_x,
            registration.scale_max_rel_y,
        ]
    )


def load_registration(config_path: Path) -> MarkerMeterRegistration:
    data = json.loads(config_path.read_text(encoding="utf-8"))
    return MarkerMeterRegistration(**data)


def save_registration(config_path: Path, registration: MarkerMeterRegistration) -> None:
    config_path.write_text(
        json.dumps(asdict(registration), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def scale_circle(circle: Circle, scale: float) -> Circle:
    return Circle(
        round(circle.x * scale),
        round(circle.y * scale),
        round(circle.radius * scale),
        circle.score,
    )


def scale_point(point: Tuple[float, float], scale: float) -> Tuple[int, int]:
    return round(point[0] * scale), round(point[1] * scale)


def draw_homography_detection(
    image,
    marker: MarkerObservation,
    registration: MarkerMeterRegistration,
    needle: Needle,
    draw_text: bool = True,
    line_thickness: int = DEFAULT_DRAW_LINE_THICKNESS,
    draw_tip_point: bool = DEFAULT_DRAW_NEEDLE_TIP_POINT,
) -> np.ndarray:
    annotated = image.copy()
    thickness = max(1, int(line_thickness))
    angles = np.linspace(0.0, 2.0 * np.pi, 120, endpoint=False)
    circle_points = np.column_stack(
        [
            registration.center_rel_x + registration.radius_rel * np.cos(angles),
            registration.center_rel_y + registration.radius_rel * np.sin(angles),
        ]
    )
    projected = transform_points(marker.marker_to_image_homography, circle_points)
    projected_int = np.round(projected).astype(np.int32)
    cv2.polylines(annotated, [projected_int], True, (0, 180, 0), thickness, cv2.LINE_AA)

    center = marker_relative_to_point(
        marker,
        registration.center_rel_x,
        registration.center_rel_y,
    )
    cv2.circle(annotated, center, 5, (255, 0, 0), -1)
    cv2.line(annotated, needle.center, needle.tip, (0, 0, 255), thickness, cv2.LINE_AA)
    if draw_tip_point:
        cv2.circle(annotated, needle.tip, 7, (0, 0, 255), -1)
    if draw_text:
        cv2.putText(
            annotated,
            f"needle {needle.image_angle_deg:.1f} deg",
            (center[0] - 90, max(20, center[1] - 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            thickness,
            cv2.LINE_AA,
        )
    return annotated


def draw_marker(
    view,
    marker: MarkerObservation,
    draw_text: bool = True,
    line_thickness: int = DEFAULT_DRAW_LINE_THICKNESS,
) -> None:
    thickness = max(1, int(line_thickness))
    corners = marker.corners.astype(np.int32)
    cv2.polylines(view, [corners], True, (255, 0, 255), thickness, cv2.LINE_AA)
    center = tuple(np.round(marker.center).astype(int))
    bottom_left = tuple(corners[3])
    x_end = tuple(np.round(marker.center + marker.x_axis * 0.5).astype(int))
    y_end = tuple(np.round(marker.center + marker.y_axis * 0.5).astype(int))
    cv2.circle(view, center, 5, (255, 0, 255), -1)
    cv2.line(view, center, x_end, (0, 0, 255), thickness, cv2.LINE_AA)
    cv2.line(view, center, y_end, (255, 0, 0), thickness, cv2.LINE_AA)
    if draw_text:
        cv2.putText(
            view,
            f"marker id {marker.marker_id}",
            text_origin_near_point(
                view,
                f"marker id {marker.marker_id}",
                bottom_left,
                "lower_left",
                0.65,
                thickness,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 0, 255),
            thickness,
            cv2.LINE_AA,
        )


def default_output_path(image_path: Path) -> Path:
    return DEFAULT_OUTPUT_DIR / f"{image_path.stem}_ar_marker_needle_detected.jpg"


def create_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detect a meter needle by estimating the meter circle from an ArUco marker."
    )
    subparsers = parser.add_subparsers(dest="command")

    register = subparsers.add_parser(
        "register",
        help="Register the meter circle position relative to an ArUco marker.",
    )
    add_common_arguments(register)
    register.add_argument("--dictionary", default="DICT_4X4_50")
    register.add_argument("--marker-id", type=int, default=0)
    register.add_argument("--center-x", type=float)
    register.add_argument("--center-y", type=float)
    register.add_argument("--radius", type=float)
    register.add_argument(
        "--auto-circle",
        action="store_true",
        help="Estimate the registration meter circle automatically.",
    )
    register.add_argument("--scale-min-x", type=float)
    register.add_argument("--scale-min-y", type=float)
    register.add_argument("--scale-max-x", type=float)
    register.add_argument("--scale-max-y", type=float)
    register.add_argument("--scale-min-value", type=float, default=0.0)
    register.add_argument("--scale-max-value", type=float, default=0.1)
    register.add_argument(
        "--scale-direction",
        choices=["clockwise", "counterclockwise"],
        default="clockwise",
        help="Direction from the minimum tick to the maximum tick in image coordinates.",
    )
    register.add_argument(
        "--auto-scale",
        action="store_true",
        help="Place the minimum and maximum ticks automatically from the registered circle.",
    )

    detect = subparsers.add_parser(
        "detect",
        help="Detect the needle using a registered marker-to-meter relationship.",
    )
    add_common_arguments(detect)
    detect.add_argument("--output", type=Path)
    detect.add_argument(
        "--no-text",
        action="store_true",
        help="Do not draw text labels on the annotated output image.",
    )

    return parser


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE_PATH)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--max-width",
        type=int,
        default=1280,
        help="Resize wide images to this width before marker and needle detection.",
    )


def command_register(args: argparse.Namespace) -> int:
    image = read_image(args.image)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {args.image}")

    work_image, scale = resize_for_processing(image, args.max_width)
    marker = find_marker(work_image, args.dictionary, args.marker_id)

    manual_circle_complete = (
        args.center_x is not None and args.center_y is not None and args.radius is not None
    )
    if manual_circle_complete:
        meter_circle = scale_circle(
            Circle(round(args.center_x), round(args.center_y), round(args.radius), 0.0),
            scale,
        )
    elif args.auto_circle or not manual_circle_complete:
        gray = cv2.cvtColor(work_image, cv2.COLOR_BGR2GRAY)
        meter_circle = find_meter_circle(gray)
    else:
        raise ValueError("Specify --center-x, --center-y and --radius, or use --auto-circle.")

    validate_circle(work_image.shape, meter_circle)
    registration = register_meter_from_marker(
        marker,
        meter_circle,
        args.dictionary,
        args.max_width,
        args.image,
        work_image.shape[:2],
    )

    scale_point_args = [
        args.scale_min_x,
        args.scale_min_y,
        args.scale_max_x,
        args.scale_max_y,
    ]
    manual_scale_complete = all(value is not None for value in scale_point_args)
    if any(value is not None for value in scale_point_args) and not manual_scale_complete:
        raise ValueError(
            "Specify all of --scale-min-x, --scale-min-y, --scale-max-x and --scale-max-y."
        )

    scale_min_point = None
    scale_max_point = None
    if manual_scale_complete:
        scale_min_point = project_point_to_circle_boundary(
            meter_circle,
            scale_point((args.scale_min_x, args.scale_min_y), scale),
        )
        scale_max_point = project_point_to_circle_boundary(
            meter_circle,
            scale_point((args.scale_max_x, args.scale_max_y), scale),
        )
    elif args.auto_scale:
        scale_min_point, scale_max_point = auto_scale_points_from_circle(meter_circle)

    if scale_min_point is not None and scale_max_point is not None:
        min_rel_x, min_rel_y = point_to_marker_relative(marker, scale_min_point)
        max_rel_x, max_rel_y = point_to_marker_relative(marker, scale_max_point)
        registration = replace(
            registration,
            scale_min_value=args.scale_min_value,
            scale_max_value=args.scale_max_value,
            scale_min_rel_x=min_rel_x,
            scale_min_rel_y=min_rel_y,
            scale_max_rel_x=max_rel_x,
            scale_max_rel_y=max_rel_y,
            scale_direction=args.scale_direction,
            registration_scale_min_x=scale_min_point[0],
            registration_scale_min_y=scale_min_point[1],
            registration_scale_max_x=scale_max_point[0],
            registration_scale_max_y=scale_max_point[1],
        )
    save_registration(args.config, registration)

    print(f"registered_config: {args.config}")
    print(f"marker: id={marker.marker_id}, side={marker.side_length:.2f}")
    print(
        "meter_circle_work_image: "
        f"center=({meter_circle.x}, {meter_circle.y}), radius={meter_circle.radius}"
    )
    print(
        "relative_meter: "
        f"center=({registration.center_rel_x:.6f}, {registration.center_rel_y:.6f}), "
        f"radius={registration.radius_rel:.6f}"
    )
    if scale_min_point is not None and scale_max_point is not None:
        print(
            "scale: "
            f"min={scale_min_point} value={registration.scale_min_value}, "
            f"max={scale_max_point} value={registration.scale_max_value}, "
            f"direction={registration.scale_direction}"
        )
    return 0


def command_detect(args: argparse.Namespace) -> int:
    registration = load_registration(args.config)
    image = read_image(args.image)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {args.image}")

    max_width = args.max_width if args.max_width is not None else registration.max_width
    work_image, _scale = resize_for_processing(image, max_width)
    draw_text = not args.no_text
    marker = None
    marker_error = None
    rectification = None
    rectified_needle = None
    try:
        marker = find_marker(work_image, registration.dictionary, registration.marker_id)
        meter_circle = estimate_meter_circle(marker, registration)
    except RuntimeError as exc:
        marker_error = exc
        meter_circle = fallback_meter_circle_from_registration(registration, work_image.shape[:2])
    if marker is not None:
        validate_projected_meter_circle(work_image.shape, meter_circle)
    else:
        validate_circle(work_image.shape, meter_circle)

    if marker is not None:
        needle, rectification, rectified_needle = detect_needle_with_homography(
            work_image,
            marker,
            registration,
        )
        annotated = draw_homography_detection(
            work_image,
            marker,
            registration,
            needle,
            draw_text=draw_text,
        )
        draw_marker(
            annotated,
            marker,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
        )
    else:
        needle = detect_needle_with_preset_circle(work_image, meter_circle)
        annotated = draw_detection(
            work_image,
            meter_circle,
            needle,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
        )
        if draw_text:
            cv2.putText(
                annotated,
                "marker not detected: using registered circle",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

    needle_value = None
    scale_fraction = None
    min_display_point = None
    max_display_point = None
    if marker is not None:
        min_point, max_point = estimate_scale_points(marker, registration)
        min_display_point = marker_scale_point_on_meter_boundary(
            marker,
            registration,
            registration.scale_min_rel_x,
            registration.scale_min_rel_y,
        )
        max_display_point = marker_scale_point_on_meter_boundary(
            marker,
            registration,
            registration.scale_max_rel_x,
            registration.scale_max_rel_y,
        )
        min_value_point = marker_relative_to_rectified_point(
            rectification,
            registration.scale_min_rel_x,
            registration.scale_min_rel_y,
        )
        max_value_point = marker_relative_to_rectified_point(
            rectification,
            registration.scale_max_rel_x,
            registration.scale_max_rel_y,
        )
        value_center = rectification.circle
        value_tip = rectified_needle.tip
    else:
        min_point, max_point = fallback_scale_points_from_registration(
            registration,
            work_image.shape[:2],
        )
        min_value_point = min_point
        max_value_point = max_point
        value_center = meter_circle
        value_tip = needle.tip
        min_display_point = project_point_to_circle_boundary(meter_circle, min_point)
        max_display_point = project_point_to_circle_boundary(meter_circle, max_point)
    if (
        has_scale_registration(registration)
        and min_point is not None
        and max_point is not None
        and min_value_point is not None
        and max_value_point is not None
    ):
        needle_value, scale_fraction = interpolate_needle_value(
            (value_center.x, value_center.y),
            value_tip,
            min_value_point,
            max_value_point,
            float(registration.scale_min_value),
            float(registration.scale_max_value),
            registration.scale_direction,
        )
        draw_scale_points(
            annotated,
            min_display_point,
            max_display_point,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
            point_radius=DEFAULT_DRAW_POINT_RADIUS,
        )

    output_path = args.output or default_output_path(args.image)
    if not output_path.is_absolute():
        output_path = BASE_DIR / output_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), annotated):
        raise RuntimeError(f"Could not write output image: {output_path}")

    print(f"image: {args.image}")
    print(f"config: {args.config}")
    print(
        "estimated_meter_circle: "
        f"center=({meter_circle.x}, {meter_circle.y}), radius={meter_circle.radius}"
    )
    if marker is None:
        print(f"marker: not detected; used registered circle only ({marker_error})")
    else:
        print(f"marker: detected id={marker.marker_id}")
    print(
        "needle: "
        f"center={needle.center}, tip={needle.tip}, "
        f"image_angle={needle.image_angle_deg:.2f} deg, "
        f"cartesian_angle={needle.cartesian_angle_deg:.2f} deg"
    )
    if needle_value is not None and scale_fraction is not None:
        print(f"needle_value: {needle_value:.6f}")
        print(f"scale_fraction: {scale_fraction:.6f}")
    print(f"output: {output_path}")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args_list = list(sys.argv[1:] if argv is None else argv)
    if not args_list:
        args_list = ["detect"]

    parser = create_parser()
    args = parser.parse_args(args_list)
    if args.command == "register":
        return command_register(args)
    if args.command == "detect":
        return command_detect(args)
    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
