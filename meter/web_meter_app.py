from __future__ import annotations

import base64
import json
import os
import sys
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, request

from detect_meter_needle import Circle, draw_detection, find_meter_circle
from detect_meter_needle_ar_marker import (
    DEFAULT_CONFIG_PATH,
    DEFAULT_DRAW_LINE_THICKNESS,
    DEFAULT_DRAW_POINT_RADIUS,
    DEFAULT_IMAGE_PATH,
    MarkerMeterRegistration,
    MarkerObservation,
    auto_scale_points_from_circle,
    detect_needle_with_homography,
    draw_homography_detection,
    draw_marker,
    draw_scale_points,
    estimate_meter_circle,
    estimate_scale_points,
    fallback_scale_points_from_registration,
    find_marker,
    has_scale_registration,
    interpolate_needle_value,
    load_registration,
    marker_relative_to_rectified_point,
    marker_scale_point_on_meter_boundary,
    point_to_marker_relative,
    project_point_to_circle_boundary,
    read_image,
    register_meter_from_marker,
    resize_for_processing,
    save_registration,
    validate_projected_meter_circle,
)
from detect_meter_needle_preset_circle import detect_needle_with_preset_circle, validate_circle


BASE_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = BASE_DIR / "output"
REGISTER_CAPTURE_PATH = OUTPUT_DIR / "web_register_capture.jpg"
REGISTER_UPLOAD_PATH = OUTPUT_DIR / "web_register_upload.jpg"

DICTIONARIES = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_6X6_50",
    "DICT_7X7_50",
]

app = Flask(__name__)
state_lock = threading.Lock()
selected_config_path = DEFAULT_CONFIG_PATH


class CameraManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cap = None
        self._index: Optional[int] = None

    def read(self, camera_index: int = 0):
        with self._lock:
            if self._cap is None or self._index != int(camera_index):
                self.release()
                backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
                self._cap = cv2.VideoCapture(int(camera_index), backend)
                self._index = int(camera_index)
            if not self._cap.isOpened():
                self.release()
                raise RuntimeError(
                    f"サーバー側USBカメラを開けませんでした。camera_index={camera_index}"
                )
            frame = None
            ok = False
            for _ in range(3):
                ok, frame = self._cap.read()
            if not ok or frame is None:
                raise RuntimeError("サーバー側USBカメラから画像を取得できませんでした。")
            return frame

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            self._index = None


camera_manager = CameraManager()


@dataclass
class RegisterState:
    source_image: Optional[np.ndarray] = None
    work_image: Optional[np.ndarray] = None
    work_scale: float = 1.0
    max_width: int = 1280
    image_path: Path = DEFAULT_IMAGE_PATH
    circle: Optional[Circle] = None
    marker: Optional[MarkerObservation] = None
    scale_min_point: Optional[Tuple[int, int]] = None
    scale_max_point: Optional[Tuple[int, int]] = None
    dictionary: str = "DICT_4X4_50"
    marker_id: int = 0
    scale_min_value: float = 0.0
    scale_max_value: float = 0.1
    scale_direction: str = "clockwise"


register_state = RegisterState()


def json_error(message: str, status: int = 400):
    response = jsonify({"ok": False, "error": message})
    response.status_code = status
    return response


def image_to_data_url(image, quality: int = 88) -> str:
    ok, buffer = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise RuntimeError("画像をJPEGへ変換できませんでした。")
    encoded = base64.b64encode(buffer).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}"


def decode_image_bytes(data: bytes):
    image_array = np.frombuffer(data, dtype=np.uint8)
    if image_array.size == 0:
        return None
    return cv2.imdecode(image_array, cv2.IMREAD_COLOR)


def capture_camera_frame(camera_index: int = 0):
    return camera_manager.read(camera_index)


def default_circle_for_image(image) -> Circle:
    height, width = image.shape[:2]
    radius = max(1, min(width, height) // 4)
    return Circle(width // 2, height // 2, radius, 0.0)


def selected_marker_id(settings: Dict[str, Any]) -> Optional[int]:
    marker_id = int(settings.get("marker_id", 0))
    return None if marker_id < 0 else marker_id


def point_from_json(value: Any) -> Optional[Tuple[int, int]]:
    if value is None:
        return None
    if isinstance(value, dict):
        if value.get("x") is None or value.get("y") is None:
            return None
        return round(float(value["x"])), round(float(value["y"]))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        if value[0] is None or value[1] is None:
            return None
        return round(float(value[0])), round(float(value[1]))
    return None


def circle_from_settings(settings: Dict[str, Any], fallback: Circle) -> Circle:
    circle_data = settings.get("circle") or {}
    return Circle(
        round(float(circle_data.get("x", fallback.x))),
        round(float(circle_data.get("y", fallback.y))),
        max(1, round(float(circle_data.get("radius", fallback.radius)))),
        0.0,
    )


def settings_from_json() -> Dict[str, Any]:
    data = request.get_json(silent=True) or {}
    settings = data.get("settings", data)
    return settings if isinstance(settings, dict) else {}


def settings_from_form() -> Dict[str, Any]:
    raw_settings = request.form.get("settings", "{}")
    try:
        settings = json.loads(raw_settings)
    except json.JSONDecodeError:
        settings = {}
    return settings if isinstance(settings, dict) else {}


def marker_payload(marker: Optional[MarkerObservation]) -> Optional[Dict[str, Any]]:
    if marker is None:
        return None
    return {
        "marker_id": marker.marker_id,
        "center": [round(float(marker.center[0])), round(float(marker.center[1]))],
        "side_length": float(marker.side_length),
        "corners": np.round(marker.corners).astype(int).tolist(),
    }


def circle_payload(circle: Circle) -> Dict[str, Any]:
    return {"x": int(circle.x), "y": int(circle.y), "radius": int(circle.radius)}


def point_payload(point: Optional[Tuple[int, int]]) -> Optional[list[int]]:
    if point is None:
        return None
    return [int(point[0]), int(point[1])]


def config_settings_payload(config_path: Optional[Path] = None) -> Optional[Dict[str, Any]]:
    path = config_path or current_config_path()
    try:
        if not path.exists():
            return None
        registration = load_registration(path)
    except Exception:
        return None

    return {
        "config_name": path.name,
        "dictionary": registration.dictionary,
        "marker_id": int(registration.marker_id),
        "max_width": int(registration.max_width),
        "registration_image": registration.registration_image,
        "registration_circle": {
            "x": int(registration.registration_circle_x),
            "y": int(registration.registration_circle_y),
            "radius": int(registration.registration_circle_radius),
        },
        "registration_work_size": {
            "width": registration.registration_work_width,
            "height": registration.registration_work_height,
        },
        "scale_min_value": registration.scale_min_value,
        "scale_max_value": registration.scale_max_value,
        "scale_direction": registration.scale_direction,
        "scale_min_point": point_payload(
            (
                registration.registration_scale_min_x,
                registration.registration_scale_min_y,
            )
            if registration.registration_scale_min_x is not None
            and registration.registration_scale_min_y is not None
            else None
        ),
        "scale_max_point": point_payload(
            (
                registration.registration_scale_max_x,
                registration.registration_scale_max_y,
            )
            if registration.registration_scale_max_x is not None
            and registration.registration_scale_max_y is not None
            else None
        ),
        "raw": asdict(registration),
    }


def config_files_payload() -> list[Dict[str, Any]]:
    base_dir = BASE_DIR.resolve()
    selected = current_config_path().resolve()
    payload = []
    for path in sorted(BASE_DIR.glob("*.json"), key=lambda item: item.name.lower()):
        resolved = path.resolve()
        if resolved.parent != base_dir:
            continue
        payload.append(
            {
                "name": path.name,
                "path": path.name,
                "selected": resolved == selected,
            }
        )
    return payload


def current_config_path() -> Path:
    return selected_config_path


def config_path_from_name(config_name: str) -> Path:
    if not config_name:
        raise ValueError("JSONファイル名が指定されていません。")
    path = (BASE_DIR / config_name).resolve()
    if path.parent != BASE_DIR.resolve() or path.suffix.lower() != ".json":
        raise ValueError("プロジェクト直下のJSONファイルだけを選択できます。")
    if not path.exists():
        raise FileNotFoundError(f"JSONファイルが見つかりません: {config_name}")
    return path


def registration_image_path(registration: MarkerMeterRegistration) -> Path:
    candidates = []
    if registration.registration_image:
        candidate = Path(registration.registration_image)
        if not candidate.is_absolute():
            candidate = BASE_DIR / candidate
        candidates.append(candidate)
    candidates.append(DEFAULT_IMAGE_PATH)

    for candidate in candidates:
        if candidate.exists() and read_image(candidate) is not None:
            return candidate
    return DEFAULT_IMAGE_PATH


def load_register_state_from_config(config_path: Path) -> str:
    registration = load_registration(config_path)
    image_path = registration_image_path(registration)
    image = read_image(image_path)
    if image is None:
        raise FileNotFoundError(f"登録画像を読み込めませんでした: {image_path}")

    work_image, scale = resize_for_processing(image, registration.max_width)
    register_state.source_image = image
    register_state.work_image = work_image
    register_state.work_scale = scale
    register_state.max_width = registration.max_width
    register_state.image_path = image_path
    register_state.circle = fallback_meter_circle_from_registration_safe(
        registration,
        work_image.shape[:2],
    )
    register_state.marker = None
    register_state.scale_min_point, register_state.scale_max_point = (
        fallback_scale_points_from_registration(registration, work_image.shape[:2])
    )
    register_state.dictionary = registration.dictionary
    register_state.marker_id = registration.marker_id
    register_state.scale_min_value = (
        float(registration.scale_min_value)
        if registration.scale_min_value is not None
        else 0.0
    )
    register_state.scale_max_value = (
        float(registration.scale_max_value)
        if registration.scale_max_value is not None
        else 0.1
    )
    register_state.scale_direction = registration.scale_direction
    return f"登録JSONを読み込みました: {config_path.name}"


def fallback_meter_circle_from_registration_safe(
    registration: MarkerMeterRegistration,
    work_shape: Tuple[int, int],
) -> Circle:
    base_width = registration.registration_work_width or registration.max_width
    scale = work_shape[1] / base_width if base_width > 0 else 1.0
    return Circle(
        round(registration.registration_circle_x * scale),
        round(registration.registration_circle_y * scale),
        max(1, round(registration.registration_circle_radius * scale)),
        0.0,
    )


def ensure_register_image(max_width: Optional[int] = None) -> None:
    if register_state.source_image is None:
        if current_config_path().exists():
            load_register_state_from_config(current_config_path())
            if max_width is not None and int(max_width) != register_state.max_width:
                reprocess_register_image(int(max_width))
            return
        image = read_image(DEFAULT_IMAGE_PATH)
        if image is None:
            raise FileNotFoundError(f"初期画像を読み込めませんでした: {DEFAULT_IMAGE_PATH}")
        set_register_source(image, DEFAULT_IMAGE_PATH, max_width or register_state.max_width)
        return

    if max_width is not None and int(max_width) != register_state.max_width:
        reprocess_register_image(int(max_width))


def set_register_source(image, image_path: Path, max_width: int) -> None:
    work_image, scale = resize_for_processing(image, int(max_width))
    register_state.source_image = image
    register_state.work_image = work_image
    register_state.work_scale = scale
    register_state.max_width = int(max_width)
    register_state.image_path = image_path
    register_state.circle = default_circle_for_image(work_image)
    register_state.marker = None
    register_state.scale_min_point = None
    register_state.scale_max_point = None


def reprocess_register_image(max_width: int) -> None:
    if register_state.source_image is None or register_state.work_image is None:
        raise RuntimeError("登録用画像がありません。")
    old_scale = register_state.work_scale
    old_circle = register_state.circle or default_circle_for_image(register_state.work_image)
    old_min = register_state.scale_min_point
    old_max = register_state.scale_max_point

    work_image, scale = resize_for_processing(register_state.source_image, int(max_width))
    ratio = scale / old_scale if old_scale > 0 else 1.0
    register_state.work_image = work_image
    register_state.work_scale = scale
    register_state.max_width = int(max_width)
    register_state.circle = Circle(
        round(old_circle.x * ratio),
        round(old_circle.y * ratio),
        max(1, round(old_circle.radius * ratio)),
        0.0,
    )
    register_state.scale_min_point = scale_point_by_ratio(old_min, ratio)
    register_state.scale_max_point = scale_point_by_ratio(old_max, ratio)
    register_state.marker = None


def scale_point_by_ratio(point: Optional[Tuple[int, int]], ratio: float) -> Optional[Tuple[int, int]]:
    if point is None:
        return None
    return round(point[0] * ratio), round(point[1] * ratio)


def apply_register_settings(settings: Dict[str, Any]) -> None:
    ensure_register_image()
    if register_state.work_image is None:
        raise RuntimeError("登録用画像がありません。")
    fallback_circle = register_state.circle or default_circle_for_image(register_state.work_image)
    register_state.circle = circle_from_settings(settings, fallback_circle)
    register_state.scale_min_point = point_from_json(settings.get("scale_min_point"))
    register_state.scale_max_point = point_from_json(settings.get("scale_max_point"))
    register_state.dictionary = settings.get("dictionary", register_state.dictionary)
    register_state.marker_id = int(settings.get("marker_id", register_state.marker_id))
    register_state.max_width = int(settings.get("max_width", register_state.max_width))
    register_state.scale_min_value = float(
        settings.get("scale_min_value", register_state.scale_min_value)
    )
    register_state.scale_max_value = float(
        settings.get("scale_max_value", register_state.scale_max_value)
    )
    register_state.scale_direction = settings.get(
        "scale_direction",
        register_state.scale_direction,
    )


def render_register_preview(draw_text: bool = True):
    if register_state.work_image is None:
        raise RuntimeError("登録用画像がありません。")
    circle = register_state.circle or default_circle_for_image(register_state.work_image)
    preview = register_state.work_image.copy()
    cv2.circle(
        preview,
        (circle.x, circle.y),
        circle.radius,
        (0, 180, 0),
        DEFAULT_DRAW_LINE_THICKNESS,
    )
    cv2.circle(preview, (circle.x, circle.y), 5, (255, 0, 0), -1)

    if register_state.marker is not None:
        draw_marker(
            preview,
            register_state.marker,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
        )

    display_min = project_point_to_circle_boundary(circle, register_state.scale_min_point)
    display_max = project_point_to_circle_boundary(circle, register_state.scale_max_point)
    draw_scale_points(
        preview,
        display_min,
        display_max,
        draw_text=draw_text,
        line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
        point_radius=DEFAULT_DRAW_POINT_RADIUS,
    )
    return preview, display_min, display_max


def register_response(message: str = "", draw_text: bool = True) -> Dict[str, Any]:
    preview, display_min, display_max = render_register_preview(draw_text=draw_text)
    circle = register_state.circle or default_circle_for_image(register_state.work_image)
    return {
        "ok": True,
        "message": message,
        "image": image_to_data_url(preview),
        "image_width": int(register_state.work_image.shape[1]),
        "image_height": int(register_state.work_image.shape[0]),
        "source_width": int(register_state.source_image.shape[1]),
        "source_height": int(register_state.source_image.shape[0]),
        "work_scale": float(register_state.work_scale),
        "image_name": register_state.image_path.name,
        "circle": circle_payload(circle),
        "scale_min_point": point_payload(display_min),
        "scale_max_point": point_payload(display_max),
        "marker": marker_payload(register_state.marker),
        "dictionary": register_state.dictionary,
        "marker_id": register_state.marker_id,
        "max_width": register_state.max_width,
        "scale_min_value": register_state.scale_min_value,
        "scale_max_value": register_state.scale_max_value,
        "scale_direction": register_state.scale_direction,
        "config_name": current_config_path().name,
        "config_path": current_config_path().name,
        "config_files": config_files_payload(),
        "config_settings": config_settings_payload(),
    }


def detect_register_needle_preview(draw_text: bool = True):
    if register_state.work_image is None:
        raise RuntimeError("登録用画像がありません。")

    circle = register_state.circle or default_circle_for_image(register_state.work_image)
    validate_circle(register_state.work_image.shape, circle)
    needle = detect_needle_with_preset_circle(register_state.work_image, circle)
    annotated = draw_detection(
        register_state.work_image,
        circle,
        needle,
        draw_text=draw_text,
        line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
    )

    display_min = project_point_to_circle_boundary(circle, register_state.scale_min_point)
    display_max = project_point_to_circle_boundary(circle, register_state.scale_max_point)
    needle_value = None
    scale_fraction = None
    if display_min is not None and display_max is not None:
        needle_value, scale_fraction = interpolate_needle_value(
            (circle.x, circle.y),
            needle.tip,
            display_min,
            display_max,
            register_state.scale_min_value,
            register_state.scale_max_value,
            register_state.scale_direction,
        )

    if display_min is not None or display_max is not None:
        draw_scale_points(
            annotated,
            display_min,
            display_max,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
            point_radius=DEFAULT_DRAW_POINT_RADIUS,
        )

    return annotated, {
        "detection_ok": True,
        "message": "登録画像で針を検出しました。登録JSONは更新していません。",
        "circle": circle_payload(circle),
        "needle": {
            "center": point_payload(needle.center),
            "tip": point_payload(needle.tip),
            "image_angle_deg": float(needle.image_angle_deg),
            "cartesian_angle_deg": float(needle.cartesian_angle_deg),
        },
        "needle_value": None if needle_value is None else float(needle_value),
        "scale_fraction": None if scale_fraction is None else float(scale_fraction),
        "scale_min_point": point_payload(display_min),
        "scale_max_point": point_payload(display_max),
    }


def detect_registered_image(image, registration: MarkerMeterRegistration, draw_text: bool = True):
    work_image, _scale = resize_for_processing(image, registration.max_width)
    rectification = None
    rectified_needle = None

    try:
        marker = find_marker(work_image, registration.dictionary, registration.marker_id)
        meter_circle = estimate_meter_circle(marker, registration)
    except RuntimeError as exc:
        annotated = annotate_detection_error(work_image, "AR marker not detected")
        return annotated, {
            "detection_ok": False,
            "message": "ARマーカーが検出できないため、針検出は行いませんでした。",
            "marker_detected": False,
            "marker_error": str(exc),
            "marker": None,
            "circle": None,
            "needle": None,
            "needle_value": None,
            "scale_fraction": None,
            "scale_min_point": None,
            "scale_max_point": None,
        }

    validate_projected_meter_circle(work_image.shape, meter_circle)
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

    needle_value = None
    scale_fraction = None
    min_display_point = None
    max_display_point = None
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

    return annotated, {
        "detection_ok": True,
        "message": "検出しました。",
        "marker_detected": marker is not None,
        "marker_error": "",
        "marker": marker_payload(marker),
        "circle": circle_payload(meter_circle),
        "needle": {
            "center": point_payload(needle.center),
            "tip": point_payload(needle.tip),
            "image_angle_deg": float(needle.image_angle_deg),
            "cartesian_angle_deg": float(needle.cartesian_angle_deg),
        },
        "needle_value": None if needle_value is None else float(needle_value),
        "scale_fraction": None if scale_fraction is None else float(scale_fraction),
        "scale_min_point": point_payload(min_display_point),
        "scale_max_point": point_payload(max_display_point),
    }


def annotate_detection_error(image, message: str):
    annotated = image.copy()
    height, width = annotated.shape[:2]
    banner_height = 74
    overlay = annotated.copy()
    cv2.rectangle(overlay, (0, 0), (width, banner_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, annotated, 0.45, 0, annotated)
    cv2.putText(
        annotated,
        "detection unavailable",
        (18, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (0, 220, 255),
        2,
        cv2.LINE_AA,
    )
    safe_message = message.encode("ascii", errors="ignore").decode("ascii").strip()
    if safe_message:
        cv2.putText(
            annotated,
            safe_message[:100],
            (18, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated


@app.get("/")
def index():
    return render_template(
        "web_meter_app.html",
        dictionaries=DICTIONARIES,
        default_config_path=current_config_path().name,
    )


@app.get("/api/config/list")
def api_config_list():
    try:
        return jsonify(
            {
                "ok": True,
                "selected_config": current_config_path().name,
                "config_files": config_files_payload(),
                "config_settings": config_settings_payload(),
            }
        )
    except Exception as exc:
        return json_error(str(exc), 500)


@app.post("/api/config/select")
def api_config_select():
    global selected_config_path
    data = request.get_json(silent=True) or {}
    config_name = data.get("config_name", "")
    try:
        path = config_path_from_name(config_name)
        with state_lock:
            selected_config_path = path
            message = load_register_state_from_config(path)
            return jsonify(register_response(message))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.get("/api/register/state")
def api_register_state():
    try:
        with state_lock:
            ensure_register_image()
            return jsonify(register_response("登録状態を読み込みました。"))
    except Exception as exc:
        return json_error(str(exc), 500)


@app.post("/api/register/capture")
def api_register_capture():
    settings = settings_from_json()
    max_width = int(settings.get("max_width", 1280))
    camera_index = int(settings.get("camera_index", 0))
    try:
        image = capture_camera_frame(camera_index)
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(REGISTER_CAPTURE_PATH), image)
        with state_lock:
            set_register_source(image, REGISTER_CAPTURE_PATH, max_width)
            return jsonify(register_response("サーバー側カメラから登録画像を取得しました。"))
    except Exception as exc:
        return json_error(str(exc), 503)


@app.post("/api/register/upload")
def api_register_upload():
    if "image" not in request.files:
        return json_error("アップロード画像が指定されていません。")
    settings = settings_from_form()
    max_width = int(settings.get("max_width", 1280))
    uploaded = request.files["image"]
    image = decode_image_bytes(uploaded.read())
    if image is None:
        return json_error("アップロード画像を読み込めませんでした。")

    try:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(REGISTER_UPLOAD_PATH), image)
        with state_lock:
            set_register_source(image, REGISTER_UPLOAD_PATH, max_width)
            return jsonify(register_response("アップロード画像を登録用画像にしました。"))
    except Exception as exc:
        return json_error(str(exc), 500)


@app.post("/api/register/reprocess")
def api_register_reprocess():
    settings = settings_from_json()
    max_width = int(settings.get("max_width", 1280))
    try:
        with state_lock:
            ensure_register_image()
            reprocess_register_image(max_width)
            return jsonify(register_response("処理画像サイズを更新しました。"))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/preview")
def api_register_preview():
    settings = settings_from_json()
    try:
        with state_lock:
            apply_register_settings(settings)
            return jsonify(register_response("プレビューを更新しました。"))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/marker")
def api_register_marker():
    settings = settings_from_json()
    try:
        with state_lock:
            apply_register_settings(settings)
            register_state.marker = find_marker(
                register_state.work_image,
                settings.get("dictionary", "DICT_4X4_50"),
                selected_marker_id(settings),
            )
            message = (
                f"ARマーカーを検出しました: id={register_state.marker.marker_id}, "
                f"side={register_state.marker.side_length:.1f}"
            )
            return jsonify(register_response(message))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/auto-circle")
def api_register_auto_circle():
    settings = settings_from_json()
    try:
        with state_lock:
            apply_register_settings(settings)
            gray = cv2.cvtColor(register_state.work_image, cv2.COLOR_BGR2GRAY)
            register_state.circle = find_meter_circle(gray)
            register_state.marker = None
            return jsonify(register_response("メーター円を自動検出しました。"))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/auto-scale")
def api_register_auto_scale():
    settings = settings_from_json()
    try:
        with state_lock:
            apply_register_settings(settings)
            validate_circle(register_state.work_image.shape, register_state.circle)
            register_state.scale_min_point, register_state.scale_max_point = (
                auto_scale_points_from_circle(register_state.circle)
            )
            return jsonify(register_response("目盛り位置を自動設定しました。"))
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/detect-needle")
def api_register_detect_needle():
    settings = settings_from_json()
    draw_text = bool(settings.get("draw_text", True))
    try:
        with state_lock:
            apply_register_settings(settings)
            annotated, detection = detect_register_needle_preview(draw_text=draw_text)
            response = register_response(detection["message"], draw_text=draw_text)
            response["image"] = image_to_data_url(annotated)
            response["needle_detection"] = detection
            response.update(detection)
            return jsonify(response)
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/register/save")
def api_register_save():
    settings = settings_from_json()
    try:
        with state_lock:
            apply_register_settings(settings)
            marker = register_state.marker or find_marker(
                register_state.work_image,
                settings.get("dictionary", "DICT_4X4_50"),
                selected_marker_id(settings),
            )
            register_state.marker = marker
            circle = register_state.circle
            validate_circle(register_state.work_image.shape, circle)
            registration = register_meter_from_marker(
                marker,
                circle,
                settings.get("dictionary", "DICT_4X4_50"),
                register_state.max_width,
                register_state.image_path,
                register_state.work_image.shape[:2],
            )

            scale_min = register_state.scale_min_point
            scale_max = register_state.scale_max_point
            if scale_min is not None or scale_max is not None:
                if scale_min is None or scale_max is None:
                    raise ValueError("最小目盛りと最大目盛りの両方を設定してください。")
                scale_min = project_point_to_circle_boundary(circle, scale_min)
                scale_max = project_point_to_circle_boundary(circle, scale_max)
                min_rel_x, min_rel_y = point_to_marker_relative(marker, scale_min)
                max_rel_x, max_rel_y = point_to_marker_relative(marker, scale_max)
                registration = MarkerMeterRegistration(
                    **{
                        **asdict(registration),
                        "scale_min_value": float(settings.get("scale_min_value", 0.0)),
                        "scale_max_value": float(settings.get("scale_max_value", 0.1)),
                        "scale_min_rel_x": min_rel_x,
                        "scale_min_rel_y": min_rel_y,
                        "scale_max_rel_x": max_rel_x,
                        "scale_max_rel_y": max_rel_y,
                        "scale_direction": settings.get("scale_direction", "clockwise"),
                        "registration_scale_min_x": scale_min[0],
                        "registration_scale_min_y": scale_min[1],
                        "registration_scale_max_x": scale_max[0],
                        "registration_scale_max_y": scale_max[1],
                    }
                )
                register_state.scale_min_point = scale_min
                register_state.scale_max_point = scale_max

            save_registration(current_config_path(), registration)
            response = register_response("登録を保存しました。")
            response["registration"] = asdict(registration)
            return jsonify(response)
    except Exception as exc:
        return json_error(str(exc), 400)


@app.post("/api/detect/latest")
def api_detect_latest():
    settings = settings_from_json()
    camera_index = int(settings.get("camera_index", 0))
    draw_text = bool(settings.get("draw_text", True))
    try:
        image = capture_camera_frame(camera_index)
    except Exception as exc:
        return json_error(str(exc), 503)

    try:
        registration = load_registration(current_config_path())
        annotated, result = detect_registered_image(
            image,
            registration,
            draw_text=draw_text,
        )
        return jsonify(
            {
                "ok": True,
                "detection_ok": True,
                "message": result.get("message", "検出しました。"),
                "config_name": current_config_path().name,
                "config_settings": config_settings_payload(),
                "image": image_to_data_url(annotated),
                **result,
            }
        )
    except Exception as exc:
        annotated = annotate_detection_error(image, str(exc))
        return jsonify(
            {
                "ok": True,
                "detection_ok": False,
                "message": f"検出できませんでした: {exc}",
                "config_name": current_config_path().name,
                "config_settings": config_settings_payload(),
                "image": image_to_data_url(annotated),
                "marker_detected": False,
                "marker_error": str(exc),
                "marker": None,
                "circle": None,
                "needle": None,
                "needle_value": None,
                "scale_fraction": None,
                "scale_min_point": None,
                "scale_max_point": None,
            }
        )


def main() -> int:
    host = os.environ.get("WEB_METER_HOST", "127.0.0.1")
    port = int(os.environ.get("WEB_METER_PORT", "5000"))
    app.run(host=host, port=port, debug=False, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
