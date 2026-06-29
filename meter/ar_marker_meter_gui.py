from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from typing import Optional, Tuple

import cv2

try:
    from PySide6.QtCore import Qt, Signal
    from PySide6.QtGui import QImage, QPixmap
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSizePolicy,
        QSlider,
        QSpinBox,
        QStatusBar,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    try:
        from PySide2.QtCore import Qt, Signal
        from PySide2.QtGui import QImage, QPixmap
        from PySide2.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QDoubleSpinBox,
            QFileDialog,
            QFormLayout,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QSizePolicy,
            QSlider,
            QSpinBox,
            QStatusBar,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as exc:
        raise SystemExit(
            "PySide is not installed. Install it in the active environment, "
            "for example: python -m pip install PySide2"
        ) from exc

from detect_meter_needle import Circle, Needle, draw_detection, find_meter_circle
from detect_meter_needle_ar_marker import (
    DEFAULT_CONFIG_PATH,
    DEFAULT_DRAW_LINE_THICKNESS,
    DEFAULT_DRAW_POINT_RADIUS,
    DEFAULT_IMAGE_PATH,
    MarkerMeterRegistration,
    MarkerObservation,
    auto_scale_points_from_circle,
    default_output_path,
    detect_needle_with_homography,
    draw_marker,
    draw_scale_points,
    estimate_meter_circle,
    estimate_scale_points,
    fallback_meter_circle_from_registration,
    fallback_scale_points_from_registration,
    find_marker,
    has_scale_registration,
    interpolate_needle_value,
    load_registration,
    marker_relative_to_rectified_point,
    point_to_marker_relative,
    project_point_to_circle_boundary,
    read_image,
    register_meter_from_marker,
    resize_for_processing,
    save_registration,
    validate_projected_meter_circle,
)
from detect_meter_needle_preset_circle import (
    detect_needle_with_preset_circle,
    validate_circle,
)


BASE_DIR = Path(__file__).resolve().parent
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


class ImageView(QLabel):
    clicked = Signal(int, int)

    def __init__(self) -> None:
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(760, 520)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #202020;")
        self._pixmap: Optional[QPixmap] = None
        self._image_size = (1, 1)
        self._display_rect = (0, 0, 1, 1)

    def set_cv_image(self, image_bgr) -> None:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        height, width, channels = image_rgb.shape
        bytes_per_line = channels * width
        q_image = QImage(
            image_rgb.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()
        self._pixmap = QPixmap.fromImage(q_image)
        self._image_size = (width, height)
        self._refresh_pixmap()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._refresh_pixmap()

    def mousePressEvent(self, event) -> None:
        image_point = self._label_point_to_image_point(event.pos().x(), event.pos().y())
        if image_point is not None:
            self.clicked.emit(image_point[0], image_point[1])

    def mouseMoveEvent(self, event) -> None:
        if event.buttons() & Qt.LeftButton:
            image_point = self._label_point_to_image_point(event.pos().x(), event.pos().y())
            if image_point is not None:
                self.clicked.emit(image_point[0], image_point[1])

    def _refresh_pixmap(self) -> None:
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.setPixmap(scaled)
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2
        self._display_rect = (x, y, scaled.width(), scaled.height())

    def _label_point_to_image_point(self, label_x: int, label_y: int) -> Optional[Tuple[int, int]]:
        rect_x, rect_y, rect_w, rect_h = self._display_rect
        if rect_w <= 0 or rect_h <= 0:
            return None
        if label_x < rect_x or label_y < rect_y:
            return None
        if label_x >= rect_x + rect_w or label_y >= rect_y + rect_h:
            return None

        image_w, image_h = self._image_size
        image_x = round((label_x - rect_x) * image_w / rect_w)
        image_y = round((label_y - rect_y) * image_h / rect_h)
        image_x = max(0, min(image_w - 1, image_x))
        image_y = max(0, min(image_h - 1, image_y))
        return image_x, image_y


class ArMarkerMeterWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("AR Marker Meter Registration")
        self.resize(1240, 780)

        self.current_image_path = DEFAULT_IMAGE_PATH
        self.config_path = DEFAULT_CONFIG_PATH
        self.source_image = self._load_source_image(self.current_image_path)
        self.work_image, self.work_scale = resize_for_processing(self.source_image, 1280)
        self.work_h, self.work_w = self.work_image.shape[:2]
        self.preview_image = self.work_image.copy()

        self.marker: Optional[MarkerObservation] = None
        self.registration: Optional[MarkerMeterRegistration] = None
        self.needle: Optional[Needle] = None
        self.scale_min_point: Optional[Tuple[int, int]] = None
        self.scale_max_point: Optional[Tuple[int, int]] = None
        self.syncing_controls = False

        self.image_view = ImageView()
        self.image_view.clicked.connect(self.handle_image_click)
        self.dictionary_combo = QComboBox()
        self.click_target_combo = QComboBox()
        self.marker_id_spin = self._make_spinbox(-1, 10000, 0)
        self.max_width_spin = self._make_spinbox(320, 4000, 1280)
        self.center_x_spin = self._make_spinbox(0, self.work_w - 1, self.work_w // 2)
        self.center_y_spin = self._make_spinbox(0, self.work_h - 1, self.work_h // 2)
        self.radius_spin = self._make_spinbox(
            1,
            min(self.work_w, self.work_h),
            min(self.work_w, self.work_h) // 4,
        )
        self.center_x_slider = self._make_slider(0, self.work_w - 1, self.center_x_spin.value())
        self.center_y_slider = self._make_slider(0, self.work_h - 1, self.center_y_spin.value())
        self.radius_slider = self._make_slider(1, min(self.work_w, self.work_h), self.radius_spin.value())
        self.scale_min_value_spin = self._make_double_spinbox(0.0)
        self.scale_max_value_spin = self._make_double_spinbox(0.1)
        self.scale_direction_combo = QComboBox()
        self.draw_text_checkbox = QCheckBox("文字列を描画")
        self.draw_text_checkbox.setChecked(True)

        self.image_label = QLabel()
        self.image_label.setWordWrap(True)
        self.config_label = QLabel()
        self.config_label.setWordWrap(True)
        self.result_label = QLabel("-")
        self.result_label.setWordWrap(True)

        self._build_layout()
        self._connect_controls()
        self.setStatusBar(QStatusBar())
        self.reset_circle_to_center()
        self.update_preview("ARマーカーを検出し、メーター円と目盛りを登録できます。")

    def _load_source_image(self, image_path: Path):
        image = read_image(image_path)
        if image is None:
            raise FileNotFoundError(f"Could not read image: {image_path}")
        return image

    def _make_spinbox(self, minimum: int, maximum: int, value: int) -> QSpinBox:
        spinbox = QSpinBox()
        spinbox.setRange(minimum, maximum)
        spinbox.setValue(value)
        return spinbox

    def _make_double_spinbox(self, value: float) -> QDoubleSpinBox:
        spinbox = QDoubleSpinBox()
        spinbox.setRange(-999999.0, 999999.0)
        spinbox.setDecimals(6)
        spinbox.setSingleStep(0.01)
        spinbox.setValue(value)
        return spinbox

    def _make_slider(self, minimum: int, maximum: int, value: int) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(value)
        return slider

    def _build_layout(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(12)
        root_layout.addWidget(self.image_view, stretch=1)

        controls = QWidget()
        controls.setFixedWidth(360)
        control_layout = QVBoxLayout(controls)
        control_layout.setSpacing(8)

        open_image_button = QPushButton("画像を開く")
        open_image_button.clicked.connect(self.open_image)
        control_layout.addWidget(open_image_button)
        control_layout.addWidget(self.image_label)

        config_button_row = QHBoxLayout()
        load_config_button = QPushButton("登録読込")
        load_config_button.clicked.connect(self.load_config)
        choose_config_button = QPushButton("設定先変更")
        choose_config_button.clicked.connect(self.choose_config_path)
        config_button_row.addWidget(load_config_button)
        config_button_row.addWidget(choose_config_button)
        control_layout.addLayout(config_button_row)
        control_layout.addWidget(self.config_label)

        self.click_target_combo.addItem("メーター中心", "center")
        self.click_target_combo.addItem("最小目盛り", "scale_min")
        self.click_target_combo.addItem("最大目盛り", "scale_max")
        self.scale_direction_combo.addItem("時計回り", "clockwise")
        self.scale_direction_combo.addItem("反時計回り", "counterclockwise")
        for dictionary_name in DICTIONARIES:
            self.dictionary_combo.addItem(dictionary_name)

        form = QFormLayout()
        form.addRow("辞書", self.dictionary_combo)
        form.addRow("マーカーID (-1 自動)", self.marker_id_spin)
        form.addRow("処理最大幅", self.max_width_spin)
        form.addRow("クリック操作", self.click_target_combo)
        form.addRow("中心 X", self.center_x_spin)
        form.addRow("", self.center_x_slider)
        form.addRow("中心 Y", self.center_y_spin)
        form.addRow("", self.center_y_slider)
        form.addRow("半径", self.radius_spin)
        form.addRow("", self.radius_slider)
        form.addRow("最小値", self.scale_min_value_spin)
        form.addRow("最大値", self.scale_max_value_spin)
        form.addRow("目盛り方向", self.scale_direction_combo)
        form.addRow("", self.draw_text_checkbox)
        control_layout.addLayout(form)

        marker_button = QPushButton("マーカー検出")
        marker_button.clicked.connect(self.detect_marker)
        auto_circle_button = QPushButton("メーター円を自動検出")
        auto_circle_button.clicked.connect(self.auto_detect_circle)
        auto_scale_button = QPushButton("目盛りを自動設定")
        auto_scale_button.clicked.connect(self.auto_set_scale_points)
        clear_scale_button = QPushButton("目盛り登録をクリア")
        clear_scale_button.clicked.connect(self.clear_scale_points)
        register_button = QPushButton("この円と目盛りで登録保存")
        register_button.clicked.connect(self.save_current_registration)
        detect_button = QPushButton("登録から針と値を検出")
        detect_button.clicked.connect(self.detect_from_registration)
        save_button = QPushButton("表示画像を保存")
        save_button.clicked.connect(self.save_preview)
        reset_button = QPushButton("円を中央へ戻す")
        reset_button.clicked.connect(self.reset_circle_to_center)

        for button in [
            marker_button,
            auto_circle_button,
            auto_scale_button,
            clear_scale_button,
            register_button,
            detect_button,
            save_button,
            reset_button,
        ]:
            control_layout.addWidget(button)

        control_layout.addWidget(QLabel("状態 / 検出結果"))
        control_layout.addWidget(self.result_label)
        control_layout.addStretch(1)
        root_layout.addWidget(controls)

    def _connect_controls(self) -> None:
        for spinbox, slider in [
            (self.center_x_spin, self.center_x_slider),
            (self.center_y_spin, self.center_y_slider),
            (self.radius_spin, self.radius_slider),
        ]:
            spinbox.valueChanged.connect(lambda value, s=slider: self.sync_from_spinbox(s, value))
            slider.valueChanged.connect(lambda value, s=spinbox: self.sync_from_slider(s, value))

        self.max_width_spin.valueChanged.connect(self.rebuild_work_image)
        self.dictionary_combo.currentTextChanged.connect(lambda _value: self.clear_marker_and_detection())
        self.marker_id_spin.valueChanged.connect(lambda _value: self.clear_marker_and_detection())
        self.scale_min_value_spin.valueChanged.connect(lambda _value: self.update_preview())
        self.scale_max_value_spin.valueChanged.connect(lambda _value: self.update_preview())
        self.scale_direction_combo.currentIndexChanged.connect(lambda _value: self.update_preview())
        self.draw_text_checkbox.stateChanged.connect(lambda _value: self.update_preview())

    def sync_from_spinbox(self, slider: QSlider, value: int) -> None:
        if self.syncing_controls:
            return
        self.syncing_controls = True
        slider.setValue(value)
        self.syncing_controls = False
        self.clear_detection()
        self.update_preview()

    def sync_from_slider(self, spinbox: QSpinBox, value: int) -> None:
        if self.syncing_controls:
            return
        self.syncing_controls = True
        spinbox.setValue(value)
        self.syncing_controls = False
        self.clear_detection()
        self.update_preview()

    def selected_marker_id(self) -> Optional[int]:
        value = self.marker_id_spin.value()
        return None if value < 0 else value

    def current_circle(self) -> Circle:
        return Circle(
            self.center_x_spin.value(),
            self.center_y_spin.value(),
            self.radius_spin.value(),
            score=0.0,
        )

    def current_click_target(self) -> str:
        target = self.click_target_combo.itemData(self.click_target_combo.currentIndex())
        return target or "center"

    def current_scale_direction(self) -> str:
        direction = self.scale_direction_combo.itemData(self.scale_direction_combo.currentIndex())
        return direction or "clockwise"

    def should_draw_text(self) -> bool:
        return self.draw_text_checkbox.isChecked()

    def set_scale_direction(self, direction: str) -> None:
        index = self.scale_direction_combo.findData(direction)
        if index >= 0:
            self.scale_direction_combo.setCurrentIndex(index)

    def set_circle_controls(self, circle: Circle) -> None:
        self.syncing_controls = True
        try:
            for spinbox, slider, value in [
                (self.center_x_spin, self.center_x_slider, circle.x),
                (self.center_y_spin, self.center_y_slider, circle.y),
                (self.radius_spin, self.radius_slider, circle.radius),
            ]:
                spinbox.setValue(value)
                slider.setValue(value)
        finally:
            self.syncing_controls = False

    def set_scale_points_from_registration(
        self,
        registration: MarkerMeterRegistration,
        marker: Optional[MarkerObservation] = None,
    ) -> None:
        if registration.scale_min_value is not None:
            self.scale_min_value_spin.setValue(float(registration.scale_min_value))
        if registration.scale_max_value is not None:
            self.scale_max_value_spin.setValue(float(registration.scale_max_value))
        self.set_scale_direction(registration.scale_direction)

        if marker is not None and has_scale_registration(registration):
            self.scale_min_point, self.scale_max_point = estimate_scale_points(marker, registration)
            return

        self.scale_min_point, self.scale_max_point = fallback_scale_points_from_registration(
            registration,
            self.work_image.shape[:2],
        )

    def update_control_ranges(self) -> None:
        radius_max = max(1, min(self.work_w, self.work_h))
        self.syncing_controls = True
        try:
            self.center_x_spin.setRange(0, self.work_w - 1)
            self.center_x_slider.setRange(0, self.work_w - 1)
            self.center_y_spin.setRange(0, self.work_h - 1)
            self.center_y_slider.setRange(0, self.work_h - 1)
            self.radius_spin.setRange(1, radius_max)
            self.radius_slider.setRange(1, radius_max)
        finally:
            self.syncing_controls = False

    def rebuild_work_image(self) -> None:
        if self.syncing_controls:
            return
        old_circle = self.current_circle()
        old_min_point = self.scale_min_point
        old_max_point = self.scale_max_point
        old_scale = self.work_scale
        self.work_image, self.work_scale = resize_for_processing(
            self.source_image,
            self.max_width_spin.value(),
        )
        self.work_h, self.work_w = self.work_image.shape[:2]
        self.update_control_ranges()

        scale_ratio = self.work_scale / old_scale if old_scale > 0 else 1.0
        self.set_circle_controls(
            Circle(
                round(old_circle.x * scale_ratio),
                round(old_circle.y * scale_ratio),
                max(1, round(old_circle.radius * scale_ratio)),
                0.0,
            )
        )
        if old_min_point is not None:
            self.scale_min_point = (
                round(old_min_point[0] * scale_ratio),
                round(old_min_point[1] * scale_ratio),
            )
        if old_max_point is not None:
            self.scale_max_point = (
                round(old_max_point[0] * scale_ratio),
                round(old_max_point[1] * scale_ratio),
            )
        self.marker = None
        self.registration = None
        self.needle = None
        self.update_preview("処理画像サイズを更新しました。")

    def apply_registration_controls(self, registration: MarkerMeterRegistration) -> None:
        dictionary_index = self.dictionary_combo.findText(registration.dictionary)
        self.syncing_controls = True
        try:
            if dictionary_index >= 0:
                self.dictionary_combo.setCurrentIndex(dictionary_index)
            self.marker_id_spin.setValue(registration.marker_id)
            self.max_width_spin.setValue(registration.max_width)
        finally:
            self.syncing_controls = False

        self.work_image, self.work_scale = resize_for_processing(
            self.source_image,
            registration.max_width,
        )
        self.work_h, self.work_w = self.work_image.shape[:2]
        self.update_control_ranges()
        self.set_circle_controls(
            fallback_meter_circle_from_registration(registration, self.work_image.shape[:2])
        )
        self.set_scale_points_from_registration(registration)

    def open_image(self) -> None:
        selected_path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "画像を開く",
            str(self.current_image_path.parent),
            "Images (*.jpg *.jpeg *.png *.bmp *.tif *.tiff);;All Files (*)",
        )
        if not selected_path:
            return
        image_path = Path(selected_path)
        image = read_image(image_path)
        if image is None:
            QMessageBox.critical(self, "読込エラー", f"画像を読み込めませんでした: {image_path}")
            return

        self.current_image_path = image_path
        self.source_image = image
        self.work_image, self.work_scale = resize_for_processing(
            self.source_image,
            self.max_width_spin.value(),
        )
        self.work_h, self.work_w = self.work_image.shape[:2]
        self.update_control_ranges()
        self.scale_min_point = None
        self.scale_max_point = None
        self.reset_circle_to_center()
        self.clear_marker_and_detection()
        self.update_preview(f"画像を変更しました: {image_path.name}")

    def choose_config_path(self) -> None:
        selected_path, _selected_filter = QFileDialog.getSaveFileName(
            self,
            "登録設定ファイルを選択",
            str(self.config_path),
            "JSON (*.json);;All Files (*)",
        )
        if selected_path:
            self.config_path = Path(selected_path)
            self.update_preview("登録設定ファイルの保存先を変更しました。")

    def detect_marker(self) -> Optional[MarkerObservation]:
        try:
            self.marker = find_marker(
                self.work_image,
                self.dictionary_combo.currentText(),
                self.selected_marker_id(),
            )
            self.update_preview(
                f"ARマーカーを検出しました: id={self.marker.marker_id}, side={self.marker.side_length:.1f}"
            )
            return self.marker
        except Exception as exc:
            QMessageBox.critical(self, "マーカー検出エラー", str(exc))
            self.update_preview(str(exc))
            return None

    def auto_detect_circle(self) -> None:
        try:
            gray = cv2.cvtColor(self.work_image, cv2.COLOR_BGR2GRAY)
            circle = find_meter_circle(gray)
            self.set_circle_controls(circle)
            self.clear_detection()
            self.update_preview(
                f"メーター円を自動検出しました: center=({circle.x}, {circle.y}), radius={circle.radius}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "円検出エラー", str(exc))
            self.update_preview(str(exc))

    def auto_set_scale_points(self) -> None:
        circle = self.current_circle()
        try:
            validate_circle(self.work_image.shape, circle)
            self.scale_min_point, self.scale_max_point = auto_scale_points_from_circle(circle)
            self.clear_detection()
            self.update_preview(
                "目盛り位置を自動設定しました: "
                f"min={self.scale_min_point}, max={self.scale_max_point}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "目盛り自動設定エラー", str(exc))
            self.update_preview(str(exc))

    def clear_scale_points(self) -> None:
        self.scale_min_point = None
        self.scale_max_point = None
        self.clear_detection()
        self.update_preview("目盛り登録をクリアしました。")

    def save_current_registration(self) -> None:
        marker = self.marker or self.detect_marker()
        if marker is None:
            return

        circle = self.current_circle()
        try:
            validate_circle(self.work_image.shape, circle)
            registration = register_meter_from_marker(
                marker,
                circle,
                self.dictionary_combo.currentText(),
                self.max_width_spin.value(),
                self.current_image_path,
                self.work_image.shape[:2],
            )

            if self.scale_min_point is not None or self.scale_max_point is not None:
                if self.scale_min_point is None or self.scale_max_point is None:
                    raise ValueError("最小目盛りと最大目盛りの両方を設定してください。")
                scale_min_point = project_point_to_circle_boundary(circle, self.scale_min_point)
                scale_max_point = project_point_to_circle_boundary(circle, self.scale_max_point)
                if scale_min_point is None or scale_max_point is None:
                    raise ValueError("最小目盛りと最大目盛りの両方を設定してください。")
                self.scale_min_point = scale_min_point
                self.scale_max_point = scale_max_point
                min_rel_x, min_rel_y = point_to_marker_relative(marker, scale_min_point)
                max_rel_x, max_rel_y = point_to_marker_relative(marker, scale_max_point)
                registration = replace(
                    registration,
                    scale_min_value=self.scale_min_value_spin.value(),
                    scale_max_value=self.scale_max_value_spin.value(),
                    scale_min_rel_x=min_rel_x,
                    scale_min_rel_y=min_rel_y,
                    scale_max_rel_x=max_rel_x,
                    scale_max_rel_y=max_rel_y,
                    scale_direction=self.current_scale_direction(),
                    registration_scale_min_x=scale_min_point[0],
                    registration_scale_min_y=scale_min_point[1],
                    registration_scale_max_x=scale_max_point[0],
                    registration_scale_max_y=scale_max_point[1],
                )

            save_registration(self.config_path, registration)
            self.registration = registration
            scale_summary = "目盛り: 未登録"
            if has_scale_registration(registration):
                scale_summary = (
                    "目盛り: "
                    f"{registration.scale_min_value} -> {registration.scale_max_value}, "
                    f"方向={registration.scale_direction}"
                )
            self.result_label.setText(
                "登録を保存しました\n"
                f"設定: {self.config_path}\n"
                f"marker_id: {registration.marker_id}\n"
                f"center_rel: ({registration.center_rel_x:.6f}, {registration.center_rel_y:.6f})\n"
                f"radius_rel: {registration.radius_rel:.6f}\n"
                f"{scale_summary}"
            )
            self.update_preview("ARマーカーからメーター円と目盛りを復元する登録を保存しました。")
        except Exception as exc:
            QMessageBox.critical(self, "登録保存エラー", str(exc))
            self.update_preview(str(exc))

    def load_config(self) -> None:
        selected_path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "登録設定ファイルを開く",
            str(self.config_path.parent),
            "JSON (*.json);;All Files (*)",
        )
        if selected_path:
            self.config_path = Path(selected_path)
        try:
            registration = load_registration(self.config_path)
            self.apply_registration_controls(registration)
            self.registration = registration
            self.result_label.setText(
                "登録を読み込みました\n"
                f"設定: {self.config_path}\n"
                f"marker_id: {registration.marker_id}\n"
                f"center_rel: ({registration.center_rel_x:.6f}, {registration.center_rel_y:.6f})\n"
                f"radius_rel: {registration.radius_rel:.6f}"
            )
            self.update_preview("登録設定を読み込みました。")
        except Exception as exc:
            QMessageBox.critical(self, "登録読込エラー", str(exc))
            self.update_preview(str(exc))

    def detect_from_registration(self) -> None:
        try:
            if self.registration is None:
                self.registration = load_registration(self.config_path)
                self.apply_registration_controls(self.registration)

            marker = None
            marker_error = None
            try:
                marker = find_marker(
                    self.work_image,
                    self.registration.dictionary,
                    self.registration.marker_id,
                )
                circle = estimate_meter_circle(marker, self.registration)
            except RuntimeError as exc:
                marker_error = exc
                circle = self.current_circle()
            if marker is not None:
                validate_projected_meter_circle(self.work_image.shape, circle)
            else:
                validate_circle(self.work_image.shape, circle)
            self.marker = marker
            self.set_circle_controls(circle)
            rectification = None
            rectified_needle = None
            if marker is not None:
                self.needle, rectification, rectified_needle = detect_needle_with_homography(
                    self.work_image,
                    marker,
                    self.registration,
                )
            else:
                self.needle = detect_needle_with_preset_circle(self.work_image, circle)

            value_text = "指示値: 目盛り未登録"
            if has_scale_registration(self.registration):
                if marker is not None:
                    self.set_scale_points_from_registration(self.registration, marker)
                elif self.scale_min_point is None or self.scale_max_point is None:
                    self.set_scale_points_from_registration(self.registration)
                if self.scale_min_point is not None and self.scale_max_point is not None:
                    value_center = circle
                    value_tip = self.needle.tip
                    min_value_point = self.scale_min_point
                    max_value_point = self.scale_max_point
                    if marker is not None and rectification is not None and rectified_needle is not None:
                        value_center = rectification.circle
                        value_tip = rectified_needle.tip
                        min_value_point = marker_relative_to_rectified_point(
                            rectification,
                            self.registration.scale_min_rel_x,
                            self.registration.scale_min_rel_y,
                        )
                        max_value_point = marker_relative_to_rectified_point(
                            rectification,
                            self.registration.scale_max_rel_x,
                            self.registration.scale_max_rel_y,
                        )
                    needle_value, fraction = interpolate_needle_value(
                        (value_center.x, value_center.y),
                        value_tip,
                        min_value_point,
                        max_value_point,
                        float(self.registration.scale_min_value),
                        float(self.registration.scale_max_value),
                        self.registration.scale_direction,
                    )
                    value_text = (
                        f"指示値: {needle_value:.6f}\n"
                        f"目盛り位置: {fraction * 100.0:.2f}%"
                    )

            self.result_label.setText(
                "登録から針を検出しました\n"
                f"円中心: ({circle.x}, {circle.y})\n"
                f"半径: {circle.radius}\n"
                f"針先: {self.needle.tip}\n"
                f"画像角度: {self.needle.image_angle_deg:.2f} deg\n"
                f"数学角度: {self.needle.cartesian_angle_deg:.2f} deg\n"
                f"{value_text}"
            )
            self.update_preview("登録値からメーター円を推定し、針と指示値を検出しました。")
            self.save_preview()
        except Exception as exc:
            QMessageBox.critical(self, "登録検出エラー", str(exc))
            self.update_preview(str(exc))

    def save_preview(self) -> None:
        output_path = default_output_path(self.current_image_path)
        if not output_path.is_absolute():
            output_path = BASE_DIR / output_path
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), self.preview_image):
            QMessageBox.critical(self, "保存エラー", f"保存できませんでした: {output_path}")
            return
        self.statusBar().showMessage(f"保存しました: {output_path.relative_to(BASE_DIR)}")

    def handle_image_click(self, x: int, y: int) -> None:
        target = self.current_click_target()
        if target == "scale_min":
            self.scale_min_point = project_point_to_circle_boundary(self.current_circle(), (x, y))
            self.clear_detection()
            self.update_preview(f"最小目盛り位置を設定しました: {self.scale_min_point}")
            return
        if target == "scale_max":
            self.scale_max_point = project_point_to_circle_boundary(self.current_circle(), (x, y))
            self.clear_detection()
            self.update_preview(f"最大目盛り位置を設定しました: {self.scale_max_point}")
            return
        self.set_circle_center(x, y)

    def set_circle_center(self, x: int, y: int) -> None:
        self.set_circle_controls(Circle(x, y, self.radius_spin.value(), 0.0))
        self.clear_detection()
        self.update_preview(f"メーター円の中心を設定しました: ({x}, {y})")

    def reset_circle_to_center(self) -> None:
        radius = max(1, min(self.work_w, self.work_h) // 4)
        self.set_circle_controls(Circle(self.work_w // 2, self.work_h // 2, radius, 0.0))
        self.clear_detection()
        self.update_preview("メーター円を画像中央へ戻しました。")

    def clear_marker_and_detection(self) -> None:
        if self.syncing_controls:
            return
        self.marker = None
        self.registration = None
        self.clear_detection()
        self.update_preview()

    def clear_detection(self) -> None:
        self.needle = None

    def update_preview(self, status: str = "") -> None:
        circle = self.current_circle()
        draw_text = self.should_draw_text()
        self.preview_image = self.work_image.copy()

        if self.needle is not None:
            self.preview_image = draw_detection(
                self.work_image,
                circle,
                self.needle,
                draw_text=draw_text,
                line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
            )
        else:
            cv2.circle(
                self.preview_image,
                (circle.x, circle.y),
                circle.radius,
                (0, 180, 0),
                DEFAULT_DRAW_LINE_THICKNESS,
            )
            cv2.circle(self.preview_image, (circle.x, circle.y), 5, (255, 0, 0), -1)

        if self.marker is not None:
            draw_marker(
                self.preview_image,
                self.marker,
                draw_text=draw_text,
                line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
            )
        display_min_point = project_point_to_circle_boundary(circle, self.scale_min_point)
        display_max_point = project_point_to_circle_boundary(circle, self.scale_max_point)
        draw_scale_points(
            self.preview_image,
            display_min_point,
            display_max_point,
            draw_text=draw_text,
            line_thickness=DEFAULT_DRAW_LINE_THICKNESS,
            point_radius=DEFAULT_DRAW_POINT_RADIUS,
        )

        self.image_view.set_cv_image(self.preview_image)
        self.image_label.setText(
            f"{self.current_image_path.name}\n"
            f"source: {self.source_image.shape[1]} x {self.source_image.shape[0]}\n"
            f"work: {self.work_w} x {self.work_h}, scale={self.work_scale:.4f}"
        )
        self.config_label.setText(f"設定: {self.config_path}")

        if status:
            self.statusBar().showMessage(status)


def main() -> int:
    app = QApplication(sys.argv)
    window = ArMarkerMeterWindow()
    window.show()
    if hasattr(app, "exec"):
        return app.exec()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
