from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    from PySide6.QtCore import Qt, Signal
    from PySide6.QtGui import QImage, QPixmap
    from PySide6.QtWidgets import (
        QApplication,
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

from detect_meter_needle import Circle, Needle, draw_detection
from detect_meter_needle_preset_circle import (
    DEFAULT_CENTER_X,
    DEFAULT_CENTER_Y,
    DEFAULT_RADIUS,
    IMAGE_PATH,
    detect_needle_with_preset_circle,
    validate_circle,
)


BASE_DIR = Path(__file__).resolve().parent


def read_image(path: Path):
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


class ImageView(QLabel):
    clicked = Signal(int, int)

    def __init__(self) -> None:
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(720, 480)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #202020;")
        self._pixmap: Optional[QPixmap] = None
        self._image_size = (1, 1)
        self._display_rect = (0, 0, 1, 1)

    def set_cv_image(self, image_bgr) -> None:
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        bytes_per_line = channels * width
        q_image = QImage(
            rgb.data,
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


class MeterNeedleWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Meter Needle Detector")
        self.resize(1180, 760)

        self.current_image_path = IMAGE_PATH
        self.image_bgr = read_image(self.current_image_path)
        if self.image_bgr is None:
            raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

        self.image_h, self.image_w = self.image_bgr.shape[:2]
        self.preview_bgr = self.image_bgr.copy()
        self.needle: Optional[Needle] = None
        self.syncing_controls = False

        self.image_view = ImageView()
        self.image_view.clicked.connect(self.set_center_from_image)

        center_x, center_y, radius = self._default_circle_values()
        self.center_x_spin = self._make_spinbox(0, self.image_w - 1, center_x)
        self.center_y_spin = self._make_spinbox(0, self.image_h - 1, center_y)
        self.radius_spin = self._make_spinbox(10, min(self.image_w, self.image_h), radius)

        self.center_x_slider = self._make_slider(0, self.image_w - 1, center_x)
        self.center_y_slider = self._make_slider(0, self.image_h - 1, center_y)
        self.radius_slider = self._make_slider(10, min(self.image_w, self.image_h), radius)

        self.image_path_label = QLabel()
        self.image_path_label.setWordWrap(True)
        self.result_label = QLabel("-")
        self.result_label.setWordWrap(True)
        self.result_label.setMinimumWidth(260)

        self._build_layout()
        self._connect_controls()
        self.setStatusBar(QStatusBar())
        self.update_preview()

    def _make_spinbox(self, minimum: int, maximum: int, value: int) -> QSpinBox:
        spinbox = QSpinBox()
        spinbox.setRange(minimum, maximum)
        spinbox.setValue(value)
        return spinbox

    def _make_slider(self, minimum: int, maximum: int, value: int) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(minimum, maximum)
        slider.setValue(value)
        return slider

    def _default_circle_values(self) -> Tuple[int, int, int]:
        try:
            is_default_image = self.current_image_path.resolve() == IMAGE_PATH.resolve()
        except OSError:
            is_default_image = False

        if is_default_image:
            center_x = min(DEFAULT_CENTER_X, self.image_w - 1)
            center_y = min(DEFAULT_CENTER_Y, self.image_h - 1)
            radius = DEFAULT_RADIUS
        else:
            center_x = self.image_w // 2
            center_y = self.image_h // 2
            radius = min(self.image_w, self.image_h) // 4

        max_inside_radius = max(
            1,
            min(center_x, center_y, self.image_w - 1 - center_x, self.image_h - 1 - center_y),
        )
        radius = max(1, min(radius, max_inside_radius))
        return center_x, center_y, radius

    def _set_control_ranges_and_values(self, center_x: int, center_y: int, radius: int) -> None:
        radius_max = max(1, min(self.image_w, self.image_h))
        self.syncing_controls = True
        try:
            self.center_x_spin.setRange(0, self.image_w - 1)
            self.center_x_slider.setRange(0, self.image_w - 1)
            self.center_y_spin.setRange(0, self.image_h - 1)
            self.center_y_slider.setRange(0, self.image_h - 1)
            self.radius_spin.setRange(1, radius_max)
            self.radius_slider.setRange(1, radius_max)

            self.center_x_spin.setValue(center_x)
            self.center_x_slider.setValue(center_x)
            self.center_y_spin.setValue(center_y)
            self.center_y_slider.setValue(center_y)
            self.radius_spin.setValue(radius)
            self.radius_slider.setValue(radius)
        finally:
            self.syncing_controls = False

    def open_image(self) -> None:
        selected_path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "画像を開く",
            str(self.current_image_path.parent),
            "Images (*.jpg *.jpeg *.png *.bmp *.tif *.tiff);;All Files (*)",
        )
        if not selected_path:
            return
        self.load_image(Path(selected_path))

    def load_image(self, image_path: Path) -> None:
        image = read_image(image_path)
        if image is None:
            QMessageBox.critical(self, "読み込みエラー", f"画像を読み込めませんでした: {image_path}")
            return

        self.current_image_path = image_path
        self.image_bgr = image
        self.image_h, self.image_w = self.image_bgr.shape[:2]
        self.preview_bgr = self.image_bgr.copy()
        self.clear_detection()

        center_x, center_y, radius = self._default_circle_values()
        self._set_control_ranges_and_values(center_x, center_y, radius)
        self.update_preview()
        self.statusBar().showMessage(f"画像を変更しました: {self.current_image_path.name}")

    def output_path_for_current_image(self) -> Path:
        stem = self.current_image_path.stem or "image"
        return BASE_DIR / "output" / f"{stem}_needle_detected_pyside.jpg"

    def _build_layout(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(12)
        root_layout.addWidget(self.image_view, stretch=1)

        controls = QWidget()
        controls.setFixedWidth(310)
        control_layout = QVBoxLayout(controls)
        control_layout.setSpacing(10)

        open_image_button = QPushButton("画像を開く")
        open_image_button.clicked.connect(self.open_image)
        control_layout.addWidget(open_image_button)
        control_layout.addWidget(self.image_path_label)
        control_layout.addSpacing(8)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.addRow("中心 X", self.center_x_spin)
        form.addRow("", self.center_x_slider)
        form.addRow("中心 Y", self.center_y_spin)
        form.addRow("", self.center_y_slider)
        form.addRow("半径", self.radius_spin)
        form.addRow("", self.radius_slider)
        control_layout.addLayout(form)

        detect_button = QPushButton("検出")
        detect_button.clicked.connect(self.detect)
        save_button = QPushButton("保存")
        save_button.clicked.connect(lambda: self.save_preview())
        reset_button = QPushButton("リセット")
        reset_button.clicked.connect(self.reset_circle)

        control_layout.addWidget(detect_button)
        control_layout.addWidget(save_button)
        control_layout.addWidget(reset_button)

        result_title = QLabel("検出結果")
        result_title.setStyleSheet("font-weight: bold;")
        control_layout.addSpacing(10)
        control_layout.addWidget(result_title)
        control_layout.addWidget(self.result_label)
        control_layout.addStretch(1)

        root_layout.addWidget(controls)

    def _connect_controls(self) -> None:
        pairs = [
            (self.center_x_spin, self.center_x_slider),
            (self.center_y_spin, self.center_y_slider),
            (self.radius_spin, self.radius_slider),
        ]
        for spinbox, slider in pairs:
            spinbox.valueChanged.connect(lambda value, s=slider: self.sync_from_spinbox(s, value))
            slider.valueChanged.connect(lambda value, s=spinbox: self.sync_from_slider(s, value))

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

    def clear_detection(self) -> None:
        self.needle = None
        self.result_label.setText("-")

    def current_circle(self) -> Circle:
        return Circle(
            self.center_x_spin.value(),
            self.center_y_spin.value(),
            self.radius_spin.value(),
            score=0.0,
        )

    def set_center_from_image(self, x: int, y: int) -> None:
        self.syncing_controls = True
        self.center_x_spin.setValue(x)
        self.center_x_slider.setValue(x)
        self.center_y_spin.setValue(y)
        self.center_y_slider.setValue(y)
        self.syncing_controls = False
        self.clear_detection()
        self.update_preview()

    def update_preview(self) -> None:
        circle = self.current_circle()
        self.preview_bgr = self.image_bgr.copy()
        self.image_path_label.setText(
            f"{self.current_image_path.name}\n"
            f"{self.image_w} x {self.image_h}"
        )

        if self.needle is not None:
            self.preview_bgr = draw_detection(self.image_bgr, circle, self.needle)
        else:
            cv2.circle(self.preview_bgr, (circle.x, circle.y), circle.radius, (0, 180, 0), 2)
            cv2.circle(self.preview_bgr, (circle.x, circle.y), 5, (255, 0, 0), -1)

        self.image_view.set_cv_image(self.preview_bgr)
        try:
            validate_circle(self.image_bgr.shape, circle)
            self.statusBar().showMessage("メーター円を指定して検出できます。")
        except ValueError as exc:
            self.statusBar().showMessage(str(exc))

    def detect(self) -> None:
        circle = self.current_circle()
        try:
            validate_circle(self.image_bgr.shape, circle)
            self.needle = detect_needle_with_preset_circle(self.image_bgr, circle)
            self.result_label.setText(
                f"中心: {self.needle.center}\n"
                f"針先: {self.needle.tip}\n"
                f"画像角度: {self.needle.image_angle_deg:.2f} deg\n"
                f"数学角度: {self.needle.cartesian_angle_deg:.2f} deg"
            )
            self.update_preview()
            self.save_preview(self.output_path_for_current_image())
        except Exception as exc:
            QMessageBox.critical(self, "検出エラー", str(exc))
            self.statusBar().showMessage(str(exc))

    def save_preview(self, output_path: Optional[Path] = None) -> None:
        if output_path is None:
            output_path = self.output_path_for_current_image()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), self.preview_bgr):
            QMessageBox.critical(self, "保存エラー", f"保存できませんでした: {output_path}")
            return
        self.statusBar().showMessage(f"保存しました: {output_path.relative_to(BASE_DIR)}")

    def reset_circle(self) -> None:
        center_x, center_y, radius = self._default_circle_values()
        self._set_control_ranges_and_values(center_x, center_y, radius)
        self.clear_detection()
        self.update_preview()


def main() -> int:
    app = QApplication(sys.argv)
    window = MeterNeedleWindow()
    window.show()
    if hasattr(app, "exec"):
        return app.exec()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
