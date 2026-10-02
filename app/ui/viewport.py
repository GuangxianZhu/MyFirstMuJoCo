"""中间的 3D 画面：显示渲染帧，并把鼠标操作变成相机移动信号。"""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget


class Viewport(QWidget):
    # 位移单位已换算成"渲染图像的像素"，直接交给 SceneRenderer 使用
    rotateRequested = Signal(float, float)
    panRequested = Signal(float, float)
    zoomRequested = Signal(float)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._image: QImage | None = None
        self._target = QRectF()
        self._last_pos = None
        self.setMinimumSize(480, 300)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    # ------------------------------------------------------------------ 显示
    def set_frame(self, rgb: np.ndarray) -> None:
        rgb = np.ascontiguousarray(rgb)
        h, w, _ = rgb.shape
        # copy() 让 QImage 拥有自己的内存，不依赖 numpy 数组的生命周期
        self._image = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(18, 20, 24))
        if self._image is None:
            return
        p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, True)
        iw, ih = self._image.width(), self._image.height()
        scale = min(self.width() / iw, self.height() / ih)
        tw, th = iw * scale, ih * scale
        self._target = QRectF((self.width() - tw) / 2, (self.height() - th) / 2, tw, th)
        p.drawImage(self._target, self._image)

    def _to_image_pixels(self) -> float:
        """控件像素 -> 渲染图像像素 的换算系数。"""
        if self._image is None or self._target.height() <= 0:
            return 1.0
        return self._image.height() / self._target.height()

    # ------------------------------------------------------------------ 鼠标
    def mousePressEvent(self, e) -> None:
        self._last_pos = e.position()
        self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseReleaseEvent(self, e) -> None:
        self._last_pos = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mouseMoveEvent(self, e) -> None:
        if self._last_pos is None:
            return
        d = e.position() - self._last_pos
        self._last_pos = e.position()
        k = self._to_image_pixels()
        dx, dy = d.x() * k, d.y() * k
        if e.buttons() & Qt.MouseButton.LeftButton:
            self.rotateRequested.emit(dx, dy)
        elif e.buttons() & (Qt.MouseButton.RightButton | Qt.MouseButton.MiddleButton):
            self.panRequested.emit(dx, dy)

    def wheelEvent(self, e) -> None:
        self.zoomRequested.emit(e.angleDelta().y() / 120.0)
