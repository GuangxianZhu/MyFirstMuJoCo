"""通用小控件：滑块+数字框、可折叠区块。"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QDoubleSpinBox, QHBoxLayout, QSlider, QToolButton, QVBoxLayout, QWidget

SLIDER_TICKS = 1000


class SliderSpin(QWidget):
    """滑块 + 数字框，两者双向同步。valueChanged 只在用户操作时发出。"""

    valueChanged = Signal(float)

    def __init__(self, lo: float, hi: float, decimals: int, step: float, suffix: str = "", parent=None):
        super().__init__(parent)
        self._lo, self._hi = float(lo), float(max(hi, lo + 1e-9))

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_TICKS)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(self._lo, self._hi)
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setSuffix(suffix)
        self.spin.setKeyboardTracking(False)   # 手动输入按回车/失焦才生效
        self.spin.setFixedWidth(84)
        self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)

        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def _to_tick(self, v: float) -> int:
        return int(round((v - self._lo) / (self._hi - self._lo) * SLIDER_TICKS))

    def _from_slider(self, tick: int) -> None:
        v = self._lo + (self._hi - self._lo) * tick / SLIDER_TICKS
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self.valueChanged.emit(self.spin.value())

    def _from_spin(self, v: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(self._to_tick(v))
        self.slider.blockSignals(False)
        self.valueChanged.emit(v)

    def value(self) -> float:
        return self.spin.value()

    def set_value(self, v: float) -> None:
        """从外部同步，不触发 valueChanged。"""
        v = min(max(float(v), self._lo), self._hi)
        self.slider.blockSignals(True)
        self.spin.blockSignals(True)
        self.slider.setValue(self._to_tick(v))
        self.spin.setValue(v)
        self.slider.blockSignals(False)
        self.spin.blockSignals(False)


class CollapsibleBox(QWidget):
    """标题行可点击展开/收起的区块。内容往 self.body 布局里加。"""

    def __init__(self, title: str, expanded: bool = False, parent=None):
        super().__init__(parent)
        self.toggle = QToolButton()
        self.toggle.setText(title)
        self.toggle.setCheckable(True)
        self.toggle.setChecked(expanded)
        self.toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle.setAutoRaise(True)
        self.toggle.setStyleSheet("QToolButton { font-weight: bold; }")
        self.toggle.setSizePolicy(self.toggle.sizePolicy().horizontalPolicy(), self.toggle.sizePolicy().verticalPolicy())
        self.toggle.toggled.connect(self._on_toggle)

        self.content = QWidget()
        self.body = QVBoxLayout(self.content)
        self.body.setContentsMargins(14, 2, 0, 6)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self.toggle)
        lay.addWidget(self.content)
        self._on_toggle(expanded)

    def _on_toggle(self, on: bool) -> None:
        self.toggle.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)
        self.content.setVisible(on)

    def set_expanded(self, on: bool) -> None:
        self.toggle.setChecked(on)
