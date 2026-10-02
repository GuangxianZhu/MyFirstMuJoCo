"""左侧关节面板（按手指/部位分组）和右侧仿真状态面板。控件由模型自省结果自动生成。"""
from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..core.params import ActuatorSpec
from .widgets import CollapsibleBox

# 默认展开的分组；其余折叠，需要时点开逐关节微调
EXPANDED_GROUPS = {"", "wrist"}


def _mono() -> QFont:
    f = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
    f.setPointSize(9)
    return f


class _Unit:
    """某个执行器在界面上的显示单位：转动关节用度(0.1° 一格)，滑动关节用毫米(0.5 mm 一格)。"""

    def __init__(self, spec: ActuatorSpec):
        if spec.angular:
            self.scale, self.ticks, self.text = 180.0 / math.pi, 10, "°"
        else:
            self.scale, self.ticks, self.text = 1000.0, 2, "mm"
        self.lo = int(round(spec.lo * self.scale * self.ticks))
        self.hi = int(round(spec.hi * self.scale * self.ticks))

    def to_tick(self, si: float) -> int:
        return int(round(si * self.scale * self.ticks))

    def from_tick(self, tick: int) -> float:
        return tick / (self.scale * self.ticks)

    def show(self, si: float) -> float:
        return si * self.scale


class JointPanel(QGroupBox):
    """每个位置执行器一行：名称 | 滑块(目标) | "目标→实际"。有分组时按组折叠。"""

    targetChanged = Signal(int, float)   # (滑块顺序下标 k, 目标值：弧度或米)
    resetRequested = Signal()

    def __init__(self, specs: list[ActuatorSpec], labels: dict[str, str] | None = None, parent: QWidget | None = None):
        super().__init__("关节目标  (目标 → 实际)", parent)
        labels = labels or {}
        self._specs = specs
        self._units = [_Unit(s) for s in specs]
        self._sliders: list[QSlider] = []
        self._labels: list[QLabel] = []

        outer = QVBoxLayout(self)
        outer.setSpacing(4)

        # 按首次出现的顺序整理分组
        order: list[str] = []
        for s in specs:
            if s.group not in order:
                order.append(s.group)

        for g in order:
            members = [k for k, s in enumerate(specs) if s.group == g]
            grid = QGridLayout()
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(3)
            for r, k in enumerate(members):
                self._add_row(grid, r, k, labels)
            grid.setColumnStretch(1, 1)

            if g:
                box = CollapsibleBox(labels.get(f"group:{g}", g), expanded=g in EXPANDED_GROUPS)
                box.body.addLayout(grid)
                outer.addWidget(box)
            else:
                outer.addLayout(grid)

        btn = QPushButton("全部目标恢复默认")
        btn.clicked.connect(self.resetRequested)
        outer.addWidget(btn)
        outer.addStretch(1)

        self.set_actual(np.array([s.default for s in specs]))

    def _add_row(self, grid: QGridLayout, r: int, k: int, labels: dict[str, str]) -> None:
        s, u = self._specs[k], self._units[k]
        text = labels.get(f"joint:{s.short}", s.short) if s.group else s.joint_name
        name = QLabel(text)
        name.setToolTip(s.joint_name)
        name.setMinimumWidth(62)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(u.lo, u.hi)
        slider.setValue(u.to_tick(s.default))
        slider.valueChanged.connect(lambda v, k=k: self._on_slider(k, v))

        info = QLabel()
        info.setFont(_mono())
        info.setMinimumWidth(128)
        info.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        grid.addWidget(name, r, 0)
        grid.addWidget(slider, r, 1)
        grid.addWidget(info, r, 2)
        self._sliders.append(slider)
        self._labels.append(info)

    def _on_slider(self, k: int, tick: int) -> None:
        self.targetChanged.emit(k, self._units[k].from_tick(tick))

    def set_targets(self, targets: np.ndarray) -> None:
        """从外部同步滑块位置（不触发 targetChanged）。targets 顺序同 specs。"""
        for slider, u, t in zip(self._sliders, self._units, targets):
            slider.blockSignals(True)
            slider.setValue(u.to_tick(float(t)))
            slider.blockSignals(False)

    def set_actual(self, q: np.ndarray) -> None:
        for slider, label, u, v in zip(self._sliders, self._labels, self._units, q):
            target = u.from_tick(slider.value()) * u.scale
            label.setText(f"{target:7.1f}→{u.show(float(v)):7.1f}{u.text}")


class StatusPanel(QGroupBox):
    """仿真状态：时间、帧率、实时倍率，以及暂停/重置按钮。"""

    pauseToggled = Signal(bool)
    resetRequested = Signal()
    cameraResetRequested = Signal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__("仿真", parent)
        form = QFormLayout()
        self._time = QLabel("0.00 s")
        self._fps = QLabel("-")
        self._rtf = QLabel("-")
        self._dt = QLabel("-")
        for lab in (self._time, self._fps, self._rtf, self._dt):
            lab.setFont(_mono())
        form.addRow("仿真时间", self._time)
        form.addRow("画面帧率", self._fps)
        form.addRow("实时倍率", self._rtf)
        form.addRow("物理步长", self._dt)

        self._pause = QPushButton("暂停")
        self._pause.setCheckable(True)
        self._pause.toggled.connect(self._on_pause)
        reset = QPushButton("重置仿真")
        reset.clicked.connect(self.resetRequested)
        cam = QPushButton("复位视角")
        cam.clicked.connect(self.cameraResetRequested)

        lay = QVBoxLayout(self)
        lay.addLayout(form)
        lay.addSpacing(8)
        lay.addWidget(self._pause)
        lay.addWidget(reset)
        lay.addWidget(cam)

    def _on_pause(self, checked: bool) -> None:
        self._pause.setText("继续" if checked else "暂停")
        self.pauseToggled.emit(checked)

    def set_paused(self, paused: bool) -> None:
        self._pause.blockSignals(True)
        self._pause.setChecked(paused)
        self._pause.setText("继续" if paused else "暂停")
        self._pause.blockSignals(False)

    def set_stats(self, sim_time: float, fps: float, rtf: float, timestep: float) -> None:
        self._time.setText(f"{sim_time:8.2f} s")
        self._fps.setText(f"{fps:6.1f}")
        self._rtf.setText(f"{rtf:6.2f}x")
        self._dt.setText(f"{timestep * 1000:.2f} ms")
