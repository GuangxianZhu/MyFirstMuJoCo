"""右栏的物理参数面板：重力 + 每类参数（全局倍率 + 可展开的单独调节）。

控件由 PhysicsParams.groups 自动生成；面板直接调用核心层的方法，不碰 MuJoCo 数据结构。
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ..core.physics import GRAVITY_MAX, GRAVITY_PRESETS, PhysicsParams, VectorParam
from .widgets import SliderSpin


class GravitySection(QGroupBox):
    def __init__(self, params: PhysicsParams, parent=None):
        super().__init__("重力", parent)
        self._p = params

        self.preset = QComboBox()
        self.preset.addItem("自定义", None)
        for name, g in GRAVITY_PRESETS:
            self.preset.addItem(f"{name}  {g:g}", g)
        self.value = SliderSpin(0.0, GRAVITY_MAX, 2, 0.1, " m/s²")

        grid = QGridLayout(self)
        grid.addWidget(QLabel("预设"), 0, 0)
        grid.addWidget(self.preset, 0, 1)
        grid.addWidget(QLabel("大小"), 1, 0)
        grid.addWidget(self.value, 1, 1)
        grid.setColumnStretch(1, 1)

        self.value.valueChanged.connect(self._on_value)
        self.preset.activated.connect(self._on_preset)
        self.sync()

    def _on_value(self, g: float) -> None:
        self._p.set_gravity(g)
        self._select_matching_preset(g)

    def _on_preset(self, i: int) -> None:
        g = self.preset.itemData(i)
        if g is None:
            return
        self._p.set_gravity(g)
        self.value.set_value(g)

    def _select_matching_preset(self, g: float) -> None:
        for i in range(1, self.preset.count()):
            if abs(self.preset.itemData(i) - g) < 0.005:
                self.preset.setCurrentIndex(i)
                return
        self.preset.setCurrentIndex(0)

    def sync(self) -> None:
        g = self._p.gravity
        self.value.set_value(g)
        self._select_matching_preset(g)


class VectorParamSection(QGroupBox):
    """一类参数：顶部是全局倍率，点开"单独调节"后每个关节/连杆各有一行。"""

    def __init__(self, param: VectorParam, parent=None):
        super().__init__(f"{param.title}  [{param.unit}]", parent)
        self._param = param
        lay = QVBoxLayout(self)

        # 全局倍率
        row = QHBoxLayout()
        row.addWidget(QLabel("倍率"))
        self.scale = SliderSpin(0.0, 5.0, 2, 0.05, "×")
        self.scale.setToolTip("按出厂值整体缩放；拖动它会覆盖下面的单独设置")
        self.scale.valueChanged.connect(self._on_scale)
        row.addWidget(self.scale, 1)
        lay.addLayout(row)

        # 展开按钮
        self.toggle = QToolButton()
        self.toggle.setCheckable(True)
        self.toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.toggle.setText("单独调节")
        self.toggle.setAutoRaise(True)
        self.toggle.toggled.connect(self._on_toggle)
        lay.addWidget(self.toggle)

        # 逐个元素
        self.rows = QWidget()
        grid = QGridLayout(self.rows)
        grid.setContentsMargins(0, 0, 0, 0)
        self._items: list[SliderSpin] = []
        for i, label in enumerate(param.labels):
            item = SliderSpin(param.min_value, param.max_for(i), param.decimals, param.step)
            item.valueChanged.connect(lambda v, i=i: self._param.set_value(i, v))
            name = QLabel(label)
            name.setFixedWidth(64)
            grid.addWidget(name, i, 0)
            grid.addWidget(item, i, 1)
            self._items.append(item)
        grid.setColumnStretch(1, 1)
        self.rows.setVisible(False)
        lay.addWidget(self.rows)

        self.sync()

    def _on_toggle(self, on: bool) -> None:
        self.toggle.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)
        self.rows.setVisible(on)

    def _on_scale(self, s: float) -> None:
        self._param.set_scale(s)
        self._sync_items()

    def _sync_items(self) -> None:
        for item, v in zip(self._items, self._param.values):
            item.set_value(v)

    def sync(self) -> None:
        self.scale.set_value(self._param.scale)
        self._sync_items()


class PhysicsPanel(QWidget):
    def __init__(self, params: PhysicsParams, parent=None):
        super().__init__(parent)
        self._p = params
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.gravity = GravitySection(params)
        lay.addWidget(self.gravity)
        self.sections = [VectorParamSection(g) for g in params.groups]
        for s in self.sections:
            lay.addWidget(s)

        reset = QPushButton("恢复默认物理参数")
        reset.setToolTip("只恢复重力/阻尼/质量/增益，不影响仿真状态")
        reset.clicked.connect(self.reset_defaults)
        lay.addWidget(reset)
        lay.addStretch(1)

    def reset_defaults(self) -> None:
        self._p.reset_all()
        self.sync()

    def sync(self) -> None:
        self.gravity.sync()
        for s in self.sections:
            s.sync()
