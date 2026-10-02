"""手势 / 协同 / 节奏动作面板（仅当场景带有 SceneExtras 时显示）。"""
from __future__ import annotations

from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..core.controllers import PATTERNS
from ..core.sim import Simulation
from .widgets import CollapsibleBox, SliderSpin

GESTURE_COLUMNS = 4
DEFAULT_TRANSITION_S = 0.6


class HandPanel(QWidget):
    def __init__(self, sim: Simulation, parent: QWidget | None = None):
        super().__init__(parent)
        assert sim.extras is not None
        self.sim = sim
        ex = sim.extras

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        # --- 预设手势 ---
        box = QGroupBox("预设手势")
        v = QVBoxLayout(box)
        grid = QGridLayout()
        for i, name in enumerate(ex.gestures):
            b = QPushButton(name)
            b.clicked.connect(lambda _=False, n=name: self._on_gesture(n))
            grid.addWidget(b, i // GESTURE_COLUMNS, i % GESTURE_COLUMNS)
        v.addLayout(grid)
        row = QHBoxLayout()
        row.addWidget(QLabel("过渡时间"))
        self.transition = SliderSpin(0.0, 3.0, 2, 0.1, " s")
        self.transition.set_value(DEFAULT_TRANSITION_S)
        row.addWidget(self.transition, 1)
        v.addLayout(row)
        lay.addWidget(box)

        # --- 协同滑块 ---
        sbox = QGroupBox("协同")
        sg = QGridLayout(sbox)
        self._syn: dict[str, SliderSpin] = {}
        for r, s in enumerate(ex.synergies):
            w = SliderSpin(s.lo, s.hi, 2, 0.05)
            w.set_value(s.default)
            w.valueChanged.connect(self._on_synergy)
            sg.addWidget(QLabel(s.label), r, 0)
            sg.addWidget(w, r, 1)
            self._syn[s.key] = w
        sg.setColumnStretch(1, 1)
        lay.addWidget(sbox)

        # --- 节奏动作 ---
        self.rhythm_box = CollapsibleBox("节奏动作", expanded=True)
        rg = QGridLayout()
        self.pattern = QComboBox()
        for key, label in PATTERNS:
            self.pattern.addItem(label, key)
        self.freq = SliderSpin(0.1, 3.0, 2, 0.05, " Hz")
        self.freq.set_value(0.8)
        self.amp = SliderSpin(0.1, 1.0, 2, 0.05)
        self.amp.set_value(1.0)
        self.lag = SliderSpin(0.0, 0.5, 2, 0.01)
        self.lag.set_value(0.12)
        rg.addWidget(QLabel("模式"), 0, 0)
        rg.addWidget(self.pattern, 0, 1)
        rg.addWidget(QLabel("频率"), 1, 0)
        rg.addWidget(self.freq, 1, 1)
        rg.addWidget(QLabel("幅度"), 2, 0)
        rg.addWidget(self.amp, 2, 1)
        rg.addWidget(QLabel("相位差"), 3, 0)
        rg.addWidget(self.lag, 3, 1)
        rg.setColumnStretch(1, 1)
        self.rhythm_box.body.addLayout(rg)
        self.start = QPushButton("开始节奏动作")
        self.start.setCheckable(True)
        self.start.toggled.connect(self._on_rhythm_toggled)
        self.rhythm_box.body.addWidget(self.start)
        hint = QLabel("相位差 = 相邻手指错开的周期比例；“数数”模式忽略幅度和相位差")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #8a93a0;")
        self.rhythm_box.body.addWidget(hint)
        lay.addWidget(self.rhythm_box)

        for w in (self.freq, self.amp, self.lag):
            w.valueChanged.connect(self._push_rhythm_params)
        self.pattern.currentIndexChanged.connect(self._push_rhythm_params)

    # ------------------------------------------------------------------ 手势 / 协同
    def _on_gesture(self, name: str) -> None:
        self.sim.set_pose(self.sim.extras.gestures[name], self.transition.value())

    def _on_synergy(self, _=None) -> None:
        values = {k: w.value() for k, w in self._syn.items()}
        self.sim.set_pose(self.sim.extras.synergy_pose(values), 0.0)

    # ------------------------------------------------------------------ 节奏
    def _push_rhythm_params(self, _=None) -> None:
        r = self.sim.rhythm
        if r is None:
            return
        r.pattern = self.pattern.currentData()
        r.freq, r.amp, r.lag = self.freq.value(), self.amp.value(), self.lag.value()

    def _on_rhythm_toggled(self, on: bool) -> None:
        self.start.setText("停止节奏动作" if on else "开始节奏动作")
        if on:
            self.sim.start_rhythm(self.pattern.currentData(), self.freq.value(), self.amp.value(), self.lag.value())
        else:
            self.sim.stop_rhythm()

    # ------------------------------------------------------------------ 外部同步
    def refresh(self) -> None:
        """仿真重置后：协同滑块回到默认值。"""
        for s in self.sim.extras.synergies:
            self._syn[s.key].set_value(s.default)
