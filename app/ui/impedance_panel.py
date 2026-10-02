"""关节阻抗面板：预设 + K / D 倍率 + 力矩上限。仅在场景里有力矩型执行器时显示。"""
from __future__ import annotations

from PySide6.QtWidgets import QComboBox, QGridLayout, QGroupBox, QLabel, QWidget

from ..core.impedance import PRESETS
from ..core.sim import Simulation
from .widgets import SliderSpin


class ImpedancePanel(QGroupBox):
    def __init__(self, sim: Simulation, parent: QWidget | None = None):
        super().__init__("手指阻抗控制", parent)
        self.sim = sim
        d = sim.drive

        self.preset = QComboBox()
        self.preset.addItem("自定义", None)
        for p in PRESETS:
            self.preset.addItem(p[0], p)
        self.preset.setCurrentIndex(2)   # 默认

        self.k = SliderSpin(0.1, 10.0, 2, 0.1, "×")
        self.d = SliderSpin(0.1, 5.0, 2, 0.1, "×")
        limit_max = float(min(sim.drive.hardware_limit[sim.drive.mask].max(), 2.0)) if d.active else 1.0
        self.limit = SliderSpin(0.05, limit_max, 2, 0.05, " N·m")
        self.info = QLabel()
        self.info.setStyleSheet("color: #8a93a0;")
        self.info.setWordWrap(True)

        g = QGridLayout(self)
        g.addWidget(QLabel("预设"), 0, 0)
        g.addWidget(self.preset, 0, 1)
        g.addWidget(QLabel("刚度 K"), 1, 0)
        g.addWidget(self.k, 1, 1)
        g.addWidget(QLabel("阻尼 D"), 2, 0)
        g.addWidget(self.d, 2, 1)
        g.addWidget(QLabel("力矩上限"), 3, 0)
        g.addWidget(self.limit, 3, 1)
        g.addWidget(self.info, 4, 0, 1, 2)
        g.setColumnStretch(1, 1)

        self.preset.activated.connect(self._on_preset)
        for w in (self.k, self.d, self.limit):
            w.valueChanged.connect(self._on_slider)
        self.sync()

    # ------------------------------------------------------------------ 交互
    def _on_preset(self, _=None) -> None:
        p = self.preset.currentData()
        if p is None:
            return
        _, ks, ds, lim = p
        self.sim.drive.apply_preset(ks, ds, lim)
        self.sync()

    def _on_slider(self, _=None) -> None:
        self.sim.drive.apply_preset(self.k.value(), self.d.value(), self.limit.value())
        self.preset.setCurrentIndex(0)
        self._update_info()

    # ------------------------------------------------------------------ 同步
    def sync(self) -> None:
        """从控制器当前参数刷新控件（演示、重置会改动参数）。"""
        d = self.sim.drive
        self.k.set_value(d.k_scale)
        self.d.set_value(d.d_scale)
        self.limit.set_value(d.tau_limit)
        match = [i for i in range(1, self.preset.count())
                 if self._same(self.preset.itemData(i), d)]
        self.preset.setCurrentIndex(match[0] if match else 0)
        self._update_info()

    @staticmethod
    def _same(p, d) -> bool:
        return abs(p[1] - d.k_scale) < 1e-6 and abs(p[2] - d.d_scale) < 1e-6 and abs(p[3] - d.tau_limit) < 1e-6

    def _update_info(self) -> None:
        d = self.sim.drive
        if not d.active:
            return
        k = float(d.k[d.mask][0])
        dd = float(d.d[d.mask][0])
        self.info.setText(f"实际 K = {k:.2f} N·m/rad，D = {dd:.3f} N·m·s/rad\n"
                          f"力矩 τ = K(目标−角度) − D·角速度，限幅 ±{d.tau_limit:.2f} N·m")
