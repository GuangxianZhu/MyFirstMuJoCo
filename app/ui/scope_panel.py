"""实时曲线：选一个关节，看 目标/实际角度、力矩（含限幅线）、各手指触觉法向力。数据来自 sim.scope。"""
from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ..core.sim import Simulation
from .tactile_view import group_order, split_segment

WINDOWS = [2.0, 5.0, 10.0, 20.0]
PALETTE = ["#e8b84a", "#4aa3e8", "#e8604a", "#5ec27a", "#b07ae8", "#e88ac0", "#8ad0d8"]


def style_plot(plot: pg.PlotWidget, title: str, ylabel: str) -> None:
    plot.setBackground("#1b1d22")
    plot.setTitle(title, color="#c8ccd2", size="10pt")
    plot.setLabel("left", ylabel, color="#9aa1ac")
    plot.showGrid(x=True, y=True, alpha=0.18)
    for ax in ("left", "bottom"):
        plot.getAxis(ax).setTextPen("#9aa1ac")
        plot.getAxis(ax).setPen("#4a505b")
    plot.setMenuEnabled(False)
    plot.setMouseEnabled(x=False, y=True)


class ScopePanel(QWidget):
    def __init__(self, sim: Simulation, labels: dict[str, str] | None = None, parent: QWidget | None = None):
        super().__init__(parent)
        self.sim = sim
        labels = labels or {}
        self._specs = sim.actuators
        self._window = 5.0

        # --- 顶部：关节选择 + 时间窗口 ---
        self.joint_box = QComboBox()
        for s in self._specs:
            g = labels.get(f"group:{s.group}", s.group)
            j = labels.get(f"joint:{s.short}", s.short) if s.group else s.joint_name
            self.joint_box.addItem(f"{g} {j}".strip(), s.joint_name)
        default = next((k for k, s in enumerate(self._specs) if s.joint_name == "index_mcp"), None)
        if default is None:
            default = next((k for k, s in enumerate(self._specs) if s.torque), 0)
        self.joint_box.setCurrentIndex(default)
        self.win_box = QComboBox()
        for w in WINDOWS:
            self.win_box.addItem(f"最近 {w:g} 秒", w)
        self.win_box.setCurrentIndex(1)

        top = QHBoxLayout()
        top.addWidget(QLabel("关节"))
        top.addWidget(self.joint_box)
        top.addSpacing(12)
        top.addWidget(QLabel("时间窗口"))
        top.addWidget(self.win_box)
        top.addStretch(1)

        # --- 三张图 ---
        self.p_angle = pg.PlotWidget()
        self.p_tau = pg.PlotWidget()
        self.p_tac = pg.PlotWidget()
        style_plot(self.p_angle, "目标 / 实际", "")
        style_plot(self.p_tau, "关节力矩", "")
        style_plot(self.p_tac, "触觉法向力", "N")
        self.p_angle.addLegend(offset=(8, 4))
        self.c_target = self.p_angle.plot(pen=pg.mkPen("#e8b84a", width=1.5, style=Qt.PenStyle.DashLine), name="目标")
        self.c_actual = self.p_angle.plot(pen=pg.mkPen("#4aa3e8", width=2), name="实际")
        self.c_tau = self.p_tau.plot(pen=pg.mkPen("#e8604a", width=2))
        lim_pen = pg.mkPen("#8a93a0", width=1, style=Qt.PenStyle.DotLine)
        self.lim_hi = pg.InfiniteLine(angle=0, pen=lim_pen)
        self.lim_lo = pg.InfiniteLine(angle=0, pen=lim_pen)
        self.p_tau.addItem(self.lim_hi)
        self.p_tau.addItem(self.lim_lo)

        self._groups = group_order(sim.tactile.names) if sim.tactile else []
        self._group_cols = {
            g: [i for i, n in enumerate(sim.tactile.names) if split_segment(n)[0] == g] for g in self._groups
        } if sim.tactile else {}
        self._tac_curves: dict[str, pg.PlotDataItem] = {}
        if self._groups:
            self.p_tac.addLegend(offset=(8, 4), colCount=3)
            for k, g in enumerate(self._groups):
                name = labels.get(f"group:{g}", labels.get(f"seg:{g}", g))
                self._tac_curves[g] = self.p_tac.plot(pen=pg.mkPen(PALETTE[k % len(PALETTE)], width=1.6), name=name)
            self._tac_total = self.p_tac.plot(pen=pg.mkPen("#ffffff", width=2.2), name="合计")
        else:
            self._tac_total = None

        for p in (self.p_tau, self.p_tac):
            p.setXLink(self.p_angle)

        plots = QHBoxLayout()
        for p in (self.p_angle, self.p_tau, self.p_tac):
            plots.addWidget(p, 1)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addLayout(top)
        lay.addLayout(plots, 1)

        self.joint_box.currentIndexChanged.connect(self._on_joint)
        self.win_box.currentIndexChanged.connect(self._on_window)
        self._on_joint()
        self._on_window()

    # ------------------------------------------------------------------ 选择
    def _on_window(self, _=None) -> None:
        self._window = float(self.win_box.currentData())
        self.p_angle.setXRange(-self._window, 0.0, padding=0.0)

    def _on_joint(self, _=None) -> None:
        k = self.joint_box.currentIndex()
        s = self._specs[k]
        self._k = k
        self._scale = 180.0 / math.pi if s.angular else 1000.0
        unit = "°" if s.angular else "mm"
        self.p_angle.setLabel("left", unit)
        if s.torque:
            self.p_tau.setLabel("left", "N·m")
        else:
            self.p_tau.setLabel("left", "N·m" if s.angular else "N")
        self.p_tau.setTitle("关节力矩（虚线 = 限幅）" if s.torque else "执行器输出", color="#c8ccd2", size="10pt")
        self.refresh()

    # ------------------------------------------------------------------ 刷新
    def refresh(self) -> None:
        v = self.sim.scope.view(self._window)
        t = v["t"]
        if t.size < 2:
            return
        x = t - t[-1]
        k = self._k
        self.c_target.setData(x, v["target"][:, k] * self._scale)
        self.c_actual.setData(x, v["q"][:, k] * self._scale)
        self.c_tau.setData(x, v["tau"][:, k])
        s = self._specs[k]
        if s.torque:
            lim = float(min(self.sim.drive.tau_limit, self.sim.drive.hardware_limit[k]))
            self.lim_hi.setValue(lim)
            self.lim_lo.setValue(-lim)
            self.lim_hi.setVisible(True)
            self.lim_lo.setVisible(True)
        else:
            self.lim_hi.setVisible(False)
            self.lim_lo.setVisible(False)
        if self._groups:
            seg = v["seg"]
            for g, cols in self._group_cols.items():
                self._tac_curves[g].setData(x, seg[:, cols].sum(axis=1))
            self._tac_total.setData(x, seg.sum(axis=1))
