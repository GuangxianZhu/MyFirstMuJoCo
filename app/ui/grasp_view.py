"""抓取页签：判定状态（大字）、指标数字表、各指/总握力曲线（有易碎阈值时画虚线），以及"重置物体"按钮。数据来自 sim.grasp。"""
from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from ..core.grasp import GROUPS
from ..core.sim import Simulation
from .scope_panel import PALETTE, style_plot

WINDOW_S = 5.0

STATE_COLORS = {
    "抓牢": "#5ec27a",
    "碎了": "#e8604a",
    "滑落": "#e8604a",
    "抬起": "#e8b84a",
    "接触": "#e8b84a",
    "未接触": "#8a93a0",
}

# (指标键, 显示名, 换算倍数, 格式)
ROWS = [
    ("grip_total", "总握力", 1.0, "{:.2f} N"),
    ("lift", "抬升高度", 1000.0, "{:.1f} mm"),
    ("displacement", "位移", 1000.0, "{:.1f} mm"),
    ("slip", "打滑（相对手）", 1000.0, "{:.1f} mm"),
    ("slip_rate", "打滑速度", 1000.0, "{:.1f} mm/s"),
    ("n_contacts", "接触点数", 1.0, "{:.0f}"),
]


class GraspView(QWidget):
    resetRequested = Signal()

    def __init__(self, sim: Simulation, labels: dict[str, str] | None = None, parent: QWidget | None = None):
        super().__init__(parent)
        self.sim = sim
        labels = labels or {}
        self._gname = {g: labels.get(f"group:{g}", labels.get(f"seg:{g}", g)) for g in GROUPS}

        # --- 顶部：状态大字 + 物体名 + 重置按钮 ---
        self.state_label = QLabel("未接触")
        self.state_label.setStyleSheet("font-size: 22px; font-weight: bold;")
        self.object_label = QLabel("")
        self.object_label.setStyleSheet("color: #aab1bc;")
        self.reset_btn = QPushButton("重置物体")
        self.reset_btn.setToolTip("重置仿真（物体和手都回到初始状态，指标清零）")
        top = QHBoxLayout()
        top.addWidget(self.state_label)
        top.addSpacing(14)
        top.addWidget(self.object_label)
        top.addStretch(1)
        top.addWidget(self.reset_btn)

        # --- 左：数字表 ---
        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        grid.setVerticalSpacing(3)
        self._values: dict[str, QLabel] = {}
        row = 0
        for key, name, _, _ in ROWS:
            grid.addWidget(QLabel(name), row, 0)
            v = QLabel("-")
            v.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            v.setStyleSheet("font-family: monospace;")
            grid.addWidget(v, row, 1)
            self._values[key] = v
            row += 1
        for g in GROUPS:
            grid.addWidget(QLabel(f"{self._gname[g]}握力"), row, 0)
            v = QLabel("-")
            v.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            v.setStyleSheet("font-family: monospace;")
            grid.addWidget(v, row, 1)
            self._values[f"grip_{g}"] = v
            row += 1
        grid.addWidget(QLabel("接触部位"), row, 0)
        self.touching = QLabel("-")
        self.touching.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(self.touching, row, 1)
        left_box = QWidget()
        left_lay = QVBoxLayout(left_box)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.addLayout(grid)
        left_lay.addStretch(1)
        left_box.setFixedWidth(250)

        # --- 右：握力曲线 ---
        self.plot = pg.PlotWidget()
        style_plot(self.plot, "手对物体的法向力（最近 5 秒）", "N")
        self.plot.addLegend(offset=(8, 4), colCount=4)
        self.plot.setXRange(-WINDOW_S, 0.0, padding=0.0)
        self._curves = {
            g: self.plot.plot(pen=pg.mkPen(PALETTE[k % len(PALETTE)], width=1.6), name=self._gname[g])
            for k, g in enumerate(GROUPS)
        }
        self._total = self.plot.plot(pen=pg.mkPen("#ffffff", width=2.2), name="合计")
        self.limit_line = pg.InfiniteLine(
            angle=0, pen=pg.mkPen("#e8604a", width=1.4, style=Qt.PenStyle.DashLine),
            label="易碎阈值", labelOpts={"color": "#e8604a", "position": 0.08},
        )
        self.plot.addItem(self.limit_line)
        self.limit_line.setVisible(False)

        body = QHBoxLayout()
        body.addWidget(left_box)
        body.addWidget(self.plot, 1)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.addLayout(top)
        lay.addLayout(body, 1)

        self.reset_btn.clicked.connect(self.resetRequested)
        self.refresh()

    # ------------------------------------------------------------------ 刷新
    def state_text(self) -> str:
        return self.state_label.text()

    def refresh(self) -> None:
        gm = self.sim.grasp
        if gm is None:
            return
        m = gm.metrics()
        st = m["state"]
        self.state_label.setText(st)
        self.state_label.setStyleSheet(f"font-size: 22px; font-weight: bold; color: {STATE_COLORS.get(st, '#c8ccd2')};")
        lim = m["fragile_limit"]
        self.object_label.setText(f"物体：{m['object']}" + (f"    易碎阈值 {lim:.1f} N（单指）" if lim is not None else ""))
        for key, _, scale, fmt in ROWS:
            self._values[key].setText(fmt.format(m[key] * scale))
        for g in GROUPS:
            self._values[f"grip_{g}"].setText(f"{m[f'grip_{g}']:.2f} N")
        self.touching.setText("、".join(self._gname[g] for g in m["fingers_touching"]) or "-")

        h = gm.history(WINDOW_S)
        t = h["t"]
        if lim is not None:
            self.limit_line.setValue(lim)
        self.limit_line.setVisible(lim is not None)
        if t.size < 2:
            for c in (*self._curves.values(), self._total):
                c.setData([], [])
            return
        x = t - t[-1]
        for g, c in self._curves.items():
            c.setData(x, h[f"grip_{g}"])
        self._total.setData(x, h["grip_total"])
