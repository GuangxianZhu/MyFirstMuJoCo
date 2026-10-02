"""演示面板：选择演示、开始/停止、运行状态、结束总结，以及可叠加对比的曲线。"""
from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from ..core.demo import Demo
from ..core.sim import Simulation
from .scope_panel import PALETTE, style_plot


class DemoPanel(QWidget):
    startRequested = Signal(object)   # 参数：Demo 实例
    stopRequested = Signal()

    def __init__(self, sim: Simulation, parent: QWidget | None = None):
        super().__init__(parent)
        assert sim.extras is not None
        self.sim = sim
        self._factories = list(sim.extras.demos)
        self._titles = [f.title for f in self._factories]
        self._shown: Demo | None = None      # 当前图/总结对应的演示实例

        self.combo = QComboBox()
        for f in self._factories:
            self.combo.addItem(f.title)
        self.start_btn = QPushButton("开始演示")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)

        self.desc = QLabel()
        self.desc.setWordWrap(True)
        self.desc.setStyleSheet("color: #aab1bc;")
        self.status = QLabel("")
        self.status.setStyleSheet("color: #e8b84a;")
        self.summary = QLabel("")
        self.summary.setWordWrap(True)
        self.summary.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

        left = QVBoxLayout()
        row = QHBoxLayout()
        row.addWidget(self.combo, 1)
        row.addWidget(self.start_btn)
        row.addWidget(self.stop_btn)
        left.addLayout(row)
        left.addWidget(self.desc)
        left.addWidget(self.status)
        left.addWidget(self.summary)
        left.addStretch(1)

        self.plot = pg.PlotWidget()
        style_plot(self.plot, "", "")
        self.plot.setLabel("bottom", "时间 (s)", color="#9aa1ac")
        self.plot.addLegend(offset=(8, 4))

        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addLayout(left, 2)
        lay.addWidget(self.plot, 3)

        self.combo.currentIndexChanged.connect(self._on_choose)
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self.stopRequested)
        self._on_choose()

    # ------------------------------------------------------------------ 交互
    def _available(self, factory) -> bool:
        return not getattr(factory, "needs_object", False) or self.sim.grasp is not None

    def _on_choose(self, _=None) -> None:
        f = self._factories[self.combo.currentIndex()]
        ok = self._available(f)
        self.desc.setText(f.description if ok else f.description + "\n\n⚠ 需要 hand_table 场景：python main.py --model app/scenes/hand/hand_table.xml")
        self.start_btn.setEnabled(ok and self.sim.demo is None)
        self.start_btn.setToolTip("" if ok else "需要 hand_table 场景")

    def _on_start(self) -> None:
        f = self._factories[self.combo.currentIndex()]
        if not self._available(f):
            return
        demo = f()
        self._shown = demo
        self.summary.setText("")
        self.plot.clear()
        self.plot.setTitle(demo.result_title, color="#c8ccd2", size="10pt")
        self.plot.setLabel("left", demo.result_ylabel, color="#9aa1ac")
        self.startRequested.emit(demo)

    # ------------------------------------------------------------------ 刷新（主窗口每隔几帧调用）
    def refresh(self) -> None:
        running = self.sim.demo
        self.start_btn.setEnabled(running is None and self._available(self._factories[self.combo.currentIndex()]))
        self.stop_btn.setEnabled(running is not None)
        self.combo.setEnabled(running is None)

        if running is not None:
            self._shown = running
            self.status.setText("运行中：" + running.progress())
            self._draw(running)
            return
        # 刚结束（自然结束或被停止）：显示总结
        done = self.sim.last_demo
        if done is not None and done is self._shown and not self.summary.text():
            self.status.setText("演示结束" if done.finished else "已停止")
            self.summary.setText(done.summary())
            self._draw(done)

    def _draw(self, demo: Demo) -> None:
        self.plot.clear()
        for k, (label, t, y) in enumerate(demo.result_curves()):
            self.plot.plot(t, y, pen=pg.mkPen(PALETTE[k % len(PALETTE)], width=2), name=label)
