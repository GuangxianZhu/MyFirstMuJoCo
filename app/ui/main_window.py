"""主窗口：左侧手势/协同/关节，中间 3D 画面，右侧仿真状态和物理参数。"""
from __future__ import annotations

import time
from pathlib import Path

from PySide6.QtCore import QTimer, Qt
import mujoco
from PySide6.QtWidgets import (
    QCheckBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..core.renderer import SceneRenderer
from ..core.sim import Simulation
from .demo_panel import DemoPanel
from .hand_panel import HandPanel
from .impedance_panel import ImpedancePanel
from .panels import JointPanel, StatusPanel
from .physics_panel import PhysicsPanel
from .scope_panel import ScopePanel
from .tactile_view import TactileView
from .viewport import Viewport

FRAME_MS = 16  # 约 60 fps
LEFT_COLUMN_WIDTH = 410
RIGHT_COLUMN_WIDTH = 360
SCOPE_EVERY = 4        # 曲线每隔几帧刷新一次（约 15 Hz）
DEMO_EVERY = 8         # 演示面板刷新间隔（约 7 Hz）
BOTTOM_HEIGHT = 300


def _scroll_column(inner: QWidget, width: int) -> QScrollArea:
    area = QScrollArea()
    area.setWidget(inner)
    area.setWidgetResizable(True)
    area.setFrameShape(QFrame.Shape.NoFrame)
    area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    area.setFixedWidth(width)
    return area


class MainWindow(QMainWindow):
    def __init__(self, xml_path: str | Path, start_timer: bool = True):
        super().__init__()
        self.setWindowTitle(f"MuJoCo 演示 - {Path(xml_path).name}")
        self.resize(1560, 860)

        self.sim = Simulation(xml_path)
        extras = self.sim.extras
        self.renderer = SceneRenderer(self.sim.model, camera=extras.camera if extras else None)

        # --- 控件 ---
        self.joint_panel = JointPanel(self.sim.actuators, extras.labels if extras else None)
        self.hand_panel = HandPanel(self.sim) if extras else None
        self.viewport = Viewport()
        self.status_panel = StatusPanel()
        self.status_panel.set_stats(0.0, 0.0, 0.0, self.sim.timestep)
        self.physics_panel = PhysicsPanel(self.sim.physics)
        labels = extras.labels if extras else None
        self.impedance_panel = ImpedancePanel(self.sim) if self.sim.drive.active else None
        self.display_box = self._make_display_box()
        self.tactile_view = TactileView(self.sim.tactile.names, labels) if self.sim.tactile else None
        self.scope_panel = ScopePanel(self.sim, labels)
        self.demo_panel = DemoPanel(self.sim) if extras and extras.demos else None

        # 左栏：手势/协同/节奏（有的话） + 关节滑块，内容多时可滚动
        left_inner = QWidget()
        left_lay = QVBoxLayout(left_inner)
        left_lay.setContentsMargins(0, 0, 6, 0)
        if self.hand_panel:
            left_lay.addWidget(self.hand_panel)
        left_lay.addWidget(self.joint_panel)
        left_lay.addStretch(1)
        self.left_scroll = _scroll_column(left_inner, LEFT_COLUMN_WIDTH)

        # 右栏：状态 + 物理参数
        right_inner = QWidget()
        right_lay = QVBoxLayout(right_inner)
        right_lay.setContentsMargins(0, 0, 6, 0)
        right_lay.addWidget(self.status_panel)
        if self.impedance_panel:
            right_lay.addWidget(self.impedance_panel)
        right_lay.addWidget(self.display_box)
        right_lay.addWidget(self.physics_panel)
        right_lay.addStretch(1)
        self.right_scroll = _scroll_column(right_inner, RIGHT_COLUMN_WIDTH)

        hint = QLabel("左键拖动：旋转    右键拖动：平移    滚轮：缩放")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color: #8a93a0;")

        # 中间：上面 3D 画面，下面是 触觉 / 曲线 / 演示 页签（可拖动分隔条调整高度）
        top = QWidget()
        top_lay = QVBoxLayout(top)
        top_lay.setContentsMargins(0, 0, 0, 0)
        top_lay.addWidget(self.viewport, 1)
        top_lay.addWidget(hint)

        self.tabs = QTabWidget()
        if self.tactile_view:
            self.tabs.addTab(self.tactile_view, "触觉")
        self.tabs.addTab(self.scope_panel, "曲线")
        if self.demo_panel:
            self.tabs.addTab(self.demo_panel, "演示")
        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.splitter.addWidget(top)
        self.splitter.addWidget(self.tabs)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setSizes([560, BOTTOM_HEIGHT])
        self.splitter.setChildrenCollapsible(False)

        root = QHBoxLayout()
        root.addWidget(self.left_scroll)
        root.addWidget(self.splitter, 1)
        root.addWidget(self.right_scroll)
        holder = QWidget()
        holder.setLayout(root)
        self.setCentralWidget(holder)

        # --- 信号 ---
        self.joint_panel.targetChanged.connect(self.sim.set_target)
        self.joint_panel.resetRequested.connect(self._reset_targets)
        self.status_panel.pauseToggled.connect(self._set_paused)
        self.status_panel.resetRequested.connect(self._reset_sim)
        self.status_panel.cameraResetRequested.connect(self.renderer.reset_camera)
        self.viewport.rotateRequested.connect(self.renderer.rotate)
        self.viewport.panRequested.connect(self.renderer.pan)
        self.viewport.zoomRequested.connect(self.renderer.zoom)
        if self.demo_panel:
            self.demo_panel.startRequested.connect(self._start_demo)
            self.demo_panel.stopRequested.connect(self._stop_demo)

        # --- 主循环 ---
        self._rev = -1                       # 已同步到滑块的 manual.revision
        self._last_wall = time.perf_counter()
        self._fps = 0.0
        self._stat_wall = self._last_wall
        self._stat_sim = 0.0
        self._rtf = 0.0
        self._frame = 0
        self._demo_was_running = False
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self.tick)
        if start_timer:
            self._timer.start(FRAME_MS)
        self.tick()

    # ------------------------------------------------------------------ 显示选项
    def _make_display_box(self) -> QGroupBox:
        box = QGroupBox("显示")
        lay = QVBoxLayout(box)
        self.cb_points = QCheckBox("接触点")
        self.cb_forces = QCheckBox("接触力箭头")
        for cb, flag in ((self.cb_points, mujoco.mjtVisFlag.mjVIS_CONTACTPOINT),
                         (self.cb_forces, mujoco.mjtVisFlag.mjVIS_CONTACTFORCE)):
            cb.toggled.connect(lambda on, f=flag: self._set_vis(f, on))
            lay.addWidget(cb)
        return box

    def _set_vis(self, flag, on: bool) -> None:
        self.renderer.option.flags[flag] = bool(on)

    # ------------------------------------------------------------------ 动作
    def _start_demo(self, demo) -> None:
        if self.hand_panel:
            self.hand_panel.start.setChecked(False)   # 演示会接管控制，先停掉节奏动作
        self.sim.paused = False
        self.status_panel.set_paused(False)
        self.sim.start_demo(demo)
        if getattr(demo, "camera", None):
            self.renderer.set_camera(demo.camera)
        self._after_reset()

    def _stop_demo(self) -> None:
        self.sim.stop_demo()
        if self.impedance_panel:
            self.impedance_panel.sync()

    def _set_paused(self, paused: bool) -> None:
        self.sim.paused = paused

    def _reset_targets(self) -> None:
        """只把目标恢复默认，不重置仿真状态。"""
        self.sim.manual.reset()
        if self.hand_panel:
            self.hand_panel.refresh()

    def _reset_sim(self) -> None:
        self.sim.stop_demo()
        self.sim.reset()
        self._after_reset()

    def _after_reset(self) -> None:
        if self.hand_panel:
            self.hand_panel.refresh()
        if self.impedance_panel:
            self.impedance_panel.sync()
        self._stat_sim = 0.0
        self._stat_wall = time.perf_counter()

    # ------------------------------------------------------------------ 每帧
    def tick(self) -> None:
        now = time.perf_counter()
        wall_dt = min(now - self._last_wall, 0.1)  # 窗口被拖动卡住后不要一次追太多
        self._last_wall = now

        self.sim.advance(wall_dt)
        # 手势过渡、重置等由程序改动了目标时，把滑块同步过去
        if self.sim.manual.revision != self._rev:
            self._rev = self.sim.manual.revision
            self.joint_panel.set_targets(self.sim.manual.targets)
        self.viewport.set_frame(self.renderer.render(self.sim.data))
        self.joint_panel.set_actual(self.sim.joint_angles())

        self._frame += 1
        if self.tactile_view is not None and self.tabs.currentWidget() is self.tactile_view:
            self.tactile_view.set_reading(self.sim.tactile_reading)
        if self._frame % SCOPE_EVERY == 0 and self.tabs.currentWidget() is self.scope_panel:
            self.scope_panel.refresh()
        if self._frame % DEMO_EVERY == 0:
            if self.demo_panel is not None and (self.sim.demo is not None or self.tabs.currentWidget() is self.demo_panel):
                self.demo_panel.refresh()
            running = self.sim.demo is not None
            if self.impedance_panel is not None and (running or self._demo_was_running):
                self.impedance_panel.sync()      # 演示会换阻抗参数（结束时还原），面板跟着变
            if self.demo_panel is not None and self._demo_was_running and not running:
                self.demo_panel.refresh()        # 刚结束：立刻出总结
            self._demo_was_running = running

        if wall_dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / wall_dt) if self._fps else 1.0 / wall_dt
        span = now - self._stat_wall
        if span >= 0.5:
            self._rtf = (self.sim.time - self._stat_sim) / span if not self.sim.paused else 0.0
            self._stat_wall, self._stat_sim = now, self.sim.time
        self.status_panel.set_stats(self.sim.time, self._fps, self._rtf, self.sim.timestep)

    def closeEvent(self, event) -> None:
        self._timer.stop()
        self.renderer.close()
        super().closeEvent(event)
