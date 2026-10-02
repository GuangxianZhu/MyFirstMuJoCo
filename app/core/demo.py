"""演示脚本框架。

一个 Demo 在仿真里按"演示时间 t"驱动手腕/手指/阻抗参数，并在结束后给出文字总结和可叠加对比的曲线。
Simulation 在每个控制周期调用 demo.update(sim, t)。本模块不依赖 Qt。
"""
from __future__ import annotations

import numpy as np


class Demo:
    key = "demo"
    title = "演示"
    description = ""
    camera: dict | None = None   # 演示开始时切换到的相机 {azimuth, elevation, distance, lookat}；None = 不动
    needs_object = False         # True = 需要有物体和抓取指标的场景（sim.grasp 不为 None，例如 hand_table）

    def __init__(self):
        self.finished = False

    # --- 生命周期 ---
    def setup(self, sim) -> None:
        """演示开始前调用一次（此时仿真已重置）。"""

    def update(self, sim, t: float) -> None:
        """每个控制周期调用，t = 演示开始后经过的仿真秒数。结束时把 self.finished 置 True。"""

    def teardown(self, sim) -> None:
        """演示结束或被中止时调用，恢复阻抗参数、把道具藏起来等。"""

    # --- 结果 ---
    def progress(self) -> str:
        """运行中显示的一行状态。"""
        return ""

    def summary(self) -> str:
        """结束后显示的总结文字。"""
        return ""

    def result_curves(self) -> list[tuple[str, np.ndarray, np.ndarray]]:
        """可选：用于叠加对比的曲线 [(标签, t, y)]。"""
        return []

    result_title = ""
    result_ylabel = ""
