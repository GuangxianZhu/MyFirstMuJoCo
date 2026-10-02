"""手动控制器：把目标角度送给位置执行器，并支持带平滑过渡的整体换姿态。"""
from __future__ import annotations

import numpy as np

from .base import Controller, Observation


class ManualController(Controller):
    name = "手动"

    def __init__(self, defaults: np.ndarray):
        self._defaults = np.asarray(defaults, dtype=float).copy()
        self.targets = self._defaults.copy()
        self._move: dict | None = None
        # 每当 targets 被"程序"改动（过渡进行中、换姿态）就加 1，界面据此刷新滑块
        self.revision = 0

    @property
    def moving(self) -> bool:
        return self._move is not None

    def reset(self) -> None:
        self.targets = self._defaults.copy()
        self._move = None
        self.revision += 1

    def set_target(self, index: int, value: float) -> None:
        """用户直接拖滑块：取消正在进行的过渡，只改这一个目标。"""
        self._move = None
        self.targets[index] = value

    def move_to(self, goal: dict[int, float], duration: float) -> None:
        """把 goal 里的执行器平滑过渡到目标值（其余保持不变）。duration<=0 表示立即到位。"""
        g = self.targets.copy()
        for i, v in goal.items():
            g[i] = v
        if duration <= 1e-6:
            self.targets = g
            self._move = None
        else:
            self._move = {"start": self.targets.copy(), "goal": g, "dur": float(duration), "t0": None}
        self.revision += 1

    def update(self, obs: Observation) -> np.ndarray:
        m = self._move
        if m is not None:
            if m["t0"] is None:
                m["t0"] = obs.time
            s = (obs.time - m["t0"]) / m["dur"]
            if s >= 1.0:
                self.targets = m["goal"].copy()
                self._move = None
            else:
                u = s * s * (3.0 - 2.0 * s)  # smoothstep
                self.targets = m["start"] * (1.0 - u) + m["goal"] * u
            self.revision += 1
        return self.targets
