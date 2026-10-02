"""关节阻抗控制：把"目标角度"换算成力矩执行器的力矩指令。

    tau = K * (q_target - q) - D * qd        再限幅到 ±tau_limit

控制律按控制周期离散采样（零阶保持），和单片机里的定时控制环一致。
位置执行器（例如手腕）原样透传目标，不经过这里。本模块不依赖 Qt。
"""
from __future__ import annotations

import numpy as np

from .params import ActuatorSpec

# 预设：(名称, K 倍率, D 倍率, 力矩上限 N·m)
PRESETS: list[tuple[str, float, float, float]] = [
    ("柔顺（阻抗）", 0.5, 0.7, 0.30),
    ("默认", 1.0, 1.0, 0.60),
    ("刚硬（位置控制）", 5.0, 2.5, 1.00),
]

DEFAULT_K = 1.5       # N·m/rad
DEFAULT_D = 0.08      # N·m·s/rad
DEFAULT_LIMIT = 0.60  # N·m


class JointImpedance:
    def __init__(
        self,
        actuators: list[ActuatorSpec],
        k: float = DEFAULT_K,
        d: float = DEFAULT_D,
        tau_limit: float = DEFAULT_LIMIT,
        hardware_limit: np.ndarray | None = None,
    ):
        self.mask = np.array([a.torque for a in actuators], dtype=bool)   # 哪些执行器是力矩型
        n = len(actuators)
        self.k_base = np.where(self.mask, k, 0.0)
        self.d_base = np.where(self.mask, d, 0.0)
        self.limit_base = tau_limit
        self.hardware_limit = hardware_limit if hardware_limit is not None else np.full(n, np.inf)

        self.k_scale = 1.0
        self.d_scale = 1.0
        self.tau_limit = float(tau_limit)

        self.last_tau = np.zeros(n)

    @property
    def active(self) -> bool:
        return bool(self.mask.any())

    @property
    def k(self) -> np.ndarray:
        return self.k_base * self.k_scale

    @property
    def d(self) -> np.ndarray:
        return self.d_base * self.d_scale

    def apply_preset(self, k_scale: float, d_scale: float, tau_limit: float) -> None:
        self.k_scale, self.d_scale, self.tau_limit = float(k_scale), float(d_scale), float(tau_limit)

    def reset(self) -> None:
        self.apply_preset(1.0, 1.0, self.limit_base)

    def compute(self, targets: np.ndarray, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        """返回每个执行器的指令（按滑块顺序）：力矩型 = 力矩，位置型 = 目标透传。"""
        tau = self.k * (targets - q) - self.d * qd
        lim = np.minimum(self.tau_limit, self.hardware_limit)
        tau = np.clip(tau, -lim, lim)
        out = np.where(self.mask, tau, targets)
        self.last_tau = np.where(self.mask, tau, 0.0)
        return out
