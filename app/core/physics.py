"""可在线调节的物理参数：重力、关节阻尼、连杆质量、位置执行器 kp/kv。

本模块不依赖 Qt。每个"向量参数"保存出厂值 base，支持：
  - 全局倍率：values = base * scale（覆盖单独设置）
  - 单独设置：只改其中一个元素的绝对值
"""
from __future__ import annotations

from typing import Callable

import mujoco
import numpy as np

from .params import ActuatorSpec

GRAVITY_PRESETS: list[tuple[str, float]] = [
    ("地球", 9.81),
    ("月球", 1.62),
    ("火星", 3.71),
    ("木星", 24.79),
    ("失重", 0.0),
]
GRAVITY_MAX = 30.0
MIN_MASS = 1e-3  # kg，避免质量为 0 导致惯量矩阵奇异


class VectorParam:
    """一组同类参数（例如每个关节的阻尼）。"""

    def __init__(
        self,
        key: str,
        title: str,
        unit: str,
        labels: list[str],
        base: np.ndarray,
        apply: Callable[[np.ndarray], None],
        decimals: int,
        step: float,
        min_value: float = 0.0,
    ):
        self.key = key
        self.title = title
        self.unit = unit
        self.labels = labels
        self.base = np.asarray(base, dtype=float).copy()
        self.values = self.base.copy()
        self.scale = 1.0
        self.decimals = decimals
        self.step = step
        self.min_value = min_value
        self._apply = apply

    @property
    def abs_max(self) -> float:
        """单独调节时的上限：出厂最大值的 6 倍（全局倍率最大 5x，留一点余量）。"""
        m = float(self.base.max()) if self.base.size else 0.0
        return 6.0 * m if m > 0 else 1.0

    def max_for(self, i: int) -> float:
        """第 i 个元素单独调节时的上限：它自己出厂值的 6 倍。
        各元素量级差很多时（手腕平移 kp=3000，转动 kp=60）不能共用一个上限，否则小的那些滑块根本拖不动。"""
        b = float(self.base[i])
        return 6.0 * b if b > 0 else self.abs_max

    def set_scale(self, s: float) -> None:
        self.scale = float(s)
        self.values = np.maximum(self.base * self.scale, self.min_value)
        self._apply(self.values)

    def set_value(self, i: int, v: float) -> None:
        self.values[i] = max(float(v), self.min_value)
        self._apply(self.values)

    def reset(self) -> None:
        self.scale = 1.0
        self.values = self.base.copy()
        self._apply(self.values)


class PhysicsParams:
    def __init__(self, model: mujoco.MjModel, actuators: list[ActuatorSpec], dof_adr: np.ndarray):
        self.model = model
        # mj_setConst 会改写传入的 MjData，所以用一份草稿数据，避免打乱正在运行的仿真状态
        self._scratch = mujoco.MjData(model)

        g = np.array(model.opt.gravity, dtype=float)
        norm = float(np.linalg.norm(g))
        self.base_gravity = norm
        self._g_dir = g / norm if norm > 1e-9 else np.array([0.0, 0.0, -1.0])

        self.groups: list[VectorParam] = []
        self._build_damping(actuators, dof_adr)
        self._build_mass()
        self._build_gains(actuators)

    # ------------------------------------------------------------------ 重力
    @property
    def gravity(self) -> float:
        return float(np.linalg.norm(self.model.opt.gravity))

    def set_gravity(self, g: float) -> None:
        """只改重力大小，方向保持模型原来的方向。"""
        self.model.opt.gravity[:] = self._g_dir * max(float(g), 0.0)

    # ------------------------------------------------------------------ 构建
    def _name(self, obj: mujoco.mjtObj, idx: int, fallback: str) -> str:
        n = mujoco.mj_id2name(self.model, obj, idx)
        return n if n else fallback

    def _build_damping(self, actuators: list[ActuatorSpec], dof_adr: np.ndarray) -> None:
        if len(actuators) == 0:
            return
        m = self.model
        dofs = np.asarray(dof_adr, dtype=int)

        def apply(values: np.ndarray) -> None:
            m.dof_damping[dofs] = values

        self.groups.append(
            VectorParam(
                "damping", "关节阻尼", "N·m·s/rad",
                [a.joint_name for a in actuators],
                m.dof_damping[dofs], apply, decimals=3, step=0.05,
            )
        )

    def _build_mass(self) -> None:
        m = self.model
        # 有质量、有几何体、且不是焊在世界上的刚体（固定底座改质量没有意义；
        # 没有几何体的"携带体"只是运动学占位，也不列出）
        bodies = np.array(
            [
                b for b in range(1, m.nbody)
                if m.body_mass[b] > 0 and m.body_weldid[b] != 0 and m.body_geomnum[b] > 0
            ],
            dtype=int,
        )
        if bodies.size == 0:
            return
        base_mass = m.body_mass[bodies].copy()
        base_inertia = m.body_inertia[bodies].copy()

        def apply(values: np.ndarray) -> None:
            ratio = values / base_mass
            m.body_mass[bodies] = values
            m.body_inertia[bodies] = base_inertia * ratio[:, None]  # 形状不变时惯量与质量成正比
            mujoco.mj_setConst(m, self._scratch)                    # 同步依赖质量的派生常量

        self.groups.append(
            VectorParam(
                "mass", "连杆质量", "kg",
                [self._name(mujoco.mjtObj.mjOBJ_BODY, int(b), f"body{b}") for b in bodies],
                base_mass, apply, decimals=3, step=0.01, min_value=MIN_MASS,
            )
        )

    def _build_gains(self, actuators: list[ActuatorSpec]) -> None:
        m = self.model
        valid = [
            a for a in actuators
            if m.actuator_gaintype[a.index] == mujoco.mjtGain.mjGAIN_FIXED
            and m.actuator_biastype[a.index] == mujoco.mjtBias.mjBIAS_AFFINE
            and abs(m.actuator_biasprm[a.index, 1] + m.actuator_gainprm[a.index, 0]) < 1e-9  # 位置执行器：bias1 = -kp
        ]
        if not valid:
            return
        idx = np.array([a.index for a in valid], dtype=int)
        labels = [a.joint_name for a in valid]

        def apply_kp(values: np.ndarray) -> None:
            m.actuator_gainprm[idx, 0] = values
            m.actuator_biasprm[idx, 1] = -values

        def apply_kv(values: np.ndarray) -> None:
            m.actuator_biasprm[idx, 2] = -values

        self.groups.append(
            VectorParam("kp", "执行器刚度 kp", "N·m/rad", labels,
                        m.actuator_gainprm[idx, 0], apply_kp, decimals=1, step=1.0)
        )
        self.groups.append(
            VectorParam("kv", "执行器阻尼 kv", "N·m·s/rad", labels,
                        -m.actuator_biasprm[idx, 2], apply_kv, decimals=2, step=0.1)
        )

    # ------------------------------------------------------------------ 重置
    def reset_all(self) -> None:
        self.set_gravity(self.base_gravity)
        for g in self.groups:
            g.reset()
