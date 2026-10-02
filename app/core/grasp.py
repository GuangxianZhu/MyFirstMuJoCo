"""抓取指标：只统计"手 ↔ 目标物体"的接触，给出握力、抬升、位移/打滑、接触统计和一个判定状态。

每个控制周期由 Simulation 调用一次 update()（在读完触觉之后）。本模块不依赖 Qt。

判定状态（互斥，按优先级从上到下）：
    碎了   易碎物上任一手指的法向力超过 fragile_limit 并持续 ≥ 20 ms（锁存到 reset）
    滑落   曾经抬起 ≥ 2 cm，现在抬升 < 1 cm 且总握力 < 0.1 N 持续 ≥ 0.2 s
    抓牢   抬升 ≥ 5 cm，最近 1 s 内打滑的变化 ≤ 4 mm，总握力 ≥ 0.5 N，至少两处（手指/掌心）接触
    抬起   抬升 ≥ 2 cm 且总握力 ≥ 0.1 N
    接触   总握力 ≥ 0.05 N
    未接触 其余
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from .tactile import TactileReading

# 手指/掌心分组（触觉段名的前缀），界面和指标里的顺序
GROUPS = ["thumb", "index", "middle", "ring", "little", "palm"]

STATES = ["未接触", "接触", "抬起", "抓牢", "滑落", "碎了"]

BROKEN_RGBA = (0.85, 0.08, 0.08, 1.0)

# history() 里的数值项
SERIES = [
    "lift", "obj_speed", "grip_total", *[f"grip_{g}" for g in GROUPS],
    "n_contacts", "n_segments", "displacement", "slip", "slip_rate", "finger_max",
]


@dataclass(frozen=True)
class Thresholds:
    """判定用的全部阈值（单位：N、m、s）。"""
    establish_grip: float = 0.5        # "建立抓取"：总握力 ≥ 此值
    establish_fingers: int = 2         # 且接触的手指/掌心 ≥ 此数
    establish_hold: float = 0.05       # 且持续 ≥ 此时长
    lifted: float = 0.02               # 抬起：抬升 ≥ 2 cm
    lifted_grip: float = 0.1           # 且总握力 ≥ 0.1 N
    held_lift: float = 0.05            # 抓牢：抬升 ≥ 5 cm
    held_slip: float = 0.004           # 且最近 held_window 内打滑的变化量（max − min）≤ 4 mm
    held_window: float = 1.0
    held_grip: float = 0.5             # 且总握力 ≥ 0.5 N
    held_fingers: int = 2              # 且接触的手指/掌心 ≥ 2
    contact_grip: float = 0.05         # 接触：总握力 ≥ 0.05 N
    drop_lift: float = 0.01            # 滑落：曾抬起后抬升 < 1 cm
    drop_grip: float = 0.1             # 且总握力 < 0.1 N
    drop_hold: float = 0.2             # 持续 ≥ 0.2 s
    fragile_hold: float = 0.02         # 易碎：单指力超限持续 ≥ 20 ms 才算碎（过滤接触瞬间的尖峰）
    slip_rate_window: float = 0.05     # 打滑速度：50 ms 窗口的平均导数


THRESHOLDS = Thresholds()


def body_id(model: mujoco.MjModel, name: str) -> int:
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if b < 0:
        raise ValueError(f"刚体不存在: {name}")
    return b


def pair_normal_force(model: mujoco.MjModel, data: mujoco.MjData, bodies_a: set[int], bodies_b: set[int]) -> float:
    """两组刚体之间所有接触的法向力之和 (N)。例如 物体 ↔ 桌面(世界刚体 0)。"""
    f6 = np.zeros(6)
    total = 0.0
    for ci in range(data.ncon):
        c = data.contact[ci]
        b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
        if (b1 in bodies_a and b2 in bodies_b) or (b2 in bodies_a and b1 in bodies_b):
            mujoco.mj_contactForce(model, data, ci, f6)
            total += max(float(f6[0]), 0.0)
    return total


class _Ring:
    """定长环形缓冲：每条记录是 SERIES + "t" 的一组数。"""

    def __init__(self, keys: list[str], capacity: int):
        self.keys = ["t", *keys]
        self.capacity = capacity
        self._buf = np.zeros((capacity, len(self.keys)))
        self._head = 0
        self._size = 0

    def clear(self) -> None:
        self._head = self._size = 0

    def push(self, row: list[float]) -> None:
        self._buf[self._head] = row
        self._head = (self._head + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def view(self, seconds: float | None) -> dict[str, np.ndarray]:
        if self._size < self.capacity:
            idx = np.arange(self._size)
        else:
            idx = (np.arange(self.capacity) + self._head) % self.capacity
        data = self._buf[idx]
        if seconds is not None and data.shape[0]:
            data = data[data[:, 0] >= data[-1, 0] - seconds]
        return {k: data[:, i].copy() for i, k in enumerate(self.keys)}


class GraspMonitor:
    def __init__(self, sim, object_body: str, fragile_limit: float | None = None,
                 thresholds: Thresholds = THRESHOLDS, capacity: int = 6000):
        if sim.tactile is None:
            raise RuntimeError("抓取指标需要触觉段配置（tactile_segments）")
        self.sim = sim
        self.th = thresholds
        m = sim.model
        self.object_name = object_body
        self.body = body_id(m, object_body)
        self.fragile_limit = None if fragile_limit is None else float(fragile_limit)
        self._palm = body_id(m, "palm")
        self._geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == self.body]
        self._rgba0 = m.geom_rgba[self._geoms].copy()      # 原色：碎了变红，reset 时还原
        self._group_idx = {g: [i for i, n in enumerate(sim.tactile.names) if n.startswith(g)] for g in GROUPS}
        self._hist = _Ring(SERIES, capacity)
        self._vel6 = np.zeros(6)
        self.reset()

    # ------------------------------------------------------------------ 生命周期
    def reset(self) -> None:
        """清零所有累计量；物体的静止高度取当前位置（sim.reset() / reset_object() 之后调用）。"""
        d = self.sim.data
        self.restore_color()
        self.z_rest = float(d.xpos[self.body][2])
        self.reading: TactileReading = self.sim.tactile.read_against(d, {self.body})
        self.state = "未接触"
        self.broken = False
        self.t_ref: float | None = None            # "建立抓取"的时刻
        self._p_ref = np.zeros(3)                   # t_ref 时物体的世界位置
        self._rel_ref = np.zeros(3)                 # t_ref 时物体在掌坐标系里的位置
        self._est_since: float | None = None        # 满足"建立抓取"条件的开始时刻（及当时的位置）
        self._est_p = np.zeros(3)
        self._est_rel = np.zeros(3)
        self._ever_lifted = False
        self._drop_since: float | None = None
        self._over_since: float | None = None
        self._slip_hist: deque[tuple[float, float]] = deque()
        self._hist.clear()
        self._m = self._empty_metrics()

    def release(self) -> None:
        """序列器主动松手放下物体时调用：此后物体回到桌面不算"滑落"（"曾经抬起"清零）。"""
        self._ever_lifted = False
        self._drop_since = None

    def restore_color(self) -> None:
        self.sim.model.geom_rgba[self._geoms] = self._rgba0

    # ------------------------------------------------------------------ 每个控制周期
    def update(self) -> None:
        sim, th = self.sim, self.th
        m, d = sim.model, sim.data
        t = float(d.time)
        r = sim.tactile.read_against(d, {self.body})
        self.reading = r

        p = d.xpos[self.body].copy()
        # 速度：mj_objectVelocity 给出刚体原点（这里就是物体中心）在世界系下的线速度（瞬时值，不做差分）
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.body, self._vel6, 0)
        speed = float(np.linalg.norm(self._vel6[3:]))
        lift = float(p[2] - self.z_rest)

        grips = {g: float(r.normal[idx].sum()) if idx else 0.0 for g, idx in self._group_idx.items()}
        grip_total = r.total_normal()
        touching = [g for g, idx in self._group_idx.items() if idx and r.count[idx].sum() > 0]
        n_contacts = int(r.count.sum())
        n_segments = int((r.count > 0).sum())
        finger_max = max(grips.values())

        # 物体在掌坐标系里的位置：手带着物体一起动不算打滑，物体相对手移动才算
        R = d.xmat[self._palm].reshape(3, 3)
        rel = R.T @ (p - d.xpos[self._palm])

        # --- 建立抓取（t_ref）---
        if self.t_ref is None:
            ok = grip_total >= th.establish_grip and len(touching) >= th.establish_fingers
            if not ok:
                self._est_since = None
            elif self._est_since is None:
                self._est_since, self._est_p, self._est_rel = t, p.copy(), rel.copy()
            elif t - self._est_since >= th.establish_hold - 1e-9:
                # 参考位置取条件刚满足时的位置，确认期间的位移也算进来
                self.t_ref, self._p_ref, self._rel_ref = self._est_since, self._est_p, self._est_rel
        if self.t_ref is not None:
            displacement = float(np.linalg.norm(p - self._p_ref))
            slip = float(np.linalg.norm(rel - self._rel_ref))
        else:
            displacement = slip = 0.0

        # --- 打滑速度：slip 在最近 50 ms 内的平均变化率 ---
        sh = self._slip_hist
        sh.append((t, slip))
        while len(sh) > 1 and t - sh[0][0] > th.slip_rate_window + 1e-9:
            sh.popleft()
        slip_rate = (sh[-1][1] - sh[0][1]) / (sh[-1][0] - sh[0][0]) if len(sh) > 1 and sh[-1][0] > sh[0][0] else 0.0

        # --- 易碎 ---
        if self.fragile_limit is not None and not self.broken:
            if finger_max > self.fragile_limit:
                if self._over_since is None:
                    self._over_since = t
                if t - self._over_since >= th.fragile_hold - 1e-9:
                    self.broken = True
                    m.geom_rgba[self._geoms] = BROKEN_RGBA
            else:
                self._over_since = None

        # --- 滑落计时 ---
        if lift >= th.lifted:
            self._ever_lifted = True
        if self._ever_lifted and lift < th.drop_lift and grip_total < th.drop_grip:
            if self._drop_since is None:
                self._drop_since = t
        else:
            self._drop_since = None

        self._m = {
            "object": self.object_name,
            "obj_pos": (float(p[0]), float(p[1]), float(p[2])),
            "obj_speed": speed,
            "lift": lift,
            "grip_total": grip_total,
            **{f"grip_{g}": v for g, v in grips.items()},
            "finger_max": finger_max,
            "n_contacts": n_contacts,
            "n_segments": n_segments,
            "fingers_touching": touching,
            "displacement": displacement,
            "slip": slip,
            "slip_rate": float(slip_rate),
            "broken": self.broken,
            "fragile_limit": self.fragile_limit,
            "t_ref": self.t_ref,
        }
        self._hist.push([t] + [float(self._m[k]) for k in SERIES])
        self.state = self._judge(t, lift, grip_total, len(touching))
        self._m["state"] = self.state

    def _judge(self, t: float, lift: float, grip: float, n_touch: int) -> str:
        th = self.th
        if self.broken:
            return "碎了"
        if self._drop_since is not None and t - self._drop_since >= th.drop_hold - 1e-9:
            return "滑落"
        if lift >= th.held_lift and grip >= th.held_grip and n_touch >= th.held_fingers:
            # slip 是从 t_ref 起的累计量。闭合挤压时物体在桌面上会被推着对中几毫米（发生在 t_ref 之后、抬起之前），
            # 若按累计值判，之后拿得再稳也永远不是"抓牢"；所以这里看最近 1 s 内 slip 变了多少（物体是否还在相对手移动）
            recent = self._hist.view(th.held_window)["slip"]
            if recent.size and float(recent.max() - recent.min()) <= th.held_slip:
                return "抓牢"
        if lift >= th.lifted and grip >= th.lifted_grip:
            return "抬起"
        if grip >= th.contact_grip:
            return "接触"
        return "未接触"

    # ------------------------------------------------------------------ 查询
    def _empty_metrics(self) -> dict:
        p = self.sim.data.xpos[self.body]
        return {
            "object": self.object_name, "obj_pos": (float(p[0]), float(p[1]), float(p[2])),
            **{k: 0.0 for k in SERIES}, "n_contacts": 0, "n_segments": 0, "fingers_touching": [],
            "broken": False, "fragile_limit": self.fragile_limit, "t_ref": None, "state": "未接触",
        }

    def metrics(self) -> dict:
        """最近一次 update 的全部指标（数值是 float/int，另有 state/object 字符串、fingers_touching 名字列表）。"""
        return dict(self._m)

    def history(self, seconds: float | None = None) -> dict[str, np.ndarray]:
        """按时间顺序的指标曲线：键 = "t" + SERIES。seconds 不为 None 时只取最近这么多秒。"""
        return self._hist.view(seconds)
