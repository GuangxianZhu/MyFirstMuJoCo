"""全手分段触觉：把 MuJoCo 的接触力按"手的各段"汇总成三维力。

每个段（掌心、各指节）得到：
  - normal : 该段所有接触的法向力之和 (N)，永远 >= 0
  - force  : 世界系下作用在该段上的合力 (N, 3 维)，含摩擦分量
  - count  : 接触点个数
  - points : 每个接触点的 (段下标, 世界位置, 作用在该段上的力)，用于画箭头
本模块不依赖 Qt。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np


@dataclass
class TactileReading:
    names: list[str]
    normal: np.ndarray                  # (n,)
    force: np.ndarray                   # (n, 3)
    count: np.ndarray                   # (n,)
    points: list[tuple[int, np.ndarray, np.ndarray]] = field(default_factory=list)

    def total_normal(self) -> float:
        return float(self.normal.sum())

    def group_normal(self, prefix: str) -> float:
        """名字以 prefix 开头的所有段的法向力之和，例如 'index' -> 食指三节之和。"""
        idx = [i for i, n in enumerate(self.names) if n.startswith(prefix)]
        return float(self.normal[idx].sum()) if idx else 0.0


class TactileSensor:
    def __init__(self, model: mujoco.MjModel, segment_bodies: list[str]):
        self.model = model
        self.names = list(segment_bodies)
        self.n = len(self.names)
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in self.names]
        if any(i < 0 for i in ids):
            bad = [n for n, i in zip(self.names, ids) if i < 0]
            raise ValueError(f"触觉段对应的刚体不存在: {bad}")
        self._seg_of_body = np.full(model.nbody, -1, dtype=int)
        for s, b in enumerate(ids):
            self._seg_of_body[b] = s
        self._f6 = np.zeros(6)

    def read(self, data: mujoco.MjData, with_points: bool = False) -> TactileReading:
        m = self.model
        normal = np.zeros(self.n)
        force = np.zeros((self.n, 3))
        count = np.zeros(self.n, dtype=int)
        points: list[tuple[int, np.ndarray, np.ndarray]] = []
        f6 = self._f6

        for ci in range(data.ncon):
            c = data.contact[ci]
            s1 = self._seg_of_body[m.geom_bodyid[c.geom1]]
            s2 = self._seg_of_body[m.geom_bodyid[c.geom2]]
            if s1 < 0 and s2 < 0:
                continue
            mujoco.mj_contactForce(m, data, ci, f6)
            # 接触坐标系：法向沿 geom1 -> geom2。f6[:3] 是该坐标系下 geom1 作用在 geom2 上的力
            fw = np.asarray(c.frame).reshape(3, 3).T @ f6[:3]
            fn = max(float(f6[0]), 0.0)
            if s2 >= 0:
                normal[s2] += fn
                force[s2] += fw
                count[s2] += 1
                if with_points:
                    points.append((s2, np.array(c.pos), fw.copy()))
            if s1 >= 0:
                normal[s1] += fn
                force[s1] -= fw
                count[s1] += 1
                if with_points:
                    points.append((s1, np.array(c.pos), -fw))
        return TactileReading(self.names, normal, force, count, points)
