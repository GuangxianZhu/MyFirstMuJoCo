"""示波器：按控制周期把关键信号写进环形缓冲，供实时曲线和演示统计使用。

不依赖 Qt。缓冲是预分配的 numpy 数组，每次 push 只是几次切片赋值，开销很小。
"""
from __future__ import annotations

import numpy as np


class Scope:
    def __init__(self, n_joints: int, n_seg: int, capacity: int = 6000):
        self.capacity = capacity
        self.n_joints, self.n_seg = n_joints, n_seg
        self._t = np.zeros(capacity)
        self._target = np.zeros((capacity, n_joints))
        self._q = np.zeros((capacity, n_joints))
        self._tau = np.zeros((capacity, n_joints))
        self._seg = np.zeros((capacity, max(n_seg, 1)))
        self._head = 0       # 下一个写入位置
        self._size = 0

    def clear(self) -> None:
        self._head = 0
        self._size = 0

    def __len__(self) -> int:
        return self._size

    def push(self, t: float, target: np.ndarray, q: np.ndarray, tau: np.ndarray, seg_normal: np.ndarray | None) -> None:
        h = self._head
        self._t[h] = t
        self._target[h] = target
        self._q[h] = q
        self._tau[h] = tau
        if seg_normal is not None and self.n_seg:
            self._seg[h, : self.n_seg] = seg_normal
        self._head = (h + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def _order(self) -> np.ndarray:
        if self._size < self.capacity:
            return np.arange(self._size)
        return (np.arange(self.capacity) + self._head) % self.capacity

    def view(self, seconds: float | None = None) -> dict[str, np.ndarray]:
        """按时间顺序返回数据；seconds 不为 None 时只取最近这么多秒。"""
        idx = self._order()
        if idx.size == 0:
            empty = np.zeros(0)
            return {"t": empty, "target": np.zeros((0, self.n_joints)), "q": np.zeros((0, self.n_joints)),
                    "tau": np.zeros((0, self.n_joints)), "seg": np.zeros((0, self.n_seg))}
        t = self._t[idx]
        if seconds is not None:
            keep = t >= t[-1] - seconds
            idx, t = idx[keep], t[keep]
        return {
            "t": t,
            "target": self._target[idx],
            "q": self._q[idx],
            "tau": self._tau[idx],
            "seg": self._seg[idx][:, : self.n_seg],
        }
