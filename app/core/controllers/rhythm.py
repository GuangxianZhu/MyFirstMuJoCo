"""手指节奏动作：波浪 / 数数 / 敲击。

rhythm_curls() 是纯函数：给定时间和参数，返回每根手指的弯曲度 0(伸直)~1(握紧)，
便于单独测试。RhythmController 把弯曲度换成关节目标，叠加在手动控制器的目标之上，
手腕等其他执行器仍由手动目标决定。
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from .base import Controller, Observation
from .manual import ManualController

PATTERNS: list[tuple[str, str]] = [
    ("wave", "波浪"),
    ("count", "数数"),
    ("tap", "敲击"),
]

# 数数序列：伸出的手指数（1=食指，2=食指+中指，…，5=拇指也伸出）
COUNT_SEQUENCE = [1, 2, 3, 4, 5, 4, 3, 2]
# 手指顺序：拇指, 食指, 中指, 无名指, 小指；数数时伸出的先后顺序
COUNT_ORDER = [1, 2, 3, 4, 0]
TAP_BASELINE = 0.10   # 敲击时手指的待机弯曲度
TAP_WIDTH = 0.35      # 一次敲击占一个周期的比例
COUNT_TRANSITION = 0.35  # 数数每一步中用于过渡的比例


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def _count_vector(step: int) -> np.ndarray:
    ext = COUNT_SEQUENCE[step % len(COUNT_SEQUENCE)]
    v = np.ones(5)
    for k in range(ext):
        v[COUNT_ORDER[k]] = 0.0
    return v


def rhythm_curls(pattern: str, t: float, freq: float, amp: float, lag: float, n: int = 5) -> np.ndarray:
    """返回 n 根手指在时刻 t 的弯曲度（0~1）。

    pattern: wave / count / tap
    freq:    周期频率 Hz（count 时表示"完整 1→5→1 一轮"的频率）
    amp:     幅度 0~1（count 忽略）
    lag:     相邻手指的相位差，单位为"周期的比例"（count 忽略）
    """
    freq = max(float(freq), 1e-6)
    out = np.zeros(n)
    if pattern == "wave":
        for k in range(n):
            phase = freq * t - lag * k
            out[k] = amp * 0.5 * (1.0 - np.cos(2.0 * np.pi * phase))
    elif pattern == "tap":
        for k in range(n):
            p = (freq * t - lag * k) % 1.0
            pulse = 0.5 * (1.0 - np.cos(2.0 * np.pi * p / TAP_WIDTH)) if p < TAP_WIDTH else 0.0
            out[k] = TAP_BASELINE + (1.0 - TAP_BASELINE) * amp * pulse
    elif pattern == "count":
        step_time = 1.0 / (len(COUNT_SEQUENCE) * freq)
        step = int(t // step_time)
        frac = (t - step * step_time) / step_time
        prev = _count_vector(step - 1) if step > 0 else _count_vector(step)
        cur = _count_vector(step)
        u = _smoothstep(frac / COUNT_TRANSITION)
        out = prev * (1.0 - u) + cur * u
        if n != 5:
            out = np.resize(out, n)
    else:
        raise ValueError(f"未知节奏模式: {pattern}")
    return np.clip(out, 0.0, 1.0)


class RhythmController(Controller):
    name = "节奏"

    def __init__(
        self,
        manual: ManualController,
        fingers: list[str],
        curl_targets: Callable[[str, float], dict[int, float]],
        pattern: str = "wave",
        freq: float = 0.8,
        amp: float = 1.0,
        lag: float = 0.12,
    ):
        self.manual = manual
        self.fingers = fingers
        self.curl_targets = curl_targets   # (手指名, 弯曲度) -> {执行器下标: 目标}
        self.pattern = pattern
        self.freq = freq
        self.amp = amp
        self.lag = lag
        self._t0: float | None = None

    def reset(self) -> None:
        self._t0 = None
        self.manual.reset()

    def update(self, obs: Observation) -> np.ndarray:
        base = self.manual.update(obs).copy()
        if self._t0 is None:
            self._t0 = obs.time
        curls = rhythm_curls(self.pattern, obs.time - self._t0, self.freq, self.amp, self.lag, len(self.fingers))
        for name, c in zip(self.fingers, curls):
            for i, v in self.curl_targets(name, float(c)).items():
                base[i] = v
        return base
