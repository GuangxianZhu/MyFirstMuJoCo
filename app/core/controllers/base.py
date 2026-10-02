"""控制器接口。

设计成"单片机视角"：控制器以固定的控制周期被调用，只看到传感器读数，
返回执行器指令。PD、轨迹、单片机逻辑都实现同一个接口，界面里可切换。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Observation:
    """控制器每个控制周期拿到的传感器读数（对应受驱动的关节，顺序同 ActuatorSpec）。"""

    time: float
    q: np.ndarray    # 关节角度 (rad)
    qd: np.ndarray   # 关节角速度 (rad/s)


class Controller:
    name = "base"

    def reset(self) -> None:
        """仿真重置时调用。"""

    def update(self, obs: Observation) -> np.ndarray:
        """返回执行器指令数组，长度 = 执行器数量。"""
        raise NotImplementedError
