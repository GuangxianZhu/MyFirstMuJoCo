"""场景附加配置：协同、预设手势、相机等"与模型相关"的信息。

约定：模型 foo.xml 旁边如果有 foo_extras.py，并且定义了 create() -> SceneExtras，
就会被自动加载；没有则返回 None（例如机械臂场景）。配置里只用关节名，不依赖 Qt。
"""
from __future__ import annotations

import importlib.util
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from .demo import Demo


@dataclass
class SynergySpec:
    key: str
    label: str
    lo: float
    hi: float
    default: float = 0.0


@dataclass
class SceneExtras:
    fingers: list[str]                                              # 节奏动作的手指顺序（拇指在前）
    synergies: list[SynergySpec]
    synergy_pose: Callable[[dict[str, float]], dict[str, float]]    # 协同取值 -> {关节名: 弧度}
    gestures: dict[str, dict[str, float]]                           # 手势名 -> {关节名: 弧度}
    curl_pose: Callable[[str, float], dict[str, float]]             # (手指名, 弯曲度0~1) -> {关节名: 弧度}
    camera: dict | None = None                                      # 初始相机: azimuth/elevation/distance/lookat
    labels: dict[str, str] = field(default_factory=dict)            # 界面显示用的中文名（组名/关节名/手势名/触觉段）
    impedance: dict | None = None                                   # 力矩关节的阻抗默认值: k / d / tau_limit
    tactile_segments: list[str] | None = None                       # 触觉段对应的刚体名（掌心、各指节）
    demos: list[Callable[[], Demo]] = field(default_factory=list)   # 演示工厂（每次运行新建一个实例）


def load_extras(xml_path: str | Path) -> SceneExtras | None:
    path = Path(xml_path)
    cfg = path.with_name(f"{path.stem}_extras.py")
    if not cfg.exists():
        return None
    spec = importlib.util.spec_from_file_location(f"_scene_extras_{path.stem}", cfg)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.create()
