"""桌面抓取场景 hand_table.xml 的附加配置：在手的配置基础上换相机、登记抓取指标和抓取演示。"""
from __future__ import annotations

from app.core.scene_extras import SceneExtras
from app.scenes.hand.hand_extras import create as _create_hand

# 默认的抓取指标对象：基线方块（演示会用 sim.set_grasp_object 切换）
GRASP = {"object": "cube", "fragile_limit": None}


def _demos():
    from app.scenes.hand.demos import CompareGraspDemo, FragilePickPlaceDemo, ImpedanceCompareDemo
    return [FragilePickPlaceDemo, CompareGraspDemo, ImpedanceCompareDemo]


def create() -> SceneExtras:
    ex = _create_hand()
    ex.camera = {"azimuth": 125.0, "elevation": -26.0, "distance": 0.55, "lookat": (0.12, 0.0, 0.18)}
    ex.demos = _demos()
    ex.grasp = dict(GRASP)
    return ex
