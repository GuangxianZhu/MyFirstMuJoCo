"""B4 测试：演示 3「轻拿轻放易碎物」。无需界面。运行 `python tests/test_demo_fragile.py` 或 `pytest`。"""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.grasp import BROKEN_RGBA  # noqa: E402
from app.core.sim import Simulation  # noqa: E402
from app.scenes.hand.demos import FragilePickPlaceDemo  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"

_CACHE = {}


def _finished_demo():
    """整个演示跑一次（约 30 s 仿真），几个测试共用结果。"""
    if "demo" not in _CACHE:
        sim = Simulation(TABLE)
        demo = FragilePickPlaceDemo()
        geoms = [g for g in range(sim.model.ngeom)
                 if sim.model.geom_bodyid[g] == mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "fragile")]
        rgba0 = sim.model.geom_rgba[geoms].copy()
        red_seen = {"v": False}
        sim.start_demo(demo)
        while sim.demo is not None and sim.time < 60.0:
            sim.step_once()
            if demo.trial == 1 and np.allclose(sim.model.geom_rgba[geoms], BROKEN_RGBA):
                red_seen["v"] = True
        _CACHE.update(sim=sim, demo=demo, geoms=geoms, rgba0=rgba0, red_seen=red_seen["v"])
    return _CACHE


def test_registered_in_table_scene_only():
    sim = Simulation(TABLE)
    keys = [f.key for f in sim.extras.demos]
    assert "fragile" in keys
    hand = Simulation(ROOT / "app" / "scenes" / "hand" / "hand.xml")
    assert "fragile" not in [f.key for f in hand.extras.demos]


def test_both_trials_finish_within_60s():
    c = _finished_demo()
    sim, demo = c["sim"], c["demo"]
    assert demo.finished and sim.demo is None
    assert sim.time < 60.0
    assert len(demo.results) == 2
    assert np.all(np.isfinite(sim.data.qpos))


def test_gentle_trial_succeeds():
    r = _finished_demo()["demo"].results[0]
    assert not r["broken"] and r["done"], r
    assert r["lift_end_state"] == "抓牢"
    assert r["place_error"] < 0.008
    assert r["peak_finger_force"] < FragilePickPlaceDemo.FRAGILE_LIMIT
    assert r["peak_table_force"] is not None and r["peak_table_force"] > 0.1


def test_forceful_trial_grips_harder_and_breaks_red():
    c = _finished_demo()
    r0, r1 = c["demo"].results
    assert r1["peak_finger_force"] > r0["peak_finger_force"]
    if r1["broken"]:
        assert c["red_seen"], "判为碎了，但方块没有变红"
        assert r1["final_state"] == "碎了" and r1["failed"]


def test_summary_and_curves():
    demo = _finished_demo()["demo"]
    lines = demo.summary().splitlines()
    assert len(lines) == 2
    assert lines[0].startswith("轻拿轻放") and "成功" in lines[0] and "放置误差" in lines[0] and "落地冲击" in lines[0]
    assert lines[1].startswith("用力抓")
    curves = demo.result_curves()
    assert len(curves) == 3 and curves[2][0].startswith("易碎阈值")
    assert np.allclose(curves[2][2], 1.8)
    for _, t, y in curves[:2]:
        assert t.shape == y.shape and t.size > 100 and 0.0 <= t[0] < 0.01


def test_teardown_restores_impedance_color_and_object():
    c = _finished_demo()
    sim = c["sim"]
    assert sim.drive.k_scale == 1.0 and sim.drive.d_scale == 1.0 and sim.drive.tau_limit == 0.60
    assert np.allclose(sim.model.geom_rgba[c["geoms"]], c["rgba0"])
    b = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "fragile")
    qa = sim.model.jnt_qposadr[sim.model.body_jntadr[b]]
    assert np.allclose(sim.data.qpos[qa:qa + 3], sim.model.qpos0[qa:qa + 3], atol=1e-3)   # 放回后还会走一个物理步
    assert sim.grasp.object_name == "cube"            # 抓取指标切回场景默认的物体


def test_stop_midway_restores_state():
    sim = Simulation(TABLE)
    demo = FragilePickPlaceDemo()
    sim.start_demo(demo)
    for _ in range(int(9.0 / sim.timestep)):          # 停在第一次试验中途（闭合阶段）
        sim.step_once()
    assert sim.drive.k_scale == 0.5
    sim.stop_demo()
    assert sim.demo is None and not demo.finished and sim.drive.k_scale == 1.0
    sim.reset()
    for _ in range(100):
        sim.step_once()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
