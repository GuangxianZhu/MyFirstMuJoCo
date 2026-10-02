"""核心层测试：无需界面。运行 `python tests/test_core.py` 或 `pytest`。"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402

XML = ROOT / "app" / "scenes" / "arm" / "arm.xml"


def test_actuators_autodetected():
    sim = Simulation(XML)
    assert [a.joint_name for a in sim.actuators] == ["base_yaw", "shoulder", "elbow"]
    assert all(a.lo < a.hi for a in sim.actuators)


def test_holds_target_under_gravity():
    sim = Simulation(XML)
    target = np.array([0.5, 0.4, -0.6])
    for i, v in enumerate(target):
        sim.controller.set_target(i, v)
    for _ in range(int(3.0 / sim.timestep)):
        sim.step_once()
    q = sim.joint_angles()
    assert np.all(np.isfinite(q))
    err = np.abs(q - target)
    assert err[0] < 0.01, q     # 底座旋转不受重力影响，应精确到位
    assert err.max() < 0.15, q  # 肩/肘在重力下有少量下垂（位置执行器的稳态误差）


def _settle_error(sim, seconds=3.0):
    target = np.array([0.5, 0.4, -0.6])
    sim.reset()
    for i, v in enumerate(target):
        sim.controller.set_target(i, v)
    for _ in range(int(seconds / sim.timestep)):
        sim.step_once()
    assert np.all(np.isfinite(sim.joint_angles()))
    return np.abs(sim.joint_angles() - target)


def _group(sim, key):
    return next(g for g in sim.physics.groups if g.key == key)


def test_physics_groups_autodetected():
    sim = Simulation(XML)
    keys = [g.key for g in sim.physics.groups]
    assert keys == ["damping", "mass", "kp", "kv"], keys
    assert _group(sim, "mass").labels == ["link1", "link2", "link3", "tip"]  # 固定底座不在列表里
    assert abs(sim.physics.gravity - 9.81) < 1e-9


def test_zero_gravity_removes_sag():
    sim = Simulation(XML)
    base = _settle_error(sim)[1]
    sim.physics.set_gravity(0.0)
    assert _settle_error(sim).max() < 0.01
    assert base > 0.03


def test_heavier_links_sag_more():
    sim = Simulation(XML)
    base = _settle_error(sim)[1]
    _group(sim, "mass").set_scale(3.0)
    assert _settle_error(sim)[1] > base * 2


def test_mass_keeps_inertia_consistent():
    sim = Simulation(XML)
    m = sim.model
    b = int(mujoco_body(sim, "link2"))
    ratio0 = m.body_inertia[b] / m.body_mass[b]
    _group(sim, "mass").set_value(1, 1.5)
    assert abs(m.body_mass[b] - 1.5) < 1e-12
    assert np.allclose(m.body_inertia[b] / m.body_mass[b], ratio0)


def mujoco_body(sim, name):
    import mujoco
    return mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, name)


def test_stiffer_kp_sags_less():
    sim = Simulation(XML)
    base = _settle_error(sim)[1]
    _group(sim, "kp").set_scale(2.0)
    kp = sim.model.actuator_gainprm[:, 0]
    assert np.allclose(kp, 120.0) and np.allclose(sim.model.actuator_biasprm[:, 1], -120.0)
    assert _settle_error(sim)[1] < base * 0.65


def test_kp_zero_arm_falls_but_stays_finite():
    sim = Simulation(XML)
    _group(sim, "kp").set_scale(0.0)
    _settle_error(sim, 2.0)  # 只检查数值不发散


def test_param_change_does_not_disturb_state_and_reset_all_restores():
    sim = Simulation(XML)
    for i, v in enumerate([0.5, 0.4, -0.6]):
        sim.controller.set_target(i, v)
    sim.advance(0.5)
    q = sim.data.qpos.copy()
    _group(sim, "mass").set_scale(2.0)  # 内部调用 mj_setConst，不应改动运行中的状态
    assert np.array_equal(q, sim.data.qpos)

    sim.physics.set_gravity(1.62)
    _group(sim, "damping").set_scale(4.0)
    sim.physics.reset_all()
    m = sim.model
    assert abs(sim.physics.gravity - 9.81) < 1e-9
    assert np.allclose(m.actuator_gainprm[:, 0], 60.0) and np.allclose(-m.actuator_biasprm[:, 2], 6.0)
    assert np.allclose(m.dof_damping, 0.4)


def test_advance_is_realtime():
    sim = Simulation(XML)
    for _ in range(60):
        sim.advance(1 / 60)
    assert abs(sim.time - 1.0) < 0.02


def test_pause_and_reset():
    sim = Simulation(XML)
    sim.advance(0.1)
    sim.paused = True
    t = sim.time
    sim.advance(0.1)
    assert sim.time == t
    sim.paused = False
    sim.reset()
    assert sim.time == 0.0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
