"""B1 测试：桌面 + 物体场景 hand_table.xml。无需界面。运行 `python tests/test_table_scene.py` 或 `pytest`。"""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"


def _run(sim, seconds):
    for _ in range(int(seconds / sim.timestep)):
        sim.step_once()
    assert np.all(np.isfinite(sim.data.qpos))


def _body(sim, name):
    return mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, name)


def test_table_scene_loads_with_hand_unchanged():
    sim = Simulation(TABLE)
    assert len(sim.actuators) == 26
    assert [a.torque for a in sim.actuators] == [False] * 6 + [True] * 20
    assert sim.extras is not None and sim.extras.grasp["object"] == "cube"
    assert sim.tactile is not None and sim.tactile.n == 16
    for name in ("table", "cube_geom", "fragile_geom"):
        gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert gid >= 0 and sim.model.geom_contype[gid] == 1 and sim.model.geom_conaffinity[gid] == 1
    for name in ("cube_geom", "fragile_geom"):
        gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert sim.model.geom_condim[gid] == 4 and abs(sim.model.geom_friction[gid][1] - 0.02) < 1e-12


def test_cube_rests_on_table():
    sim = Simulation(TABLE)
    b = _body(sim, "cube")
    p0 = sim.data.xpos[b].copy()
    _run(sim, 2.0)
    p = sim.data.xpos[b]
    assert np.linalg.norm(p[:2] - p0[:2]) < 0.0005, p - p0
    # XML 里初始 z = 0.1155（留 0.5 mm 间隙防穿插），落下后静止在 桌面 0.10 + 半边长 0.015 处（软接触有 <0.1 mm 的压入）
    assert abs(p[2] - 0.115) < 0.0005, p[2]
    assert abs(p[2] - 0.1155) < 0.001, p[2]


def test_reset_restores_object_pose():
    sim = Simulation(TABLE)
    b = _body(sim, "cube")
    adr = sim.model.jnt_qposadr[sim.model.body_jntadr[b]]
    q0 = sim.data.qpos[adr:adr + 7].copy()
    sim.data.qpos[adr:adr + 3] += [0.05, -0.03, 0.1]
    sim.data.qvel[:] = 0.3
    _run(sim, 0.3)
    sim.reset()
    assert np.allclose(sim.data.qpos[adr:adr + 7], q0)
    assert np.allclose(sim.model.qpos0[adr:adr + 7], q0)


def test_objects_listed_in_mass_panel():
    sim = Simulation(TABLE)
    mass = next(g for g in sim.physics.groups if g.key == "mass")
    assert "cube" in mass.labels and "fragile" in mass.labels
    i = mass.labels.index("cube")
    assert abs(mass.base[i] - 0.03) < 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
