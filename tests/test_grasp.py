"""B2 测试：只统计手↔物体的触觉、抓取指标 GraspMonitor 与判定状态。无需界面。

运行 `python tests/test_grasp.py` 或 `pytest`。这里用一个精简的捏取流程（与 docs/prototypes/pinch_cube_proto.py 相同），
不依赖 B3 的序列器。
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.grasp import BROKEN_RGBA  # noqa: E402
from app.core.impedance import PRESETS  # noqa: E402
from app.core.sim import Simulation  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"


# ---------------------------------------------------------------- 精简捏取流程
def _pinch_pose(s):
    c, opp = 0.30 + 0.25 * s, 0.50 + 0.35 * s
    pose = {f"index_{j}": v * c for j, v in {"mcp": 1.35, "pip": 1.75, "dip": 1.15}.items()}
    pose.update({"thumb_rot": 1.026 * opp, "thumb_cmc": 0.628 * opp, "thumb_mcp": 0.312 * opp, "thumb_ip": 0.392 * opp})
    for f in ("middle", "ring", "little"):
        pose.update({f"{f}_mcp": 0.1, f"{f}_pip": 0.1, f"{f}_dip": 0.05})
    return pose


def _tips(m, d):
    def c(body, L):
        b = d.body(body)
        return b.xpos + b.xmat.reshape(3, 3)[:, 0] * L
    return c("index_dist", 0.024), c("thumb_dist", 0.026)


def _pinch(sim, F_T=1.5, lift=0.10, hold=1.0, on_tick=None):
    """张开 → 到物体上方 → 下降 → 力终止闭合 → 抬起 → 保持。物体是 sim.grasp 当前的对象。"""
    m, d, gm = sim.model, sim.data, sim.grasp
    J, T = sim.joint_index, sim.manual.targets
    scratch = mujoco.MjData(m)
    qadr = {a.joint_name: a.qpos_adr for a in sim.actuators}

    def tips_at(s):
        scratch.qpos[:] = 0
        for k, v in _pinch_pose(s).items():
            scratch.qpos[qadr[k]] = v
        mujoco.mj_kinematics(m, scratch)
        return _tips(m, scratch)

    ss = np.linspace(0, 1, 101)
    gap = 0.03 + 0.0077 + 0.0085
    s_c = ss[int(np.argmin([abs(np.linalg.norm(np.subtract(*tips_at(x))) - gap) for x in ss]))]
    a, b = tips_at(s_c)
    yaw = np.arctan2((b - a)[1], (b - a)[0])
    qa = m.jnt_qposadr[m.body_jntadr[gm.body]]
    d.qpos[qa + 3: qa + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    w = d.xpos[gm.body] - (a + b) / 2

    def setpose(s):
        for k, v in _pinch_pose(s).items():
            T[J(k)] = v

    def run(sec, hook=None):
        for _ in range(int(round(sec / sim.control_dt))):
            if hook:
                hook()
            if on_tick:
                on_tick()
            sim.step_once()
            sim.step_once()

    setpose(0.0)
    T[J("wrist_x")], T[J("wrist_y")], T[J("wrist_z")] = w[0], w[1], w[2] + 0.06
    run(2.0)
    run(1.8, lambda: T.__setitem__(J("wrist_z"), max(T[J("wrist_z")] - 0.04 * sim.control_dt, w[2])))
    run(0.5)
    st = {"s": 0.0, "done": False}

    def close():
        r = gm.reading
        if min(r.group_normal("index"), r.group_normal("thumb")) >= F_T:
            st["done"] = True
        if not st["done"] and st["s"] < 1.0:
            st["s"] += 0.15 * sim.control_dt
        setpose(st["s"])

    run(6.0, close)
    t_lift = sim.time
    run(lift / 0.08 + 0.3, lambda: T.__setitem__(J("wrist_z"), min(T[J("wrist_z")] + 0.08 * sim.control_dt, w[2] + lift)))
    run(hold)
    return t_lift


def _geoms(sim, body):
    return [g for g in range(sim.model.ngeom) if sim.model.geom_bodyid[g] == body]


# ---------------------------------------------------------------- 1. read_against
def _press(sim, target, seconds=3.0):
    zi = sim.joint_index("wrist_z")
    z = 0.0
    for i in range(int(seconds / sim.timestep)):
        r = sim.tactile_reading
        F = r.total_normal() if r else 0.0
        if i % 2 == 0:
            z -= 0.10 * sim.control_dt * float(np.clip((target - F) / target, -0.5, 1.0))
            sim.manual.set_target(zi, max(z, -0.295))
        sim.step_once()


def test_read_against_filters_by_body():
    # 手移到桌面空处按压：触觉总读数有力，但"对方块"的读数全 0
    sim = Simulation(TABLE)
    sim.set_pose({"wrist_y": -0.20, "wrist_pitch": 0.35}, 0.0)
    _press(sim, 4.0)
    cube = sim.grasp.body
    full = sim.tactile.read(sim.data)
    only = sim.tactile.read_against(sim.data, {cube})
    assert full.total_normal() > 2.0
    assert only.total_normal() == 0.0 and only.count.sum() == 0 and not np.any(only.force)
    assert sim.grasp.metrics()["grip_total"] == 0.0
    # 桌面属于世界刚体 0：read_against({0}) 等于全部读数
    world = sim.tactile.read_against(sim.data, {0})
    assert np.allclose(world.normal, full.normal)

    # 把方块放到食指指尖正下方，按下去：对方块的读数 = 全读数里对应段的读数
    sim = Simulation(TABLE)
    sim.set_pose({"wrist_pitch": 0.35}, 0.0)
    for _ in range(200):
        sim.step_once()
    tip = sim.data.site("index_tip").xpos
    qa = sim.model.jnt_qposadr[sim.model.body_jntadr[cube]]
    sim.data.qpos[qa:qa + 2] = tip[:2] + [-0.008, 0.0]
    mujoco.mj_forward(sim.model, sim.data)
    _press(sim, 2.0)
    full = sim.tactile.read(sim.data)
    only = sim.tactile.read_against(sim.data, {cube})
    i = sim.tactile.names.index("index_dist")
    assert only.normal[i] > 0.3, only.normal
    assert abs(only.normal[i] - full.normal[i]) < 1e-9
    assert np.allclose(only.force[i], full.force[i])
    assert only.total_normal() <= full.total_normal() + 1e-9


# ---------------------------------------------------------------- 2. 抓起方块 → 抓牢
def test_pinch_cube_is_held():
    sim = Simulation(TABLE)
    t_lift = _pinch(sim)
    gm = sim.grasp
    m = gm.metrics()
    assert gm.state == "抓牢", (gm.state, m)
    assert m["lift"] >= 0.05
    # 打滑：抬起开始之后（真正"拿在手里"的那段）几乎没有相对手的移动
    h = gm.history()
    after = h["slip"][h["t"] >= t_lift]
    assert after.max() - after.min() <= 0.004, after.max() - after.min()
    # 注：累计 slip（从 t_ref 起）约 7 mm，其中约 6 mm 发生在桌面上闭合挤压时（物体被两指推着对中），见附录 A
    assert m["slip"] < 0.010
    assert abs(m["grip_index"] - m["grip_thumb"]) < 0.15 * max(m["grip_index"], m["grip_thumb"])
    assert set(m["fingers_touching"]) == {"thumb", "index"} and m["n_segments"] == 2
    assert m["t_ref"] is not None and m["displacement"] > m["lift"] - 0.005


def test_low_friction_cube_slips_out():
    # 负例：方块摩擦系数降到 0.05（握力 2×1.5 N × 0.05 = 0.15 N < 重力 0.29 N），夹不住
    sim = Simulation(TABLE)
    g = _geoms(sim, sim.grasp.body)
    sim.model.geom_friction[g, 0] = 0.05
    sim.model.geom_priority[g] = 1          # 接触参数默认取两者的最大值；提高优先级让方块的摩擦系数生效
    states = []
    _pinch(sim, on_tick=lambda: states.append(sim.grasp.state))
    assert "抓牢" not in states
    assert sim.grasp.state in ("滑落", "接触", "未接触"), sim.grasp.state
    assert sim.grasp.metrics()["lift"] < 0.02


# ---------------------------------------------------------------- 3. 易碎
def test_fragile_breaks_with_stiff_grip_and_reset_restores_color():
    sim = Simulation(TABLE)
    gm = sim.set_grasp_object("fragile", fragile_limit=1.8)
    geoms = _geoms(sim, gm.body)
    rgba0 = sim.model.geom_rgba[geoms].copy()
    _, ks, ds, lim = PRESETS[2]
    sim.drive.apply_preset(ks, ds, lim)
    _pinch(sim, F_T=3.0, hold=0.3)
    assert gm.broken and gm.state == "碎了"
    assert np.allclose(sim.model.geom_rgba[geoms], BROKEN_RGBA)
    assert gm.metrics()["finger_max"] > 1.8 or gm.history()["finger_max"].max() > 1.8
    sim.reset()
    assert not gm.broken and gm.state == "未接触"
    assert np.allclose(sim.model.geom_rgba[geoms], rgba0)


def test_fragile_soft_grip_does_not_break():
    sim = Simulation(TABLE)
    gm = sim.set_grasp_object("fragile", fragile_limit=1.8)
    _pinch(sim, F_T=1.2)
    assert not gm.broken and gm.state == "抓牢", gm.state
    assert gm.history()["finger_max"].max() < 1.8


def test_fragile_needs_20ms_over_limit():
    # 注入假的触觉读数：食指 2.0 N（> 1.8 N）。持续 16 ms（4 个控制周期）不算碎，持续 ≥ 20 ms 才算
    from app.core.tactile import TactileReading
    sim = Simulation(TABLE)
    gm = sim.set_grasp_object("fragile", fragile_limit=1.8)
    names = sim.tactile.names
    force = {"v": 0.0}

    def fake(data, body_ids, with_points=False):
        n = np.zeros(len(names))
        n[names.index("index_dist")] = force["v"]
        n[names.index("thumb_dist")] = force["v"]
        return TactileReading(names, n, np.zeros((len(names), 3)), (n > 0).astype(int))

    sim.tactile.read_against = fake
    ticks = lambda k: [sim.step_once() for _ in range(2 * k)]  # noqa: E731  每个控制周期 2 个物理步
    force["v"] = 2.0
    ticks(4)
    force["v"] = 1.0
    ticks(3)
    assert not gm.broken
    force["v"] = 2.0
    ticks(7)
    assert gm.broken and gm.state == "碎了"
    force["v"] = 0.0
    ticks(5)
    assert gm.broken and gm.state == "碎了"        # 锁存
    sim.reset()
    assert not gm.broken


# ---------------------------------------------------------------- 4. t_ref 与 reset
def test_slip_zero_before_grasp_established_and_reset_clears():
    sim = Simulation(TABLE)
    _pinch(sim)
    gm = sim.grasp
    h = gm.history()
    assert gm.t_ref is not None
    before = h["t"] < gm.t_ref
    assert before.any() and np.all(h["slip"][before] == 0.0) and np.all(h["displacement"][before] == 0.0)
    # t_ref 时：总握力 ≥ 0.5 N 且两指都接触，并已持续 50 ms
    k = np.searchsorted(h["t"], gm.t_ref)
    assert np.all(h["grip_total"][k:k + 12] >= 0.5)
    sim.reset()
    m = gm.metrics()
    assert gm.t_ref is None and gm.state == "未接触"
    assert m["slip"] == 0.0 and m["displacement"] == 0.0 and m["grip_total"] == 0.0
    assert gm.history()["t"].size == 0
    assert abs(gm.z_rest - 0.1155) < 1e-6


def test_reset_object_puts_object_back():
    sim = Simulation(TABLE)
    _pinch(sim, hold=0.2)
    gm = sim.grasp
    assert gm.metrics()["lift"] > 0.05
    sim.manual.reset()
    sim.reset_object("cube")
    qa = sim.model.jnt_qposadr[sim.model.body_jntadr[gm.body]]
    assert np.allclose(sim.data.qpos[qa:qa + 7], sim.model.qpos0[qa:qa + 7])
    assert gm.t_ref is None and gm.history()["t"].size == 0


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
