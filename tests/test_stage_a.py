"""阶段 A 测试：关节阻抗、触觉、示波器、两个演示。无需界面。运行 `python tests/test_stage_a.py` 或 `pytest`。"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.impedance import PRESETS  # noqa: E402
from app.core.scope import Scope  # noqa: E402
from app.core.sim import Simulation  # noqa: E402

HAND = ROOT / "app" / "scenes" / "hand" / "hand.xml"
ARM = ROOT / "app" / "scenes" / "arm" / "arm.xml"


def _run(sim, seconds):
    for _ in range(int(seconds / sim.timestep)):
        sim.step_once()
    assert np.all(np.isfinite(sim.data.qpos))


# ---------------------------------------------------------------- 阻抗
def test_finger_actuators_are_torque_and_wrist_is_position():
    sim = Simulation(HAND)
    torque = [a.torque for a in sim.actuators]
    assert torque == [False] * 6 + [True] * 20
    assert sim.drive.active and sim.drive.mask.sum() == 20


def test_arm_has_no_impedance_and_passes_targets_through():
    sim = Simulation(ARM)
    assert not sim.drive.active
    sim.set_target(0, 0.5)
    _run(sim, 0.1)
    assert abs(sim.data.ctrl[sim.actuators[0].index] - 0.5) < 1e-9     # 位置执行器：ctrl 就是目标角


def test_impedance_law_and_saturation():
    sim = Simulation(HAND)
    d = sim.drive
    n = len(sim.actuators)
    k = sim.joint_index("index_mcp")
    q = np.zeros(n)
    qd = np.zeros(n)
    tgt = np.zeros(n)
    tgt[k] = 0.1
    tau = d.compute(tgt, q, qd)[k]
    assert abs(tau - 1.5 * 0.1) < 1e-12                       # K*(目标-角度)
    qd[k] = 1.0
    assert abs(d.compute(tgt, q, qd)[k] - (0.15 - 0.08)) < 1e-12   # 再减 D*角速度
    tgt[k] = 2.0
    qd[k] = 0.0
    assert d.compute(tgt, q, qd)[k] == 0.60                   # 限幅
    d.apply_preset(5.0, 1.0, 1.0)
    assert d.compute(tgt, q, qd)[k] == 1.0
    # 手腕(位置执行器)原样透传
    tgt[0] = 0.07
    assert d.compute(tgt, q, qd)[0] == 0.07
    d.reset()
    assert d.k_scale == 1.0 and d.tau_limit == 0.60


def test_stiffer_impedance_responds_faster_without_blowing_up():
    t90 = {}
    for name, ks, ds, lim in PRESETS:
        sim = Simulation(HAND)
        sim.drive.apply_preset(ks, ds, lim)
        k = sim.joint_index("index_mcp")
        sim.set_pose({"index_mcp": 0.8}, 0.0)
        hit = None
        peak = 0.0
        for i in range(int(2.0 / sim.timestep)):
            sim.step_once()
            q = sim.data.qpos[sim._qadr[k]]
            peak = max(peak, q)
            if hit is None and q >= 0.72:
                hit = sim.time
        assert hit is not None, name
        assert peak < 0.85, (name, peak)            # 几乎没有超调
        t90[name] = hit
    names = [p[0] for p in PRESETS]
    assert t90[names[0]] > t90[names[1]] > t90[names[2]]


# ---------------------------------------------------------------- 触觉
def _press_floor(sim, target_n=6.0, seconds=4.0):
    sim.set_pose({"wrist_pitch": 0.35}, 0.0)
    zi = sim.joint_index("wrist_z")
    z = 0.0
    for i in range(int(seconds / sim.timestep)):
        r = sim.tactile_reading
        F = r.total_normal() if r else 0.0
        if i % 5 == 0:
            z -= 0.10 * sim.control_dt * 5 * float(np.clip((target_n - F) / target_n, -0.5, 1.0))
            sim.manual.set_target(zi, max(z, -0.295))
        sim.step_once()
    return sim.tactile_reading


def test_tactile_zero_in_free_air():
    sim = Simulation(HAND)
    _run(sim, 0.5)
    r = sim.tactile_reading
    assert r.total_normal() == 0.0 and r.count.sum() == 0


def test_tactile_pressing_floor_force_sign_and_balance():
    sim = Simulation(HAND)
    r = _press_floor(sim)
    assert abs(r.total_normal() - 6.0) < 0.8
    assert r.force[:, 2].sum() > 5.0                       # 地面把手往上推：合力 z 为正
    assert abs(r.force[:, 0].sum()) < 0.5 and abs(r.force[:, 1].sum()) < 0.5
    touching = {n for n, c in zip(r.names, r.count) if c}
    assert touching and all(n.endswith("_dist") for n in touching), touching   # 只有指尖
    assert r.group_normal("index") > 0.5 and r.group_normal("thumb") == 0.0
    # 摩擦很小时，法向力 ≈ 合力 z
    assert abs(r.normal.sum() - r.force[:, 2].sum()) < 0.5


def test_tactile_points_for_arrows():
    sim = Simulation(HAND)
    _press_floor(sim)
    r = sim.tactile.read(sim.data, with_points=True)
    assert len(r.points) == int(r.count.sum())
    seg, pos, f = r.points[0]
    assert pos.shape == (3,) and f.shape == (3,) and pos[2] < 0.02 and f[2] > 0


# ---------------------------------------------------------------- 物理参数面板
def test_per_element_slider_range_follows_each_element():
    sim = Simulation(HAND)
    kp = next(p for p in sim.physics.groups if p.key == "kp")
    # 手腕平移 kp=3000、转动 kp=60：各自的上限是自己出厂值的 6 倍
    assert abs(kp.max_for(0) - 6 * kp.base[0]) < 1e-9
    assert abs(kp.max_for(3) - 6 * kp.base[3]) < 1e-9 and kp.max_for(3) < kp.max_for(0)


# ---------------------------------------------------------------- 示波器
def test_scope_ring_buffer_order_and_window():
    sc = Scope(n_joints=2, n_seg=3, capacity=10)
    for i in range(25):
        sc.push(i * 0.1, np.array([i, 0.0]), np.array([0.0, i]), np.zeros(2), np.array([i, 0, 0.0]))
    assert len(sc) == 10
    v = sc.view()
    assert np.allclose(v["t"], np.arange(15, 25) * 0.1)          # 按时间顺序，保留最近 10 个
    assert np.allclose(v["target"][:, 0], np.arange(15, 25))
    assert np.allclose(v["seg"][:, 0], np.arange(15, 25))
    w = sc.view(0.35)
    assert np.allclose(w["t"], [2.1, 2.2, 2.3, 2.4])
    sc.clear()
    assert len(sc) == 0 and sc.view()["t"].size == 0


def test_sim_records_scope_each_control_tick():
    sim = Simulation(HAND)
    _run(sim, 0.4)
    v = sim.scope.view()
    assert abs(len(v["t"]) - 0.4 / sim.control_dt) <= 1
    assert v["target"].shape[1] == 26 and v["seg"].shape[1] == 16
    sim.reset()
    assert len(sim.scope) == 0


# ---------------------------------------------------------------- 演示
def _run_demo(sim, demo, limit=60.0):
    sim.start_demo(demo)
    while sim.demo is not None and sim.time < limit:
        sim.step_once()
    assert sim.demo is None and demo.finished, "演示没有在限定时间内结束"
    assert np.all(np.isfinite(sim.data.qpos))


def test_demos_are_registered():
    sim = Simulation(HAND)
    assert [f.key for f in sim.extras.demos] == ["press", "impedance"]
    assert sim.extras.demos[0]() is not sim.extras.demos[0]()      # 每次新建实例


def test_press_demo_tip_then_flat():
    sim = Simulation(HAND)
    demo = sim.extras.demos[0]()
    _run_demo(sim, demo)
    seg = dict(demo._phase_seg)
    names = sim.tactile.names
    tip = {n for n, v in zip(names, seg["tip"]) if v > 0.05}
    flat = {n for n, v in zip(names, seg["flat"]) if v > 0.05}
    assert tip and all(n.endswith("_dist") for n in tip)
    assert flat == {"palm"}                                       # 放平后由掌心承力
    assert 5.0 < seg["tip"].sum() < 7.5 and 8.0 < seg["flat"].sum() < 12.0
    assert 8.0 < demo._peak < 14.0
    assert "指尖按压" in demo.summary() and "palm" in demo.summary()
    (label, t, f), = demo.result_curves()
    assert t.shape == f.shape and t.size > 100
    assert sim.last_demo is demo


def test_impedance_demo_ordering_and_cleanup():
    sim = Simulation(HAND)
    demo = sim.extras.demos[1]()
    _run_demo(sim, demo)
    stats = [demo.trial_stats(i) for i in range(3)]
    soft, mid, hard = (s["steady"] for s in stats)
    assert 0.5 < soft < mid < hard                                 # 越刚硬，顶得越用力
    assert hard > 2.0 * soft
    assert all(s["onset"] is not None for s in stats)
    # 结束后：阻抗参数还原、方块收回地下
    assert sim.drive.k_scale == 1.0 and sim.drive.tau_limit == 0.60
    import mujoco
    bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "prop_block")
    assert sim.data.mocap_pos[sim.model.body_mocapid[bid]][2] < -0.5
    curves = demo.result_curves()
    assert [c[0] for c in curves] == [p[0] for p in PRESETS]
    assert "刚硬" in demo.summary()


def test_stop_demo_midway_restores_state():
    sim = Simulation(HAND)
    demo = sim.extras.demos[1]()
    sim.start_demo(demo)
    for _ in range(int(3.5 / sim.timestep)):                    # 停在第二次试验里
        sim.step_once()
    assert sim.drive.k_scale == 1.0 and sim.demo is demo
    sim.stop_demo()
    assert sim.demo is None and sim.drive.k_scale == 1.0 and not demo.finished
    sim.reset()                                                  # 重置不会因为遗留状态出错
    _run(sim, 0.2)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
