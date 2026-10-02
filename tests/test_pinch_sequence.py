"""B3 测试：捏取序列器 PinchGraspSequence。无需界面。运行 `python tests/test_pinch_sequence.py` 或 `pytest`。"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.impedance import PRESETS  # noqa: E402
from app.core.sim import Simulation  # noqa: E402
from app.scenes.hand.grasp_sequence import LinearPinch, PinchGraspSequence, PinchParams, run_sequence  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"


def _sim(xy=None, mass=None):
    sim = Simulation(TABLE)
    b = sim.grasp.body
    if xy is not None:
        qa = sim.model.jnt_qposadr[sim.model.body_jntadr[b]]
        sim.model.qpos0[qa:qa + 2] = xy           # 初始位置写进 qpos0，reset 后就在这里
        sim.reset()
    if mass is not None:
        g = next(g for g in sim.physics.groups if g.key == "mass")
        g.set_value(g.labels.index("cube"), mass)
    return sim


def _run(sim, **kw):
    seq = PinchGraspSequence()
    seq.start(sim, "cube", PinchParams(**kw), t=sim.time)
    run_sequence(sim, seq, timeout=40.0)
    assert seq.finished, (seq.state, seq.failed)
    assert np.all(np.isfinite(sim.data.qpos))
    return seq


def test_baseline_runs_to_done_and_is_held_after_lift():
    sim = _sim()
    seq = _run(sim)
    assert seq.done and seq.failed is None and seq.state == "DONE"
    assert seq.lift_end_state == "抓牢"
    states = [s for _, s in seq.events]
    assert states == ["APPROACH", "DESCEND", "CLOSE", "SETTLE", "LIFT", "CARRY", "LOWER", "RELEASE", "RETREAT", "DONE"]
    # 规划：两指尖球心间距 = 边长 + 两个指尖半径
    assert abs(seq.plan["tip_gap_at_sc"] - seq.plan["gap"]) < 0.002
    assert 0.5 < seq.plan["s_c"] < 0.8
    # 放回原处（没给放置点）：位置误差小，最后手已松开
    xy = np.array(sim.grasp.metrics()["obj_pos"][:2])
    assert np.linalg.norm(xy - [0.15, 0.0]) < 0.008
    assert sim.grasp.metrics()["grip_total"] == 0.0 and sim.grasp.state == "未接触"
    assert seq.peak["finger"] >= 1.5


def test_sequence_does_not_touch_impedance():
    sim = _sim()
    _, ks, ds, lim = PRESETS[0]
    sim.drive.apply_preset(ks, ds, lim)
    _run(sim)
    assert (sim.drive.k_scale, sim.drive.d_scale, sim.drive.tau_limit) == (ks, ds, lim)


def test_four_positions_all_succeed():
    for xy in [(0.12, 0.02), (0.18, -0.02), (0.20, 0.0), (0.10, -0.04)]:
        seq = _run(_sim(xy))
        assert seq.done and seq.lift_end_state == "抓牢", (xy, seq.failed, seq.lift_end_state)


def test_carry_to_place_point():
    sim = _sim()
    seq = _run(sim, place_xy=(0.15, -0.10))
    assert seq.done
    xy = np.array(sim.grasp.metrics()["obj_pos"][:2])
    assert np.linalg.norm(xy - [0.15, -0.10]) < 0.008, xy
    assert sim.grasp.state != "滑落"                     # 主动放下不算滑落


def test_weak_grip_fails_readably():
    # 0.6 N 在默认摩擦(μ=1)下其实拿得住（2×0.6×1 = 1.2 N > 重力 0.29 N，见附录 A）；
    # 把方块摩擦降到 0.2 后 2×0.6×0.2 = 0.24 N < 0.29 N，应当失败，并给出可读的原因
    sim = _sim()
    g = [i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] == sim.grasp.body]
    sim.model.geom_friction[g, 0] = 0.2
    sim.model.geom_priority[g] = 1
    seq = _run(sim, F_target=0.6)
    assert not seq.done and seq.failed and "抬起" in seq.failed, seq.failed
    assert ("滑落" in seq.failed) or ("没抬起" in seq.failed)
    # 失败收尾：手指张开、手腕抬起，序列停在 IDLE
    assert seq.state == "IDLE" and seq._s == 0.0
    T = sim.manual.targets
    for k, v in LinearPinch().pose(0.0).items():
        assert abs(T[sim.joint_index(k)] - v) < 1e-9


def test_weak_grip_with_normal_friction_still_lifts():
    seq = _run(_sim(), F_target=0.6)
    assert seq.done and seq.lift_end_state == "抓牢"


def test_heavy_object_fails_without_crash():
    seq = _run(_sim(mass=0.12))
    assert not seq.done and seq.failed
    assert seq.state == "IDLE"


def test_force_out_of_reach_fails_in_close():
    # 默认阻抗下 s=1 时每指最多约 2 N（陷阱 T7），要 3 N 夹不到
    seq = _run(_sim(), F_target=3.0)
    assert seq.failed and seq.failed.startswith("闭合阶段失败"), seq.failed
    assert "N" in seq.failed


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
