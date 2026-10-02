"""手模型与手势/节奏测试：无需界面。运行 `python tests/test_hand.py` 或 `pytest`。"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.controllers.rhythm import COUNT_SEQUENCE, rhythm_curls  # noqa: E402
from app.core.scene_extras import load_extras  # noqa: E402
from app.core.sim import Simulation  # noqa: E402

HAND = ROOT / "app" / "scenes" / "hand" / "hand.xml"
ARM = ROOT / "app" / "scenes" / "arm" / "arm.xml"


def _run(sim, seconds):
    peak = 0.0
    for _ in range(int(seconds / sim.timestep)):
        sim.step_once()
        peak = max(peak, float(np.abs(sim.data.qvel).max()))
    assert np.all(np.isfinite(sim.data.qpos)), "数值发散"
    return peak


def _tip_dist(sim, a, b):
    return float(np.linalg.norm(sim.data.site(a).xpos - sim.data.site(b).xpos))


# ---------------------------------------------------------------- 模型与分组
def test_hand_actuators_grouped():
    sim = Simulation(HAND)
    from collections import Counter
    assert len(sim.actuators) == 26
    assert Counter(a.group for a in sim.actuators) == {
        "wrist": 6, "thumb": 4, "index": 4, "middle": 4, "ring": 4, "little": 4}
    # 手腕前 3 个是滑动关节(米)，其余是转动关节(弧度)
    assert [a.angular for a in sim.actuators[:6]] == [False, False, False, True, True, True]


def test_arm_has_no_extras_and_stays_flat():
    assert load_extras(ARM) is None
    sim = Simulation(ARM)
    assert sim.extras is None and all(a.group == "" for a in sim.actuators)


def test_rest_pose_holds_wrist_and_is_stable():
    sim = Simulation(HAND)
    peak = _run(sim, 2.0)
    palm = sim.data.body("palm").xpos
    assert abs(palm[2] - 0.30) < 0.002, palm      # 手腕重力补偿 + 高刚度，几乎不下沉
    assert peak < 1.0                              # 初始有手指在重力下的轻微沉降，不应剧烈
    assert np.abs(sim.data.qvel).max() < 1e-3      # 最终静止


def test_wrist_moves_to_target():
    sim = Simulation(HAND)
    sim.set_pose({"wrist_x": 0.1, "wrist_z": -0.1, "wrist_yaw": 0.5}, 0.0)
    _run(sim, 2.0)
    palm = sim.data.body("palm").xpos
    assert np.allclose(palm[:3], [0.1, 0.0, 0.2], atol=0.003), palm
    q = dict(zip([a.joint_name for a in sim.actuators], sim.joint_angles()))
    assert abs(q["wrist_yaw"] - 0.5) < 0.01


# ---------------------------------------------------------------- 手势
def test_gestures_reference_valid_joints_in_range():
    sim = Simulation(HAND)
    spec = {a.joint_name: a for a in sim.actuators}
    assert list(sim.extras.gestures) == ["张开", "握拳", "OK", "1", "2", "3", "4", "5"]
    for name, pose in sim.extras.gestures.items():
        for j, v in pose.items():
            assert j in spec, (name, j)
            assert spec[j].lo - 1e-9 <= v <= spec[j].hi + 1e-9, (name, j, v)
        assert not any(j.startswith("wrist") for j in pose), name  # 手势不动手腕


def test_every_gesture_reaches_its_pose():
    sim = Simulation(HAND)
    spec = {a.joint_name: k for k, a in enumerate(sim.actuators)}
    for name, pose in sim.extras.gestures.items():
        sim.reset()
        sim.set_pose(pose, 0.5)
        peak = _run(sim, 1.8)
        q = sim.joint_angles()
        err = max(abs(q[spec[j]] - v) for j, v in pose.items())
        assert err < 0.08, (name, err)
        assert peak < 30, (name, peak)


def test_transition_is_smooth_and_timed():
    sim = Simulation(HAND)
    sim.set_pose(sim.extras.gestures["握拳"], 1.0)
    k = sim.joint_index("index_mcp")
    seen = []
    for step in range(int(1.4 / sim.timestep)):
        sim.step_once()
        if step % 50 == 0:
            seen.append(sim.manual.targets[k])
    assert seen[0] < 0.05 and abs(seen[-1] - 1.35) < 1e-9     # 起点 / 终点
    assert all(b >= a - 1e-12 for a, b in zip(seen, seen[1:]))  # 单调
    assert not sim.manual.moving


def test_ok_gesture_touches_thumb_and_index():
    sim = Simulation(HAND)
    sim.set_pose(sim.extras.gestures["OK"], 0.0)
    _run(sim, 1.5)
    assert _tip_dist(sim, "thumb_tip", "index_tip") < 0.025
    sim.reset()
    sim.set_pose(sim.extras.gestures["张开"], 0.0)
    _run(sim, 1.5)
    assert _tip_dist(sim, "thumb_tip", "index_tip") > 0.08


def test_fist_brings_fingertips_to_palm_and_five_extends():
    sim = Simulation(HAND)
    sim.set_pose(sim.extras.gestures["握拳"], 0.0)
    _run(sim, 1.5)
    palm = sim.data.body("palm").xpos
    for f in ("index", "middle", "ring", "little"):
        tip = sim.data.site(f"{f}_tip").xpos
        assert abs(tip[0] - palm[0]) < 0.11 and tip[2] < palm[2], f   # 指尖回到掌心下方
    sim.reset()
    sim.set_pose(sim.extras.gestures["5"], 0.0)
    _run(sim, 1.5)
    for f in ("index", "middle", "ring", "little"):
        assert sim.data.site(f"{f}_tip").xpos[0] > palm[0] + 0.15, f


def test_number_gestures_extend_the_right_fingers():
    sim = Simulation(HAND)
    palm_x = 0.0
    expect = {"1": ["index"], "2": ["index", "middle"], "3": ["index", "middle", "ring"],
              "4": ["index", "middle", "ring", "little"]}
    for g, ext in expect.items():
        sim.reset()
        sim.set_pose(sim.extras.gestures[g], 0.0)
        _run(sim, 1.5)
        for f in ("index", "middle", "ring", "little"):
            reach = sim.data.site(f"{f}_tip").xpos[0] - palm_x
            assert (reach > 0.15) == (f in ext), (g, f, reach)


def test_synergies():
    sim = Simulation(HAND)
    ex = sim.extras
    fist = ex.synergy_pose({"curl": 1.0, "spread": 0.0, "oppose": 0.0})
    assert abs(fist["index_pip"] - 1.75) < 1e-9
    sim.set_pose(ex.synergy_pose({"curl": 0.0, "spread": 1.0, "oppose": 0.0}), 0.0)
    _run(sim, 1.0)
    q = dict(zip([a.joint_name for a in sim.actuators], sim.joint_angles()))
    assert q["index_spread"] > 0.1 and q["little_spread"] < -0.15   # 张开 = 食指/小指朝两侧分开
    sim.set_pose(ex.synergy_pose({"curl": 0.0, "spread": 0.0, "oppose": 1.0}), 0.0)
    _run(sim, 1.0)
    q = dict(zip([a.joint_name for a in sim.actuators], sim.joint_angles()))
    assert q["thumb_rot"] > 0.8


# ---------------------------------------------------------------- 节奏
def test_rhythm_curls_ranges_and_shapes():
    for pat in ("wave", "count", "tap"):
        for t in np.linspace(0, 6, 241):
            c = rhythm_curls(pat, t, 0.8, 1.0, 0.12)
            assert c.shape == (5,) and np.all((c >= 0) & (c <= 1)), (pat, t, c)
    # 波浪：相邻手指相位滞后 lag 个周期
    p0 = rhythm_curls("wave", 0.25, 1.0, 1.0, 0.25)
    assert abs(p0[0] - 0.5) < 1e-9 and abs(p0[1] - 0.0) < 1e-9   # 拇指在半程，食指相位差 1/4 周期 → 刚开始
    # 敲击：同一时刻至多少数手指处于敲击中，其余在待机弯曲度
    tap = rhythm_curls("tap", 0.0, 1.0, 1.0, 0.2)
    assert abs(tap[0] - 0.10) < 1e-9 and np.all(tap >= 0.10 - 1e-12)


def test_rhythm_count_sequence():
    freq = 0.5
    step_time = 1.0 / (len(COUNT_SEQUENCE) * freq)
    # 取每一步的末尾（过渡已完成），伸出的手指数应等于序列
    for s, n in enumerate(COUNT_SEQUENCE):
        c = rhythm_curls("count", (s + 0.95) * step_time, freq, 1.0, 0.0)
        extended = int(np.sum(c < 0.02))
        assert extended == n, (s, n, c)
    c = rhythm_curls("count", 0.95 * step_time, freq, 1.0, 0.0)      # "1" = 食指(下标1)伸出
    assert c[1] < 0.02 and np.all(c[[0, 2, 3, 4]] > 0.98)


def test_rhythm_runs_stable_and_moves_fingers():
    for pat in ("wave", "count", "tap"):
        sim = Simulation(HAND)
        sim.start_rhythm(pat, freq=1.0, amp=1.0, lag=0.15)
        k = sim.joint_index("middle_mcp")
        seen, peak = [], 0.0
        for step in range(int(4.0 / sim.timestep)):
            sim.step_once()
            peak = max(peak, float(np.abs(sim.data.qvel).max()))
            if step % 25 == 0:
                seen.append(sim.data.qpos[sim._qadr[k]])
        assert np.all(np.isfinite(sim.data.qpos)), pat
        assert max(seen) - min(seen) > 0.3, (pat, "手指几乎没动")
        assert peak < 40, (pat, peak)
        # 手腕不受节奏影响
        assert abs(sim.data.body("palm").xpos[2] - 0.30) < 0.003


def test_rhythm_stop_hands_over_without_jump():
    sim = Simulation(HAND)
    sim.start_rhythm("wave", freq=1.0)
    _run(sim, 1.37)
    cmd = sim._last_targets.copy()   # 手指的 data.ctrl 是力矩，这里要比较的是"目标角度"
    sim.stop_rhythm()
    assert not sim.rhythm_active
    assert np.allclose(sim.manual.targets, cmd, atol=1e-9)
    _run(sim, 0.5)  # 之后手指保持在停下的位置，不发散
    assert np.all(np.isfinite(sim.data.qpos))


def test_reset_restores_defaults_even_with_rhythm():
    sim = Simulation(HAND)
    sim.set_pose(sim.extras.gestures["握拳"], 0.0)
    sim.start_rhythm("tap")
    _run(sim, 1.0)
    sim.reset()
    assert sim.time == 0.0 and np.allclose(sim.manual.targets, 0.0)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
