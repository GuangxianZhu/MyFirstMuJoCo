"""B5 测试：演示 4「位置控制 vs 阻抗控制抓球」。无需界面。运行 `python tests/test_demo_compare.py` 或 `pytest`。

只做不依赖结论方向的断言（哪组推得远、哪组冲击大由 tools/report_compare.py 打印，人工看）。
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402
from app.scenes.hand.demos import CompareGraspDemo  # noqa: E402
from app.scenes.hand.grasp_sequence import LevelPinch  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"
_CACHE = {}


def _finished_demo():
    if "demo" not in _CACHE:
        sim = Simulation(TABLE)
        demo = CompareGraspDemo()
        sim.start_demo(demo)
        while sim.demo is not None and sim.time < 90.0:
            sim.step_once()
        _CACHE.update(sim=sim, demo=demo)
    return _CACHE


def test_registered():
    sim = Simulation(TABLE)
    assert "compare" in [f.key for f in sim.extras.demos]
    assert "球" in CompareGraspDemo.title


def test_six_trials_finish_within_90s():
    c = _finished_demo()
    sim, demo = c["sim"], c["demo"]
    assert demo.finished and sim.demo is None and sim.time < 90.0
    assert len(demo.results) == 6
    assert np.all(np.isfinite(sim.data.qpos))
    groups = {(r["group"], round(r["error"] * 1000)) for r in demo.results}
    assert len(groups) == 6


def test_metrics_are_finite_numbers():
    for r in _finished_demo()["demo"].results:
        for k in ("impact_peak", "steady_force", "push_away"):
            assert isinstance(r[k], float) and math.isfinite(r[k]) and r[k] >= 0.0, (k, r)
        assert isinstance(r["lifted"], bool) and isinstance(r["final_state"], str)


def test_zero_error_lifts_in_at_least_one_group():
    rs = [r for r in _finished_demo()["demo"].results if r["error"] == 0.0]
    assert len(rs) == 2 and any(r["lifted"] for r in rs)


def test_summary_has_six_rows_and_curves_for_8mm():
    demo = _finished_demo()["demo"]
    rows = [ln for ln in demo.summary().splitlines() if " 偏 " in ln and "冲击峰值" in ln and not ln.startswith("偏 8 mm 时")]
    assert len(rows) == 6
    curves = demo.result_curves()
    assert [c[0] for c in curves] == [g for g, _ in CompareGraspDemo.GROUPS]
    for _, t, f in curves:
        assert t.size > 50 and t.shape == f.shape and abs(t[0]) < 0.01


def test_teardown_restores():
    sim = _finished_demo()["sim"]
    assert sim.drive.k_scale == 1.0 and sim.drive.tau_limit == 0.60
    assert sim.grasp.object_name == "cube"
    b = sim.model.body("ball").id
    qa = sim.model.jnt_qposadr[sim.model.body_jntadr[b]]
    assert np.allclose(sim.data.qpos[qa:qa + 3], sim.model.qpos0[qa:qa + 3], atol=1e-3)


def test_level_pinch_table_keeps_tips_level():
    # 决策门 G1 第 1 轮的水平对齐姿态族：表里每个开口两指尖球心高度差 < 2 mm（实测没有提高成功率，见附录 A）
    sim = Simulation(TABLE)
    fam = LevelPinch(sim.model, {a.joint_name: a.qpos_adr for a in sim.actuators})
    assert fam.max_dz() < 0.002
    assert len(fam.table) == 11


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
