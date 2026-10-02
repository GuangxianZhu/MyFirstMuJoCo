"""灵巧手演示脚本。每个演示是一个 Demo 子类，由 hand_extras.create() 注册。

只通过 sim 的公开接口驱动（set_pose / manual.set_target / drive.apply_preset / data.mocap_pos），
不依赖 Qt，可以在测试里直接批量运行。
"""
from __future__ import annotations

import mujoco
import numpy as np

from app.core.demo import Demo
from app.core.impedance import PRESETS

FINGERS_OPEN = {
    f"{f}_{j}": 0.0 for f in ("index", "middle", "ring", "little") for j in ("spread", "mcp", "pip", "dip")
}


def _smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


class _PhaseClock:
    """演示里的简易状态机：记录当前阶段名和进入该阶段的时刻。"""

    def __init__(self, first: str = ""):
        self.name = first
        self.t0 = 0.0

    def go(self, name: str, t: float) -> None:
        self.name, self.t0 = name, t

    def elapsed(self, t: float) -> float:
        return t - self.t0


# ====================================================================== 演示 1：按压桌面
class PressDemo(Demo):
    key = "press"
    title = "触觉：按压桌面"
    description = (
        "手腕带着手慢慢下压到桌面：先用四根指尖按压，再把手放平让整根手指和掌心一起受力。"
        "下压速度由总法向力闭环控制（力到了目标值就不再往下压）。看下方“触觉”页签里每一节的受力。"
    )
    result_title = "按压过程中的总法向力"
    result_ylabel = "力 (N)"

    camera = {"azimuth": 125.0, "elevation": -22.0, "distance": 0.60, "lookat": (0.12, 0.0, 0.11)}

    TARGET_TIP_N = 6.0      # 阶段一：指尖按压的目标总力
    TARGET_FLAT_N = 10.0    # 阶段二：放平后的目标总力
    SPEED = 0.10            # 最大下压速度 m/s
    HOLD_S = 1.5

    def __init__(self):
        super().__init__()
        self.clock = _PhaseClock("prepare")
        self._z = 0.0
        self._t: list[float] = []
        self._f: list[float] = []
        self._phase_seg: dict[str, np.ndarray] = {}
        self._phase_cnt: dict[str, int] = {}
        self._peak = 0.0
        self._seg_names: list[str] = []
        self._label = "准备姿态"

    def setup(self, sim) -> None:
        self._zi = sim.joint_index("wrist_z")
        self._pi = sim.joint_index("wrist_pitch")
        self._z = 0.0
        self._pitch = 0.0
        self._pitch_goal = 0.35
        sim.set_pose(FINGERS_OPEN, 0.0)
        self._seg_names = list(sim.tactile.names) if sim.tactile else []

    PITCH_RATE = 0.4        # 手腕俯仰转动速度 rad/s

    def _pitch_step(self, sim) -> None:
        """手腕俯仰自己按速度逼近目标。不能用 set_pose 做过渡：下压时每个周期都在 set_target 改高度，会取消过渡。"""
        step = self.PITCH_RATE * sim.control_dt
        self._pitch += float(np.clip(self._pitch_goal - self._pitch, -step, step))
        sim.manual.set_target(self._pi, self._pitch)

    def _force_step(self, sim, F: float, target: float) -> None:
        err = (target - F) / target                 # 1 = 完全没受力, 0 = 到了目标, <0 = 压过头
        v = self.SPEED * float(np.clip(err, -0.5, 1.0))
        self._z = float(np.clip(self._z - v * sim.control_dt, -0.295, 0.0))
        sim.manual.set_target(self._zi, self._z)

    def update(self, sim, t: float) -> None:
        r = sim.tactile_reading
        F = r.total_normal() if r is not None else 0.0
        self._t.append(t)
        self._f.append(F)
        self._peak = max(self._peak, F)
        c = self.clock
        self._pitch_step(sim)

        if c.name == "prepare":
            if c.elapsed(t) > 1.2:
                c.go("tip", t)
                self._label = "指尖按压"
        elif c.name == "tip":
            self._force_step(sim, F, self.TARGET_TIP_N)
            self._accumulate(c.name, r, F, self.TARGET_TIP_N)
            if c.elapsed(t) > 6.0 or self._settled(c, t, F, self.TARGET_TIP_N):
                c.go("flat", t)
                self._label = "放平手掌"
                self._pitch_goal = 0.0
        elif c.name == "flat":
            self._force_step(sim, F, self.TARGET_FLAT_N)
            if c.elapsed(t) > 1.2:          # 等手腕转平之后再统计
                self._accumulate(c.name, r, F, self.TARGET_FLAT_N)
            if c.elapsed(t) > 8.0 or (c.elapsed(t) > 1.6 and self._settled(c, t, F, self.TARGET_FLAT_N)):
                c.go("lift", t)
                self._label = "抬起"
                self._pitch_goal = 0.0
                self._z = self._z   # 抬起阶段下面按速度回到 0
        elif c.name == "lift":
            self._z = min(self._z + 0.15 * sim.control_dt, 0.0)
            sim.manual.set_target(self._zi, self._z)
            if c.elapsed(t) > 2.0:
                self.finished = True

    def _settled(self, c: _PhaseClock, t: float, F: float, target: float) -> bool:
        """力进入目标 ±15% 后稳定保持 HOLD_S 秒。"""
        if abs(F - target) > 0.15 * target:
            self._hold_since = None
            return False
        if getattr(self, "_hold_since", None) is None:
            self._hold_since = t
        return t - self._hold_since >= self.HOLD_S

    def _accumulate(self, phase: str, r, F: float, target: float) -> None:
        """只统计力接近目标之后（压稳了）的各段平均受力。"""
        if r is None or abs(F - target) > 0.25 * target:
            return
        acc = self._phase_seg.setdefault(phase, np.zeros(len(r.normal)))
        n = self._phase_cnt.get(phase, 0)
        self._phase_seg[phase] = (acc * n + r.normal) / (n + 1)
        self._phase_cnt[phase] = n + 1

    def teardown(self, sim) -> None:
        pass

    def progress(self) -> str:
        return f"{self._label}    当前总力 {self._f[-1]:.1f} N    峰值 {self._peak:.1f} N" if self._f else ""

    def summary(self) -> str:
        lines = [f"整个过程总力峰值 {self._peak:.1f} N"]
        names = {"tip": "指尖按压", "flat": "放平手掌"}
        for ph in ("tip", "flat"):
            seg = self._phase_seg.get(ph)
            if seg is None:
                lines.append(f"{names[ph]}：没有压稳（目标力未达到）")
                continue
            active = [(self._seg_names[i], float(v)) for i, v in enumerate(seg) if v > 0.05]
            active.sort(key=lambda x: -x[1])
            txt = "、".join(f"{n} {v:.1f}N" for n, v in active[:8])
            lines.append(f"{names[ph]}（{len(active)} 段受力，合计 {seg.sum():.1f} N）：{txt}")
        return "\n".join(lines)

    def result_curves(self):
        return [("总法向力", np.array(self._t), np.array(self._f))]


# ====================================================================== 演示 2：阻抗对比
class ImpedanceCompareDemo(Demo):
    key = "impedance"
    title = "阻抗：柔顺 / 默认 / 刚硬对比"
    description = (
        "食指下方放一块固定的蓝色方块。同样是“食指往下压到比方块更深的位置”这个指令，"
        "依次用 柔顺 / 默认 / 刚硬 三组阻抗参数各做一次，比较食指接触力：刚硬的像位置控制，"
        "指令越过头就会死命顶住；柔顺的只给出有限的力。"
    )
    result_title = "食指接触力对比（对齐到各自动作开始时刻）"
    result_ylabel = "食指法向力 (N)"

    MCP_PER_C = 1.2         # 食指保持伸直，只转根部关节：c=1 对应 1.2 rad
    C_CONTACT = 0.30        # 指尖刚好碰到方块时的 c
    C_CMD = 0.55            # 指令 c：比碰到方块的位置多转 0.3 rad（指令"越过头"）
    T_SETTLE = 0.3
    T_RAMP = 0.5
    T_HOLD = 1.4
    T_BACK = 0.6
    T_GAP = 0.3

    def __init__(self):
        super().__init__()
        self.trials = [PRESETS[0], PRESETS[1], PRESETS[2]]
        self._labels = [p[0] for p in self.trials]
        self.clock = _PhaseClock()
        self._trial = 0
        self._stage = -1
        self._t0 = 0.0
        self._curves: list[tuple[list[float], list[float]]] = [([], []) for _ in self.trials]
        self._index_force = 0.0

    @property
    def trial_len(self) -> float:
        return self.T_SETTLE + self.T_RAMP + self.T_HOLD + self.T_BACK + self.T_GAP

    # --- 场景布置 ---
    def _place_block(self, sim) -> None:
        """用正运动学找到食指弯到 C_CONTACT 时指尖的位置，把方块摆在指尖前进方向上。"""
        ex = sim.extras
        m = sim.model
        d = mujoco.MjData(m)
        qadr = {a.joint_name: a.qpos_adr for a in sim.actuators}

        def tip(c: float) -> np.ndarray:
            d.qpos[:] = 0
            d.qpos[qadr["index_mcp"]] = self.MCP_PER_C * c
            mujoco.mj_kinematics(m, d)
            return d.site("index_tip").xpos.copy()

        p0 = tip(self.C_CONTACT)
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "prop_block_geom")
        half = m.geom_size[gid]
        r_tip = 0.0077
        # 指尖从上往下压到方块上表面；方块向后(-x)多伸出一截，指尖往后滑也不会掉下去
        center = np.array([p0[0] - 0.02, p0[1], p0[2] - r_tip - half[2]])
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "prop_block")
        sim.data.mocap_pos[m.body_mocapid[bid]] = center

    def setup(self, sim) -> None:
        self._sim_setup(sim)

    def _sim_setup(self, sim) -> None:
        self._place_block(sim)
        sim.set_pose(FINGERS_OPEN, 0.0)
        self._apply_preset(sim, 0)
        # 方块的碰撞要在下一次 mj_forward 后才生效
        mujoco.mj_forward(sim.model, sim.data)

    def _sweep(self, c: float) -> dict[str, float]:
        return {"index_mcp": self.MCP_PER_C * c}

    def _apply_preset(self, sim, i: int) -> None:
        _, ks, ds, lim = self.trials[i]
        sim.drive.apply_preset(ks, ds, lim)

    # --- 每个控制周期 ---
    def update(self, sim, t: float) -> None:
        i = int(t // self.trial_len)
        if i >= len(self.trials):
            self.finished = True
            return
        if i != self._trial:
            self._trial, self._stage = i, -1
            self._apply_preset(sim, i)
        local = t - i * self.trial_len

        r = sim.tactile_reading
        self._index_force = r.group_normal("index") if r is not None else 0.0
        ts, fs = self._curves[i]
        ts.append(local)
        fs.append(self._index_force)

        # 事件：只在刚进入某阶段时下发一次目标
        stage = 0
        if local >= self.T_SETTLE:
            stage = 1
        if local >= self.T_SETTLE + self.T_RAMP + self.T_HOLD:
            stage = 2
        if stage != self._stage:
            self._stage = stage
            if stage == 1:
                sim.set_pose(self._sweep(self.C_CMD), self.T_RAMP)
            elif stage == 2:
                sim.set_pose(self._sweep(0.0), self.T_BACK)

    def teardown(self, sim) -> None:
        sim.drive.reset()
        bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "prop_block")
        sim.data.mocap_pos[sim.model.body_mocapid[bid]] = (0.0, 0.0, -1.0)

    # --- 结果 ---
    def trial_stats(self, i: int) -> dict:
        ts, fs = (np.array(x) for x in self._curves[i])
        if ts.size == 0:
            return {"peak": 0.0, "steady": 0.0, "onset": None}
        hold_lo = self.T_SETTLE + self.T_RAMP
        hold_hi = hold_lo + self.T_HOLD
        steady_mask = (ts >= hold_hi - 0.5) & (ts <= hold_hi)
        in_hold = (ts >= self.T_SETTLE) & (ts <= hold_hi)
        touched = np.nonzero(in_hold & (fs > 0.05))[0]
        return {
            "peak": float(fs[in_hold].max()) if in_hold.any() else 0.0,
            "steady": float(fs[steady_mask].mean()) if steady_mask.any() else 0.0,
            "onset": float(ts[touched[0]] - self.T_SETTLE) if touched.size else None,
        }

    def progress(self) -> str:
        i = min(self._trial, len(self.trials) - 1)
        return f"第 {i + 1}/{len(self.trials)} 次：{self._labels[i]}    食指受力 {self._index_force:.2f} N"

    def summary(self) -> str:
        lines = []
        for i, name in enumerate(self._labels):
            s = self.trial_stats(i)
            onset = "未碰到" if s["onset"] is None else f"{s['onset']:.2f}s 后接触"
            _, ks, ds, lim = self.trials[i]
            lines.append(
                f"{name}（K×{ks:g}, D×{ds:g}, 力矩≤{lim:g}N·m）：峰值 {s['peak']:.2f} N，稳态 {s['steady']:.2f} N，{onset}"
            )
        return "\n".join(lines)

    def result_curves(self):
        return [(self._labels[i], np.array(t), np.array(f)) for i, (t, f) in enumerate(self._curves) if t]
