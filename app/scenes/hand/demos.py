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


# ====================================================================== 抓取演示的共用小工具
def _trial_line_err(err: float | None) -> str:
    return "—" if err is None else f"{err * 1000:.1f} mm"


class _HandHome:
    """把手腕和手指目标平滑地送回默认值（每周期直接写 targets，不用 set_pose，见陷阱 T1）。"""

    def __init__(self, sim, t: float, duration: float = 1.5):
        self.start = sim.manual.targets.copy()
        self.goal = np.array([a.default for a in sim.actuators], dtype=float)
        self.t0, self.dur = t, duration

    def update(self, sim, t: float) -> bool:
        u = _smooth((t - self.t0) / self.dur)
        sim.manual.targets[:] = self.start + (self.goal - self.start) * u
        sim.manual.revision += 1
        return t - self.t0 >= self.dur


# ====================================================================== 演示 3：轻拿轻放易碎物
class FragilePickPlaceDemo(Demo):
    key = "fragile"
    title = "抓取：轻拿轻放易碎物"
    description = (
        "黄色方块是“易碎物”：任何一根手指对它的力超过 1.8 N 并持续 20 ms 就算碎（变红）。"
        "同一套捏取动作（靠近→下降→闭合到目标力→抬起→搬到绿色放置区→放下→松开）做两次："
        "① 柔顺阻抗、目标力 1.0 N；② 刚硬（≈位置控制）、目标力 3.0 N。看“抓取”页签的握力曲线和判定。"
        "（原计划柔顺用 1.2 N，实测柔顺参数下手指闭合到底拇指最多约 1.1 N，所以改成 1.0 N。）"
    )
    # 曲线画"单指最大力"而不是总握力：易碎阈值是按单指算的，拿总握力（两指之和）和它比会误导
    result_title = "两次试验的单指最大力（红线 = 易碎阈值）"
    result_ylabel = "单指最大力 (N)"
    camera = {"azimuth": 135.0, "elevation": -22.0, "distance": 0.55, "lookat": (0.15, -0.02, 0.15)}
    needs_object = True

    OBJECT = "fragile"
    FRAGILE_LIMIT = 1.8
    PLACE_XY = (0.15, -0.12)
    TRIALS = [
        # (标签, 阻抗预设下标, 目标力 N)
        ("轻拿轻放（柔顺, 目标1.0N）", 0, 1.0),     # 柔顺参数下闭合到底：食指 1.32 N、拇指 1.10 N，够不到 1.2 N
        ("用力抓（刚硬, 目标3.0N）", 2, 3.0),
    ]
    LAND_WINDOW = 0.3       # 落地冲击：物体碰到桌面后这么长时间内的物体—桌面法向力峰值

    def __init__(self):
        super().__init__()
        self.phase = "TRIAL"
        self.trial = 0
        self.results: list[dict] = []
        self._curves: list[tuple[list[float], list[float]]] = [([], []) for _ in self.TRIALS]
        self.seq = None

    # --- 生命周期 ---
    def setup(self, sim) -> None:
        self._default_object = sim.grasp.object_name if sim.grasp else None
        self._default_limit = sim.grasp.fragile_limit if sim.grasp else None
        sim.set_grasp_object(self.OBJECT, self.FRAGILE_LIMIT)
        self._start_trial(sim, 0, 0.0)

    def _start_trial(self, sim, i: int, t: float) -> None:
        from app.scenes.hand.grasp_sequence import PinchGraspSequence, PinchParams
        self.trial, self.phase = i, "TRIAL"
        _, preset, F = self.TRIALS[i]
        _, ks, ds, lim = PRESETS[preset]
        sim.drive.apply_preset(ks, ds, lim)
        self.seq = PinchGraspSequence()
        self.seq.start(sim, self.OBJECT, PinchParams(F_target=F, place_xy=self.PLACE_XY), t=t)
        self._t0 = t
        self._land_t: float | None = None
        self._table_peak = 0.0
        self._finger_peak = 0.0

    def update(self, sim, t: float) -> None:
        from app.core.grasp import pair_normal_force
        if self.phase == "TRIAL":
            seq = self.seq
            seq.update(sim, t)
            gm = sim.grasp
            m = gm.metrics()
            ts, fs = self._curves[self.trial]
            ts.append(t - self._t0)
            fs.append(m["finger_max"])
            self._finger_peak = max(self._finger_peak, m["finger_max"])
            # 落地冲击：放下阶段物体第一次碰到桌面起 0.3 s 内的物体—桌面法向力峰值
            if seq.state in ("LOWER", "RELEASE", "RETREAT") or self._land_t is not None:
                f_table = pair_normal_force(sim.model, sim.data, {gm.body}, {0})
                # 序列器在碰到桌面的同一周期就进入 RELEASE，所以两个状态都算"刚落地"
                if self._land_t is None and seq.state in ("LOWER", "RELEASE") and f_table >= 0.1:
                    self._land_t = t
                if self._land_t is not None and t - self._land_t <= self.LAND_WINDOW:
                    self._table_peak = max(self._table_peak, f_table)
            if seq.finished:
                self._record(sim, t)
                if self.trial + 1 < len(self.TRIALS):
                    self.phase = "RESET_OBJECT"
                    self._home = _HandHome(sim, t)
                else:
                    self.finished = True
        elif self.phase == "RESET_OBJECT":
            # 不能 sim.reset()（会把演示状态也清掉）：手回到初始，物体单独放回 A 点
            if self._home.update(sim, t):
                sim.reset_object(self.OBJECT)
                self._start_trial(sim, self.trial + 1, t)

    def _record(self, sim, t: float) -> None:
        seq, gm = self.seq, sim.grasp
        m = gm.metrics()
        placed = seq.done
        err = float(np.linalg.norm(np.array(m["obj_pos"][:2]) - self.PLACE_XY)) if placed else None
        self.results.append({
            "label": self.TRIALS[self.trial][0],
            "done": seq.done,
            "failed": seq.failed,
            "broken": gm.broken,
            "lift_end_state": seq.lift_end_state,
            "final_state": gm.state,
            "peak_finger_force": self._finger_peak,
            "place_error": err,
            "peak_table_force": self._table_peak if self._land_t is not None else None,
            "duration": t - self._t0,
        })

    def teardown(self, sim) -> None:
        sim.drive.reset()
        if sim.grasp is not None:
            sim.grasp.restore_color()
        sim.reset_object(self.OBJECT)
        if self._default_object:
            sim.set_grasp_object(self._default_object, self._default_limit)

    # --- 结果 ---
    def progress(self) -> str:
        if self.phase == "RESET_OBJECT":
            return f"第 {self.trial + 1} 次结束，手复位、物体放回 A 点"
        label = self.TRIALS[self.trial][0]
        seq = self.seq
        return f"第 {self.trial + 1}/{len(self.TRIALS)} 次：{label}    阶段：{seq.label if seq else ''}    单指峰值 {self._finger_peak:.2f} N"

    def summary(self) -> str:
        lines = []
        for r in self.results:
            head = f"{r['label']}："
            if r["broken"]:
                lines.append(f"{head}碎了（单指峰值握力 {r['peak_finger_force']:.2f} N > 阈值 {self.FRAGILE_LIMIT:.1f} N），"
                             f"用时 {r['duration']:.1f} s")
            elif r["done"]:
                ok = "成功" if r["lift_end_state"] == "抓牢" else f"放下了，但抬起时判定为“{r['lift_end_state']}”"
                impact = "—" if r["peak_table_force"] is None else f"{r['peak_table_force']:.2f} N"
                lines.append(f"{head}{ok}，峰值握力 {r['peak_finger_force']:.2f} N，放置误差 {_trial_line_err(r['place_error'])}，"
                             f"落地冲击 {impact}，用时 {r['duration']:.1f} s")
            else:
                lines.append(f"{head}失败：{r['failed']}（单指峰值握力 {r['peak_finger_force']:.2f} N），用时 {r['duration']:.1f} s")
        return "\n".join(lines)

    def result_curves(self):
        out = [(self.TRIALS[i][0], np.array(t), np.array(f)) for i, (t, f) in enumerate(self._curves) if t]
        if out:
            t_max = max(float(c[1][-1]) for c in out)
            out.append((f"易碎阈值 {self.FRAGILE_LIMIT:.1f} N", np.array([0.0, t_max]), np.array([self.FRAGILE_LIMIT] * 2)))
        return out


# ====================================================================== 演示 4：位置控制 vs 阻抗控制抓球
class CompareGraspDemo(Demo):
    key = "compare"
    title = "抓取：位置控制 vs 阻抗控制抓球"
    description = (
        "同一套捏取动作、同样的目标力 1.5 N，只换手指的控制刚度：位置式（刚硬 K×5）和阻抗式（柔顺 K×0.5）。"
        "每组再故意把手腕目标沿捏取轴偏 0 / 4 / 8 mm（模拟定位误差），共 6 次。"
        "记录闭合时的冲击力峰值、稳态握力、球被推开的距离、抬起时的判定。"
        "（原计划若球捏不稳就换圆柱；实测有了静摩擦求解后球能稳定捏起，圆柱反而没更好，所以保留球，见附录 A。）"
    )
    result_title = "偏 8 mm 时两组的总握力（以开始闭合为 0 点）"
    result_ylabel = "总握力 (N)"
    camera = {"azimuth": 150.0, "elevation": -20.0, "distance": 0.42, "lookat": (0.03, -0.10, 0.13)}
    needs_object = True

    OBJECT = "ball"
    F_TARGET = 1.5
    LIFT = 0.06              # 抓牢要求抬升 ≥ 5 cm，抬 6 cm 才有余量
    GROUPS = [("位置式（刚硬）", 2), ("阻抗式（柔顺）", 0)]
    ERRORS = [0.0, 0.004, 0.008]

    def __init__(self):
        super().__init__()
        self.trials = [(g, p, e) for e in self.ERRORS for g, p in self.GROUPS]   # 两组交替：同一误差下前后对比
        self.results: list[dict] = []
        self._curves: list[tuple[list[float], list[float]]] = [([], []) for _ in self.trials]
        self.trial = 0
        self.phase = "TRIAL"
        self.seq = None

    def setup(self, sim) -> None:
        self._default_object = sim.grasp.object_name if sim.grasp else None
        self._default_limit = sim.grasp.fragile_limit if sim.grasp else None
        sim.set_grasp_object(self.OBJECT)
        self._start_trial(sim, 0, 0.0)

    def _start_trial(self, sim, i: int, t: float) -> None:
        from app.scenes.hand.grasp_sequence import PinchGraspSequence, PinchParams
        self.trial, self.phase = i, "TRIAL"
        _, preset, e = self.trials[i]
        _, ks, ds, lim = PRESETS[preset]
        sim.drive.apply_preset(ks, ds, lim)
        self.seq = PinchGraspSequence()
        self.seq.start(sim, self.OBJECT, PinchParams(F_target=self.F_TARGET, lift=self.LIFT, offset=e), t=t)
        self._t0 = t
        self._p0 = np.array(sim.grasp.metrics()["obj_pos"][:2])
        self._impact = 0.0
        self._push = 0.0
        self._settle: list[tuple[float, float]] = []

    def update(self, sim, t: float) -> None:
        if self.phase == "WAIT":
            if t - self._wait_t0 >= 0.3:
                sim.reset_object(self.OBJECT)
                self._start_trial(sim, self.trial + 1, t)
            return
        seq = self.seq
        seq.update(sim, t)
        m = sim.grasp.metrics()
        if seq.state in ("CLOSE", "SETTLE"):
            t_close = seq.entered("CLOSE")
            ts, fs = self._curves[self.trial]
            ts.append(t - t_close)
            fs.append(m["grip_total"])
            self._impact = max(self._impact, m["grip_total"])
            if seq.state == "CLOSE":
                self._push = max(self._push, float(np.linalg.norm(np.array(m["obj_pos"][:2]) - self._p0)))
            else:
                self._settle.append((t, m["grip_total"]))
        if seq.finished:
            self._record(sim)
            if self.trial + 1 < len(self.trials):
                self.phase, self._wait_t0 = "WAIT", t
            else:
                self.finished = True

    def _record(self, sim) -> None:
        seq = self.seq
        name, _, e = self.trials[self.trial]
        if self._settle:
            t_end = self._settle[-1][0]
            steady = float(np.mean([f for tt, f in self._settle if tt >= t_end - 0.2]))
        else:
            steady = 0.0
        lifted = seq.lift_end_state is not None and seq.entered("CARRY") is not None
        self.results.append({
            "group": name, "error": e,
            "impact_peak": self._impact,
            "steady_force": steady,
            "push_away": self._push,
            "final_state": seq.lift_end_state or "未抬起",
            "lifted": lifted,
            "failed": seq.failed,
        })

    def teardown(self, sim) -> None:
        sim.drive.reset()
        sim.reset_object(self.OBJECT)
        if self._default_object:
            sim.set_grasp_object(self._default_object, self._default_limit)

    # --- 结果 ---
    def progress(self) -> str:
        name, _, e = self.trials[self.trial]
        stage = "物体放回原处" if self.phase == "WAIT" else (self.seq.label if self.seq else "")
        return f"第 {self.trial + 1}/{len(self.trials)} 次：{name}，偏 {e * 1000:.0f} mm    阶段：{stage}"

    def _row(self, r: dict) -> str:
        why = f"（{r['failed']}）" if r["failed"] and not r["lifted"] else ""
        return (f"{r['group']} 偏 {r['error'] * 1000:.0f} mm：冲击峰值 {r['impact_peak']:.2f} N，稳态 {r['steady_force']:.2f} N，"
                f"推开 {r['push_away'] * 1000:.1f} mm，抬起时判定“{r['final_state']}”{why}")

    def conclusion(self) -> str:
        """一句话总结：只写测到的事实（偏 8 mm 时两组推开的距离和冲击力）。"""
        by = {(r["group"], round(r["error"] * 1000)): r for r in self.results}
        stiff, soft = self.GROUPS[0][0], self.GROUPS[1][0]
        a, b = by.get((stiff, 8)), by.get((soft, 8))
        if not (a and b):
            return ""
        more = "刚硬" if a["push_away"] > b["push_away"] else "柔顺"
        return (f"偏 8 mm 时：刚硬把球推开 {a['push_away'] * 1000:.1f} mm、冲击峰值 {a['impact_peak']:.2f} N；"
                f"柔顺推开 {b['push_away'] * 1000:.1f} mm、冲击峰值 {b['impact_peak']:.2f} N（{more}推得更远）。"
                f"两组抬起成功 {sum(r['lifted'] for r in self.results if r['group'] == stiff)}/3 与 "
                f"{sum(r['lifted'] for r in self.results if r['group'] == soft)}/3。")

    def summary(self) -> str:
        lines = [self._row(r) for r in self.results]
        c = self.conclusion()
        return "\n".join(lines + ([c] if c else []))

    def result_curves(self):
        out = []
        for i, (name, _, e) in enumerate(self.trials):
            t, f = self._curves[i]
            if t and abs(e - 0.008) < 1e-9:
                out.append((name, np.array(t), np.array(f)))
        return out
