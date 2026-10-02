"""拇指 + 食指捏取序列器：靠近 → 下降 → 闭合（力终止）→ 稳定 → 抬起 → 搬运 → 放下 → 松开 → 撤离。

由 docs/prototypes/pinch_cube_proto.py 整理而来，供演示复用。用法：

    seq = PinchGraspSequence()
    seq.start(sim, "cube", PinchParams(F_target=1.5))
    每个控制周期：seq.update(sim, t)        # 演示里在 Demo.update 中调用
    seq.state / seq.failed / seq.done / seq.finished

约定：
  - 每个周期直接写 sim.manual.targets（并让 revision+1，界面滑块跟着动），不用 set_pose 过渡（陷阱 T1）
  - 不改 sim.drive 的阻抗参数（由演示负责），只管运动和力终止
  - 手指对物体的力取自 sim.grasp.reading（只含手 ↔ 物体），所以 start() 会把抓取指标切到这个物体
  - 任何失败都不抛异常：记下原因（failed），张开手指、抬起手腕（ABORT），然后 finished
本模块不依赖 Qt。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from app.core.grasp import pair_normal_force
from app.scenes.hand.hand_extras import FINGER_CURL, OK_THUMB

# 指尖胶囊：从 dist 关节原点到"半球球心"的轴长、半径（hand.xml）。*_tip site 在胶囊最外端，不是球心
L_INDEX, R_INDEX = 0.024, 0.0077
L_THUMB, R_THUMB = 0.026, 0.0085

WRIST = ("wrist_x", "wrist_y", "wrist_z")
NON_TIP_PREFIX = ("palm",) + tuple(f"{f}_{p}" for f in ("thumb", "index", "middle", "ring", "little") for p in ("prox", "mid"))

STATE_NAMES = {
    "IDLE": "待命", "APPROACH": "靠近", "DESCEND": "下降", "CLOSE": "闭合", "SETTLE": "稳定", "LIFT": "抬起",
    "CARRY": "搬运", "LOWER": "放下", "RELEASE": "松开", "RETREAT": "撤离", "DONE": "完成", "ABORT": "失败收尾",
}


@dataclass
class PinchParams:
    F_target: float = 1.5          # 闭合停止的力：食指、拇指对物体的法向力都 ≥ 此值 (N)
    close_rate: float = 0.15       # 闭合速度：s 每秒增加多少
    lift: float = 0.10             # 抬起高度 (m)
    lift_speed: float = 0.08       # m/s
    carry_speed: float = 0.10      # 搬运最大速度 m/s
    place_xy: tuple[float, float] | None = None   # 放置点；None = 放回原处
    hold: float = 0.3              # 抬到位后停一会儿再搬运 (s)
    approach_height: float = 0.06  # 先到物体上方这么高 (m)
    approach_speed: float = 0.10   # 最大速度 m/s
    descend_speed: float = 0.04
    lower_speed: float = 0.04
    release_rate: float = 0.30
    retreat: float = 0.06
    offset: float = 0.0            # 手腕目标沿捏取轴（食指→拇指方向）偏离物体中心的距离（模拟定位误差，m）
    align_object: bool = True      # 方块：开始前把它转到捏取轴方向（两个受力面的法线沿指尖连线）


# ---------------------------------------------------------------------- 手指姿态族
class LinearPinch:
    """s∈[0,1] 在 张开的捏取姿态 A 和 捏紧姿态 B 之间线性插值（docs/NEXT_STAGE_B.md 5.1）。"""

    name = "线性插值"

    def pose(self, s: float) -> dict[str, float]:
        s = min(max(s, 0.0), 1.0)
        c = 0.30 + 0.25 * s
        opp = 0.50 + 0.35 * s
        pose = {f"index_{j}": v * c for j, v in FINGER_CURL.items()}
        pose.update({k: v * opp for k, v in OK_THUMB.items()})   # 拇指各关节按 OK 姿态的比例
        for f in ("middle", "ring", "little"):                     # 其余三指稍微收起，别碰桌面
            pose.update({f"{f}_mcp": 0.1, f"{f}_pip": 0.1, f"{f}_dip": 0.05})
        return pose


def tip_centers(d: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    """食指、拇指指尖"半球球心"的世界坐标。"""
    def c(body: str, L: float) -> np.ndarray:
        b = d.body(body)
        return b.xpos + b.xmat.reshape(3, 3)[:, 0] * L
    return c("index_dist", L_INDEX), c("thumb_dist", L_THUMB)


def object_width(model: mujoco.MjModel, body: int) -> float:
    """物体在水平面上被捏的宽度（方块：边长；球、圆柱：直径）。"""
    g = next(i for i in range(model.ngeom) if model.geom_bodyid[i] == body)
    t, size = int(model.geom_type[g]), model.geom_size[g]
    G = mujoco.mjtGeom
    if t == int(G.mjGEOM_BOX):
        return 2.0 * float(size[0])
    if t in (int(G.mjGEOM_SPHERE), int(G.mjGEOM_CYLINDER), int(G.mjGEOM_CAPSULE)):
        return 2.0 * float(size[0])
    raise ValueError(f"不支持的物体形状: geom type {t}")


class _Ramp:
    """一维/多维从 a 到 b 的运动：smooth=True 用 smoothstep（峰值速度 = 1.5×平均速度 ≤ vmax），否则匀速。"""

    def __init__(self, a, b, vmax: float, t0: float, smooth: bool):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        dist = float(np.linalg.norm(self.b - self.a))
        self.dur = max(dist * (1.5 if smooth else 1.0) / vmax, 1e-6)
        self.t0, self.smooth = t0, smooth

    def at(self, t: float) -> np.ndarray:
        u = min(max((t - self.t0) / self.dur, 0.0), 1.0)
        if self.smooth:
            u = u * u * (3.0 - 2.0 * u)
        return self.a + (self.b - self.a) * u

    def done(self, t: float) -> bool:
        return t - self.t0 >= self.dur


class PinchGraspSequence:
    def __init__(self, family=None):
        self.family = family or LinearPinch()
        self.state = "IDLE"
        self.failed: str | None = None
        self.done = False
        self.events: list[tuple[float, str]] = []     # (进入时刻, 状态)
        self.plan: dict = {}
        self.lift_end_state: str | None = None

    @property
    def finished(self) -> bool:
        return self.done or (self.failed is not None and self.state == "IDLE")

    @property
    def label(self) -> str:
        return STATE_NAMES.get(self.state, self.state)

    def entered(self, state: str) -> float | None:
        """最近一次进入某状态的时刻（演示时间）。"""
        for t, s in reversed(self.events):
            if s == state:
                return t
        return None

    # ------------------------------------------------------------------ 规划
    def _tips_at(self, s: float) -> tuple[np.ndarray, np.ndarray]:
        sc = self._scratch
        sc.qpos[:] = 0.0
        for k, v in self.family.pose(s).items():
            sc.qpos[self._qadr[k]] = v
        mujoco.mj_kinematics(self._m, sc)
        return tip_centers(sc)

    def start(self, sim, object_body: str, params: PinchParams | None = None, t: float = 0.0) -> None:
        self.p = p = params or PinchParams()
        m, d = sim.model, sim.data
        self._m = m
        self._scratch = mujoco.MjData(m)           # 正运动学用草稿，不动 sim.data
        self._qadr = {a.joint_name: a.qpos_adr for a in sim.actuators}
        self._J = {n: sim.joint_index(n) for n in (*WRIST, *self.family.pose(0.0).keys())}

        if sim.grasp is None or sim.grasp.object_name != object_body:
            sim.set_grasp_object(object_body)
        self.body = sim.grasp.body
        self.object_body = object_body

        # 1) 接触时两指尖球心距离 = 物体宽度 + 两个指尖半径；在 s∈[0,1] 里找最接近的 s_c
        gap = object_width(m, self.body) + R_INDEX + R_THUMB
        ss = np.linspace(0.0, 1.0, 101)
        dist = np.array([np.linalg.norm(np.subtract(*self._tips_at(x))) for x in ss])
        s_c = float(ss[int(np.argmin(np.abs(dist - gap)))])
        a, b = self._tips_at(s_c)
        mid = (a + b) / 2.0
        axis = b - a
        axis_h = np.array([axis[0], axis[1], 0.0])
        axis_h /= max(np.linalg.norm(axis_h), 1e-9)
        yaw = math.atan2(axis[1], axis[0])

        # 2) 方块转到捏取轴方向（只转绕 z 的角度）
        j = int(m.body_jntadr[self.body])
        is_box = int(m.geom_type[next(g for g in range(m.ngeom) if m.geom_bodyid[g] == self.body)]) == int(mujoco.mjtGeom.mjGEOM_BOX)
        if p.align_object and is_box and j >= 0 and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            qa = int(m.jnt_qposadr[j])
            d.qpos[qa + 3: qa + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
            mujoco.mj_forward(m, d)
            sim.grasp.reset()

        # 3) 手腕零位时指尖中点在 mid，要让它落到物体中心：wrist = obj − mid（手腕关节零位就是草稿里的位置）
        obj = d.xpos[self.body].copy()
        wrist = obj - mid + p.offset * axis_h
        self.plan = {
            "s_c": s_c, "gap": gap, "tip_gap_at_sc": float(dist[int(round(s_c * 100))]),
            "wrist": wrist, "yaw": yaw, "axis": axis_h, "tip_dz": float(b[2] - a[2]), "object0": obj,
        }
        self._w = np.array([sim.manual.targets[self._J[n]] for n in WRIST])   # 当前手腕指令
        self._s = 0.0
        self.failed, self.done = None, False
        self.events = []
        self.peak = {"finger": 0.0}
        sim.manual._move = None                    # 取消可能正在进行的整体过渡（T1）
        self._go("APPROACH", t)
        self._ramp = _Ramp(self._w, wrist + [0.0, 0.0, p.approach_height], p.approach_speed, t, smooth=True)
        self._settle_since: float | None = None
        self._hit_end: float | None = None
        self.lift_end_state: str | None = None     # LIFT 结束（停够 hold）时抓取指标的判定

    # ------------------------------------------------------------------ 状态机
    def _go(self, state: str, t: float) -> None:
        self.state = state
        self._t0 = t
        self.events.append((t, state))

    def _fail(self, sim, t: float, why: str) -> None:
        if self.failed is None:
            self.failed = f"{STATE_NAMES.get(self.state, self.state)}阶段失败：{why}"
        self._go("ABORT", t)
        self._ramp = _Ramp(self._w, self._w + [0.0, 0.0, self.p.retreat], self.p.approach_speed, t, smooth=True)

    def _write(self, sim) -> None:
        T = sim.manual.targets
        for i, n in enumerate(WRIST):
            a = sim.actuators[self._J[n]]
            T[self._J[n]] = min(max(float(self._w[i]), a.lo), a.hi)
        for k, v in self.family.pose(self._s).items():
            T[self._J[k]] = v
        sim.manual.revision += 1                   # 界面滑块跟着动

    def _forces(self, sim) -> tuple[float, float]:
        r = sim.grasp.reading
        return r.group_normal("index"), r.group_normal("thumb")

    def update(self, sim, t: float) -> None:
        if self.state in ("IDLE", "DONE"):
            return
        p, dt = self.p, sim.control_dt
        el = t - self._t0
        fi, ft = self._forces(sim)
        self.peak["finger"] = max(self.peak["finger"], fi, ft)
        gstate = sim.grasp.state
        st = self.state

        if st == "APPROACH":
            self._s = 0.0
            self._w = self._ramp.at(t)
            if self._ramp.done(t):
                err = np.linalg.norm(sim.data.qpos[[self._qadr[n] for n in WRIST]] - self._w)
                if err < 0.002:
                    if self._settle_since is None:
                        self._settle_since = t
                    if t - self._settle_since >= 0.3:
                        self._go("DESCEND", t)
                else:
                    self._settle_since = None
            if el > 10.0:
                self._fail(sim, t, "10 s 内手腕没有到达物体上方")

        elif st == "DESCEND":
            wz = self.plan["wrist"][2]
            self._w[2] = max(self._w[2] - p.descend_speed * dt, wz)
            table = sim.tactile.read_against(sim.data, {0})
            bump = sum(table.normal[i] for i, n in enumerate(table.names) if n.startswith(NON_TIP_PREFIX))
            if bump > 0.3:
                self._fail(sim, t, f"手掌/指节碰到桌面（{bump:.2f} N），位置不对")
            elif self._w[2] <= wz + 1e-9:
                self._go("CLOSE", t)

        elif st == "CLOSE":
            if min(fi, ft) >= p.F_target:
                self._go("SETTLE", t)
            elif self._s >= 1.0:
                # s 到头后再给 0.5 s 让力跟上（阻抗有滞后）
                self._hit_end = self._hit_end or t
                if t - self._hit_end > 0.5:
                    self._fail(sim, t, f"手指已闭合到底仍达不到目标力 {p.F_target:.1f} N（食指 {fi:.2f} N、拇指 {ft:.2f} N）")
            else:
                self._hit_end = None
                self._s = min(self._s + p.close_rate * dt, 1.0)
            # 超时：文档原定 6 s，但 0.15/s 的闭合速度从 0 闭合到 1 本身就要 6.7 s，"s 到 1 仍不达标"永远触发不了；
            # 改成"闭合到底所需时间 + 1.5 s"
            limit = 1.0 / max(p.close_rate, 1e-6) + 1.5
            if self.state == "CLOSE" and el > limit:
                self._fail(sim, t, f"{limit:.1f} s 内没有夹到目标力（食指 {fi:.2f} N、拇指 {ft:.2f} N）")

        elif st == "SETTLE":
            if el >= 0.4:
                if min(fi, ft) < 0.5 * p.F_target:
                    self._fail(sim, t, f"夹持力掉到 {min(fi, ft):.2f} N（目标 {p.F_target:.1f} N），物体没夹稳")
                else:
                    self._go("LIFT", t)
                    self._lift_goal = self.plan["wrist"][2] + p.lift

        elif st in ("LIFT", "CARRY", "LOWER"):
            if gstate in ("滑落", "碎了"):
                self._fail(sim, t, "物体碎了" if gstate == "碎了" else "物体从手里滑落")
            elif st == "LIFT":
                self._w[2] = min(self._w[2] + p.lift_speed * dt, self._lift_goal)
                if self._w[2] >= self._lift_goal - 1e-9:
                    if el >= p.lift / p.lift_speed + p.hold:
                        self.lift_end_state = gstate
                        lift = sim.grasp.metrics()["lift"]
                        if lift < 0.5 * p.lift:
                            self._fail(sim, t, f"力不够，物体没抬起（只升高 {lift * 1000:.0f} mm）")
                        else:
                            self._go("CARRY", t)
                            xy = p.place_xy if p.place_xy is not None else self.plan["object0"][:2]
                            goal = self._w.copy()
                            goal[:2] = np.asarray(xy) - self.plan["object0"][:2] + self.plan["wrist"][:2]
                            self._ramp = _Ramp(self._w, goal, p.carry_speed, t, smooth=True)
            elif st == "CARRY":
                self._w = self._ramp.at(t)
                if self._ramp.done(t) and el >= self._ramp.dur + 0.2:
                    self._go("LOWER", t)
            else:   # LOWER：降到物体碰到桌面（物体 ↔ 世界的接触力 ≥ 0.1 N）
                self._w[2] = max(self._w[2] - p.lower_speed * dt, self.plan["wrist"][2] - 0.01)
                if pair_normal_force(sim.model, sim.data, {self.body}, {0}) >= 0.1 or self._w[2] <= self.plan["wrist"][2] - 0.01 + 1e-9:
                    sim.grasp.release()               # 主动放下：之后物体在桌上不算"滑落"
                    self._go("RELEASE", t)

        elif st == "RELEASE":
            self._s = max(self._s - p.release_rate * dt, 0.0)
            if fi + ft < 0.05 and sim.grasp.reading.total_normal() < 0.05:
                self._go("RETREAT", t)
                self._ramp = _Ramp(self._w, self._w + [0.0, 0.0, p.retreat], p.approach_speed, t, smooth=True)
            elif el > 4.0:
                self._fail(sim, t, "4 s 内没能松开物体")

        elif st == "RETREAT":
            self._s = max(self._s - p.release_rate * dt, 0.0)
            self._w = self._ramp.at(t)
            if self._ramp.done(t):
                self._go("DONE", t)
                self.done = True

        elif st == "ABORT":
            self._s = max(self._s - p.release_rate * dt, 0.0)
            self._w = self._ramp.at(t)
            if self._ramp.done(t) and self._s <= 0.0:
                self._go("IDLE", t)

        self._write(sim)


def run_sequence(sim, seq: PinchGraspSequence, timeout: float = 60.0) -> None:
    """无界面运行：每个控制周期调用一次 seq.update，直到结束或超时。"""
    t_end = sim.time + timeout
    while not seq.finished and sim.time < t_end:
        if sim.control_due:
            seq.update(sim, sim.time)
        sim.step_once()


# ---------------------------------------------------------------------- 水平对齐的捏取姿态族（决策门 G1 第 1 轮）
IK_JOINTS = ["index_mcp", "index_pip", "index_dip", "thumb_rot", "thumb_cmc", "thumb_mcp", "thumb_ip"]


class LevelPinch:
    """两指尖球心始终等高、中点不动的捏取姿态族：s∈[0,1] 对应开口 a 从 a_max 线性减到 a_min。

    用小 IK（高斯-牛顿、有限差分雅可比、阻尼最小二乘）预先算一张表：变量 = 食指 mcp/pip/dip + 拇指 rot/cmc/mcp/ip，
    约束 = ① 两指尖球心距离 = a；② 两指尖球心 z 相同；③ 两指尖中点在"水平面内垂直于捏取轴"的方向和竖直方向不动
    （线性插值闭合时中点一路往掌心方向、往下漂约 3 cm；沿捏取轴的漂移无所谓，等于定位误差沿轴，实测 ±1 cm 都能抓）。
    中点三个方向全固定在这只手上解不出来（拇指 mcp/ip 顶到限位，残差到 1 cm）。
    开口逐步减小，每步用上一步的解做初值（连续路径）。
    """

    name = "水平对齐（IK）"

    def __init__(self, model: mujoco.MjModel, qpos_adr: dict[str, int], a_max: float = 0.08, a_min: float = 0.03,
                 step: float = 0.005):
        self.a_values = np.round(np.arange(a_max, a_min - 1e-9, -step), 6)
        self._lin = LinearPinch()
        self._m, self._qadr = model, qpos_adr
        self._d = mujoco.MjData(model)
        self._lo = np.array([model.jnt_range[model.joint(n).id][0] for n in IK_JOINTS])
        self._hi = np.array([model.jnt_range[model.joint(n).id][1] for n in IK_JOINTS])
        self.table, self.residual = self._build()

    # --- IK ---
    def _tips(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        d = self._d
        d.qpos[:] = 0.0
        for k, v in self._lin.pose(0.0).items():          # 其余三指与线性族相同
            d.qpos[self._qadr[k]] = v
        for k, v in zip(IK_JOINTS, q):
            d.qpos[self._qadr[k]] = v
        mujoco.mj_kinematics(self._m, d)
        return tip_centers(d)

    def _residual(self, q, a, mid):
        A, B = self._tips(q)
        r = [np.linalg.norm(A - B) - a, A[2] - B[2]]
        if mid is not None:
            dm = (A + B) / 2.0 - mid
            r += [float(dm[:2] @ self._perp), float(dm[2])]
        return np.array(r)

    def _solve(self, q, a, mid, iters=60, lam=1e-3, eps=1e-6):
        q = q.copy()
        for _ in range(iters):
            r = self._residual(q, a, mid)
            if np.max(np.abs(r)) < 1e-5:
                break
            J = np.zeros((r.size, q.size))
            for j in range(q.size):
                dq = np.zeros_like(q)
                dq[j] = eps
                J[:, j] = (self._residual(q + dq, a, mid) - r) / eps
            step = -J.T @ np.linalg.solve(J @ J.T + lam * np.eye(r.size), r)
            q = np.clip(q + step, self._lo, self._hi)
        return q, self._residual(q, a, mid)

    def _build(self):
        lin0 = self._lin.pose(0.0)
        q = np.array([lin0[k] for k in IK_JOINTS])
        # 第一步只要求开口和等高，得到起始中点；之后中点固定
        q, r = self._solve(q, float(self.a_values[0]), None)
        A, B = self._tips(q)
        mid = (A + B) / 2.0
        ax = (B - A)[:2] / np.linalg.norm((B - A)[:2])
        self._perp = np.array([-ax[1], ax[0]])
        table, res = [], []
        for a in self.a_values:
            q, r = self._solve(q, float(a), mid)
            table.append(q.copy())
            res.append(r.copy())
        return np.array(table), res

    def max_dz(self) -> float:
        """表里每一行两指尖球心的高度差（绝对值）的最大值 (m)。"""
        return float(max(abs(self._residual(q, 0.0, None)[1]) for q in self.table))

    # --- 姿态族接口 ---
    def pose(self, s: float) -> dict[str, float]:
        s = min(max(s, 0.0), 1.0)
        x = s * (len(self.table) - 1)
        i = min(int(x), len(self.table) - 2)
        u = x - i
        q = self.table[i] * (1.0 - u) + self.table[i + 1] * u
        pose = {k: v for k, v in self._lin.pose(0.0).items() if k not in IK_JOINTS}
        pose.update({k: float(v) for k, v in zip(IK_JOINTS, q)})
        return pose
