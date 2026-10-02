"""仿真封装：加载模型、按真实时间推进、控制器按固定控制周期调用。

控制链（每个控制周期）：
    演示脚本(可选) -> 控制器(手动/节奏) 得到目标角度 -> 关节阻抗(力矩型执行器)得到指令 -> data.ctrl
同时读一次触觉、更新抓取指标（有物体的场景）、往示波器写一行数据。

本模块不依赖 Qt，可以无界面批量运行。
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .controllers import Controller, ManualController, Observation, RhythmController
from .demo import Demo
from .grasp import GraspMonitor
from .impedance import DEFAULT_D, DEFAULT_K, DEFAULT_LIMIT, JointImpedance
from .params import ActuatorSpec, describe_actuators
from .physics import PhysicsParams
from .scene_extras import SceneExtras, load_extras
from .scope import Scope
from .tactile import TactileReading, TactileSensor

MAX_STEPS_PER_ADVANCE = 200  # 防止卡顿后一次追太多步


class Simulation:
    def __init__(self, xml_path: str | Path, control_dt: float = 0.004):
        self.xml_path = Path(xml_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)

        self.actuators: list[ActuatorSpec] = describe_actuators(self.model)
        self._act_idx = np.array([a.index for a in self.actuators], dtype=int)
        self._qadr = np.array([a.qpos_adr for a in self.actuators], dtype=int)
        # 关节速度地址（hinge/slide 的 dof 地址）
        self._dofadr = np.array(
            [int(self.model.jnt_dofadr[self.model.actuator_trnid[a.index, 0]]) for a in self.actuators],
            dtype=int,
        )

        # 控制周期必须是物理步长的整数倍
        steps = max(1, round(control_dt / self.model.opt.timestep))
        self.control_dt = steps * float(self.model.opt.timestep)
        self.paused = False
        self._accum = 0.0          # 尚未推进的仿真时间
        self._next_control = 0.0

        # 可在线调节的物理参数（重力/阻尼/质量/kp/kv）；"重置仿真"不会改动它们
        self.physics = PhysicsParams(self.model, self.actuators, self._dofadr)

        defaults = np.array([a.default for a in self.actuators], dtype=float)
        # manual 始终存在，保存滑块/手势设定的目标；controller 是当前生效的控制器
        # （手动时就是 manual，节奏模式时是包着 manual 的 RhythmController）
        self.manual = ManualController(defaults)
        self.controller: Controller = self.manual
        self._joint_index = {a.joint_name: k for k, a in enumerate(self.actuators)}
        self._last_targets = defaults.copy()

        # 场景附加配置（协同、预设手势、触觉段、演示…），没有就是 None
        self.extras: SceneExtras | None = load_extras(self.xml_path)
        self.rhythm: RhythmController | None = None

        # 关节阻抗：只作用于力矩型执行器，位置型原样透传
        imp = (self.extras.impedance if self.extras else None) or {}
        hw = np.array(
            [
                float(np.abs(self.model.actuator_ctrlrange[a.index]).max()) if a.torque else np.inf
                for a in self.actuators
            ]
        )
        self.drive = JointImpedance(
            self.actuators,
            k=imp.get("k", DEFAULT_K),
            d=imp.get("d", DEFAULT_D),
            tau_limit=imp.get("tau_limit", DEFAULT_LIMIT),
            hardware_limit=hw,
        )

        # 触觉
        seg = self.extras.tactile_segments if self.extras else None
        self.tactile: TactileSensor | None = TactileSensor(self.model, seg) if seg else None
        n_seg = self.tactile.n if self.tactile else 0
        self.tactile_reading: TactileReading | None = None

        self.scope = Scope(len(self.actuators), n_seg)

        # 抓取指标（只在有物体的场景里有）：只统计"手 ↔ 目标物体"的接触
        g = self.extras.grasp if self.extras else None
        self.grasp: GraspMonitor | None = (
            GraspMonitor(self, g["object"], g.get("fragile_limit")) if g and self.tactile else None
        )

        # 演示
        self.demo: Demo | None = None
        self.last_demo: Demo | None = None
        self._demo_t0 = 0.0

        self.reset()

    # ------------------------------------------------------------------ 状态
    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    def joint_angles(self) -> np.ndarray:
        """受驱动关节当前角度 (rad)。"""
        return self.data.qpos[self._qadr].copy()

    def observation(self) -> Observation:
        return Observation(
            time=self.time,
            q=self.data.qpos[self._qadr].copy(),
            qd=self.data.qvel[self._dofadr].copy(),
        )

    def joint_torques(self) -> np.ndarray:
        """最近一个物理步里各执行器施加的力/力矩（按滑块顺序）。"""
        return self.data.actuator_force[self._act_idx].copy()

    # ------------------------------------------------------------------ 控制
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.controller.reset()
        # 让关节从默认目标位置开始，避免一开始大幅摆动
        for a, adr in zip(self.actuators, self._qadr):
            self.data.qpos[adr] = a.default
        mujoco.mj_forward(self.model, self.data)
        self._accum = 0.0
        self._next_control = 0.0
        self._last_targets = self.manual.targets.copy()
        self.drive.last_tau[:] = 0.0
        self.scope.clear()
        self.tactile_reading = self.tactile.read(self.data) if self.tactile else None
        if self.grasp is not None:
            self.grasp.reset()

    # --- 抓取物体 ---
    def set_grasp_object(self, name: str, fragile_limit: float | None = None) -> GraspMonitor:
        """切换抓取指标统计的物体（演示用）。旧物体的颜色先还原。"""
        if self.tactile is None:
            raise RuntimeError("当前场景没有触觉配置，不能统计抓取指标")
        if self.grasp is not None:
            self.grasp.restore_color()
        self.grasp = GraspMonitor(self, name, fragile_limit)
        return self.grasp

    def reset_object(self, name: str) -> None:
        """只把一个自由物体放回 XML 里写的初始位姿（速度清零），不动手和演示状态。

        初始位姿取 model.qpos0（就是 XML 里写的位置）。若它正是抓取指标的对象，指标一并清零。
        """
        m = self.model
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if b < 0:
            raise ValueError(f"刚体不存在: {name}")
        j = int(m.body_jntadr[b])
        if j < 0 or m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError(f"{name} 不是自由物体")
        qa, va = int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j])
        self.data.qpos[qa:qa + 7] = m.qpos0[qa:qa + 7]
        self.data.qvel[va:va + 6] = 0.0
        mujoco.mj_forward(m, self.data)
        if self.grasp is not None and self.grasp.body == b:
            self.grasp.reset()

    def set_controller(self, controller: Controller) -> None:
        """切换当前生效的控制器（不会重置 manual 里的目标）。"""
        self.controller = controller

    # --- 目标设定：滑块、手势、协同都走这里，下标是"滑块顺序"(self.actuators 的顺序) ---
    def joint_index(self, joint_name: str) -> int:
        return self._joint_index[joint_name]

    def set_target(self, k: int, value: float) -> None:
        self.manual.set_target(k, value)

    def set_pose(self, pose: dict[str, float], duration: float = 0.0) -> None:
        """按关节名设置一组目标，duration 秒内平滑过渡（0 = 立即）。未提及的关节保持不变。"""
        goal = {}
        for name, v in pose.items():
            k = self._joint_index[name]
            a = self.actuators[k]
            goal[k] = min(max(float(v), a.lo), a.hi)
        self.manual.move_to(goal, duration)

    # --- 节奏动作 ---
    @property
    def rhythm_active(self) -> bool:
        return self.rhythm is not None and self.controller is self.rhythm

    def start_rhythm(self, pattern: str = "wave", freq: float = 0.8, amp: float = 1.0, lag: float = 0.12) -> RhythmController:
        if self.extras is None:
            raise RuntimeError("当前场景没有节奏动作配置")
        ex = self.extras

        def curl_targets(finger: str, curl: float) -> dict[int, float]:
            return {self._joint_index[j]: v for j, v in ex.curl_pose(finger, curl).items()}

        self.rhythm = RhythmController(self.manual, ex.fingers, curl_targets, pattern, freq, amp, lag)
        self.controller = self.rhythm
        return self.rhythm

    def stop_rhythm(self) -> None:
        """回到手动控制；手动目标取节奏停下那一刻的目标角度，手指不会突然跳变。"""
        if self.rhythm is not None:
            self.manual.targets = np.clip(
                self._last_targets.copy(),
                [a.lo for a in self.actuators],
                [a.hi for a in self.actuators],
            )
            self.manual._move = None
            self.manual.revision += 1
        self.controller = self.manual
        self.rhythm = None

    # --- 演示 ---
    @property
    def demo_active(self) -> bool:
        return self.demo is not None

    @property
    def demo_time(self) -> float:
        return self.time - self._demo_t0

    def start_demo(self, demo: Demo) -> None:
        """重置仿真并开始一个演示。"""
        self.stop_demo()
        self.stop_rhythm()
        self.reset()
        demo.finished = False
        demo.setup(self)
        self.demo = demo
        self._demo_t0 = self.time

    def stop_demo(self) -> None:
        if self.demo is not None:
            self.demo.teardown(self)
            self.last_demo = self.demo
            self.demo = None

    # ------------------------------------------------------------------ 推进
    def _control_tick(self) -> None:
        obs = self.observation()
        if self.demo is not None:
            self.demo.update(self, obs.time - self._demo_t0)
            if self.demo.finished:
                self.stop_demo()

        targets = np.asarray(self.controller.update(obs), dtype=float)
        self._last_targets = targets.copy()
        cmd = self.drive.compute(targets, obs.q, obs.qd) if self.drive.active else targets
        self.data.ctrl[self._act_idx] = cmd

        if self.tactile is not None:
            self.tactile_reading = self.tactile.read(self.data)
        if self.grasp is not None:
            self.grasp.update()
        tau = np.where(self.drive.mask, self.drive.last_tau, self.joint_torques())
        seg = self.tactile_reading.normal if self.tactile_reading is not None else None
        self.scope.push(obs.time, targets, obs.q, tau, seg)

    @property
    def control_due(self) -> bool:
        """下一次 step_once 会先跑控制链（无界面脚本据此"每个控制周期"做一次事情）。"""
        return self.data.time >= self._next_control - 1e-12

    def step_once(self) -> None:
        """推进一个物理步；到了控制周期就先跑一遍控制链。"""
        if self.control_due:
            self._control_tick()
            self._next_control += self.control_dt
        mujoco.mj_step(self.model, self.data)

    def advance(self, wall_dt: float) -> int:
        """按真实时间 wall_dt 推进仿真，返回实际执行的物理步数。"""
        if self.paused or wall_dt <= 0:
            return 0
        self._accum += wall_dt
        n = int(self._accum / self.timestep)
        n = min(n, MAX_STEPS_PER_ADVANCE)
        if n == MAX_STEPS_PER_ADVANCE:
            self._accum = 0.0  # 跟不上实时，丢弃积压
        else:
            self._accum -= n * self.timestep
        for _ in range(n):
            self.step_once()
        return n
