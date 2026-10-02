"""从 MuJoCo 模型自省出可调参数，界面据此自动生成控件。

本模块不依赖 Qt。
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass

import mujoco


@dataclass(frozen=True)
class ActuatorSpec:
    """一个关节位置执行器对应的滑块规格（角度单位弧度，长度单位米）。"""

    index: int          # 在 data.ctrl 中的下标
    name: str           # 执行器名
    joint_name: str     # 驱动的关节名
    qpos_adr: int       # 关节在 data.qpos 中的地址
    lo: float
    hi: float
    default: float
    angular: bool = True    # True: 转动关节(rad)；False: 滑动关节(m)
    group: str = ""         # 分组名（关节名前缀），空字符串 = 不分组
    short: str = ""         # 组内显示名（去掉前缀后的部分）
    torque: bool = False    # True: 力矩(motor)执行器，指令是力矩，目标角度要经过阻抗控制器换算


def _name(model: mujoco.MjModel, obj: mujoco.mjtObj, idx: int, fallback: str) -> str:
    n = mujoco.mj_id2name(model, obj, idx)
    return n if n else fallback


def _assign_groups(joint_names: list[str]) -> list[tuple[str, str]]:
    """按关节名前缀分组：'index_mcp' -> ('index', 'mcp')。

    只有一个成员的前缀不成组（例如机械臂的 base_yaw / shoulder / elbow 保持平铺）。
    """
    prefixes = [n.split("_", 1)[0] if "_" in n else "" for n in joint_names]
    counts = Counter(p for p in prefixes if p)
    out = []
    for n, p in zip(joint_names, prefixes):
        if p and counts[p] >= 2:
            out.append((p, n.split("_", 1)[1]))
        else:
            out.append(("", n))
    return out


def describe_actuators(model: mujoco.MjModel) -> list[ActuatorSpec]:
    """列出模型里所有驱动单个关节的执行器。"""
    raw = []
    for i in range(model.nu):
        if model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT:
            continue  # 其他传动类型（肌腱、滑块曲柄等）暂不生成滑块
        jid = int(model.actuator_trnid[i, 0])
        jtype = int(model.jnt_type[jid])
        hinge, slide = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
        if jtype not in (hinge, slide):
            continue
        angular = jtype == hinge

        # 位置执行器：指令就是目标位置，范围取 ctrlrange；
        # 力矩执行器：ctrlrange 是力矩范围，滑块（目标角度）的范围取关节范围
        is_position = (
            model.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_FIXED
            and model.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_AFFINE
            and abs(model.actuator_biasprm[i, 1] + model.actuator_gainprm[i, 0]) < 1e-9
        )
        if is_position and model.actuator_ctrllimited[i]:
            lo, hi = (float(x) for x in model.actuator_ctrlrange[i])
        elif model.jnt_limited[jid]:
            lo, hi = (float(x) for x in model.jnt_range[jid])
        else:
            lo, hi = (-math.pi, math.pi) if angular else (-0.5, 0.5)

        q0 = float(model.qpos0[model.jnt_qposadr[jid]])
        raw.append((i, jid, angular, lo, hi, q0, not is_position))

    joint_names = [_name(model, mujoco.mjtObj.mjOBJ_JOINT, jid, f"joint{jid}") for _, jid, *_ in raw]
    groups = _assign_groups(joint_names)

    specs: list[ActuatorSpec] = []
    for (i, jid, angular, lo, hi, q0, torque), jname, (grp, short) in zip(raw, joint_names, groups):
        specs.append(
            ActuatorSpec(
                index=i,
                name=_name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i, f"actuator{i}"),
                joint_name=jname,
                qpos_adr=int(model.jnt_qposadr[jid]),
                lo=lo,
                hi=hi,
                default=min(max(q0, lo), hi),
                angular=angular,
                group=grp,
                short=short,
                torque=torque,
            )
        )
    return specs
