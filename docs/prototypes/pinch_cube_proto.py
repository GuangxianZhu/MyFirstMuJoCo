"""【参考原型，不是正式代码】拇指+食指捏起桌上的方块：验证"能抓起来"的最小实验。

    MUJOCO_GL=egl python docs/prototypes/pinch_cube_proto.py        # 无头环境；Windows 直接运行即可

做了什么（这就是 docs/NEXT_STAGE_B.md 里 B3 要写成正式模块的流程）：
  1. 在 hand.xml 文本里注入"桌子 + 方块"，写到临时目录（旁边放一个 extras 垫片，Simulation 才会加载手的配置）
  2. 手指姿态用一个参数 s∈[0,1] 在"张开的捏取姿态 A"和"捏紧姿态 B"之间插值
  3. 用正运动学求出：指尖间距 = 方块边长 + 两个指尖半径 时的 s_c 和两指尖中点 M
  4. 手腕摆到使 M 与方块中心重合 → 从上方下降 → 以 s 闭合，直到食指/拇指对方块的法向力都 ≥ F_T
  5. 抬起 10 cm，保持 1 s，判断方块是否跟着升起

已验证的结果（本机沙箱 MuJoCo 3.14）：每指约 1.4 N，抬升约 8.4 cm 并保持；
把 CONDIM 改成 3（不开扭转摩擦）→ 抬不起来：方块在指尖间绕捏取轴转动后掉回桌面，最终 dz≈0。
"""
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402

HAND_XML = ROOT / "app" / "scenes" / "hand" / "hand.xml"

# ---- 参数（都是实测可用的起点）----
HALF = 0.015            # 方块半边长 (m)，边长 3 cm
MASS = 0.03             # kg
TABLE_TOP = 0.10        # 桌面高度 (m)
CONDIM, TORSION = 4, 0.02   # 扭转摩擦：condim=4 + friction 第二项。不开的话方块会在指尖间转掉
F_T = 1.5               # 闭合停止的法向力目标 (N)，食指和拇指都达到才停
CLOSE_RATE = 0.15       # s 的闭合速度 (1/s)
LIFT, LIFT_SPEED = 0.10, 0.08
R_INDEX, R_THUMB = 0.0077, 0.0085     # 指尖胶囊半径
L_INDEX, L_THUMB = 0.024, 0.026       # 指尖胶囊轴长（从 dist 关节原点到"半球球心"）

# 张开的捏取姿态 A → 捏紧姿态 B。c = 食指弯曲度，opp = 拇指对掌程度（见 hand_extras.py 的 FINGER_CURL / OK_THUMB）
A = dict(c=0.30, opp=0.50)
B = dict(c=0.55, opp=0.85)


def pinch_pose(s: float) -> dict[str, float]:
    c = A["c"] + (B["c"] - A["c"]) * s
    opp = A["opp"] + (B["opp"] - A["opp"]) * s
    pose = {f"index_{j}": v * c for j, v in {"mcp": 1.35, "pip": 1.75, "dip": 1.15}.items()}
    pose.update({"thumb_rot": 1.026 * opp, "thumb_cmc": 0.628 * opp, "thumb_mcp": 0.312 * opp, "thumb_ip": 0.392 * opp})
    for f in ("middle", "ring", "little"):          # 其余三指稍微收起，别碰桌面
        pose.update({f"{f}_mcp": 0.1, f"{f}_pip": 0.1, f"{f}_dip": 0.05})
    return pose


def build_scene(tmp: Path, cube_xy=(0.15, 0.0)) -> Path:
    text = HAND_XML.read_text(encoding="utf-8")
    inject = f"""
    <geom name="table" type="box" size="0.35 0.35 0.05" pos="0.10 0 0.05" material="grid" contype="1" conaffinity="1" friction="1.0 0.005 0.0001"/>
    <body name="cube" pos="{cube_xy[0]} {cube_xy[1]} {TABLE_TOP + HALF + 0.0005}">
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="box" size="{HALF} {HALF} {HALF}" mass="{MASS}" rgba="0.9 0.35 0.3 1"
            contype="1" conaffinity="1" condim="{CONDIM}" friction="1.0 {TORSION} 0.0001"/>
    </body>
"""
    marker = '    <body name="wrist_x_carrier"'
    assert marker in text
    xml = tmp / "pinch_scene.xml"
    xml.write_text(text.replace(marker, inject + "\n" + marker, 1), encoding="utf-8")
    (tmp / "pinch_scene_extras.py").write_text(
        f"import sys\nsys.path.insert(0, r'{ROOT}')\nfrom app.scenes.hand.hand_extras import create\n", encoding="utf-8")
    return xml


def tip_centers(m, d):
    """食指/拇指指尖"半球球心"的世界坐标（注意：hand.xml 里的 *_tip site 在胶囊最外端，不是球心）。"""
    def c(body, L):
        b = d.body(body)
        return b.xpos + b.xmat.reshape(3, 3)[:, 0] * L
    return c("index_dist", L_INDEX), c("thumb_dist", L_THUMB)


def run(cube_xy=(0.15, 0.0), verbose=True) -> dict:
    with tempfile.TemporaryDirectory() as t:
        sim = Simulation(build_scene(Path(t), cube_xy))
    m, d = sim.model, sim.data
    J, T = sim.joint_index, sim.manual.targets
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")

    # --- 3) 正运动学：找 s_c 和指尖中点 M ---
    scratch = mujoco.MjData(m)
    qadr = {a.joint_name: a.qpos_adr for a in sim.actuators}

    def tips_at(s):
        scratch.qpos[:] = 0
        for k, v in pinch_pose(s).items():
            scratch.qpos[qadr[k]] = v
        mujoco.mj_kinematics(m, scratch)
        return tip_centers(m, scratch)

    gap = 2 * HALF + R_INDEX + R_THUMB                 # 接触时两指尖球心的距离
    ss = np.linspace(0, 1, 101)
    dist = [np.linalg.norm(np.subtract(*tips_at(x))) for x in ss]
    s_c = float(ss[int(np.argmin(np.abs(np.array(dist) - gap)))])
    a, b = tips_at(s_c)
    M = (a + b) / 2
    cube0 = d.xpos[cid].copy()
    # 方块朝向对准捏取轴（两面法线沿指尖连线），否则面不平行，容易歪
    yaw = np.arctan2((b - a)[1], (b - a)[0])
    qa = m.jnt_qposadr[m.body_jntadr[cid]]
    d.qpos[qa + 3: qa + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    wrist = cube0 - M                                  # 手腕零位下 M 在 M，要让 M 到方块中心

    def setpose(s):
        for k, v in pinch_pose(s).items():
            T[J(k)] = v

    def run_for(sec, hook=None):
        for i in range(int(sec / sim.timestep)):
            if hook and i % 2 == 0:
                hook()
            sim.step_once()

    def forces():
        out = {"index": 0.0, "thumb": 0.0}
        for ci in range(d.ncon):
            c = d.contact[ci]
            if gid not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == gid else c.geom1
            name = m.body(m.geom_bodyid[other]).name
            f6 = np.zeros(6)
            mujoco.mj_contactForce(m, d, ci, f6)
            for k in out:
                if name.startswith(k):
                    out[k] += max(f6[0], 0.0)
        return out

    # --- 4) 张开 → 到方块上方 6 cm → 下降 → 闭合 ---
    setpose(0.0)
    T[J("wrist_x")], T[J("wrist_y")], T[J("wrist_z")] = wrist[0], wrist[1], wrist[2] + 0.06
    run_for(2.0)
    run_for(1.8, lambda: T.__setitem__(J("wrist_z"), max(T[J("wrist_z")] - 0.04 * sim.control_dt * 2, wrist[2])))
    run_for(0.5)
    st = {"s": 0.0, "done": False}

    def close():
        f = forces()
        if min(f.values()) >= F_T:
            st["done"] = True
        if not st["done"] and st["s"] < 1.0:
            st["s"] += CLOSE_RATE * sim.control_dt * 2
        setpose(st["s"])

    run_for(6.0, close)
    f_close, z0 = forces(), float(d.xpos[cid][2])

    # --- 5) 抬起并保持 ---
    run_for(LIFT / LIFT_SPEED + 0.3, lambda: T.__setitem__(J("wrist_z"), min(T[J("wrist_z")] + LIFT_SPEED * sim.control_dt * 2, wrist[2] + LIFT)))
    run_for(1.0)
    dz = float(d.xpos[cid][2]) - z0
    res = {"lifted": dz > 0.8 * LIFT, "dz": dz, "s_c": s_c, "s_end": st["s"], "force_at_close": f_close, "wrist_target": wrist}
    if verbose:
        print(f"lifted={res['lifted']}  dz={dz * 1000:.1f} mm  s_c={s_c:.2f}  s_end={st['s']:.2f}  "
              f"F(index/thumb)={f_close['index']:.2f}/{f_close['thumb']:.2f} N  wrist_target={np.round(wrist, 3)}")
    return res


if __name__ == "__main__":
    run()
