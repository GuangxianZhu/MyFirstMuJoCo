"""决策门 G1（docs/NEXT_STAGE_B.md 第 7 节）的实验，可复现附录 A 里的数字。

    python tools/g1_ball_gate.py

每组实验 = 5 个位置：手按球的名义位置 (0.03, -0.12) 规划，球实际放在名义位置或偏 ±1 cm（x、y 各两次），
即"定位误差 ±1 cm"。成功 = 序列器抬起结束时抓取判定为"抓牢"。
另外打印"手知道球的真实位置"（无定位误差）时 5 个位置的结果。
"""
import shutil
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402
from app.scenes.hand.grasp_sequence import LevelPinch, PinchGraspSequence, PinchParams  # noqa: E402

SCENE_DIR = ROOT / "app" / "scenes" / "hand"
NOMINAL = np.array([0.03, -0.12])
OFFSETS = [(0.0, 0.0), (0.01, 0.0), (-0.01, 0.0), (0.0, 0.01), (0.0, -0.01)]

# 第 2 轮：4 块板围成的座，内口 24 mm（球半径的 1.2 倍），板高 4 mm（再高就挡住食指指尖）
H, T, IN = 0.004, 0.002, 0.012
SEAT = f"""
    <body name="ball_seat" pos="0.03 -0.12 {0.10 + H / 2}">
      <geom type="box" size="{T / 2} {IN + T} {H / 2}" pos="{IN + T / 2} 0 0" rgba="0.5 0.5 0.55 1"/>
      <geom type="box" size="{T / 2} {IN + T} {H / 2}" pos="{-IN - T / 2} 0 0" rgba="0.5 0.5 0.55 1"/>
      <geom type="box" size="{IN} {T / 2} {H / 2}" pos="0 {IN + T / 2} 0" rgba="0.5 0.5 0.55 1"/>
      <geom type="box" size="{IN} {T / 2} {H / 2}" pos="0 {-IN - T / 2} 0" rgba="0.5 0.5 0.55 1"/>
    </body>
"""
CYLINDER = """
    <body name="ball" pos="0.03 -0.12 0.1205">
      <freejoint name="ball_free"/>
      <geom name="ball_geom" type="cylinder" size="0.015 0.02" mass="0.03" rgba="0.30 0.60 0.90 1"
            contype="1" conaffinity="1" condim="4" friction="1.0 0.02 0.0001"/>
    </body>
"""


def variant_scene(tmp: Path, name: str, extra: str = "", cylinder: bool = False) -> Path:
    """在临时目录里生成一份 hand_table 的变体（座、圆柱），旁边放手模型和 extras 垫片。"""
    shutil.copy(SCENE_DIR / "hand.xml", tmp / "hand.xml")
    text = (SCENE_DIR / "hand_table.xml").read_text(encoding="utf-8")
    if cylinder:
        a = text.index('    <body name="ball"')
        b = text.index("    </body>", a) + len("    </body>\n")
        text = text[:a] + CYLINDER + text[b:]
    text = text.replace("  </worldbody>", extra + "  </worldbody>", 1)
    xml = tmp / f"{name}.xml"
    xml.write_text(text, encoding="utf-8")
    (tmp / f"{name}_extras.py").write_text(
        f"import sys\nsys.path.insert(0, r'{ROOT}')\nfrom app.scenes.hand.hand_table_extras import create\n", encoding="utf-8")
    return xml


def trial(xml, dxy, *, family=False, noslip=True, damping=0.0, know_position=False) -> tuple[bool, str]:
    sim = Simulation(xml)
    m = sim.model
    if not noslip:
        m.opt.noslip_iterations = 0
    b = m.body("ball").id
    if damping:
        da = m.jnt_dofadr[m.body_jntadr[b]]
        m.dof_damping[da:da + 6] = damping
    qa = m.jnt_qposadr[m.body_jntadr[b]]
    seat = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball_seat")
    if know_position:                       # 球真的在别处，手也知道：改初始位置后再规划
        m.qpos0[qa:qa + 2] = NOMINAL + dxy
        if seat >= 0:
            m.body_pos[seat][:2] = NOMINAL + dxy
        sim.reset()
    fam = LevelPinch(m, {a.joint_name: a.qpos_adr for a in sim.actuators}) if family else None
    seq = PinchGraspSequence(fam)
    seq.start(sim, "ball", PinchParams(), t=0.0)
    if not know_position:                   # 按名义位置规划之后，球（和座）实际偏了 dxy
        sim.data.qpos[qa:qa + 2] += dxy
        if seat >= 0:
            m.body_pos[seat][:2] += dxy
        mujoco.mj_forward(m, sim.data)
        sim.grasp.reset()
    p0 = sim.data.xpos[b][:2].copy()
    push = 0.0
    while not seq.finished and sim.time < 40.0:
        if sim.control_due:
            seq.update(sim, sim.time)
            if seq.state == "CLOSE":
                push = max(push, float(np.linalg.norm(sim.data.xpos[b][:2] - p0)))
        sim.step_once()
    ok = seq.lift_end_state == "抓牢"
    return ok, f"{dxy}: {seq.lift_end_state or seq.failed}，闭合时推开 {push * 1000:.0f} mm"


def run(title: str, xml, **kw) -> None:
    res = [trial(xml, np.array(d), **kw) for d in OFFSETS]
    print(f"{title}: {sum(ok for ok, _ in res)}/5")
    for _, line in res:
        print("    " + line)


def main() -> None:
    xml = SCENE_DIR / "hand_table.xml"
    run("参考 · 无定位误差（手按球的真实位置规划）", xml, know_position=True)
    run("参考 · 关掉 noslip（软摩擦，等于附录 A 实验 8 的条件）", xml, noslip=False)
    run("参考 · 定位误差 ±1 cm", xml)
    run("第 1 轮 · 水平对齐姿态族（IK）", xml, family=True)
    with tempfile.TemporaryDirectory() as t:
        seat = variant_scene(Path(t), "g1_seat", SEAT)
        sim = Simulation(seat)
        seq = PinchGraspSequence()
        seq.start(sim, "ball", PinchParams())
        a, b = seq._tips_at(seq.plan["s_c"])
        w = seq.plan["wrist"]
        print(f"（座顶 z = {0.10 + H:.4f}；接触时指尖底部 z：食指 {a[2] + w[2] - 0.0077:.4f}、拇指 {b[2] + w[2] - 0.0085:.4f}）")
        run("第 2 轮 · 球放在座里", seat)
        cyl = variant_scene(Path(t), "g1_cylinder", cylinder=True)
        run("降级方案 · 直立圆柱（定位误差 ±1 cm）", cyl)
    run("第 3 轮 · 自由关节阻尼 0.02", xml, damping=0.02)


if __name__ == "__main__":
    main()
