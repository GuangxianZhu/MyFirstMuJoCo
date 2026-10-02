"""手模型 hand.xml 的附加配置：弯曲度曲线、协同、预设手势、初始相机、界面中文名。

只使用关节名，不依赖 Qt / MuJoCo。手势里没有出现的关节（手腕）保持不变。
"""
from __future__ import annotations

from app.core.scene_extras import SceneExtras, SynergySpec

FINGERS = ["index", "middle", "ring", "little"]

# 弯曲度 c(0=伸直, 1=握紧) 对应的各关节角（弧度），线性映射
FINGER_CURL = {"mcp": 1.35, "pip": 1.75, "dip": 1.15}
THUMB_CURL = {"cmc": 0.40, "mcp": 1.00, "ip": 1.10}

# 手指张开(spread=1)时各指侧摆角：食指朝拇指一侧，小指朝外
SPREAD = {"index": 0.18, "middle": 0.04, "ring": -0.09, "little": -0.24}

# 拇指对掌(opposition=1)：摆向手指 + 朝掌心弯
THUMB_OPPOSE = {"rot": 0.95, "cmc": 0.90}

# 拇指收到掌心前的姿态（握拳 / 数字 1~4 用）：横过手掌、指尖向掌心侧弯（用逆运动学求得）
THUMB_TUCK = {"thumb_rot": 1.18, "thumb_cmc": 0.0, "thumb_mcp": 0.97, "thumb_ip": 1.06}
THUMB_OPEN = {"thumb_rot": 0.0, "thumb_cmc": 0.0, "thumb_mcp": 0.0, "thumb_ip": 0.0}

# "OK"：拇指尖与食指尖相碰（用数值逆运动学求得，tests/test_hand.py 里验证指尖距离）
OK_THUMB = {"thumb_rot": 1.026, "thumb_cmc": 0.628, "thumb_mcp": 0.312, "thumb_ip": 0.392}
OK_INDEX = {"index_mcp": 0.890, "index_pip": 0.908, "index_dip": 0.454}


def curl_pose(finger: str, curl: float) -> dict[str, float]:
    """某根手指弯曲度 curl(0~1) 对应的弯曲关节角（不含侧摆/对掌）。"""
    if finger == "thumb":
        return {f"thumb_{k}": v * curl for k, v in THUMB_CURL.items()}
    return {f"{finger}_{k}": v * curl for k, v in FINGER_CURL.items()}


def _finger_pose(finger: str, curl: float, spread: float = 0.0) -> dict[str, float]:
    pose = curl_pose(finger, curl)
    pose[f"{finger}_spread"] = SPREAD[finger] * spread
    return pose


def synergy_pose(values: dict[str, float]) -> dict[str, float]:
    """协同取值 -> 全部手指关节目标。"""
    curl = values.get("curl", 0.0)
    spread = values.get("spread", 0.0)
    oppose = values.get("oppose", 0.0)
    pose: dict[str, float] = {}
    for f in FINGERS:
        pose.update(_finger_pose(f, curl, spread))
    pose.update(
        {
            "thumb_rot": THUMB_OPPOSE["rot"] * oppose,
            "thumb_cmc": THUMB_OPPOSE["cmc"] * oppose + THUMB_CURL["cmc"] * curl,
            "thumb_mcp": THUMB_CURL["mcp"] * curl,
            "thumb_ip": THUMB_CURL["ip"] * curl,
        }
    )
    return pose


def _numbers_gesture(extended: list[str], thumb_out: bool = False, spread: float = 0.0) -> dict[str, float]:
    pose: dict[str, float] = {}
    for f in FINGERS:
        pose.update(_finger_pose(f, 0.0 if f in extended else 1.0, spread if f in extended else 0.0))
    pose.update(THUMB_OPEN if thumb_out else THUMB_TUCK)
    return pose


def _build_gestures() -> dict[str, dict[str, float]]:
    ok = _numbers_gesture(["middle", "ring", "little"], spread=0.5)
    for f in ("middle", "ring", "little"):
        ok.update(curl_pose(f, 0.08))
    ok.update(OK_INDEX)
    ok["index_spread"] = 0.0
    ok.update(OK_THUMB)

    return {
        "张开": synergy_pose({"curl": 0.0, "spread": 0.6, "oppose": 0.0}),
        "握拳": {**synergy_pose({"curl": 1.0, "spread": 0.0, "oppose": 0.0}), **THUMB_TUCK},
        "OK": ok,
        "1": _numbers_gesture(["index"]),
        "2": _numbers_gesture(["index", "middle"], spread=0.5),
        "3": _numbers_gesture(["index", "middle", "ring"], spread=0.5),
        "4": _numbers_gesture(FINGERS, spread=0.5),
        "5": _numbers_gesture(FINGERS, thumb_out=True, spread=0.5),
    }


# 触觉段（刚体名）：掌心 + 每根手指的 3 节
TACTILE_SEGMENTS = ["palm"] + [
    f"{f}_{part}" for f in ["thumb", "index", "middle", "ring", "little"] for part in ("prox", "mid", "dist")
]

# 关节阻抗默认值（力矩型手指执行器）：tau = K(q_d - q) - D*qd，限幅 tau_limit
IMPEDANCE = {"k": 1.5, "d": 0.08, "tau_limit": 0.60}

LABELS = {
    # 组名
    "group:wrist": "手腕", "group:thumb": "拇指", "group:index": "食指",
    "group:middle": "中指", "group:ring": "无名指", "group:little": "小指",
    # 关节名（组内）
    "joint:x": "前后 X", "joint:y": "左右 Y", "joint:z": "高度 Z",
    "joint:yaw": "偏航", "joint:pitch": "俯仰", "joint:roll": "滚转",
    "joint:spread": "侧摆", "joint:mcp": "根部", "joint:pip": "中节", "joint:dip": "末节",
    "joint:rot": "对掌摆动", "joint:cmc": "腕掌", "joint:ip": "末节",
    # 触觉段
    "part:prox": "近节", "part:mid": "中节", "part:dist": "指尖",
    "seg:palm": "掌心",
    **{f"seg:{f}_{p}": f"{cn}{pn}" for f, cn in
       [("thumb", "拇指"), ("index", "食指"), ("middle", "中指"), ("ring", "无名指"), ("little", "小指")]
       for p, pn in [("prox", "近节"), ("mid", "中节"), ("dist", "指尖")]},
}


def _demos():
    # 延迟导入：demos.py 依赖 mujoco，且只在真正创建场景配置时才需要
    from app.scenes.hand.demos import ImpedanceCompareDemo, PressDemo
    return [PressDemo, ImpedanceCompareDemo]


def create() -> SceneExtras:
    return SceneExtras(
        fingers=["thumb"] + FINGERS,
        synergies=[
            SynergySpec("curl", "整体屈伸", 0.0, 1.0, 0.0),
            SynergySpec("spread", "手指张开", 0.0, 1.0, 0.0),
            SynergySpec("oppose", "拇指对掌", 0.0, 1.0, 0.0),
        ],
        synergy_pose=synergy_pose,
        gestures=_build_gestures(),
        curl_pose=curl_pose,
        camera={"azimuth": 125.0, "elevation": -28.0, "distance": 0.48, "lookat": (0.10, 0.0, 0.27)},
        labels=LABELS,
        impedance=IMPEDANCE,
        tactile_segments=TACTILE_SEGMENTS,
        demos=_demos(),
    )
