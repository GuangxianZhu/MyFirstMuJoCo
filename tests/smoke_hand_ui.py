"""灵巧手界面冒烟测试（无需显示器）：

    QT_QPA_PLATFORM=offscreen python tests/smoke_hand_ui.py [输出截图.png]

Linux 无显卡环境下渲染需要: MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
from PySide6.QtWidgets import QApplication, QPushButton  # noqa: E402

from main import DEFAULT_MODEL, apply_dark_theme  # noqa: E402
from app.ui.main_window import MainWindow  # noqa: E402


def run(win, app, seconds):
    for _ in range(int(seconds * 60)):
        win._last_wall = time.perf_counter() - 1 / 60
        win.tick()
        app.processEvents()


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "smoke_hand.png"
    app = QApplication([])
    apply_dark_theme(app)
    win = MainWindow(DEFAULT_MODEL, start_timer=False)
    win.show()
    sim, hp, jp = win.sim, win.hand_panel, win.joint_panel
    assert hp is not None and len(jp._sliders) == 26

    # 1) 手势按钮：握拳，过渡期间滑块跟随，结束后到位
    buttons = {b.text(): b for b in hp.findChildren(QPushButton)}
    hp.transition.set_value(0.4)
    buttons["握拳"].click()
    run(win, app, 0.2)
    mid = jp._sliders[sim.joint_index("index_pip")].value() / 10.0
    assert 0.0 < mid < 100.0, mid                       # 度；过渡中，介于 0 和 100.3° 之间
    run(win, app, 1.2)
    pip_deg = jp._sliders[sim.joint_index("index_pip")].value() / 10.0
    assert abs(pip_deg - 100.3) < 0.2, pip_deg           # 1.75 rad
    q = sim.joint_angles()[sim.joint_index("index_pip")]
    assert abs(q - 1.75) < 0.08

    # 2) 协同滑块：整体屈伸 -> 0，张开 -> 1
    hp._syn["curl"].slider.setValue(0)
    hp._syn["spread"].slider.setValue(1000)
    run(win, app, 1.2)
    assert abs(sim.manual.targets[sim.joint_index("index_pip")]) < 1e-9
    assert sim.manual.targets[sim.joint_index("index_spread")] > 0.15

    # 3) 手腕滑块（毫米、度）
    wx = sim.joint_index("wrist_x")
    jp._sliders[wx].setValue(100)                        # 0.5 mm 一格 -> 50 mm
    assert abs(sim.manual.targets[wx] - 0.05) < 1e-9
    run(win, app, 1.0)
    assert abs(sim.data.body("palm").xpos[0] - 0.05) < 0.003

    # 4) 节奏动作：开始 -> 手指在动 -> 停止
    hp.pattern.setCurrentIndex(0)
    hp.start.setChecked(True)
    assert sim.rhythm_active
    k = sim.joint_index("middle_mcp")
    seen = []
    for _ in range(120):
        run(win, app, 1 / 60)
        seen.append(sim.joint_angles()[k])
    assert max(seen) - min(seen) > 0.3
    hp.freq.slider.setValue(500)                          # 实时改参数
    assert abs(sim.rhythm.freq - hp.freq.value()) < 1e-9
    run(win, app, 0.5)
    win.grab().save(str(out))
    hp.start.setChecked(False)
    assert not sim.rhythm_active
    run(win, app, 0.3)

    # 5) 重置
    win.status_panel.resetRequested.emit()
    run(win, app, 0.1)
    assert np.allclose(sim.manual.targets, 0.0)
    assert abs(jp._sliders[wx].value()) == 0
    win.status_panel.cameraResetRequested.emit()
    print("saved", out)
    win.close()


if __name__ == "__main__":
    main()
