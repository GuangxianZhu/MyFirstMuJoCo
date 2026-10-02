"""阶段 B 界面冒烟测试（无需显示器）：桌面场景、抓取页签、演示 3（轻拿轻放易碎物）、演示按钮的场景检查。

    QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless \\
        python tests/smoke_stage_b.py [截图目录]
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication  # noqa: E402

from main import DEFAULT_MODEL, apply_dark_theme  # noqa: E402
from app.ui.demo_panel import DemoPanel  # noqa: E402
from app.ui.main_window import MainWindow  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"


def run(win, app, seconds):
    for _ in range(int(seconds * 60)):
        win._last_wall = time.perf_counter() - 1 / 60
        win.tick()
        app.processEvents()


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT
    out.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    apply_dark_theme(app)

    # 1) 只有手的场景：没有抓取页签；需要物体的演示按钮是灰的
    win = MainWindow(DEFAULT_MODEL, start_timer=False)
    assert win.grasp_view is None
    assert "抓取" not in [win.tabs.tabText(i) for i in range(win.tabs.count())]
    from app.scenes.hand.demos import FragilePickPlaceDemo
    win.sim.extras.demos.append(FragilePickPlaceDemo)
    panel = DemoPanel(win.sim)
    panel.combo.setCurrentIndex(panel.combo.count() - 1)
    assert not panel.start_btn.isEnabled() and "hand_table" in panel.desc.text()
    panel.start_btn.click()
    assert win.sim.demo is None
    win.sim.extras.demos.pop()
    win.close()

    # 2) 桌面场景：抓取页签
    win = MainWindow(TABLE, start_timer=False)
    win.resize(1560, 900)
    win.show()
    sim = win.sim
    assert win.grasp_view is not None and sim.grasp is not None
    tabs = [win.tabs.tabText(i) for i in range(win.tabs.count())]
    assert tabs[:2] == ["触觉", "抓取"], tabs
    win.tabs.setCurrentWidget(win.grasp_view)
    run(win, app, 0.5)
    assert win.grasp_view.state_text() == "未接触"
    win.grab().save(str(out / "stageB_table.png"))

    # 3) 演示 3：轻拿轻放易碎物（在抓取页签里看）
    dp = win.demo_panel
    keys = [f.key for f in dp._factories]
    dp.combo.setCurrentIndex(keys.index("fragile"))
    assert dp.start_btn.isEnabled()
    dp.start_btn.click()
    assert sim.demo is not None and sim.grasp.object_name == "fragile"
    assert abs(win.renderer.camera.distance - FragilePickPlaceDemo.camera["distance"]) < 1e-9
    win.tabs.setCurrentWidget(win.grasp_view)
    seen = set()
    shots = set()
    for _ in range(60 * 60):
        run(win, app, 1 / 60)
        st = win.grasp_view.state_text()
        seen.add(st)
        if st in ("抓牢", "碎了") and st not in shots:
            shots.add(st)
            win.grab().save(str(out / f"stageB_grasp_{'held' if st == '抓牢' else 'broken'}.png"))
        if sim.demo is None:
            break
    assert sim.demo is None, "演示 3 没有在 60 秒内结束"
    assert seen & {"抓牢", "碎了"}, seen
    win.tabs.setCurrentWidget(dp)
    run(win, app, 0.3)
    assert "轻拿轻放" in dp.summary.text() and "用力抓" in dp.summary.text(), dp.summary.text()
    win.grab().save(str(out / "stageB_demo3_done.png"))
    assert sim.drive.k_scale == 1.0 and sim.grasp.object_name == "cube"

    # 4) 重置按钮
    win.tabs.setCurrentWidget(win.grasp_view)
    win.grasp_view.reset_btn.click()
    run(win, app, 0.2)
    assert win.grasp_view.state_text() == "未接触" and sim.time < 0.5
    print("ok", sorted(seen))
    win.close()


if __name__ == "__main__":
    main()
