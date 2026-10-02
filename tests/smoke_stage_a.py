"""阶段 A 界面冒烟测试（无需显示器）：触觉页签、曲线、阻抗面板、两个演示。

    QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless \
        python tests/smoke_stage_a.py [截图目录]
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication  # noqa: E402

from main import DEFAULT_MODEL, apply_dark_theme  # noqa: E402
from app.ui.main_window import MainWindow  # noqa: E402


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
    win = MainWindow(DEFAULT_MODEL, start_timer=False)
    win.resize(1560, 900)
    win.show()
    sim = win.sim
    assert win.tactile_view and win.demo_panel and win.impedance_panel

    # 1) 阻抗面板 -> 控制器
    ip = win.impedance_panel
    ip.preset.setCurrentIndex(1)           # 柔顺
    ip._on_preset()
    assert abs(sim.drive.k_scale - 0.5) < 1e-9 and abs(sim.drive.tau_limit - 0.30) < 1e-9
    ip.k.slider.setValue(500)              # 手动拖 K -> 预设变"自定义"
    assert ip.preset.currentIndex() == 0
    ip.preset.setCurrentIndex(2)
    ip._on_preset()
    assert sim.drive.k_scale == 1.0

    # 2) 显示选项
    import mujoco
    win.cb_points.setChecked(True)
    win.cb_forces.setChecked(True)
    assert win.renderer.option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE]

    # 3) 演示 1：按压桌面
    win.tabs.setCurrentWidget(win.tactile_view)
    dp = win.demo_panel
    dp.combo.setCurrentIndex(0)
    dp.start_btn.click()
    assert sim.demo is not None
    shot_taken = False
    for _ in range(60 * 20):
        run(win, app, 1 / 60)
        if sim.demo is None:
            break
        if not shot_taken and sim.demo.clock.name == "flat" and sim.demo.clock.elapsed(sim.demo_time) > 2.0:
            win.grab().save(str(out / "stageA_tactile_flat.png"))
            shot_taken = True
        if not shot_taken and sim.demo.clock.name == "tip" and sim.demo.clock.elapsed(sim.demo_time) > 2.5:
            win.grab().save(str(out / "stageA_tactile_tip.png"))
    assert sim.demo is None, "演示 1 没有在 20 秒内结束"
    win.tabs.setCurrentWidget(dp)
    run(win, app, 0.3)
    assert "整个过程" in dp.summary.text(), dp.summary.text()
    win.grab().save(str(out / "stageA_demo1_done.png"))

    # 4) 演示 2：阻抗对比
    dp.combo.setCurrentIndex(1)
    dp.start_btn.click()
    win.tabs.setCurrentWidget(win.scope_panel)
    seen = set()
    for _ in range(60 * 20):
        run(win, app, 1 / 60)
        if sim.demo is None:
            break
        seen.add(round(sim.drive.k_scale, 2))
    assert sim.demo is None
    assert seen == {0.5, 1.0, 5.0}, seen         # 三组参数都用过
    assert sim.drive.k_scale == 1.0              # 结束后恢复
    win.tabs.setCurrentWidget(dp)
    run(win, app, 0.3)
    assert "刚硬" in dp.summary.text()
    win.grab().save(str(out / "stageA_demo2_done.png"))

    # 5) 曲线页签
    win.tabs.setCurrentWidget(win.scope_panel)
    win.joint_panel_idx = None
    run(win, app, 0.5)
    win.grab().save(str(out / "stageA_scope.png"))
    print("ok")
    win.close()


if __name__ == "__main__":
    main()
