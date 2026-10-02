"""界面冒烟测试（无需显示器）：

    QT_QPA_PLATFORM=offscreen python tests/smoke_ui.py [输出截图.png]

Linux 无显卡环境下渲染需要: MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless
"""
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication  # noqa: E402

from main import apply_dark_theme  # noqa: E402
from app.ui.main_window import MainWindow  # noqa: E402

ARM_MODEL = ROOT / "app" / "scenes" / "arm" / "arm.xml"


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "smoke.png"
    app = QApplication([])
    apply_dark_theme(app)
    win = MainWindow(ARM_MODEL, start_timer=False)
    win.show()

    # 拖动第 1、2 个滑块，推进约 1.5 秒仿真
    p = win.joint_panel
    p._sliders[0].setValue(450)    # 45.0°
    p._sliders[1].setValue(300)    # 30.0°
    p._sliders[2].setValue(-600)   # -60.0°
    for _ in range(90):
        win._last_wall = time.perf_counter() - 1 / 60
        win.tick()
        app.processEvents()

    q = win.sim.joint_angles()
    print("joint angles (deg):", [round(math.degrees(x), 1) for x in q])
    assert abs(math.degrees(q[0]) - 45.0) < 3.0, "yaw 没有跟上滑块目标"

    # 相机交互：旋转 + 平移 + 缩放
    cam = win.renderer.camera
    d0, az0, look0 = cam.distance, cam.azimuth, cam.lookat.copy()
    win.viewport.rotateRequested.emit(40.0, 10.0)
    win.viewport.panRequested.emit(30.0, -20.0)
    win.viewport.zoomRequested.emit(2.0)
    assert cam.distance < d0 and cam.azimuth != az0 and (cam.lookat != look0).any()
    win.tick()

    # 暂停与重置
    win.status_panel._pause.setChecked(True)
    t = win.sim.time
    win.tick()
    assert win.sim.time == t
    win.status_panel.resetRequested.emit()
    assert win.sim.time == 0.0

    win.status_panel._pause.setChecked(False)
    p._sliders[0].setValue(-500)
    for _ in range(60):
        win._last_wall = time.perf_counter() - 1 / 60
        win.tick()

    # --- 物理参数面板 ---
    pp = win.physics_panel
    model = win.sim.model
    # 重力预设：月球
    moon = next(i for i in range(pp.gravity.preset.count()) if "月球" in pp.gravity.preset.itemText(i))
    pp.gravity.preset.setCurrentIndex(moon)
    pp.gravity._on_preset(moon)
    assert abs(win.sim.physics.gravity - 1.62) < 1e-9
    # 重力滑块手动改 -> 预设回到"自定义"或匹配项
    pp.gravity.value.spin.setValue(5.0)
    assert abs(win.sim.physics.gravity - 5.0) < 1e-9 and pp.gravity.preset.currentIndex() == 0

    sec = {s._param.key: s for s in pp.sections}
    # 倍率 -> 数值与单独调节行同步
    sec["kp"].scale.spin.setValue(2.0)
    assert abs(model.actuator_gainprm[0, 0] - 120.0) < 1e-6
    assert abs(sec["kp"]._items[0].value() - 120.0) < 0.06
    # 展开单独调节，只改一个关节
    sec["damping"].toggle.setChecked(True)
    sec["damping"]._items[1].spin.setValue(1.0)
    assert abs(model.dof_damping[1] - 1.0) < 1e-9 and abs(model.dof_damping[0] - 0.4) < 1e-9
    sec["mass"].toggle.setChecked(True)
    sec["mass"].scale.spin.setValue(2.0)
    for _ in range(30):
        win._last_wall = time.perf_counter() - 1 / 60
        win.tick()
    app.processEvents()
    win.grab().save(str(out))
    print("saved", out)

    # 恢复默认
    pp.reset_defaults()
    assert abs(win.sim.physics.gravity - 9.81) < 1e-9
    assert abs(model.actuator_gainprm[0, 0] - 60.0) < 1e-9
    assert abs(sec["kp"].scale.value() - 1.0) < 1e-9
    win.close()


if __name__ == "__main__":
    main()
