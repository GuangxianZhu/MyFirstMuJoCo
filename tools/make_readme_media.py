"""生成 README 用的截图和 GIF（直接录真实界面，无需显示器）。依赖 Pillow：pip install pillow

    QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless \
        python tools/make_readme_media.py [输出目录，默认 docs/media] [--only fragile,compare]

Windows 上有显示器时去掉那几个环境变量即可。每帧按固定的仿真时间推进（和电脑快慢无关），所以录出来是 1 倍速
（两个抓取演示较长，按 2 倍 / 4 倍速录，README 里注明）。--only 只录指定的几段：hero / press / impedance / scope / fragile / compare。
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PIL import Image  # noqa: E402
from PySide6.QtGui import QImage  # noqa: E402
from PySide6.QtWidgets import QApplication, QPushButton  # noqa: E402

import app.ui.main_window as mw  # noqa: E402
from main import DEFAULT_MODEL, apply_dark_theme  # noqa: E402

TABLE_MODEL = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"
SEGMENTS = ["hero", "press", "impedance", "scope", "fragile", "compare"]

FPS = 15
GIF_WIDTH = 1000          # 整窗 GIF 的宽度（像素）；只录中间一列时用 CENTER_WIDTH
CENTER_WIDTH = 820
COLORS = 128


class Recorder:
    def __init__(self, win, app):
        self.win, self.app = win, app
        self.frames: list[Image.Image] = []
        self.recording = False
        self.target = win          # 录哪个控件：整窗，或只录中间那一列（win.splitter）
        self.speed = 1.0           # 每帧推进 speed/FPS 秒仿真时间（>1 = 快放）

    def grab(self) -> Image.Image:
        img = self.target.grab().toImage().convertToFormat(QImage.Format.Format_RGB888)
        w, h = img.width(), img.height()
        return Image.frombytes("RGB", (w, h), bytes(img.constBits()), "raw", "RGB", img.bytesPerLine())

    def run(self, seconds: float, per_frame=None) -> None:
        for _ in range(round(seconds * FPS)):
            self.win._last_wall = time.perf_counter() - self.speed / FPS
            if per_frame:
                per_frame()
            self.win.tick()
            self.app.processEvents()
            if self.recording:
                self.frames.append(self.grab())

    def start(self) -> None:
        self.frames, self.recording = [], True

    def save_gif(self, path: Path) -> None:
        self.recording = False
        width = GIF_WIDTH if self.target is self.win else CENTER_WIDTH
        h = round(self.frames[0].height * width / self.frames[0].width)
        out = [f.resize((width, h), Image.LANCZOS).quantize(colors=COLORS, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE)
               for f in self.frames]
        out[0].save(path, save_all=True, append_images=out[1:], duration=round(1000 / FPS), loop=0, optimize=True, disposal=1)
        print(f"{path.name}: {len(out)} 帧, {path.stat().st_size / 1e6:.1f} MB")

    def still(self, path: Path, width: int | None = None) -> None:
        im = self.grab()
        width = width or (1500 if self.target is self.win else 1000)
        h = round(im.height * width / im.width)
        im.resize((width, h), Image.LANCZOS).quantize(colors=192, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE).save(path, optimize=True)
        print(f"{path.name}: {path.stat().st_size / 1e3:.0f} KB")


def new_window(app, model=DEFAULT_MODEL) -> mw.MainWindow:
    win = mw.MainWindow(model, start_timer=False)
    win.resize(1560, 900)
    win.show()
    return win


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=str(ROOT / "docs" / "media"))
    ap.add_argument("--only", default=",".join(SEGMENTS), help="逗号分隔：" + ",".join(SEGMENTS))
    args = ap.parse_args()
    out = Path(args.out)
    only = set(args.only.split(","))
    out.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    apply_dark_theme(app)
    mw.SCOPE_EVERY, mw.DEMO_EVERY = 1, 2      # 录制时提高曲线刷新率（每帧 = 1/15 秒仿真时间）
    for seg in SEGMENTS:
        if seg in only:
            globals()[f"record_{seg}"](app, out)


def record_hero(app, out: Path) -> None:
    # ---------------------------------------------------------------- 1) 主图：手势 + 节奏动作，镜头缓慢旋转
    win = new_window(app)
    rec = Recorder(win, app)
    btn = {b.text(): b for b in win.hand_panel.findChildren(QPushButton)}
    win.hand_panel.transition.set_value(0.5)
    import math
    cam = win.renderer.camera
    cam.distance, cam.elevation = 0.36, -24.0
    cam.lookat[:] = (0.11, 0.0, 0.27)
    clock = {"n": 0}

    def spin():   # 镜头在 ±20° 内缓慢摆动，露出手的立体感
        clock["n"] += 1
        cam.azimuth = 128.0 + 20.0 * math.sin(2 * math.pi * clock["n"] / (FPS * 8.0))
    rec.run(0.6, spin)
    rec.start()
    for g, hold in [("张开", 1.0), ("握拳", 1.1), ("张开", 0.9), ("OK", 1.3), ("2", 1.1), ("5", 1.1)]:
        btn[g].click()
        rec.run(hold, spin)
        if g == "OK":
            rec.still(out / "hero.png")
    win.hand_panel.pattern.setCurrentIndex(0)
    win.hand_panel.start.setChecked(True)
    rec.run(3.2, spin)
    rec.save_gif(out / "hero.gif")
    win.close()


def record_press(app, out: Path) -> None:
    # ---------------------------------------------------------------- 2) 触觉：按压桌面
    win = new_window(app)
    rec = Recorder(win, app)
    win.cb_points.setChecked(True)
    win.cb_forces.setChecked(True)
    win.tabs.setCurrentWidget(win.tactile_view)
    rec.target = win.splitter
    rec.run(0.3)
    win.demo_panel.combo.setCurrentIndex(0)
    rec.start()
    win.demo_panel.start_btn.click()
    win.renderer.set_camera({"azimuth": 128.0, "elevation": -20.0, "distance": 0.52, "lookat": (0.12, 0.0, 0.14)})
    shot = {"tip": False, "flat": False}

    def watch():
        d = win.sim.demo
        if d is None:
            return
        for ph in shot:
            if not shot[ph] and d.clock.name == ph and d.clock.elapsed(win.sim.demo_time) > (1.8 if ph == "tip" else 1.4):
                shot[ph] = True
                rec.still(out / f"tactile_{ph}.png")
    for _ in range(FPS * 20):
        rec.run(1 / FPS, watch)
        if win.sim.demo is None:
            break
    rec.run(1.2)
    rec.save_gif(out / "demo_press.gif")
    win.close()


def record_impedance(app, out: Path) -> None:
    # ---------------------------------------------------------------- 3) 阻抗对比
    win = new_window(app)
    rec = Recorder(win, app)
    win.cb_points.setChecked(True)
    win.cb_forces.setChecked(True)
    win.tabs.setCurrentWidget(win.demo_panel)
    win.demo_panel.combo.setCurrentIndex(1)
    rec.target = win.splitter
    rec.run(0.3)
    rec.start()
    win.demo_panel.start_btn.click()
    win.renderer.set_camera({"azimuth": 128.0, "elevation": -14.0, "distance": 0.50, "lookat": (0.12, 0.0, 0.24)})
    took = False

    def watch_imp():
        nonlocal took
        d = win.sim.demo
        if d is not None and not took and win.sim.demo_time > 2 * d.trial_len + 1.5:
            took = True
            rec.still(out / "demo_impedance.png")
    for _ in range(FPS * 20):
        rec.run(1 / FPS, watch_imp)
        if win.sim.demo is None:
            break
    rec.run(1.5)
    rec.save_gif(out / "demo_impedance.gif")
    win.close()


def record_scope(app, out: Path) -> None:
    # ---------------------------------------------------------------- 4) 实时曲线截图（节奏动作 + 触觉按压中的曲线）
    win = new_window(app)
    rec = Recorder(win, app)
    win.tabs.setCurrentWidget(win.scope_panel)
    rec.target = win.splitter
    win.hand_panel.pattern.setCurrentIndex(0)
    win.hand_panel.start.setChecked(True)
    rec.run(6.0)
    rec.still(out / "scope.png")
    win.close()


def _record_grasp_demo(app, out: Path, key: str, speed: float, name: str) -> None:
    """桌面场景里跑一个抓取演示：运行时看"抓取"页签，结束后切到"演示"页签停一会儿看总结和对比曲线。"""
    win = new_window(app, TABLE_MODEL)
    rec = Recorder(win, app)
    win.cb_points.setChecked(True)
    win.cb_forces.setChecked(True)
    win.tabs.setCurrentWidget(win.grasp_view)
    rec.target = win.splitter
    dp = win.demo_panel
    dp.combo.setCurrentIndex([f.key for f in dp._factories].index(key))
    rec.run(0.3)
    rec.start()
    rec.speed = speed
    dp.start_btn.click()
    for _ in range(FPS * 120):
        rec.run(1 / FPS)
        if win.sim.demo is None:
            break
    rec.speed = 1.0
    win.tabs.setCurrentWidget(dp)
    rec.run(0.3)
    rec.still(out / f"{name}.png")
    rec.run(2.2)
    rec.save_gif(out / f"{name}.gif")
    win.close()


def record_fragile(app, out: Path) -> None:
    # ---------------------------------------------------------------- 5) 演示 3：轻拿轻放易碎物（2 倍速）
    _record_grasp_demo(app, out, "fragile", 2.0, "demo_fragile")


def record_compare(app, out: Path) -> None:
    # ---------------------------------------------------------------- 6) 演示 4：位置控制 vs 阻抗控制抓球（4 倍速）
    _record_grasp_demo(app, out, "compare", 4.0, "demo_compare")


if __name__ == "__main__":
    main()
