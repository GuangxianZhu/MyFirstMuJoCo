"""入口：python main.py [--model 路径/to/model.xml]"""
import argparse
import sys
from pathlib import Path

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from app.ui.main_window import MainWindow

DEFAULT_MODEL = Path(__file__).parent / "app" / "scenes" / "hand" / "hand.xml"


def apply_dark_theme(app: QApplication) -> None:
    app.setStyle("Fusion")
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(37, 40, 46))
    p.setColor(QPalette.ColorRole.WindowText, QColor(225, 228, 233))
    p.setColor(QPalette.ColorRole.Base, QColor(27, 29, 34))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(37, 40, 46))
    p.setColor(QPalette.ColorRole.Text, QColor(225, 228, 233))
    p.setColor(QPalette.ColorRole.Button, QColor(52, 56, 64))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(225, 228, 233))
    p.setColor(QPalette.ColorRole.Highlight, QColor(61, 142, 224))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    p.setColor(QPalette.ColorRole.ToolTipBase, QColor(52, 56, 64))
    p.setColor(QPalette.ColorRole.ToolTipText, QColor(225, 228, 233))
    app.setPalette(p)


def main() -> int:
    ap = argparse.ArgumentParser(description="MuJoCo 机械臂演示")
    ap.add_argument("--model", default=str(DEFAULT_MODEL), help="MuJoCo XML 模型路径")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    apply_dark_theme(app)
    win = MainWindow(args.model)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
