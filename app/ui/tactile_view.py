"""触觉示意图：每根手指 3 节（指尖在上）+ 掌心，颜色和数字表示该段法向力。

分组方式从触觉段名字自动得到："index_dist" -> 组 "index"、部位 "dist"；没有下划线的名字（"palm"）单独成组。
"""
from __future__ import annotations

import numpy as np
from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

# 部位自上而下的显示顺序（指尖在上）
PART_ORDER = ["dist", "mid", "prox"]
MIN_SCALE_N = 2.0


def split_segment(name: str) -> tuple[str, str]:
    """'index_dist' -> ('index', 'dist')；'palm' -> ('palm', '')。"""
    if "_" in name:
        g, part = name.rsplit("_", 1)
        return g, part
    return name, ""


def group_order(names: list[str]) -> list[str]:
    out: list[str] = []
    for n in names:
        g = split_segment(n)[0]
        if g not in out:
            out.append(g)
    return out


def force_color(v: float, scale: float) -> QColor:
    """0 -> 深灰，一半 -> 黄，满量程 -> 红。"""
    t = float(np.clip(v / scale, 0.0, 1.0))
    if v < 0.02:
        return QColor(46, 51, 59)
    stops = [(0.0, (70, 90, 120)), (0.5, (240, 200, 60)), (1.0, (232, 72, 60))]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            return QColor(*(int(a + (b - a) * u) for a, b in zip(c0, c1)))
    return QColor(*stops[-1][1])


class TactileView(QWidget):
    def __init__(self, names: list[str], labels: dict[str, str] | None = None, parent: QWidget | None = None):
        super().__init__(parent)
        self.names = list(names)
        self.labels = labels or {}
        self.groups = group_order(self.names)
        self.values = np.zeros(len(self.names))
        self.counts = np.zeros(len(self.names), dtype=int)
        self.scale = MIN_SCALE_N
        self.setMinimumHeight(200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    # ------------------------------------------------------------------ 数据
    def set_reading(self, reading) -> None:
        if reading is None:
            return
        self.values = np.asarray(reading.normal, dtype=float)
        self.counts = np.asarray(reading.count)
        # 量程：自动放大，缓慢回落
        self.scale = max(MIN_SCALE_N, self.scale * 0.995, float(self.values.max(initial=0.0)))
        self.update()

    def _name(self, seg: str) -> str:
        return self.labels.get(f"seg:{seg}", seg)

    def _group_name(self, g: str) -> str:
        return self.labels.get(f"group:{g}", self.labels.get(f"seg:{g}", g))

    # ------------------------------------------------------------------ 绘制
    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        p.fillRect(self.rect(), QColor(27, 29, 34))
        W, H = self.width(), self.height()
        pad, head, foot = 12, 22, 34
        total = float(self.values.sum())

        small = QFont(self.font())
        small.setPointSize(9)
        p.setFont(small)
        p.setPen(QColor(200, 204, 210))
        p.drawText(QRectF(pad, 2, W - 2 * pad, head), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"总法向力 {total:.2f} N        颜色满量程 {self.scale:.1f} N")

        # 列：一般的组占 1 份宽度，只有一个段的组（掌心）占 2 份
        widths = []
        for g in self.groups:
            n_seg = sum(1 for s in self.names if split_segment(s)[0] == g)
            widths.append(1.0 if n_seg > 1 else 2.0)
        gap = 8
        unit = (W - 2 * pad - gap * (len(self.groups) - 1)) / sum(widths)
        x = float(pad)
        top, bottom = head + 2, H - foot

        for g, w in zip(self.groups, widths):
            col_w = unit * w
            idx = [i for i, s in enumerate(self.names) if split_segment(s)[0] == g]
            parts = {split_segment(self.names[i])[1]: i for i in idx}
            if len(idx) == 1:
                order = [idx[0]]
            else:
                order = [parts[k] for k in PART_ORDER if k in parts] + [i for i in idx if split_segment(self.names[i])[1] not in PART_ORDER]
            h_each = (bottom - top - 4 * (len(order) - 1)) / len(order)
            for r, i in enumerate(order):
                rect = QRectF(x, top + r * (h_each + 4), col_w, h_each)
                v = float(self.values[i])
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(force_color(v, self.scale))
                p.drawRoundedRect(rect, 6, 6)
                p.setPen(QColor(20, 22, 26) if v > 0.5 * self.scale else QColor(215, 218, 224))
                txt = f"{v:.2f} N" if v >= 0.005 else "–"
                p.drawText(rect, Qt.AlignmentFlag.AlignCenter, txt)
                if len(idx) > 1:
                    seg_part = split_segment(self.names[i])[1]
                    p.setPen(QColor(150, 156, 166))
                    tiny = QFont(small)
                    tiny.setPointSize(8)
                    p.setFont(tiny)
                    short = self.labels.get(f"part:{seg_part}", seg_part)
                    p.drawText(rect.adjusted(4, 2, 0, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop, short)
                    p.setFont(small)
            gsum = float(self.values[idx].sum())
            p.setPen(QColor(210, 214, 220))
            p.drawText(QRectF(x - 4, bottom + 2, col_w + 8, foot - 2), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                       f"{self._group_name(g)}\n{gsum:.2f} N")
            x += col_w + gap
        p.end()
