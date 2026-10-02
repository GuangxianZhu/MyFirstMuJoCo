"""跑演示 4「位置控制 vs 阻抗控制抓球」并把 6 次试验的指标打印成表（人工看趋势用，结果贴进 docs/NEXT_STAGE_B.md 附录 A）。

    python tools/report_compare.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.core.sim import Simulation  # noqa: E402
from app.scenes.hand.demos import CompareGraspDemo  # noqa: E402

TABLE = ROOT / "app" / "scenes" / "hand" / "hand_table.xml"


def main() -> None:
    sim = Simulation(TABLE)
    demo = CompareGraspDemo()
    sim.start_demo(demo)
    while sim.demo is not None and sim.time < 120.0:
        sim.step_once()
    print(f"仿真用时 {sim.time:.1f} s，完成={demo.finished}\n")
    print("| 控制 | 定位误差 | 冲击峰值 (N) | 稳态握力 (N) | 推开 (mm) | 抬起时判定 | 抬起 |")
    print("|---|---|---|---|---|---|---|")
    for r in demo.results:
        print(f"| {r['group']} | {r['error'] * 1000:.0f} mm | {r['impact_peak']:.2f} | {r['steady_force']:.2f} | "
              f"{r['push_away'] * 1000:.1f} | {r['final_state']} | {'是' if r['lifted'] else '否'} |")
    print()
    print(demo.conclusion())


if __name__ == "__main__":
    main()
