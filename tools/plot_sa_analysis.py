#!/usr/bin/env python3
"""文件职责：根据 sa_trace.json 生成 SA 诊断图，写入 output/analysis/。"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any


# 将 next_cost 过高的点视为“罚分悬崖”，避免线性纵轴被撑爆。
HIGH_COST_THRESHOLD = 100.0
# 滑动窗口接受率窗口长度（轮）。
ACCEPT_WINDOW = 20


# 读取 sa_trace.json；缺少 iterations 时返回空列表。
def load_iterations(trace_path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    data = json.loads(trace_path.read_text(encoding="utf-8"))
    iterations = list(data.get("iterations") or [])
    return data, iterations


# 绘制 best/current 曲线，并单独标出可行邻域内的 next_cost。
def plot_cost_curves(iterations: list[dict[str, Any]], out_path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [int(item["index"]) for item in iterations]
    best = [float(item["best_cost"]) for item in iterations]
    current = [float(item["current_cost"]) for item in iterations]
    next_costs = [float(item["next_cost"]) for item in iterations]
    feasible_x = [x for x, cost in zip(xs, next_costs) if cost < HIGH_COST_THRESHOLD]
    feasible_y = [cost for cost in next_costs if cost < HIGH_COST_THRESHOLD]

    fig, ax = plt.subplots(figsize=(10.0, 5.5), dpi=140)
    ax.plot(xs, best, color="#1565C0", linewidth=2.0, label="best_cost")
    ax.plot(xs, current, color="#EF6C00", linewidth=1.4, alpha=0.9, label="current_cost")
    if feasible_x:
        ax.scatter(
            feasible_x,
            feasible_y,
            s=12,
            color="#43A047",
            alpha=0.55,
            label=f"next_cost < {HIGH_COST_THRESHOLD:g}",
            zorder=3,
        )
    ax.set_xlabel("SA iteration")
    ax.set_ylabel("cost")
    ax.set_title(title)
    ax.grid(True, color="#EEEEEE", linewidth=0.8)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


# 绘制温度曲线与滑动窗口接受率。
def plot_temperature_accept(iterations: list[dict[str, Any]], out_path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [int(item["index"]) for item in iterations]
    temps = [float(item["temperature"]) for item in iterations]
    accepts = [1.0 if item.get("accept") else 0.0 for item in iterations]
    window = min(ACCEPT_WINDOW, max(1, len(accepts)))
    rolling: list[float] = []
    for index in range(len(accepts)):
        left = max(0, index + 1 - window)
        segment = accepts[left : index + 1]
        rolling.append(sum(segment) / len(segment))

    fig, ax_temp = plt.subplots(figsize=(10.0, 5.5), dpi=140)
    ax_temp.plot(xs, temps, color="#6A1B9A", linewidth=1.6, label="temperature")
    ax_temp.set_xlabel("SA iteration")
    ax_temp.set_ylabel("temperature", color="#6A1B9A")
    ax_temp.tick_params(axis="y", labelcolor="#6A1B9A")
    ax_temp.set_yscale("log")
    ax_temp.grid(True, color="#EEEEEE", linewidth=0.8)

    ax_acc = ax_temp.twinx()
    ax_acc.plot(
        xs,
        rolling,
        color="#00838F",
        linewidth=1.5,
        label=f"accept rate (window={window})",
    )
    ax_acc.set_ylabel("accept rate", color="#00838F")
    ax_acc.tick_params(axis="y", labelcolor="#00838F")
    ax_acc.set_ylim(-0.05, 1.05)

    lines_a, labels_a = ax_temp.get_legend_handles_labels()
    lines_b, labels_b = ax_acc.get_legend_handles_labels()
    ax_temp.legend(lines_a + lines_b, labels_a + labels_b, loc="best", fontsize=8)
    ax_temp.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


# 按扰动类型统计次数、接受率与高代价（悬崖）比例。
def plot_move_stats(iterations: list[dict[str, Any]], out_path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    counts: dict[str, int] = defaultdict(int)
    accepts: dict[str, int] = defaultdict(int)
    cliffs: dict[str, int] = defaultdict(int)
    for item in iterations:
        move = str(item.get("move") or "none")
        counts[move] += 1
        if item.get("accept"):
            accepts[move] += 1
        if float(item.get("next_cost", 0.0)) >= HIGH_COST_THRESHOLD:
            cliffs[move] += 1

    moves = sorted(counts.keys(), key=lambda name: (-counts[name], name))
    total = [counts[move] for move in moves]
    accept_rate = [accepts[move] / counts[move] for move in moves]
    cliff_rate = [cliffs[move] / counts[move] for move in moves]

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.0), dpi=140)
    ax_count, ax_rate = axes
    positions = list(range(len(moves)))
    ax_count.bar(positions, total, color="#546E7A")
    ax_count.set_xticks(positions)
    ax_count.set_xticklabels(moves, rotation=35, ha="right", fontsize=8)
    ax_count.set_ylabel("count")
    ax_count.set_title("move frequency")
    ax_count.grid(True, axis="y", color="#EEEEEE", linewidth=0.8)

    width = 0.38
    ax_rate.bar([p - width / 2 for p in positions], accept_rate, width=width, color="#2E7D32", label="accept rate")
    ax_rate.bar([p + width / 2 for p in positions], cliff_rate, width=width, color="#C62828", label=f"next≥{HIGH_COST_THRESHOLD:g}")
    ax_rate.set_xticks(positions)
    ax_rate.set_xticklabels(moves, rotation=35, ha="right", fontsize=8)
    ax_rate.set_ylim(0.0, 1.05)
    ax_rate.set_ylabel("ratio")
    ax_rate.set_title("accept vs cliff ratio")
    ax_rate.legend(loc="best", fontsize=8)
    ax_rate.grid(True, axis="y", color="#EEEEEE", linewidth=0.8)

    fig.suptitle(title, fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


# 根据 sa_trace 生成 analysis 目录下的全部诊断图；无 SA 记录时跳过。
def generate_sa_analysis_plots(trace_path: Path, analysis_dir: Path, render_name: str = "") -> list[Path]:
    data, iterations = load_iterations(trace_path)
    if not iterations:
        return []

    analysis_dir.mkdir(parents=True, exist_ok=True)
    prefix = render_name.strip() or analysis_dir.parent.name or "sa"
    early = bool(data.get("terminated_early"))
    recorded = int(data.get("recorded") or len(iterations))
    total = int(data.get("sa_iterations") or recorded)
    subtitle = f"{prefix}: {recorded}/{total} iters"
    if early:
        subtitle += " (early-stop)"

    outputs = [
        (
            analysis_dir / "sa_cost_curves.png",
            lambda path: plot_cost_curves(iterations, path, f"SA cost curves — {subtitle}"),
        ),
        (
            analysis_dir / "sa_temperature_accept.png",
            lambda path: plot_temperature_accept(iterations, path, f"SA temperature & accept — {subtitle}"),
        ),
        (
            analysis_dir / "sa_move_stats.png",
            lambda path: plot_move_stats(iterations, path, f"SA move stats — {subtitle}"),
        ),
    ]
    written: list[Path] = []
    for path, drawer in outputs:
        drawer(path)
        written.append(path)
    return written


# 解析命令行并写出分析图。
def main() -> int:
    parser = argparse.ArgumentParser(description="Plot SA analysis figures from sa_trace.json")
    parser.add_argument("--trace", required=True, type=Path, help="Path to sa_trace.json")
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Analysis output directory (typically <run-output>/analysis)",
    )
    parser.add_argument("--name", default="", help="Optional title prefix")
    args = parser.parse_args()

    if not args.trace.is_file():
        raise SystemExit(f"sa_trace not found: {args.trace}")

    written = generate_sa_analysis_plots(args.trace, args.output_dir, args.name)
    if not written:
        print("no SA iterations; skip analysis plots")
        return 0
    for path in written:
        print(path.as_posix())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
