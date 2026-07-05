#!/usr/bin/env python3
"""文件职责：将 B*-tree 调试 trace 渲染为树结构图和 packing contour 图。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any


LEFT_COLOR = "#1E88E5"
RIGHT_COLOR = "#FB8C00"
NODE_COLOR = "#FFFFFF"
PACK_COLOR = "#4CAF50"
OCCUPIED_COLOR = "#90CAF9"
SPACE_COLOR = "#F9A825"


# 输出目录名为空时提供稳定的默认图片名前缀。
def default_name(trace_path: Path) -> str:
    stem = trace_path.stem
    return stem if stem else "btree"


# 根据 root/left/right 递归计算逻辑二叉树坐标，不使用实际 placement 坐标。
def compute_tree_positions(trace: dict[str, Any]) -> dict[str, tuple[float, float]]:
    nodes = {node["id"]: node for node in trace.get("nodes", [])}
    root = trace.get("root")
    positions: dict[str, tuple[float, float]] = {}
    next_x = 0.0

    def visit(node_id: str, depth: int) -> float:
        nonlocal next_x
        node = nodes.get(node_id)
        if node is None:
            return next_x
        child_x: list[float] = []
        for child_key in ("left", "right"):
            child = node.get(child_key)
            if child:
                child_x.append(visit(child, depth + 1))
        if child_x:
            x = sum(child_x) / len(child_x)
        else:
            x = next_x
            next_x += 1.0
        positions[node_id] = (x, -1.45 * float(depth))
        return x

    if root:
        visit(root, 0)
    for node_id in nodes:
        if node_id not in positions:
            positions[node_id] = (next_x, 0.0)
            next_x += 1.0
    return positions


# 绘制 B*-tree 逻辑结构：left child 表示放到父节点右侧，right child 表示放到父节点上方。
def render_structure(trace: dict[str, Any], output_dir: Path, name: str, dpi: int) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    nodes = {node["id"]: node for node in trace.get("nodes", [])}
    positions = compute_tree_positions(trace)
    width = max(8.0, min(18.0, 2.0 * max(len(nodes), 1)))
    max_depth = max((-y for _x, y in positions.values()), default=0.0)
    height = max(5.0, min(12.0, 1.4 * (1 + max_depth)))
    fig, ax = plt.subplots(figsize=(width, height), dpi=dpi)
    ax.set_aspect("auto")
    ax.axis("off")

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        for child_key, color, label in (
            ("left", LEFT_COLOR, "left child = placed right"),
            ("right", RIGHT_COLOR, "right child = placed above"),
        ):
            child = node.get(child_key)
            if not child or child not in positions:
                continue
            cx, cy = positions[child]
            ax.annotate(
                "",
                xy=(cx, cy + 0.28),
                xytext=(x, y - 0.28),
                arrowprops={"arrowstyle": "->", "color": color, "linewidth": 2.0},
                zorder=1,
            )
            ax.text(
                (x + cx) / 2.0 + 0.18,
                (y + cy) / 2.0,
                label,
                color=color,
                fontsize=7,
                ha="center",
                va="center",
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.78, "pad": 1.2},
            )

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        label = f"{node.get('module', node_id)}\nid={node_id}\nangle={node.get('angle', 0)}"
        box = mpatches.FancyBboxPatch(
            (x - 0.56, y - 0.32),
            1.12,
            0.64,
            boxstyle="round,pad=0.05",
            facecolor=NODE_COLOR,
            edgecolor="#333333",
            linewidth=1.2,
            zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, label, ha="center", va="center", fontsize=8, zorder=4)

    if positions:
        xs = [x for x, _y in positions.values()]
        ys = [y for _x, y in positions.values()]
        ax.set_xlim(min(xs) - 1.4, max(xs) + 1.4)
        ax.set_ylim(min(ys) - 0.8, max(ys) + 0.8)

    ax.legend(
        handles=[
            mpatches.Patch(color=LEFT_COLOR, label="left child: placed right"),
            mpatches.Patch(color=RIGHT_COLOR, label="right child: placed above"),
        ],
        loc="upper right",
        fontsize=8,
    )
    ax.text(
        0.02,
        0.02,
        "B*-tree semantics: left child is packed to the parent's right side; "
        "right child is packed above the parent.",
        transform=ax.transAxes,
        fontsize=8,
        ha="left",
        va="bottom",
    )
    ax.set_title(f"{name} B*-tree structure", fontsize=12, fontweight="bold")
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{name}_btree_structure.png"
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


# 绘制 contour packing 中每个 step 的 occupied bbox 和 routing space 信息。
def render_packing(trace: dict[str, Any], output_dir: Path, name: str, dpi: int) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    steps = trace.get("packing_steps", [])
    fig, ax = plt.subplots(figsize=(12, 7), dpi=dpi)
    ax.set_aspect("equal")
    ax.grid(True, color="#EEEEEE", linewidth=0.5, alpha=0.8)

    all_x: list[float] = []
    all_y: list[float] = []
    centers: dict[str, tuple[float, float]] = {}
    for index, step in enumerate(steps):
        bbox = step["occupied_bbox"]
        x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
        all_x.extend([x1, x2])
        all_y.extend([y1, y2])
        centers[step["tree_node"]] = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        rect = mpatches.Rectangle(
            (x1, y1),
            x2 - x1,
            y2 - y1,
            facecolor=OCCUPIED_COLOR,
            edgecolor="#1565C0",
            alpha=0.25,
            linewidth=1.4,
            zorder=1,
        )
        ax.add_patch(rect)
        module_rect = mpatches.Rectangle(
            (step["x"], step["y"]),
            max(0.1, x2 - x1 - step.get("right_space", 0.0)),
            max(0.1, y2 - y1 - step.get("top_space", 0.0)),
            facecolor=PACK_COLOR,
            edgecolor="#2E7D32",
            alpha=0.32,
            linewidth=1.0,
            zorder=2,
        )
        ax.add_patch(module_rect)
        if step.get("right_space", 0.0) > 0.0:
            ax.add_patch(
                mpatches.Rectangle(
                    (x2 - step["right_space"], y1),
                    step["right_space"],
                    y2 - y1,
                    facecolor=SPACE_COLOR,
                    edgecolor="none",
                    alpha=0.22,
                    zorder=3,
                )
            )
        if step.get("top_space", 0.0) > 0.0:
            ax.add_patch(
                mpatches.Rectangle(
                    (x1, y2 - step["top_space"]),
                    x2 - x1,
                    step["top_space"],
                    facecolor=SPACE_COLOR,
                    edgecolor="none",
                    alpha=0.22,
                    zorder=3,
                )
            )
        label = (
            f"{index}: {step['module']}\n"
            f"contour={step.get('contour_y', 0.0):.1f}\n"
            f"R={step.get('right_space', 0.0):.1f}, T={step.get('top_space', 0.0):.1f}"
        )
        ax.text((x1 + x2) / 2.0, (y1 + y2) / 2.0, label, ha="center", va="center", fontsize=7, zorder=5)

    for step in steps:
        start = centers.get(step["tree_node"])
        if start is None:
            continue
        for child_key, color, label in (
            ("left", LEFT_COLOR, "left -> right side"),
            ("right", RIGHT_COLOR, "right -> top side"),
        ):
            child = step.get(child_key)
            end = centers.get(child)
            if end is None:
                continue
            ax.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops={"arrowstyle": "->", "color": color, "linewidth": 1.2, "linestyle": "--"},
                zorder=4,
            )
            ax.text(
                (start[0] + end[0]) / 2.0,
                (start[1] + end[1]) / 2.0,
                label,
                color=color,
                fontsize=7,
                ha="center",
                va="center",
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.72, "pad": 1.0},
                zorder=5,
            )

    if not all_x:
        all_x = [0.0, 1.0]
        all_y = [0.0, 1.0]
    margin = max(max(all_x) - min(all_x), max(all_y) - min(all_y), 1.0) * 0.1
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
    ax.set_xlabel("X (um)")
    ax.set_ylabel("Y (um)")
    ax.set_title(f"{name} packing contour trace", fontsize=12, fontweight="bold")
    ax.legend(
        handles=[
            mpatches.Patch(facecolor=OCCUPIED_COLOR, edgecolor="#1565C0", alpha=0.25, label="occupied bbox"),
            mpatches.Patch(facecolor=PACK_COLOR, edgecolor="#2E7D32", alpha=0.32, label="module area approx"),
            mpatches.Patch(facecolor=SPACE_COLOR, alpha=0.22, label="routing space"),
            mpatches.Patch(color=LEFT_COLOR, label="left child: right side"),
            mpatches.Patch(color=RIGHT_COLOR, label="right child: above"),
        ],
        loc="upper right",
        fontsize=8,
    )
    fig.tight_layout()
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{name}_btree_packing.png"
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


# 解析命令行参数并输出两张 B*-tree 调试图。
def main() -> int:
    parser = argparse.ArgumentParser(description="Render SAPR B*-tree trace JSON as debug PNGs.")
    parser.add_argument("--trace", required=True, type=Path, help="Path to btree_trace.json")
    parser.add_argument("--output", required=True, type=Path, help="Directory for generated PNG files")
    parser.add_argument("--name", default=None, help="PNG basename prefix; defaults to trace filename")
    parser.add_argument("--dpi", type=int, default=200, help="Output PNG DPI")
    args = parser.parse_args()

    trace = json.loads(args.trace.read_text(encoding="utf-8"))
    name = args.name or default_name(args.trace)
    structure = render_structure(trace, args.output, name, args.dpi)
    packing = render_packing(trace, args.output, name, args.dpi)
    print(os.fspath(structure))
    print(os.fspath(packing))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
