#!/usr/bin/env python3
"""文件职责：将 B*-tree 调试 trace 渲染为结构图和 packing contour 图。"""

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


# 根据 root/left/right 递归计算树图中的节点坐标。
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
        positions[node_id] = (x, -float(depth))
        return x

    if root:
        visit(root, 0)
    for node_id in nodes:
        if node_id not in positions:
            positions[node_id] = (next_x, 0.0)
            next_x += 1.0
    return positions


# 从 packing step 中查找对应节点的 placement 坐标，用于结构图标签。
def placement_by_node(trace: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {step["tree_node"]: step for step in trace.get("packing_steps", [])}


# 绘制 B*-tree 逻辑结构，蓝边表示 left child，橙边表示 right child。
def render_structure(trace: dict[str, Any], output_dir: Path, name: str, dpi: int) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    nodes = {node["id"]: node for node in trace.get("nodes", [])}
    positions = compute_tree_positions(trace)
    packed = placement_by_node(trace)
    width = max(8.0, min(20.0, 2.2 * max(len(nodes), 1)))
    height = max(5.0, 1.6 * (1 + max((-y for _x, y in positions.values()), default=0.0)))
    fig, ax = plt.subplots(figsize=(width, height), dpi=dpi)
    ax.set_aspect("equal")
    ax.axis("off")

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        for child_key, color, label in (("left", LEFT_COLOR, "left child"), ("right", RIGHT_COLOR, "right child")):
            child = node.get(child_key)
            if not child or child not in positions:
                continue
            cx, cy = positions[child]
            ax.plot([x, cx], [y - 0.12, cy + 0.12], color=color, linewidth=2.0, zorder=1)
            ax.text((x + cx) / 2.0, (y + cy) / 2.0, label.split()[0], color=color, fontsize=7)

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        step = packed.get(node_id, {})
        label = f"{node.get('module', node_id)}\nangle={node.get('angle', 0)}"
        if step:
            label += f"\nx={step.get('x', 0):.1f}, y={step.get('y', 0):.1f}"
        box = mpatches.FancyBboxPatch(
            (x - 0.52, y - 0.28),
            1.04,
            0.56,
            boxstyle="round,pad=0.05",
            facecolor=NODE_COLOR,
            edgecolor="#333333",
            linewidth=1.2,
            zorder=3,
        )
        ax.add_patch(box)
        ax.text(x, y, label, ha="center", va="center", fontsize=8, zorder=4)

    ax.legend(
        handles=[
            mpatches.Patch(color=LEFT_COLOR, label="left child"),
            mpatches.Patch(color=RIGHT_COLOR, label="right child"),
        ],
        loc="upper right",
        fontsize=8,
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
        for child_key, color in (("left", LEFT_COLOR), ("right", RIGHT_COLOR)):
            child = step.get(child_key)
            end = centers.get(child)
            if end is None:
                continue
            ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linewidth=1.2, linestyle="--", zorder=4)

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
