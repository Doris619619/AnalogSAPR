#!/usr/bin/env python3
"""文件职责：将 enhanced B*-tree trace 渲染为结构图和 packing contour 图。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any


LEFT_COLOR = "#1E88E5"
RIGHT_COLOR = "#FB8C00"
MODULE_COLOR = "#FFFFFF"
SPACE_NODE_COLOR = "#F9A825"
LCP_COLOR = "#7E57C2"
PACK_COLOR = "#4CAF50"
OCCUPIED_COLOR = "#90CAF9"
ROUTING_SPACE_COLOR = "#F9A825"
SPACE_EDGE_COLOR = "#616161"


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
            next_x += 1.6
        positions[node_id] = (x, -2.0 * float(depth))
        return x

    if root:
        visit(root, 0)
    for node_id in nodes:
        if node_id not in positions:
            positions[node_id] = (next_x, 0.0)
            next_x += 1.6
    return positions


# 返回 space node 的简短标签，用论文树侧命名和几何空间命名同时说明含义。
def space_label(space: dict[str, Any], fallback: str) -> str:
    if not space:
        return fallback
    if space.get("tree_side") == "left_space_node":
        prefix = "LS"
    else:
        prefix = "RS"
    geometry = "right" if space.get("geometry_space") == "right_space" else "top"
    return f"{prefix}\n{geometry}"


# 绘制 LCP 小点；数量较多时只画前几个点，并显示总数。
def draw_lcp_markers(ax: Any, x: float, y: float, count: int) -> None:
    import matplotlib.patches as mpatches

    if count <= 0:
        return
    shown = min(count, 3)
    spacing = 0.12
    start = x - spacing * (shown - 1) / 2.0
    for index in range(shown):
        ax.add_patch(
            mpatches.Circle(
                (start + index * spacing, y),
                0.045,
                facecolor=LCP_COLOR,
                edgecolor="#4527A0",
                linewidth=0.6,
                zorder=5,
            )
        )
    label = f"LCP x{count}" if count > 1 else "LCP"
    ax.text(x, y - 0.18, label, ha="center", va="top", fontsize=6, color=LCP_COLOR, zorder=6)


# 绘制某个 module node 对应的论文 space node 结构。
def draw_space_nodes(ax: Any, node: dict[str, Any], x: float, y: float) -> None:
    import matplotlib.patches as mpatches

    specs = [
        (node.get("left_space_node", {}), x - 0.52, y - 0.78, "left_space_node"),
        (node.get("right_space_node", {}), x + 0.52, y - 0.78, "right_space_node"),
    ]
    for space, sx, sy, fallback in specs:
        ax.plot([x, sx], [y - 0.36, sy + 0.12], color=SPACE_EDGE_COLOR, linestyle="--", linewidth=1.0, zorder=1)
        square = mpatches.FancyBboxPatch(
            (sx - 0.22, sy - 0.16),
            0.44,
            0.32,
            boxstyle="round,pad=0.02",
            facecolor=SPACE_NODE_COLOR,
            edgecolor="#8D6E00",
            linewidth=1.0,
            alpha=0.9,
            zorder=3,
        )
        ax.add_patch(square)
        ax.text(sx, sy, space_label(space, fallback), ha="center", va="center", fontsize=6, zorder=4)
        draw_lcp_markers(ax, sx, sy - 0.34, int(space.get("lcp_count", 0) or 0))


# 绘制 enhanced B*-tree：圆形 module node、方形 space node 和 LCP 归属关系。
def render_structure(trace: dict[str, Any], output_dir: Path, name: str, dpi: int) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.lines as mlines
    import matplotlib.patches as mpatches

    nodes = {node["id"]: node for node in trace.get("nodes", [])}
    positions = compute_tree_positions(trace)
    width = max(9.0, min(20.0, 2.4 * max(len(nodes), 1)))
    max_depth = max((-y for _x, y in positions.values()), default=0.0)
    height = max(6.0, min(14.0, 1.35 * (2.2 + max_depth)))
    fig, ax = plt.subplots(figsize=(width, height), dpi=dpi)
    ax.set_aspect("equal")
    ax.axis("off")

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        for child_key, color in (("left", LEFT_COLOR), ("right", RIGHT_COLOR)):
            child = node.get(child_key)
            if not child or child not in positions:
                continue
            cx, cy = positions[child]
            ax.annotate(
                "",
                xy=(cx, cy + 0.46),
                xytext=(x, y - 0.46),
                arrowprops={"arrowstyle": "->", "color": color, "linewidth": 2.0},
                zorder=0,
            )

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        circle = mpatches.Circle((x, y), 0.43, facecolor=MODULE_COLOR, edgecolor="#212121", linewidth=1.4, zorder=4)
        ax.add_patch(circle)
        ax.text(x, y + 0.04, node.get("module", node_id), ha="center", va="center", fontsize=9, fontweight="bold", zorder=5)
        ax.text(x, y - 0.18, f"ang={node.get('angle', 0)}", ha="center", va="center", fontsize=6, zorder=5)
        draw_space_nodes(ax, node, x, y)

    if positions:
        xs = [x for x, _y in positions.values()]
        ys = [y for _x, y in positions.values()]
        ax.set_xlim(min(xs) - 1.7, max(xs) + 1.7)
        ax.set_ylim(min(ys) - 1.45, max(ys) + 1.1)

    legend_handles = [
        mpatches.Circle((0, 0), 0.12, facecolor=MODULE_COLOR, edgecolor="#212121", label="module node"),
        mpatches.FancyBboxPatch((0, 0), 0.2, 0.2, boxstyle="round,pad=0.02", facecolor=SPACE_NODE_COLOR, edgecolor="#8D6E00", label="space node"),
        mlines.Line2D([], [], color=LCP_COLOR, marker="o", linestyle="None", markersize=5, label="linking-control point"),
        mlines.Line2D([], [], color=LEFT_COLOR, linewidth=2.0, label="left child = placed right"),
        mlines.Line2D([], [], color=RIGHT_COLOR, linewidth=2.0, label="right child = placed above"),
        mlines.Line2D([], [], color=SPACE_EDGE_COLOR, linewidth=1.0, linestyle="--", label="module to space node"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    ax.set_title(
        f"{name} enhanced B*-tree structure\n"
        "LS = left space node / right-side routing space, RS = right space node / top routing space",
        fontsize=12,
        fontweight="bold",
    )
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
    import matplotlib.lines as mlines
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
        ax.add_patch(
            mpatches.Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                facecolor=OCCUPIED_COLOR,
                edgecolor="#1565C0",
                alpha=0.25,
                linewidth=1.4,
                zorder=1,
            )
        )
        ax.add_patch(
            mpatches.Rectangle(
                (step["x"], step["y"]),
                max(0.1, x2 - x1 - step.get("right_space", 0.0)),
                max(0.1, y2 - y1 - step.get("top_space", 0.0)),
                facecolor=PACK_COLOR,
                edgecolor="#2E7D32",
                alpha=0.32,
                linewidth=1.0,
                zorder=2,
            )
        )
        if step.get("right_space", 0.0) > 0.0:
            ax.add_patch(
                mpatches.Rectangle(
                    (x2 - step["right_space"], y1),
                    step["right_space"],
                    y2 - y1,
                    facecolor=ROUTING_SPACE_COLOR,
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
                    facecolor=ROUTING_SPACE_COLOR,
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
            ax.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops={"arrowstyle": "->", "color": color, "linewidth": 1.2, "linestyle": "--"},
                zorder=4,
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
            mpatches.Patch(facecolor=ROUTING_SPACE_COLOR, alpha=0.22, label="routing space"),
            mlines.Line2D([], [], color=LEFT_COLOR, linewidth=1.2, linestyle="--", label="left child: right side"),
            mlines.Line2D([], [], color=RIGHT_COLOR, linewidth=1.2, linestyle="--", label="right child: above"),
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


# 解析命令行参数并输出两张 enhanced B*-tree 调试图。
def main() -> int:
    parser = argparse.ArgumentParser(description="Render SAPR enhanced B*-tree trace JSON as debug PNGs.")
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
