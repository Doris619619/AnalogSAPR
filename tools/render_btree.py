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
ROUTING_ARC_COLORS = ["#D81B60", "#00897B", "#5E35B1", "#C0CA33", "#6D4C41", "#039BE5"]


# 输出目录名为空时提供稳定的默认图片名前缀。
def default_name(trace_path: Path) -> str:
    stem = trace_path.stem
    return stem if stem else "btree"


# 根据 root/left/right 递归计算 module node 的逻辑树坐标。
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
            next_x += 1.8
        positions[node_id] = (x, -2.35 * float(depth))
        return x

    if root:
        visit(root, 0)
    for node_id in nodes:
        if node_id not in positions:
            positions[node_id] = (next_x, 0.0)
            next_x += 1.8
    return positions


# 返回每个 module node 对应 LS/RS space node 的绘图坐标。
def compute_space_positions(positions: dict[str, tuple[float, float]]) -> dict[tuple[str, str], tuple[float, float]]:
    result: dict[tuple[str, str], tuple[float, float]] = {}
    for node_id, (x, y) in positions.items():
        result[(node_id, "left")] = (x - 0.58, y - 0.86)
        result[(node_id, "right")] = (x + 0.58, y - 0.86)
    return result


# 建立 module id 和 space node id 到结构图坐标的映射，供 routing topology arc 查找端点。
def build_structure_anchor_maps(
    nodes: dict[str, dict[str, Any]],
    positions: dict[str, tuple[float, float]],
    space_positions: dict[tuple[str, str], tuple[float, float]],
) -> tuple[dict[str, tuple[float, float]], dict[str, tuple[float, float]]]:
    module_positions: dict[str, tuple[float, float]] = {}
    space_node_positions: dict[str, tuple[float, float]] = {}
    for node_id, node in nodes.items():
        if node_id not in positions:
            continue
        module_positions[node_id] = positions[node_id]
        module_positions[node.get("module", node_id)] = positions[node_id]
        for side, field in (("left", "left_space_node"), ("right", "right_space_node")):
            space = node.get(field, {})
            space_id = space.get("id")
            if space_id:
                space_node_positions[space_id] = space_positions[(node_id, side)]
    return module_positions, space_node_positions


# 从 terminal 名称中取出 module 名称，LCP terminal 会返回 None。
def module_from_terminal(terminal: str) -> str | None:
    if "." not in terminal:
        return None
    module, _pin = terminal.split(".", 1)
    return module


# 将 routing topology 中的 pin/LCP endpoint 映射到结构图坐标。
def endpoint_position(
    endpoint: str,
    module_positions: dict[str, tuple[float, float]],
    lcp_positions: dict[str, tuple[float, float]],
) -> tuple[float, float] | None:
    if endpoint in lcp_positions:
        return lcp_positions[endpoint]
    module = module_from_terminal(endpoint)
    if module and module in module_positions:
        return module_positions[module]
    return module_positions.get(endpoint)


# 为每个 LCP 选择结构图中的显示坐标；位置落在所属 space node 方块内部。
def build_lcp_positions(trace: dict[str, Any], space_node_positions: dict[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
    lcp_positions: dict[str, tuple[float, float]] = {}
    per_space_count: dict[str, int] = {}
    for topology in trace.get("routing_topologies", []):
        for point in topology.get("linking_points", []):
            lcp_id = point.get("id")
            space_id = point.get("space_node_id")
            if not lcp_id or not space_id or space_id not in space_node_positions:
                continue
            index = per_space_count.get(space_id, 0)
            per_space_count[space_id] = index + 1
            base_x, base_y = space_node_positions[space_id]
            shown_index = min(index, 2)
            x = base_x + (shown_index - 1) * 0.10
            y = base_y - 0.10
            lcp_positions[lcp_id] = (x, y)
    return lcp_positions


# 绘制论文拓扑意义上的 module -> LCP -> module 布线弧线。
def draw_routing_topology_arcs(
    ax: Any,
    trace: dict[str, Any],
    module_positions: dict[str, tuple[float, float]],
    lcp_positions: dict[str, tuple[float, float]],
) -> bool:
    topologies = trace.get("routing_topologies", [])
    if not topologies:
        return False
    drew_any = False
    for topology_index, topology in enumerate(topologies):
        color = ROUTING_ARC_COLORS[topology_index % len(ROUTING_ARC_COLORS)]
        net = topology.get("net", "")
        priority = topology.get("priority", "normal")
        linewidth = 2.0 if priority == "critical" else 1.35
        for segment_index, segment in enumerate(topology.get("segments", [])):
            start = endpoint_position(segment.get("from", ""), module_positions, lcp_positions)
            end = endpoint_position(segment.get("to", ""), module_positions, lcp_positions)
            if start is None or end is None:
                continue
            rad = 0.22 if segment_index % 2 == 0 else -0.22
            ax.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops={
                    "arrowstyle": "->",
                    "color": color,
                    "linewidth": linewidth,
                    "connectionstyle": f"arc3,rad={rad}",
                    "alpha": 0.86,
                },
                zorder=3,
            )
            drew_any = True
        if topology.get("segments"):
            label_pos = None
            for segment in topology.get("segments", []):
                label_pos = endpoint_position(segment.get("to", ""), module_positions, lcp_positions)
                if label_pos is not None:
                    break
            if label_pos is not None:
                ax.text(label_pos[0] + 0.08, label_pos[1] + 0.08, str(net), fontsize=6, color=color, zorder=7)
    return drew_any


# 返回 space node 的短标签，同时展示论文树侧命名和几何空间含义。
def space_label(space: dict[str, Any], fallback: str) -> str:
    if not space:
        return fallback
    prefix = "LS" if space.get("tree_side") == "left_space_node" else "RS"
    geometry = "right" if space.get("geometry_space") == "right_space" else "top"
    return f"{prefix}\n{geometry}"


# 在 space node 方块内部绘制 LCP 小点；点太多时用 xN 压缩显示。
def draw_lcp_inside_space(ax: Any, x: float, y: float, count: int) -> None:
    import matplotlib.patches as mpatches

    if count <= 0:
        return
    shown = min(count, 3)
    spacing = 0.10
    start = x - spacing * (shown - 1) / 2.0
    dot_y = y - 0.10
    for index in range(shown):
        ax.add_patch(
            mpatches.Circle(
                (start + index * spacing, dot_y),
                0.035,
                facecolor=LCP_COLOR,
                edgecolor="#4527A0",
                linewidth=0.5,
                zorder=6,
            )
        )
    if count > 3:
        ax.text(x + 0.20, dot_y, f"x{count}", ha="left", va="center", fontsize=6, color=LCP_COLOR, zorder=6)


# 绘制单个 space node 方块，LCP 被包含在方块内部。
def draw_space_node(ax: Any, x: float, y: float, space: dict[str, Any], fallback: str) -> None:
    import matplotlib.patches as mpatches

    square = mpatches.FancyBboxPatch(
        (x - 0.28, y - 0.20),
        0.56,
        0.40,
        boxstyle="round,pad=0.03",
        facecolor=SPACE_NODE_COLOR,
        edgecolor="#8D6E00",
        linewidth=1.0,
        alpha=0.92,
        zorder=4,
    )
    ax.add_patch(square)
    ax.text(x, y + 0.07, space_label(space, fallback), ha="center", va="center", fontsize=6, zorder=5)
    draw_lcp_inside_space(ax, x, y, int(space.get("lcp_count", 0) or 0))


# 根据 SA 轮次元数据生成结构图多行标题，突出扰动序号与操作。
def sa_iteration_title(trace: dict[str, Any], name: str) -> str:
    meta = trace.get("sa_iteration")
    if not isinstance(meta, dict):
        return (
            f"{name} enhanced B*-tree structure\n"
            "LS = left space node / right-side routing space, RS = right space node / top routing space"
        )
    index = meta.get("index", "?")
    total = meta.get("total", "?")
    move = meta.get("move", "none")
    changed = "yes" if meta.get("changed") else "no"
    accept = "ACCEPT" if meta.get("accept") else "REJECT"
    next_cost = meta.get("next_cost")
    before_cost = meta.get("current_cost_before")
    temperature = meta.get("temperature")
    delta = None
    if isinstance(next_cost, (int, float)) and isinstance(before_cost, (int, float)):
        delta = float(next_cost) - float(before_cost)
    lines = [
        f"perturbation #{index} / {total}   move={move}   changed={changed}   {accept}",
    ]
    detail_parts: list[str] = []
    if isinstance(next_cost, (int, float)):
        detail_parts.append(f"next_cost={float(next_cost):.4g}")
    if delta is not None:
        detail_parts.append(f"delta={delta:+.4g}")
    if isinstance(temperature, (int, float)):
        detail_parts.append(f"T={float(temperature):.4g}")
    if detail_parts:
        lines.append("   ".join(detail_parts))
    lines.append("LS = left space node / right-side routing space, RS = right space node / top routing space")
    return "\n".join(lines)


# 绘制 enhanced B*-tree：module 通过对应 space node 再连接到 child module。
def render_structure(
    trace: dict[str, Any],
    output_dir: Path,
    name: str,
    dpi: int,
    output_basename: str | None = None,
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.lines as mlines
    import matplotlib.patches as mpatches

    nodes = {node["id"]: node for node in trace.get("nodes", [])}
    positions = compute_tree_positions(trace)
    space_positions = compute_space_positions(positions)
    module_positions, space_node_positions = build_structure_anchor_maps(nodes, positions, space_positions)
    lcp_positions = build_lcp_positions(trace, space_node_positions)
    width = max(9.0, min(22.0, 2.6 * max(len(nodes), 1)))
    max_depth = max((-y for _x, y in positions.values()), default=0.0)
    height = max(6.0, min(15.0, 1.35 * (2.5 + max_depth)))
    fig, ax = plt.subplots(figsize=(width, height), dpi=dpi)
    ax.set_aspect("equal")
    ax.axis("off")

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        left_space = space_positions[(node_id, "left")]
        right_space = space_positions[(node_id, "right")]

        ax.plot([x, left_space[0]], [y - 0.42, left_space[1] + 0.20], color=SPACE_EDGE_COLOR, linestyle="--", linewidth=1.0, zorder=1)
        ax.plot([x, right_space[0]], [y - 0.42, right_space[1] + 0.20], color=SPACE_EDGE_COLOR, linestyle="--", linewidth=1.0, zorder=1)

        child = node.get("left")
        if child and child in positions:
            cx, cy = positions[child]
            ax.annotate(
                "",
                xy=(cx, cy + 0.46),
                xytext=(left_space[0], left_space[1] - 0.22),
                arrowprops={"arrowstyle": "->", "color": LEFT_COLOR, "linewidth": 2.0},
                zorder=2,
            )

        child = node.get("right")
        if child and child in positions:
            cx, cy = positions[child]
            ax.annotate(
                "",
                xy=(cx, cy + 0.46),
                xytext=(right_space[0], right_space[1] - 0.22),
                arrowprops={"arrowstyle": "->", "color": RIGHT_COLOR, "linewidth": 2.0},
                zorder=2,
            )

    drew_routing_arcs = draw_routing_topology_arcs(ax, trace, module_positions, lcp_positions)

    for node_id, node in nodes.items():
        x, y = positions[node_id]
        circle = mpatches.Circle((x, y), 0.43, facecolor=MODULE_COLOR, edgecolor="#212121", linewidth=1.4, zorder=5)
        ax.add_patch(circle)
        ax.text(x, y + 0.04, node.get("module", node_id), ha="center", va="center", fontsize=9, fontweight="bold", zorder=6)
        ax.text(x, y - 0.18, f"ang={node.get('angle', 0)}", ha="center", va="center", fontsize=6, zorder=6)
        draw_space_node(ax, *space_positions[(node_id, "left")], node.get("left_space_node", {}), "left_space_node")
        draw_space_node(ax, *space_positions[(node_id, "right")], node.get("right_space_node", {}), "right_space_node")

    if positions:
        xs = [x for x, _y in positions.values()] + [x for x, _y in space_positions.values()]
        ys = [y for _x, y in positions.values()] + [y for _x, y in space_positions.values()]
        ax.set_xlim(min(xs) - 1.7, max(xs) + 1.7)
        ax.set_ylim(min(ys) - 0.9, max(ys) + 1.1)

    legend_handles = [
        mpatches.Circle((0, 0), 0.12, facecolor=MODULE_COLOR, edgecolor="#212121", label="module node"),
        mpatches.FancyBboxPatch((0, 0), 0.2, 0.2, boxstyle="round,pad=0.02", facecolor=SPACE_NODE_COLOR, edgecolor="#8D6E00", label="space node"),
        mlines.Line2D([], [], color=LCP_COLOR, marker="o", linestyle="None", markersize=5, label="LCP inside space node"),
        mlines.Line2D([], [], color=SPACE_EDGE_COLOR, linewidth=1.0, linestyle="--", label="parent module -> space node"),
        mlines.Line2D([], [], color=LEFT_COLOR, linewidth=2.0, label="left space node -> left child = placed right"),
        mlines.Line2D([], [], color=RIGHT_COLOR, linewidth=2.0, label="right space node -> right child = placed above"),
    ]
    if drew_routing_arcs:
        legend_handles.extend(
            [
                mlines.Line2D([], [], color=ROUTING_ARC_COLORS[0], linewidth=1.35, label="routing topology arc"),
                mlines.Line2D([], [], color=ROUTING_ARC_COLORS[0], linewidth=2.0, label="critical routing topology"),
            ]
        )
    ax.legend(handles=legend_handles, loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    title = sa_iteration_title(trace, name)
    title_fontsize = 13 if "sa_iteration" in trace else 12
    ax.set_title(title, fontsize=title_fontsize, fontweight="bold", pad=16, loc="left")
    fig.subplots_adjust(top=0.82)
    output_dir.mkdir(parents=True, exist_ok=True)
    file_stem = output_basename or f"{name}_btree_structure"
    out_path = output_dir / f"{file_stem}.png"
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


# 解析命令行参数并输出 enhanced B*-tree 调试图。
def main() -> int:
    parser = argparse.ArgumentParser(description="Render SAPR enhanced B*-tree trace JSON as debug PNGs.")
    parser.add_argument("--trace", required=True, type=Path, help="Path to btree_trace.json")
    parser.add_argument("--output", required=True, type=Path, help="Directory for generated PNG files")
    parser.add_argument("--name", default=None, help="PNG basename prefix; defaults to trace filename")
    parser.add_argument("--dpi", type=int, default=200, help="Output PNG DPI")
    parser.add_argument(
        "--structure-only",
        action="store_true",
        help="Only render the structure PNG (skip packing contour figure)",
    )
    parser.add_argument(
        "--output-basename",
        default=None,
        help="Exact structure PNG stem (without .png); defaults to <name>_btree_structure",
    )
    args = parser.parse_args()

    trace = json.loads(args.trace.read_text(encoding="utf-8"))
    name = args.name or default_name(args.trace)
    structure = render_structure(trace, args.output, name, args.dpi, args.output_basename)
    print(os.fspath(structure))
    if not args.structure_only:
        packing = render_packing(trace, args.output, name, args.dpi)
        print(os.fspath(packing))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
