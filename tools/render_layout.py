#!/usr/bin/env python3
"""文件职责：将 SAPR 的 IO 文本输出渲染为布局 PNG。"""

from __future__ import annotations

import argparse
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass
# 表示 input/modules.txt 中的器件尺寸、有源区和类型信息。
class Module:
    module_id: str
    width: float
    height: float
    active: tuple[float, float, float, float]
    ox: float
    oy: float
    device_type: str
    info: str


@dataclass
# 表示 input/pins.txt 中的器件局部 pin 坐标。
class Pin:
    module_id: str
    name: str
    x: float
    y: float
    layer: str


@dataclass
# 表示 output/placement.txt 中的器件全局放置结果。
class Placement:
    module_id: str
    x: float
    y: float
    angle: int
    orient: str


@dataclass
# 表示 output/routing.txt 中的一段走线中心线和线宽。
class Route:
    net: str
    layer: str
    x1: float
    y1: float
    x2: float
    y2: float
    width: float


DEV_COLORS = {
    "nmos": "#2196F3",
    "pmos": "#F44336",
    "capacitor": "#4CAF50",
    "resistor": "#FF9800",
    "unknown": "#999999",
}

LAYER_COLORS = {
    "M1": "#2196F3",
    "M2": "#F44336",
    "M3": "#4CAF50",
    "M4": "#FF9800",
    "M5": "#9C27B0",
    "M6": "#00BCD4",
    "M7": "#795548",
}


# 拆分一行数据和尾部注释，空白字段用于后续格式校验。
def strip_line(raw: str) -> tuple[list[str], str]:
    body, sep, comment = raw.partition("#")
    fields = body.strip().split()
    return fields, comment.strip() if sep else ""


# 读取 IO 文本文件中的非空数据行，并保留注释用于器件类型推断。
def read_rows(path: Path) -> list[tuple[list[str], str]]:
    rows: list[tuple[list[str], str]] = []
    with path.open(encoding="utf-8") as handle:
        for raw in handle:
            fields, comment = strip_line(raw)
            if fields:
                rows.append((fields, comment))
    return rows


# 将 x1,y1,x2,y2 字符串解析为矩形坐标。
def parse_rect(value: str) -> tuple[float, float, float, float]:
    parts = value.split(",")
    if len(parts) != 4:
        raise ValueError(f"invalid rectangle: {value}")
    return tuple(float(part) for part in parts)  # type: ignore[return-value]


# 根据 modules.txt 的注释字段推断器件类型，用于选择渲染颜色。
def infer_device_type(comment: str) -> str:
    lowered = comment.lower()
    for device_type in ("nmos", "pmos", "capacitor", "resistor"):
        if device_type in lowered:
            return device_type
    if "nch" in lowered:
        return "nmos"
    if "pch" in lowered:
        return "pmos"
    if "cap" in lowered or "cfmom" in lowered or "mim" in lowered:
        return "capacitor"
    if "res" in lowered or "rupoly" in lowered or "rppoly" in lowered:
        return "resistor"
    return "unknown"


def parse_comment_offset(comment: str, key: str) -> float:
    match = re.search(rf"\b{re.escape(key)}=([-+]?\d+(?:\.\d+)?)", comment)
    return float(match.group(1)) if match else 0.0


# 加载器件定义，要求 modules.txt 至少包含 id、宽、高、有源区四列。
def load_modules(input_dir: Path) -> dict[str, Module]:
    modules: dict[str, Module] = {}
    for fields, comment in read_rows(input_dir / "modules.txt"):
        if len(fields) < 4:
            raise ValueError("modules.txt rows must have at least 4 columns")
        module = Module(
            module_id=fields[0],
            width=float(fields[1]),
            height=float(fields[2]),
            active=parse_rect(fields[3]),
            ox=parse_comment_offset(comment, "ox"),
            oy=parse_comment_offset(comment, "oy"),
            device_type=infer_device_type(comment),
            info=comment,
        )
        modules[module.module_id] = module
    return modules


# 加载 pin 定义，pin 坐标为相对器件 BB 左下角的局部坐标。
def load_pins(input_dir: Path) -> list[Pin]:
    pins: list[Pin] = []
    for fields, _comment in read_rows(input_dir / "pins.txt"):
        if len(fields) != 5:
            raise ValueError("pins.txt rows must have 5 columns")
        pins.append(Pin(fields[0], fields[1], float(fields[2]), float(fields[3]), fields[4]))
    return pins


# 加载器件放置结果，供器件和 pin 渲染复用。
def load_placements(output_dir: Path) -> dict[str, Placement]:
    placements: dict[str, Placement] = {}
    for fields, _comment in read_rows(output_dir / "placement.txt"):
        if len(fields) != 5:
            raise ValueError("placement.txt rows must have 5 columns")
        placement = Placement(fields[0], float(fields[1]), float(fields[2]), int(fields[3]), fields[4])
        placements[placement.module_id] = placement
    return placements


# 加载走线中心线段，线宽用于扩展成可视化多边形。
def load_routes(output_dir: Path) -> list[Route]:
    routes: list[Route] = []
    for fields, _comment in read_rows(output_dir / "routing.txt"):
        if len(fields) != 7:
            raise ValueError("routing.txt rows must have 7 columns")
        routes.append(
            Route(
                net=fields[0],
                layer=fields[1],
                x1=float(fields[2]),
                y1=float(fields[3]),
                x2=float(fields[4]),
                y2=float(fields[5]),
                width=float(fields[6]),
            )
        )
    return routes


# 将 BB 左下角局部坐标转换为全局坐标；ox/oy 仅作为 GDS 元数据保留。
def transform_point(x: float, y: float, module: Module, placement: Placement) -> tuple[float, float]:
    orient = placement.orient
    if orient == "MX":
        gx, gy = x, module.height - y
    elif orient == "MY":
        gx, gy = module.width - x, y
    elif orient == "MXR90":
        gx, gy = y, x
    elif orient == "MYR90":
        gx, gy = y, x
    else:
        angle = (placement.angle % 360 + 360) % 360
        if angle == 0:
            gx, gy = x, y
        elif angle == 90:
            gx, gy = module.height - y, x
        elif angle == 180:
            gx, gy = module.width - x, module.height - y
        elif angle == 270:
            gx, gy = y, module.width - x
        else:
            raise ValueError(f"unsupported placement angle: {placement.angle}")
    return placement.x + gx, placement.y + gy


# 返回器件外接框四角的全局坐标，用于绘制器件多边形。
def module_corners(module: Module, placement: Placement) -> list[tuple[float, float]]:
    local = [(0.0, 0.0), (module.width, 0.0), (module.width, module.height), (0.0, module.height)]
    return [transform_point(x, y, module, placement) for x, y in local]


# 将走线中心线段按实际线宽扩展为渲染多边形。
def route_polygon(route: Route) -> list[tuple[float, float]]:
    half_width = max(route.width, 0.001) / 2.0
    dx = route.x2 - route.x1
    dy = route.y2 - route.y1
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return [
            (route.x1 - half_width, route.y1 - half_width),
            (route.x1 + half_width, route.y1 - half_width),
            (route.x1 + half_width, route.y1 + half_width),
            (route.x1 - half_width, route.y1 + half_width),
        ]
    px = -dy / length * half_width
    py = dx / length * half_width
    return [
        (route.x1 + px, route.y1 + py),
        (route.x1 - px, route.y1 - py),
        (route.x2 - px, route.y2 - py),
        (route.x2 + px, route.y2 + py),
    ]


# 输出目录名为空时提供稳定的默认 PNG 前缀。
def default_render_name(output_dir: Path) -> str:
    name = output_dir.name.strip()
    return name if name else "layout"


# 渲染完整布局 PNG，并返回生成的文件路径。
def render_layout(
    input_dir: Path,
    output_dir: Path,
    render_name: str,
    dpi: int,
    show_labels: bool,
    show_pins: bool,
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt

    modules = load_modules(input_dir)
    pins = load_pins(input_dir)
    placements = load_placements(output_dir)
    routes = load_routes(output_dir)

    pin_by_module: dict[str, list[Pin]] = {}
    for pin in pins:
        pin_by_module.setdefault(pin.module_id, []).append(pin)

    all_x: list[float] = []
    all_y: list[float] = []
    placed_modules: list[tuple[Module, Placement, list[tuple[float, float]]]] = []
    for module_id, module in modules.items():
        placement = placements.get(module_id)
        if placement is None:
            continue
        corners = module_corners(module, placement)
        placed_modules.append((module, placement, corners))
        all_x.extend(x for x, _y in corners)
        all_y.extend(y for _x, y in corners)

    for route in routes:
        half_width = max(route.width, 0.001) / 2.0
        all_x.extend([route.x1 - half_width, route.x1 + half_width, route.x2 - half_width, route.x2 + half_width])
        all_y.extend([route.y1 - half_width, route.y1 + half_width, route.y2 - half_width, route.y2 + half_width])

    if not all_x:
        all_x = [0.0, 100.0]
        all_y = [0.0, 100.0]

    span_x = max(all_x) - min(all_x)
    span_y = max(all_y) - min(all_y)
    margin = max(span_x, span_y, 1.0) * 0.08
    xlim = (min(all_x) - margin, max(all_x) + margin)
    ylim = (min(all_y) - margin, max(all_y) + margin)
    fig_w = max(8.0, min(24.0, (xlim[1] - xlim[0]) / 20.0))
    fig_h = max(8.0, min(24.0, (ylim[1] - ylim[0]) / 20.0))

    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    ax.set_aspect("equal")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.grid(True, color="#EEEEEE", linewidth=0.5, alpha=0.7)

    sorted_routes = sorted(routes, key=lambda route: -max(abs(route.x2 - route.x1), abs(route.y2 - route.y1)))
    for route in sorted_routes[:5000]:
        color = LAYER_COLORS.get(route.layer, "#999999")
        patch = mpatches.Polygon(route_polygon(route), closed=True, facecolor=color, edgecolor="none", alpha=0.55, zorder=1)
        ax.add_patch(patch)

    for module, placement, corners in placed_modules:
        color = DEV_COLORS.get(module.device_type, DEV_COLORS["unknown"])
        patch = mpatches.Polygon(corners, closed=True, facecolor=color, edgecolor=color, linewidth=1.0, alpha=0.35, zorder=3)
        ax.add_patch(patch)
        cx = sum(x for x, _y in corners) / len(corners)
        cy = sum(y for _x, y in corners) / len(corners)
        ax.text(cx, cy, module.module_id, ha="center", va="center", fontsize=5, fontweight="bold", color="#111111", zorder=10)
        if show_pins:
            for pin in pin_by_module.get(module.module_id, []):
                gx, gy = transform_point(pin.x, pin.y, module, placement)
                pin_color = LAYER_COLORS.get(pin.layer, "#333333")
                ax.plot(gx, gy, marker="x", color=pin_color, markersize=4, markeredgewidth=1.5, zorder=11)
                if show_labels:
                    ax.annotate(pin.name, (gx, gy), textcoords="offset points", xytext=(3, 3), fontsize=4, color=pin_color, zorder=12)

    legend: list[mpatches.Patch] = []
    for device_type in sorted({module.device_type for module, _placement, _corners in placed_modules}):
        color = DEV_COLORS.get(device_type, DEV_COLORS["unknown"])
        legend.append(mpatches.Patch(facecolor=color, edgecolor=color, alpha=0.35, label=device_type))
    for layer in sorted({route.layer for route in routes}):
        legend.append(mpatches.Patch(color=LAYER_COLORS.get(layer, "#999999"), label=f"{layer} routing"))
    if legend:
        ax.legend(handles=legend, loc="upper right", fontsize=7, framealpha=0.8)

    ax.set_title(f"{render_name} - {len(placed_modules)} modules", fontsize=10, fontweight="bold")
    ax.set_xlabel("X (um)")
    ax.set_ylabel("Y (um)")
    ax.tick_params(labelsize=7)
    fig.tight_layout()

    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{render_name}_layout.png"
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


# 解析命令行参数并执行渲染。
def main() -> int:
    parser = argparse.ArgumentParser(description="Render SAPR input/output text files as a layout PNG.")
    parser.add_argument("--input", required=True, type=Path, help="Directory containing modules.txt and pins.txt")
    parser.add_argument("--output", required=True, type=Path, help="Directory containing placement.txt and routing.txt")
    parser.add_argument("--name", default=None, help="PNG basename prefix; defaults to output directory name")
    parser.add_argument("--dpi", type=int, default=200, help="Output PNG DPI")
    parser.add_argument("--no-labels", action="store_true", help="Hide pin labels")
    parser.add_argument("--no-pins", action="store_true", help="Hide pin markers")
    args = parser.parse_args()

    render_name = args.name or default_render_name(args.output)
    out_path = render_layout(
        input_dir=args.input,
        output_dir=args.output,
        render_name=render_name,
        dpi=args.dpi,
        show_labels=not args.no_labels,
        show_pins=not args.no_pins,
    )
    print(os.fspath(out_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
