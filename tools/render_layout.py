#!/usr/bin/env python3
"""文件职责：将 SAPR 的 IO 文本输出渲染为布局 PNG（合图 + 按金属层分图）。"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# 允许从仓库根或 tools/ 目录直接运行本脚本时找到同目录色板模块。
_TOOLS_DIR = Path(__file__).resolve().parent
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from net_colors import net_color_map

# 短边小于该阈值（μm）的器件视为小器件：外置模块名、外置 pin 名、缩小 pin 标记。
SMALL_MODULE_EDGE_UM = 2.0
# 小器件 pin 间距小于该值时视为簇，统一做外置标注。
PIN_CLUSTER_UM = 0.25
# 未连线 / 已连线 pin 的标记颜色（偏浅，避免压过加粗走线）。
PIN_COLOR_IDLE = "#F5F5F5"
PIN_COLOR_CONNECTED = "#EF9A9A"
PIN_EDGE_IDLE = "#BDBDBD"
PIN_EDGE_CONNECTED = "#E57373"
# 实际线宽常为 0.05μm，大画布上几乎看不见；渲染时抬到该下限（刻意加粗便于看路径）。
MIN_VISUAL_WIRE_UM = 0.22
# pin 标记整体透明度：越低越浅。
PIN_MARKER_ALPHA = 0.35
PIN_LABEL_ALPHA = 0.55
# 判定同点换层（via）的坐标容差。
VIA_MATCH_UM = 0.08
VIA_COLOR = "#212121"
# 最终选用 LCP 的标记颜色（与 btree 图中 LCP 点风格接近）。
LCP_FACE_COLOR = "#EDE7F6"
LCP_EDGE_COLOR = "#4527A0"

# MOS 常用 pin 的优先方位：上/右/下/左。
PIN_CARDINAL_PREFERENCE = {
    "G": "N",
    "GATE": "N",
    "PLUS": "N",
    "D": "E",
    "DRAIN": "E",
    "S": "W",
    "SOURCE": "W",
    "B": "S",
    "BULK": "S",
    "BODY": "S",
    "MINUS": "S",
}

# 上下左右四个方位的单位向量与文本对齐。
CARDINAL_LAYOUT = {
    "N": (0.0, 1.0, "center", "bottom"),
    "E": (1.0, 0.0, "left", "center"),
    "S": (0.0, -1.0, "center", "top"),
    "W": (-1.0, 0.0, "right", "center"),
}
CARDINAL_ORDER = ("N", "E", "S", "W")


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


@dataclass
# 表示最终选用的 LCP 物理位置，供 layout 图标注。
class SelectedLcp:
    net: str
    lcp_id: str
    candidate_id: str
    x: float
    y: float


DEV_COLORS = {
    "nmos": "#2196F3",
    "pmos": "#F44336",
    "capacitor": "#4CAF50",
    "resistor": "#FF9800",
    "unknown": "#999999",
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


# 从 nets.txt 读取 net -> 端子集合（module.pin）。
def load_net_terminals(input_dir: Path) -> dict[str, set[str]]:
    terminals: dict[str, set[str]] = {}
    nets_path = input_dir / "nets.txt"
    if not nets_path.exists():
        return terminals
    for fields, _comment in read_rows(nets_path):
        if len(fields) < 3:
            continue
        net = fields[0]
        pins = {terminal for terminal in fields[2:] if "." in terminal}
        if pins:
            terminals[net] = pins
    return terminals


# 点到线段的最短距离（μm）。
def point_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x2 - x1
    dy = y2 - y1
    length2 = dx * dx + dy * dy
    if length2 < 1e-18:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


# 根据当前图中的走线几何，判断哪些 pin 在本层/本图被真正接到。
def active_pins_for_routes(
    routes: list[Route],
    pins: list[Pin],
    modules: dict[str, Module],
    placements: dict[str, Placement],
    net_terminals: dict[str, set[str]],
    threshold_um: float = 0.8,
) -> set[str]:
    routes_by_net: dict[str, list[Route]] = {}
    for route in routes:
        routes_by_net.setdefault(route.net, []).append(route)

    pin_to_nets: dict[str, list[str]] = {}
    for net, terminals in net_terminals.items():
        for terminal in terminals:
            pin_to_nets.setdefault(terminal, []).append(net)

    active: set[str] = set()
    for pin in pins:
        key = f"{pin.module_id}.{pin.name}"
        nets = pin_to_nets.get(key)
        if not nets:
            continue
        module = modules.get(pin.module_id)
        placement = placements.get(pin.module_id)
        if module is None or placement is None:
            continue
        gx, gy = transform_point(pin.x, pin.y, module, placement)
        for net in nets:
            for route in routes_by_net.get(net, []):
                if point_segment_distance(gx, gy, route.x1, route.y1, route.x2, route.y2) <= threshold_um:
                    active.add(key)
                    break
            if key in active:
                break
    return active


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


# 将走线中心线段按可视化线宽扩展为渲染多边形。
def route_polygon(route: Route, visual_width: float | None = None) -> list[tuple[float, float]]:
    half_width = max(visual_width if visual_width is not None else route.width, 0.001) / 2.0
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


# 判断点是否落在线段中心线上（含端点），用于识别 T 接换层 via。
def point_on_route_centerline(x: float, y: float, route: Route, tol: float = VIA_MATCH_UM) -> bool:
    dx = route.x2 - route.x1
    dy = route.y2 - route.y1
    length = math.hypot(dx, dy)
    if length < 1e-12:
        return math.hypot(x - route.x1, y - route.y1) <= tol
    t = ((x - route.x1) * dx + (y - route.y1) * dy) / (length * length)
    if t < -1e-9 or t > 1.0 + 1e-9:
        return False
    t = min(1.0, max(0.0, t))
    proj_x = route.x1 + t * dx
    proj_y = route.y1 + t * dy
    return math.hypot(x - proj_x, y - proj_y) <= tol


# 找出换层 via 点：同网异层端点重合，或一端点落在另一层同网线段中段（T 接）。
def find_via_points(all_routes: list[Route], layer: str | None = None) -> list[tuple[float, float, str]]:
    scale = 1.0 / max(VIA_MATCH_UM, 1e-6)
    layered: dict[tuple[str, int, int], set[str]] = {}
    for route in all_routes:
        for x, y in ((route.x1, route.y1), (route.x2, route.y2)):
            key = (route.net, int(round(x * scale)), int(round(y * scale)))
            layered.setdefault(key, set()).add(route.layer)

    vias: list[tuple[float, float, str]] = []
    seen: set[tuple[str, int, int]] = set()

    def add_via(net: str, x: float, y: float) -> None:
        key = (net, int(round(x * scale)), int(round(y * scale)))
        if key in seen:
            return
        seen.add(key)
        vias.append((x, y, net))

    focus_routes = [route for route in all_routes if layer is None or route.layer == layer]
    other_routes = [route for route in all_routes if layer is None or route.layer != layer]

    for route in focus_routes:
        for x, y in ((route.x1, route.y1), (route.x2, route.y2)):
            key = (route.net, int(round(x * scale)), int(round(y * scale)))
            layers = layered.get(key, set())
            if len(layers) >= 2:
                add_via(route.net, x, y)
                continue
            for other in other_routes:
                if other.net != route.net:
                    continue
                if point_on_route_centerline(x, y, other):
                    add_via(route.net, x, y)
                    break

    # 反向 T 接：其他层端点落在本层线段中段时，本层分图也要标 via。
    if layer is not None:
        for other in other_routes:
            for x, y in ((other.x1, other.y1), (other.x2, other.y2)):
                for route in focus_routes:
                    if route.net != other.net:
                        continue
                    if point_on_route_centerline(x, y, route):
                        add_via(other.net, x, y)
                        break
    return vias


# 从 routing_debug.json 读取最终选用的 LCP 物理坐标；文件缺失时返回空列表。
def load_selected_lcps(output_dir: Path) -> list[SelectedLcp]:
    debug_path = output_dir / "routing_debug.json"
    if not debug_path.is_file():
        return []
    try:
        data = json.loads(debug_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []

    location_by_id: dict[str, SelectedLcp] = {}
    for topology in data.get("lcp_topologies", []):
        net = str(topology.get("net", ""))
        for point in topology.get("linking_points", []):
            lcp_id = str(point.get("id", ""))
            for candidate in point.get("location_candidates", []):
                candidate_id = str(candidate.get("id", ""))
                if not candidate_id:
                    continue
                try:
                    x = float(candidate["x"])
                    y = float(candidate["y"])
                except (KeyError, TypeError, ValueError):
                    continue
                location_by_id[candidate_id] = SelectedLcp(
                    net=net,
                    lcp_id=lcp_id or candidate_id,
                    candidate_id=candidate_id,
                    x=x,
                    y=y,
                )

    selected: dict[str, SelectedLcp] = {}

    def remember(candidate_id: str) -> None:
        marker = location_by_id.get(candidate_id)
        if marker is not None:
            selected[candidate_id] = marker

    # 优先 detailed_traces：反映 detailed legalize / A* fallback 后的最终绑定。
    for trace in data.get("detailed_traces", []):
        for segment in trace.get("segments", []):
            candidate_id = str(segment.get("lcp_candidate_id", "") or "")
            if candidate_id:
                remember(candidate_id)

    # 回退到 DP traceback，兼容没有 detailed_traces 的旧输出。
    if not selected:
        for candidate in data.get("dp_traceback_candidates", []):
            for key in ("lcp_candidate_id", "target_lcp_candidate_id", "source_lcp_candidate_id"):
                candidate_id = str(candidate.get(key, "") or "")
                if candidate_id:
                    remember(candidate_id)

    return sorted(selected.values(), key=lambda item: (item.net, item.lcp_id, item.candidate_id))


# 输出目录名为空时提供稳定的默认 PNG 前缀。
def default_render_name(output_dir: Path) -> str:
    name = output_dir.name.strip()
    return name if name else "layout"


# 判断器件是否过小，需要外置标签并缩小 pin 标记。
def is_small_module(module: Module) -> bool:
    return min(module.width, module.height) < SMALL_MODULE_EDGE_UM


# 金属层排序：M1 < M2 < ...，未知层名排在后面按字典序。
def sort_layers(layers: list[str]) -> list[str]:
    def key(layer: str) -> tuple[int, str]:
        match = re.fullmatch(r"M(\d+)", layer.upper())
        if match:
            return (0, f"{int(match.group(1)):02d}")
        return (1, layer)

    return sorted(dict.fromkeys(layers), key=key)


# 小器件 pin 标记尺寸：尽量小，避免盖住走线端点。
def pin_marker_style(module: Module) -> tuple[str, float, float]:
    if is_small_module(module):
        return "o", 1.1, 0.35
    return "x", 2.8, 0.9


# 在器件中心或框外绘制模块名；小器件外置并画短引线。
def draw_module_label(
    ax: Any,
    module: Module,
    corners: list[tuple[float, float]],
) -> None:
    xs = [x for x, _y in corners]
    ys = [y for _x, y in corners]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    if not is_small_module(module):
        ax.text(cx, cy, module.module_id, ha="center", va="center", fontsize=5, fontweight="bold", color="#111111", zorder=10)
        return
    # 小器件：名字放到左上外侧，不画引线，避免和 pin 标注混淆。
    label_x = min(xs) - 1.2
    label_y = max(ys) + 1.2
    ax.text(
        label_x,
        label_y,
        module.module_id,
        ha="right",
        va="bottom",
        fontsize=5,
        fontweight="bold",
        color="#111111",
        zorder=10,
        bbox={"boxstyle": "round,pad=0.15", "facecolor": "white", "edgecolor": "#BDBDBD", "linewidth": 0.5, "alpha": 0.9},
    )


# 判断该器件的 pin 是否需要外置标注（小器件或 pin 簇过密）。
def needs_pin_callouts(module: Module, pin_points: list[tuple[float, float]]) -> bool:
    if is_small_module(module):
        return True
    if len(pin_points) <= 1:
        return False
    for index, (x1, y1) in enumerate(pin_points):
        for x2, y2 in pin_points[index + 1 :]:
            if math.hypot(x1 - x2, y1 - y2) < PIN_CLUSTER_UM:
                return True
    return False


# 按 pin 名偏好与相对位置，把标签分配到上/右/下/左。
def assign_cardinal_sides(pins: list[Pin], pin_points: list[tuple[float, float]], center: tuple[float, float]) -> list[str]:
    cx, cy = center
    count = len(pins)
    sides: list[str | None] = [None] * count
    used: set[str] = set()

    # 先按常用 pin 名占位（G 上、D 右、S 左、B 下）。
    for index, pin in enumerate(pins):
        preferred = PIN_CARDINAL_PREFERENCE.get(pin.name.upper())
        if preferred is not None and preferred not in used:
            sides[index] = preferred
            used.add(preferred)

    # 剩余 pin：按相对中心方位角就近落到空闲方位。
    remaining = [index for index, side in enumerate(sides) if side is None]
    free = [side for side in CARDINAL_ORDER if side not in used]
    if remaining and free:
        def angle_of(index: int) -> float:
            gx, gy = pin_points[index]
            return math.atan2(gy - cy, gx - cx)

        side_angle = {"N": math.pi / 2, "E": 0.0, "S": -math.pi / 2, "W": math.pi}
        for index in sorted(remaining, key=angle_of):
            if not free:
                # 方位用尽时按顺序循环复用，并在半径上错开。
                free = list(CARDINAL_ORDER)
            pin_angle = angle_of(index)

            def angle_distance(side: str) -> float:
                delta = abs(pin_angle - side_angle[side])
                return min(delta, 2 * math.pi - delta)

            chosen = min(free, key=angle_distance)
            sides[index] = chosen
            free.remove(chosen)

    return [side if side is not None else "E" for side in sides]


# 根据方位生成标签相对器件中心的偏移。
def cardinal_label_offset(side: str, radius: float, slot_index: int = 0) -> tuple[float, float, str, str]:
    dx, dy, ha, va = CARDINAL_LAYOUT[side]
    # 同一方位多个 pin 时沿切向微移，避免文字重叠。
    tangent_x, tangent_y = -dy, dx
    shift = 0.55 * slot_index
    return (radius * dx + shift * tangent_x, radius * dy + shift * tangent_y, ha, va)


# 绘制单个器件的全部 pin：大器件就地标注，小器件/密 pin 上下左右写名字（无引线）；仅连线 pin 换色。
def draw_module_pins(
    ax: Any,
    module: Module,
    placement: Placement,
    corners: list[tuple[float, float]],
    pins: list[Pin],
    show_labels: bool,
    connected_pins: set[str],
) -> None:
    if not pins:
        return
    pin_points = [transform_point(pin.x, pin.y, module, placement) for pin in pins]
    marker, markersize, markeredgewidth = pin_marker_style(module)
    use_callouts = show_labels and needs_pin_callouts(module, pin_points)

    xs = [x for x, _y in corners]
    ys = [y for _x, y in corners]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    bbox_diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    # 标签紧贴器件外侧；标记始终画在真实 pin 坐标，保证与走线端点对齐。
    radius = max(0.85, bbox_diag * 1.15, 0.55 * len(pins))

    sides = assign_cardinal_sides(pins, pin_points, (cx, cy)) if use_callouts else ["E"] * len(pins)
    side_slots: dict[str, int] = {}
    label_specs: list[tuple[float, float, str, str]] = []
    for side in sides:
        slot = side_slots.get(side, 0)
        side_slots[side] = slot + 1
        label_specs.append(cardinal_label_offset(side, radius, slot))

    # 密 pin 真实坐标几乎重合（常 <0.1μm），大画布上会叠成一个点。
    # 标记做十字微偏只为“数得清有几个”；走线仍接到真实坐标（用 active 时画在真点上）。
    display_points = list(pin_points)
    clustered = needs_pin_callouts(module, pin_points) and len(pins) > 1
    if clustered:
        jitter = max(0.22, min(module.width, module.height) * 0.35)
        for index, (gx, gy) in enumerate(pin_points):
            dx, dy, _ha, _va = CARDINAL_LAYOUT[sides[index]]
            display_points[index] = (gx + jitter * dx, gy + jitter * dy)

    for pin, (gx, gy), (dx, dy), (ox, oy, ha, va) in zip(pins, pin_points, display_points, label_specs):
        pin_key = f"{pin.module_id}.{pin.name}"
        connected = pin_key in connected_pins
        small = is_small_module(module)
        if connected:
            face = PIN_COLOR_CONNECTED
            edge = PIN_EDGE_CONNECTED
            size = markersize + (0.15 if small else 0.4)
            width = markeredgewidth + (0.1 if small else 0.2)
            # 小器件 pin 标记压在走线之下，只靠外侧文字辨认，避免挡住 routing。
            order = 2 if small else 12
            # 本图真正接到的 pin：画在真实坐标，保证和走线端点对齐。
            mx, my = gx, gy
        else:
            face = PIN_COLOR_IDLE
            edge = PIN_EDGE_IDLE
            size = markersize
            width = markeredgewidth
            order = 2 if small else 11
            # 未接到的密 pin：微偏显示，避免 4 个名字对应 1 个点。
            mx, my = (dx, dy) if clustered else (gx, gy)
        ax.plot(
            mx,
            my,
            marker=marker,
            color=edge,
            markersize=size,
            markeredgewidth=width,
            markerfacecolor=face if marker == "o" else "none",
            alpha=PIN_MARKER_ALPHA,
            zorder=order,
        )
        if not show_labels:
            continue
        if use_callouts:
            # 名字写在上下左右，标记仍在真实坐标。
            label_x = cx + ox
            label_y = cy + oy
            ax.text(
                label_x,
                label_y,
                pin.name,
                ha=ha,
                va=va,
                fontsize=5.5,
                fontweight="bold",
                color="#C62828" if connected else "#9E9E9E",
                zorder=12,
                alpha=PIN_LABEL_ALPHA,
                bbox={
                    "boxstyle": "round,pad=0.12",
                    "facecolor": "#FFEBEE" if connected else "white",
                    "edgecolor": edge if connected else "#E0E0E0",
                    "linewidth": 0.5 if connected else 0.3,
                    "alpha": 0.55,
                },
            )
        else:
            ax.annotate(
                pin.name,
                (gx, gy),
                textcoords="offset points",
                xytext=(3, 3),
                fontsize=4,
                color="#C62828" if connected else "#BDBDBD",
                alpha=PIN_LABEL_ALPHA,
                zorder=12,
            )


# 计算合图/分图共用的坐标范围。
def compute_bounds(
    placed_modules: list[tuple[Module, Placement, list[tuple[float, float]]]],
    routes: list[Route],
    lcps: list[SelectedLcp] | None = None,
) -> tuple[tuple[float, float], tuple[float, float]]:
    all_x: list[float] = []
    all_y: list[float] = []
    for _module, _placement, corners in placed_modules:
        all_x.extend(x for x, _y in corners)
        all_y.extend(y for _x, y in corners)
    for route in routes:
        half_width = max(route.width, 0.001) / 2.0
        all_x.extend([route.x1 - half_width, route.x1 + half_width, route.x2 - half_width, route.x2 + half_width])
        all_y.extend([route.y1 - half_width, route.y1 + half_width, route.y2 - half_width, route.y2 + half_width])
    for lcp in lcps or []:
        all_x.append(lcp.x)
        all_y.append(lcp.y)
    if not all_x:
        all_x = [0.0, 100.0]
        all_y = [0.0, 100.0]
    span_x = max(all_x) - min(all_x)
    span_y = max(all_y) - min(all_y)
    margin = max(span_x, span_y, 1.0) * 0.08
    xlim = (min(all_x) - margin, max(all_x) + margin)
    ylim = (min(all_y) - margin, max(all_y) + margin)
    if any(is_small_module(module) for module, _placement, _corners in placed_modules):
        # 小器件模块名 + 上下左右 pin 外置标注需要更大边距。
        margin_extra = max(span_x, span_y, 1.0) * 0.16
        xlim = (xlim[0] - margin_extra, xlim[1] + margin_extra)
        ylim = (ylim[0] - margin_extra, ylim[1] + margin_extra)
    return xlim, ylim


# 在布局图上标注最终选用的 LCP 物理位置（仅形状，不写文字）。
def draw_selected_lcps(
    ax: Any,
    lcps: list[SelectedLcp],
    colors_by_net: dict[str, str],
) -> None:
    for lcp in lcps:
        edge = colors_by_net.get(lcp.net, LCP_EDGE_COLOR)
        ax.plot(
            lcp.x,
            lcp.y,
            marker="D",
            markersize=3.8,
            markerfacecolor=LCP_FACE_COLOR,
            markeredgecolor=edge,
            markeredgewidth=1.0,
            zorder=6,
        )


# 在给定 axes 上绘制一层或全部层的布局内容。
def draw_layout_axes(
    ax: Any,
    mpatches: Any,
    placed_modules: list[tuple[Module, Placement, list[tuple[float, float]]]],
    pin_by_module: dict[str, list[Pin]],
    routes: list[Route],
    all_routes: list[Route],
    colors_by_net: dict[str, str],
    connected_pins: set[str],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    title: str,
    show_labels: bool,
    show_pins: bool,
    focus_layer: str | None,
    lcps: list[SelectedLcp] | None = None,
) -> None:
    ax.set_aspect("equal")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.grid(True, color="#EEEEEE", linewidth=0.5, alpha=0.7)

    sorted_routes = sorted(routes, key=lambda route: -max(abs(route.x2 - route.x1), abs(route.y2 - route.y1)))
    for route in sorted_routes[:5000]:
        # 合图与分图层统一按 net 上色，避免同层异网糊成一种颜色。
        color = colors_by_net.get(route.net, "#999999")
        visual_width = max(route.width, MIN_VISUAL_WIRE_UM)
        patch = mpatches.Polygon(
            route_polygon(route, visual_width=visual_width),
            closed=True,
            facecolor=color,
            edgecolor="none",
            alpha=0.7,
            zorder=5,
        )
        ax.add_patch(patch)

    # 分图层时标出 via：本层线段在此换到其他层，避免被误认为断线或多余 pin。
    if focus_layer is not None:
        vias = find_via_points(all_routes, layer=focus_layer)
        for x, y, _net in vias:
            ax.plot(
                x,
                y,
                marker="s",
                markersize=5.0,
                markerfacecolor="#FFF9C4",
                markeredgecolor=VIA_COLOR,
                markeredgewidth=1.1,
                zorder=13,
            )
            ax.text(
                x + 0.35,
                y + 0.35,
                "via",
                fontsize=5,
                color=VIA_COLOR,
                ha="left",
                va="bottom",
                zorder=14,
                bbox={"boxstyle": "round,pad=0.1", "facecolor": "#FFF9C4", "edgecolor": VIA_COLOR, "linewidth": 0.5, "alpha": 0.95},
            )

    for module, placement, corners in placed_modules:
        color = DEV_COLORS.get(module.device_type, DEV_COLORS["unknown"])
        patch = mpatches.Polygon(corners, closed=True, facecolor=color, edgecolor=color, linewidth=1.0, alpha=0.35, zorder=3)
        ax.add_patch(patch)
        draw_module_label(ax, module, corners)
        if show_pins:
            draw_module_pins(
                ax,
                module,
                placement,
                corners,
                pin_by_module.get(module.module_id, []),
                show_labels,
                connected_pins,
            )

    if lcps:
        draw_selected_lcps(ax, lcps, colors_by_net)

    legend: list[Any] = []
    for device_type in sorted({module.device_type for module, _placement, _corners in placed_modules}):
        color = DEV_COLORS.get(device_type, DEV_COLORS["unknown"])
        legend.append(mpatches.Patch(facecolor=color, edgecolor=color, alpha=0.35, label=device_type))
    # 图例始终按本图出现的 net 列出颜色。
    nets_in_view = list(dict.fromkeys(route.net for route in routes))
    for net in nets_in_view:
        legend.append(mpatches.Patch(color=colors_by_net.get(net, "#999999"), label=f"net {net}"))
    if focus_layer is not None:
        legend.append(mpatches.Patch(facecolor="white", edgecolor=VIA_COLOR, label="via (to other layer)"))
    if show_pins:
        legend.append(mpatches.Patch(facecolor=PIN_COLOR_CONNECTED, edgecolor=PIN_EDGE_CONNECTED, label="active pin (this view)"))
        legend.append(mpatches.Patch(facecolor=PIN_COLOR_IDLE, edgecolor=PIN_EDGE_IDLE, label="other pin"))
    if lcps:
        legend.append(
            mpatches.Patch(facecolor=LCP_FACE_COLOR, edgecolor=LCP_EDGE_COLOR, label="LCP (selected)")
        )
    if legend:
        ax.legend(handles=legend, loc="upper right", fontsize=7, framealpha=0.8)

    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_xlabel("X (um)")
    ax.set_ylabel("Y (um)")
    ax.tick_params(labelsize=7)


# 渲染一张布局图并保存。
def save_layout_figure(
    placed_modules: list[tuple[Module, Placement, list[tuple[float, float]]]],
    pin_by_module: dict[str, list[Pin]],
    routes: list[Route],
    all_routes: list[Route],
    colors_by_net: dict[str, str],
    connected_pins: set[str],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    out_path: Path,
    title: str,
    dpi: int,
    show_labels: bool,
    show_pins: bool,
    focus_layer: str | None,
    lcps: list[SelectedLcp] | None = None,
) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt

    fig_w = max(8.0, min(24.0, (xlim[1] - xlim[0]) / 20.0))
    fig_h = max(8.0, min(24.0, (ylim[1] - ylim[0]) / 20.0))
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    draw_layout_axes(
        ax=ax,
        mpatches=mpatches,
        placed_modules=placed_modules,
        pin_by_module=pin_by_module,
        routes=routes,
        all_routes=all_routes,
        colors_by_net=colors_by_net,
        connected_pins=connected_pins,
        xlim=xlim,
        ylim=ylim,
        title=title,
        show_labels=show_labels,
        show_pins=show_pins,
        focus_layer=focus_layer,
        lcps=lcps,
    )
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path


# 渲染完整布局：始终生成合图；若存在多层走线，再为每层各生成一张。
def render_layout(
    input_dir: Path,
    output_dir: Path,
    render_name: str,
    dpi: int,
    show_labels: bool,
    show_pins: bool,
) -> list[Path]:
    modules = load_modules(input_dir)
    pins = load_pins(input_dir)
    net_terminals = load_net_terminals(input_dir)
    placements = load_placements(output_dir)
    routes = load_routes(output_dir)
    lcps = load_selected_lcps(output_dir)
    colors_by_net = net_color_map([route.net for route in routes] + [lcp.net for lcp in lcps])

    pin_by_module: dict[str, list[Pin]] = {}
    for pin in pins:
        pin_by_module.setdefault(pin.module_id, []).append(pin)

    placed_modules: list[tuple[Module, Placement, list[tuple[float, float]]]] = []
    for module_id, module in modules.items():
        placement = placements.get(module_id)
        if placement is None:
            continue
        corners = module_corners(module, placement)
        placed_modules.append((module, placement, corners))

    xlim, ylim = compute_bounds(placed_modules, routes, lcps)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 合图：凡在任意层接到的 pin 都标红；分图层：只标本层端点附近的 pin。
    active_all = active_pins_for_routes(routes, pins, modules, placements, net_terminals)

    out_paths: list[Path] = []
    combined_path = output_dir / f"{render_name}_layout.png"
    save_layout_figure(
        placed_modules=placed_modules,
        pin_by_module=pin_by_module,
        routes=routes,
        all_routes=routes,
        colors_by_net=colors_by_net,
        connected_pins=active_all,
        xlim=xlim,
        ylim=ylim,
        out_path=combined_path,
        title=f"{render_name} - {len(placed_modules)} modules (all layers)",
        dpi=dpi,
        show_labels=show_labels,
        show_pins=show_pins,
        focus_layer=None,
        lcps=lcps,
    )
    out_paths.append(combined_path)

    layers = sort_layers([route.layer for route in routes])
    # 仅当实际出现多层时才额外导出分图层，避免单层结果重复出图。
    if len(layers) >= 2:
        for layer in layers:
            layer_routes = [route for route in routes if route.layer == layer]
            active_layer = active_pins_for_routes(layer_routes, pins, modules, placements, net_terminals)
            via_count = len(find_via_points(routes, layer=layer))
            layer_path = output_dir / f"{render_name}_layout_{layer}.png"
            save_layout_figure(
                placed_modules=placed_modules,
                pin_by_module=pin_by_module,
                routes=layer_routes,
                all_routes=routes,
                colors_by_net=colors_by_net,
                connected_pins=active_layer,
                xlim=xlim,
                ylim=ylim,
                out_path=layer_path,
                title=f"{render_name} - layer {layer} ({len(layer_routes)} segs, {via_count} vias)",
                dpi=dpi,
                show_labels=show_labels,
                show_pins=show_pins,
                focus_layer=layer,
                lcps=lcps,
            )
            out_paths.append(layer_path)

    return out_paths


# 解析命令行参数并执行渲染。
def main() -> int:
    parser = argparse.ArgumentParser(description="Render SAPR input/output text files as layout PNG(s).")
    parser.add_argument("--input", required=True, type=Path, help="Directory containing modules.txt and pins.txt")
    parser.add_argument("--output", required=True, type=Path, help="Directory containing placement.txt and routing.txt")
    parser.add_argument("--name", default=None, help="PNG basename prefix; defaults to output directory name")
    parser.add_argument("--dpi", type=int, default=200, help="Output PNG DPI")
    parser.add_argument("--no-labels", action="store_true", help="Hide pin labels")
    parser.add_argument("--no-pins", action="store_true", help="Hide pin markers")
    args = parser.parse_args()

    render_name = args.name or default_render_name(args.output)
    out_paths = render_layout(
        input_dir=args.input,
        output_dir=args.output,
        render_name=render_name,
        dpi=args.dpi,
        show_labels=not args.no_labels,
        show_pins=not args.no_pins,
    )
    for out_path in out_paths:
        print(os.fspath(out_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
