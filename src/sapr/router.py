from __future__ import annotations

from collections import defaultdict

from .geometry import placed_pin
from .model import Circuit, Metrics, Placement, Priority, RouteSegment, Solution


def default_width(circuit: Circuit, net_name: str) -> float:
    constraint = circuit.constraints.wire_widths.get(net_name)
    if constraint:
        return (constraint.min_width + constraint.max_width) / 2.0
    return 1.0


def route_manhattan(circuit: Circuit, placements: dict[str, Placement]) -> list[RouteSegment]:
    routes: list[RouteSegment] = []
    nets = sorted(circuit.nets.values(), key=lambda n: _priority_order(n.priority))
    for net in nets:
        pins = [circuit.pins[t] for t in net.terminals if t in circuit.pins]
        if len(pins) < 2:
            continue
        root = pins[0]
        root_xy = placed_pin(circuit.modules[root.module], root, placements[root.module])
        width = default_width(circuit, net.name)
        for pin in pins[1:]:
            end_xy = placed_pin(circuit.modules[pin.module], pin, placements[pin.module])
            mid = (end_xy[0], root_xy[1])
            layer1 = root.layer
            layer2 = pin.layer if pin.layer != layer1 else layer1
            _append_nonzero(routes, RouteSegment(net.name, layer1, root_xy[0], root_xy[1], mid[0], mid[1], width))
            _append_nonzero(routes, RouteSegment(net.name, layer2, mid[0], mid[1], end_xy[0], end_xy[1], width))
    return routes


def measure(circuit: Circuit, solution: Solution) -> Metrics:
    max_x = 0.0
    max_y = 0.0
    for placement in solution.placements.values():
        module = circuit.modules[placement.module]
        max_x = max(max_x, placement.x + module.width)
        max_y = max(max_y, placement.y + module.height)
    wirelength = sum(abs(r.x2 - r.x1) + abs(r.y2 - r.y1) for r in solution.routes)
    bend_count = _count_bends(solution.routes)
    via_count = _count_vias(solution.routes)
    return Metrics(area=max_x * max_y, wirelength=wirelength, bend_count=bend_count, via_count=via_count, penalty=0.0)


def _priority_order(priority: Priority) -> int:
    return {Priority.CRITICAL: 0, Priority.SYMMETRY: 1, Priority.NORMAL: 2}[priority]


def _append_nonzero(routes: list[RouteSegment], segment: RouteSegment) -> None:
    if (segment.x1, segment.y1) != (segment.x2, segment.y2):
        routes.append(segment)


def _count_bends(routes: list[RouteSegment]) -> int:
    by_net: dict[str, list[RouteSegment]] = defaultdict(list)
    for route in routes:
        by_net[route.net].append(route)
    bends = 0
    for segments in by_net.values():
        for first, second in zip(segments, segments[1:]):
            same_point = (first.x2, first.y2) == (second.x1, second.y1)
            if same_point and first.layer == second.layer:
                bends += 1
    return bends


def _count_vias(routes: list[RouteSegment]) -> int:
    by_net: dict[str, list[RouteSegment]] = defaultdict(list)
    for route in routes:
        by_net[route.net].append(route)
    vias = 0
    for segments in by_net.values():
        for first, second in zip(segments, segments[1:]):
            same_point = (first.x2, first.y2) == (second.x1, second.y1)
            if same_point and first.layer != second.layer:
                vias += 1
    return vias
