from __future__ import annotations

import re
from pathlib import Path

from .model import (
    Axis,
    Circuit,
    Constraints,
    FlowConstraint,
    Module,
    Net,
    Pin,
    Placement,
    Priority,
    Rect,
    RouteSegment,
    Solution,
    SymmetryPair,
    SymmetrySelf,
    WireWidthConstraint,
)

_OX_RE = re.compile(r"\box=([-+]?\d+(?:\.\d+)?)")
_OY_RE = re.compile(r"\boy=([-+]?\d+(?:\.\d+)?)")


def _data_lines(path: Path):
    for raw in path.read_text(encoding="utf-8").splitlines():
        body = raw.split("#", 1)[0].strip()
        comment = raw.split("#", 1)[1].strip() if "#" in raw else ""
        if body:
            yield body.split(), comment


def load_circuit(input_dir: str | Path) -> Circuit:
    root = Path(input_dir)
    modules = _load_modules(root / "modules.txt")
    pins = _load_pins(root / "pins.txt")
    nets = _load_nets(root / "nets.txt")
    constraints = _load_constraints(root / "constraints.txt")
    return Circuit(modules=modules, pins=pins, nets=nets, constraints=constraints)


def _load_modules(path: Path) -> dict[str, Module]:
    modules: dict[str, Module] = {}
    for parts, comment in _data_lines(path):
        if len(parts) < 4:
            raise ValueError(f"{path}: expected module row with 4 columns, got {parts}")
        x1, y1, x2, y2 = (float(v) for v in parts[3].split(","))
        ox = float(_OX_RE.search(comment).group(1)) if _OX_RE.search(comment) else 0.0
        oy = float(_OY_RE.search(comment).group(1)) if _OY_RE.search(comment) else 0.0
        modules[parts[0]] = Module(
            id=parts[0],
            width=float(parts[1]),
            height=float(parts[2]),
            active=Rect(x1, y1, x2, y2),
            ox=ox,
            oy=oy,
            info=comment,
        )
    return modules


def _load_pins(path: Path) -> dict[str, Pin]:
    pins: dict[str, Pin] = {}
    for parts, _ in _data_lines(path):
        if len(parts) != 5:
            raise ValueError(f"{path}: expected pin row with 5 columns, got {parts}")
        pin = Pin(module=parts[0], name=parts[1], x=float(parts[2]), y=float(parts[3]), layer=parts[4])
        pins[pin.key] = pin
    return pins


def _load_nets(path: Path) -> dict[str, Net]:
    nets: dict[str, Net] = {}
    for parts, _ in _data_lines(path):
        if len(parts) < 3:
            raise ValueError(f"{path}: expected net row with at least 3 columns, got {parts}")
        nets[parts[0]] = Net(parts[0], Priority(parts[1]), tuple(parts[2:]))
    return nets


def _load_constraints(path: Path) -> Constraints:
    constraints = Constraints()
    for parts, _ in _data_lines(path):
        kind = parts[0]
        if kind == "SYMMETRY_PAIR" and len(parts) == 5:
            constraints.symmetry_pairs.append(SymmetryPair(parts[1], Axis(parts[2]), parts[3], parts[4]))
        elif kind == "SYMMETRY_SELF" and len(parts) == 4:
            constraints.symmetry_selfs.append(SymmetrySelf(parts[1], Axis(parts[2]), parts[3]))
        elif kind == "FLOW" and len(parts) == 4:
            constraints.flows.append(FlowConstraint(parts[1], parts[2], parts[3]))
        elif kind == "WIRE_WIDTH" and len(parts) == 4:
            constraints.wire_widths[parts[1]] = WireWidthConstraint(parts[1], float(parts[2]), float(parts[3]))
        else:
            raise ValueError(f"{path}: unsupported constraint row: {parts}")
    return constraints


def write_solution(solution: Solution, output_dir: str | Path) -> None:
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    _write_placements(solution, root / "placement.txt")
    _write_routes(solution, root / "routing.txt")


def _write_placements(solution: Solution, path: Path) -> None:
    lines = [
        "# module       x           y           angle  orient",
        "",
    ]
    for placement in solution.placements.values():
        lines.append(
            f"{placement.module:<14} {placement.x:<11.3f} {placement.y:<11.3f} "
            f"{placement.angle:<6d} {placement.orient}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_routes(solution: Solution, path: Path) -> None:
    lines = [
        "# net         layer  x1       y1       x2       y2       width",
        "",
    ]
    for seg in solution.routes:
        lines.append(
            f"{seg.net:<12} {seg.layer:<5} {seg.x1:<8.3f} {seg.y1:<8.3f} "
            f"{seg.x2:<8.3f} {seg.y2:<8.3f} {seg.width:<8.3f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_solution(output_dir: str | Path) -> Solution:
    root = Path(output_dir)
    placements: dict[str, Placement] = {}
    for parts, _ in _data_lines(root / "placement.txt"):
        placements[parts[0]] = Placement(parts[0], float(parts[1]), float(parts[2]), int(parts[3]), parts[4])
    routes = []
    for parts, _ in _data_lines(root / "routing.txt"):
        routes.append(RouteSegment(parts[0], parts[1], *(float(v) for v in parts[2:])))
    return Solution(placements, routes)
