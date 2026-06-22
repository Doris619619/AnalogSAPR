from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable


class Priority(str, Enum):
    CRITICAL = "critical"
    SYMMETRY = "symmetry"
    NORMAL = "normal"


class Axis(str, Enum):
    VERTICAL = "vertical"
    HORIZONTAL = "horizontal"


@dataclass(frozen=True)
class Rect:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1


@dataclass(frozen=True)
class Module:
    id: str
    width: float
    height: float
    active: Rect
    ox: float = 0.0
    oy: float = 0.0
    info: str = ""


@dataclass(frozen=True)
class Pin:
    module: str
    name: str
    x: float
    y: float
    layer: str

    @property
    def key(self) -> str:
        return f"{self.module}.{self.name}"


@dataclass(frozen=True)
class Net:
    name: str
    priority: Priority
    terminals: tuple[str, ...]


@dataclass(frozen=True)
class SymmetryPair:
    name: str
    axis: Axis
    left: str
    right: str


@dataclass(frozen=True)
class SymmetrySelf:
    name: str
    axis: Axis
    module: str


@dataclass(frozen=True)
class FlowConstraint:
    net: str
    out_pin: str
    in_pin: str


@dataclass(frozen=True)
class WireWidthConstraint:
    net: str
    min_width: float
    max_width: float


@dataclass
class Constraints:
    symmetry_pairs: list[SymmetryPair] = field(default_factory=list)
    symmetry_selfs: list[SymmetrySelf] = field(default_factory=list)
    flows: list[FlowConstraint] = field(default_factory=list)
    wire_widths: dict[str, WireWidthConstraint] = field(default_factory=dict)


@dataclass
class Circuit:
    modules: dict[str, Module]
    pins: dict[str, Pin]
    nets: dict[str, Net]
    constraints: Constraints

    def pins_for_net(self, net_name: str) -> Iterable[Pin]:
        for terminal in self.nets[net_name].terminals:
            yield self.pins[terminal]


@dataclass(frozen=True)
class Placement:
    module: str
    x: float
    y: float
    angle: int = 0
    orient: str = "R0"


@dataclass(frozen=True)
class RouteSegment:
    net: str
    layer: str
    x1: float
    y1: float
    x2: float
    y2: float
    width: float


@dataclass
class Solution:
    placements: dict[str, Placement]
    routes: list[RouteSegment]


@dataclass(frozen=True)
class Metrics:
    area: float
    wirelength: float
    bend_count: int
    via_count: int
    penalty: float
