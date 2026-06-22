from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WireSegmentRef:
    net: str
    min_width: float
    max_width: float
    direction: str | None = None


@dataclass
class LinkingControlPoint:
    id: str
    space_node_id: str
    segments: list[WireSegmentRef] = field(default_factory=list)

    @property
    def required_width(self) -> float:
        return max((segment.max_width for segment in self.segments), default=0.0)


@dataclass
class SpaceNode:
    id: str
    owner: str
    kind: str
    linking_points: list[LinkingControlPoint] = field(default_factory=list)

    def required_space(self) -> float:
        return sum(point.required_width * max(len(point.segments), 2) / 2.0 for point in self.linking_points)


@dataclass
class BStarNode:
    module: str
    left: str | None = None
    right: str | None = None
    angle: int = 0
    right_space: SpaceNode | None = None
    top_space: SpaceNode | None = None


@dataclass
class EnhancedBStarTree:
    root: str | None
    nodes: dict[str, BStarNode]

    @classmethod
    def chain(cls, modules: list[str]) -> "EnhancedBStarTree":
        nodes: dict[str, BStarNode] = {}
        for index, module in enumerate(modules):
            next_module = modules[index + 1] if index + 1 < len(modules) else None
            nodes[module] = BStarNode(
                module=module,
                left=next_module,
                right=None,
                right_space=SpaceNode(f"{module}:right", module, "right"),
                top_space=SpaceNode(f"{module}:top", module, "top"),
            )
        return cls(root=modules[0] if modules else None, nodes=nodes)
