from __future__ import annotations

from dataclasses import dataclass

from .constraints import validate_circuit
from .model import Circuit, Placement, Solution
from .router import route_manhattan
from .tree import EnhancedBStarTree


@dataclass(frozen=True)
class SolverConfig:
    spacing: float = 5.0
    row_width: float = 40.0
    seed: int = 1


class BaselineSolver:
    """Constructive baseline used to keep the reproduction framework executable."""

    def __init__(self, config: SolverConfig | None = None):
        self.config = config or SolverConfig()

    def solve(self, circuit: Circuit) -> Solution:
        errors = validate_circuit(circuit)
        if errors:
            raise ValueError("invalid circuit:\n" + "\n".join(f"- {error}" for error in errors))
        tree = EnhancedBStarTree.chain(list(circuit.modules))
        placements = self._pack_chain(circuit, tree)
        routes = route_manhattan(circuit, placements)
        return Solution(placements=placements, routes=routes)

    def _pack_chain(self, circuit: Circuit, tree: EnhancedBStarTree) -> dict[str, Placement]:
        placements: dict[str, Placement] = {}
        x = 0.0
        y = 0.0
        row_height = 0.0
        module_id = tree.root
        while module_id is not None:
            module = circuit.modules[module_id]
            if x > 0 and x + module.width > self.config.row_width:
                x = 0.0
                y += row_height + self.config.spacing
                row_height = 0.0
            placements[module_id] = Placement(module=module_id, x=x, y=y, angle=0, orient="R0")
            x += module.width + self.config.spacing
            row_height = max(row_height, module.height)
            module_id = tree.nodes[module_id].left
        return placements
