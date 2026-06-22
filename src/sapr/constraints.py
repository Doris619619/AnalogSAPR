from __future__ import annotations

from .model import Circuit, Solution


def validate_circuit(circuit: Circuit) -> list[str]:
    errors: list[str] = []
    for pin in circuit.pins.values():
        if pin.module not in circuit.modules:
            errors.append(f"pin {pin.key} references missing module {pin.module}")

    for net in circuit.nets.values():
        for terminal in net.terminals:
            if terminal not in circuit.pins:
                errors.append(f"net {net.name} references missing pin {terminal}")

    for pair in circuit.constraints.symmetry_pairs:
        for module in (pair.left, pair.right):
            if module not in circuit.modules:
                errors.append(f"symmetry pair {pair.name} references missing module {module}")

    for item in circuit.constraints.symmetry_selfs:
        if item.module not in circuit.modules:
            errors.append(f"symmetry self {item.name} references missing module {item.module}")

    for flow in circuit.constraints.flows:
        if flow.net not in circuit.nets:
            errors.append(f"flow references missing net {flow.net}")
        if flow.out_pin not in circuit.pins:
            errors.append(f"flow {flow.net} references missing out pin {flow.out_pin}")
        if flow.in_pin not in circuit.pins:
            errors.append(f"flow {flow.net} references missing in pin {flow.in_pin}")
        if flow.net in circuit.nets:
            terminals = set(circuit.nets[flow.net].terminals)
            if flow.out_pin in circuit.pins and flow.out_pin not in terminals:
                errors.append(f"flow {flow.net} out pin {flow.out_pin} is not on the net")
            if flow.in_pin in circuit.pins and flow.in_pin not in terminals:
                errors.append(f"flow {flow.net} in pin {flow.in_pin} is not on the net")

    for net, width in circuit.constraints.wire_widths.items():
        if net not in circuit.nets:
            errors.append(f"wire width references missing net {net}")
        if width.min_width <= 0 or width.max_width < width.min_width:
            errors.append(f"wire width for {net} has invalid range [{width.min_width}, {width.max_width}]")

    return errors


def validate_solution(circuit: Circuit, solution: Solution) -> list[str]:
    errors: list[str] = []
    for module in circuit.modules:
        if module not in solution.placements:
            errors.append(f"missing placement for module {module}")
    for route in solution.routes:
        if route.net not in circuit.nets:
            errors.append(f"route references missing net {route.net}")
        width = circuit.constraints.wire_widths.get(route.net)
        if width and not (width.min_width <= route.width <= width.max_width):
            errors.append(f"route {route.net} width {route.width} violates [{width.min_width}, {width.max_width}]")
    return errors
