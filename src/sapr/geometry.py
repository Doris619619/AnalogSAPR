from __future__ import annotations

from .model import Module, Pin, Placement


def placed_pin(module: Module, pin: Pin, placement: Placement) -> tuple[float, float]:
    x, y = pin.x, pin.y
    angle = placement.angle % 360
    if angle == 0:
        tx, ty = x, y
    elif angle == 90:
        tx, ty = module.height - y, x
    elif angle == 180:
        tx, ty = module.width - x, module.height - y
    elif angle == 270:
        tx, ty = y, module.width - x
    else:
        raise ValueError(f"unsupported angle {placement.angle}")
    return placement.x + tx, placement.y + ty


def placed_size(module: Module, placement: Placement) -> tuple[float, float]:
    if placement.angle % 180 == 0:
        return module.width, module.height
    return module.height, module.width
