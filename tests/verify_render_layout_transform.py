# 验证 layout renderer 对全部 Cadence orient 的点变换与 C++ 几何定义一致。
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


# 从仓库工具目录加载 renderer，直接测试其公开的坐标变换函数。
def load_renderer():
    source_dir = Path(__file__).resolve().parents[1]
    renderer_path = source_dir / "tools" / "render_layout.py"
    spec = importlib.util.spec_from_file_location("render_layout", renderer_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load renderer: {renderer_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# 对非正方形模块验证八种 orient 的局部点全局坐标。
def main() -> int:
    renderer = load_renderer()
    module = renderer.Module("T", 4.0, 3.0, (0.0, 0.0, 4.0, 3.0), 0.0, 0.0, "unknown", "")
    expectations = (
        (0, "R0", (3.5, 5.75)),
        (90, "R90", (5.25, 5.5)),
        (180, "R180", (6.5, 7.25)),
        (270, "R270", (3.75, 8.5)),
        (0, "MX", (3.5, 7.25)),
        (0, "MY", (6.5, 5.75)),
        (90, "MXR90", (3.75, 5.5)),
        (270, "MYR90", (5.25, 8.5)),
    )
    for angle, orient, expected in expectations:
        placement = renderer.Placement("T", 3.0, 5.0, angle, orient)
        actual = renderer.transform_point(0.5, 0.75, module, placement)
        if actual != expected:
            raise AssertionError(f"{orient}: expected {expected}, got {actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
