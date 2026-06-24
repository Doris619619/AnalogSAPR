# AnalogSAPR-Repro

论文 *Simultaneous Analog Placement and Routing with Current Flow and Current Density Considerations* 的 C++20 算法复现工程。当前版本完成标准 I/O、约束校验、链式 baseline placement、Manhattan routing 和指标统计；增强 B*-tree、DP routing 与模拟退火将在后续阶段实现。

## 构建与运行

正式构建使用 CMake：

```powershell
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

当前 Windows 环境也可直接使用 MinGW：

```powershell
g++ -std=c++20 -Wall -Wextra -Wpedantic -Iinclude src/*.cpp -o build/sapr.exe
build/sapr.exe validate --input input
build/sapr.exe run --input input --output output
```

## IO 格式说明 v2

模拟电路 PnR 框架的标准输入输出格式。所有坐标单位 μm，空白字符分隔，`#` 开头为注释。

## 目录结构

```
IO/
├── input/
│   ├── modules.txt        # 器件 BB 尺寸、有源区、原点偏移
│   ├── pins.txt           # pin 坐标（相对器件）和金属层
│   ├── nets.txt           # 线网连接关系 + 优先级
│   └── constraints.txt    # 对称、电流流向、线宽约束
└── output/
    ├── placement.txt      # 器件放置位置 + 角度 + orient
    └── routing.txt        # 走线中心线段 + 金属层 + 线宽
```

---

## input/modules.txt

```
# id          w       h       active(x1,y1,x2,y2)         # info
M1           4.000   3.000   0.400,0.300,3.600,2.700  # nmos nch_ulvt_mac 4.0/1.0 ox=-0.300 oy=-0.210
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string | 器件名，全局唯一 |
| `w` | float | BB 宽度 (μm) |
| `h` | float | BB 高度 (μm) |
| `active` | `x1,y1,x2,y2` | 有源区矩形，相对 BB 左下角。布线禁止穿越 |
| `ox`, `oy` | float（注释中） | Cell 原点在 cell-local 坐标系中的偏移。GDSII cell bbox 不一定从 (0,0) 开始 |

BB 与 active 的关系：

```
   (0,h)                    (w,h)
     ┌──────────────────────────┐
     │      routing allowed     │  ← BB 边界留白区
     │   ┌──────────────────┐   │
     │   │  active region   │   │  ← 禁止布线
     │   └──────────────────┘   │
     │      routing allowed     │
     └──────────────────────────┘
   (0,0)                    (w,0)
```

---

## input/pins.txt

```
# module      pin      x        y        layer
M1            D        2.000    1.500    M2
M1            G        0.800    1.500    M1
M1            S        2.000    0.000    M1
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `module` | string | 所属器件 |
| `pin` | string | pin 名称（S/D/G/B/PLUS/MINUS） |
| `x` | float | 相对器件 BB 左下角的 x (μm) |
| `y` | float | 相对器件 BB 左下角的 y (μm) |
| `layer` | M1 \| M2 \| ... \| M7 | pin 所在金属层 |

- `module.pin` 组合全局唯一
- pin 坐标已在 GDSII 提取时减去 cell 原点偏移 (ox, oy)

---

## input/nets.txt

```
# net          priority    terminals
VDD            critical    M3.S M4.S
OUT            critical    M3.D M1.D
TAIL           symmetry    M1.S M2.S
IN_P           normal      M1.G
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `net` | string | 线网名 |
| `priority` | critical \| symmetry \| normal | 布线优先级 |
| `terminals` | string[] | 该网所有 `module.pin` |

优先级：
- **critical**：电源、输出等关键线网，详细布线阶段优先
- **symmetry**：对称线网，两半边需匹配长度
- **normal**：普通线网

---

## input/constraints.txt

```
SYMMETRY_PAIR    sg1   vertical   M1     M2
SYMMETRY_SELF    sg3   vertical   M5
FLOW             OUT   M3.D       M1.D
WIRE_WIDTH       VDD   2          8
```

### SYMMETRY_PAIR — 对称对

`name  axis  left  right`

left 器件完整定义，right 为其关于 axis（`vertical`/`horizontal`）的镜像，不进入 ASF-B*-tree。

### SYMMETRY_SELF — 自对称

`name  axis  module`

器件自身中心线即对称轴，在 ASF-B*-tree 中置于最右支。

### FLOW — 电流流向

`net  out_pin  in_pin`

同一 net 上电流从 out_pin 流向 in_pin。DP 阶段方向不对 → 高罚分。

### WIRE_WIDTH — 线宽

`net  min_w  max_w`

布线器在 [min_w, max_w] 范围内选择实际线宽 (μm)。

---

## output/placement.txt

```
# module       x           y           angle  orient
M1             5.00        10.00       0      R0
M2             5.00        20.00       0      MX
M3             15.00       10.00       180    R180
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `module` | string | 器件名 |
| `x` | float | Cell 原点在全局坐标系中的 x (μm) |
| `y` | float | Cell 原点在全局坐标系中的 y (μm) |
| `angle` | 0 \| 90 \| 180 \| 270 | 逆时针旋转角度 |
| `orient` | R0 \| R90 \| R180 \| R270 \| MX \| MY \| MXR90 \| MYR90 | Cadence orient 码 |

**orient 码**：

| 码 | 含义 | GDSII 变换 |
|----|------|-----------|
| R0 | 无变换 | — |
| R90 | 旋转 90° CCW | rot=90° |
| R180 | 旋转 180° | rot=180° |
| R270 | 旋转 270° CCW | rot=270° |
| MX | X 轴镜像 | rot=0°, x_reflection |
| MY | Y 轴镜像 | rot=180°, x_reflection |
| MXR90 | X 镜像 + 旋转 90° | rot=90°, x_reflection |
| MYR90 | Y 镜像 + 旋转 90° | rot=270°, x_reflection |

GDSII 变换顺序：**reflect X（如果有）→ rotate CCW → translate**。`angle` 是 pre-mirror 旋转角。

---

## output/routing.txt

```
# net         layer  x1       y1       x2       y2       width
VDD            M1     0.000    20.000   15.000   20.000   5.000
VDD            M2     15.000   20.000   15.000   40.000   5.000
OUT            M3     25.000   35.000   25.000   15.000   2.000
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `net` | string | 线网名 |
| `layer` | M1 \| M2 \| ... \| M7 | 该段所在金属层 |
| `x1,y1` | float | 起点 (μm) |
| `x2,y2` | float | 终点 (μm) |
| `width` | float | 线宽 (μm) |

- `(x1,y1)→(x2,y2)` 定义走线**中心线**，实际金属向两侧各扩 half width
- 相邻段同点不同层 → 隐式 via
- 相邻段同点同层 → bend
- 各金属层均可走任意方向，不硬限

---

## 解析规则

- `#` 开头的行为注释，跳过
- 数据列空白字符分隔，`split()` 直接解析
- 坐标：output 为绝对坐标，input/pins.txt 和 active 为相对器件 BB 左下角
- 单位统一 μm

## 版本历史

- **v2**：L1/L2 → M1~M7；placement 新增 `orient` 列；modules 新增 `ox`/`oy` 原点偏移；routing 含真实 GDSII 提取数据
- **v1**：两层金属抽象 L1/L2，无 orient，无原点偏移
