# AnalogSAPR-Repro

论文 *Simultaneous Analog Placement and Routing with Current Flow and Current Density Considerations* 的 C++20 算法复现工程。当前版本已接入 placement-aware 联合求解流程：标准 I/O、约束校验、增强 B*-tree/ASF 对称组、contour packing、模拟退火搜索、A*/bottom-up DP routing、top-down detailed routing、routing feedback 动态预留布线资源和指标统计均已实现。历史 `solve_baseline` 接口目前保留名称，但默认指向同一套 placement-aware 求解流程。

## 当前算法流程

主流程由 `sapr run` 触发，整体顺序为：

```text
构造 enhanced B*-tree
→ contour packing 生成 placement candidate
→ A* 生成候选路径，bottom-up DP 选择一致 traceback
→ top-down detailed routing 输出最终线段
→ routing feedback 写回 space node 的资源需求
→ 在模拟退火中继续扰动 tree 并搜索更优解
```

其中增强 B*-tree 负责器件代表节点、对称组、right/top/group/cluster space node 和 LCP 拓扑；模拟退火会执行 module swap/delete-insert/rotate 以及 LCP delete-insert/swap/split/merge 等扰动。每个候选解内部会按 `routing_feedback_iterations` 做有限轮 routing feedback → re-pack 闭环，使详细布线阶段发现的线宽、耦合和空间需求反馈到下一轮 packing。

## 环境要求

项目使用 C++20 和 CMake 构建，不依赖 Boost、JSON 库或其他第三方运行库。

| 工具 | 最低要求 | 推荐/已验证版本 | 用途 |
|------|----------|-----------------|------|
| CMake | 3.20 | 4.3.3 | 配置项目和生成构建文件 |
| Ninja | 1.10 | Miniconda/MSYS2 附带版本 | 执行增量构建 |
| C++ 编译器 | 支持 C++20 | GCC 15.2.0（MinGW-w64） | 编译核心程序和测试 

Windows 推荐使用以下组合：

- CMake：推荐通过 `winget` 安装，安装程序会自动配置命令行入口：

  ```powershell
  winget install -e --id Kitware.CMake --source winget
  ```

  也可以从 [CMake 官网](https://cmake.org/download/)下载安装，安装时选择将 CMake 加入 `PATH`。
- Ninja：可通过 MSYS2、Miniconda 或独立安装获得。
- MinGW-w64 GCC：确保 `g++.exe` 所在目录已加入 `PATH`。

安装完成后新开一个 PowerShell，检查环境：

```powershell
cmake --version
ninja --version
g++ --version
```

若命令仍提示“无法识别”，关闭并重新打开终端，使新的 `PATH` 生效。

## 构建项目

在仓库根目录执行：

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

构建完成后，Windows 可执行文件位于：

```text
build/sapr.exe
build/sapr_tests.exe
```

重新构建时直接执行 `cmake --build build` 即可。需要完全重新配置时，可以删除 `build` 目录后再次运行上述命令。

## 运行测试

```powershell
ctest --test-dir build --output-on-failure
```

当前测试包括输入解析、约束校验、几何变换、增强 B*-tree 基础结构、contour packing、模拟退火、A*/DP routing、routing feedback、detailed routing、CLI 校验及端到端输出。

## 运行程序

验证输入文件：

```powershell
.\build\sapr.exe validate --input input
```

输入合法时输出：

```text
OK
```

运行 placement-aware 联合布局布线：

```powershell
.\build\sapr.exe run --input input --output output
```

指定器件间距和单行最大宽度：

```powershell
.\build\sapr.exe run `
  --input input `
  --output output `
  --spacing 5 `
  --row-width 40 `
  --sa-iterations 250 `
  --seed 1
```

配置芯片左、下边界预留：

```powershell
.\build\sapr.exe run --input input --output output --boundary-margin 1.5 --boundary-clearance 0.1
```

未指定 `--boundary-margin` 时，程序按最大线宽的一半、边界 clearance 和两倍有效网格步长自动估算；该参数独立于器件间 `--spacing`。

布线金属层数默认仅 `M1`（`--routing-layers 1`），避免高层逃逸掩盖 placement/拓扑问题。需要有限 via 或恢复旧多层行为时：

```powershell
.\build\sapr.exe run --input input --output output --routing-layers 3
.\build\sapr.exe run --input input --output output --routing-layers 7
```

需要查看 routing/packing 联合评估细节时，可以追加：

```powershell
.\build\sapr.exe run --input input --output output --dump-routing-eval
```

程序会生成：

- `output/placement.txt`：器件放置位置、角度和朝向。
- `output/routing.txt`：线网、金属层、中心线坐标和线宽。
- `output/*_layout.png`：全层合并的布局布线可视化图，默认每次运行都会生成。
- `output/*_layout_M*.png`：当 `routing.txt` 含多层时，额外按金属层各生成一张（仅该层走线 + 全部器件/pin）。
- `output/metrics.json`：基础指标与 `routing_evaluation` 摘要。
- `output/routing_debug.json`：routing/LCP/DP/detailed routing 诊断快照，默认每次运行都会生成。
- 标准输出：面积、线长、bend、via 和 penalty 指标。
- `--dump-routing-eval` 仅额外在标准输出打印：phi cost、routing cost、DP 状态数、packing trace 步数、packing-time DP segment 数、space feedback 节点数和 routing feedback 收敛信息。

## 不使用 CMake 的临时编译方式

仅用于快速排查编译环境，不建议作为日常构建方式：

```powershell
New-Item -ItemType Directory -Force build | Out-Null
g++ -std=c++20 -Wall -Wextra -Wpedantic -Iinclude `
  src/constraints.cpp src/geometry.cpp src/io.cpp `
  src/optimizer.cpp src/router.cpp src/routing_evaluator.cpp src/tree.cpp `
  src/routing/astar.cpp src/routing/dp_router.cpp `
  src/routing/geometry.cpp src/routing/global_router.cpp `
  src/routing/grid.cpp src/routing/layer.cpp `
  src/routing/obstacle.cpp src/routing/routing_context.cpp `
  src/routing/topology.cpp src/routing/transform.cpp `
  src/main.cpp `
  -o build/sapr.exe
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
    ├── routing.txt        # 走线中心线段 + 金属层 + 线宽
    ├── *_layout.png       # 全层合并布局布线图
    ├── *_layout_M*.png    # 多层时按层分图（可选）
    ├── metrics.json       # 指标与 routing_evaluation 摘要
    ├── routing_debug.json # 布线诊断详情
    └── routing_debug.json # 每次 run 自动生成的 routing 诊断快照
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
