# 论文复现计划: Simultaneous Analog Placement and Routing with Current Flow and Current Density Considerations

## 1. 复现目标

复现论文提出的模拟电路同步布局布线算法。输入采用本仓库 `input/*.txt` 的标准格式，输出生成 `output/placement.txt` 与 `output/routing.txt`。第一阶段先完成可运行框架和基线结果，第二阶段逐步替换为论文算法核心。

论文核心目标函数:

```text
phi = alpha * A/A_hat + beta * W/W_hat + gamma * B/B_hat + delta * V/V_hat + p
```

其中 `A` 是芯片面积，`W` 是布线长度，`B` 是 bend 数，`V` 是 via 数，`p` 是不可布线或约束违反惩罚。

## 2. 算法模块拆解

1. 输入解析与校验
   - 解析 modules、pins、nets、constraints。
   - 校验 module/pin 引用、对称组、FLOW、电流密度线宽范围。
   - 所有坐标统一为 um，输出保留 3 位小数。

2. 增强 B*-tree 表示
   - Module node: 表示器件。
   - Space node: 每个节点的 right/top routing space。
   - Linking-control point: 表示类似 Steiner/bend point 的拓扑控制点。
   - ASF-B*-tree: 后续用于 symmetry pair/self 的自动对称可行布局。

3. 扰动与模拟退火
   - B*-tree 基础扰动: delete-insert、swap、rotate。
   - Linking-control point 扰动: delete-insert、swap、split、merge。
   - 接受准则: 标准 SA，代价采用论文总目标函数。

4. 动态布线资源分配
   - 对每个 space node 计算:

```text
R_i = sum(L_j * K_j / 2)
```

   - `L_j` 是 linking-control point 连接线段最大线宽，`K_j` 是连接线段数。
   - 用该空间扩展 B*-tree packing 时的 right/top spacing。

5. 同步 packing 与 routing
   - Bottom-up DP-based global routing:

```text
C_ij = alpha * W_ij + beta * B_ij + p
```

   - 用 A* 或网格 Manhattan 路径生成候选拓扑。
   - 常数时间检查 FLOW 方向，违反时加大惩罚。
   - Top-down detailed routing 按 symmetry/critical/normal 优先级回溯并写出线段。

## 3. 当前项目框架

当前代码先提供端到端骨架:

- `sapr.io`: 读写仓库 I/O 格式。
- `sapr.model`: 数据结构。
- `sapr.constraints`: 约束引用校验和结果指标校验。
- `sapr.tree`: 增强 B*-tree、space node、linking-control point 的接口。
- `sapr.optimizer`: 可替换的求解器接口和 baseline 求解器。
- `sapr.router`: 简单 Manhattan routing、wirelength/bend/via 指标。
- `sapr.cli`: `validate` 与 `run` 命令。

baseline 求解器不是论文最终算法，只用于验证 I/O、数据流和评价指标。

## 4. 实验与验收

最小验收:

```powershell
python -m sapr.cli validate --input input
python -m sapr.cli run --input input --output output
```

阶段性验收:

1. 解析器能发现所有缺失 module/pin 引用。
2. baseline 能生成语法正确的 placement/routing 文件。
3. 指标输出包含 area、wirelength、bend_count、via_count、penalty。
4. SA + enhanced B*-tree 版本在相同输入上优于 baseline 的总目标函数。
5. FLOW、WIRE_WIDTH、SYMMETRY 违反数降为 0。

## 5. 后续优先级

1. 实现 ASF-B*-tree 的 symmetry pair/self packing。
2. 实现 linking-control point 生成与四类扰动。
3. 实现 space node 资源预留并反馈到 packing。
4. 用 A* global routing 替换 baseline Manhattan routing。
5. 加入 benchmark 批处理和与论文 Table 2 指标一致的报告格式。
