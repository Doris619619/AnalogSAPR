# LCP（Linking Control Point）当前逻辑

本文记录仓库中与 LCP 相关的**当前实现**，覆盖拓扑生成、space 归属、物理候选、SA 扰动、routing/DP、detailed 协商与 space feedback。对应代码以 `src/lcp_generator.cpp`、`src/tree.cpp`、`src/optimizer.cpp`、`src/routing_evaluator.cpp` 为准。

## 1. 概念与职责边界

| 概念 | 含义 |
|------|------|
| **LCP** | 多端 net 的拓扑枢纽（`LinkingControlPoint`），挂在某个 `SpaceNode` 上，用若干 `WireSegmentRef` 连接 pin 或其他 LCP |
| **Space 归属** | `space_node_id` / `SpaceNode.linking_points`，只由**初始绑定**与 **SA LCP 扰动**改变 |
| **物理候选** | `location_candidates`，随 packing/placement 刷新；不改变拓扑与 space 归属 |
| **NetTopology** | 按 net 汇总 LCP 与逻辑 segment，供 A*/DP/detailed 使用 |

**关键约束：**

- 初始 LCP 拓扑在 SA 开始前建立一次。
- 之后每次 packing 只 `refresh_lcp_location_candidates`，**不会**因几何变化自动重绑 space。
- 两 pin 及以下的 net **不生成** LCP，走普通 direct routing。

## 2. 核心数据结构

定义见 `include/sapr/model.hpp`：

- `LinkingControlPoint`：`id`、`space_node_id`、`segments`、`location_candidates`
- `PhysicalLocationCandidate`：坐标、`validity`（`strict` / `relaxed` / `emergency`）、`penalty`、`source`、`inside_space_region` 等
- `SpaceNode`：owner、kind（Right/Top/…）、挂载的 LCP、`physical_region`、`allocated_space`、`coupling_extra_space`
- `NetTopology`：某 net 的 pins / linking_points / segments
- `RoutingEvaluationRequest` 相关字段：
  - `lcp_candidate_seed`：候选随机采样种子（通常等于 `SolverConfig.seed`）
  - `strict_lcp_dp`：DP 失败时禁止含 LCP 的 net 退回 greedy
  - `allow_lcp_location_negotiation`：detailed 前是否协商整网单一 LCP 位置（默认开启）

合法 LCP 校验（`tree.cpp::is_valid_lcp`）：

- 非空 `id` / `space_node_id`
- 至少 2 条 segment，且同属一个 net
- 每条 segment 至少一端是该 LCP
- 无重复 segment id

## 3. 求解流程中的位置

```text
make_enhanced_tree
    → initialize_lcp_topology          # pack 一次几何 → 生成拓扑+绑定(+候选) → 写回 tree
    → SA 循环:
         perturb (含 4 类 LCP move)
         pack_enhanced_tree
         populate_routing_context      # 仅刷新候选 + 组装 NetTopology
         evaluate_routing / detailed
         space feedback 写回 tree
```

入口：

- `optimizer.cpp::initialize_lcp_topology` → `generate_initial_lcp_topology`
- `optimizer.cpp::populate_routing_context` → 有既有 LCP 时只 `refresh_lcp_location_candidates`
- 兼容入口 `generate_automatic_lcps`：有 LCP 则刷新候选，否则生成初始拓扑

## 4. 初始拓扑生成（`lcp_generator.cpp`）

对每个 net（按 `circuit.net_order`），若 pin 数 > 2：

### 4.1 Leaf 数量与聚类

- `target_leaf_count`：由 pin 数、几何长宽比、线宽共同决定；每个 leaf 最多挂 `kMaxPinsPerLeafLcp = 3` 个 pin
- 聚类：Prim MST + 删最长边；距离含 Manhattan 与 layer 惩罚
- 全 pin 重合时退化为按 terminal 顺序均匀分组
- `repair_large_clusters`：过大 cluster 继续切分

### 4.2 Leaf / Root 结构

- 每个 cluster → `net:leaf:i`
  - ideal 点 = cluster pin 坐标中位数
  - `support_bbox` = pin 点 + 所属 module bbox
  - pin–leaf segment：默认 pin→leaf；若 pin 是 FLOW `in_pin`，则 leaf→pin
- 多个 leaf 时额外建 `net:root`
  - ideal = leaf ideal 按 pin 数加权后的中位数
  - leaf–root 边按 FLOW source/sink 定向，保证 out→…→in 可追踪

线宽：来自 `WIRE_WIDTH` 的 min/max；无约束时默认 `1.0`。  
注意：自动拓扑阶段用 **min_width** 作为 `required_width`，避免默认 1μm 抬高 A* 障碍膨胀。

### 4.3 Space 绑定（仅初始）

绑定顺序：按 `required_width` 降序，再按 net / id 稳定排序。  
每个 LCP 选分数最低的 space：

```text
score = 估计星形线长
      + 溢出惩罚（剩余宽度不足时 +10000）
      + 占用惩罚（已有 LCP 数 × 线宽）
      + 方向惩罚（水平 net 绑 Top / 垂直 net 绑非 Top 各 +50）
```

- 有 placement 时：星形线长 = space 锚点到关联 pin 的 Manhattan 和
- 无 module_boxes 时：线长估计为 0（避免虚构几何影响归属）
- space 锚点：有 `physical_region` 用中心；否则按 Right/Top 贴 module 边

绑定后写入 `point.space_node_id`，并挂到对应 `SpaceNode.linking_points`。

### 4.4 物理候选生成

仅当 request 已有 `placements` 与 `placed_pins` 时生成。每个 LCP 最多保留 `kMaxCandidatesPerLcp = 16` 个，至少凑够 `kMinCandidatesPerLcp = 3`。

候选来源（摘要）：

1. **所属 space region 内**：clamped ideal、region 中心、sample 中心（strict）；边界中点/角点（relaxed）；1 个确定性随机点（seed 由全局 seed + LCP id + space id 派生）
2. **几何锚点**：median / bbox_center / space_projection（strict）；aligned_x/y、width_offset（relaxed）
3. **emergency**：相对 space 锚点的小偏移，用于凑够最小候选数

分类规则：

- 落在 space region 外：抬惩罚，并可能把 `strict` 降为 `relaxed` / `emergency`
- 按线宽扩张后碰到 `active_region_blockers`：标为 `emergency`，惩罚 +10000
- 最终按「到 ideal 的 Manhattan + penalty」排序截断

**刷新**（`refresh_lcp_location_candidates`）：从已有 segment 端点恢复 `WorkingLcp` 几何上下文，重新生成候选；**不改** `space_node_id` 与 segments。

## 5. SA 中的 LCP 扰动（`tree.cpp`）

有树内 LCP 时，扰动编号包含四类 LCP move（无 ASF 时 move∈[0,6]，有 ASF 时上限到 8）：

| move | 行为 |
|------|------|
| `lcp-delete-insert` | 把一个 LCP 移到另一个 concrete space |
| `lcp-swap` | 交换**不同 space** 上的两个 LCP；同 space 内换序不算有效 swap |
| `lcp-split` | segments≥4 时对半拆成两个 LCP，并改写端点引用 |
| `lcp-merge` | 合并同 net 的两个 LCP，保留全部 segment，折叠自环/重复边 |

扰动后校验 `has_valid_lcp_topology`；失败则该次 move 视为未改变（`changed=false`）。  
SA 会在同一温度轮内对不可执行 LCP 操作重采样，避免空耗迭代。

`PerturbationReport` 记录：`move`、`used_lcp_move`、`lcp_before` / `lcp_after`。

## 6. Routing 侧逻辑（`routing_evaluator.cpp`）

### 6.1 候选生成

1. 先生成所有 net 的 direct A* 候选
2. 对带 LCP topology 的 net：**删除**其 direct 候选，改走 `generate_lcp_route_candidates`
3. 对每条逻辑 segment：
   - pin–LCP：枚举该 LCP 的物理候选
   - LCP–LCP：枚举两端位置组合；先做低代价估计，再 top-K 截断，并**补齐两端 location 覆盖**（避免 multi-terminal root 被误剪）
4. `filter_multi_terminal_unreachable_lcp_candidates`：某个 LCP 物理点必须覆盖其全部 incident segment；否则该候选标记失败（`multi_terminal_missing`）

### 6.2 Bottom-up DP 与 fallback

- 有 `net_topologies` 且树有 root 时跑 `run_bottom_up_routing_dp`
- **DP 成功**：用 traceback 作为 global 选中路径
- **DP 失败**：
  - `strict_lcp_dp=true`：含 LCP 的 net **不**退回 direct greedy；`strict_lcp_dp_blocked_fallback=true`
  - 默认：回退到 direct 候选评估（`LCP fallback: no_multi_terminal_reachable_lcp_candidate`）

CLI：`--strict-lcp-dp` → `SolverConfig.strict_lcp_dp`。

### 6.3 Detailed 与位置协商

- 带 LCP 的 net 按整网分组；同一 net 的分支必须绑定一致的 LCP 物理位置
- `allow_lcp_location_negotiation`（默认 true）且该网**只有一个 LCP** 时：detailed 提交前可尝试切换到其他可达 location，整网统一落点
- 多 LCP 网：不做组合协商，严格沿用 DP 绑定，避免搜索爆炸

### 6.4 Space feedback 与 LCP

Space 需求（`SpaceNode::formula_required_space`）：

```text
R = Σ (L_j * max(K_j, 2) / 2)
```

其中 `L_j` 为 LCP 内 segment 的最大线宽，`K_j` 为 segment 数。  
最终 `required_space()` = `max(allocated + coupling_extra, formula)`。

当 LCP 走 **direct fallback**（DP 失败且非 strict）时，对仍挂有 LCP 的 space，反馈宽度会额外抬升：

```text
required ≥ (required_space - coupling_extra) + max_lcp_width_in_space
```

用于下一轮 packing 扩大预留。

## 7. 配置与诊断

| 项 | 默认 | 作用 |
|----|------|------|
| `seed` / `lcp_candidate_seed` | 1 | LCP 区域随机候选可复现 |
| `--strict-lcp-dp` | 关 | 禁止 LCP net 在 DP 失败后 greedy 回退 |
| `negotiate_lcp_locations` | true | 单 LCP net detailed 前协商位置 |

主要输出：

- `routing_debug.json`：`lcp_topologies`、候选覆盖、DP 状态、detailed 映射
- `sa_trace.json`：每轮 `move`（含 `lcp-*`）、accept、feedback
- layout 渲染（`tools/render_layout.py`）：从 debug 读取最终选用 LCP，画菱形标记

相关测试：`tests/test_lcp_binding.cpp`（初始绑定 + packing 不重绑）、`tests/test_routing_evaluator.cpp`（LCP A*/DP/detailed）、`tests/test_router.cpp`（四类扰动）。

## 8. 源文件索引

| 文件 | 职责 |
|------|------|
| `include/sapr/lcp_generator.hpp` / `src/lcp_generator.cpp` | 初始拓扑、space 绑定、候选生成/刷新 |
| `include/sapr/model.hpp` | LCP / SpaceNode / NetTopology / request 字段 |
| `src/tree.cpp` | LCP 合法性、四类 SA 扰动、`required_space` 公式 |
| `src/optimizer.cpp` | SA 前初始化、packing 后刷新、写回 tree、fallback feedback |
| `src/routing_evaluator.cpp` | LCP 候选 A*、multi-terminal 过滤、DP fallback、detailed 协商 |
| `tools/render_layout.py` | 可视化最终选用 LCP |

## 9. 与论文/计划文档的关系

`docs/REPRODUCTION_PLAN.md` 中的 LCP 扰动四类、space 资源公式 `Σ Lj·Kj/2`、bottom-up DP + top-down detailed，与当前实现对齐。  
实现上额外固化了若干工程规则（space 归属仅 SA 可变、multi-terminal 全覆盖过滤、strict DP、单 LCP 协商等），阅读代码时以本节为准。
