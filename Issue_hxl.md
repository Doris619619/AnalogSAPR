# Routing 算法说明与差距检查

本文档用于说明当前工程中布线部分已经使用的算法、关键概念、代价公式和约束检查方式，方便团队成员对齐实现内容，并检查与论文 *Simultaneous Analog Placement and Routing with Current Flow and Current Density Considerations* 的差距。

## 当前 Routing 流程总览

当前布线主流程位于 `routing_evaluator` 和 `routing` 模块中，整体数据流如下：

```text
Enhanced B*-tree / placement candidate
        ↓
RoutingEvaluationRequest
        ↓
RoutingContext
        ↓
Grid / Obstacle / Global pins
        ↓
A* route candidate generation
        ↓
LCP-aware bottom-up DP global routing
        ↓
DP traceback selected candidates
        ↓
Performance-aware detailed routing
        ↓
RouteSegment / Metrics / phi_cost
```

当前主流程保持一个入口：

```cpp
evaluate_routing(circuit, request)
run_detailed_routing(circuit, request, evaluation)
```

optimizer / SA 只需要调用 routing evaluator，不直接操作 A*、DP 或 detailed routing 的内部细节。

---

## 1. RoutingContext：布线世界建模

`RoutingContext` 的作用是把当前 placement 转换成可布线的多层网格环境。

### 输入

- `Circuit`
- `Placement`
- module active region
- pin local coordinate
- WIRE_WIDTH 约束

### 输出

- 全局 pin 坐标
- 多层 grid
- active region obstacle
- net 默认线宽

### 核心概念

| 概念 | 当前实现 |
|------|----------|
| grid point | 离散网格点 `(ix, iy, layer)` |
| physical point | 连续坐标 `(x, y)` |
| layer | `M1 ~ M7`，当前全部允许水平和垂直走线 |
| obstacle | active region 默认阻塞所有金属层 |
| pin access | pin 所在点允许作为起点或终点 |

坐标转换规则：

```text
local pin coordinate
        ↓
orient transform
        ↓
placement translate
        ↓
global pin coordinate
        ↓
snap_to_grid()
```

---

## 2. A* 路径搜索

A* 用于在多层网格上为 terminal pair 生成候选路径。

### A* 做什么

给定：

```text
start grid point
goal grid point
obstacle map
wire width
```

搜索一条可行路径：

```text
GridPath {
    points;
    wirelength;
    bend_count;
    via_count;
    success;
}
```

### A* 搜索空间

当前每个 grid point 的邻居包括：

```text
同层四邻居:
left / right / up / down

via 邻居:
same (ix, iy), layer + 1
same (ix, iy), layer - 1
```

### A* 代价

当前 A* 使用网格路径代价，主要考虑：

```text
wirelength
bend_count
via_count
obstacle blocking
```

其中 via 会被统计，但在后续 DP global routing 选择代价中不参与选择。

### A* 与 DP 的关系

当前实现中：

```text
A* 负责生成几何路径候选
DP 负责从候选路径中选择组合
```

也就是说，DP 不直接搜索几何路径，DP 的路径来源仍然是 A*。

---

## 3. LCP-aware 候选路径生成

LCP 是 linking-control point，用于在 enhanced B*-tree 中表示可变的布线拓扑控制点。

### LCP 的作用

论文中 LCP 的作用是：

- link interconnects
- control routing topology between modules
- record wire width range
- record current direction
- provide physical location candidates

当前代码中，LCP 会被展开为虚拟 routing terminal。

例如：

```text
原始 net:
M1.D ---- M2.G

带 LCP 后:
M1.D ---- LCP1 ---- M2.G
```

每一段会生成 A* candidate：

```text
M1.D -> LCP1
LCP1 -> M2.G
```

### LCP 多候选位置

一个 LCP 可以有多个 `location_candidate`。

当前实现会为每个候选位置生成对应路径：

```text
LCP1:first
LCP1:second
LCP1:third
```

并在 global routing / bottom-up DP 中保证：

```text
同一个 LCP 在同一个 routing solution 中只能选择一个 lcp_candidate_id
```

如果同一 LCP 的两条 segment 选择了不同 candidate id，则该组合非法。

---

## 4. DP-based Global Routing

当前实现已经从简单的 candidate filtering 推进到 B*-tree post-order 的 bottom-up DP。

### DP 输入

```text
Circuit
RoutingEvaluationRequest
RoutingContext
RouteCandidate[]
```

其中 `RoutingEvaluationRequest` 包含：

```text
placement
space_nodes
linking_points
net_topologies
tree snapshot
```

### DP state

当前 `RoutingDpState` 记录：

```text
tree_node
lcp_location_by_id
covered_terminals
covered_wire_segments
selected_transitions
failure_messages
selected_candidates
metrics
penalty
cost
parent_left
parent_right
```

### Bottom-up DP 过程

DP 按 B*-tree post-order 执行：

```text
leaf node
    ↓
生成初始 state

internal node
    ↓
合并 left child state
合并 right child state
检查 LCP location 是否一致
补全当前 node 负责的 local wire segment
从 A* candidates 中选择路径
剪枝保留低成本 state
```

### DP 代价公式

当前 DP 选择代价：

```text
cost = wirelength + bend_weight * bend_count + penalty
```

代码中当前使用：

```text
bend_weight = 3.0
```

所以：

```text
cost = W + 3B + p
```

这对应论文中的 global routing 局部代价：

```text
Cij = αWij + βBij + p
```

当前阶段约定：

```text
via_count 只统计，不参与 DP candidate selection cost
```

### DP failure penalty

如果 required wire segment 找不到成功 A* candidate：

```text
penalty += 100000
failure_messages.push_back(...)
```

这样不会静默漏掉 segment。

---

## 5. GlobalRoutingResult

DP 结束后，会把 root best state 的 selected candidates 转换为 global routing result。

### 输出内容

```text
GlobalRoutingResult {
    net_routes
    total_metrics
    total_penalty
    failed_nets
    flow_penalty
    current_density_penalty
    coupling_penalty
    routing_failure_penalty
}
```

### 全局布线 cost

当前 global routing cost：

```text
global_cost = wirelength + 3 * bend_count + total_penalty
```

注意：

```text
via_count 不进入 global routing cost
```

---

## 6. Performance-aware Detailed Routing

Detailed routing 当前从 DP traceback candidates 出发，把 A* 网格路径压缩成 `RouteSegment`，同时做性能约束检查。

### 输入

```text
Circuit
RoutingEvaluationRequest
RoutingEvaluation
```

### 输出

```text
DetailedRoutingResult {
    routes
    report
    required_space_by_node
    coupling_space_by_node
    detailed_cost
    flow_penalty
    current_density_penalty
    coupling_penalty
    design_rule_penalty
    routing_failure_penalty
    detailed_routing_penalty
}
```

### Traceback 顺序

当前 detailed routing 会优先使用：

```text
bottom_up_dp.traceback_candidates
```

然后按：

```text
NetTopology::segments
```

恢复顺序。

这使 detailed routing 不再只是遍历扁平路径，而是尽量沿 LCP topology 做 top-down traceback。

### 路径压缩

A* 路径是 grid point 序列：

```text
p0 -> p1 -> p2 -> ... -> pn
```

Detailed routing 会做：

```text
去除 A-B-A 即时回折
去除零长度线段
合并同层同方向连续线段
via move 不写入 routing.txt
```

输出为：

```text
RouteSegment {
    net;
    layer;
    x1, y1;
    x2, y2;
    width;
}
```

---

## 7. Current-flow 检查

FLOW 约束表示某条 net 上电流或信号必须从：

```text
out_pin -> in_pin
```

### Candidate-level FLOW

候选路径生成阶段会先检查：

```text
candidate.from_terminal == in_pin
candidate.to_terminal == out_pin
```

如果反向，则标记为 `flow_ok = false`。

### Detailed traceback FLOW

Detailed routing 阶段会进一步根据 selected candidates 建有向图：

```text
candidate.from_terminal -> candidate.to_terminal
```

然后检查是否存在路径：

```text
out_pin ⇒ in_pin
```

如果不存在：

```text
flow_violations += 1
flow_penalty += 50000
```

如果发现直接反向 segment：

```text
in_pin -> out_pin
```

也会记录 FLOW violation。

当前 FLOW 检查属于拓扑方向检查，不是完整电流强度传播模型。

---

## 8. Current-density / WIRE_WIDTH 检查

论文中的 current-density 约束与线宽有关，用于避免 IR-drop 和 electromigration 问题。

当前实现使用 `WIRE_WIDTH` 作为 current-density 的代理模型。

### Net-level WIRE_WIDTH

输入约束：

```text
WIRE_WIDTH net min_width max_width
```

表示该 net 的 route width 必须满足：

```text
min_width <= width <= max_width
```

### Segment-level width

LCP 的 `WireSegmentRef` 也保存：

```text
min_width
max_width
```

Detailed routing 会检查 candidate 原始线宽是否满足：

```text
segment.min_width <= candidate.wire_width <= segment.max_width
```

以及 net-level WIRE_WIDTH。

如果违反：

```text
current_density_violations += 1
current_density_penalty += 50000
```

并在 report 中记录具体 segment。

### 输出线宽

最终写入 `routing.txt` 的 width 会尽量落入合法范围：

```text
width = max(candidate_width, min_width)
width = min(width, max_width)
```

因此当前检查分两层：

```text
candidate 原始宽度是否违反约束
最终 RouteSegment.width 是否写成合法宽度
```

---

## 9. DRC active-region 检查

当前 DRC v1 主要检查 route metal rectangle 是否穿越 active region。

### Metal rectangle

RouteSegment 是中心线：

```text
(x1, y1) -> (x2, y2), width
```

实际金属占用近似为：

```text
segment_to_rect(centerline, width)
```

### DRC 规则

如果 metal rect 与 module active region 相交：

```text
intersects(metal_rect, active_region)
```

且不是 pin access corridor，则记为 DRC violation。

Penalty：

```text
design_rule_penalty = 100000 * design_rule_violations
```

当前 DRC 还没有覆盖完整制造规则，例如：

- metal spacing
- via enclosure
- min area
- layer-specific rule
- same-net spacing exception

---

## 10. Coupling noise 几何近似

论文目标中包含 coupling noise。当前工程没有做 parasitic extraction，而是采用几何近似。

### 当前 coupling 条件

两条 route segment 如果满足：

```text
不同 net
同一 layer
方向平行
间距小于 spacing
投影有重叠
```

则认为存在 coupling risk。

### overlap length

当前 coupling v2 不再只按线段对计数，而是计算平行重叠长度。

水平线段：

```text
overlap = min(max(x1), max(x2)) - max(min(x1), min(x2))
```

垂直线段：

```text
overlap = min(max(y1), max(y2)) - max(min(y1), min(y2))
```

### Coupling penalty

当前公式：

```text
coupling_penalty += coupling_weight * overlap_length / spacing
```

其中：

```text
coupling_weight = 100
spacing = 1.0
```

所以：

```text
coupling_penalty += 100 * overlap_length
```

report 中会记录：

```text
netA<->netB routes=i,j overlap=...
```

---

## 11. Routing space feedback

论文中 space node 用于为 interconnect 预留布线资源。

论文给出的 routing space 公式为：

```text
Ri = Σ (Lj * Kj / 2), for all LCP j in space node i
```

含义：

| 符号 | 含义 |
|------|------|
| `Ri` | space node i 需要预留的 routing space |
| `Lj` | LCP j 连接线段所需的最大线宽 |
| `Kj` | LCP j 连接的 wire segment 数量 |

当前工程中：

```text
SpaceNode::required_space()
```

会根据 LCP required width、segment 数量、allocated space 和 coupling extra space 计算预留空间。

Detailed routing 还会根据实际 route width 更新：

```text
required_space_by_node[space_node] = max(old, route_width + spacing)
```

如果检测到 coupling risk：

```text
coupling_space_by_node[space_node] = spacing
```

该 feedback 会回传给 placement / SA。

---

## 12. 论文总成本 phi

当前 optimizer 使用论文总成本：

```text
phi = α * A/A_hat
    + β * W/W_hat
    + γ * B/B_hat
    + δ * V/V_hat
    + p
```

其中：

| 项 | 含义 |
|----|------|
| `A` | chip area |
| `W` | routed wirelength |
| `B` | bend count |
| `V` | via count |
| `p` | penalty |

当前 `p` 汇总：

```text
global routing penalty
detailed routing penalty
flow penalty
current-density penalty
coupling penalty
DRC penalty
routing failure penalty
```

注意：

```text
via 不进入 DP candidate selection cost
via 进入最终 phi_cost
```

---

## 13. 当前已实现算法清单

| 模块 | 已实现内容 |
|------|------------|
| 多层网格模型 | `M1 ~ M7`，grid point，via neighbor，四邻居 |
| 障碍物模型 | active region blocker，pin access exception |
| A* routing | terminal pair path search，wirelength/bend/via metrics |
| LCP candidate generation | pin-LCP / LCP-pin / LCP-LCP segment candidate |
| LCP 多候选位置 | 同一 LCP 的 selected segment 共享一个 `lcp_candidate_id` |
| DP global routing | B*-tree post-order，child state merge，local segment transition |
| DP traceback | root best state 输出 selected candidates |
| Detailed routing | topology order traceback，path cleanup，RouteSegment 输出 |
| FLOW 检查 | candidate direction + detailed directed graph reachability |
| Current-density 代理 | WIRE_WIDTH / segment width range 检查 |
| DRC v1 | active region crossing |
| Coupling v2 | same-layer parallel overlap length penalty |
| Phi cost | area/wire/bend/via normalized cost + penalty |

---

## 14. 与论文相比仍需检查的点

当前实现已经覆盖论文算法的主要框架，但仍有以下简化：

| 论文要求 | 当前实现 | 后续可补 |
|----------|----------|----------|
| packing 与 routing 完全同步 | 当前 SA candidate 中会调用 routing evaluator，但 DP 仍是 evaluator 内部执行 | 更深度地把 DP state 与 packing contour 更新绑定 |
| performance-aware detailed routing | 已有 traceback + penalty，但没有 track assignment | 实现局部 track assignment / layer assignment |
| current-density 物理模型 | 当前用 WIRE_WIDTH 代理 | 引入 current demand / EM capacity / IR-drop 估算 |
| coupling noise | 当前用 overlap length 近似 | 引入更真实的耦合电容或 spacing model |
| 完整 DRC | 当前只检查 active crossing | 扩展 spacing / via enclosure / min-area 等规则 |
| via optimization | via 统计并进入 phi，但 DP 不计 via | 可在 detailed routing 层优化 via 数量 |
| benchmark 对比 | 已有 smoke / unit test | 补论文 benchmark 表格和可视化报告 |

---

## 15. 下一步建议

优先级建议：

1. 补 detailed routing 的局部 track assignment。
2. 扩展 DRC 到 spacing / via / enclosure。
3. 为 current-density 引入 net current demand 和 layer capacity。
4. 让 coupling penalty 与 spacing / overlap / width 更细粒度相关。
5. 生成 benchmark 统计表，对比 area、wirelength、bend、via、penalty、runtime。

当前最值得继续做的是：

```text
Top-down detailed routing local refinement
```

原因：

- 已经有 DP traceback candidates。
- 已经有 LCP / space node / topology trace。
- 继续补 track assignment 和局部优化最自然。
- 能进一步接近论文中 performance-aware detailed routing 的描述。
 
---

## 16. DP 与 packing contour 绑定的最新进展

当前已经完成了 DP state 与 packing contour 的三层绑定。目标是让 routing DP 不再只是“最终 placement 之后的独立评估”，而是能追溯并消费 packing 过程中产生的 contour step 信息。

### 16.1 PackingContourTrace：记录 packing 中间状态

`pack_enhanced_tree()` 会为每个 B*-tree representative node 记录一个 `PackingContourStep`：

```text
PackingContourStep {
    tree_node
    module
    x, y
    occupied_bbox
    desired_x, desired_y
    contour_y
    right_space
    top_space
    coupling_extra_space
    left, right
    subtree_modules
    local_wire_segments
    cross_child_wire_segments
}
```

其中：

| 字段 | 含义 |
|------|------|
| `occupied_bbox` | 当前 node 在 contour packing 后占用的 bbox |
| `contour_y` | 当前 node 被已有 contour 顶起后的 y 高度 |
| `right_space/top_space` | 当前 node 右侧/上侧预留 routing space |
| `subtree_modules` | 当前 B*-tree node 覆盖的子树模块集合 |
| `local_wire_segments` | 当前 packing step 应补全的 routing DP transition |
| `cross_child_wire_segments` | 跨 left/right child subtree 的 local segment |

这一步的意义是：DP state 可以知道自己对应哪个 packing step，而不是只知道最终 placement。

### 16.2 Routing space feedback：routing 结果影响下一轮 contour

Detailed routing 会输出：

```text
required_space_by_node
coupling_space_by_node
```

`apply_routing_feedback()` 会把这些结果写回 enhanced B*-tree 的 `SpaceNode`：

```text
allocated_space = max(old allocated_space, detailed required space)
coupling_extra_space = max(old coupling_extra_space, detailed coupling space)
```

然后下一次 `pack_enhanced_tree()` 会通过：

```text
SpaceNode::required_space()
```

影响 `occupied_size()` 和 contour height。

当前 candidate 评估内部也已经有有限轮 feedback 闭环：

```text
pack contour
    -> A*/DP + detailed routing
    -> apply routing space feedback
    -> re-pack contour with updated SpaceNode
    -> re-route / re-score
```

对应调试指标：

```text
routing_feedback_iterations
routing_feedback_converged
space_feedback_nodes
```

### 16.3 Packing-time DP transition obligations

最新实现进一步把 DP transition 来源从“pack 后推断”推进为“packing trace 显式记录”。

在 `pack_enhanced_tree()` 完成 routing context 后，会根据：

```text
NetTopology::segments
current step subtree_modules
left child subtree_modules
right child subtree_modules
LCP 所属 space node owner
```

为每个 contour step 标注它负责补全的 `local_wire_segments`。

判断规则：

```text
如果 segment 两端都属于当前 step subtree
并且不完全属于 left child subtree
并且不完全属于 right child subtree
则该 segment 是当前 packing step 的 local DP transition
```

随后 `run_bottom_up_routing_dp()` 会优先使用：

```text
PackingContourStep.local_wire_segments
```

作为当前 node 的 transition 来源。若 trace 中没有 local segment，则回退到原来的 subtree 推断逻辑，保证旧 request 和测试场景仍能运行。

对应调试指标：

```text
packing_time_dp_used
packing_time_dp_segments
```

当前 smoke test 中可以看到：

```text
packing_time_dp_used: true
packing_time_dp_segments: 25
```

### 16.4 新部分与之前实现的区别

| 阶段 | 之前做到什么 | 最新补充了什么 |
|------|--------------|----------------|
| trace 绑定 | DP state 记录 `packing_step_index` 和 `contour_y` | trace 中进一步记录 routing space 和 local wire segments |
| space feedback | detailed routing 产生 required/coupling space | candidate 内部执行 feedback -> re-pack -> re-route 闭环 |
| DP transition | DP 根据 subtree/child subtree 在 routing 阶段推断 local segment | packing trace 显式告诉 DP 当前 step 应处理哪些 segment |
| 可解释性 | 可以知道 DP state 属于哪个 node | 可以知道每条 selected transition 来自哪个 packing step / space node |
| 论文对齐程度 | placement 后 routing evaluator 与 packing 有关联 | 更接近论文 bottom-up packing/routing DP 的执行结构 |

简而言之，之前是：

```text
final placement / tree snapshot
        -> routing DP 推断 local segment
```

现在是：

```text
packing contour step
        -> 显式生成 local routing obligation
        -> routing DP 消费 obligation
        -> detailed routing feedback 回写 space node
        -> 下一轮 packing 使用更新后的 space
```

### 16.5 当前仍不是完整论文同步 DP 的地方

虽然已经完成工程上可运行的 DP-contour 绑定，但仍有一个边界需要说明：

```text
A* candidates 仍由 routing evaluator 统一生成；
DP 在 packing trace 记录的 local segment 上选择候选路径；
还没有把 A* 搜索直接嵌入 place_node() 递归内部。
```

也就是说，当前实现是：

```text
packing-time obligation generation + bottom-up DP consumption
```

还不是：

```text
place_node() 每放置一个 node 时立即运行 A* 并完成 DP transition
```

这个设计保留了较低冲突和较好可测试性，同时已经能让 DP transition 与 contour step 对齐。

---

## 17. 接下来的工作分析

### 17.1 优先级最高：top-down detailed routing 的 local track assignment

现在 DP 已经能输出 packing-time traceback candidates，下一步最自然的是让 detailed routing 不只是压缩 A* path，而是做局部 track 分配：

```text
DP selected candidates
    -> 按 NetTopology / LCP / space node traceback
    -> 在每个 space node 内分配 local tracks
    -> 避免同层过近平行线
    -> 降低 coupling / DRC penalty
```

建议新增能力：

| 能力 | 目的 |
|------|------|
| local track index | 让同一 space node 内的多条线有离散 track |
| same-layer spacing check | 降低 coupling 和 spacing DRC |
| simple layer preference | 为后续 via / layer assignment 做准备 |
| track demand feedback | 把真实 track 数反馈给 `required_space_by_node` |

### 17.2 第二优先级：约束模型继续细化

当前约束仍有简化：

| 约束 | 当前实现 | 后续建议 |
|------|----------|----------|
| current density | WIRE_WIDTH 代理 | 加入 net current demand / layer capacity |
| coupling | parallel overlap 几何近似 | 加入 spacing、width、overlap 的更细公式 |
| DRC | active crossing | 扩展 metal spacing / via enclosure / min area |
| via | 统计并进入 phi | detailed routing 中减少 via 或加入 via placement 检查 |

### 17.3 第三优先级：实验和报告

算法主链条已经比较完整，后续需要把结果做成稳定实验输出：

```text
area
wirelength
bend_count
via_count
phi_cost
penalty breakdown
failed_nets
runtime
packing_time_dp_segments
routing_feedback_iterations
```

建议补：

- benchmark 批量运行脚本
- CSV/JSON summary
- routing 可视化图
- 与 baseline Manhattan routing 的对比表

### 17.4 推荐下一阶段目标

下一阶段建议做：

```text
Detailed routing local track assignment v1
```

原因：

- DP 与 packing contour 绑定已经能提供明确 traceback 来源；
- routing feedback 已能影响下一轮 contour；
- 当前 detailed routing 仍是最接近论文但也最明显简化的部分；
- 完成 track assignment 后，coupling、DRC、space feedback 都会更有物理意义。
