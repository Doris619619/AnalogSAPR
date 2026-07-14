// 声明论文 placement-aware 联合求解器和增强 B*-tree packing 接口。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 在模拟退火前按网表和增强 B*-tree 创建并持久化初始 LCP 拓扑。
void initialize_lcp_topology(
    const Circuit& circuit,
    EnhancedBStarTree& tree,
    const SolverConfig& config = {});

// 按增强 B*-tree 和 ASF 对称组生成当前候选布局。
RoutingEvaluationRequest pack_enhanced_tree(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config);

// 使用 routing adapter 评价当前 placement candidate。
RoutingFeedback evaluate_with_routing_adapter(const Circuit& circuit, const RoutingEvaluationRequest& request);

// 使用论文 placement 框架、SA 和 routing adapter 生成布局布线解。
Solution solve_placement_aware(const Circuit& circuit, const SolverConfig& config = {});

// 保持历史 CLI/API 名称，当前默认指向论文 placement-aware 求解流程。
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config = {});

}  // namespace sapr
