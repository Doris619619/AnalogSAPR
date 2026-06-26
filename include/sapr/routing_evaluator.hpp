// 文件职责：声明供布局、模拟退火和 CLI 调用的布线评估公开接口。
#pragma once

#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr {

// 汇总一次 placement 布线评估产生的上下文、候选路径和全局布线结果。
struct RoutingEvaluation {
    routing::RoutingContext context;
    std::vector<routing::RouteCandidate> candidates;
    routing::GlobalRoutingResult global_routing;
    double routing_cost{};
    int failed_nets{};
};

// 根据当前 placement 构建布线环境、生成 A* 候选路径并执行 DP 全局布线。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements);

// 根据 placement candidate 中的 LCP 拓扑执行 A*/DP 布线评估。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request);

// 将 DP 全局布线选中的 A* 网格路径转换为当前 routing.txt 使用的中心线线段。
std::vector<RouteSegment> selected_candidates_to_segments(
    const RoutingEvaluation& evaluation);

}  // namespace sapr
