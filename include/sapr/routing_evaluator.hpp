// 文件职责：声明供布局、模拟退火和 CLI 调用的布线评估公开接口。
#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/dp_router.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr {

// 汇总一次 placement 布线评估产生的上下文、候选路径和全局布线结果。
struct RoutingEvaluation {
    routing::RoutingContext context;
    std::vector<routing::RouteCandidate> candidates;
    routing::GlobalRoutingResult global_routing;
    std::optional<routing::RoutingDpResult> bottom_up_dp;
    double routing_cost{};
    int failed_nets{};
    bool used_bottom_up_dp{};
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

// 执行论文 top-down detailed routing 阶段，当前基于 DP 选中子问题回溯并清理路径。
DetailedRoutingResult run_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation);

}  // namespace sapr
