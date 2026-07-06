// 文件职责：声明 DP-based global routing 的数据结构和求解接口。
#pragma once

#include <string>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr::routing {

// 表示全局布线阶段的代价权重和惩罚参数，via 只统计不计入代价。
struct GlobalRouterConfig {
    double wirelength_weight{1.0};
    double bend_weight{3.0};
    double conflict_penalty_per_point{100000.0};
    double failed_pair_penalty{100000.0};
    double flow_violation_penalty{50000.0};
    double current_density_penalty{50000.0};
    double short_conflict_penalty{1000000.0};
};

// 表示一条 net 在全局布线阶段选中的候选路径集合。
struct NetRouteChoice {
    std::string net;
    std::vector<RouteCandidate> selected_candidates;
    PathMetrics metrics;
    double penalty{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double routing_failure_penalty{};
    bool success{true};
    std::string message;
};

// 表示所有 net 的全局布线汇总结果。
struct GlobalRoutingResult {
    std::vector<NetRouteChoice> net_routes;
    PathMetrics total_metrics;
    double total_penalty{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double routing_failure_penalty{};
    int failed_nets{};
};

// 从 A* 候选路径中为每条 net 选择 DP 代价最低的路径组合。
GlobalRoutingResult run_global_routing(
    const Circuit& circuit,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    const GlobalRouterConfig& config = GlobalRouterConfig{});

}  // namespace sapr::routing
