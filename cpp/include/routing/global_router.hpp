// 文件职责：声明阶段 3 的 DP-based global routing 数据结构和求解接口。
#pragma once

#include <string>
#include <vector>

#include "core/model.hpp"
#include "routing/path.hpp"
#include "routing/routing_context.hpp"

namespace analog_sapr {

// 表示全局路由阶段的代价权重和惩罚参数，via 只统计不参与 cost。
struct GlobalRouterConfig {
    double wirelength_weight = 1.0;
    double bend_weight = 3.0;
    double conflict_penalty_per_point = 100.0;
    double failed_pair_penalty = 100000.0;
};

// 表示一条 net 的全局路径选择结果。
struct NetRouteChoice {
    std::string net;
    std::vector<RouteCandidate> selected_candidates;
    PathMetrics metrics;
    double penalty = 0.0;
    bool success = true;
    std::string message;
};

// 表示所有 net 的 global routing 汇总结果。
struct GlobalRoutingResult {
    std::vector<NetRouteChoice> net_routes;
    PathMetrics total_metrics;
    double total_penalty = 0.0;
    int failed_nets = 0;
};

// 从阶段 2 候选路径中选择每条 net 的全局候选路径集合。
GlobalRoutingResult run_global_routing(
    const Circuit& circuit,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    const GlobalRouterConfig& config = GlobalRouterConfig{});

}  // namespace analog_sapr
