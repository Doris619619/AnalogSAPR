// 文件职责：声明基于 B*-tree 自底向上的 routing DP 状态和求解接口。
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr::routing {

// 表示 bottom-up routing DP 中的一个候选状态。
struct RoutingDpState {
    int id{};
    std::string tree_node;
    std::unordered_map<std::string, std::string> lcp_location_by_id;
    std::vector<RouteCandidate> selected_candidates;
    PathMetrics metrics;
    double penalty{};
    double cost{};
    int parent_left{-1};
    int parent_right{-1};
    std::string choice_message;
};

// 表示一个 B*-tree node 对应的 DP 状态集合。
struct NodeRoutingDpResult {
    std::string tree_node;
    std::vector<RoutingDpState> states;
};

// 表示完整 bottom-up routing DP 的结果和 traceback。
struct RoutingDpResult {
    bool success{};
    std::vector<NodeRoutingDpResult> node_results;
    RoutingDpState best_state;
    std::vector<RouteCandidate> traceback_candidates;
    int dp_nodes{};
    int dp_states{};
    int dp_pruned_states{};
};

// 按 B*-tree post-order 运行 routing DP，并从 A* candidates 中选择一致的 traceback 路径。
RoutingDpResult run_bottom_up_routing_dp(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    int max_states_per_node = 8);

}  // namespace sapr::routing
