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
    int packing_step_index{-1};
    double contour_y{};
    std::unordered_map<std::string, std::string> lcp_location_by_id;
    // 记录当前 state 已覆盖的 pin/LCP terminal，供 traceback 和测试检查。
    std::vector<std::string> covered_terminals;
    // 记录当前 state 已完成的逻辑 wire segment。
    std::vector<std::string> covered_wire_segments;
    // 记录当前 state 在各个 DP transition 中选择的 segment。
    std::vector<std::string> selected_transitions;
    // 记录缺失 candidate 或 transition 失败的原因。
    std::vector<std::string> failure_messages;
    std::vector<RouteCandidate> selected_candidates;
    // 记录当前 state 已选候选的金属占用，用于提前排除异网同层短路。
    std::vector<RouteSegment> occupied_routes;
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
    int packing_time_dp_segments{};
    bool packing_time_dp_used{};
};

// 按 B*-tree post-order 运行 routing DP，并从 A* candidates 中选择一致的 traceback 路径。
RoutingDpResult run_bottom_up_routing_dp(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    int max_states_per_node = 8);

}  // namespace sapr::routing
