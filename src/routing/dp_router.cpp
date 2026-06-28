// 文件职责：实现 B*-tree 自底向上的 routing DP 与 DP traceback 恢复。
#include "sapr/routing/dp_router.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace sapr::routing {
namespace {

// 表示同一 terminal pair 的候选路径集合。
struct CandidateGroup {
    std::string key;
    std::vector<RouteCandidate> candidates;
};

// 返回 terminal 所属模块，LCP 或无法识别时返回空字符串。
std::string module_for_terminal(const std::string& terminal) {
    const auto dot = terminal.find('.');
    return dot == std::string::npos ? std::string{} : terminal.substr(0, dot);
}

// 构造 terminal pair 的稳定分组 key。
std::string group_key(const RouteCandidate& candidate) {
    return candidate.net + "|" + candidate.from_terminal + "|" + candidate.to_terminal;
}

// 返回 candidate 是否和当前子树模块相关。
bool candidate_touches_subtree(
    const RouteCandidate& candidate,
    const std::unordered_set<std::string>& subtree_modules,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const std::string from_module = module_for_terminal(candidate.from_terminal);
    const std::string to_module = module_for_terminal(candidate.to_terminal);
    const auto lcp_owner = lcp_owner_by_id.find(candidate.lcp_id);
    return (!from_module.empty() && subtree_modules.contains(from_module)) ||
           (!to_module.empty() && subtree_modules.contains(to_module)) ||
           (lcp_owner != lcp_owner_by_id.end() && subtree_modules.contains(lcp_owner->second));
}

// 将候选路径代价累加到 DP state。
void add_candidate_metrics(RoutingDpState& state, const RouteCandidate& candidate) {
    state.metrics.wirelength += candidate.path.metrics.wirelength;
    state.metrics.bend_count += candidate.path.metrics.bend_count;
    state.metrics.via_count += candidate.path.metrics.via_count;
    state.penalty += candidate.flow_penalty + candidate.current_density_penalty + candidate.coupling_cost;
    state.cost = state.metrics.wirelength + 3.0 * static_cast<double>(state.metrics.bend_count) + state.penalty;
    state.metrics.cost = state.cost;
}

// 尝试把 candidate 加入 state，并维护 LCP 位置一致性。
bool append_candidate_if_consistent(RoutingDpState& state, const RouteCandidate& candidate) {
    if (!candidate.path.success) return false;
    if (!candidate.lcp_id.empty()) {
        const auto assigned = state.lcp_location_by_id.find(candidate.lcp_id);
        if (assigned != state.lcp_location_by_id.end() && assigned->second != candidate.lcp_candidate_id) return false;
        if (assigned == state.lcp_location_by_id.end()) state.lcp_location_by_id[candidate.lcp_id] = candidate.lcp_candidate_id;
    }
    state.selected_candidates.push_back(candidate);
    add_candidate_metrics(state, candidate);
    return true;
}

// 将候选按 terminal pair 分组。
std::vector<CandidateGroup> make_candidate_groups(const std::vector<RouteCandidate>& candidates) {
    std::vector<CandidateGroup> groups;
    std::unordered_map<std::string, std::size_t> index_by_key;
    for (const auto& candidate : candidates) {
        const std::string key = group_key(candidate);
        const auto found = index_by_key.find(key);
        if (found == index_by_key.end()) {
            index_by_key[key] = groups.size();
            groups.push_back(CandidateGroup{key, {candidate}});
        } else {
            groups[found->second].candidates.push_back(candidate);
        }
    }
    return groups;
}

// 根据 request.space_nodes 建立 LCP 所属模块查找表。
std::unordered_map<std::string, std::string> make_lcp_owner_map(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::string> result;
    for (const auto& space : request.space_nodes) {
        for (const auto& point : space.linking_points) result[point.id] = space.owner;
    }
    return result;
}

// 按代价剪枝并重新编号状态。
void prune_states(std::vector<RoutingDpState>& states, int max_states, int& pruned_states) {
    std::sort(states.begin(), states.end(), [](const auto& left, const auto& right) {
        if (left.cost != right.cost) return left.cost < right.cost;
        return left.selected_candidates.size() < right.selected_candidates.size();
    });
    if (max_states > 0 && states.size() > static_cast<std::size_t>(max_states)) {
        pruned_states += static_cast<int>(states.size() - static_cast<std::size_t>(max_states));
        states.resize(static_cast<std::size_t>(max_states));
    }
    for (std::size_t index = 0; index < states.size(); ++index) states[index].id = static_cast<int>(index);
}

// 按 request.tree 构造 node 查找表。
std::unordered_map<std::string, RoutingTreeNodeRef> tree_nodes_by_id(const RoutingTreeSnapshot& tree) {
    std::unordered_map<std::string, RoutingTreeNodeRef> result;
    for (const auto& node : tree.nodes) result[node.id] = node;
    return result;
}

// 收集 B*-tree post-order 顺序。
void collect_post_order(
    const std::string& id,
    const std::unordered_map<std::string, RoutingTreeNodeRef>& nodes,
    std::unordered_set<std::string>& visited,
    std::vector<std::string>& order) {
    if (visited.contains(id)) return;
    visited.insert(id);
    const auto found = nodes.find(id);
    if (found == nodes.end()) return;
    if (found->second.left.has_value()) collect_post_order(*found->second.left, nodes, visited, order);
    if (found->second.right.has_value()) collect_post_order(*found->second.right, nodes, visited, order);
    order.push_back(id);
}

// 收集每个 tree node 对应子树内的模块集合。
std::unordered_set<std::string> collect_subtree_modules(
    const std::string& id,
    const std::unordered_map<std::string, RoutingTreeNodeRef>& nodes,
    std::unordered_map<std::string, std::unordered_set<std::string>>& cache) {
    const auto cached = cache.find(id);
    if (cached != cache.end()) return cached->second;
    std::unordered_set<std::string> modules;
    const auto found = nodes.find(id);
    if (found == nodes.end()) return modules;
    modules.insert(found->second.module.empty() ? found->second.id : found->second.module);
    if (found->second.left.has_value()) {
        const auto left = collect_subtree_modules(*found->second.left, nodes, cache);
        modules.insert(left.begin(), left.end());
    }
    if (found->second.right.has_value()) {
        const auto right = collect_subtree_modules(*found->second.right, nodes, cache);
        modules.insert(right.begin(), right.end());
    }
    cache[id] = modules;
    return modules;
}

// 生成一个 tree node 的 DP states。
std::vector<RoutingDpState> build_states_for_node(
    const std::string& node_id,
    const std::unordered_set<std::string>& subtree_modules,
    const std::vector<CandidateGroup>& groups,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id,
    int max_states,
    int& pruned_states) {
    RoutingDpState initial;
    initial.id = 0;
    initial.tree_node = node_id;
    std::vector<RoutingDpState> states{initial};
    for (const auto& group : groups) {
        bool touches = false;
        for (const auto& candidate : group.candidates) {
            if (candidate_touches_subtree(candidate, subtree_modules, lcp_owner_by_id)) {
                touches = true;
                break;
            }
        }
        if (!touches) continue;

        std::vector<RoutingDpState> next_states;
        for (const auto& state : states) {
            for (const auto& candidate : group.candidates) {
                if (!candidate_touches_subtree(candidate, subtree_modules, lcp_owner_by_id)) continue;
                RoutingDpState next = state;
                if (!append_candidate_if_consistent(next, candidate)) continue;
                next.choice_message = group.key;
                next_states.push_back(std::move(next));
            }
        }
        if (!next_states.empty()) {
            prune_states(next_states, max_states, pruned_states);
            states = std::move(next_states);
        }
    }
    prune_states(states, max_states, pruned_states);
    return states;
}

// 从 root state 生成 traceback candidates，并按 terminal pair 去重。
std::vector<RouteCandidate> traceback_candidates_from_state(const RoutingDpState& state) {
    std::vector<RouteCandidate> result;
    std::unordered_set<std::string> seen;
    for (const auto& candidate : state.selected_candidates) {
        const std::string key = group_key(candidate);
        if (seen.insert(key).second) result.push_back(candidate);
    }
    return result;
}

}  // namespace

// 按 B*-tree post-order 运行 routing DP，并从 A* candidates 中选择一致的 traceback 路径。
RoutingDpResult run_bottom_up_routing_dp(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    int max_states_per_node) {
    (void)circuit;
    (void)context;
    RoutingDpResult result;
    if (!request.tree.root.has_value() || request.tree.nodes.empty() || candidates.empty()) return result;

    const auto nodes = tree_nodes_by_id(request.tree);
    std::vector<std::string> post_order;
    std::unordered_set<std::string> visited;
    collect_post_order(*request.tree.root, nodes, visited, post_order);
    if (post_order.empty()) return result;

    const auto groups = make_candidate_groups(candidates);
    const auto lcp_owner_by_id = make_lcp_owner_map(request);
    std::unordered_map<std::string, std::unordered_set<std::string>> subtree_cache;
    for (const auto& node_id : post_order) {
        const auto subtree_modules = collect_subtree_modules(node_id, nodes, subtree_cache);
        NodeRoutingDpResult node_result;
        node_result.tree_node = node_id;
        node_result.states = build_states_for_node(
            node_id,
            subtree_modules,
            groups,
            lcp_owner_by_id,
            max_states_per_node,
            result.dp_pruned_states);
        result.dp_nodes += 1;
        result.dp_states += static_cast<int>(node_result.states.size());
        result.node_results.push_back(std::move(node_result));
    }

    auto& root_result = result.node_results.back();
    if (root_result.states.empty()) return result;
    result.best_state = root_result.states.front();
    result.traceback_candidates = traceback_candidates_from_state(result.best_state);
    result.success = !result.traceback_candidates.empty();
    return result;
}

}  // namespace sapr::routing
