// 文件职责：实现基于 B*-tree 自底向上的 routing DP 和 DP traceback 恢复。
#include "sapr/routing/dp_router.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "sapr/routing/path_geometry.hpp"

namespace sapr::routing {
namespace {

constexpr double kBendWeight = 3.0;
constexpr double kMissingSegmentPenalty = 100000.0;
constexpr double kShortConflictPenalty = 1000000.0;

// 表示同一逻辑 wire segment 对应的一组 A* 候选路径。
struct CandidateGroup {
    std::string key;
    std::string net;
    std::string from;
    std::string to;
    std::string segment_id;
    std::vector<RouteCandidate> candidates;
};

// 表示 B*-tree node 的左右子树模块集合，供 local transition 判断使用。
struct ChildSubtreeModules {
    std::unordered_set<std::string> left;
    std::unordered_set<std::string> right;
};

// 表示 packing contour trace 在 DP 中的快速索引。
struct PackingTraceIndex {
    std::unordered_map<std::string, const PackingContourStep*> step_by_node;
    std::unordered_map<std::string, int> index_by_node;
};

// 返回 terminal 所属模块，LCP 或无法识别时返回空字符串。
std::string module_for_terminal(const std::string& terminal) {
    const auto dot = terminal.find('.');
    return dot == std::string::npos ? std::string{} : terminal.substr(0, dot);
}

// 返回 LCP 或 pin terminal 的 owner module。
std::string owner_for_terminal(
    const std::string& terminal,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const std::string module = module_for_terminal(terminal);
    if (!module.empty()) return module;
    const auto lcp = lcp_owner_by_id.find(terminal);
    return lcp == lcp_owner_by_id.end() ? std::string{} : lcp->second;
}

// 生成逻辑线段的稳定 key，优先使用 NetTopology 中的 segment id。
std::string segment_key(const std::string& net, const std::string& from, const std::string& to, const std::string& id) {
    if (!id.empty()) return id;
    return net + "|" + from + "|" + to;
}

// 生成候选路径的逻辑线段 key。
std::string candidate_segment_key(const RouteCandidate& candidate) {
    return segment_key(candidate.net, candidate.from_terminal, candidate.to_terminal, candidate.segment_id);
}

// 判断 candidate 是否能实现指定的逻辑线段。
bool candidate_matches_group(const RouteCandidate& candidate, const CandidateGroup& group) {
    if (candidate.net != group.net) return false;
    if (!group.segment_id.empty() && candidate.segment_id == group.segment_id) return true;
    return (candidate.from_terminal == group.from && candidate.to_terminal == group.to) ||
           (candidate.from_terminal == group.to && candidate.to_terminal == group.from);
}

// 将路径代价累加到 DP state；via 只统计，不进入 DP 选择代价。
void recompute_state_cost(RoutingDpState& state) {
    state.cost = state.metrics.wirelength + kBendWeight * static_cast<double>(state.metrics.bend_count) + state.penalty;
    state.metrics.cost = state.cost;
}

// 将候选路径代价累加到 DP state。
void add_candidate_metrics(RoutingDpState& state, const RouteCandidate& candidate) {
    state.metrics.wirelength += candidate.path.metrics.wirelength;
    state.metrics.bend_count += candidate.path.metrics.bend_count;
    state.metrics.via_count += candidate.path.metrics.via_count;
    state.penalty += candidate.flow_penalty + candidate.current_density_penalty + candidate.coupling_cost;
    recompute_state_cost(state);
}

// 追加不重复字符串，避免 traceback 列表出现重复项。
void append_unique(std::vector<std::string>& values, const std::string& value) {
    if (value.empty()) return;
    if (std::find(values.begin(), values.end(), value) == values.end()) values.push_back(value);
}

bool contains_value(const std::vector<std::string>& values, const std::string& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

// 合并字符串集合字段，用于 child state 合并。
void merge_unique_values(std::vector<std::string>& target, const std::vector<std::string>& source) {
    for (const auto& value : source) append_unique(target, value);
}

// 合并 LCP 位置绑定；若同一 LCP 绑定到不同候选位置，则该 transition 非法。
bool merge_lcp_locations(RoutingDpState& target, const RoutingDpState& source) {
    for (const auto& [lcp_id, candidate_id] : source.lcp_location_by_id) {
        const auto found = target.lcp_location_by_id.find(lcp_id);
        if (found != target.lcp_location_by_id.end() && found->second != candidate_id) return false;
        target.lcp_location_by_id.emplace(lcp_id, candidate_id);
    }
    return true;
}

// 合并左右 child state，形成当前 node 的 transition 起点。
bool merge_child_state(RoutingDpState& target, const RoutingDpState& child, bool left_child) {
    if (!merge_lcp_locations(target, child)) return false;
    merge_unique_values(target.covered_terminals, child.covered_terminals);
    merge_unique_values(target.covered_wire_segments, child.covered_wire_segments);
    merge_unique_values(target.selected_transitions, child.selected_transitions);
    merge_unique_values(target.failure_messages, child.failure_messages);
    target.selected_candidates.insert(
        target.selected_candidates.end(),
        child.selected_candidates.begin(),
        child.selected_candidates.end());
    target.occupied_routes.insert(
        target.occupied_routes.end(),
        child.occupied_routes.begin(),
        child.occupied_routes.end());
    target.metrics.wirelength += child.metrics.wirelength;
    target.metrics.bend_count += child.metrics.bend_count;
    target.metrics.via_count += child.metrics.via_count;
    target.penalty += child.penalty;
    if (left_child) {
        target.parent_left = child.id;
    } else {
        target.parent_right = child.id;
    }
    recompute_state_cost(target);
    return true;
}

// 尝试把 candidate 加入 state，并维护 LCP 位置一致性。
bool append_candidate_if_consistent(
    RoutingDpState& state,
    const RouteCandidate& candidate,
    const RoutingContext& context) {
    if (!candidate.path.success) return false;
    const double width = candidate.wire_width > 0.0 ? candidate.wire_width : context.default_width_for_net(candidate.net);
    auto candidate_routes = candidate_to_route_segments(context.grid(), candidate, width);
    const bool has_short = routes_short_with_existing(candidate_routes, state.occupied_routes);
    if (!candidate.lcp_id.empty()) {
        const auto assigned = state.lcp_location_by_id.find(candidate.lcp_id);
        if (assigned != state.lcp_location_by_id.end() && assigned->second != candidate.lcp_candidate_id) return false;
        if (assigned == state.lcp_location_by_id.end()) state.lcp_location_by_id[candidate.lcp_id] = candidate.lcp_candidate_id;
    }
    state.selected_candidates.push_back(candidate);
    state.occupied_routes.insert(state.occupied_routes.end(), candidate_routes.begin(), candidate_routes.end());
    add_candidate_metrics(state, candidate);
    if (has_short) {
        state.penalty += kShortConflictPenalty;
        recompute_state_cost(state);
    }
    append_unique(state.covered_terminals, candidate.from_terminal);
    append_unique(state.covered_terminals, candidate.to_terminal);
    append_unique(state.covered_wire_segments, candidate_segment_key(candidate));
    append_unique(state.selected_transitions, candidate_segment_key(candidate));
    state.choice_message = candidate_segment_key(candidate);
    return true;
}

// 将候选路径对应的 LCP owner packing step 追加到 traceback 说明。
void append_candidate_packing_trace(
    RoutingDpState& state,
    const RouteCandidate& candidate,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id,
    const std::unordered_map<std::string, std::string>& lcp_space_by_id,
    const PackingTraceIndex& trace_index) {
    if (candidate.lcp_id.empty()) return;
    const auto owner = lcp_owner_by_id.find(candidate.lcp_id);
    if (owner == lcp_owner_by_id.end()) return;
    const auto step = trace_index.index_by_node.find(owner->second);
    if (step == trace_index.index_by_node.end()) return;
    std::string transition = candidate_segment_key(candidate) + "@packing_step=" + std::to_string(step->second);
    const auto space = lcp_space_by_id.find(candidate.lcp_id);
    if (space != lcp_space_by_id.end()) transition += "@space=" + space->second;
    append_unique(
        state.selected_transitions,
        transition);
}

// 收集论文 LCP topology 中必须被 DP 覆盖的唯一 wire segment id。
std::vector<std::string> required_segment_keys(const RoutingEvaluationRequest& request) {
    std::vector<std::string> keys;
    for (const auto& topology : request.net_topologies) {
        for (const auto& segment : topology.segments) {
            append_unique(keys, segment_key(segment.net, segment.from, segment.to, segment.id));
        }
    }
    return keys;
}

// 收集必须在 DP state 中绑定到唯一物理候选位置的 LCP id。
std::vector<std::string> required_lcp_ids(const RoutingEvaluationRequest& request) {
    std::vector<std::string> ids;
    for (const auto& point : request.linking_points) append_unique(ids, point.id);
    return ids;
}

// 检查 root state 是否完整覆盖 topology 中的所有必需 segment。
bool state_covers_required_segments(const RoutingDpState& state, const std::vector<std::string>& required) {
    for (const auto& key : required) {
        if (!contains_value(state.covered_wire_segments, key)) return false;
    }
    return true;
}

// 检查每个 LCP 是否恰好被绑定到一个候选物理位置。
bool state_binds_required_lcps(const RoutingDpState& state, const std::vector<std::string>& required) {
    for (const auto& lcp_id : required) {
        const auto found = state.lcp_location_by_id.find(lcp_id);
        if (found == state.lcp_location_by_id.end() || found->second.empty()) return false;
    }
    return true;
}

// 根据论文语义判定 DP 是否成功：完整覆盖、LCP 位置一致且没有失败 transition。
bool is_successful_root_state(
    const RoutingDpState& state,
    const std::vector<std::string>& required_segments,
    const std::vector<std::string>& required_lcps) {
    if (!state.failure_messages.empty()) return false;
    if (required_segments.empty()) return !state.selected_candidates.empty();
    if (!state_covers_required_segments(state, required_segments)) return false;
    if (!state_binds_required_lcps(state, required_lcps)) return false;
    return true;
}

// 在 DP 未成功但没有底层 A* failure 时，补充 topology 覆盖或 LCP 绑定的具体失败原因。
void append_success_check_failures(
    RoutingDpState& state,
    const std::vector<std::string>& required_segments,
    const std::vector<std::string>& required_lcps) {
    if (!state.failure_messages.empty()) return;
    if (required_segments.empty() && state.selected_candidates.empty()) {
        append_unique(state.failure_messages, "bottom-up DP selected no traceback candidate");
        return;
    }
    for (const auto& key : required_segments) {
        if (!contains_value(state.covered_wire_segments, key)) {
            append_unique(state.failure_messages, "required topology segment was not covered: " + key);
        }
    }
    for (const auto& lcp_id : required_lcps) {
        const auto found = state.lcp_location_by_id.find(lcp_id);
        if (found == state.lcp_location_by_id.end() || found->second.empty()) {
            append_unique(state.failure_messages, "LCP has no unique physical candidate binding: " + lcp_id);
        }
    }
}

// 将所有候选按逻辑线段分组。
std::vector<CandidateGroup> make_candidate_groups(
    const RoutingEvaluationRequest& request,
    const std::vector<RouteCandidate>& candidates) {
    std::vector<CandidateGroup> groups;
    std::unordered_map<std::string, std::size_t> index_by_key;

    for (const auto& topology : request.net_topologies) {
        for (const auto& segment : topology.segments) {
            const std::string key = segment_key(segment.net, segment.from, segment.to, segment.id);
            if (index_by_key.contains(key)) continue;
            CandidateGroup group;
            group.key = key;
            group.net = segment.net;
            group.from = segment.from;
            group.to = segment.to;
            group.segment_id = segment.id;
            index_by_key[key] = groups.size();
            groups.push_back(std::move(group));
        }
    }

    for (const auto& candidate : candidates) {
        const std::string key = candidate_segment_key(candidate);
        auto found = index_by_key.find(key);
        if (found == index_by_key.end()) {
            CandidateGroup group;
            group.key = key;
            group.net = candidate.net;
            group.from = candidate.from_terminal;
            group.to = candidate.to_terminal;
            group.segment_id = candidate.segment_id;
            index_by_key[key] = groups.size();
            groups.push_back(std::move(group));
            found = index_by_key.find(key);
        }
        groups[found->second].candidates.push_back(candidate);
    }

    return groups;
}

// 根据 request.space_nodes 建立 LCP 所属模块查找表。
std::unordered_map<std::string, std::string> make_lcp_owner_map(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::string> result;
    std::unordered_map<std::string, std::string> owner_by_space;
    for (const auto& space : request.space_nodes) {
        owner_by_space[space.id] = space.owner;
        for (const auto& point : space.linking_points) result[point.id] = space.owner;
    }
    for (const auto& point : request.linking_points) {
        const auto found = owner_by_space.find(point.space_node_id);
        if (found != owner_by_space.end()) result[point.id] = found->second;
    }
    return result;
}

// 鏍规嵁 request.space_nodes 寤虹珛 LCP 鎵€灞?space node 鏌ユ壘琛紝渚?DP trace 璇存槑 feedback 鏉ユ簮銆?
std::unordered_map<std::string, std::string> make_lcp_space_map(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::string> result;
    for (const auto& space : request.space_nodes) {
        for (const auto& point : space.linking_points) result[point.id] = space.id;
    }
    for (const auto& point : request.linking_points) {
        if (!point.space_node_id.empty()) result[point.id] = point.space_node_id;
    }
    return result;
}

// 按代价剪枝并重新编号状态。
void prune_states(std::vector<RoutingDpState>& states, int max_states, int& pruned_states) {
    std::sort(states.begin(), states.end(), [](const auto& left, const auto& right) {
        if (left.cost != right.cost) return left.cost < right.cost;
        if (left.covered_wire_segments.size() != right.covered_wire_segments.size()) {
            return left.covered_wire_segments.size() > right.covered_wire_segments.size();
        }
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
// 从 packing trace 建立 node 到 contour step 的查找表。
PackingTraceIndex make_packing_trace_index(const PackingContourTrace& trace) {
    PackingTraceIndex index;
    for (std::size_t step = 0; step < trace.steps.size(); ++step) {
        const auto& item = trace.steps[step];
        index.step_by_node[item.tree_node] = &item;
        index.index_by_node[item.tree_node] = static_cast<int>(step);
    }
    return index;
}

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

// 判断逻辑线段的两个端点是否都落在指定模块集合里。
// 优先使用 packing contour trace 中记录的子树模块，否则回退到 B*-tree 递归收集。
std::unordered_set<std::string> modules_for_dp_step(
    const std::string& id,
    const PackingTraceIndex& trace_index,
    const std::unordered_map<std::string, RoutingTreeNodeRef>& nodes,
    std::unordered_map<std::string, std::unordered_set<std::string>>& cache) {
    const auto trace = trace_index.step_by_node.find(id);
    if (trace != trace_index.step_by_node.end()) {
        return {trace->second->subtree_modules.begin(), trace->second->subtree_modules.end()};
    }
    return collect_subtree_modules(id, nodes, cache);
}

bool segment_inside_modules(
    const CandidateGroup& group,
    const std::unordered_set<std::string>& modules,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const std::string from_owner = owner_for_terminal(group.from, lcp_owner_by_id);
    const std::string to_owner = owner_for_terminal(group.to, lcp_owner_by_id);
    return !from_owner.empty() && !to_owner.empty() && modules.contains(from_owner) && modules.contains(to_owner);
}

// 判断逻辑线段是否应在当前 node 的 child merge 阶段被补全。
bool is_local_segment_for_node(
    const CandidateGroup& group,
    const std::unordered_set<std::string>& current_modules,
    const ChildSubtreeModules& children,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    if (!segment_inside_modules(group, current_modules, lcp_owner_by_id)) return false;
    if (!children.left.empty() && segment_inside_modules(group, children.left, lcp_owner_by_id)) return false;
    if (!children.right.empty() && segment_inside_modules(group, children.right, lcp_owner_by_id)) return false;
    return true;
}

// 为当前 node 生成基础状态：先合并左右 child state，再把当前 module terminal 标记为已覆盖。
std::vector<RoutingDpState> merge_child_states_for_node(
    const RoutingTreeNodeRef& node,
    const std::unordered_map<std::string, NodeRoutingDpResult>& result_by_node,
    const PackingTraceIndex& trace_index) {
    std::vector<const RoutingDpState*> left_states{nullptr};
    std::vector<const RoutingDpState*> right_states{nullptr};
    if (node.left.has_value()) {
        left_states.clear();
        const auto found = result_by_node.find(*node.left);
        if (found != result_by_node.end()) {
            for (const auto& state : found->second.states) left_states.push_back(&state);
        }
    }
    if (node.right.has_value()) {
        right_states.clear();
        const auto found = result_by_node.find(*node.right);
        if (found != result_by_node.end()) {
            for (const auto& state : found->second.states) right_states.push_back(&state);
        }
    }

    std::vector<RoutingDpState> merged;
    for (const auto* left : left_states) {
        for (const auto* right : right_states) {
            RoutingDpState state;
            state.id = 0;
            state.tree_node = node.id;
            const auto trace_step = trace_index.step_by_node.find(node.id);
            if (trace_step != trace_index.step_by_node.end()) state.contour_y = trace_step->second->contour_y;
            const auto trace_index_it = trace_index.index_by_node.find(node.id);
            if (trace_index_it != trace_index.index_by_node.end()) state.packing_step_index = trace_index_it->second;
            if (left != nullptr && !merge_child_state(state, *left, true)) continue;
            if (right != nullptr && !merge_child_state(state, *right, false)) continue;
            append_unique(state.covered_terminals, node.module);
            merged.push_back(std::move(state));
        }
    }
    if (merged.empty() && !node.left.has_value() && !node.right.has_value()) {
        RoutingDpState state;
        state.tree_node = node.id;
        const auto trace_step = trace_index.step_by_node.find(node.id);
        if (trace_step != trace_index.step_by_node.end()) state.contour_y = trace_step->second->contour_y;
        const auto trace_index_it = trace_index.index_by_node.find(node.id);
        if (trace_index_it != trace_index.index_by_node.end()) state.packing_step_index = trace_index_it->second;
        append_unique(state.covered_terminals, node.module);
        merged.push_back(std::move(state));
    }
    return merged;
}

// 对当前 node 负责的一个 local segment 执行 A* candidate 选择 transition。
void apply_segment_transition(
    std::vector<RoutingDpState>& states,
    const CandidateGroup& group,
    const RoutingContext& context,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id,
    const std::unordered_map<std::string, std::string>& lcp_space_by_id,
    const PackingTraceIndex& trace_index,
    int max_states,
    int& pruned_states) {
    std::vector<RoutingDpState> next_states;
    for (const auto& state : states) {
        bool selected_any = false;
        for (const auto& candidate : group.candidates) {
            if (!candidate_matches_group(candidate, group)) continue;
            RoutingDpState next = state;
            if (!append_candidate_if_consistent(next, candidate, context)) continue;
            append_candidate_packing_trace(next, candidate, lcp_owner_by_id, lcp_space_by_id, trace_index);
            selected_any = true;
            next_states.push_back(std::move(next));
        }
        if (!selected_any) {
            RoutingDpState failed = state;
            failed.penalty += kMissingSegmentPenalty;
            append_unique(failed.covered_wire_segments, group.key);
            std::string message = "missing successful A* candidate for " + group.key;
            for (const auto& candidate : group.candidates) {
                if (!candidate_matches_group(candidate, group)) continue;
                if (!candidate.path.message.empty()) {
                    message += " [" + candidate.lcp_candidate_id + ": " + candidate.path.message + "]";
                }
            }
            append_unique(failed.failure_messages, message);
            failed.choice_message = "missing " + group.key;
            recompute_state_cost(failed);
            next_states.push_back(std::move(failed));
        }
    }
    prune_states(next_states, max_states, pruned_states);
    states = std::move(next_states);
}

// 生成一个 tree node 的 DP states。
std::vector<RoutingDpState> build_states_for_node(
    const RoutingTreeNodeRef& node,
    const std::unordered_map<std::string, NodeRoutingDpResult>& result_by_node,
    const std::unordered_set<std::string>& subtree_modules,
    const ChildSubtreeModules& children,
    const std::vector<CandidateGroup>& groups,
    const RoutingContext& context,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id,
    const std::unordered_map<std::string, std::string>& lcp_space_by_id,
    const PackingTraceIndex& trace_index,
    const std::vector<std::string>* packing_time_segments,
    bool use_packing_time_segments,
    int max_states,
    int& pruned_states) {
    auto states = merge_child_states_for_node(node, result_by_node, trace_index);
    if (states.empty()) return states;

    for (const auto& group : groups) {
        if (use_packing_time_segments) {
            if (packing_time_segments == nullptr || !contains_value(*packing_time_segments, group.key)) continue;
        } else if (!is_local_segment_for_node(group, subtree_modules, children, lcp_owner_by_id)) {
            continue;
        }
        bool already_covered = true;
        for (const auto& state : states) {
            if (std::find(state.covered_wire_segments.begin(), state.covered_wire_segments.end(), group.key) ==
                state.covered_wire_segments.end()) {
                already_covered = false;
                break;
            }
        }
        if (already_covered) continue;
        apply_segment_transition(
            states,
            group,
            context,
            lcp_owner_by_id,
            lcp_space_by_id,
            trace_index,
            max_states,
            pruned_states);
    }

    prune_states(states, max_states, pruned_states);
    return states;
}

// 从 root state 生成 traceback candidates，并按逻辑线段去重。
std::vector<RouteCandidate> traceback_candidates_from_state(const RoutingDpState& state) {
    std::vector<RouteCandidate> result;
    std::unordered_set<std::string> seen;
    for (const auto& candidate : state.selected_candidates) {
        const std::string key = candidate_segment_key(candidate);
        if (seen.insert(key).second) result.push_back(candidate);
    }
    return result;
}

}  // namespace

// 按 B*-tree post-order 运行 routing DP，并通过 child-state transition 选择一致的 A* traceback 路径。
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

    const auto required_segments = required_segment_keys(request);
    const auto required_lcps = required_lcp_ids(request);
    const auto groups = make_candidate_groups(request, candidates);
    const auto lcp_owner_by_id = make_lcp_owner_map(request);
    const auto lcp_space_by_id = make_lcp_space_map(request);
    const auto trace_index = make_packing_trace_index(request.packing_trace);
    for (const auto& step : request.packing_trace.steps) {
        result.packing_time_dp_segments += static_cast<int>(step.local_wire_segments.size());
    }
    result.packing_time_dp_used = result.packing_time_dp_segments > 0;
    std::unordered_map<std::string, std::unordered_set<std::string>> subtree_cache;
    std::unordered_map<std::string, NodeRoutingDpResult> result_by_node;

    for (const auto& node_id : post_order) {
        const auto node_it = nodes.find(node_id);
        if (node_it == nodes.end()) continue;
        const auto subtree_modules = modules_for_dp_step(node_id, trace_index, nodes, subtree_cache);
        ChildSubtreeModules children;
        if (node_it->second.left.has_value()) {
            children.left = modules_for_dp_step(*node_it->second.left, trace_index, nodes, subtree_cache);
        }
        if (node_it->second.right.has_value()) {
            children.right = modules_for_dp_step(*node_it->second.right, trace_index, nodes, subtree_cache);
        }

        NodeRoutingDpResult node_result;
        node_result.tree_node = node_id;
        const auto trace_step = trace_index.step_by_node.find(node_id);
        const auto* packing_time_segments =
            trace_step == trace_index.step_by_node.end() ? nullptr : &trace_step->second->local_wire_segments;
        node_result.states = build_states_for_node(
            node_it->second,
            result_by_node,
            subtree_modules,
            children,
            groups,
            context,
            lcp_owner_by_id,
            lcp_space_by_id,
            trace_index,
            packing_time_segments,
            result.packing_time_dp_used,
            max_states_per_node,
            result.dp_pruned_states);
        result.dp_nodes += 1;
        result.dp_states += static_cast<int>(node_result.states.size());
        result_by_node[node_id] = node_result;
        result.node_results.push_back(std::move(node_result));
    }

    auto root_found = result_by_node.find(*request.tree.root);
    if (root_found == result_by_node.end() || root_found->second.states.empty()) return result;
    result.best_state = root_found->second.states.front();
    result.traceback_candidates = traceback_candidates_from_state(result.best_state);
    result.success = is_successful_root_state(result.best_state, required_segments, required_lcps);
    if (!result.success) {
        append_success_check_failures(result.best_state, required_segments, required_lcps);
    }
    return result;
}

}  // namespace sapr::routing
