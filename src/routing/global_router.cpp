// 文件职责：实现不计 via cost 的 DP-based global routing 选择逻辑。
#include "sapr/routing/global_router.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "sapr/routing/path_geometry.hpp"

namespace sapr::routing {
namespace {

// 表示同一 net 的同一 terminal pair 候选集合。
struct CandidateGroup {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::vector<RouteCandidate> candidates;
};

// 表示一条 net 在 LCP 位置一致性约束下的一组候选选择。
struct ConsistentSelection {
    std::vector<RouteCandidate> candidates;
    std::vector<double> conflict_penalties;
    double cost{std::numeric_limits<double>::infinity()};
};

// 将 net 优先级转换为排序权重。
int priority_rank(Priority priority) {
    switch (priority) {
        case Priority::Critical:
            return 0;
        case Priority::Symmetry:
            return 1;
        case Priority::Normal:
            return 2;
    }
    return 2;
}

// 构造 terminal pair 分组 key。
std::string group_key(const RouteCandidate& candidate) {
    return candidate.net + "|" + candidate.from_terminal + "|" + candidate.to_terminal;
}

// 将网格点编码为全局占用表 key。
std::int64_t point_key(const GridPoint& point) {
    return ((static_cast<std::int64_t>(point.ix) * 1000003 + point.iy) * 31) + point.layer;
}

// 计算候选路径的 DP 代价，不包含 via cost。
double candidate_dp_cost(const RouteCandidate& candidate, const GlobalRouterConfig& config) {
    return config.wirelength_weight * candidate.path.metrics.wirelength +
           config.bend_weight * static_cast<double>(candidate.path.metrics.bend_count) +
           candidate.flow_penalty + candidate.current_density_penalty + candidate.coupling_cost;
}

// 将路径指标累加到 net 指标中，via 只统计不进入 cost。
void add_metrics(PathMetrics& target, const PathMetrics& source, const GlobalRouterConfig& config) {
    target.wirelength += source.wirelength;
    target.bend_count += source.bend_count;
    target.via_count += source.via_count;
    target.cost = config.wirelength_weight * target.wirelength +
                  config.bend_weight * static_cast<double>(target.bend_count);
}

// 将候选路径按 net 和 terminal pair 分组。
std::vector<CandidateGroup> groups_for_net(const std::string& net_name, const std::vector<RouteCandidate>& candidates) {
    std::vector<CandidateGroup> groups;
    std::unordered_map<std::string, std::size_t> index_by_key;

    for (const auto& candidate : candidates) {
        if (candidate.net != net_name) {
            continue;
        }
        const std::string key = group_key(candidate);
        const auto index_it = index_by_key.find(key);
        if (index_it == index_by_key.end()) {
            CandidateGroup group;
            group.net = candidate.net;
            group.from_terminal = candidate.from_terminal;
            group.to_terminal = candidate.to_terminal;
            group.candidates.push_back(candidate);
            index_by_key[key] = groups.size();
            groups.push_back(group);
        } else {
            groups[index_it->second].candidates.push_back(candidate);
        }
    }
    return groups;
}

// 按 priority 和输入顺序返回需要布线的 net。
std::vector<const Net*> ordered_nets(const Circuit& circuit) {
    std::vector<const Net*> nets;
    if (!circuit.net_order.empty()) {
        for (const auto& net_name : circuit.net_order) {
            const auto net_it = circuit.nets.find(net_name);
            if (net_it != circuit.nets.end() && net_it->second.terminals.size() >= 2) {
                nets.push_back(&net_it->second);
            }
        }
    } else {
        for (const auto& [_, net] : circuit.nets) {
            if (net.terminals.size() >= 2) {
                nets.push_back(&net);
            }
        }
    }
    std::stable_sort(nets.begin(), nets.end(), [](const Net* lhs, const Net* rhs) {
        return priority_rank(lhs->priority) < priority_rank(rhs->priority);
    });
    return nets;
}

// 计算候选路径与已选其它 net 路径的网格点冲突惩罚。
double conflict_penalty(
    const RouteCandidate& candidate,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net,
    const GlobalRouterConfig& config) {
    double penalty = 0.0;
    for (const auto& point : candidate.path.points) {
        const auto occupied_it = occupied_by_net.find(point_key(point));
        if (occupied_it != occupied_by_net.end() && occupied_it->second != candidate.net) {
            penalty += config.conflict_penalty_per_point;
        }
    }
    return penalty;
}

// 判断候选路径是否与已选异网路径共用网格点，避免把耦合风险放大成真实短路。
// 按 detailed routing 的金属矩形语义判断候选是否会与既有异网短路。
bool shorts_with_existing_routes(
    const RouteCandidate& candidate,
    const RoutingContext& context,
    const std::vector<RouteSegment>& occupied_routes) {
    const double width = candidate.wire_width > 0.0 ? candidate.wire_width : context.default_width_for_net(candidate.net);
    const auto routes = candidate_to_route_segments(context.grid(), candidate, width);
    return routes_short_with_existing(routes, occupied_routes);
}

bool conflicts_with_other_net(
    const RouteCandidate& candidate,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net) {
    for (const auto& point : candidate.path.points) {
        const auto occupied_it = occupied_by_net.find(point_key(point));
        if (occupied_it != occupied_by_net.end() && occupied_it->second != candidate.net) {
            return true;
        }
    }
    return false;
}

// 将选中的候选路径写入全局占用表。
void occupy_path(const RouteCandidate& candidate, std::unordered_map<std::int64_t, std::string>& occupied_by_net) {
    for (const auto& point : candidate.path.points) {
        occupied_by_net.emplace(point_key(point), candidate.net);
    }
}

// 从一个 terminal pair group 中选择 DP cost 最低的成功候选。
// 将选中候选的金属占用追加到全局短路检查表。
void occupy_route_segments(
    const RouteCandidate& candidate,
    const RoutingContext& context,
    std::vector<RouteSegment>& occupied_routes) {
    const double width = candidate.wire_width > 0.0 ? candidate.wire_width : context.default_width_for_net(candidate.net);
    auto routes = candidate_to_route_segments(context.grid(), candidate, width);
    occupied_routes.insert(occupied_routes.end(), routes.begin(), routes.end());
}

const RouteCandidate* choose_best_candidate(
    const CandidateGroup& group,
    const RoutingContext& context,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net,
    const std::vector<RouteSegment>& occupied_routes,
    const GlobalRouterConfig& config,
    double& selected_penalty) {
    const RouteCandidate* best = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();
    bool best_has_short = true;
    bool best_has_conflict = true;
    selected_penalty = 0.0;

    for (const auto& candidate : group.candidates) {
        if (!candidate.path.success) {
            continue;
        }
        const bool has_short = shorts_with_existing_routes(candidate, context, occupied_routes);
        const bool has_conflict = conflicts_with_other_net(candidate, occupied_by_net);
        const double penalty = conflict_penalty(candidate, occupied_by_net, config) +
                               (has_short ? config.short_conflict_penalty : 0.0);
        const double cost = candidate_dp_cost(candidate, config) + penalty;
        if ((best_has_short && !has_short) ||
            (has_short == best_has_short && best_has_conflict && !has_conflict) ||
            (has_short == best_has_short && has_conflict == best_has_conflict && cost < best_cost)) {
            best = &candidate;
            best_cost = cost;
            best_has_short = has_short;
            best_has_conflict = has_conflict;
            selected_penalty = penalty;
        }
    }
    return best;
}

// 判断一个 net 的候选组是否包含 LCP 多位置选择。
bool has_lcp_candidates(const std::vector<CandidateGroup>& groups) {
    for (const auto& group : groups) {
        for (const auto& candidate : group.candidates) {
            if (!candidate.lcp_id.empty() || !candidate.source_lcp_id.empty() || !candidate.target_lcp_id.empty()) return true;
        }
    }
    return false;
}

// 递归枚举同一 net 的候选组合，并强制同一 LCP 使用同一个物理候选位置。
void search_consistent_lcp_selection(
    const std::vector<CandidateGroup>& groups,
    std::size_t group_index,
    const RoutingContext& context,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net,
    const std::vector<RouteSegment>& occupied_routes,
    const GlobalRouterConfig& config,
    std::unordered_map<std::string, std::string>& assigned_location_by_lcp,
    std::vector<RouteCandidate>& current_candidates,
    std::vector<double>& current_conflict_penalties,
    double current_cost,
    ConsistentSelection& best) {
    if (current_cost >= best.cost) return;
    if (group_index == groups.size()) {
        best.candidates = current_candidates;
        best.conflict_penalties = current_conflict_penalties;
        best.cost = current_cost;
        return;
    }

    const auto& group = groups[group_index];
    for (const auto& candidate : group.candidates) {
        if (!candidate.path.success) continue;

        std::vector<std::string> assigned_here;
        auto bind_lcp = [&](const std::string& lcp_id, const std::string& candidate_id) {
            if (lcp_id.empty()) return true;
            const auto assigned = assigned_location_by_lcp.find(lcp_id);
            if (assigned != assigned_location_by_lcp.end()) return assigned->second == candidate_id;
            assigned_location_by_lcp[lcp_id] = candidate_id;
            assigned_here.push_back(lcp_id);
            return true;
        };
        if (!bind_lcp(candidate.lcp_id, candidate.lcp_candidate_id) ||
            !bind_lcp(candidate.source_lcp_id, candidate.source_lcp_candidate_id) ||
            !bind_lcp(candidate.target_lcp_id, candidate.target_lcp_candidate_id)) {
            for (const auto& lcp_id : assigned_here) assigned_location_by_lcp.erase(lcp_id);
            continue;
        }

        const bool has_short = shorts_with_existing_routes(candidate, context, occupied_routes);
        const double penalty = conflict_penalty(candidate, occupied_by_net, config) +
                               (has_short ? config.short_conflict_penalty : 0.0);
        current_candidates.push_back(candidate);
        current_conflict_penalties.push_back(penalty);
        search_consistent_lcp_selection(
            groups,
            group_index + 1,
            context,
            occupied_by_net,
            occupied_routes,
            config,
            assigned_location_by_lcp,
            current_candidates,
            current_conflict_penalties,
            current_cost + candidate_dp_cost(candidate, config) + penalty,
            best);
        current_conflict_penalties.pop_back();
        current_candidates.pop_back();

        for (const auto& lcp_id : assigned_here) assigned_location_by_lcp.erase(lcp_id);
    }
}

// 选择满足 LCP 位置一致性的最低代价候选组合。
ConsistentSelection choose_consistent_lcp_selection(
    const std::vector<CandidateGroup>& groups,
    const RoutingContext& context,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net,
    const std::vector<RouteSegment>& occupied_routes,
    const GlobalRouterConfig& config) {
    ConsistentSelection best;
    std::unordered_map<std::string, std::string> assigned_location_by_lcp;
    std::vector<RouteCandidate> current_candidates;
    std::vector<double> current_conflict_penalties;
    search_consistent_lcp_selection(
        groups,
        0,
        context,
        occupied_by_net,
        occupied_routes,
        config,
        assigned_location_by_lcp,
        current_candidates,
        current_conflict_penalties,
        0.0,
        best);
    return best;
}

// 拼接失败信息，方便 CLI 或测试输出。
std::string append_message(const std::string& current, const std::string& next) {
    if (current.empty()) {
        return next;
    }
    return current + "; " + next;
}

}  // namespace

// 为所有 net 运行全局路径选择并汇总线长、拐弯、via 和惩罚。
GlobalRoutingResult run_global_routing(
    const Circuit& circuit,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    const GlobalRouterConfig& config) {
    GlobalRoutingResult result;
    std::unordered_map<std::int64_t, std::string> occupied_by_net;
    std::vector<RouteSegment> occupied_routes;

    for (const Net* net : ordered_nets(circuit)) {
        NetRouteChoice choice;
        choice.net = net->name;
        choice.success = true;

        const auto groups = groups_for_net(net->name, candidates);
        if (groups.empty()) {
            choice.success = false;
            choice.routing_failure_penalty += config.failed_pair_penalty;
            choice.penalty += config.failed_pair_penalty;
            choice.message = "no terminal-pair candidates";
        }

        if (!groups.empty() && has_lcp_candidates(groups)) {
            const auto selection = choose_consistent_lcp_selection(groups, context, occupied_by_net, occupied_routes, config);
            if (selection.candidates.size() != groups.size()) {
                choice.success = false;
                choice.routing_failure_penalty += config.failed_pair_penalty;
                choice.penalty += config.failed_pair_penalty;
                choice.message = append_message(
                    choice.message,
                    "no consistent LCP location assignment");
            } else {
                for (std::size_t index = 0; index < selection.candidates.size(); ++index) {
                    const auto& selected = selection.candidates[index];
                    const double penalty = selection.conflict_penalties[index];
                    choice.selected_candidates.push_back(selected);
                    choice.flow_penalty += selected.flow_penalty;
                    choice.current_density_penalty += selected.current_density_penalty;
                    choice.coupling_penalty += penalty + selected.coupling_cost;
                    choice.penalty += penalty;
                    choice.penalty += selected.flow_penalty + selected.current_density_penalty + selected.coupling_cost;
                    add_metrics(choice.metrics, selected.path.metrics, config);
                }
            }
        } else {
            for (const auto& group : groups) {
                double penalty = 0.0;
                const RouteCandidate* selected =
                    choose_best_candidate(group, context, occupied_by_net, occupied_routes, config, penalty);
                if (selected == nullptr) {
                    choice.success = false;
                    choice.routing_failure_penalty += config.failed_pair_penalty;
                    choice.penalty += config.failed_pair_penalty;
                    choice.message = append_message(
                        choice.message,
                        group.from_terminal + " -> " + group.to_terminal + " has no successful candidate");
                    continue;
                }

                choice.selected_candidates.push_back(*selected);
                choice.flow_penalty += selected->flow_penalty;
                choice.current_density_penalty += selected->current_density_penalty;
                choice.coupling_penalty += penalty + selected->coupling_cost;
                choice.penalty += penalty;
                choice.penalty += selected->flow_penalty + selected->current_density_penalty + selected->coupling_cost;
                add_metrics(choice.metrics, selected->path.metrics, config);
            }
        }

        choice.metrics.cost += choice.penalty;
        if (!choice.success) {
            ++result.failed_nets;
        } else {
            for (const auto& selected : choice.selected_candidates) {
                occupy_path(selected, occupied_by_net);
                occupy_route_segments(selected, context, occupied_routes);
            }
        }

        result.total_penalty += choice.penalty;
        result.flow_penalty += choice.flow_penalty;
        result.current_density_penalty += choice.current_density_penalty;
        result.coupling_penalty += choice.coupling_penalty;
        result.routing_failure_penalty += choice.routing_failure_penalty;
        result.total_metrics.wirelength += choice.metrics.wirelength;
        result.total_metrics.bend_count += choice.metrics.bend_count;
        result.total_metrics.via_count += choice.metrics.via_count;
        result.net_routes.push_back(choice);
    }

    result.total_metrics.cost = config.wirelength_weight * result.total_metrics.wirelength +
                                config.bend_weight * static_cast<double>(result.total_metrics.bend_count) +
                                result.total_penalty;
    return result;
}

}  // namespace sapr::routing
