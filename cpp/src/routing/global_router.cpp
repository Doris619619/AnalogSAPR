// 文件职责：实现阶段 3 的 DP-based global routing 候选选择逻辑。
#include "routing/global_router.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace analog_sapr {
namespace {

// 表示同一 net 的同一 terminal pair 候选集合。
struct CandidateGroup {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::vector<RouteCandidate> candidates;
};

// 将 net priority 转成排序权重。
int priority_rank(const std::string& priority) {
    if (priority == "critical") {
        return 0;
    }
    if (priority == "symmetry") {
        return 1;
    }
    return 2;
}

// 构造 terminal pair 分组 key。
std::string group_key(const RouteCandidate& candidate) {
    return candidate.net + "|" + candidate.from_terminal + "|" + candidate.to_terminal;
}

// 将网格点编码为占用表 key。
std::int64_t point_key(const GridPoint& point) {
    return ((static_cast<std::int64_t>(point.ix) * 1000003 + point.iy) * 31) + point.layer;
}

// 计算 DP 阶段候选路径 cost，不包含 via cost。
double candidate_dp_cost(const RouteCandidate& candidate, const GlobalRouterConfig& config) {
    return config.wirelength_weight * candidate.path.metrics.wirelength +
           config.bend_weight * static_cast<double>(candidate.path.metrics.bend_count);
}

// 将候选路径的指标累加到 net 指标中，via 只统计不参与 cost。
void add_metrics(PathMetrics& target, const PathMetrics& source, const GlobalRouterConfig& config) {
    target.wirelength += source.wirelength;
    target.bend_count += source.bend_count;
    target.via_count += source.via_count;
    target.cost = config.wirelength_weight * target.wirelength +
                  config.bend_weight * static_cast<double>(target.bend_count);
}

// 将候选列表按 terminal pair 分组。
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

// 按论文求解顺序对 net 排序，critical 优先，其次 symmetry，再 normal。
std::vector<const Net*> ordered_nets(const Circuit& circuit) {
    std::vector<const Net*> nets;
    for (const auto& [_, net] : circuit.nets) {
        if (net.terminals.size() >= 2) {
            nets.push_back(&net);
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

// 将选中的候选路径写入全局占用表。
void occupy_path(const RouteCandidate& candidate, std::unordered_map<std::int64_t, std::string>& occupied_by_net) {
    for (const auto& point : candidate.path.points) {
        occupied_by_net.emplace(point_key(point), candidate.net);
    }
}

// 从一个 terminal pair group 中选出 DP cost 最低的成功候选。
const RouteCandidate* choose_best_candidate(
    const CandidateGroup& group,
    const std::unordered_map<std::int64_t, std::string>& occupied_by_net,
    const GlobalRouterConfig& config,
    double& selected_penalty) {
    const RouteCandidate* best = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();
    selected_penalty = 0.0;

    for (const auto& candidate : group.candidates) {
        if (!candidate.path.success) {
            continue;
        }
        const double penalty = conflict_penalty(candidate, occupied_by_net, config);
        const double cost = candidate_dp_cost(candidate, config) + penalty;
        if (cost < best_cost) {
            best = &candidate;
            best_cost = cost;
            selected_penalty = penalty;
        }
    }
    return best;
}

// 拼接失败信息，便于 CLI 输出。
std::string append_message(const std::string& current, const std::string& next) {
    if (current.empty()) {
        return next;
    }
    return current + "; " + next;
}

}  // namespace

GlobalRoutingResult run_global_routing(
    const Circuit& circuit,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    const GlobalRouterConfig& config) {
    (void)context;

    GlobalRoutingResult result;
    std::unordered_map<std::int64_t, std::string> occupied_by_net;

    for (const Net* net : ordered_nets(circuit)) {
        NetRouteChoice choice;
        choice.net = net->name;
        choice.success = true;

        const auto groups = groups_for_net(net->name, candidates);
        if (groups.empty()) {
            choice.success = false;
            choice.penalty += config.failed_pair_penalty;
            choice.message = "没有候选 terminal pair";
        }

        for (const auto& group : groups) {
            double penalty = 0.0;
            const RouteCandidate* selected = choose_best_candidate(group, occupied_by_net, config, penalty);
            if (selected == nullptr) {
                choice.success = false;
                choice.penalty += config.failed_pair_penalty;
                choice.message = append_message(
                    choice.message,
                    group.from_terminal + " -> " + group.to_terminal + " 没有成功候选路径");
                continue;
            }

            choice.selected_candidates.push_back(*selected);
            choice.penalty += penalty;
            add_metrics(choice.metrics, selected->path.metrics, config);
        }

        choice.metrics.cost += choice.penalty;
        if (!choice.success) {
            ++result.failed_nets;
        } else {
            for (const auto& selected : choice.selected_candidates) {
                occupy_path(selected, occupied_by_net);
            }
        }

        result.total_penalty += choice.penalty;
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

}  // namespace analog_sapr
