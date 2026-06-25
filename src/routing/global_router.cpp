#include "sapr/routing/global_router.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace sapr::routing {
namespace {

struct CandidateGroup {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::vector<RouteCandidate> candidates;
};

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

std::string group_key(const RouteCandidate& candidate) {
    return candidate.net + "|" + candidate.from_terminal + "|" + candidate.to_terminal;
}

std::int64_t point_key(const GridPoint& point) {
    return ((static_cast<std::int64_t>(point.ix) * 1000003 + point.iy) * 31) + point.layer;
}

double candidate_dp_cost(const RouteCandidate& candidate, const GlobalRouterConfig& config) {
    return config.wirelength_weight * candidate.path.metrics.wirelength +
           config.bend_weight * static_cast<double>(candidate.path.metrics.bend_count);
}

void add_metrics(PathMetrics& target, const PathMetrics& source, const GlobalRouterConfig& config) {
    target.wirelength += source.wirelength;
    target.bend_count += source.bend_count;
    target.via_count += source.via_count;
    target.cost = config.wirelength_weight * target.wirelength +
                  config.bend_weight * static_cast<double>(target.bend_count);
}

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

void occupy_path(const RouteCandidate& candidate, std::unordered_map<std::int64_t, std::string>& occupied_by_net) {
    for (const auto& point : candidate.path.points) {
        occupied_by_net.emplace(point_key(point), candidate.net);
    }
}

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
            choice.message = "no terminal-pair candidates";
        }

        for (const auto& group : groups) {
            double penalty = 0.0;
            const RouteCandidate* selected = choose_best_candidate(group, occupied_by_net, config, penalty);
            if (selected == nullptr) {
                choice.success = false;
                choice.penalty += config.failed_pair_penalty;
                choice.message = append_message(
                    choice.message,
                    group.from_terminal + " -> " + group.to_terminal + " has no successful candidate");
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

}  // namespace sapr::routing
