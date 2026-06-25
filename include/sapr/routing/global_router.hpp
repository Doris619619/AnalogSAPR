#pragma once

#include <string>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr::routing {

struct GlobalRouterConfig {
    double wirelength_weight{1.0};
    double bend_weight{3.0};
    double conflict_penalty_per_point{100.0};
    double failed_pair_penalty{100000.0};
};

struct NetRouteChoice {
    std::string net;
    std::vector<RouteCandidate> selected_candidates;
    PathMetrics metrics;
    double penalty{};
    bool success{true};
    std::string message;
};

struct GlobalRoutingResult {
    std::vector<NetRouteChoice> net_routes;
    PathMetrics total_metrics;
    double total_penalty{};
    int failed_nets{};
};

GlobalRoutingResult run_global_routing(
    const Circuit& circuit,
    const RoutingContext& context,
    const std::vector<RouteCandidate>& candidates,
    const GlobalRouterConfig& config = GlobalRouterConfig{});

}  // namespace sapr::routing
