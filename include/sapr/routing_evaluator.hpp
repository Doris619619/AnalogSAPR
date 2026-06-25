#pragma once

#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr {

struct RoutingEvaluation {
    routing::RoutingContext context;
    std::vector<routing::RouteCandidate> candidates;
    routing::GlobalRoutingResult global_routing;
    double routing_cost{};
    int failed_nets{};
};

RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements);

}  // namespace sapr
