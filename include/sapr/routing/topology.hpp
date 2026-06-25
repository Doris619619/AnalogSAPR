#pragma once

#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr::routing {

struct CandidateConfig {
    int max_candidates_per_pair{3};
};

std::vector<RouteCandidate> generate_route_candidates(
    const Circuit& circuit,
    const RoutingContext& context,
    const CandidateConfig& config = CandidateConfig{});

}  // namespace sapr::routing
