#include "sapr/routing_evaluator.hpp"

#include <utility>

#include "sapr/routing/topology.hpp"

namespace sapr {

RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements) {
    routing::RoutingContext context(circuit, placements);
    auto candidates = routing::generate_route_candidates(circuit, context);
    auto global_routing = routing::run_global_routing(circuit, context, candidates);

    RoutingEvaluation evaluation{
        std::move(context),
        std::move(candidates),
        std::move(global_routing),
        0.0,
        0,
    };
    evaluation.routing_cost = evaluation.global_routing.total_metrics.cost;
    evaluation.failed_nets = evaluation.global_routing.failed_nets;
    return evaluation;
}

}  // namespace sapr
