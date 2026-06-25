// 文件职责：声明从 net terminal 生成 A* 候选拓扑路径的接口。
#pragma once

#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr::routing {

// 表示每对 terminal 最多保留的候选路径数量。
struct CandidateConfig {
    int max_candidates_per_pair{3};
};

// 为所有多端 net 生成 root terminal 到其它 terminal 的候选路径。
std::vector<RouteCandidate> generate_route_candidates(
    const Circuit& circuit,
    const RoutingContext& context,
    const CandidateConfig& config = CandidateConfig{});

}  // namespace sapr::routing
