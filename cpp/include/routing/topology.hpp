// 文件职责：声明阶段 2 的候选布线拓扑生成接口。
#pragma once

#include <vector>

#include "core/model.hpp"
#include "routing/path.hpp"
#include "routing/routing_context.hpp"

namespace analog_sapr {

// 表示候选拓扑生成参数。
struct CandidateConfig {
    int max_candidates_per_pair = 3;
};

// 为所有 net 生成从第一个 terminal 到其它 terminal 的候选 A* 路径。
std::vector<RouteCandidate> generate_route_candidates(
    const Circuit& circuit,
    const RoutingContext& context,
    const CandidateConfig& config = CandidateConfig{});

}  // namespace analog_sapr
