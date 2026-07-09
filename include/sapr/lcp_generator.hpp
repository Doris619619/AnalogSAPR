// 文件职责：声明自动 LCP 拓扑生成、space 绑定和物理候选点刷新入口。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 初始生成 placement-aware LCP 拓扑、space 归属和候选点，并写入 routing request。
void generate_initial_lcp_topology(const Circuit& circuit, RoutingEvaluationRequest& request);

// 基于 request 中已有的 LCP/space 归属刷新物理候选点，不改变 LCP 拓扑。
void refresh_lcp_location_candidates(const Circuit& circuit, RoutingEvaluationRequest& request);

// 保持历史入口名称，等价于初始生成 LCP 拓扑。
void generate_automatic_lcps(const Circuit& circuit, RoutingEvaluationRequest& request);

}  // namespace sapr
