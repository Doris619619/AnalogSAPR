// 文件职责：声明自动 LCP 拓扑生成、space 绑定和物理候选点刷新入口。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 初始生成 LCP 拓扑及其初始 space 归属；存在 placement 时同时生成物理候选点。
void generate_initial_lcp_topology(const Circuit& circuit, RoutingEvaluationRequest& request);

// 基于 request 中已有的 LCP/space 归属刷新物理候选点，不改变 LCP 拓扑或 space 归属。
void refresh_lcp_location_candidates(const Circuit& circuit, RoutingEvaluationRequest& request);

// 兼容历史入口：已有 LCP 时仅刷新候选点；否则生成初始 LCP。
void generate_automatic_lcps(const Circuit& circuit, RoutingEvaluationRequest& request);

}  // namespace sapr
