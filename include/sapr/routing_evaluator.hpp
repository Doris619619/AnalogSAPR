// 鏂囦欢鑱岃矗锛氬０鏄庝緵甯冨眬銆佹ā鎷熼€€鐏拰 CLI 璋冪敤鐨勫竷绾胯瘎浼板叕寮€鎺ュ彛銆?
#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/dp_router.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/path.hpp"
#include "sapr/routing/routing_context.hpp"

namespace sapr {

// 记录 LCP 多端覆盖检查删除 A* 候选的具体位置与原因。
struct LcpCandidateFilterEvent {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::string segment_id;
    std::string lcp_candidate_id;
    std::string source_lcp_candidate_id;
    std::string target_lcp_candidate_id;
    std::string reason;
};

// 姹囨€讳竴娆?placement 甯冪嚎璇勪及浜х敓鐨勪笂涓嬫枃銆佸€欓€夎矾寰勫拰鍏ㄥ眬甯冪嚎缁撴灉銆?
struct RoutingEvaluation {
    routing::RoutingContext context;
    std::vector<routing::RouteCandidate> candidates;
    routing::GlobalRoutingResult global_routing;
    std::optional<routing::RoutingDpResult> bottom_up_dp;
    double routing_cost{};
    int failed_nets{};
    bool used_bottom_up_dp{};
    std::vector<routing::RouteCandidate> debug_candidates;
    bool strict_lcp_dp_blocked_fallback{};
    // 记录 LCP 多端覆盖过滤使候选失效的原因，供 routing_debug.json 定位 DP 前的删除。
    std::vector<LcpCandidateFilterEvent> lcp_candidate_filter_events;
};


// 表示一个 LCP 物理候选点对其所有 incident segments 的覆盖情况。
struct LcpCandidateCoverage {
    std::string lcp_id;
    std::string candidate_id;
    std::vector<std::string> required_segments;
    std::vector<std::string> reachable_segments;
    std::vector<std::string> missing_segments;
    bool covers_all_incident_segments{};
};

// 根据当前 placement 构建布线环境、生成 A* 候选路径并执行 DP 全局布线。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements);

// 根据 placement candidate 中的 LCP 拓扑执行 A*/DP 布线评估。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request);

// 将 DP 全局布线选中的 A* 网格路径转换为当前 routing.txt 使用的中心线线段。
std::vector<RouteSegment> selected_candidates_to_segments(
    const RoutingEvaluation& evaluation);

// 执行论文 top-down detailed routing 阶段，当前基于 DP 选中子问题回溯并清理路径。
DetailedRoutingResult run_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation);

// 统计每个 LCP 物理候选点是否能覆盖该 LCP 的全部 incident segments。
std::vector<LcpCandidateCoverage> analyze_lcp_candidate_coverage(
    const RoutingEvaluationRequest& request,
    const std::vector<routing::RouteCandidate>& candidates);

}  // namespace sapr
