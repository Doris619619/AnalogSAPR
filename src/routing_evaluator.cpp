// 鏂囦欢鑱岃矗锛氬疄鐜?placement 鍒?A*/DP 甯冪嚎璇勪及缁撴灉鐨勫皝瑁呮祦绋嬨€?
#include "sapr/routing_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "sapr/routing/astar.hpp"
#include "sapr/routing/geometry.hpp"
#include "sapr/routing/layer.hpp"
#include "sapr/routing/path_geometry.hpp"
#include "sapr/routing/topology.hpp"

namespace sapr {
namespace {

constexpr double kDetailedFailurePenalty = 100000.0;
constexpr double kDetailedSpacing = 1.0;
constexpr double kDetailedCouplingPenaltyPerPair = 100.0;
constexpr double kDetailedFlowPenalty = 50000.0;
constexpr double kDetailedCurrentDensityPenalty = 50000.0;
constexpr double kDetailedViaWeight = 5.0;

// 姹囨€?LCP銆乻pace node 鍜屾嫇鎵?segment 鐨勫揩閫熸煡鎵捐〃銆?
struct DetailedTopologyIndex {
    std::unordered_map<std::string, LinkingControlPoint> lcp_by_id;
    std::unordered_map<std::string, std::string> lcp_space_by_id;
    std::unordered_map<std::string, std::string> space_owner_by_id;
    std::unordered_set<std::string> lcp_without_location;
    std::unordered_set<std::string> topology_segment_keys;
    std::vector<WireSegmentRef> topology_segments;
};

// 鍚堝苟 DP traceback 鍊欓€夊拰鏈 DP 瑕嗙洊 net 鐨勬櫘閫氬€欓€夛紝閬垮厤 direct net 鍦?DP 鎴愬姛鍚庝涪澶便€?
std::vector<routing::RouteCandidate> merge_dp_traceback_with_uncovered_nets(
    const std::vector<routing::RouteCandidate>& all_candidates,
    const std::vector<routing::RouteCandidate>& traceback_candidates) {
    std::vector<routing::RouteCandidate> merged = traceback_candidates;
    std::unordered_set<std::string> covered_nets;
    for (const auto& candidate : traceback_candidates) covered_nets.insert(candidate.net);
    for (const auto& candidate : all_candidates) {
        if (!covered_nets.contains(candidate.net)) merged.push_back(candidate);
    }
    return merged;
}

// 鎵ц鍊欓€夎矾寰勭敓鎴愪笌 DP 鍏ㄥ眬甯冪嚎閫夋嫨鐨勫叕鍏卞皝瑁呫€?
RoutingEvaluation make_evaluation(
    routing::RoutingContext context,
    std::vector<routing::RouteCandidate> candidates,
    const Circuit& circuit,
    std::optional<routing::RoutingDpResult> bottom_up_dp = std::nullopt,
    std::vector<routing::RouteCandidate> debug_candidates = {},
    bool strict_lcp_dp_blocked_fallback = false,
    std::vector<LcpCandidateFilterEvent> lcp_candidate_filter_events = {}) {
    const bool use_dp = bottom_up_dp.has_value() && bottom_up_dp->success;
    const auto routing_candidates = use_dp
                                        ? merge_dp_traceback_with_uncovered_nets(candidates, bottom_up_dp->traceback_candidates)
                                        : candidates;
    auto global_routing = routing::run_global_routing(circuit, context, routing_candidates);
    if (debug_candidates.empty()) debug_candidates = candidates;
    const double routing_cost = global_routing.total_metrics.cost;
    const int failed_nets = global_routing.failed_nets;
    // 直接构造返回值，避免 Debug 构建移动含调试迭代器的候选容器。
    return RoutingEvaluation{
        std::move(context),
        std::move(candidates),
        std::move(global_routing),
        std::move(bottom_up_dp),
        routing_cost,
        failed_nets,
        use_dp,
        std::move(debug_candidates),
        strict_lcp_dp_blocked_fallback,
        std::move(lcp_candidate_filter_events),
    };
}

// 杩斿洖褰撳墠 request 涓甫 LCP 鎷撴墤鐨?net 闆嗗悎銆?
std::unordered_set<std::string> nets_with_lcp_topology(const RoutingEvaluationRequest& request) {
    std::unordered_set<std::string> result;
    for (const auto& point : request.linking_points) {
        for (const auto& segment : point.segments) result.insert(segment.net);
    }
    return result;
}

// 鎶?LCP 鐨勮繛缁€欓€変綅缃浆鎹负鎸囧畾灞備笂鐨勭綉鏍肩偣銆?
// 鏀堕泦 LCP 鐗╃悊鍊欓€変綅缃紝璁?routing grid 瑕嗙洊璁烘枃 DP 瑕佹悳绱㈢殑 resource point銆?
std::vector<routing::Point> lcp_routing_points(const RoutingEvaluationRequest& request) {
    std::vector<routing::Point> points;
    for (const auto& point : request.linking_points) {
        for (const auto& location : point.location_candidates) {
            points.push_back(routing::Point{location.x, location.y});
        }
    }
    return points;
}

routing::GridPoint lcp_grid_point(
    const routing::RoutingContext& context,
    const PhysicalLocationCandidate& location,
    int layer) {
    return context.grid().snap_to_grid(routing::Point{location.x, location.y}, layer);
}

// 鏌ユ壘鏅€?pin terminal 鐨勫叏灞€缃戞牸鐐广€?
std::optional<routing::GridPoint> pin_grid_point(
    const routing::RoutingContext& context,
    const std::string& terminal) {
    const auto found = context.global_pins().find(terminal);
    if (found == context.global_pins().end()) return std::nullopt;
    return context.grid().snap_to_grid(found->second.location, found->second.layer);
}

// 閫夋嫨 LCP segment 鐨勬悳绱㈠眰锛屼紭鍏堟部鐢ㄥ彟涓€绔?pin 鎵€鍦ㄥ眰銆?
int layer_for_lcp_segment(
    const routing::RoutingContext& context,
    const WireSegmentRef& segment) {
    const auto from_pin = context.global_pins().find(segment.from);
    if (from_pin != context.global_pins().end()) return from_pin->second.layer;
    const auto to_pin = context.global_pins().find(segment.to);
    if (to_pin != context.global_pins().end()) return to_pin->second.layer;
    return 0;
}

// 灏?LCP segment endpoint 瑙ｆ瀽涓?A* 鍙敤鐨勭綉鏍肩偣銆?
// 鏀寔 pin-LCP 涓?LCP-LCP segment锛屽皢浠绘剰 endpoint 瑙ｆ瀽鍒板綋鍓嶅€欓€夌粦瀹氫笅鐨勭綉鏍肩偣銆?
std::optional<routing::GridPoint> endpoint_grid_point(
    const routing::RoutingContext& context,
    const std::unordered_map<std::string, LinkingControlPoint>& lcp_by_id,
    const std::unordered_map<std::string, PhysicalLocationCandidate>& location_by_lcp,
    const WireSegmentRef& segment,
    const std::string& endpoint) {
    const auto lcp = lcp_by_id.find(endpoint);
    if (lcp != lcp_by_id.end()) {
        const auto location = location_by_lcp.find(endpoint);
        if (location == location_by_lcp.end()) return std::nullopt;
        return lcp_grid_point(context, location->second, layer_for_lcp_segment(context, segment));
    }
    return pin_grid_point(context, endpoint);
}

// 杩斿洖鍊欓€変腑 endpoint 瀵瑰簲鐨?LCP 鍊欓€変綅缃?id锛沺in endpoint 杩斿洖绌恒€?
std::string lcp_candidate_for_endpoint(
    const std::unordered_map<std::string, PhysicalLocationCandidate>& location_by_lcp,
    const std::string& endpoint) {
    const auto found = location_by_lcp.find(endpoint);
    return found == location_by_lcp.end() ? std::string{} : found->second.id;
}

// 鍦ㄨ繍琛?A* 鍓嶄及璁?LCP-LCP 鍊欓€夌粍鍚堜唬浠凤紝鐢ㄤ簬 top-K 鎴柇銆?
double endpoint_pair_distance(
    const PhysicalLocationCandidate& left,
    const PhysicalLocationCandidate& right) {
    return std::abs(left.x - right.x) + std::abs(left.y - right.y) + left.penalty + right.penalty;
}

// 表示一个 LCP-LCP segment 两端物理位置的组合候选。
struct LocationBinding {
    std::unordered_map<std::string, PhysicalLocationCandidate> by_lcp;
    double estimate{};
};

// 生成物理位置组合的去重键，避免 coverage 补齐时重复加入同一组 LCP 位置。
std::string location_binding_key(
    const LocationBinding& binding,
    const std::string& first_lcp,
    const std::string& second_lcp) {
    return lcp_candidate_for_endpoint(binding.by_lcp, first_lcp) + "|" +
           lcp_candidate_for_endpoint(binding.by_lcp, second_lcp);
}

// 为指定 LCP 物理位置查找代价最低的组合候选。
std::optional<std::size_t> best_binding_for_location(
    const std::vector<LocationBinding>& bindings,
    const std::string& lcp_id,
    const std::string& location_id) {
    std::optional<std::size_t> best;
    for (std::size_t index = 0; index < bindings.size(); ++index) {
        if (lcp_candidate_for_endpoint(bindings[index].by_lcp, lcp_id) != location_id) continue;
        if (!best.has_value() || bindings[index].estimate < bindings[*best].estimate) best = index;
    }
    return best;
}

// 在保留低代价组合的同时补齐两端 LCP location 覆盖，避免 multi-terminal root 被 top-K 剪枝误判不可达。
std::vector<LocationBinding> select_coverage_aware_bindings(
    const std::vector<LocationBinding>& sorted_bindings,
    const LinkingControlPoint& first,
    const LinkingControlPoint& second,
    std::size_t base_cap,
    std::size_t max_cap) {
    std::vector<std::size_t> selected_indices;
    std::vector<bool> required_indices;
    std::unordered_map<std::string, std::size_t> selected_by_key;

    const auto add_index = [&](std::size_t index, bool required) {
        const std::string key = location_binding_key(sorted_bindings[index], first.id, second.id);
        const auto found = selected_by_key.find(key);
        if (found != selected_by_key.end()) {
            if (required) required_indices[found->second] = true;
            return;
        }
        selected_by_key[key] = selected_indices.size();
        selected_indices.push_back(index);
        required_indices.push_back(required);
    };

    const std::size_t low_cost_count = std::min(base_cap, sorted_bindings.size());
    for (std::size_t index = 0; index < low_cost_count; ++index) add_index(index, false);

    for (const auto& location : first.location_candidates) {
        const auto best = best_binding_for_location(sorted_bindings, first.id, location.id);
        if (best.has_value()) add_index(*best, true);
    }
    for (const auto& location : second.location_candidates) {
        const auto best = best_binding_for_location(sorted_bindings, second.id, location.id);
        if (best.has_value()) add_index(*best, true);
    }

    std::vector<std::size_t> required;
    std::vector<std::size_t> optional;
    for (std::size_t index = 0; index < selected_indices.size(); ++index) {
        (required_indices[index] ? required : optional).push_back(selected_indices[index]);
    }
    const auto by_estimate = [&](std::size_t lhs, std::size_t rhs) {
        if (sorted_bindings[lhs].estimate != sorted_bindings[rhs].estimate) {
            return sorted_bindings[lhs].estimate < sorted_bindings[rhs].estimate;
        }
        return location_binding_key(sorted_bindings[lhs], first.id, second.id) <
               location_binding_key(sorted_bindings[rhs], first.id, second.id);
    };
    std::sort(required.begin(), required.end(), by_estimate);
    std::sort(optional.begin(), optional.end(), by_estimate);

    std::vector<std::size_t> final_indices = required;
    for (const auto index : optional) {
        if (final_indices.size() >= max_cap) break;
        final_indices.push_back(index);
    }
    std::sort(final_indices.begin(), final_indices.end(), by_estimate);

    std::vector<LocationBinding> result;
    result.reserve(final_indices.size());
    for (const auto index : final_indices) result.push_back(sorted_bindings[index]);
    return result;
}
// 鏌ユ壘 net 鐨?FLOW 绾︽潫銆?
std::optional<FlowConstraint> flow_for_net(const Circuit& circuit, const std::string& net) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net == net) return flow;
    }
    return std::nullopt;
}

// 鍒ゆ柇鍊欓€夋嫇鎵戞柟鍚戞槸鍚︽弧瓒?FLOW 閫昏緫鏂瑰悜銆?
bool flow_ok_for_candidate(
    const Circuit& circuit,
    const routing::RouteCandidate& candidate,
    const std::optional<WireSegmentRef>& source_segment = std::nullopt) {
    const auto flow = flow_for_net(circuit, candidate.net);
    if (!flow.has_value()) return true;
    if (candidate.from_terminal == flow->in_pin) return false;
    if (candidate.to_terminal == flow->out_pin) return false;
    if (source_segment.has_value()) {
        if (source_segment->current_direction == CurrentDirection::Out && candidate.to_terminal == source_segment->from) return false;
        if (source_segment->current_direction == CurrentDirection::In && candidate.from_terminal == source_segment->to) return false;
    }
    if (candidate.from_terminal == flow->out_pin || candidate.to_terminal == flow->in_pin) return true;
    return true;
}

// 琛ュ厖鍊欓€夎矾寰勭殑璁烘枃 penalty 鍒嗛」銆?
void annotate_candidate(
    const Circuit& circuit,
    routing::RouteCandidate& candidate,
    double current_density_penalty,
    double flow_penalty,
    const std::optional<WireSegmentRef>& source_segment = std::nullopt) {
    candidate.flow_ok = flow_ok_for_candidate(circuit, candidate, source_segment);
    candidate.current_density_ok = candidate.path.success;
    candidate.flow_penalty = candidate.flow_ok ? 0.0 : flow_penalty;
    candidate.current_density_penalty = candidate.current_density_ok ? 0.0 : current_density_penalty;
}

// 杩斿洖鍊欓€夌粦瀹氬埌鎸囧畾 LCP 鐨勭墿鐞嗗€欓€?id锛屾櫘閫?pin 杩斿洖绌哄瓧绗︿覆銆?
std::string candidate_location_for_lcp(const routing::RouteCandidate& candidate, const std::string& lcp_id) {
    if (!candidate.lcp_id.empty() && candidate.lcp_id == lcp_id) return candidate.lcp_candidate_id;
    if (!candidate.source_lcp_id.empty() && candidate.source_lcp_id == lcp_id) return candidate.source_lcp_candidate_id;
    if (!candidate.target_lcp_id.empty() && candidate.target_lcp_id == lcp_id) return candidate.target_lcp_candidate_id;
    return {};
}

// 鐢熸垚 LCP id 鍜岀墿鐞嗗€欓€?id 鐨勮仈鍚?key锛岀敤浜庣粺璁″悓涓€ hub 鍊欓€夋槸鍚﹁鐩栧叏閮?incident segment銆?
std::string lcp_location_key(const std::string& lcp_id, const std::string& candidate_id) {
    return lcp_id + "|" + candidate_id;
}

// 汇总每个 LCP 候选点需要覆盖和已经连通的 incident segments。
std::vector<LcpCandidateCoverage> collect_lcp_candidate_coverage(
    const RoutingEvaluationRequest& request,
    const std::vector<routing::RouteCandidate>& candidates) {
    std::unordered_map<std::string, std::unordered_set<std::string>> required_segments_by_location;
    std::unordered_map<std::string, std::unordered_set<std::string>> reachable_segments_by_location;
    std::vector<std::pair<std::string, std::string>> ordered_locations;
    std::unordered_set<std::string> seen_locations;

    for (const auto& topology : request.net_topologies) {
        for (const auto& point : topology.linking_points) {
            for (const auto& location : point.location_candidates) {
                const std::string key = lcp_location_key(point.id, location.id);
                if (seen_locations.insert(key).second) ordered_locations.push_back({point.id, location.id});
                for (const auto& segment : point.segments) {
                    const std::string segment_id =
                        segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
                    required_segments_by_location[key].insert(segment_id);
                }
            }
        }
    }

    for (const auto& candidate : candidates) {
        if (!candidate.path.success) continue;
        const auto add_reachable = [&](const std::string& lcp_id) {
            const std::string candidate_id = candidate_location_for_lcp(candidate, lcp_id);
            if (candidate_id.empty()) return;
            reachable_segments_by_location[lcp_location_key(lcp_id, candidate_id)].insert(candidate.segment_id);
        };
        add_reachable(candidate.lcp_id);
        add_reachable(candidate.source_lcp_id);
        add_reachable(candidate.target_lcp_id);
    }

    std::vector<LcpCandidateCoverage> result;
    for (const auto& [lcp_id, candidate_id] : ordered_locations) {
        const std::string key = lcp_location_key(lcp_id, candidate_id);
        LcpCandidateCoverage coverage;
        coverage.lcp_id = lcp_id;
        coverage.candidate_id = candidate_id;
        const auto required = required_segments_by_location.find(key);
        if (required != required_segments_by_location.end()) {
            coverage.required_segments.assign(required->second.begin(), required->second.end());
            std::sort(coverage.required_segments.begin(), coverage.required_segments.end());
        }
        const auto reachable = reachable_segments_by_location.find(key);
        if (reachable != reachable_segments_by_location.end()) {
            coverage.reachable_segments.assign(reachable->second.begin(), reachable->second.end());
            std::sort(coverage.reachable_segments.begin(), coverage.reachable_segments.end());
        }
        for (const auto& segment_id : coverage.required_segments) {
            if (std::find(coverage.reachable_segments.begin(), coverage.reachable_segments.end(), segment_id) ==
                coverage.reachable_segments.end()) {
                coverage.missing_segments.push_back(segment_id);
            }
        }
        coverage.covers_all_incident_segments = coverage.missing_segments.empty();
        result.push_back(std::move(coverage));
    }
    return result;
}

void filter_multi_terminal_unreachable_lcp_candidates(
    const RoutingEvaluationRequest& request,
    std::vector<routing::RouteCandidate>& candidates,
    std::vector<LcpCandidateFilterEvent>* filter_events) {
    std::unordered_map<std::string, std::unordered_set<std::string>> required_segments_by_location;
    std::unordered_map<std::string, std::unordered_set<std::string>> reachable_segments_by_location;

    for (const auto& topology : request.net_topologies) {
        for (const auto& point : topology.linking_points) {
            for (const auto& location : point.location_candidates) {
                const std::string key = lcp_location_key(point.id, location.id);
                for (const auto& segment : point.segments) {
                    const std::string segment_id =
                        segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
                    required_segments_by_location[key].insert(segment_id);
                }
            }
        }
    }

    for (const auto& candidate : candidates) {
        if (!candidate.path.success) continue;
        const auto add_reachable = [&](const std::string& lcp_id) {
            const std::string candidate_id = candidate_location_for_lcp(candidate, lcp_id);
            if (candidate_id.empty()) return;
            reachable_segments_by_location[lcp_location_key(lcp_id, candidate_id)].insert(candidate.segment_id);
        };
        add_reachable(candidate.lcp_id);
        add_reachable(candidate.source_lcp_id);
        add_reachable(candidate.target_lcp_id);
    }

    for (auto& candidate : candidates) {
        if (!candidate.path.success) continue;
        std::vector<std::string> missing_messages;
        std::unordered_set<std::string> checked_lcps;
        std::unordered_set<std::string> emitted_missing;
        const auto check_lcp = [&](const std::string& lcp_id) {
            if (lcp_id.empty() || !checked_lcps.insert(lcp_id).second) return;
            const std::string candidate_id = candidate_location_for_lcp(candidate, lcp_id);
            if (candidate_id.empty()) return;
            const std::string key = lcp_location_key(lcp_id, candidate_id);
            const auto required = required_segments_by_location.find(key);
            if (required == required_segments_by_location.end()) return;
            const auto reachable = reachable_segments_by_location.find(key);
            for (const auto& segment_id : required->second) {
                if (reachable == reachable_segments_by_location.end() || !reachable->second.contains(segment_id)) {
                    const std::string message = lcp_id + "@" + candidate_id + " missing " + segment_id;
                    if (emitted_missing.insert(message).second) missing_messages.push_back(message);
                }
            }
        };
        check_lcp(candidate.lcp_id);
        check_lcp(candidate.source_lcp_id);
        check_lcp(candidate.target_lcp_id);
        if (missing_messages.empty()) continue;
        candidate.path.success = false;
        candidate.path.points.clear();
        candidate.path.metrics = routing::PathMetrics{};
        candidate.path.message = "multi_terminal_missing: LCP candidate is not multi-terminal reachable";
        for (const auto& message : missing_messages) candidate.path.message += "; " + message;
        candidate.current_density_ok = false;
        candidate.current_density_penalty = 50000.0;
        if (filter_events != nullptr) {
            filter_events->push_back(LcpCandidateFilterEvent{
                candidate.net,
                candidate.from_terminal,
                candidate.to_terminal,
                candidate.segment_id,
                candidate.lcp_candidate_id,
                candidate.source_lcp_candidate_id,
                candidate.target_lcp_candidate_id,
                candidate.path.message,
            });
        }
    }
}

// 涓?LCP 鎷撴墤涓殑姣忔潯閫昏緫 segment 鐢熸垚 A* 鍊欓€夎矾寰勩€?
std::vector<routing::RouteCandidate> generate_lcp_route_candidates(
    const routing::RoutingContext& context,
    const RoutingEvaluationRequest& request,
    const Circuit& circuit,
    std::vector<routing::RouteCandidate>* raw_candidates = nullptr,
    std::vector<LcpCandidateFilterEvent>* filter_events = nullptr) {
    std::vector<routing::RouteCandidate> candidates;
    std::unordered_map<std::string, LinkingControlPoint> lcp_by_id;
    for (const auto& point : request.linking_points) lcp_by_id[point.id] = point;

    std::unordered_map<std::string, WireSegmentRef> segment_by_id;
    for (const auto& point : request.linking_points) {
        for (const auto& segment : point.segments) {
            const std::string id = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
            segment_by_id.try_emplace(id, segment);
        }
    }

    constexpr std::size_t kBasePairwiseCandidatesPerSegment = 32;
    constexpr std::size_t kMaxPairwiseCandidatesPerSegment = 96;
    for (const auto& [_, segment] : segment_by_id) {
        std::vector<std::string> lcp_endpoints;
        if (lcp_by_id.contains(segment.from)) lcp_endpoints.push_back(segment.from);
        if (lcp_by_id.contains(segment.to)) lcp_endpoints.push_back(segment.to);
        if (lcp_endpoints.empty()) continue;

        bool missing_location = false;
        for (const auto& lcp_id : lcp_endpoints) {
            if (lcp_by_id.at(lcp_id).location_candidates.empty()) missing_location = true;
        }
        if (missing_location) {
            routing::RouteCandidate candidate;
            candidate.net = segment.net;
            candidate.from_terminal = segment.from;
            candidate.to_terminal = segment.to;
            candidate.segment_id = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
            candidate.lcp_id = lcp_endpoints.front();
            if (lcp_by_id.contains(segment.from)) candidate.source_lcp_id = segment.from;
            if (lcp_by_id.contains(segment.to)) candidate.target_lcp_id = segment.to;
            candidate.wire_width = std::max(segment.min_width, 1e-9);
            candidate.path = routing::GridPath{false, "LCP has no physical location candidate", {}, {}};
            annotate_candidate(circuit, candidate, 50000.0, 50000.0, segment);
            candidates.push_back(std::move(candidate));
            continue;
        }

        std::vector<LocationBinding> bindings;
        if (lcp_endpoints.size() == 1) {
            const auto& lcp = lcp_by_id.at(lcp_endpoints.front());
            for (const auto& location : lcp.location_candidates) {
                LocationBinding binding;
                binding.by_lcp[lcp.id] = location;
                binding.estimate = location.penalty;
                bindings.push_back(std::move(binding));
            }
        } else {
            const auto& first = lcp_by_id.at(lcp_endpoints[0]);
            const auto& second = lcp_by_id.at(lcp_endpoints[1]);
            for (const auto& first_location : first.location_candidates) {
                for (const auto& second_location : second.location_candidates) {
                    LocationBinding binding;
                    binding.by_lcp[first.id] = first_location;
                    binding.by_lcp[second.id] = second_location;
                    binding.estimate = endpoint_pair_distance(first_location, second_location);
                    bindings.push_back(std::move(binding));
                }
            }
            std::sort(bindings.begin(), bindings.end(), [](const auto& lhs, const auto& rhs) {
                return lhs.estimate < rhs.estimate;
            });
            bindings = select_coverage_aware_bindings(
                bindings,
                first,
                second,
                kBasePairwiseCandidatesPerSegment,
                kMaxPairwiseCandidatesPerSegment);
        }

        for (const auto& binding : bindings) {
            const auto start = endpoint_grid_point(context, lcp_by_id, binding.by_lcp, segment, segment.from);
            const auto goal = endpoint_grid_point(context, lcp_by_id, binding.by_lcp, segment, segment.to);
            routing::RouteCandidate candidate;
            candidate.net = segment.net;
            candidate.from_terminal = segment.from;
            candidate.to_terminal = segment.to;
            candidate.segment_id = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
            candidate.wire_width = std::max(segment.min_width, 1e-9);
            if (!lcp_endpoints.empty()) {
                candidate.lcp_id = lcp_endpoints.front();
                candidate.lcp_candidate_id = lcp_candidate_for_endpoint(binding.by_lcp, lcp_endpoints.front());
            }
            if (lcp_by_id.contains(segment.from)) {
                candidate.source_lcp_id = segment.from;
                candidate.source_lcp_candidate_id = lcp_candidate_for_endpoint(binding.by_lcp, segment.from);
            }
            if (lcp_by_id.contains(segment.to)) {
                candidate.target_lcp_id = segment.to;
                candidate.target_lcp_candidate_id = lcp_candidate_for_endpoint(binding.by_lcp, segment.to);
            }
            if (!start.has_value() || !goal.has_value()) {
                candidate.path = routing::GridPath{false, "LCP endpoint cannot be resolved", {}, {}};
                annotate_candidate(circuit, candidate, 50000.0, 50000.0, segment);
                candidates.push_back(std::move(candidate));
                continue;
            }
            routing::AStarConfig config;
            config.wire_width = candidate.wire_width;
            candidate.path = routing::find_astar_path(context.grid(), context.obstacles(), *start, *goal, config);
            for (const auto& [__, location] : binding.by_lcp) candidate.coupling_cost += location.penalty;
            annotate_candidate(circuit, candidate, 50000.0, 50000.0, segment);
            candidates.push_back(std::move(candidate));
        }
    }
    if (raw_candidates != nullptr) *raw_candidates = candidates;
    filter_multi_terminal_unreachable_lcp_candidates(request, candidates, filter_events);
    return candidates;
}
[[maybe_unused]] std::vector<routing::GridPoint> prune_backtracks(std::vector<routing::GridPoint> points) {
    bool changed = true;
    while (changed) {
        changed = false;
        std::vector<routing::GridPoint> pruned;
        for (const auto& point : points) {
            if (pruned.size() >= 2) {
                const auto& before = pruned[pruned.size() - 2];
                if (before.ix == point.ix && before.iy == point.iy && before.layer == point.layer) {
                    pruned.pop_back();
                    changed = true;
                    continue;
                }
            }
            pruned.push_back(point);
        }
        points = std::move(pruned);
    }
    return points;
}

// 鍒ゆ柇涓や釜缃戞牸姝ヨ繘鏄惁鍦ㄥ悓涓€閲戝睘灞備笂鍏辩嚎銆?
[[maybe_unused]] bool same_line_direction(
    const routing::GridPoint& first,
    const routing::GridPoint& second,
    const routing::GridPoint& third) {
    if (first.layer != second.layer || second.layer != third.layer) return false;
    const int dx1 = second.ix - first.ix;
    const int dy1 = second.iy - first.iy;
    const int dx2 = third.ix - second.ix;
    const int dy2 = third.iy - second.iy;
    return dx1 == dx2 && dy1 == dy2;
}

// 鍚戣緭鍑鸿拷鍔犱竴鏉￠潪闆堕暱搴︿腑蹇冪嚎绾挎銆?
// 鍒ゆ柇涓や釜杩炵画鍧愭爣鏄惁鍙涓哄悓涓€鐐癸紝閬垮厤娴偣璇樊褰卞搷绾挎鍚堝苟鍜屾鏌ャ€?
bool same_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

// 杩斿洖 topology segment 鐨勭ǔ瀹氬尮閰嶉敭銆?
std::string segment_key(const std::string& net, const std::string& from, const std::string& to) {
    return net + "|" + from + "|" + to;
}

// 杩斿洖鍊欓€夎矾寰勫搴旂殑 topology segment 鍖归厤閿€?
std::string segment_key(const routing::RouteCandidate& candidate) {
    return segment_key(candidate.net, candidate.from_terminal, candidate.to_terminal);
}

// 鏋勫缓 detailed routing 鍥炴函鎵€闇€鐨?LCP 鍜?segment 绱㈠紩銆?
DetailedTopologyIndex build_detailed_topology_index(const RoutingEvaluationRequest& request) {
    DetailedTopologyIndex index;
    for (const auto& point : request.linking_points) {
        index.lcp_by_id[point.id] = point;
        index.lcp_space_by_id[point.id] = point.space_node_id;
        for (const auto& segment : point.segments) {
            index.topology_segment_keys.insert(segment_key(segment.net, segment.from, segment.to));
            index.topology_segments.push_back(segment);
        }
    }
    for (const auto& topology : request.net_topologies) {
        for (const auto& point : topology.linking_points) {
            index.lcp_by_id.try_emplace(point.id, point);
            if (!point.space_node_id.empty()) index.lcp_space_by_id.try_emplace(point.id, point.space_node_id);
        }
        for (const auto& segment : topology.segments) {
            index.topology_segment_keys.insert(segment_key(segment.net, segment.from, segment.to));
            index.topology_segments.push_back(segment);
        }
    }
    for (const auto& space : request.space_nodes) {
        index.space_owner_by_id[space.id] = space.owner;
        for (const auto& point : space.linking_points) {
            index.lcp_by_id.try_emplace(point.id, point);
            index.lcp_space_by_id[point.id] = space.id;
        }
    }
    for (const auto& [id, point] : index.lcp_by_id) {
        if (point.location_candidates.empty()) index.lcp_without_location.insert(id);
    }
    return index;
}

// 鍒ゆ柇 terminal 鏄惁鏄?LCP id銆?
bool is_lcp_terminal(const DetailedTopologyIndex& index, const std::string& terminal) {
    return index.lcp_by_id.contains(terminal) || index.lcp_space_by_id.contains(terminal);
}

// 杩斿洖鍊欓€夎矾寰勮繛鎺ュ埌鐨?LCP id銆?
std::string lcp_id_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    if (!candidate.lcp_id.empty()) return candidate.lcp_id;
    if (is_lcp_terminal(index, candidate.from_terminal)) return candidate.from_terminal;
    if (is_lcp_terminal(index, candidate.to_terminal)) return candidate.to_terminal;
    return {};
}

// 杩斿洖鍊欓€夎矾寰勬墍灞炵殑 space node id銆?
std::string space_node_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    const std::string lcp_id = lcp_id_for_candidate(index, candidate);
    if (lcp_id.empty()) return {};
    const auto found = index.lcp_space_by_id.find(lcp_id);
    return found == index.lcp_space_by_id.end() ? std::string{} : found->second;
}

// 杩斿洖鎴栧垱寤烘寚瀹?net 鐨?detailed route trace銆?
// 鏀堕泦鍊欓€夎矾寰勫叧鑱斿埌鐨勫叏閮?LCP id锛孡CP-LCP segment 闇€瑕佸悓鏃跺弽棣堜袱绔?space銆?
std::vector<std::string> lcp_ids_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    std::vector<std::string> ids;
    const auto append = [&](const std::string& id) {
        if (id.empty()) return;
        if (!index.lcp_by_id.contains(id) && !index.lcp_space_by_id.contains(id)) return;
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
    };
    append(candidate.source_lcp_id);
    append(candidate.target_lcp_id);
    append(candidate.lcp_id);
    if (is_lcp_terminal(index, candidate.from_terminal)) append(candidate.from_terminal);
    if (is_lcp_terminal(index, candidate.to_terminal)) append(candidate.to_terminal);
    return ids;
}

// 鏀堕泦鍊欓€夎矾寰勫叧鑱斿埌鐨勫叏閮?space node id锛屽苟鍘婚噸銆?
std::vector<std::string> space_nodes_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    std::vector<std::string> spaces;
    for (const auto& lcp_id : lcp_ids_for_candidate(index, candidate)) {
        const auto found = index.lcp_space_by_id.find(lcp_id);
        if (found == index.lcp_space_by_id.end()) continue;
        if (std::find(spaces.begin(), spaces.end(), found->second) == spaces.end()) spaces.push_back(found->second);
    }
    return spaces;
}

DetailedRouteTrace& trace_for_net(DetailedRoutingReport& report, const std::string& net) {
    for (auto& trace : report.traces) {
        if (trace.net == net) return trace;
    }
    report.traces.push_back(DetailedRouteTrace{net, {}, {}, {}});
    return report.traces.back();
}

// 璁板綍 detailed traceback 涓嚭鐜扮殑 terminal 鎴?LCP 鑺傜偣銆?
void append_trace_node(
    DetailedRouteTrace& trace,
    const DetailedTopologyIndex& index,
    const routing::RoutingContext& context,
    const std::string& terminal) {
    for (const auto& node : trace.nodes) {
        if (node.id == terminal) return;
    }
    DetailedRouteNode node;
    node.id = terminal;
    if (is_lcp_terminal(index, terminal)) {
        node.kind = "lcp";
        const auto space = index.lcp_space_by_id.find(terminal);
        if (space != index.lcp_space_by_id.end()) node.space_node_id = space->second;
        const auto lcp = index.lcp_by_id.find(terminal);
        if (lcp != index.lcp_by_id.end() && !lcp->second.location_candidates.empty()) {
            node.x = lcp->second.location_candidates.front().x;
            node.y = lcp->second.location_candidates.front().y;
        }
    } else {
        node.kind = "pin";
        const auto pin = context.global_pins().find(terminal);
        if (pin != context.global_pins().end()) {
            node.x = pin->second.location.x;
            node.y = pin->second.location.y;
            node.layer = routing::index_to_layer(pin->second.layer);
        }
    }
    trace.nodes.push_back(std::move(node));
}

// 鍒ゆ柇鍊欓€夋槸鍚﹁兘鍜?request 涓殑 LCP topology 瀵逛笂銆?
bool candidate_matches_lcp_topology(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    if (index.topology_segment_keys.empty()) return true;
    if (lcp_ids_for_candidate(index, candidate).empty()) return true;
    if (index.topology_segment_keys.contains(segment_key(candidate))) return true;
    return index.topology_segment_keys.contains(segment_key(candidate.net, candidate.to_terminal, candidate.from_terminal));
}

// 鏌ユ壘鍊欓€夎矾寰勫搴旂殑鍘熷 WireSegmentRef锛岀敤浜庤缁嗗竷绾跨害鏉熷綊鍥犮€?
std::optional<WireSegmentRef> wire_segment_for_candidate(
    const DetailedTopologyIndex& index,
    const routing::RouteCandidate& candidate) {
    for (const auto& segment : index.topology_segments) {
        if (segment.net != candidate.net) continue;
        if (!segment.id.empty() && segment.id == candidate.segment_id) return segment;
        const bool same_direction = segment.from == candidate.from_terminal && segment.to == candidate.to_terminal;
        const bool reverse_direction = segment.from == candidate.to_terminal && segment.to == candidate.from_terminal;
        if (same_direction || reverse_direction) return segment;
    }
    return std::nullopt;
}

// 鍒ゆ柇 route segment 鏄惁涓烘按骞充腑蹇冪嚎銆?
bool route_is_horizontal(const RouteSegment& route) {
    return same_coord(route.y1, route.y2);
}

// 鍒ゆ柇 route segment 鏄惁涓哄瀭鐩翠腑蹇冪嚎銆?
bool route_is_vertical(const RouteSegment& route) {
    return same_coord(route.x1, route.x2);
}

// 杩斿洖 detailed routing 闃舵瀹為檯鍐欏叆鐨勭嚎瀹斤紝浼樺厛婊¤冻 WIRE_WIDTH 绾︽潫鑼冨洿銆?
double detailed_width_for_candidate(
    const Circuit& circuit,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    double width = candidate.wire_width > 0.0 ? candidate.wire_width : evaluation.context.default_width_for_net(candidate.net);
    const auto constraint = circuit.constraints.wire_widths.find(candidate.net);
    if (constraint == circuit.constraints.wire_widths.end()) return width;
    width = std::max(width, constraint->second.min_width);
    if (constraint->second.max_width > 0.0) width = std::min(width, constraint->second.max_width);
    return width;
}

// 杩斿洖鏃ц皟璇曟帴鍙ｄ娇鐢ㄧ殑绾垮锛屼笉渚濊禆 Circuit 浠ヤ繚鎸佸叕寮€鍑芥暟绛惧悕绋冲畾銆?
// 鍒ゆ柇鍊欓€夎矾寰勭殑鍘熷绾垮鏄惁婊¤冻 segment 涓?net 绾?WIRE_WIDTH/current-density 浠ｇ悊绾︽潫銆?
bool candidate_width_satisfies_constraints(
    const Circuit& circuit,
    const std::optional<WireSegmentRef>& segment,
    const routing::RouteCandidate& candidate) {
    const double width = candidate.wire_width;
    if (segment.has_value()) {
        if (segment->min_width > 0.0 && width + 1e-9 < segment->min_width) return false;
        if (segment->max_width > 0.0 && width - 1e-9 > segment->max_width) return false;
    }
    const auto net_width = circuit.constraints.wire_widths.find(candidate.net);
    if (net_width != circuit.constraints.wire_widths.end()) {
        if (net_width->second.min_width > 0.0 && width + 1e-9 < net_width->second.min_width) return false;
        if (net_width->second.max_width > 0.0 && width - 1e-9 > net_width->second.max_width) return false;
    }
    return true;
}

double selected_width_for_candidate(
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    return candidate.wire_width > 0.0 ? candidate.wire_width : evaluation.context.default_width_for_net(candidate.net);
}

// 鍒ゆ柇鏂扮嚎娈垫槸鍚﹁兘鍜屼笂涓€鏉＄嚎娈靛悎骞朵负鍚屼竴鏉′腑蹇冪嚎銆?
bool can_merge_with_last(const RouteSegment& last, const RouteSegment& edge) {
    if (last.net != edge.net || last.layer != edge.layer || !same_coord(last.width, edge.width)) return false;
    if (!same_coord(last.x2, edge.x1) || !same_coord(last.y2, edge.y1)) return false;
    const bool horizontal = route_is_horizontal(last) && route_is_horizontal(edge) && same_coord(last.y1, edge.y1);
    const bool vertical = route_is_vertical(last) && route_is_vertical(edge) && same_coord(last.x1, edge.x1);
    return horizontal || vertical;
}

[[maybe_unused]] void append_segment(
    std::vector<RouteSegment>& routes,
    const routing::Grid& grid,
    const std::string& net,
    int layer,
    const routing::GridPoint& start,
    const routing::GridPoint& end,
    double width) {
    const auto start_xy = grid.grid_to_point(start);
    const auto end_xy = grid.grid_to_point(end);
    if (start_xy.x == end_xy.x && start_xy.y == end_xy.y) return;
    RouteSegment edge{net, routing::index_to_layer(layer), start_xy.x, start_xy.y, end_xy.x, end_xy.y, width};
    if (!routes.empty() && can_merge_with_last(routes.back(), edge)) {
        routes.back().x2 = edge.x2;
        routes.back().y2 = edge.y2;
        return;
    }
    for (const auto& route : routes) {
        if (route.net != net || route.layer != edge.layer) continue;
        const bool same_direction = route.x1 == start_xy.x && route.y1 == start_xy.y && route.x2 == end_xy.x && route.y2 == end_xy.y;
        const bool reverse_direction = route.x1 == end_xy.x && route.y1 == end_xy.y && route.x2 == start_xy.x && route.y2 == start_xy.y;
        if (same_direction || reverse_direction) return;
    }
    routes.push_back(std::move(edge));
}

// 灏嗕竴鏉?A* 缃戞牸璺緞鍘嬬缉涓哄悓灞傚叡绾跨殑 routing segment銆?
void append_path_segments(
    std::vector<RouteSegment>& routes,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const double width = selected_width_for_candidate(evaluation, candidate);
    auto converted =
        routing::candidate_to_route_segments(evaluation.context.grid(), candidate, width, evaluation.context.active_regions());
    routes.insert(routes.end(), converted.begin(), converted.end());
}

// 灏嗕竴鏉?A* 缃戞牸璺緞鎸?detailed routing 绾垮瑙勫垯鍘嬬缉鎴?route segment銆?
std::vector<RouteSegment> detailed_path_segments(
    const Circuit& circuit,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const double width = detailed_width_for_candidate(circuit, evaluation, candidate);
    return routing::candidate_to_route_segments(
        evaluation.context.grid(),
        candidate,
        width,
        evaluation.context.active_regions());
}

// 琛ㄧず detailed routing 鍚堟硶鍖栧悗瀹為檯閲囩敤鐨勫€欓€夊拰閲戝睘绾挎銆?
struct DetailedLegalization {
    bool success{};
    routing::RouteCandidate candidate;
    std::vector<RouteSegment> routes;
    routing::PathMetrics metrics;
    bool used_alternative{};
    bool used_reroute{};
    std::vector<std::string> failure_messages;
};

// 鍒ゆ柇涓や釜鍊欓€夋槸鍚﹁繛鎺ュ悓涓€閫昏緫 terminal pair銆?
bool same_logical_candidate_pair(
    const routing::RouteCandidate& lhs,
    const routing::RouteCandidate& rhs) {
    if (lhs.net != rhs.net) return false;
    if (!lhs.segment_id.empty() && !rhs.segment_id.empty() && lhs.segment_id == rhs.segment_id) return true;
    const bool same_direction = lhs.from_terminal == rhs.from_terminal && lhs.to_terminal == rhs.to_terminal;
    const bool reverse_direction = lhs.from_terminal == rhs.to_terminal && lhs.to_terminal == rhs.from_terminal;
    return same_direction || reverse_direction;
}

// 鍒ゆ柇鍊欓€夋槸鍚︽惡甯?LCP 鐗╃悊浣嶇疆缁戝畾锛岀敤浜?detailed 闃舵鏁寸綉閿佸畾銆?
bool candidate_has_lcp_binding(const routing::RouteCandidate& candidate) {
    return !candidate.lcp_id.empty() || !candidate.source_lcp_id.empty() || !candidate.target_lcp_id.empty();
}

std::unordered_map<std::string, std::string> lcp_bindings_for_candidate(const routing::RouteCandidate& candidate) {
    std::unordered_map<std::string, std::string> bindings;
    auto add = [&](const std::string& lcp_id, const std::string& location_id) {
        if (!lcp_id.empty() && !location_id.empty()) bindings[lcp_id] = location_id;
    };
    add(candidate.lcp_id, candidate.lcp_candidate_id);
    add(candidate.source_lcp_id, candidate.source_lcp_candidate_id);
    add(candidate.target_lcp_id, candidate.target_lcp_candidate_id);
    return bindings;
}

bool merge_lcp_binding(
    std::unordered_map<std::string, std::string>& bindings,
    const std::string& lcp_id,
    const std::string& location_id,
    std::string* conflict = nullptr) {
    if (lcp_id.empty() || location_id.empty()) return true;
    const auto found = bindings.find(lcp_id);
    if (found == bindings.end()) {
        bindings[lcp_id] = location_id;
        return true;
    }
    if (found->second == location_id) return true;
    if (conflict != nullptr) *conflict = "LCP " + lcp_id + " binds both " + found->second + " and " + location_id;
    return false;
}

bool merge_lcp_bindings(
    std::unordered_map<std::string, std::string>& bindings,
    const routing::RouteCandidate& candidate,
    std::string* conflict = nullptr) {
    for (const auto& [lcp_id, location_id] : lcp_bindings_for_candidate(candidate)) {
        if (!merge_lcp_binding(bindings, lcp_id, location_id, conflict)) return false;
    }
    return true;
}

bool candidate_matches_lcp_bindings(
    const routing::RouteCandidate& candidate,
    const std::unordered_map<std::string, std::string>& bindings) {
    auto matches = [&](const std::string& lcp_id, const std::string& location_id) {
        if (lcp_id.empty()) return true;
        const auto found = bindings.find(lcp_id);
        return found == bindings.end() || found->second == location_id;
    };
    return matches(candidate.lcp_id, candidate.lcp_candidate_id) &&
           matches(candidate.source_lcp_id, candidate.source_lcp_candidate_id) &&
           matches(candidate.target_lcp_id, candidate.target_lcp_candidate_id);
}

// 鏈?LCP 缁戝畾鏃讹紝alternative 蹇呴』淇濇寔鍚屼竴 lcp_candidate_id锛岀姝㈡寜鏀矾鎷嗘暎 DP 缁戝畾銆?
bool same_lcp_location_binding(
    const routing::RouteCandidate& lhs,
    const routing::RouteCandidate& rhs) {
    if (!candidate_has_lcp_binding(lhs) && !candidate_has_lcp_binding(rhs)) return true;
    if (lhs.lcp_id != rhs.lcp_id || lhs.lcp_candidate_id != rhs.lcp_candidate_id) return false;
    if (lhs.source_lcp_id != rhs.source_lcp_id ||
        lhs.source_lcp_candidate_id != rhs.source_lcp_candidate_id) {
        return false;
    }
    if (lhs.target_lcp_id != rhs.target_lcp_id ||
        lhs.target_lcp_candidate_id != rhs.target_lcp_candidate_id) {
        return false;
    }
    return true;
}

// 濡傛灉鍊欓€夌嚎娈典笉涓庢棦鏈夊紓缃戦噾灞炵煭璺紝鍒欒繑鍥炲彲鍐欏叆鐨勭嚎娈点€?
// 鍒ゆ柇涓や釜鍊欓€夋槸鍚︽嫢鏈夊畬鍏ㄧ浉鍚岀殑缃戞牸璺緞锛岀敤浜庨伩鍏嶈鍒犲悓 terminal pair 鐨勪笉鍚?A* 澶囬€夎矾銆?
bool same_candidate_path(
    const routing::RouteCandidate& lhs,
    const routing::RouteCandidate& rhs) {
    if (lhs.path.points.size() != rhs.path.points.size()) return false;
    for (std::size_t index = 0; index < lhs.path.points.size(); ++index) {
        const auto& left = lhs.path.points[index];
        const auto& right = rhs.path.points[index];
        if (left.ix != right.ix || left.iy != right.iy || left.layer != right.layer) return false;
    }
    return true;
}

// 鍒ゆ柇涓や釜缃戞牸鐐规槸鍚﹀畬鍏ㄧ浉鍚屻€?
bool same_grid_point(const routing::GridPoint& lhs, const routing::GridPoint& rhs) {
    return lhs.ix == rhs.ix && lhs.iy == rhs.iy && lhs.layer == rhs.layer;
}

// 判断候选金属线段是否违反 active-region DRC；在 detailed 合法化前用于触发重布线。
bool routes_violate_active_regions(
    const RoutingEvaluationRequest& request,
    const std::vector<RouteSegment>& routes);

std::optional<std::vector<RouteSegment>> legal_routes_without_short(
    std::vector<RouteSegment> routes,
    const std::vector<RouteSegment>& occupied_routes) {
    if (routes.empty()) return std::nullopt;
    if (routing::routes_short_with_existing(routes, occupied_routes)) return std::nullopt;
    return routes;
}

// 杩斿洖绗竴澶勪笌宸插竷绾垮彂鐢熺殑鍚屽眰寮傜綉鐭矾锛屼緵 detailed legalize 璇婃柇銆?
std::string first_short_description(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& occupied_routes) {
    for (const auto& route : routes) {
        for (const auto& occupied : occupied_routes) {
            if (!routing::same_layer_short(route, occupied)) continue;
            return route.net + ":" + route.layer + " (" + std::to_string(route.x1) + "," +
                   std::to_string(route.y1) + ")->(" + std::to_string(route.x2) + "," +
                   std::to_string(route.y2) + ") with " + occupied.net + ":" + occupied.layer +
                   " (" + std::to_string(occupied.x1) + "," + std::to_string(occupied.y1) +
                   ")->(" + std::to_string(occupied.x2) + "," + std::to_string(occupied.y2) + ")";
        }
    }
    return {};
}

// 鎸?routing.txt 鐨勬渶缁堢嚎娈佃涔夌粺璁?detailed route 鐨勭嚎闀裤€乥end 鍜?via銆?
routing::PathMetrics route_metrics_from_segments(const std::vector<RouteSegment>& routes) {
    routing::PathMetrics metrics;
    std::unordered_map<std::string, std::vector<const RouteSegment*>> by_net;
    for (const auto& route : routes) {
        metrics.wirelength += std::abs(route.x2 - route.x1) + std::abs(route.y2 - route.y1);
        by_net[route.net].push_back(&route);
    }
    for (const auto& [net, segments] : by_net) {
        (void)net;
        for (std::size_t index = 1; index < segments.size(); ++index) {
            const auto& previous = *segments[index - 1];
            const auto& current = *segments[index];
            if (previous.x2 != current.x1 || previous.y2 != current.y1) continue;
            if (previous.layer == current.layer) {
                ++metrics.bend_count;
            } else {
                ++metrics.via_count;
            }
        }
    }
    metrics.cost =
        metrics.wirelength + 3.0 * static_cast<double>(metrics.bend_count) +
        kDetailedViaWeight * static_cast<double>(metrics.via_count);
    return metrics;
}

// 纭鍘嬬缉鍚庣殑閲戝睘绾挎浠嶈兘鐢?routing.txt 璇箟杩炴帴鍘熷璧风偣鍜岀粓鐐广€?
// 鎸夋渶缁?A* path 閲嶆柊缁熻 legalized candidate 鐨勭湡瀹炵嚎闀裤€乥end 鍜?via锛岃鐩栫鐐?via銆?
routing::PathMetrics route_metrics_from_candidate_path(
    const routing::Grid& grid,
    const routing::RouteCandidate& candidate) {
    routing::PathMetrics metrics;
    routing::GridPoint previous_point{};
    bool has_previous_point = false;
    routing::GridPoint previous_planar_from{};
    routing::GridPoint previous_planar_to{};
    bool has_previous_planar = false;
    for (const auto& point : candidate.path.points) {
        if (!has_previous_point) {
            previous_point = point;
            has_previous_point = true;
            continue;
        }
        if (previous_point.layer != point.layer) {
            ++metrics.via_count;
            previous_point = point;
            continue;
        }
        const auto previous_xy = grid.grid_to_point(previous_point);
        const auto current_xy = grid.grid_to_point(point);
        metrics.wirelength += std::abs(current_xy.x - previous_xy.x) + std::abs(current_xy.y - previous_xy.y);
        if (has_previous_planar &&
            previous_planar_from.layer == previous_planar_to.layer &&
            previous_planar_to.layer == previous_point.layer) {
            const int prev_dx = previous_planar_to.ix - previous_planar_from.ix;
            const int prev_dy = previous_planar_to.iy - previous_planar_from.iy;
            const int next_dx = point.ix - previous_point.ix;
            const int next_dy = point.iy - previous_point.iy;
            const bool previous_horizontal = prev_dx != 0 && prev_dy == 0;
            const bool previous_vertical = prev_dx == 0 && prev_dy != 0;
            const bool next_horizontal = next_dx != 0 && next_dy == 0;
            const bool next_vertical = next_dx == 0 && next_dy != 0;
            if ((previous_horizontal && next_vertical) || (previous_vertical && next_horizontal)) ++metrics.bend_count;
        }
        previous_planar_from = previous_point;
        previous_planar_to = point;
        has_previous_planar = true;
        previous_point = point;
    }
    metrics.cost =
        metrics.wirelength + 3.0 * static_cast<double>(metrics.bend_count) +
        kDetailedViaWeight * static_cast<double>(metrics.via_count);
    return metrics;
}

// 纭鍘嬬缉鍚庣殑閲戝睘绾挎浠嶈兘杩炴帴鍒板師濮嬭捣缁堢偣鍧愭爣锛涚鐐瑰眰宸敱 A* path 鐨?via move 琛ㄧず銆?
bool route_segments_connect_path_endpoints(
    const std::vector<RouteSegment>& routes,
    const routing::Grid& grid,
    const routing::GridPoint& start,
    const routing::GridPoint& goal) {
    if (routes.empty()) return false;
    const auto start_xy = grid.grid_to_point(start);
    const auto goal_xy = grid.grid_to_point(goal);
    const auto& first = routes.front();
    const auto& last = routes.back();
    return same_coord(first.x1, start_xy.x) &&
           same_coord(first.y1, start_xy.y) &&
           same_coord(last.x2, goal_xy.x) &&
           same_coord(last.y2, goal_xy.y);
}

// 鎸夎鏂?detailed routing 璇箟瀵瑰€欓€夊仛灞€閮ㄥ悎娉曞寲锛氭浛浠ｅ€欓€変紭鍏堬紝涓嶈繘琛屽厤璐规暣鏉℃崲灞傘€?
// 灏嗗凡甯冨紓缃?detailed 閲戝睘娉ㄥ唽涓?A* 闅滅锛岄噸鏂板鎵句竴鏉＄湡瀹炲惈 via 鎴愭湰鐨勫悎娉曞寲璺緞銆?
// 鐢熸垚鍊欓€夌殑 detailed 绾挎锛屽苟鎷掔粷鍘嬬缉鍚庢棤娉曡繛鎺ュ師濮嬭矾寰勮捣缁堢偣鐨勭粨鏋溿€?
std::optional<std::vector<RouteSegment>> legal_candidate_routes(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate,
    const std::vector<RouteSegment>& occupied_routes,
    std::string* failure_reason = nullptr) {
    if (!candidate.path.success || candidate.path.points.empty()) {
        if (failure_reason != nullptr) *failure_reason = "candidate path failed";
        return std::nullopt;
    }
    // LCP 与引脚落在同一网格点时，A* 正确返回单点路径；该连接无需写入金属线段。
    if (candidate.path.points.size() == 1) return std::vector<RouteSegment>{};
    auto routes = detailed_path_segments(circuit, evaluation, candidate);
    if (!route_segments_connect_path_endpoints(
            routes,
            evaluation.context.grid(),
            candidate.path.points.front(),
            candidate.path.points.back())) {
        if (failure_reason != nullptr) *failure_reason = "candidate route segments do not connect endpoints";
        return std::nullopt;
    }
    if (routes_violate_active_regions(request, routes)) {
        if (failure_reason != nullptr) *failure_reason = "candidate route segments cross active region";
        return std::nullopt;
    }
    auto legal = legal_routes_without_short(std::move(routes), occupied_routes);
    if (!legal.has_value() && failure_reason != nullptr) {
        *failure_reason = "candidate route segments short with occupied routes: " +
                          first_short_description(detailed_path_segments(circuit, evaluation, candidate), occupied_routes);
    }
    return legal;
}

std::optional<DetailedLegalization> reroute_candidate_avoiding_detailed_routes(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate,
    const std::vector<RouteSegment>& occupied_routes,
    bool used_alternative,
    std::string* failure_reason = nullptr) {
    if (!candidate.path.success || candidate.path.points.size() < 2) {
        if (failure_reason != nullptr) *failure_reason = "reroute skipped because candidate path failed";
        return std::nullopt;
    }

    auto obstacles = evaluation.context.obstacles();
    for (const auto& route : occupied_routes) {
        if (route.net == candidate.net) continue;
        obstacles.add_obstacle(routing::Obstacle{
            routing::segment_to_rect(
                routing::Segment{
                    routing::Point{route.x1, route.y1},
                    routing::Point{route.x2, route.y2},
                },
                route.width),
            routing::layer_to_index(route.layer),
            "detailed_route",
            route.net,
        });
    }

    const auto start = candidate.path.points.front();
    const auto goal = candidate.path.points.back();
    for (int layer = 0; layer < evaluation.context.grid().layer_count(); ++layer) {
        obstacles.add_terminal_point(routing::GridPoint{start.ix, start.iy, layer});
        obstacles.add_terminal_point(routing::GridPoint{goal.ix, goal.iy, layer});
    }

    routing::AStarConfig config;
    config.wire_width = detailed_width_for_candidate(circuit, evaluation, candidate);
    config.max_expanded_nodes = 100000;

    std::vector<std::pair<int, int>> endpoint_layers;
    endpoint_layers.push_back({start.layer, goal.layer});
    for (int layer = 0; layer < evaluation.context.grid().layer_count(); ++layer) {
        if (layer != start.layer) endpoint_layers.push_back({layer, goal.layer});
    }
    for (int layer = 0; layer < evaluation.context.grid().layer_count(); ++layer) {
        if (layer != goal.layer) endpoint_layers.push_back({start.layer, layer});
    }
    for (int start_layer = 0; start_layer < evaluation.context.grid().layer_count(); ++start_layer) {
        for (int goal_layer = 0; goal_layer < evaluation.context.grid().layer_count(); ++goal_layer) {
            if (start_layer == start.layer || goal_layer == goal.layer) continue;
            endpoint_layers.push_back({start_layer, goal_layer});
        }
    }

    std::string last_failure;
    for (const auto& [start_layer, goal_layer] : endpoint_layers) {
            const routing::GridPoint search_start{start.ix, start.iy, start_layer};
            const routing::GridPoint search_goal{goal.ix, goal.iy, goal_layer};
            auto path = routing::find_astar_path(evaluation.context.grid(), obstacles, search_start, search_goal, config);
            if (!path.success) {
                last_failure = "reroute A* failed: " + path.message;
                continue;
            }
            std::vector<routing::GridPoint> full_points{start};
            if (!same_grid_point(start, search_start)) full_points.push_back(search_start);
            for (std::size_t index = 1; index < path.points.size(); ++index) full_points.push_back(path.points[index]);
            if (!same_grid_point(full_points.back(), goal)) full_points.push_back(goal);

            routing::RouteCandidate rerouted = candidate;
            rerouted.path = std::move(path);
            rerouted.path.points = std::move(full_points);
            auto routes = detailed_path_segments(circuit, evaluation, rerouted);
            if (!route_segments_connect_path_endpoints(routes, evaluation.context.grid(), start, goal)) {
                last_failure = "reroute route segments do not connect endpoints";
                continue;
            }
            if (routes_violate_active_regions(request, routes)) {
                last_failure = "reroute route segments cross active region";
                continue;
            }
            auto legal_routes = legal_routes_without_short(std::move(routes), occupied_routes);
            if (!legal_routes.has_value()) {
                last_failure = "reroute route segments short with occupied routes: " +
                               first_short_description(detailed_path_segments(circuit, evaluation, rerouted), occupied_routes);
                continue;
            }
            const auto metrics = route_metrics_from_candidate_path(evaluation.context.grid(), rerouted);
            return DetailedLegalization{
                true,
                std::move(rerouted),
                std::move(*legal_routes),
                metrics,
                used_alternative,
                true,
                {}};
    }
    if (failure_reason != nullptr) *failure_reason = last_failure.empty() ? "reroute found no legal endpoint-layer combination" : last_failure;
    return std::nullopt;
}

// 鎸夎鏂?detailed routing 璇箟瀵瑰€欓€夊仛灞€閮ㄥ悎娉曞寲锛氬悓 LCP 缁戝畾鍐呯殑鏇夸唬璺緞浼樺厛锛岀劧鍚?A* reroute銆?
DetailedLegalization legalize_detailed_candidate(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& selected,
    const std::vector<RouteSegment>& occupied_routes) {
    std::vector<const routing::RouteCandidate*> attempts{&selected};
    for (const auto& candidate : evaluation.candidates) {
        if (!candidate.path.success || !same_logical_candidate_pair(candidate, selected)) continue;
        // 鏈?LCP 鏃剁姝㈡崲鍒颁笉鍚?lcp_candidate_id锛岄伩鍏嶅悇鏀矾鎷嗘暎 DP 涓€鑷存€с€?
        if (!same_lcp_location_binding(candidate, selected)) continue;
        bool duplicate = false;
        for (const auto* existing : attempts) {
            if (existing == &candidate ||
                (existing->segment_id == candidate.segment_id &&
                 existing->lcp_candidate_id == candidate.lcp_candidate_id &&
                 same_candidate_path(*existing, candidate))) {
                duplicate = true;
            }
        }
        if (!duplicate) attempts.push_back(&candidate);
    }

    std::optional<DetailedLegalization> best_legal;
    std::vector<std::string> failure_messages;
    double best_legal_cost = std::numeric_limits<double>::infinity();
    for (const auto* candidate : attempts) {
        std::string failure_reason;
        auto legal = legal_candidate_routes(circuit, request, evaluation, *candidate, occupied_routes, &failure_reason);
        if (legal.has_value()) {
            const auto legal_metrics = route_metrics_from_candidate_path(evaluation.context.grid(), *candidate);
            const double legal_cost = legal_metrics.cost;
            if (!best_legal.has_value() || legal_cost < best_legal_cost) {
                best_legal_cost = legal_cost;
                best_legal = DetailedLegalization{
                    true,
                    *candidate,
                    std::move(*legal),
                    legal_metrics,
                    candidate != &selected,
                    false,
                    {}};
            }
        } else if (!failure_reason.empty()) {
            failure_messages.push_back(candidate->net + ": legal candidate rejected: " + failure_reason);
        }
    }
    if (best_legal.has_value()) return std::move(*best_legal);

    for (const auto* candidate : attempts) {
        std::string failure_reason;
        auto rerouted = reroute_candidate_avoiding_detailed_routes(
            circuit,
            request,
            evaluation,
            *candidate,
            occupied_routes,
            candidate != &selected,
            &failure_reason);
        if (rerouted.has_value()) return std::move(*rerouted);
        if (!failure_reason.empty()) failure_messages.push_back(candidate->net + ": reroute rejected: " + failure_reason);
    }
    return DetailedLegalization{false, selected, {}, {}, false, false, std::move(failure_messages)};
}
/*
// 琛ㄧず鏁寸綉鍦ㄥ悓涓€ LCP location 涓嬪悎娉曞寲鍚庣殑缁撴灉銆?*/
struct LcpNetLegalization {
    bool success{};
    std::string chosen_lcp_candidate_id;
    bool switched_lcp_location{};
    std::vector<DetailedLegalization> branch_results;
    std::vector<std::string> failure_messages;
};

// 返回候选上用于整网锁定的主 LCP id（优先 lcp_id，其次 source/target）。
// 返回候选上与 primary_lcp_id 对应的 location candidate id。
// 判断候选是否绑定到指定 LCP 的指定物理位置。
// 收集该 net 在指定 LCP 下出现过的全部 location id，DP 选中的排在最前。
// 在固定 LCP location 下为一条支路挑选候选：同 location 的路径优先，再同 location 下 A* reroute。
// 判断候选是否绑定到指定 LCP 的指定物理位置。
bool candidate_matches_lcp_location(
    const routing::RouteCandidate& candidate,
    const std::string& lcp_id,
    const std::string& location_id) {
    if (lcp_id.empty() || location_id.empty()) return false;
    return (candidate.lcp_id == lcp_id && candidate.lcp_candidate_id == location_id) ||
           (candidate.source_lcp_id == lcp_id && candidate.source_lcp_candidate_id == location_id) ||
           (candidate.target_lcp_id == lcp_id && candidate.target_lcp_candidate_id == location_id);
}

// 收集指定 net/LCP 可选位置；DP 首选位置始终排在第一位。
std::vector<std::string> ordered_lcp_locations_for_net(
    const RoutingEvaluation& evaluation,
    const std::string& net,
    const std::string& lcp_id,
    const std::string& preferred_location) {
    std::vector<std::string> locations;
    auto append_unique = [&](const std::string& location_id) {
        if (location_id.empty() || std::find(locations.begin(), locations.end(), location_id) != locations.end()) return;
        locations.push_back(location_id);
    };
    append_unique(preferred_location);
    for (const auto& candidate : evaluation.candidates) {
        if (candidate.net != net || !candidate.path.success) continue;
        if (candidate.lcp_id == lcp_id) append_unique(candidate.lcp_candidate_id);
        if (candidate.source_lcp_id == lcp_id) append_unique(candidate.source_lcp_candidate_id);
        if (candidate.target_lcp_id == lcp_id) append_unique(candidate.target_lcp_candidate_id);
    }
    return locations;
}

// 在固定 LCP location 下为一条支路挑选候选：同位置路径优先，再尝试同位置 reroute。
DetailedLegalization legalize_branch_at_lcp_location(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& selected,
    const std::string& lcp_id,
    const std::string& location_id,
    const std::vector<RouteSegment>& occupied_routes) {
    std::vector<const routing::RouteCandidate*> attempts;
    auto push_attempt = [&](const routing::RouteCandidate& candidate) {
        if (!candidate.path.success) return;
        if (!same_logical_candidate_pair(candidate, selected)) return;
        if (!candidate_matches_lcp_location(candidate, lcp_id, location_id)) return;
        for (const auto* existing : attempts) {
            if (existing == &candidate ||
                (existing->segment_id == candidate.segment_id &&
                 existing->lcp_candidate_id == candidate.lcp_candidate_id &&
                 same_candidate_path(*existing, candidate))) {
                return;
            }
        }
        attempts.push_back(&candidate);
    };

    // 鑻?selected 宸叉槸鐩爣 location锛屼紭鍏堝皾璇曞畠锛涘惁鍒欏彧浠?evaluation.candidates 鍙栧悓 location 鍊欓€夈€?
    if (candidate_matches_lcp_location(selected, lcp_id, location_id)) {
        push_attempt(selected);
    }
    for (const auto& candidate : evaluation.candidates) {
        push_attempt(candidate);
    }

    std::vector<std::string> failure_messages;
    if (attempts.empty()) {
        failure_messages.push_back(
            selected.net + ": no candidate at LCP location " + lcp_id + "@" + location_id + " for " +
            selected.from_terminal + "->" + selected.to_terminal);
        return DetailedLegalization{false, selected, {}, {}, false, false, std::move(failure_messages)};
    }

    std::optional<DetailedLegalization> best_legal;
    double best_legal_cost = std::numeric_limits<double>::infinity();
    for (const auto* candidate : attempts) {
        std::string failure_reason;
        auto legal = legal_candidate_routes(circuit, request, evaluation, *candidate, occupied_routes, &failure_reason);
        if (legal.has_value()) {
            const auto legal_metrics = route_metrics_from_candidate_path(evaluation.context.grid(), *candidate);
            if (!best_legal.has_value() || legal_metrics.cost < best_legal_cost) {
                best_legal_cost = legal_metrics.cost;
                const bool used_alternative =
                    !same_candidate_path(*candidate, selected) ||
                    candidate->lcp_candidate_id != selected.lcp_candidate_id ||
                    candidate->source_lcp_candidate_id != selected.source_lcp_candidate_id ||
                    candidate->target_lcp_candidate_id != selected.target_lcp_candidate_id;
                best_legal = DetailedLegalization{
                    true,
                    *candidate,
                    std::move(*legal),
                    legal_metrics,
                    used_alternative,
                    false,
                    {}};
            }
        } else if (!failure_reason.empty()) {
            failure_messages.push_back(candidate->net + ": legal candidate rejected: " + failure_reason);
        }
    }
    if (best_legal.has_value()) return std::move(*best_legal);

    for (const auto* candidate : attempts) {
        std::string failure_reason;
        auto rerouted = reroute_candidate_avoiding_detailed_routes(
            circuit,
            request,
            evaluation,
            *candidate,
            occupied_routes,
            true,
            &failure_reason);
        if (rerouted.has_value()) return std::move(*rerouted);
        if (!failure_reason.empty()) {
            failure_messages.push_back(candidate->net + ": reroute rejected: " + failure_reason);
        }
    }
    return DetailedLegalization{false, selected, {}, {}, false, false, std::move(failure_messages)};
}

// 对带 LCP 的整网在提交 detailed route 前协商单一物理位置；多 LCP net 保持严格绑定。
LcpNetLegalization legalize_lcp_net(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const std::vector<routing::RouteCandidate>& selected_branches,
    const std::vector<RouteSegment>& occupied_routes,
    bool allow_location_negotiation) {
    LcpNetLegalization result;
    if (selected_branches.empty()) return result;

    const std::string net = selected_branches.front().net;
    std::unordered_map<std::string, std::string> selected_bindings;
    for (const auto& branch : selected_branches) {
        std::string conflict;
        if (!merge_lcp_bindings(selected_bindings, branch, &conflict)) {
            result.failure_messages.push_back(
                net + ": selected branches disagree on LCP binding before detailed legalize: " + conflict);
            return result;
        }
    }
    if (selected_bindings.empty()) {
        result.failure_messages.push_back(net + ": LCP net is missing physical location binding");
        return result;
    }

    // 单 LCP net 在提交 detailed route 前显式协商整网位置；多 LCP net 仍保持严格绑定，避免组合搜索失控。
    if (allow_location_negotiation && selected_bindings.size() == 1) {
        const auto& [lcp_id, preferred_location] = *selected_bindings.begin();
        for (const auto& location_id : ordered_lcp_locations_for_net(evaluation, net, lcp_id, preferred_location)) {
            std::vector<DetailedLegalization> branch_results;
            std::vector<RouteSegment> tentative_occupied = occupied_routes;
            bool location_ok = true;
            for (const auto& branch : selected_branches) {
                auto legal = legalize_branch_at_lcp_location(
                    circuit, request, evaluation, branch, lcp_id, location_id, tentative_occupied);
                if (!legal.success) {
                    location_ok = false;
                    break;
                }
                tentative_occupied.insert(tentative_occupied.end(), legal.routes.begin(), legal.routes.end());
                branch_results.push_back(std::move(legal));
            }
            if (!location_ok) continue;

            result.success = true;
            result.chosen_lcp_candidate_id = location_id;
            result.switched_lcp_location = location_id != preferred_location;
            result.branch_results = std::move(branch_results);
            return result;
        }
    }

    // 未启用协商或多 LCP 网时，保持 DP 选定的物理 LCP 绑定直到 detailed route 结束。
    for (const auto& branch : selected_branches) {
        if (!candidate_matches_lcp_bindings(branch, selected_bindings)) {
            result.failure_messages.push_back(net + ": selected branch is inconsistent with net LCP binding");
            return result;
        }
    }

    std::vector<RouteSegment> tentative_occupied = occupied_routes;
    for (const auto& branch : selected_branches) {
        auto legal = legalize_detailed_candidate(circuit, request, evaluation, branch, tentative_occupied);
        if (!legal.success) {
            result.failure_messages.insert(
                result.failure_messages.end(),
                legal.failure_messages.begin(),
                legal.failure_messages.end());
            result.failure_messages.push_back(
                net + ": could not legalize " + branch.from_terminal + "->" + branch.to_terminal +
                " with selected LCP bindings");
            return result;
        }
        if (!candidate_matches_lcp_bindings(legal.candidate, selected_bindings)) {
            result.failure_messages.push_back(net + ": legalization changed an LCP binding unexpectedly");
            return result;
        }
        tentative_occupied.insert(tentative_occupied.end(), legal.routes.begin(), legal.routes.end());
        result.branch_results.push_back(std::move(legal));
    }
    result.success = true;
    result.chosen_lcp_candidate_id = selected_bindings.begin()->second;
    result.failure_messages.clear();
    return result;

    // 鏍￠獙 selected 鏀矾鍦ㄨ繘鍏?detailed 鍓嶅凡鍏变韩鍚屼竴 LCP 缁戝畾銆?
}

// 灏嗗€欓€夎矾寰勫啓鍏?detailed route锛屽悓鏃惰褰?route segment 鍒?LCP/space-node 鐨勫洖婧槧灏勩€?
void append_traced_detailed_path(
    DetailedRoutingResult& result,
    DetailedRouteTrace& trace,
    const DetailedTopologyIndex& index,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate,
    const std::vector<RouteSegment>& routes,
    int dp_state_id,
    const std::string& tree_node) {
    append_trace_node(trace, index, evaluation.context, candidate.from_terminal);
    append_trace_node(trace, index, evaluation.context, candidate.to_terminal);
    const std::size_t first_route = result.routes.size();
    result.routes.insert(result.routes.end(), routes.begin(), routes.end());
    const std::string lcp_id = lcp_id_for_candidate(index, candidate);
    const std::string space_node_id = space_node_for_candidate(index, candidate);
    for (std::size_t route_index = first_route; route_index < result.routes.size(); ++route_index) {
        trace.segments.push_back(DetailedRouteSegment{
            route_index,
            dp_state_id,
            candidate.net,
            candidate.from_terminal,
            candidate.to_terminal,
            tree_node,
            candidate.segment_id,
            lcp_id,
            candidate.lcp_candidate_id,
            space_node_id,
        });
    }
}

// 鏍规嵁 detailed route 鐨勫疄闄呯嚎瀹芥洿鏂版墍灞?space node 鐨勯鐣欑┖闂撮渶姹傘€?
void update_space_requirement(
    DetailedRoutingResult& result,
    const std::string& space_node_id,
    double route_width) {
    if (space_node_id.empty()) return;
    const double required = route_width + kDetailedSpacing;
    const auto found = result.required_space_by_node.find(space_node_id);
    if (found == result.required_space_by_node.end()) {
        result.required_space_by_node[space_node_id] = required;
    } else {
        found->second = std::max(found->second, required);
    }
}

// 璁板綍 detailed traceback 澶辫触锛屽苟鎶婂け璐ュ啓鍏?report 鍜?penalty銆?
void add_traceback_failure(DetailedRoutingResult& result, DetailedRouteTrace& trace, const std::string& message) {
    ++result.traceback_failures;
    result.routing_failure_penalty += kDetailedFailurePenalty;
    trace.warnings.push_back(message);
    result.report.warnings.push_back(message);
}

// 灏?route segment 杞负閲戝睘鍗犵敤鐭╁舰锛岀敤浜?DRC 鍜?coupling 妫€鏌ャ€?
Rect route_to_rect(const RouteSegment& route) {
    return routing::segment_to_rect(
        routing::Segment{routing::Point{route.x1, route.y1}, routing::Point{route.x2, route.y2}},
        route.width);
}

// 鍒ゆ柇涓や釜杩炵画鍧愭爣鏄惁瓒冲鎺ヨ繎锛岀敤浜庡尮閰?pin access 璧风偣銆?
bool near_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-6;
}

// 鍒ゆ柇 route 绔偣鏄惁璐磋繎鏌愪釜鐪熷疄 pin锛岄伩鍏嶆妸浠绘剰 active 鍐呯鐐归兘褰撲綔 pin access銆?
bool endpoint_matches_pin(const routing::Point& point, const std::string& layer, const RoutingEvaluationRequest& request) {
    constexpr double kPinAccessSnapTolerance = 1.0;
    (void)layer;
    for (const auto& pin : request.placed_pins) {
        if (std::abs(point.x - pin.x) <= kPinAccessSnapTolerance &&
            std::abs(point.y - pin.y) <= kPinAccessSnapTolerance) {
            return true;
        }
    }
    return false;
}

// 鍒ゆ柇鐐规槸鍚︿綅浜?active 鍐呴儴锛屼笉鎶婅竟鐣屾帴瑙﹀綋浣滃唴閮ㄧ┛瓒娿€?
bool point_strictly_inside(const Rect& active, const routing::Point& point) {
    const Rect rect = routing::normalize_rect(active);
    constexpr double kBoundaryTolerance = 0.1;
    return point.x > rect.x1 + kBoundaryTolerance && point.x < rect.x2 - kBoundaryTolerance &&
           point.y > rect.y1 + kBoundaryTolerance && point.y < rect.y2 - kBoundaryTolerance;
}

// 鍒ゆ柇鐐规槸鍚︿綅浜?active 杈圭晫涓娿€?
bool point_on_active_boundary(const Rect& active, const routing::Point& point) {
    const Rect rect = routing::normalize_rect(active);
    constexpr double kBoundaryTolerance = 0.1;
    const bool on_vertical =
        (std::abs(point.x - rect.x1) <= kBoundaryTolerance || std::abs(point.x - rect.x2) <= kBoundaryTolerance) &&
        point.y >= rect.y1 - kBoundaryTolerance && point.y <= rect.y2 + kBoundaryTolerance;
    const bool on_horizontal =
        (std::abs(point.y - rect.y1) <= kBoundaryTolerance || std::abs(point.y - rect.y2) <= kBoundaryTolerance) &&
        point.x >= rect.x1 - kBoundaryTolerance && point.x <= rect.x2 + kBoundaryTolerance;
    return on_vertical || on_horizontal;
}

// 杩斿洖浠?active 鍐呯鐐规部褰撳墠绾挎閫冮€稿埌杈圭晫鐨勮窛绂伙紱闈炴浜ら€冮€歌繑鍥炴棤鏁堝€笺€?
std::optional<double> access_distance_to_boundary(const routing::Point& inside, const routing::Point& outside, const Rect& active) {
    const Rect rect = routing::normalize_rect(active);
    if (near_coord(inside.x, outside.x)) {
        if (outside.y < rect.y1) return inside.y - rect.y1;
        if (outside.y > rect.y2) return rect.y2 - inside.y;
        return std::nullopt;
    }
    if (near_coord(inside.y, outside.y)) {
        if (outside.x < rect.x1) return inside.x - rect.x1;
        if (outside.x > rect.x2) return rect.x2 - inside.x;
        return std::nullopt;
    }
    return std::nullopt;
}

// 杩斿洖 pin 娌垮綋鍓?access 鏂瑰悜鍒?active 杈圭晫鐨勮窛绂汇€?
std::optional<double> access_distance_from_pin_to_boundary(
    const routing::Point& pin,
    const routing::Point& toward,
    const Rect& active) {
    const Rect rect = routing::normalize_rect(active);
    if (near_coord(pin.x, toward.x)) {
        if (toward.y < pin.y) return pin.y - rect.y1;
        if (toward.y > pin.y) return rect.y2 - pin.y;
        return std::nullopt;
    }
    if (near_coord(pin.y, toward.y)) {
        if (toward.x < pin.x) return pin.x - rect.x1;
        if (toward.x > pin.x) return rect.x2 - pin.x;
        return std::nullopt;
    }
    return std::nullopt;
}

// 鍙厑璁哥湡瀹?pin 闄勮繎鐨勪竴灏忔 active 閫冮€歌蛋绾匡紝绂佹鐢?pin 绔偣璞佸厤闀胯窛绂绘í绌?active銆?
bool route_is_local_pin_access(
    const RoutingEvaluationRequest& request,
    const RouteSegment& route,
    const Rect& active) {
    const Rect rect = routing::normalize_rect(active);
    const routing::Point first{route.x1, route.y1};
    const routing::Point second{route.x2, route.y2};
    const bool first_inside = routing::contains_point(rect, first);
    const bool second_inside = routing::contains_point(rect, second);
    const bool first_strict_inside = point_strictly_inside(rect, first);
    const bool second_strict_inside = point_strictly_inside(rect, second);
    const bool first_on_boundary = point_on_active_boundary(rect, first);
    const bool second_on_boundary = point_on_active_boundary(rect, second);
    if (!first_strict_inside && !second_strict_inside && (first_on_boundary != second_on_boundary)) return true;

    const double max_access_length = std::min(2.0, 0.6 * std::min(routing::rect_width(rect), routing::rect_height(rect)));
    if (first_inside != second_inside) {
        const routing::Point inside = first_inside ? first : second;
        const routing::Point outside = first_inside ? second : first;
        if (!endpoint_matches_pin(inside, route.layer, request)) return false;

        const auto distance = access_distance_to_boundary(inside, outside, rect);
        if (!distance.has_value() || *distance < -1e-6) return false;
        return *distance <= max_access_length + 1e-6;
    }
    if (first_inside && second_inside) {
        const double route_length = std::abs(first.x - second.x) + std::abs(first.y - second.y);
        if ((endpoint_matches_pin(first, route.layer, request) || endpoint_matches_pin(second, route.layer, request)) &&
            route_length <= max_access_length + 1e-6) {
            return true;
        }
        if (endpoint_matches_pin(first, route.layer, request)) {
            const auto distance = access_distance_from_pin_to_boundary(first, second, rect);
            return distance.has_value() && *distance <= max_access_length + 1e-6;
        }
        if (endpoint_matches_pin(second, route.layer, request)) {
            const auto distance = access_distance_from_pin_to_boundary(second, first, rect);
            return distance.has_value() && *distance <= max_access_length + 1e-6;
        }
    }
    return false;
}

// 鏀堕泦 detailed route 绌胯秺 active region 鐨勫熀纭€ DRC 杩濆弽绾挎绱㈠紩銆?
// 浠呮鏌?M1锛歛ctive region 瀵瑰簲鍣ㄤ欢浣庡眰鍗犵敤锛岄珮灞傞噾灞炶法杩囦笉绠?active crossing銆?
std::vector<std::size_t> collect_active_region_crossings(
    const RoutingEvaluationRequest& request,
    const std::vector<RouteSegment>& routes) {
    std::vector<std::size_t> violations;
    for (std::size_t index = 0; index < routes.size(); ++index) {
        const auto& route = routes[index];
        if (route.layer != "M1") continue;
        const Rect metal = route_to_rect(route);
        for (const auto& active : request.active_region_blockers) {
            if (!routing::intersects(metal, active)) continue;
            if (route_is_local_pin_access(request, route, active)) continue;
            violations.push_back(index);
            break;
        }
    }
    return violations;
}

// 写入一条候选在 detailed 阶段的结构化落地结果，避免仅靠 warning 文本判断最终状态。
void append_detailed_transition_outcome(
    DetailedRoutingResult& result,
    const routing::RouteCandidate& candidate,
    bool selected_by_dp,
    bool legalized,
    const std::string& failure_reason = {}) {
    result.transition_outcomes.push_back(DetailedTransitionOutcome{
        candidate.net,
        candidate.from_terminal,
        candidate.to_terminal,
        candidate.segment_id,
        candidate.lcp_id,
        candidate.source_lcp_id,
        candidate.target_lcp_id,
        selected_by_dp,
        true,
        legalized,
        legalized,
        legalized ? std::string{} : "detailed_route_legalization",
        failure_reason,
    });
}

// 判断整组候选金属是否包含 active-region DRC，供 detailed 候选筛选和 A* 重布线共用。
bool routes_violate_active_regions(
    const RoutingEvaluationRequest& request,
    const std::vector<RouteSegment>& routes) {
    return !collect_active_region_crossings(request, routes).empty();
}

// 鍒ゆ柇涓ゆ潯鍚屽眰寮傜綉绾挎鏄惁瀛樺湪杩戣窛绂诲钩琛岃€﹀悎椋庨櫓銆?
// 鏀堕泦鍚屽眰寮傜綉閲戝睘閲嶅彔锛岄噸鍙犱唬琛ㄧ湡瀹炵煭璺紝涓嶈兘鍙綔涓?coupling 椋庨櫓澶勭悊銆?
std::vector<std::pair<std::size_t, std::size_t>> collect_same_layer_shorts(const std::vector<RouteSegment>& routes) {
    std::vector<std::pair<std::size_t, std::size_t>> findings;
    for (std::size_t i = 0; i < routes.size(); ++i) {
        for (std::size_t j = i + 1; j < routes.size(); ++j) {
            if (routing::same_layer_short(routes[i], routes[j])) {
                findings.push_back({i, j});
            }
        }
    }
    return findings;
}

bool near_parallel_coupling(const RouteSegment& lhs, const RouteSegment& rhs, double spacing) {
    if (lhs.net == rhs.net || lhs.layer != rhs.layer) return false;
    const bool lhs_horizontal = route_is_horizontal(lhs);
    const bool rhs_horizontal = route_is_horizontal(rhs);
    if (lhs_horizontal != rhs_horizontal) return false;
    if (lhs_horizontal) {
        const double distance = std::abs(lhs.y1 - rhs.y1) - (lhs.width + rhs.width) / 2.0;
        const bool overlap = std::max(std::min(lhs.x1, lhs.x2), std::min(rhs.x1, rhs.x2)) <=
                             std::min(std::max(lhs.x1, lhs.x2), std::max(rhs.x1, rhs.x2));
        return distance < spacing && overlap;
    }
    const double distance = std::abs(lhs.x1 - rhs.x1) - (lhs.width + rhs.width) / 2.0;
    const bool overlap = std::max(std::min(lhs.y1, lhs.y2), std::min(rhs.y1, rhs.y2)) <=
                         std::min(std::max(lhs.y1, lhs.y2), std::max(rhs.y1, rhs.y2));
    return distance < spacing && overlap;
}

// 鏀堕泦 detailed route 鐨勫悓灞傚钩琛岃€﹀悎绾挎瀵广€?
[[maybe_unused]] std::vector<std::pair<std::size_t, std::size_t>> collect_detailed_coupling_pairs(const std::vector<RouteSegment>& routes) {
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    for (std::size_t i = 0; i < routes.size(); ++i) {
        for (std::size_t j = i + 1; j < routes.size(); ++j) {
            if (near_parallel_coupling(routes[i], routes[j], kDetailedSpacing)) pairs.push_back({i, j});
        }
    }
    return pairs;
}

struct CouplingFinding {
    std::size_t left{};
    std::size_t right{};
    double overlap_length{};
    double spacing{};
};

// 璁＄畻涓ゆ潯鍚屽眰寮傜綉绾挎鐨勫钩琛岄噸鍙犻暱搴︼紱璺濈瓒呰繃 spacing 鏃惰繑鍥?0銆?
double parallel_overlap_length(const RouteSegment& lhs, const RouteSegment& rhs, double spacing) {
    if (lhs.net == rhs.net || lhs.layer != rhs.layer) return 0.0;
    const bool lhs_horizontal = route_is_horizontal(lhs);
    const bool rhs_horizontal = route_is_horizontal(rhs);
    if (lhs_horizontal != rhs_horizontal) return 0.0;
    if (lhs_horizontal) {
        const double distance = std::abs(lhs.y1 - rhs.y1) - (lhs.width + rhs.width) / 2.0;
        if (distance >= spacing) return 0.0;
        const double overlap_start = std::max(std::min(lhs.x1, lhs.x2), std::min(rhs.x1, rhs.x2));
        const double overlap_end = std::min(std::max(lhs.x1, lhs.x2), std::max(rhs.x1, rhs.x2));
        return std::max(0.0, overlap_end - overlap_start);
    }
    const double distance = std::abs(lhs.x1 - rhs.x1) - (lhs.width + rhs.width) / 2.0;
    if (distance >= spacing) return 0.0;
    const double overlap_start = std::max(std::min(lhs.y1, lhs.y2), std::min(rhs.y1, rhs.y2));
    const double overlap_end = std::min(std::max(lhs.y1, lhs.y2), std::max(rhs.y1, rhs.y2));
    return std::max(0.0, overlap_end - overlap_start);
}

// 鏀堕泦 detailed route 鐨勫悓灞傚钩琛岃€﹀悎绾挎瀵癸紝骞惰褰曞疄闄呴噸鍙犻暱搴︺€?
std::vector<CouplingFinding> collect_detailed_coupling_findings(const std::vector<RouteSegment>& routes) {
    std::vector<CouplingFinding> findings;
    for (std::size_t i = 0; i < routes.size(); ++i) {
        for (std::size_t j = i + 1; j < routes.size(); ++j) {
            const double overlap = parallel_overlap_length(routes[i], routes[j], kDetailedSpacing);
            if (overlap > 0.0) findings.push_back({i, j, overlap, kDetailedSpacing});
        }
    }
    return findings;
}

}  // namespace

// 杩斿洖 net priority 鐨?detailed routing 鍥炴函椤哄簭鏉冮噸銆?
// 输出每个 LCP 候选点是否覆盖全部 incident segments 的诊断信息。
std::vector<LcpCandidateCoverage> analyze_lcp_candidate_coverage(
    const RoutingEvaluationRequest& request,
    const std::vector<routing::RouteCandidate>& candidates) {
    return collect_lcp_candidate_coverage(request, candidates);
}

// 鎸夎鏂?detailed routing 浼樺厛绾ф帓搴?net route銆?
bool module_in_symmetry_constraints(const Circuit& circuit, const std::string& module) {
    for (const auto& pair : circuit.constraints.symmetry_pairs) {
        if (pair.left == module || pair.right == module) return true;
    }
    for (const auto& self : circuit.constraints.symmetry_selfs) {
        if (self.module == module) return true;
    }
    return false;
}

bool net_touches_symmetry_group(const Circuit& circuit, const std::string& net) {
    const auto found = circuit.nets.find(net);
    if (found == circuit.nets.end()) return false;
    for (const auto& terminal : found->second.terminals) {
        const auto pin = circuit.pins.find(terminal);
        if (pin != circuit.pins.end() && module_in_symmetry_constraints(circuit, pin->second.module)) return true;
    }
    return false;
}

// 按对称关系与关键性共同确定 detailed routing 的四级提交顺序；线宽约束只参与 DRC，不降低关键网顺序。
int detailed_net_rank(const Circuit& circuit, const std::string& net) {
    const auto found = circuit.nets.find(net);
    const Priority priority = found == circuit.nets.end() ? Priority::Normal : found->second.priority;
    const bool symmetry_related = net_touches_symmetry_group(circuit, net) || priority == Priority::Symmetry;
    if (symmetry_related && priority == Priority::Critical) return 0;
    if (symmetry_related) return 1;
    if (priority == Priority::Critical) return 2;
    return 3;
}

std::vector<const routing::NetRouteChoice*> ordered_detailed_routes(
    const Circuit& circuit,
    const RoutingEvaluation& evaluation) {
    std::vector<const routing::NetRouteChoice*> routes;
    for (const auto& route : evaluation.global_routing.net_routes) routes.push_back(&route);
    std::stable_sort(routes.begin(), routes.end(), [&](const auto* left, const auto* right) {
        return detailed_net_rank(circuit, left->net) < detailed_net_rank(circuit, right->net);
    });
    return routes;
}

// 杩斿洖 detailed routing 搴斾娇鐢ㄧ殑鍊欓€夎矾寰勶紝浼樺厛浣跨敤 bottom-up DP traceback銆?
std::vector<routing::RouteCandidate> selected_candidates_for_detailed_routing(
    const RoutingEvaluation& evaluation) {
    if (evaluation.bottom_up_dp.has_value()) {
        if (evaluation.bottom_up_dp->success) {
            std::vector<routing::RouteCandidate> selected = evaluation.bottom_up_dp->traceback_candidates;
            std::unordered_set<std::string> covered_nets;
            for (const auto& candidate : selected) covered_nets.insert(candidate.net);
            for (const auto& net_route : evaluation.global_routing.net_routes) {
                if (!net_route.success || covered_nets.contains(net_route.net)) continue;
                selected.insert(
                    selected.end(),
                    net_route.selected_candidates.begin(),
                    net_route.selected_candidates.end());
            }
            return selected;
        }
    }
    std::vector<routing::RouteCandidate> candidates;
    for (const auto& net_route : evaluation.global_routing.net_routes) {
        if (!net_route.success) continue;
        candidates.insert(candidates.end(), net_route.selected_candidates.begin(), net_route.selected_candidates.end());
    }
    return candidates;
}

// 鎸?NetTopology 涓殑 wire segment 椤哄簭鎭㈠ detailed routing 鍊欓€夛紝浣撶幇 top-down traceback 椤哄簭銆?
std::vector<routing::RouteCandidate> order_candidates_by_topology(
    const RoutingEvaluationRequest& request,
    const std::vector<routing::RouteCandidate>& candidates) {
    if (request.net_topologies.empty()) return candidates;

    std::vector<routing::RouteCandidate> ordered;
    std::vector<bool> used(candidates.size(), false);
    for (const auto& topology : request.net_topologies) {
        for (const auto& segment : topology.segments) {
            for (std::size_t index = 0; index < candidates.size(); ++index) {
                if (used[index]) continue;
                const auto& candidate = candidates[index];
                if (candidate.net != segment.net) continue;
                const bool same_id = !segment.id.empty() && candidate.segment_id == segment.id;
                const bool same_direction =
                    candidate.from_terminal == segment.from && candidate.to_terminal == segment.to;
                const bool reverse_direction =
                    candidate.from_terminal == segment.to && candidate.to_terminal == segment.from;
                if (!same_id && !same_direction && !reverse_direction) continue;
                ordered.push_back(candidate);
                used[index] = true;
                break;
            }
        }
    }
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (!used[index]) ordered.push_back(candidates[index]);
    }
    return ordered;
}

// 鏍规嵁褰撳墠 placement 鎵ц甯冪嚎涓婁笅鏂囨瀯寤恒€佸€欓€夎矾寰勭敓鎴愬拰鍏ㄥ眬璺緞閫夋嫨銆?
// 鍒ゆ柇涓€涓?net 鐨?detailed traceback 鏄惁鑳戒粠 FLOW out pin 杩借釜鍒?in pin銆?
/*
 * 根据 packing contour trace 建立 detailed routing 的逆序回溯索引。
 * 论文要求从增强 B*-tree 叶子开始按 packing 逆序回溯；wire segment 的 packing
 * 归属优先使用 DP 已记录的 local/cross-child segment，未记录时才回退到 LCP
 * space owner 或端点模块，避免直接网络失去排序依据。
 */
struct DetailedPackingTraceOrder {
    std::unordered_map<std::string, int> segment_rank_by_id;
    std::unordered_map<std::string, int> module_rank;
};

/* 构建 wire segment 与模块在 contour packing 中最后出现的索引。 */
DetailedPackingTraceOrder build_detailed_packing_trace_order(const RoutingEvaluationRequest& request) {
    DetailedPackingTraceOrder order;
    for (std::size_t index = 0; index < request.packing_trace.steps.size(); ++index) {
        const auto& step = request.packing_trace.steps[index];
        const int rank = static_cast<int>(index);
        const auto record_module = [&](const std::string& module) {
            if (module.empty()) return;
            const auto found = order.module_rank.find(module);
            if (found == order.module_rank.end() || found->second < rank) order.module_rank[module] = rank;
        };
        record_module(step.module);
        for (const auto& module : step.subtree_modules) record_module(module);
        const auto record_segment = [&](const std::string& segment_id) {
            if (segment_id.empty()) return;
            const auto found = order.segment_rank_by_id.find(segment_id);
            if (found == order.segment_rank_by_id.end() || found->second < rank) {
                order.segment_rank_by_id[segment_id] = rank;
            }
        };
        for (const auto& segment_id : step.local_wire_segments) record_segment(segment_id);
        for (const auto& segment_id : step.cross_child_wire_segments) record_segment(segment_id);
    }
    return order;
}

/* 返回 terminal 所属模块在 packing trace 中的逆序回溯优先级。 */
int packing_rank_for_terminal(
    const Circuit& circuit,
    const DetailedPackingTraceOrder& order,
    const std::string& terminal) {
    const auto pin = circuit.pins.find(terminal);
    if (pin == circuit.pins.end()) return -1;
    const auto module = order.module_rank.find(pin->second.module);
    return module == order.module_rank.end() ? -1 : module->second;
}

/* 返回 candidate 应在 detailed routing 中回溯的 packing 索引，较大值表示更靠近叶子。 */
int detailed_packing_rank(
    const Circuit& circuit,
    const DetailedTopologyIndex& topology_index,
    const DetailedPackingTraceOrder& order,
    const routing::RouteCandidate& candidate) {
    const auto segment = order.segment_rank_by_id.find(candidate.segment_id);
    if (segment != order.segment_rank_by_id.end()) return segment->second;

    int rank = std::max(
        packing_rank_for_terminal(circuit, order, candidate.from_terminal),
        packing_rank_for_terminal(circuit, order, candidate.to_terminal));
    for (const auto& space_id : space_nodes_for_candidate(topology_index, candidate)) {
        const auto owner_id = topology_index.space_owner_by_id.find(space_id);
        if (owner_id == topology_index.space_owner_by_id.end()) continue;
        const auto owner_rank = order.module_rank.find(owner_id->second);
        if (owner_rank != order.module_rank.end()) rank = std::max(rank, owner_rank->second);
    }
    return rank;
}

/*
 * 仅在成功的 bottom-up DP traceback 中按论文规定应用逆 packing 顺序。
 * 对称网和关键网仍优先；同一 net 保持连续，防止多端 LCP 的物理位置绑定被拆散。
 */
std::vector<routing::RouteCandidate> order_candidates_for_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const DetailedTopologyIndex& topology_index,
    const std::vector<routing::RouteCandidate>& candidates,
    bool use_dp_traceback) {
    auto ordered = order_candidates_by_topology(request, candidates);
    // 论文的 top-down detailed routing 仅回溯成功构造的 DP 子问题；fallback 没有合法 traceback，保留原顺序。
    if (!use_dp_traceback) {
        std::stable_sort(ordered.begin(), ordered.end(), [&](const auto& left, const auto& right) {
            return detailed_net_rank(circuit, left.net) < detailed_net_rank(circuit, right.net);
        });
        return ordered;
    }
    const auto packing_order = build_detailed_packing_trace_order(request);

    struct NetCandidateGroup {
        std::string net;
        int priority_rank{};
        int packing_rank{-1};
        std::vector<routing::RouteCandidate> candidates;
    };
    std::vector<NetCandidateGroup> groups;
    std::unordered_map<std::string, std::size_t> group_index_by_net;
    for (const auto& candidate : ordered) {
        const auto [found, inserted] = group_index_by_net.emplace(candidate.net, groups.size());
        if (inserted) groups.push_back({candidate.net, detailed_net_rank(circuit, candidate.net), -1, {}});
        auto& group = groups[found->second];
        group.packing_rank = std::max(
            group.packing_rank,
            detailed_packing_rank(circuit, topology_index, packing_order, candidate));
        group.candidates.push_back(candidate);
    }
    for (auto& group : groups) {
        std::stable_sort(group.candidates.begin(), group.candidates.end(), [&](const auto& left, const auto& right) {
            return detailed_packing_rank(circuit, topology_index, packing_order, left) >
                   detailed_packing_rank(circuit, topology_index, packing_order, right);
        });
    }
    std::stable_sort(groups.begin(), groups.end(), [](const auto& left, const auto& right) {
        if (left.priority_rank != right.priority_rank) return left.priority_rank < right.priority_rank;
        return left.packing_rank > right.packing_rank;
    });

    std::vector<routing::RouteCandidate> result;
    for (const auto& group : groups) {
        result.insert(result.end(), group.candidates.begin(), group.candidates.end());
    }
    return result;
}

bool has_flow_path(
    const std::string& out_pin,
    const std::string& in_pin,
    const std::unordered_map<std::string, std::vector<std::string>>& adjacency) {
    std::vector<std::string> frontier{out_pin};
    std::unordered_set<std::string> visited;
    while (!frontier.empty()) {
        const std::string current = frontier.back();
        frontier.pop_back();
        if (current == in_pin) return true;
        if (!visited.insert(current).second) continue;
        const auto next = adjacency.find(current);
        if (next == adjacency.end()) continue;
        frontier.insert(frontier.end(), next->second.begin(), next->second.end());
    }
    return false;
}

// 鍩轰簬 DP traceback 鍚庣殑鏈夊悜鎷撴墤妫€鏌?FLOW 绾︽潫锛屽苟鎶婂け璐ユ潵婧愬啓鍏?detailed report銆?
void apply_detailed_flow_check(
    const Circuit& circuit,
    const std::vector<routing::RouteCandidate>& candidates,
    DetailedRoutingResult& result) {
    for (const auto& flow : circuit.constraints.flows) {
        std::unordered_map<std::string, std::vector<std::string>> adjacency;
        bool saw_net = false;
        for (const auto& candidate : candidates) {
            if (candidate.net != flow.net) continue;
            saw_net = true;
            adjacency[candidate.from_terminal].push_back(candidate.to_terminal);
            if (candidate.from_terminal == flow.in_pin && candidate.to_terminal == flow.out_pin) {
                ++result.flow_violations;
                result.flow_penalty += kDetailedFlowPenalty;
                const std::string message =
                    flow.net + ": reverse FLOW segment " + candidate.from_terminal + "->" + candidate.to_terminal;
                result.report.flow_segments.push_back(message);
                result.report.warnings.push_back(message);
            }
        }
        if (!saw_net) continue;
        if (!has_flow_path(flow.out_pin, flow.in_pin, adjacency)) {
            ++result.flow_violations;
            result.flow_penalty += kDetailedFlowPenalty;
            const std::string message = flow.net + ": no detailed FLOW path " + flow.out_pin + "->" + flow.in_pin;
            result.report.flow_segments.push_back(message);
            result.report.warnings.push_back(message);
        }
    }
}

RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements) {
    routing::RoutingContext context(circuit, placements);
    auto candidates = routing::generate_route_candidates(circuit, context);
    for (auto& candidate : candidates) {
        annotate_candidate(circuit, candidate, 50000.0, 50000.0);
    }
    return make_evaluation(std::move(context), std::move(candidates), circuit);
}

// 鏍规嵁 placement candidate 涓殑 LCP 鎷撴墤鎵ц A*/DP 甯冪嚎璇勪及銆?
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request) {
    routing::RoutingContext context(
        circuit,
        request.placements,
        routing::make_grid_config_for_routing_layers(request.routing_layers),
        lcp_routing_points(request));
    auto direct_candidates = routing::generate_route_candidates(circuit, context);
    for (auto& candidate : direct_candidates) {
        annotate_candidate(circuit, candidate, 50000.0, 50000.0);
    }
    auto candidates = direct_candidates;
    std::vector<routing::RouteCandidate> debug_candidates = direct_candidates;
    std::vector<LcpCandidateFilterEvent> lcp_candidate_filter_events;
    const auto lcp_nets = nets_with_lcp_topology(request);
    if (!lcp_nets.empty()) {
        candidates.erase(
            std::remove_if(
                candidates.begin(),
                candidates.end(),
                [&](const auto& candidate) { return lcp_nets.contains(candidate.net); }),
            candidates.end());
        debug_candidates.erase(
            std::remove_if(
                debug_candidates.begin(),
                debug_candidates.end(),
                [&](const auto& candidate) { return lcp_nets.contains(candidate.net); }),
            debug_candidates.end());
        std::vector<routing::RouteCandidate> raw_lcp_candidates;
        auto lcp_candidates = generate_lcp_route_candidates(
            context,
            request,
            circuit,
            &raw_lcp_candidates,
            &lcp_candidate_filter_events);
        candidates.insert(
            candidates.end(),
            std::make_move_iterator(lcp_candidates.begin()),
            std::make_move_iterator(lcp_candidates.end()));
        debug_candidates.insert(
            debug_candidates.end(),
            std::make_move_iterator(raw_lcp_candidates.begin()),
            std::make_move_iterator(raw_lcp_candidates.end()));
    }
    if (request.net_topologies.empty() || !request.tree.root.has_value()) {
        return make_evaluation(
            std::move(context),
            std::move(candidates),
            circuit,
            std::nullopt,
            std::move(debug_candidates),
            false,
            std::move(lcp_candidate_filter_events));
    }
    auto bottom_up_dp = routing::run_bottom_up_routing_dp(
        circuit, request, context, candidates, request.dp_beam_width);
    if (!bottom_up_dp.success) {
        if (request.strict_lcp_dp && !lcp_nets.empty()) {
            return make_evaluation(
                std::move(context),
                std::move(candidates),
                circuit,
                std::move(bottom_up_dp),
                std::move(debug_candidates),
                true,
                std::move(lcp_candidate_filter_events));
        }
        return make_evaluation(
            std::move(context),
            std::move(direct_candidates),
            circuit,
            std::move(bottom_up_dp),
            std::move(debug_candidates),
            false,
            std::move(lcp_candidate_filter_events));
    }
    return make_evaluation(
        std::move(context),
        std::move(candidates),
        circuit,
        std::move(bottom_up_dp),
        std::move(debug_candidates),
        false,
        std::move(lcp_candidate_filter_events));
}

// 灏?DP 鍏ㄥ眬甯冪嚎閫変腑鐨?A* 缃戞牸璺緞杞崲涓哄綋鍓?routing.txt 浣跨敤鐨勪腑蹇冪嚎绾挎銆?
std::vector<RouteSegment> selected_candidates_to_segments(
    const RoutingEvaluation& evaluation) {
    std::vector<RouteSegment> routes;
    for (const auto& candidate : selected_candidates_for_detailed_routing(evaluation)) {
        append_path_segments(routes, evaluation, candidate);
    }
    // active 边界会在路径转线段时切开；再次合并会把合法的 pin access 段伪造成穿越 active 的长线段。
    if (!evaluation.context.active_regions().empty()) return routes;
    return routing::merge_collinear_same_net_routes(routes);
}

// 鎵ц璁烘枃 top-down detailed routing 闃舵锛屽綋鍓嶅熀浜?DP 閫変腑瀛愰棶棰樺洖婧苟娓呯悊璺緞銆?
DetailedRoutingResult run_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation) {
    DetailedRoutingResult result;
    const auto topology_index = build_detailed_topology_index(request);
    std::unordered_set<std::string> routed_space_nodes;
    const bool has_dp_traceback = evaluation.bottom_up_dp.has_value() && evaluation.bottom_up_dp->success;
    const auto selected_candidates =
        order_candidates_for_detailed_routing(
            circuit,
            request,
            topology_index,
            selected_candidates_for_detailed_routing(evaluation),
            has_dp_traceback);
    const int dp_state_id = has_dp_traceback ? evaluation.bottom_up_dp->best_state.id : -1;
    const std::string tree_node = has_dp_traceback ? evaluation.bottom_up_dp->best_state.tree_node : std::string{};
    // 判断 detailed 候选是否来自 DP traceback；未覆盖 net 的全局候选会被标记为非 DP 选择。
    const auto selected_by_dp = [&](const routing::RouteCandidate& candidate) {
        if (!has_dp_traceback) return false;
        return std::any_of(
            evaluation.bottom_up_dp->traceback_candidates.begin(),
            evaluation.bottom_up_dp->traceback_candidates.end(),
            [&](const auto& traceback) {
                return traceback.net == candidate.net &&
                       traceback.from_terminal == candidate.from_terminal &&
                       traceback.to_terminal == candidate.to_terminal &&
                       traceback.segment_id == candidate.segment_id &&
                       traceback.lcp_candidate_id == candidate.lcp_candidate_id &&
                       traceback.source_lcp_candidate_id == candidate.source_lcp_candidate_id &&
                       traceback.target_lcp_candidate_id == candidate.target_lcp_candidate_id;
            });
    };
    if (evaluation.bottom_up_dp.has_value() && !evaluation.bottom_up_dp->success) {
        auto& trace = trace_for_net(result.report, "__dp__");
        if (evaluation.bottom_up_dp->best_state.failure_messages.empty()) {
            add_traceback_failure(result, trace, "bottom-up DP failed without a legal topology traceback");
        } else {
            for (const auto& failure : evaluation.bottom_up_dp->best_state.failure_messages) {
                add_traceback_failure(result, trace, failure);
            }
        }
    }

    // 鎸?net 鍒嗙粍锛屼繚鐣?priority/topology 椤哄簭锛涘甫 LCP 鐨?net 鏁寸綉閿佸畾鍚屼竴 location銆?
    std::vector<std::vector<routing::RouteCandidate>> net_groups;
    for (const auto& candidate : selected_candidates) {
        if (net_groups.empty() || net_groups.back().front().net != candidate.net) {
            net_groups.push_back({candidate});
        } else {
            net_groups.back().push_back(candidate);
        }
    }

    std::vector<routing::RouteCandidate> legalized_candidates;
    routing::PathMetrics detailed_metrics;

    // 灏嗕竴鏉″凡鍚堟硶鍖栨敮璺啓鍏?detailed 缁撴灉锛屽苟鏇存柊 space / FLOW / 绾垮妫€鏌ャ€?
    auto commit_legalized_branch = [&](DetailedLegalization& legal, DetailedRouteTrace& trace) {
        const auto& actual_candidate = legal.candidate;
        if (legal.used_alternative) {
            trace.warnings.push_back(actual_candidate.net + ": detailed routing used alternative candidate");
            result.report.warnings.push_back(actual_candidate.net + ": detailed routing used alternative candidate");
        }
        if (legal.used_reroute) {
            trace.warnings.push_back(actual_candidate.net + ": detailed routing used A* reroute fallback");
            result.report.warnings.push_back(actual_candidate.net + ": detailed routing used A* reroute fallback");
        }
        append_traced_detailed_path(
            result,
            trace,
            topology_index,
            evaluation,
            actual_candidate,
            legal.routes,
            dp_state_id,
            tree_node);
        append_detailed_transition_outcome(result, actual_candidate, selected_by_dp(actual_candidate), true);
        detailed_metrics.wirelength += legal.metrics.wirelength;
        detailed_metrics.bend_count += legal.metrics.bend_count;
        detailed_metrics.via_count += legal.metrics.via_count;
        legalized_candidates.push_back(actual_candidate);
        const auto space_node_ids = space_nodes_for_candidate(topology_index, actual_candidate);
        const auto source_segment = wire_segment_for_candidate(topology_index, actual_candidate);
        for (const auto& space_node_id : space_node_ids) {
            routed_space_nodes.insert(space_node_id);
            update_space_requirement(
                result,
                space_node_id,
                detailed_width_for_candidate(circuit, evaluation, actual_candidate));
        }
        for (const auto& candidate_lcp_id : lcp_ids_for_candidate(topology_index, actual_candidate)) {
            if (topology_index.lcp_without_location.contains(candidate_lcp_id)) {
                add_traceback_failure(result, trace, "LCP " + candidate_lcp_id + " has no location candidate");
            }
        }
        if (!candidate_matches_lcp_topology(topology_index, actual_candidate)) {
            add_traceback_failure(
                result,
                trace,
                "selected candidate does not match LCP topology: " + actual_candidate.net + " " +
                    actual_candidate.from_terminal + " -> " + actual_candidate.to_terminal);
        }
        if (!actual_candidate.flow_ok) {
            trace.warnings.push_back(actual_candidate.net + ": candidate-level FLOW warning");
        }
        if (!actual_candidate.current_density_ok ||
            !candidate_width_satisfies_constraints(circuit, source_segment, actual_candidate)) {
            ++result.current_density_violations;
            result.current_density_penalty += kDetailedCurrentDensityPenalty;
            const std::string segment_id =
                source_segment.has_value() && !source_segment->id.empty() ? source_segment->id
                                                                         : actual_candidate.segment_id;
            const std::string message = actual_candidate.net + ": width violation at " + segment_id;
            trace.warnings.push_back(message);
            result.report.current_density_segments.push_back(message);
            result.report.warnings.push_back(message);
        }
    };

    for (const auto& group : net_groups) {
        auto& trace = trace_for_net(result.report, group.front().net);
        const bool is_lcp_net = std::any_of(group.begin(), group.end(), [](const auto& candidate) {
            return candidate_has_lcp_binding(candidate);
        });

        if (is_lcp_net) {
            auto net_legal = legalize_lcp_net(
                circuit, request, evaluation, group, result.routes, request.allow_lcp_location_negotiation);
            if (!net_legal.success) {
                for (const auto& failure : net_legal.failure_messages) {
                    trace.warnings.push_back(failure);
                    result.report.warnings.push_back(failure);
                }
                for (const auto& branch : group) {
                    const std::string failure_reason = net_legal.failure_messages.empty()
                                                           ? "LCP detailed legalization failed"
                                                           : net_legal.failure_messages.front();
                    append_detailed_transition_outcome(
                        result, branch, selected_by_dp(branch), false, failure_reason);
                    add_traceback_failure(
                        result,
                        trace,
                        "detailed routing could not legalize same-layer short for " + branch.net + " " +
                            branch.from_terminal + " -> " + branch.to_terminal);
                }
                continue;
            }
            if (net_legal.switched_lcp_location) {
                const std::string message =
                    group.front().net + ": negotiated whole-net LCP location to " +
                    net_legal.chosen_lcp_candidate_id;
                trace.warnings.push_back(message);
                result.report.warnings.push_back(message);
            }
            for (auto& legal : net_legal.branch_results) {
                commit_legalized_branch(legal, trace);
            }
            continue;
        }

        for (const auto& candidate : group) {
            auto legal = legalize_detailed_candidate(circuit, request, evaluation, candidate, result.routes);
            if (!legal.success) {
                for (const auto& failure : legal.failure_messages) {
                    trace.warnings.push_back(failure);
                    result.report.warnings.push_back(failure);
                }
                const std::string failure_reason = legal.failure_messages.empty()
                                                       ? "detailed legalization failed"
                                                       : legal.failure_messages.front();
                append_detailed_transition_outcome(
                    result, candidate, selected_by_dp(candidate), false, failure_reason);
                add_traceback_failure(
                    result,
                    trace,
                    "detailed routing could not legalize same-layer short for " + candidate.net + " " +
                        candidate.from_terminal + " -> " + candidate.to_terminal);
                continue;
            }
            commit_legalized_branch(legal, trace);
        }
    }
    apply_detailed_flow_check(circuit, legalized_candidates, result);
    // 保留 active 边界切段，避免输出归一化重新合并 pin access 段并引入伪 DRC。
    if (request.active_region_blockers.empty()) {
        result.routes = routing::merge_collinear_same_net_routes(result.routes);
    }
    // 在最终全局 DRC 前保留已成功 detailed legalization 的路线，避免 routes 清空后丢失诊断证据。
    result.raw_routes = result.routes;
    const auto drc_routes = collect_active_region_crossings(request, result.routes);
    const auto short_pairs = collect_same_layer_shorts(result.routes);
    result.design_rule_violations = static_cast<int>(drc_routes.size() + short_pairs.size());
    for (const auto route_index : drc_routes) {
        const auto& route = result.routes[route_index];
        const std::string message =
            route.net + ":" + route.layer + ":" + std::to_string(route_index) +
            " (" + std::to_string(route.x1) + "," + std::to_string(route.y1) + ")->(" +
            std::to_string(route.x2) + "," + std::to_string(route.y2) + ")";
        result.report.design_rule_segments.push_back(message);
        result.report.warnings.push_back("active-region DRC " + message);
    }
    for (const auto& [left_index, right_index] : short_pairs) {
        const auto& left = result.routes[left_index];
        const auto& right = result.routes[right_index];
        const std::string message =
            left.net + "<->" + right.net + ":" + left.layer + ":" +
            std::to_string(left_index) + "," + std::to_string(right_index);
        result.report.design_rule_segments.push_back(message);
        result.report.warnings.push_back("same-layer short DRC " + message);
    }
    result.design_rule_penalty = 100000.0 * static_cast<double>(result.design_rule_violations);
    const auto coupling_pairs = collect_detailed_coupling_findings(result.routes);
    for (const auto& coupling : coupling_pairs) {
        result.coupling_penalty +=
            kDetailedCouplingPenaltyPerPair * coupling.overlap_length / std::max(coupling.spacing, 1e-9);
        const auto& left_route = result.routes[coupling.left];
        const auto& right_route = result.routes[coupling.right];
        result.report.coupling_pairs.push_back(
            left_route.net + "<->" + right_route.net +
            " routes=" + std::to_string(coupling.left) + "," + std::to_string(coupling.right) +
            " overlap=" + std::to_string(coupling.overlap_length));
    }
    for (const auto& space_node_id : routed_space_nodes) {
        result.coupling_space_by_node[space_node_id] = result.coupling_penalty > 0.0 ? kDetailedSpacing : 0.0;
    }
    if (result.design_rule_violations > 0) {
        result.routing_failure_penalty += kDetailedFailurePenalty * static_cast<double>(result.design_rule_violations);
        result.report.warnings.push_back("detailed routing discarded routes with DRC violations");
        for (auto& outcome : result.transition_outcomes) outcome.final_output = false;
        result.routes.clear();
        routed_space_nodes.clear();
        detailed_metrics = routing::PathMetrics{};
    }
    result.space_nodes_with_routes = static_cast<int>(routed_space_nodes.size());
    result.detailed_routing_penalty =
        result.flow_penalty + result.current_density_penalty + result.design_rule_penalty +
        result.coupling_penalty + result.routing_failure_penalty;
    detailed_metrics.cost =
        detailed_metrics.wirelength + 3.0 * static_cast<double>(detailed_metrics.bend_count) +
        kDetailedViaWeight * static_cast<double>(detailed_metrics.via_count);
    result.detailed_wirelength = detailed_metrics.wirelength;
    result.detailed_bend_count = detailed_metrics.bend_count;
    result.detailed_via_count = detailed_metrics.via_count;
    result.detailed_cost = detailed_metrics.cost + result.detailed_routing_penalty;
    result.used_global_fallback = evaluation.bottom_up_dp.has_value() && !evaluation.bottom_up_dp->success;
    return result;
}

}  // namespace sapr
