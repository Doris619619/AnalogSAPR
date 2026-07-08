// 文件职责：实现 placement 到 A*/DP 布线评估结果的封装流程。
#include "sapr/routing_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
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

// 汇总 LCP、space node 和拓扑 segment 的快速查找表。
struct DetailedTopologyIndex {
    std::unordered_map<std::string, LinkingControlPoint> lcp_by_id;
    std::unordered_map<std::string, std::string> lcp_space_by_id;
    std::unordered_set<std::string> lcp_without_location;
    std::unordered_set<std::string> topology_segment_keys;
    std::vector<WireSegmentRef> topology_segments;
};

// 合并 DP traceback 候选和未被 DP 覆盖 net 的普通候选，避免 direct net 在 DP 成功后丢失。
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

// 执行候选路径生成与 DP 全局布线选择的公共封装。
RoutingEvaluation make_evaluation(
    routing::RoutingContext context,
    std::vector<routing::RouteCandidate> candidates,
    const Circuit& circuit,
    std::optional<routing::RoutingDpResult> bottom_up_dp = std::nullopt) {
    const bool use_dp = bottom_up_dp.has_value() && bottom_up_dp->success;
    const auto routing_candidates = use_dp
                                        ? merge_dp_traceback_with_uncovered_nets(candidates, bottom_up_dp->traceback_candidates)
                                        : candidates;
    auto global_routing = routing::run_global_routing(circuit, context, routing_candidates);
    RoutingEvaluation evaluation{
        std::move(context),
        std::move(candidates),
        std::move(global_routing),
        std::move(bottom_up_dp),
        0.0,
        0,
        use_dp,
    };
    evaluation.routing_cost = evaluation.global_routing.total_metrics.cost;
    evaluation.failed_nets = evaluation.global_routing.failed_nets;
    return evaluation;
}

// 返回当前 request 中带 LCP 拓扑的 net 集合。
std::unordered_set<std::string> nets_with_lcp_topology(const RoutingEvaluationRequest& request) {
    std::unordered_set<std::string> result;
    for (const auto& point : request.linking_points) {
        for (const auto& segment : point.segments) result.insert(segment.net);
    }
    return result;
}

// 把 LCP 的连续候选位置转换为指定层上的网格点。
// 收集 LCP 物理候选位置，让 routing grid 覆盖论文 DP 要搜索的 resource point。
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

// 查找普通 pin terminal 的全局网格点。
std::optional<routing::GridPoint> pin_grid_point(
    const routing::RoutingContext& context,
    const std::string& terminal) {
    const auto found = context.global_pins().find(terminal);
    if (found == context.global_pins().end()) return std::nullopt;
    return context.grid().snap_to_grid(found->second.location, found->second.layer);
}

// 选择 LCP segment 的搜索层，优先沿用另一端 pin 所在层。
int layer_for_lcp_segment(
    const routing::RoutingContext& context,
    const WireSegmentRef& segment) {
    const auto from_pin = context.global_pins().find(segment.from);
    if (from_pin != context.global_pins().end()) return from_pin->second.layer;
    const auto to_pin = context.global_pins().find(segment.to);
    if (to_pin != context.global_pins().end()) return to_pin->second.layer;
    return 0;
}

// 将 LCP segment endpoint 解析为 A* 可用的网格点。
// 支持 pin-LCP 与 LCP-LCP segment，将任意 endpoint 解析到当前候选绑定下的网格点。
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

// 返回候选中 endpoint 对应的 LCP 候选位置 id；pin endpoint 返回空。
std::string lcp_candidate_for_endpoint(
    const std::unordered_map<std::string, PhysicalLocationCandidate>& location_by_lcp,
    const std::string& endpoint) {
    const auto found = location_by_lcp.find(endpoint);
    return found == location_by_lcp.end() ? std::string{} : found->second.id;
}

// 在运行 A* 前估计 LCP-LCP 候选组合代价，用于 top-K 截断。
double endpoint_pair_distance(
    const PhysicalLocationCandidate& left,
    const PhysicalLocationCandidate& right) {
    return std::abs(left.x - right.x) + std::abs(left.y - right.y) + left.penalty + right.penalty;
}
// 查找 net 的 FLOW 约束。
std::optional<FlowConstraint> flow_for_net(const Circuit& circuit, const std::string& net) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net == net) return flow;
    }
    return std::nullopt;
}

// 判断候选拓扑方向是否满足 FLOW 逻辑方向。
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

// 补充候选路径的论文 penalty 分项。
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

// 为 LCP 拓扑中的每条逻辑 segment 生成 A* 候选路径。
std::vector<routing::RouteCandidate> generate_lcp_route_candidates(
    const routing::RoutingContext& context,
    const RoutingEvaluationRequest& request,
    const Circuit& circuit) {
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

    constexpr std::size_t kMaxPairwiseCandidatesPerSegment = 12;
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

        struct LocationBinding {
            std::unordered_map<std::string, PhysicalLocationCandidate> by_lcp;
            double estimate{};
        };
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
            if (bindings.size() > kMaxPairwiseCandidatesPerSegment) bindings.resize(kMaxPairwiseCandidatesPerSegment);
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

// 判断两个网格步进是否在同一金属层上共线。
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

// 向输出追加一条非零长度中心线线段。
// 判断两个连续坐标是否可视为同一点，避免浮点误差影响线段合并和检查。
bool same_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

// 返回 topology segment 的稳定匹配键。
std::string segment_key(const std::string& net, const std::string& from, const std::string& to) {
    return net + "|" + from + "|" + to;
}

// 返回候选路径对应的 topology segment 匹配键。
std::string segment_key(const routing::RouteCandidate& candidate) {
    return segment_key(candidate.net, candidate.from_terminal, candidate.to_terminal);
}

// 构建 detailed routing 回溯所需的 LCP 和 segment 索引。
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

// 判断 terminal 是否是 LCP id。
bool is_lcp_terminal(const DetailedTopologyIndex& index, const std::string& terminal) {
    return index.lcp_by_id.contains(terminal) || index.lcp_space_by_id.contains(terminal);
}

// 返回候选路径连接到的 LCP id。
std::string lcp_id_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    if (!candidate.lcp_id.empty()) return candidate.lcp_id;
    if (is_lcp_terminal(index, candidate.from_terminal)) return candidate.from_terminal;
    if (is_lcp_terminal(index, candidate.to_terminal)) return candidate.to_terminal;
    return {};
}

// 返回候选路径所属的 space node id。
std::string space_node_for_candidate(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    const std::string lcp_id = lcp_id_for_candidate(index, candidate);
    if (lcp_id.empty()) return {};
    const auto found = index.lcp_space_by_id.find(lcp_id);
    return found == index.lcp_space_by_id.end() ? std::string{} : found->second;
}

// 返回或创建指定 net 的 detailed route trace。
// 收集候选路径关联到的全部 LCP id，LCP-LCP segment 需要同时反馈两端 space。
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

// 收集候选路径关联到的全部 space node id，并去重。
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

// 记录 detailed traceback 中出现的 terminal 或 LCP 节点。
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

// 判断候选是否能和 request 中的 LCP topology 对上。
bool candidate_matches_lcp_topology(const DetailedTopologyIndex& index, const routing::RouteCandidate& candidate) {
    if (index.topology_segment_keys.empty()) return true;
    if (lcp_ids_for_candidate(index, candidate).empty()) return true;
    if (index.topology_segment_keys.contains(segment_key(candidate))) return true;
    return index.topology_segment_keys.contains(segment_key(candidate.net, candidate.to_terminal, candidate.from_terminal));
}

// 查找候选路径对应的原始 WireSegmentRef，用于详细布线约束归因。
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

// 判断 route segment 是否为水平中心线。
bool route_is_horizontal(const RouteSegment& route) {
    return same_coord(route.y1, route.y2);
}

// 判断 route segment 是否为垂直中心线。
bool route_is_vertical(const RouteSegment& route) {
    return same_coord(route.x1, route.x2);
}

// 返回 detailed routing 阶段实际写入的线宽，优先满足 WIRE_WIDTH 约束范围。
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

// 返回旧调试接口使用的线宽，不依赖 Circuit 以保持公开函数签名稳定。
// 判断候选路径的原始线宽是否满足 segment 与 net 级 WIRE_WIDTH/current-density 代理约束。
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

// 判断新线段是否能和上一条线段合并为同一条中心线。
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

// 将一条 A* 网格路径压缩为同层共线的 routing segment。
void append_path_segments(
    std::vector<RouteSegment>& routes,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const double width = selected_width_for_candidate(evaluation, candidate);
    auto converted =
        routing::candidate_to_route_segments(evaluation.context.grid(), candidate, width, evaluation.context.active_regions());
    routes.insert(routes.end(), converted.begin(), converted.end());
}

// 将一条 A* 网格路径按 detailed routing 线宽规则压缩成 route segment。
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

// 表示 detailed routing 合法化后实际采用的候选和金属线段。
struct DetailedLegalization {
    bool success{};
    routing::RouteCandidate candidate;
    std::vector<RouteSegment> routes;
    bool used_alternative{};
    bool reassigned_layer{};
};

// 判断两个候选是否连接同一逻辑 terminal pair。
bool same_logical_candidate_pair(
    const routing::RouteCandidate& lhs,
    const routing::RouteCandidate& rhs) {
    if (lhs.net != rhs.net) return false;
    if (!lhs.segment_id.empty() && !rhs.segment_id.empty() && lhs.segment_id == rhs.segment_id) return true;
    const bool same_direction = lhs.from_terminal == rhs.from_terminal && lhs.to_terminal == rhs.to_terminal;
    const bool reverse_direction = lhs.from_terminal == rhs.to_terminal && lhs.to_terminal == rhs.from_terminal;
    return same_direction || reverse_direction;
}

// 如果候选线段不与既有异网金属短路，则返回可写入的线段。
std::optional<std::vector<RouteSegment>> legal_routes_without_short(
    std::vector<RouteSegment> routes,
    const std::vector<RouteSegment>& occupied_routes) {
    if (routes.empty()) return std::nullopt;
    if (routing::routes_short_with_existing(routes, occupied_routes)) return std::nullopt;
    return routes;
}

// 尝试把整条候选路径换到相邻层，作为最小版 local layer assignment。
std::optional<std::vector<RouteSegment>> try_adjacent_layer_assignment(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& occupied_routes) {
    if (routes.empty()) return std::nullopt;
    int base_layer = 0;
    try {
        base_layer = routing::layer_to_index(routes.front().layer);
    } catch (...) {
        return std::nullopt;
    }
    for (const int delta : {1, -1, 2, -2}) {
        const int next_layer = base_layer + delta;
        if (!routing::is_valid_layer_index(next_layer)) continue;
        auto reassigned = routing::reassign_routes_to_layer(routes, next_layer);
        if (!routing::routes_short_with_existing(reassigned, occupied_routes)) return reassigned;
    }
    return std::nullopt;
}

// 按论文 detailed routing 语义对候选做局部合法化：替代候选优先，最后尝试换层。
DetailedLegalization legalize_detailed_candidate(
    const Circuit& circuit,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& selected,
    const std::vector<RouteSegment>& occupied_routes) {
    std::vector<const routing::RouteCandidate*> attempts{&selected};
    for (const auto& candidate : evaluation.candidates) {
        if (!candidate.path.success || !same_logical_candidate_pair(candidate, selected)) continue;
        bool duplicate = false;
        for (const auto* existing : attempts) {
            if (existing == &candidate ||
                (existing->segment_id == candidate.segment_id &&
                 existing->lcp_candidate_id == candidate.lcp_candidate_id)) {
                duplicate = true;
            }
        }
        if (!duplicate) attempts.push_back(&candidate);
    }

    for (const auto* candidate : attempts) {
        auto legal = legal_routes_without_short(
            detailed_path_segments(circuit, evaluation, *candidate),
            occupied_routes);
        if (legal.has_value()) {
            return DetailedLegalization{true, *candidate, std::move(*legal), candidate != &selected, false};
        }
    }
    for (const auto* candidate : attempts) {
        auto legal = try_adjacent_layer_assignment(
            detailed_path_segments(circuit, evaluation, *candidate),
            occupied_routes);
        if (legal.has_value()) {
            return DetailedLegalization{true, *candidate, std::move(*legal), candidate != &selected, true};
        }
    }
    return DetailedLegalization{false, selected, {}, false, false};
}

// 将候选路径写入 detailed route，同时记录 route segment 到 LCP/space-node 的回溯映射。
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

// 根据 detailed route 的实际线宽更新所属 space node 的预留空间需求。
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

// 记录 detailed traceback 失败，并把失败写入 report 和 penalty。
void add_traceback_failure(DetailedRoutingResult& result, DetailedRouteTrace& trace, const std::string& message) {
    ++result.traceback_failures;
    result.routing_failure_penalty += kDetailedFailurePenalty;
    trace.warnings.push_back(message);
    result.report.warnings.push_back(message);
}

// 将 route segment 转为金属占用矩形，用于 DRC 和 coupling 检查。
Rect route_to_rect(const RouteSegment& route) {
    return routing::segment_to_rect(
        routing::Segment{routing::Point{route.x1, route.y1}, routing::Point{route.x2, route.y2}},
        route.width);
}

// 判断两个连续坐标是否足够接近，用于匹配 pin access 起点。
bool near_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-6;
}

// 判断 route 端点是否贴近某个真实 pin，避免把任意 active 内端点都当作 pin access。
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

// 判断点是否位于 active 内部，不把边界接触当作内部穿越。
bool point_strictly_inside(const Rect& active, const routing::Point& point) {
    const Rect rect = routing::normalize_rect(active);
    constexpr double kBoundaryTolerance = 0.1;
    return point.x > rect.x1 + kBoundaryTolerance && point.x < rect.x2 - kBoundaryTolerance &&
           point.y > rect.y1 + kBoundaryTolerance && point.y < rect.y2 - kBoundaryTolerance;
}

// 判断点是否位于 active 边界上。
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

// 返回从 active 内端点沿当前线段逃逸到边界的距离；非正交逃逸返回无效值。
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

// 返回 pin 沿当前 access 方向到 active 边界的距离。
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

// 只允许真实 pin 附近的一小段 active 逃逸走线，禁止用 pin 端点豁免长距离横穿 active。
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

// 收集 detailed route 穿越 active region 的基础 DRC 违反线段索引。
std::vector<std::size_t> collect_active_region_crossings(
    const RoutingEvaluationRequest& request,
    const std::vector<RouteSegment>& routes) {
    std::vector<std::size_t> violations;
    for (std::size_t index = 0; index < routes.size(); ++index) {
        const auto& route = routes[index];
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

// 判断两条同层异网线段是否存在近距离平行耦合风险。
// 收集同层异网金属重叠，重叠代表真实短路，不能只作为 coupling 风险处理。
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

// 收集 detailed route 的同层平行耦合线段对。
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

// 计算两条同层异网线段的平行重叠长度；距离超过 spacing 时返回 0。
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

// 收集 detailed route 的同层平行耦合线段对，并记录实际重叠长度。
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

// 返回 net priority 的 detailed routing 回溯顺序权重。
int detailed_priority_rank(Priority priority) {
    if (priority == Priority::Symmetry) return 0;
    if (priority == Priority::Critical) return 1;
    return 2;
}

// 按论文 detailed routing 优先级排序 net route。
std::vector<const routing::NetRouteChoice*> ordered_detailed_routes(
    const Circuit& circuit,
    const RoutingEvaluation& evaluation) {
    std::vector<const routing::NetRouteChoice*> routes;
    for (const auto& route : evaluation.global_routing.net_routes) routes.push_back(&route);
    std::stable_sort(routes.begin(), routes.end(), [&](const auto* left, const auto* right) {
        const auto left_it = circuit.nets.find(left->net);
        const auto right_it = circuit.nets.find(right->net);
        const Priority left_priority = left_it == circuit.nets.end() ? Priority::Normal : left_it->second.priority;
        const Priority right_priority = right_it == circuit.nets.end() ? Priority::Normal : right_it->second.priority;
        return detailed_priority_rank(left_priority) < detailed_priority_rank(right_priority);
    });
    return routes;
}

// 返回 detailed routing 应使用的候选路径，优先使用 bottom-up DP traceback。
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

// 按 NetTopology 中的 wire segment 顺序恢复 detailed routing 候选，体现 top-down traceback 顺序。
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

// 根据当前 placement 执行布线上下文构建、候选路径生成和全局路径选择。
// 判断一个 net 的 detailed traceback 是否能从 FLOW out pin 追踪到 in pin。
// 先保留同一 net 内的 topology segment 顺序，再按论文要求让 symmetry/critical net 优先 detailed routing。
std::vector<routing::RouteCandidate> order_candidates_for_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const std::vector<routing::RouteCandidate>& candidates) {
    auto ordered = order_candidates_by_topology(request, candidates);
    std::stable_sort(ordered.begin(), ordered.end(), [&](const auto& left, const auto& right) {
        const auto left_it = circuit.nets.find(left.net);
        const auto right_it = circuit.nets.find(right.net);
        const Priority left_priority = left_it == circuit.nets.end() ? Priority::Normal : left_it->second.priority;
        const Priority right_priority = right_it == circuit.nets.end() ? Priority::Normal : right_it->second.priority;
        return detailed_priority_rank(left_priority) < detailed_priority_rank(right_priority);
    });
    return ordered;
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

// 基于 DP traceback 后的有向拓扑检查 FLOW 约束，并把失败来源写入 detailed report。
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

// 根据 placement candidate 中的 LCP 拓扑执行 A*/DP 布线评估。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request) {
    routing::RoutingContext context(circuit, request.placements, routing::GridConfig{}, lcp_routing_points(request));
    auto direct_candidates = routing::generate_route_candidates(circuit, context);
    for (auto& candidate : direct_candidates) {
        annotate_candidate(circuit, candidate, 50000.0, 50000.0);
    }
    auto candidates = direct_candidates;
    const auto lcp_nets = nets_with_lcp_topology(request);
    if (!lcp_nets.empty()) {
        candidates.erase(
            std::remove_if(
                candidates.begin(),
                candidates.end(),
                [&](const auto& candidate) { return lcp_nets.contains(candidate.net); }),
            candidates.end());
        auto lcp_candidates = generate_lcp_route_candidates(context, request, circuit);
        candidates.insert(
            candidates.end(),
            std::make_move_iterator(lcp_candidates.begin()),
            std::make_move_iterator(lcp_candidates.end()));
    }
    if (request.net_topologies.empty() || !request.tree.root.has_value()) {
        return make_evaluation(std::move(context), std::move(candidates), circuit);
    }
    auto bottom_up_dp = routing::run_bottom_up_routing_dp(circuit, request, context, candidates);
    if (!bottom_up_dp.success) {
        return make_evaluation(std::move(context), std::move(direct_candidates), circuit, std::move(bottom_up_dp));
    }
    return make_evaluation(std::move(context), std::move(candidates), circuit, std::move(bottom_up_dp));
}

// 将 DP 全局布线选中的 A* 网格路径转换为当前 routing.txt 使用的中心线线段。
std::vector<RouteSegment> selected_candidates_to_segments(
    const RoutingEvaluation& evaluation) {
    std::vector<RouteSegment> routes;
    for (const auto& candidate : selected_candidates_for_detailed_routing(evaluation)) {
        append_path_segments(routes, evaluation, candidate);
    }
    return routes;
}

// 执行论文 top-down detailed routing 阶段，当前基于 DP 选中子问题回溯并清理路径。
DetailedRoutingResult run_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation) {
    DetailedRoutingResult result;
    const auto topology_index = build_detailed_topology_index(request);
    std::unordered_set<std::string> routed_space_nodes;
    const auto selected_candidates =
        order_candidates_for_detailed_routing(circuit, request, selected_candidates_for_detailed_routing(evaluation));
    const bool has_dp_traceback = evaluation.bottom_up_dp.has_value() && evaluation.bottom_up_dp->success;
    const int dp_state_id = has_dp_traceback ? evaluation.bottom_up_dp->best_state.id : -1;
    const std::string tree_node = has_dp_traceback ? evaluation.bottom_up_dp->best_state.tree_node : std::string{};
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
    routing::PathMetrics detailed_metrics;
    std::vector<routing::RouteCandidate> legalized_candidates;
    for (const auto& candidate : selected_candidates) {
        auto& trace = trace_for_net(result.report, candidate.net);
        auto legal = legalize_detailed_candidate(circuit, evaluation, candidate, result.routes);
        if (!legal.success) {
            add_traceback_failure(
                result,
                trace,
                "detailed routing could not legalize same-layer short for " + candidate.net + " " +
                    candidate.from_terminal + " -> " + candidate.to_terminal);
            continue;
        }
        const auto& actual_candidate = legal.candidate;
        if (legal.used_alternative) {
            trace.warnings.push_back(actual_candidate.net + ": detailed routing used alternative candidate");
            result.report.warnings.push_back(actual_candidate.net + ": detailed routing used alternative candidate");
        }
        if (legal.reassigned_layer) {
            trace.warnings.push_back(actual_candidate.net + ": detailed routing reassigned candidate layer");
            result.report.warnings.push_back(actual_candidate.net + ": detailed routing reassigned candidate layer");
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
        legalized_candidates.push_back(actual_candidate);
        detailed_metrics.wirelength += actual_candidate.path.metrics.wirelength;
        detailed_metrics.bend_count += actual_candidate.path.metrics.bend_count;
        detailed_metrics.via_count += actual_candidate.path.metrics.via_count;
        const auto space_node_ids = space_nodes_for_candidate(topology_index, actual_candidate);
        const auto source_segment = wire_segment_for_candidate(topology_index, actual_candidate);
        for (const auto& space_node_id : space_node_ids) {
            routed_space_nodes.insert(space_node_id);
            update_space_requirement(result, space_node_id, detailed_width_for_candidate(circuit, evaluation, actual_candidate));
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
                source_segment.has_value() && !source_segment->id.empty() ? source_segment->id : actual_candidate.segment_id;
            const std::string message = actual_candidate.net + ": width violation at " + segment_id;
            trace.warnings.push_back(message);
            result.report.current_density_segments.push_back(message);
            result.report.warnings.push_back(message);
        }
    }
    apply_detailed_flow_check(circuit, legalized_candidates, result);
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
        result.routes.clear();
        routed_space_nodes.clear();
    }
    result.space_nodes_with_routes = static_cast<int>(routed_space_nodes.size());
    result.detailed_routing_penalty =
        result.flow_penalty + result.current_density_penalty + result.design_rule_penalty +
        result.coupling_penalty + result.routing_failure_penalty;
    result.detailed_cost =
        detailed_metrics.wirelength + 3.0 * static_cast<double>(detailed_metrics.bend_count) +
        0.2 * static_cast<double>(detailed_metrics.via_count) + result.detailed_routing_penalty;
    result.used_global_fallback = evaluation.bottom_up_dp.has_value() && !evaluation.bottom_up_dp->success;
    return result;
}

}  // namespace sapr
