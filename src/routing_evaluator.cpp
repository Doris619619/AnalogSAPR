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

// 执行候选路径生成与 DP 全局布线选择的公共封装。
RoutingEvaluation make_evaluation(
    routing::RoutingContext context,
    std::vector<routing::RouteCandidate> candidates,
    const Circuit& circuit,
    std::optional<routing::RoutingDpResult> bottom_up_dp = std::nullopt) {
    const bool use_dp = bottom_up_dp.has_value() && bottom_up_dp->success;
    auto global_routing = use_dp
                              ? routing::run_global_routing(circuit, context, bottom_up_dp->traceback_candidates)
                              : routing::run_global_routing(circuit, context, candidates);
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
std::optional<routing::GridPoint> endpoint_grid_point(
    const routing::RoutingContext& context,
    const LinkingControlPoint& point,
    const PhysicalLocationCandidate& location,
    const WireSegmentRef& segment,
    const std::string& endpoint) {
    if (endpoint == point.id) return lcp_grid_point(context, location, layer_for_lcp_segment(context, segment));
    return pin_grid_point(context, endpoint);
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
    for (const auto& point : request.linking_points) {
        const auto locations = point.location_candidates.empty()
                                   ? std::vector<PhysicalLocationCandidate>{{0.0, 0.0, point.id + ":fallback"}}
                                   : point.location_candidates;
        for (const auto& segment : point.segments) {
            for (const auto& location : locations) {
                const auto start = endpoint_grid_point(context, point, location, segment, segment.from);
                const auto goal = endpoint_grid_point(context, point, location, segment, segment.to);
                routing::RouteCandidate candidate;
                candidate.net = segment.net;
                candidate.from_terminal = segment.from;
                candidate.to_terminal = segment.to;
                candidate.segment_id = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
                candidate.lcp_id = point.id;
                candidate.lcp_candidate_id = location.id;
                candidate.wire_width = std::max(segment.min_width, 1.0);
                if (!start.has_value() || !goal.has_value()) {
                    candidate.path = routing::GridPath{false, "LCP endpoint cannot be resolved", {}, {}};
                    annotate_candidate(circuit, candidate, 50000.0, 50000.0, segment);
                    candidates.push_back(std::move(candidate));
                    continue;
                }
                routing::AStarConfig config;
                config.wire_width = candidate.wire_width;
                candidate.path = routing::find_astar_path(context.grid(), context.obstacles(), *start, *goal, config);
                annotate_candidate(circuit, candidate, 50000.0, 50000.0, segment);
                candidates.push_back(std::move(candidate));
            }
        }
    }
    return candidates;
}

// 移除 A-B-A 型即时回折，减少 detailed routing 输出中的重复线段。
std::vector<routing::GridPoint> prune_backtracks(std::vector<routing::GridPoint> points) {
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
bool same_line_direction(
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

void append_segment(
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
    const auto points = prune_backtracks(candidate.path.points);
    if (points.size() < 2) return;

    const double width = selected_width_for_candidate(evaluation, candidate);
    std::size_t start = 0;
    std::size_t cursor = 1;
    while (cursor < points.size()) {
        if (points[start].layer != points[cursor].layer) {
            start = cursor;
            ++cursor;
            continue;
        }
        while (cursor + 1 < points.size() && same_line_direction(points[cursor - 1], points[cursor], points[cursor + 1])) {
            ++cursor;
        }
        append_segment(routes, evaluation.context.grid(), candidate.net, points[start].layer, points[start], points[cursor], width);
        start = cursor;
        ++cursor;
    }
}

// 将一条 A* 网格路径按 detailed routing 线宽规则压缩成 route segment。
void append_detailed_path_segments(
    std::vector<RouteSegment>& routes,
    const Circuit& circuit,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const auto points = prune_backtracks(candidate.path.points);
    if (points.size() < 2) return;

    const double width = detailed_width_for_candidate(circuit, evaluation, candidate);
    std::size_t start = 0;
    std::size_t cursor = 1;
    while (cursor < points.size()) {
        if (points[start].layer != points[cursor].layer) {
            start = cursor;
            ++cursor;
            continue;
        }
        while (cursor + 1 < points.size() && same_line_direction(points[cursor - 1], points[cursor], points[cursor + 1])) {
            ++cursor;
        }
        append_segment(routes, evaluation.context.grid(), candidate.net, points[start].layer, points[start], points[cursor], width);
        start = cursor;
        ++cursor;
    }
}

// 将候选路径写入 detailed route，同时记录 route segment 到 LCP/space-node 的回溯映射。
void append_traced_detailed_path(
    DetailedRoutingResult& result,
    DetailedRouteTrace& trace,
    const DetailedTopologyIndex& index,
    const Circuit& circuit,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate,
    int dp_state_id,
    const std::string& tree_node) {
    append_trace_node(trace, index, evaluation.context, candidate.from_terminal);
    append_trace_node(trace, index, evaluation.context, candidate.to_terminal);
    const std::size_t first_route = result.routes.size();
    append_detailed_path_segments(result.routes, circuit, evaluation, candidate);
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

// 判断 route segment 端点是否位于指定 active region 内，允许 pin access corridor 从 active 内出入。
bool route_endpoint_inside(const RouteSegment& route, const Rect& active) {
    return routing::contains_point(active, routing::Point{route.x1, route.y1}) ||
           routing::contains_point(active, routing::Point{route.x2, route.y2});
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
            if (route_endpoint_inside(route, active)) continue;
            violations.push_back(index);
            break;
        }
    }
    return violations;
}

// 判断两条同层异网线段是否存在近距离平行耦合风险。
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
    if (evaluation.bottom_up_dp.has_value() && evaluation.bottom_up_dp->success) {
        return evaluation.bottom_up_dp->traceback_candidates;
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
    routing::RoutingContext context(circuit, request.placements);
    auto candidates = routing::generate_route_candidates(circuit, context);
    for (auto& candidate : candidates) {
        annotate_candidate(circuit, candidate, 50000.0, 50000.0);
    }
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
    auto bottom_up_dp = routing::run_bottom_up_routing_dp(circuit, request, context, candidates);
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
        order_candidates_by_topology(request, selected_candidates_for_detailed_routing(evaluation));
    const bool has_dp_traceback = evaluation.bottom_up_dp.has_value() && evaluation.bottom_up_dp->success;
    const int dp_state_id = has_dp_traceback ? evaluation.bottom_up_dp->best_state.id : -1;
    const std::string tree_node = has_dp_traceback ? evaluation.bottom_up_dp->best_state.tree_node : std::string{};
    routing::PathMetrics detailed_metrics;
    for (const auto& candidate : selected_candidates) {
        auto& trace = trace_for_net(result.report, candidate.net);
        append_traced_detailed_path(result, trace, topology_index, circuit, evaluation, candidate, dp_state_id, tree_node);
        detailed_metrics.wirelength += candidate.path.metrics.wirelength;
        detailed_metrics.bend_count += candidate.path.metrics.bend_count;
        detailed_metrics.via_count += candidate.path.metrics.via_count;
        const std::string lcp_id = lcp_id_for_candidate(topology_index, candidate);
        const std::string space_node_id = space_node_for_candidate(topology_index, candidate);
        const auto source_segment = wire_segment_for_candidate(topology_index, candidate);
        if (!space_node_id.empty()) {
            routed_space_nodes.insert(space_node_id);
            update_space_requirement(result, space_node_id, detailed_width_for_candidate(circuit, evaluation, candidate));
        }
        if (!lcp_id.empty() && topology_index.lcp_without_location.contains(lcp_id)) {
            add_traceback_failure(result, trace, "LCP " + lcp_id + " has no location candidate");
        }
        if (!candidate_matches_lcp_topology(topology_index, candidate)) {
            add_traceback_failure(
                result,
                trace,
                "selected candidate does not match LCP topology: " + candidate.net + " " +
                    candidate.from_terminal + " -> " + candidate.to_terminal);
        }
        if (!candidate.flow_ok) {
            trace.warnings.push_back(candidate.net + ": candidate-level FLOW warning");
        }
        if (!candidate.current_density_ok || !candidate_width_satisfies_constraints(circuit, source_segment, candidate)) {
            ++result.current_density_violations;
            result.current_density_penalty += kDetailedCurrentDensityPenalty;
            const std::string segment_id =
                source_segment.has_value() && !source_segment->id.empty() ? source_segment->id : candidate.segment_id;
            const std::string message = candidate.net + ": width violation at " + segment_id;
            trace.warnings.push_back(message);
            result.report.current_density_segments.push_back(message);
            result.report.warnings.push_back(message);
        }
    }
    apply_detailed_flow_check(circuit, selected_candidates, result);
    const auto drc_routes = collect_active_region_crossings(request, result.routes);
    result.design_rule_violations = static_cast<int>(drc_routes.size());
    for (const auto route_index : drc_routes) {
        const auto& route = result.routes[route_index];
        result.report.design_rule_segments.push_back(
            route.net + ":" + route.layer + ":" + std::to_string(route_index));
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
    result.space_nodes_with_routes = static_cast<int>(routed_space_nodes.size());
    result.detailed_routing_penalty =
        result.flow_penalty + result.current_density_penalty + result.design_rule_penalty +
        result.coupling_penalty + result.routing_failure_penalty;
    result.detailed_cost =
        detailed_metrics.wirelength + 3.0 * static_cast<double>(detailed_metrics.bend_count) +
        0.2 * static_cast<double>(detailed_metrics.via_count) + result.detailed_routing_penalty;
    result.used_global_fallback = false;
    return result;
}

}  // namespace sapr
