// 文件职责：实现 placement 到 A*/DP 布线评估结果的封装流程。
#include "sapr/routing_evaluator.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

#include "sapr/routing/astar.hpp"
#include "sapr/routing/layer.hpp"
#include "sapr/routing/topology.hpp"

namespace sapr {
namespace {

// 执行候选路径生成与 DP 全局布线选择的公共封装。
RoutingEvaluation make_evaluation(
    routing::RoutingContext context,
    std::vector<routing::RouteCandidate> candidates,
    const Circuit& circuit) {
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
bool flow_ok_for_candidate(const Circuit& circuit, const routing::RouteCandidate& candidate) {
    const auto flow = flow_for_net(circuit, candidate.net);
    if (!flow.has_value()) return true;
    if (candidate.from_terminal == flow->in_pin && candidate.to_terminal == flow->out_pin) return false;
    if (candidate.from_terminal == flow->out_pin || candidate.to_terminal == flow->in_pin) return true;
    return true;
}

// 补充候选路径的论文 penalty 分项。
void annotate_candidate(const Circuit& circuit, routing::RouteCandidate& candidate, double current_density_penalty, double flow_penalty) {
    candidate.flow_ok = flow_ok_for_candidate(circuit, candidate);
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
                candidate.lcp_candidate_id = location.id;
                candidate.wire_width = std::max(segment.min_width, 1.0);
                if (!start.has_value() || !goal.has_value()) {
                    candidate.path = routing::GridPath{false, "LCP endpoint cannot be resolved", {}, {}};
                    annotate_candidate(circuit, candidate, 50000.0, 50000.0);
                    candidates.push_back(std::move(candidate));
                    continue;
                }
                routing::AStarConfig config;
                config.wire_width = candidate.wire_width;
                candidate.path = routing::find_astar_path(context.grid(), context.obstacles(), *start, *goal, config);
                annotate_candidate(circuit, candidate, 50000.0, 50000.0);
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
    for (const auto& route : routes) {
        if (route.net != net || route.layer != routing::index_to_layer(layer)) continue;
        const bool same_direction = route.x1 == start_xy.x && route.y1 == start_xy.y && route.x2 == end_xy.x && route.y2 == end_xy.y;
        const bool reverse_direction = route.x1 == end_xy.x && route.y1 == end_xy.y && route.x2 == start_xy.x && route.y2 == start_xy.y;
        if (same_direction || reverse_direction) return;
    }
    routes.push_back({net, routing::index_to_layer(layer), start_xy.x, start_xy.y, end_xy.x, end_xy.y, width});
}

// 将一条 A* 网格路径压缩为同层共线的 routing segment。
void append_path_segments(
    std::vector<RouteSegment>& routes,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const auto points = prune_backtracks(candidate.path.points);
    if (points.size() < 2) return;

    const double width = candidate.wire_width > 0.0 ? candidate.wire_width : evaluation.context.default_width_for_net(candidate.net);
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

// 根据当前 placement 执行布线上下文构建、候选路径生成和全局路径选择。
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
        auto lcp_candidates = generate_lcp_route_candidates(context, request, circuit);
        candidates.insert(
            candidates.end(),
            std::make_move_iterator(lcp_candidates.begin()),
            std::make_move_iterator(lcp_candidates.end()));
    }
    return make_evaluation(std::move(context), std::move(candidates), circuit);
}

// 将 DP 全局布线选中的 A* 网格路径转换为当前 routing.txt 使用的中心线线段。
std::vector<RouteSegment> selected_candidates_to_segments(
    const RoutingEvaluation& evaluation) {
    std::vector<RouteSegment> routes;
    for (const auto& net_route : evaluation.global_routing.net_routes) {
        if (!net_route.success) continue;
        for (const auto& candidate : net_route.selected_candidates) {
            append_path_segments(routes, evaluation, candidate);
        }
    }
    return routes;
}

// 执行论文 top-down detailed routing 阶段，当前基于 DP 选中子问题回溯并清理路径。
DetailedRoutingResult run_detailed_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation) {
    (void)request;
    DetailedRoutingResult result;
    for (const auto* net_route : ordered_detailed_routes(circuit, evaluation)) {
        if (!net_route->success) continue;
        for (const auto& candidate : net_route->selected_candidates) {
            append_path_segments(result.routes, evaluation, candidate);
            result.coupling_penalty += candidate.coupling_cost;
            if (!candidate.flow_ok) ++result.flow_violations;
            if (!candidate.current_density_ok) ++result.current_density_violations;
        }
        result.coupling_penalty += net_route->coupling_penalty;
    }
    result.used_global_fallback = false;
    return result;
}

}  // namespace sapr
