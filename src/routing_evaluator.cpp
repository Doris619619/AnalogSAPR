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

// 过滤掉已由 LCP 拓扑接管的 net 的直接 terminal-to-terminal 候选。
std::vector<routing::RouteCandidate> remove_direct_candidates_for_lcp_nets(
    std::vector<routing::RouteCandidate> candidates,
    const std::unordered_set<std::string>& lcp_nets) {
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(), [&](const auto& candidate) {
            return lcp_nets.contains(candidate.net);
        }),
        candidates.end());
    return candidates;
}

// 把 LCP 的连续候选位置转换为指定层上的网格点。
routing::GridPoint lcp_grid_point(
    const routing::RoutingContext& context,
    const LinkingControlPoint& point,
    int layer) {
    const auto location = point.location_candidates.empty() ? PhysicalLocationCandidate{} : point.location_candidates.front();
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
    const WireSegmentRef& segment,
    const std::string& endpoint) {
    if (endpoint == point.id) return lcp_grid_point(context, point, layer_for_lcp_segment(context, segment));
    return pin_grid_point(context, endpoint);
}

// 为 LCP 拓扑中的每条逻辑 segment 生成 A* 候选路径。
std::vector<routing::RouteCandidate> generate_lcp_route_candidates(
    const routing::RoutingContext& context,
    const RoutingEvaluationRequest& request) {
    std::vector<routing::RouteCandidate> candidates;
    for (const auto& point : request.linking_points) {
        for (const auto& segment : point.segments) {
            const auto start = endpoint_grid_point(context, point, segment, segment.from);
            const auto goal = endpoint_grid_point(context, point, segment, segment.to);
            if (!start.has_value() || !goal.has_value()) {
                candidates.push_back({segment.net, segment.from, segment.to, routing::GridPath{false, "LCP endpoint cannot be resolved", {}, {}}});
                continue;
            }
            auto path = routing::find_astar_path(context.grid(), context.obstacles(), *start, *goal);
            candidates.push_back({segment.net, segment.from, segment.to, std::move(path)});
        }
    }
    return candidates;
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
    routes.push_back({net, routing::index_to_layer(layer), start_xy.x, start_xy.y, end_xy.x, end_xy.y, width});
}

// 将一条 A* 网格路径压缩为同层共线的 routing segment。
void append_path_segments(
    std::vector<RouteSegment>& routes,
    const RoutingEvaluation& evaluation,
    const routing::RouteCandidate& candidate) {
    const auto& points = candidate.path.points;
    if (points.size() < 2) return;

    const double width = evaluation.context.default_width_for_net(candidate.net);
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

// 根据当前 placement 执行布线上下文构建、候选路径生成和全局路径选择。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements) {
    routing::RoutingContext context(circuit, placements);
    auto candidates = routing::generate_route_candidates(circuit, context);
    return make_evaluation(std::move(context), std::move(candidates), circuit);
}

// 根据 placement candidate 中的 LCP 拓扑执行 A*/DP 布线评估。
RoutingEvaluation evaluate_routing(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request) {
    routing::RoutingContext context(circuit, request.placements);
    auto candidates = routing::generate_route_candidates(circuit, context);
    const auto lcp_nets = nets_with_lcp_topology(request);
    if (!lcp_nets.empty()) {
        candidates = remove_direct_candidates_for_lcp_nets(std::move(candidates), lcp_nets);
        auto lcp_candidates = generate_lcp_route_candidates(context, request);
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

}  // namespace sapr
