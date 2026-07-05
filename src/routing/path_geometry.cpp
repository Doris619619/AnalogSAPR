// 文件职责：实现候选路径压缩、金属矩形短路检查和简单换层合法化工具。
#include "sapr/routing/path_geometry.hpp"

#include <cmath>

#include "sapr/routing/geometry.hpp"
#include "sapr/routing/layer.hpp"

namespace sapr::routing {
namespace {

// 判断两个连续坐标是否可视为相同，避免浮点误差影响线段合并。
bool same_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

// 移除 A-B-A 型即时回折，减少后续短路检查中的重复线段。
std::vector<GridPoint> prune_backtracks(std::vector<GridPoint> points) {
    bool changed = true;
    while (changed) {
        changed = false;
        std::vector<GridPoint> pruned;
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

// 判断三个网格点是否在同一金属层沿同一方向连续前进。
bool same_line_direction(const GridPoint& first, const GridPoint& second, const GridPoint& third) {
    if (first.layer != second.layer || second.layer != third.layer) return false;
    const int dx1 = second.ix - first.ix;
    const int dy1 = second.iy - first.iy;
    const int dx2 = third.ix - second.ix;
    const int dy2 = third.iy - second.iy;
    return dx1 == dx2 && dy1 == dy2;
}

// 判断网格点是否落在任一需要切分的矩形内。
bool point_inside_any_rect(const Grid& grid, const GridPoint& point, const std::vector<Rect>& rects) {
    const Point xy = grid.grid_to_point(point);
    for (const auto& rect : rects) {
        if (contains_point(rect, xy)) return true;
    }
    return false;
}

// 判断继续延长线段是否会跨过 active 内外边界，跨界后应立即切段。
bool can_extend_without_crossing_split(
    const Grid& grid,
    const GridPoint& start,
    const GridPoint& current,
    const GridPoint& next,
    const std::vector<Rect>& split_rects) {
    if (split_rects.empty()) return true;
    const bool start_inside = point_inside_any_rect(grid, start, split_rects);
    const bool current_inside = point_inside_any_rect(grid, current, split_rects);
    if (start_inside != current_inside) return false;
    (void)next;
    return true;
}

// 判断新线段是否能与上一条同网同层线段合并。
bool can_merge_with_last(const RouteSegment& last, const RouteSegment& edge) {
    if (last.net != edge.net || last.layer != edge.layer || !same_coord(last.width, edge.width)) return false;
    if (!same_coord(last.x2, edge.x1) || !same_coord(last.y2, edge.y1)) return false;
    const bool horizontal = same_coord(last.y1, last.y2) && same_coord(edge.y1, edge.y2) &&
                            same_coord(last.y1, edge.y1);
    const bool vertical = same_coord(last.x1, last.x2) && same_coord(edge.x1, edge.x2) &&
                          same_coord(last.x1, edge.x1);
    return horizontal || vertical;
}

// 将一个网格步进追加为非零长度金属线段，并合并同向连续线段。
void append_segment(
    std::vector<RouteSegment>& routes,
    const Grid& grid,
    const std::string& net,
    int layer,
    const GridPoint& start,
    const GridPoint& end,
    double width,
    bool allow_merge = true) {
    const auto start_xy = grid.grid_to_point(start);
    const auto end_xy = grid.grid_to_point(end);
    if (same_coord(start_xy.x, end_xy.x) && same_coord(start_xy.y, end_xy.y)) return;
    RouteSegment edge{net, index_to_layer(layer), start_xy.x, start_xy.y, end_xy.x, end_xy.y, width};
    if (allow_merge && !routes.empty() && can_merge_with_last(routes.back(), edge)) {
        routes.back().x2 = edge.x2;
        routes.back().y2 = edge.y2;
        return;
    }
    for (const auto& route : routes) {
        if (route.net != net || route.layer != edge.layer) continue;
        const bool same_direction = same_coord(route.x1, start_xy.x) && same_coord(route.y1, start_xy.y) &&
                                    same_coord(route.x2, end_xy.x) && same_coord(route.y2, end_xy.y);
        const bool reverse_direction = same_coord(route.x1, end_xy.x) && same_coord(route.y1, end_xy.y) &&
                                       same_coord(route.x2, start_xy.x) && same_coord(route.y2, start_xy.y);
        if (same_direction || reverse_direction) return;
    }
    routes.push_back(std::move(edge));
}

// 将 route segment 转为金属占用矩形。
Rect route_to_rect(const RouteSegment& route) {
    return segment_to_rect(Segment{Point{route.x1, route.y1}, Point{route.x2, route.y2}}, route.width);
}

}  // namespace

std::vector<RouteSegment> candidate_to_route_segments(
    const Grid& grid,
    const RouteCandidate& candidate,
    double width,
    const std::vector<Rect>& split_rects) {
    std::vector<RouteSegment> routes;
    const auto points = prune_backtracks(candidate.path.points);
    if (!candidate.path.success || points.size() < 2) return routes;

    std::size_t start = 0;
    std::size_t cursor = 1;
    while (cursor < points.size()) {
        if (points[start].layer != points[cursor].layer) {
            start = cursor;
            ++cursor;
            continue;
        }
        while (cursor + 1 < points.size() &&
               same_line_direction(points[cursor - 1], points[cursor], points[cursor + 1]) &&
               can_extend_without_crossing_split(grid, points[start], points[cursor], points[cursor + 1], split_rects)) {
            ++cursor;
        }
        append_segment(
            routes,
            grid,
            candidate.net,
            points[start].layer,
            points[start],
            points[cursor],
            width,
            split_rects.empty());
        start = cursor;
        ++cursor;
    }
    return routes;
}

std::vector<RouteSegment> candidate_to_route_segments(
    const Grid& grid,
    const RouteCandidate& candidate,
    double width) {
    return candidate_to_route_segments(grid, candidate, width, {});
}

bool same_layer_short(const RouteSegment& lhs, const RouteSegment& rhs) {
    if (lhs.net == rhs.net || lhs.layer != rhs.layer) return false;
    return intersects(route_to_rect(lhs), route_to_rect(rhs));
}

bool routes_short_with_existing(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& existing) {
    for (const auto& route : routes) {
        for (const auto& occupied : existing) {
            if (same_layer_short(route, occupied)) return true;
        }
    }
    return false;
}

std::vector<RouteSegment> reassign_routes_to_layer(std::vector<RouteSegment> routes, int layer_index) {
    const std::string layer = index_to_layer(layer_index);
    for (auto& route : routes) route.layer = layer;
    return routes;
}

}  // namespace sapr::routing
