// 文件职责：实现候选路径压缩、金属矩形短路检查和简单换层合法化工具。
#include "sapr/routing/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "sapr/routing/geometry.hpp"
#include "sapr/routing/layer.hpp"

namespace sapr::routing {
namespace {

// 表示 route segment 可参与输出合并的几何方向。
enum class RouteOrientation { Horizontal, Vertical, Other };

// 表示同一条中心线上的一维覆盖区间。
struct MergeInterval {
    double begin{};
    double end{};
};

// 收集同网同层同宽且共线的 route segment，用于批量区间合并。
struct MergeGroup {
    std::size_t first_index{};
    RouteSegment sample;
    RouteOrientation orientation{RouteOrientation::Other};
    double fixed{};
    std::vector<MergeInterval> intervals;
};

// 判断两个连续坐标是否可视为相同，避免浮点误差影响线段合并。
bool same_coord(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-9;
}

// 判断输出线段的方向，只有水平和垂直线段参与区间合并。
RouteOrientation route_orientation(const RouteSegment& route) {
    if (same_coord(route.y1, route.y2) && !same_coord(route.x1, route.x2)) return RouteOrientation::Horizontal;
    if (same_coord(route.x1, route.x2) && !same_coord(route.y1, route.y2)) return RouteOrientation::Vertical;
    return RouteOrientation::Other;
}

// 返回共线分组使用的固定坐标，水平线取 y，垂直线取 x。
double fixed_coord_for_route(const RouteSegment& route, RouteOrientation orientation) {
    return orientation == RouteOrientation::Horizontal ? route.y1 : route.x1;
}

// 返回可合并方向上的区间端点，统一为 min/max 方便排序合并。
MergeInterval interval_for_route(const RouteSegment& route, RouteOrientation orientation) {
    if (orientation == RouteOrientation::Horizontal) {
        return {std::min(route.x1, route.x2), std::max(route.x1, route.x2)};
    }
    return {std::min(route.y1, route.y2), std::max(route.y1, route.y2)};
}

// 判断线段是否属于同一个同网同层同宽共线合并组。
bool route_matches_group(const RouteSegment& route, RouteOrientation orientation, double fixed, const MergeGroup& group) {
    return group.orientation == orientation &&
           group.sample.net == route.net &&
           group.sample.layer == route.layer &&
           same_coord(group.sample.width, route.width) &&
           same_coord(group.fixed, fixed);
}

// 将合并后的区间转换回标准方向的 route segment。
RouteSegment route_from_interval(const MergeGroup& group, const MergeInterval& interval) {
    RouteSegment route = group.sample;
    if (group.orientation == RouteOrientation::Horizontal) {
        route.y1 = group.fixed;
        route.y2 = group.fixed;
        if (group.sample.x1 <= group.sample.x2) {
            route.x1 = interval.begin;
            route.x2 = interval.end;
        } else {
            route.x1 = interval.end;
            route.x2 = interval.begin;
        }
    } else {
        route.x1 = group.fixed;
        route.x2 = group.fixed;
        if (group.sample.y1 <= group.sample.y2) {
            route.y1 = interval.begin;
            route.y2 = interval.end;
        } else {
            route.y1 = interval.end;
            route.y2 = interval.begin;
        }
    }
    return route;
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

// 按 detailed DRC 使用的金属矩形语义检查异网最小边缘间距。
bool routes_violate_spacing_with_existing(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& existing,
    double spacing) {
    if (spacing <= 0.0) return false;
    for (const auto& route : routes) {
        for (const auto& occupied : existing) {
            if (route.net == occupied.net || route.layer != occupied.layer) continue;
            if (intersects(expand_rect(route_to_rect(route), spacing), route_to_rect(occupied))) return true;
        }
    }
    return false;
}

// 合并同网同层同宽的共线输出段，去掉重复、包含、重叠和首尾相接区间。
std::vector<RouteSegment> merge_collinear_same_net_routes(
    const std::vector<RouteSegment>& routes) {
    std::vector<MergeGroup> groups;
    std::vector<RouteSegment> passthrough;
    passthrough.reserve(routes.size());

    for (std::size_t index = 0; index < routes.size(); ++index) {
        const auto& route = routes[index];
        const auto orientation = route_orientation(route);
        if (orientation == RouteOrientation::Other) {
            passthrough.push_back(route);
            continue;
        }
        const double fixed = fixed_coord_for_route(route, orientation);
        const auto interval = interval_for_route(route, orientation);
        if (same_coord(interval.begin, interval.end)) continue;

        auto group = std::find_if(groups.begin(), groups.end(), [&](const MergeGroup& candidate) {
            return route_matches_group(route, orientation, fixed, candidate);
        });
        if (group == groups.end()) {
            MergeGroup next;
            next.first_index = index;
            next.sample = route;
            next.orientation = orientation;
            next.fixed = fixed;
            next.intervals.push_back(interval);
            groups.push_back(std::move(next));
        } else {
            group->intervals.push_back(interval);
        }
    }

    std::sort(groups.begin(), groups.end(), [](const MergeGroup& lhs, const MergeGroup& rhs) {
        return lhs.first_index < rhs.first_index;
    });

    std::vector<RouteSegment> merged;
    merged.reserve(routes.size());
    for (auto& group : groups) {
        std::sort(group.intervals.begin(), group.intervals.end(), [](const auto& lhs, const auto& rhs) {
            if (!same_coord(lhs.begin, rhs.begin)) return lhs.begin < rhs.begin;
            return lhs.end < rhs.end;
        });

        std::vector<MergeInterval> normalized;
        for (const auto& interval : group.intervals) {
            if (normalized.empty() || interval.begin > normalized.back().end + 1e-9) {
                normalized.push_back(interval);
            } else {
                normalized.back().end = std::max(normalized.back().end, interval.end);
            }
        }
        for (const auto& interval : normalized) {
            merged.push_back(route_from_interval(group, interval));
        }
    }
    merged.insert(merged.end(), passthrough.begin(), passthrough.end());
    return merged;
}

std::vector<RouteSegment> reassign_routes_to_layer(std::vector<RouteSegment> routes, int layer_index) {
    const std::string layer = index_to_layer(layer_index);
    for (auto& route : routes) route.layer = layer;
    return routes;
}

}  // namespace sapr::routing
