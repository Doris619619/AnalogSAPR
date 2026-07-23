// 文件职责：声明候选网格路径到金属占用线段的转换与短路检查工具。
#pragma once

#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/grid.hpp"
#include "sapr/routing/path.hpp"

namespace sapr::routing {

// 将 A* 候选路径压缩成同层共线的金属中心线段。
std::vector<RouteSegment> candidate_to_route_segments(
    const Grid& grid,
    const RouteCandidate& candidate,
    double width);

// 将 A* 路径压缩成线段，并在穿出/进入 active blocker 时切段，保留局部 pin access 边界。
std::vector<RouteSegment> candidate_to_route_segments(
    const Grid& grid,
    const RouteCandidate& candidate,
    double width,
    const std::vector<Rect>& split_rects);

// 判断两条已压缩金属线段是否形成同层异网短路。
bool same_layer_short(const RouteSegment& lhs, const RouteSegment& rhs);

// 判断一组新线段是否与既有线段形成同层异网短路。
bool routes_short_with_existing(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& existing);

// 判断一组新线段是否与既有异网线段违反同层最小边缘间距；间距为零时不构成违例。
bool routes_violate_spacing_with_existing(
    const std::vector<RouteSegment>& routes,
    const std::vector<RouteSegment>& existing,
    double spacing);

// 合并同网同层同宽的共线重叠或相接线段，用于输出前清理重复金属段。
std::vector<RouteSegment> merge_collinear_same_net_routes(
    const std::vector<RouteSegment>& routes);

// 将线段整体映射到指定金属层，用于 detailed routing 的最小换层合法化。
std::vector<RouteSegment> reassign_routes_to_layer(
    std::vector<RouteSegment> routes,
    int layer_index);

}  // namespace sapr::routing
