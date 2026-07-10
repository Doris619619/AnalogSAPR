// 实现 baseline Manhattan routing 和结果指标统计。
#include "sapr/router.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "sapr/geometry.hpp"

namespace sapr {
namespace {

// 返回优先级用于稳定排序的整数等级。
int priority_order(Priority priority) {
    if (priority == Priority::Critical) return 0;
    if (priority == Priority::Symmetry) return 1;
    return 2;
}

// 仅追加非零长度线段。
void append_nonzero(std::vector<RouteSegment>& routes, RouteSegment segment) {
    if (segment.x1 != segment.x2 || segment.y1 != segment.y2) routes.push_back(std::move(segment));
}

// 按当前输出语义统计相邻同层线段的 bend。
int count_bends(const std::vector<RouteSegment>& routes) {
    std::unordered_map<std::string, std::vector<const RouteSegment*>> by_net;
    for (const auto& route : routes) by_net[route.net].push_back(&route);
    int bends = 0;
    for (const auto& [net, segments] : by_net) {
        (void)net;
        for (std::size_t i = 1; i < segments.size(); ++i) {
            const auto& first = *segments[i - 1];
            const auto& second = *segments[i];
            if (first.x2 == second.x1 && first.y2 == second.y1 && first.layer == second.layer) ++bends;
        }
    }
    return bends;
}

// 按当前输出语义统计相邻异层线段的 via。
int count_vias(const std::vector<RouteSegment>& routes) {
    std::unordered_map<std::string, std::vector<const RouteSegment*>> by_net;
    for (const auto& route : routes) by_net[route.net].push_back(&route);
    int vias = 0;
    for (const auto& [net, segments] : by_net) {
        (void)net;
        for (std::size_t i = 1; i < segments.size(); ++i) {
            const auto& first = *segments[i - 1];
            const auto& second = *segments[i];
            if (first.x2 == second.x1 && first.y2 == second.y1 && first.layer != second.layer) ++vias;
        }
    }
    return vias;
}

}  // namespace

// 返回线网的约束中值线宽或默认线宽。
double default_width(const Circuit& circuit, const std::string& net_name) {
    const auto found = circuit.constraints.wire_widths.find(net_name);
    return found == circuit.constraints.wire_widths.end()
               ? 1.0
               : (found->second.min_width + found->second.max_width) / 2.0;
}

// 根据放置结果生成确定性的 Manhattan 布线。
std::vector<RouteSegment> route_manhattan(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements) {
    std::vector<std::string> net_names = circuit.net_order;
    std::stable_sort(net_names.begin(), net_names.end(), [&](const auto& left, const auto& right) {
        return priority_order(circuit.nets.at(left).priority) < priority_order(circuit.nets.at(right).priority);
    });
    std::vector<RouteSegment> routes;
    for (const auto& name : net_names) {
        const auto& net = circuit.nets.at(name);
        std::vector<const Pin*> pins;
        for (const auto& terminal : net.terminals) {
            const auto found = circuit.pins.find(terminal);
            if (found != circuit.pins.end()) pins.push_back(&found->second);
        }
        if (pins.size() < 2) continue;
        const auto& root = *pins.front();
        const auto root_xy = placed_pin(circuit.modules.at(root.module), root, placements.at(root.module));
        const double width = default_width(circuit, name);
        for (std::size_t index = 1; index < pins.size(); ++index) {
            const auto& pin = *pins[index];
            const auto end_xy = placed_pin(circuit.modules.at(pin.module), pin, placements.at(pin.module));
            const std::pair<double, double> mid{end_xy.first, root_xy.second};
            const std::string layer2 = pin.layer != root.layer ? pin.layer : root.layer;
            append_nonzero(routes, {name, root.layer, root_xy.first, root_xy.second, mid.first, mid.second, width});
            append_nonzero(routes, {name, layer2, mid.first, mid.second, end_xy.first, end_xy.second, width});
        }
    }
    return routes;
}

// 计算面积、线长、bend 和 via 等基础指标。
Metrics measure(const Circuit& circuit, const Solution& solution) {
    double max_x = 0.0;
    double max_y = 0.0;
    for (const auto& id : solution.placement_order) {
        const auto& placement = solution.placements.at(id);
        const auto size = placed_size(circuit.modules.at(id), placement);
        max_x = std::max(max_x, placement.x + size.first);
        max_y = std::max(max_y, placement.y + size.second);
    }
    double wirelength = 0.0;
    for (const auto& route : solution.routes) {
        wirelength += std::abs(route.x2 - route.x1) + std::abs(route.y2 - route.y1);
    }
    Metrics metrics;
    metrics.area = max_x * max_y;
    metrics.wirelength = wirelength;
    metrics.bend_count = count_bends(solution.routes);
    metrics.via_count = count_vias(solution.routes);
    return metrics;
}

}  // namespace sapr

