// 文件职责：实现由 Circuit 和 placement 构建多层布线环境的逻辑。
#include "sapr/routing/routing_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "sapr/routing/layer.hpp"
#include "sapr/routing/transform.hpp"

namespace sapr::routing {
namespace {

// 将一个点纳入连续坐标边界统计。
void include_point(Point point, double& min_x, double& min_y, double& max_x, double& max_y, bool& has_bounds) {
    if (!has_bounds) {
        min_x = max_x = point.x;
        min_y = max_y = point.y;
        has_bounds = true;
        return;
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
}

// 将一个矩形纳入连续坐标边界统计。
void include_rect(Rect rect, double& min_x, double& min_y, double& max_x, double& max_y, bool& has_bounds) {
    const Rect normalized = normalize_rect(rect);
    include_point(Point{normalized.x1, normalized.y1}, min_x, min_y, max_x, max_y, has_bounds);
    include_point(Point{normalized.x2, normalized.y2}, min_x, min_y, max_x, max_y, has_bounds);
}

// 在指定网格段上放行 terminal access 点，让 active 内部 pin 可以逃逸到模块外。
void add_access_line(ObstacleMap& obstacles, const Grid& grid, GridPoint start, GridPoint end) {
    const int dx = (end.ix > start.ix) ? 1 : (end.ix < start.ix ? -1 : 0);
    const int dy = (end.iy > start.iy) ? 1 : (end.iy < start.iy ? -1 : 0);
    GridPoint current = start;
    while (true) {
        for (int layer = 0; layer < grid.layer_count(); ++layer) {
            obstacles.add_terminal_point(GridPoint{current.ix, current.iy, layer});
        }
        if (current.ix == end.ix && current.iy == end.iy) break;
        current.ix += dx;
        current.iy += dy;
    }
}

// 为 pin 选择离 active region 最近的一侧，并放行一条到 active 外的短 access corridor。
void add_pin_access(
    ObstacleMap& obstacles,
    const Grid& grid,
    const GlobalPin& pin,
    const Rect& active) {
    const Rect rect = normalize_rect(active);
    GridPoint start = grid.snap_to_grid(pin.location, pin.layer);
    for (int layer = 0; layer < grid.layer_count(); ++layer) {
        obstacles.add_terminal_point(GridPoint{start.ix, start.iy, layer});
    }

    const double escape = 2.0 * grid.step();
    const double left = std::abs(pin.location.x - rect.x1);
    const double right = std::abs(rect.x2 - pin.location.x);
    const double bottom = std::abs(pin.location.y - rect.y1);
    const double top = std::abs(rect.y2 - pin.location.y);

    struct AccessCandidate {
        double distance{};
        Point target;
    };
    const std::vector<AccessCandidate> candidates{
        {left, Point{rect.x1 - escape, pin.location.y}},
        {right, Point{rect.x2 + escape, pin.location.y}},
        {bottom, Point{pin.location.x, rect.y1 - escape}},
        {top, Point{pin.location.x, rect.y2 + escape}},
    };
    Point target = pin.location;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) {
        const auto snapped = grid.snap_to_grid(candidate.target, pin.layer);
        if (!grid.in_bounds(snapped)) continue;
        if (candidate.distance < best_distance) {
            best_distance = candidate.distance;
            target = candidate.target;
        }
    }
    if (!std::isfinite(best_distance)) {
        target.x = std::clamp(pin.location.x, grid.min_x(), grid.max_x());
        target.y = std::clamp(pin.location.y, grid.min_y(), grid.max_y());
    }
    add_access_line(obstacles, grid, start, grid.snap_to_grid(target, pin.layer));
}

}  // namespace

// 构建网格、障碍物、全局 pin 和 terminal 例外点。
RoutingContext::RoutingContext(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements,
    const GridConfig& config)
    : circuit_(circuit) {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool has_bounds = false;

    std::vector<std::pair<std::string, Rect>> active_regions;
    std::unordered_map<std::string, Rect> active_by_module;
    for (const auto& [module_id, placement] : placements) {
        const auto module_it = circuit_.modules.find(module_id);
        if (module_it == circuit_.modules.end()) {
            warnings_.push_back("placement references missing module: " + module_id);
            continue;
        }
        const Module& module = module_it->second;
        const Rect bbox = transform_module_bbox_to_global(module, placement);
        const Rect active = transform_active_to_global(module, placement);
        include_rect(bbox, min_x, min_y, max_x, max_y, has_bounds);
        include_rect(active, min_x, min_y, max_x, max_y, has_bounds);
        active_regions.push_back({module_id, active});
        active_by_module[module_id] = active;
    }

    for (const auto& [key, pin] : circuit_.pins) {
        const auto module_it = circuit_.modules.find(pin.module);
        if (module_it == circuit_.modules.end()) {
            warnings_.push_back("pin references missing module: " + key);
            continue;
        }
        const auto placement_it = placements.find(pin.module);
        if (placement_it == placements.end()) {
            warnings_.push_back("pin module is missing placement: " + key);
            continue;
        }

        try {
            GlobalPin global_pin;
            global_pin.key = key;
            global_pin.module = pin.module;
            global_pin.pin = pin.name;
            global_pin.location = transform_pin_to_global(module_it->second, pin, placement_it->second);
            global_pin.layer = layer_to_index(pin.layer);
            include_point(global_pin.location, min_x, min_y, max_x, max_y, has_bounds);
            global_pins_[key] = global_pin;
        } catch (const std::exception& error) {
            warnings_.push_back("pin transform failed for " + key + ": " + error.what());
        }
    }

    if (!has_bounds) {
        min_x = min_y = 0.0;
        max_x = max_y = 0.0;
    }

    grid_ = std::make_unique<Grid>(config, min_x, min_y, max_x, max_y);

    for (const auto& [owner, active] : active_regions) {
        for (int layer = 0; layer < grid_->layer_count(); ++layer) {
            obstacles_.add_obstacle(Obstacle{active, layer, "active_region", owner});
        }
    }

    for (const auto& [key, global_pin] : global_pins_) {
        (void)key;
        const auto active_it = active_by_module.find(global_pin.module);
        if (active_it != active_by_module.end() && contains_point(active_it->second, global_pin.location)) {
            add_pin_access(obstacles_, *grid_, global_pin, active_it->second);
        } else {
            obstacles_.add_terminal_point(grid_->snap_to_grid(global_pin.location, global_pin.layer));
        }
    }
}

// 返回已构建的规则网格。
const Grid& RoutingContext::grid() const {
    if (!grid_) {
        throw std::runtime_error("RoutingContext grid has not been built");
    }
    return *grid_;
}

// 返回障碍物地图。
const ObstacleMap& RoutingContext::obstacles() const {
    return obstacles_;
}

// 返回全局 pin 查询表。
const std::unordered_map<std::string, GlobalPin>& RoutingContext::global_pins() const {
    return global_pins_;
}

// 根据 WIRE_WIDTH 约束返回 net 默认线宽。
double RoutingContext::default_width_for_net(const std::string& net) const {
    const auto width_it = circuit_.constraints.wire_widths.find(net);
    if (width_it != circuit_.constraints.wire_widths.end()) {
        return width_it->second.min_width;
    }
    return 1.0;
}

// 返回构建上下文时记录的非致命警告。
const std::vector<std::string>& RoutingContext::warnings() const {
    return warnings_;
}

}  // namespace sapr::routing
