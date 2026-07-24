// 文件职责：实现由 Circuit 和 placement 构建多层布线环境的逻辑。
#include "sapr/routing/routing_context.hpp"

#include "sapr/constraints.hpp"

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
// 根据小尺寸版图和最小线宽选择更细的 grid step，避免相邻 pin 被 1um 默认网格吸到同一点。
GridConfig make_effective_grid_config(const Circuit& circuit, const GridConfig& config, double width, double height) {
    GridConfig adapted = config;
    double min_width = std::numeric_limits<double>::infinity();
    for (const auto& [_, rule] : circuit.constraints.wire_widths) {
        min_width = std::min(min_width, rule.min_width);
    }
    const double extent = std::max(width, height);
    if (std::isfinite(min_width) && extent > 0.0 && extent <= 50.0) {
        const double detailed_step = std::max(min_width, extent / 300.0);
        adapted.step = std::min(adapted.step, detailed_step);
    }
    return adapted;
}

// 表示 pin access 的目标点及其不可由坐标截断推断的逃逸方向。
struct PinAccessTarget {
    Point point;
    bool horizontal{};
    int direction{};
};

// 选择 pin 到 active 外最近且仍位于芯片范围内的逃逸目标与方向。
PinAccessTarget pin_access_target(const Point& pin_location, const Rect& active, double escape) {
    const Rect rect = normalize_rect(active);
    const double left = std::abs(pin_location.x - rect.x1);
    const double right = std::abs(rect.x2 - pin_location.x);
    const double bottom = std::abs(pin_location.y - rect.y1);
    const double top = std::abs(rect.y2 - pin_location.y);

    struct AccessCandidate {
        double distance{};
        Point target;
        bool horizontal{};
        int direction{};
    };
    const std::vector<AccessCandidate> candidates{
        {left, Point{rect.x1 - escape, pin_location.y}, true, -1},
        {right, Point{rect.x2 + escape, pin_location.y}, true, 1},
        {bottom, Point{pin_location.x, rect.y1 - escape}, false, -1},
        {top, Point{pin_location.x, rect.y2 + escape}, false, 1},
    };
    const auto legal = std::min_element(
        candidates.begin(),
        candidates.end(),
        [](const auto& lhs, const auto& rhs) {
            const bool lhs_legal = lhs.target.x >= 0.0 && lhs.target.y >= 0.0;
            const bool rhs_legal = rhs.target.x >= 0.0 && rhs.target.y >= 0.0;
            if (lhs_legal != rhs_legal) return lhs_legal;
            return lhs.distance < rhs.distance;
        });
    if (legal == candidates.end() || legal->target.x < 0.0 || legal->target.y < 0.0) {
        throw std::runtime_error("pin has no in-chip access direction");
    }
    return PinAccessTarget{legal->target, legal->horizontal, legal->direction};
}

// 返回可能使用该 pin 的 net 所需的最大默认线宽，保证 access point 对所有关联 net 都在 keep-out 外。
double maximum_width_for_pin(const Circuit& circuit, const std::string& pin_key) {
    double result = 1.0;
    for (const auto& [net_name, net] : circuit.nets) {
        if (std::find(net.terminals.begin(), net.terminals.end(), pin_key) == net.terminals.end()) continue;
        const auto width = circuit.constraints.wire_widths.find(net_name);
        if (width != circuit.constraints.wire_widths.end()) {
            result = std::max(result, width->second.min_width);
        }
    }
    return result;
}

// 计算引脚 access point 必须跨出的 keep-out 距离，并为网格吸附保留一个步长余量。
double pin_access_clearance(const Circuit& circuit, const std::string& pin_key, int layer, double grid_step) {
    return active_route_spacing(circuit, index_to_layer(layer)) +
           maximum_width_for_pin(circuit, pin_key) / 2.0 + grid_step;
}

// 将 access 网格点沿既定逃逸方向推至 active keep-out 外，避免网格吸附把端点拉回禁入区。
GridPoint push_access_outside_keep_out(
    const Grid& grid,
    GridPoint access,
    const PinAccessTarget& target,
    const Rect& active,
    double clearance) {
    const Rect rect = normalize_rect(active);
    const auto outside = [&](const Point& point) {
        return target.horizontal
                   ? (target.direction < 0 ? point.x <= rect.x1 - clearance : point.x >= rect.x2 + clearance)
                   : (target.direction < 0 ? point.y <= rect.y1 - clearance : point.y >= rect.y2 + clearance);
    };
    while (grid.in_bounds(access) && !outside(grid.grid_to_point(access))) {
        if (target.horizontal) access.ix += target.direction;
        else access.iy += target.direction;
    }
    return access;
}

// 在给定网格上计算 pin 的正交 access corridor 及其实际 A* access 格点，不直接修改上下文状态。
std::pair<PinAccessCorridor, GridPoint> make_pin_access_corridor(
    const Circuit& circuit,
    const Grid& grid,
    const GlobalPin& pin,
    const Rect& active) {
    const Rect rect = normalize_rect(active);
    const double clearance = pin_access_clearance(circuit, pin.key, pin.layer, grid.step());
    const PinAccessTarget target = pin_access_target(pin.location, rect, clearance);
    const GridPoint access = push_access_outside_keep_out(
        grid, grid.snap_to_grid(target.point, pin.layer), target, rect, clearance);
    const Point access_point = grid.grid_to_point(access);
    const Point bend = target.horizontal
                           ? Point{access_point.x, pin.location.y}
                           : Point{pin.location.x, access_point.y};
    return {PinAccessCorridor{pin.key, pin.layer, pin.location, bend, access_point}, access};
}

}  // namespace

// 根据允许的金属层数构造 GridConfig；非法层数直接抛错，避免静默截断。
GridConfig make_grid_config_for_routing_layers(int routing_layers) {
    const int max_layers = static_cast<int>(supported_layers().size());
    if (routing_layers < 1 || routing_layers > max_layers) {
        throw std::runtime_error(
            "routing_layers must be in [1, " + std::to_string(max_layers) + "], got " +
            std::to_string(routing_layers));
    }
    GridConfig config;
    config.layer_count = routing_layers;
    return config;
}

GridConfig effective_grid_config_for_layout(
    const Circuit& circuit,
    const GridConfig& config,
    double width,
    double height) {
    return make_effective_grid_config(circuit, config, width, height);
}

// 构建网格、障碍物、全局 pin 和 terminal 例外点。
RoutingContext::RoutingContext(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements,
    const GridConfig& config,
    const std::vector<Point>& extra_routing_points)
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
        active_regions_.push_back(active);
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

    for (const auto& point : extra_routing_points) {
        include_point(point, min_x, min_y, max_x, max_y, has_bounds);
    }

    for (const auto& [_, global_pin] : global_pins_) {
        const auto active_it = active_by_module.find(global_pin.module);
        if (active_it != active_by_module.end() && contains_point(active_it->second, global_pin.location)) {
            const int layer = std::clamp(global_pin.layer, 0, config.layer_count - 1);
            const double clearance = pin_access_clearance(circuit_, global_pin.key, layer, config.step);
            include_point(
                pin_access_target(global_pin.location, active_it->second, clearance).point,
                min_x,
                min_y,
                max_x,
                max_y,
                has_bounds);
        }
    }

    if (!has_bounds) {
        min_x = min_y = 0.0;
        max_x = max_y = 0.0;
    }
    // 芯片左/下边界固定为 0；即使最左器件不贴边，也必须为边界侧 pin access 保留网格点。
    min_x = 0.0;
    min_y = 0.0;

    const GridConfig effective_config =
        effective_grid_config_for_layout(circuit_, config, max_x - min_x, max_y - min_y);
    grid_ = std::make_unique<Grid>(effective_config, min_x, min_y, max_x, max_y);

    for (auto& [key, global_pin] : global_pins_) {
        if (global_pin.layer < 0 || global_pin.layer >= grid_->layer_count()) {
            warnings_.push_back(
                "pin layer exceeds routing_layers for " + key + "; clamping to " +
                index_to_layer(grid_->layer_count() - 1));
            global_pin.layer = grid_->layer_count() - 1;
        }
    }

    // active region 只阻挡器件所在低层 M1；M2+ 允许从器件上方跨过，避免被建成贯穿各层的垂直障碍柱。
    // 第一阶段网格只用于求出离散化后的真实 access 格点；若其越界，则将该格点纳入边界后重建正式网格。
    bool expand_for_access = false;
    for (const auto& [_, global_pin] : global_pins_) {
        const auto active_it = active_by_module.find(global_pin.module);
        if (active_it == active_by_module.end() || !contains_point(active_it->second, global_pin.location)) continue;
        const auto [corridor, access] = make_pin_access_corridor(circuit_, *grid_, global_pin, active_it->second);
        if (grid_->in_bounds(access)) continue;
        include_point(corridor.access_point, min_x, min_y, max_x, max_y, has_bounds);
        expand_for_access = true;
    }
    if (expand_for_access) {
        grid_ = std::make_unique<Grid>(effective_config, min_x, min_y, max_x, max_y);
    }

    for (const auto& [owner, active] : active_regions) {
        for (int layer = 0; layer < grid_->layer_count(); ++layer) {
            const std::string layer_name = index_to_layer(layer);
            if (!active_region_blocked(circuit_, layer_name)) continue;
            obstacles_.add_obstacle(Obstacle{
                active, layer, "active_region", owner, ObstacleKind::ActiveRegion,
                active_route_spacing(circuit_, layer_name), true});
        }
    }

    for (auto& [key, global_pin] : global_pins_) {
        const auto active_it = active_by_module.find(global_pin.module);
        if (active_it != active_by_module.end() && contains_point(active_it->second, global_pin.location)) {
            const auto [corridor, access] = make_pin_access_corridor(circuit_, *grid_, global_pin, active_it->second);
            if (!grid_->in_bounds(access)) {
                throw std::runtime_error(
                    "pin access point exceeds rebuilt routing grid: " + key +
                    " access=(" + std::to_string(corridor.access_point.x) + "," +
                    std::to_string(corridor.access_point.y) + ") grid=[(" +
                    std::to_string(grid_->min_x()) + "," + std::to_string(grid_->min_y()) + ")-(" +
                    std::to_string(grid_->max_x()) + "," + std::to_string(grid_->max_y()) + ")]");
            }
            pin_access_corridors_[key] = corridor;
            global_pin.location = corridor.access_point;
        } else {
            // 非 active 内 pin 无需障碍物例外；A* 起终点也必须通过统一 keep-out 检查。
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

const std::vector<Rect>& RoutingContext::active_regions() const {
    return active_regions_;
}

// 返回全局 pin 查询表。
const std::unordered_map<std::string, GlobalPin>& RoutingContext::global_pins() const {
    return global_pins_;
}

const std::unordered_map<std::string, PinAccessCorridor>& RoutingContext::pin_access_corridors() const {
    return pin_access_corridors_;
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
