#include "sapr/routing/routing_context.hpp"

#include <algorithm>
#include <stdexcept>

#include "sapr/routing/layer.hpp"
#include "sapr/routing/transform.hpp"

namespace sapr::routing {
namespace {

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

void include_rect(Rect rect, double& min_x, double& min_y, double& max_x, double& max_y, bool& has_bounds) {
    const Rect normalized = normalize_rect(rect);
    include_point(Point{normalized.x1, normalized.y1}, min_x, min_y, max_x, max_y, has_bounds);
    include_point(Point{normalized.x2, normalized.y2}, min_x, min_y, max_x, max_y, has_bounds);
}

}  // namespace

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
        obstacles_.add_terminal_point(grid_->snap_to_grid(global_pin.location, global_pin.layer));
    }
}

const Grid& RoutingContext::grid() const {
    if (!grid_) {
        throw std::runtime_error("RoutingContext grid has not been built");
    }
    return *grid_;
}

const ObstacleMap& RoutingContext::obstacles() const {
    return obstacles_;
}

const std::unordered_map<std::string, GlobalPin>& RoutingContext::global_pins() const {
    return global_pins_;
}

double RoutingContext::default_width_for_net(const std::string& net) const {
    const auto width_it = circuit_.constraints.wire_widths.find(net);
    if (width_it != circuit_.constraints.wire_widths.end()) {
        return width_it->second.min_width;
    }
    return 1.0;
}

const std::vector<std::string>& RoutingContext::warnings() const {
    return warnings_;
}

}  // namespace sapr::routing
