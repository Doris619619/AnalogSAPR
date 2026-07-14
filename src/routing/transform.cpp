// Routing placement transformations using shared geometry rules.
#include "sapr/routing/transform.hpp"

#include <algorithm>
#include <array>
#include <utility>

#include "sapr/geometry.hpp"

namespace sapr::routing {
namespace {

// Transform rectangle corners and normalize the resulting global bounding box.
Rect transform_rect_to_global(const Module& module, const Rect& rect, const Placement& placement) {
    const Rect normalized = normalize_rect(rect);
    const std::array<Point, 4> corners = {
        Point{normalized.x1, normalized.y1},
        Point{normalized.x1, normalized.y2},
        Point{normalized.x2, normalized.y1},
        Point{normalized.x2, normalized.y2},
    };

    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    bool first = true;
    for (const auto& corner : corners) {
        const Point global = transform_local_point_to_global(module, corner, placement);
        if (first) {
            min_x = max_x = global.x;
            min_y = max_y = global.y;
            first = false;
        } else {
            min_x = std::min(min_x, global.x);
            min_y = std::min(min_y, global.y);
            max_x = std::max(max_x, global.x);
            max_y = std::max(max_y, global.y);
        }
    }
    return Rect{min_x, min_y, max_x, max_y};
}

}  // namespace

// Transform a local point through the shared placement geometry implementation.
Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement) {
    const auto [x, y] = transform_placed_point(module, local_point.x, local_point.y, placement);
    return Point{x, y};
}

// Transform a pin through the shared local point path.
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_local_point_to_global(module, Point{pin.x, pin.y}, placement);
}

// Transform the active region to a global axis-aligned rectangle.
Rect transform_active_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, module.active, placement);
}

// Transform the full module bounding box to global coordinates.
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, Rect{0.0, 0.0, module.width, module.height}, placement);
}

}  // namespace sapr::routing
