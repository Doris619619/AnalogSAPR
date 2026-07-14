/* 文件职责：按共用 placement 几何规则实现 routing 坐标变换。 */
#include "sapr/routing/transform.hpp"

#include <algorithm>
#include <array>
#include <utility>

#include "sapr/geometry.hpp"

namespace sapr::routing {
namespace {

/* 变换局部矩形四角并归一化为全局轴对齐外接框。 */
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

/* 复用共用 placement 几何实现变换局部点。 */
Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement) {
    const auto [x, y] = transform_placed_point(module, local_point.x, local_point.y, placement);
    return Point{x, y};
}

/* 复用局部点路径变换 routing 引脚。 */
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_local_point_to_global(module, Point{pin.x, pin.y}, placement);
}

/* 将 active region 变换为全局轴对齐矩形。 */
Rect transform_active_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, module.active, placement);
}

/* 将完整器件外接框变换为全局坐标。 */
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, Rect{0.0, 0.0, module.width, module.height}, placement);
}

}  // namespace sapr::routing
