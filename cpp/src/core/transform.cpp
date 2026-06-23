// 文件职责：实现器件局部坐标到全局布局坐标的转换。
#include "core/transform.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace analog_sapr {
namespace {

// 对点执行 X 轴镜像，即 y 取反。
Point reflect_x(const Point& point) {
    return Point{point.x, -point.y};
}

// 对点执行逆时针旋转。
Point rotate_ccw(const Point& point, int angle) {
    const int normalized = ((angle % 360) + 360) % 360;
    switch (normalized) {
        case 0:
            return point;
        case 90:
            return Point{-point.y, point.x};
        case 180:
            return Point{-point.x, -point.y};
        case 270:
            return Point{point.y, -point.x};
        default:
            throw std::runtime_error("只支持 0/90/180/270 旋转角度");
    }
}

// 根据 Cadence orient 解码为是否 X 镜像和旋转角。
std::pair<bool, int> decode_orient(const Placement& placement) {
    if (placement.orient == "R0") {
        return {false, 0};
    }
    if (placement.orient == "R90") {
        return {false, 90};
    }
    if (placement.orient == "R180") {
        return {false, 180};
    }
    if (placement.orient == "R270") {
        return {false, 270};
    }
    if (placement.orient == "MX") {
        return {true, 0};
    }
    if (placement.orient == "MY") {
        return {true, 180};
    }
    if (placement.orient == "MXR90") {
        return {true, 90};
    }
    if (placement.orient == "MYR90") {
        return {true, 270};
    }
    throw std::runtime_error("不支持的 orient: " + placement.orient);
}

// 将矩形四角变换后重新包围成轴对齐全局矩形。
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

Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement) {
    (void)module;
    const auto [has_reflection, rotation] = decode_orient(placement);
    Point transformed = local_point;
    if (has_reflection) {
        transformed = reflect_x(transformed);
    }
    transformed = rotate_ccw(transformed, rotation);
    return Point{transformed.x + placement.x, transformed.y + placement.y};
}

Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_local_point_to_global(module, Point{pin.x, pin.y}, placement);
}

Rect transform_active_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, module.active, placement);
}

Rect transform_module_bbox_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, Rect{0.0, 0.0, module.width, module.height}, placement);
}

}  // namespace analog_sapr
