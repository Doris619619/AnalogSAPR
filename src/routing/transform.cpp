// 文件职责：实现器件局部坐标到全局 placement 坐标的转换。
#include "sapr/routing/transform.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace sapr::routing {
namespace {

// 对局部点执行 X 轴镜像。
Point reflect_x(const Point& point) {
    return Point{point.x, -point.y};
}

// 对局部点执行逆时针旋转。
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
            throw std::runtime_error("only 0/90/180/270 rotations are supported");
    }
}

// 将 placement 的 orient 解码为是否镜像和旋转角度。
std::pair<bool, int> decode_orient(const Placement& placement) {
    if (placement.orient == "R0") return {false, 0};
    if (placement.orient == "R90") return {false, 90};
    if (placement.orient == "R180") return {false, 180};
    if (placement.orient == "R270") return {false, 270};
    if (placement.orient == "MX") return {true, 0};
    if (placement.orient == "MY") return {true, 180};
    if (placement.orient == "MXR90") return {true, 90};
    if (placement.orient == "MYR90") return {true, 270};
    return {false, placement.angle};
}

// 将局部矩形四角变换后重新包围成全局轴对齐矩形。
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

// 将局部点先镜像、再旋转、最后平移到全局坐标。
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

// 将 pin 坐标从器件局部坐标转换为全局坐标。
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_local_point_to_global(module, Point{pin.x, pin.y}, placement);
}

// 将器件 active region 转换为全局坐标矩形。
Rect transform_active_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, module.active, placement);
}

// 将器件外接框转换为全局坐标矩形。
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement) {
    return transform_rect_to_global(module, Rect{0.0, 0.0, module.width, module.height}, placement);
}

}  // namespace sapr::routing
