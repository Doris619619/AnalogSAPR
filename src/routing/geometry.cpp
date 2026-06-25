// 文件职责：实现布线子模块使用的基础几何计算。
#include "sapr/routing/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sapr::routing {
namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

// 计算规范化矩形的宽度。
double rect_width(const Rect& rect) {
    const Rect normalized = normalize_rect(rect);
    return normalized.x2 - normalized.x1;
}

// 计算规范化矩形的高度。
double rect_height(const Rect& rect) {
    const Rect normalized = normalize_rect(rect);
    return normalized.y2 - normalized.y1;
}

// 计算规范化矩形的面积。
double rect_area(const Rect& rect) {
    return rect_width(rect) * rect_height(rect);
}

// 将任意角点顺序的矩形转换为 x1/y1 较小的形式。
Rect normalize_rect(const Rect& rect) {
    return Rect{
        std::min(rect.x1, rect.x2),
        std::min(rect.y1, rect.y2),
        std::max(rect.x1, rect.x2),
        std::max(rect.y1, rect.y2),
    };
}

// 判断点是否在矩形范围内。
bool contains_point(const Rect& rect, const Point& point) {
    const Rect normalized = normalize_rect(rect);
    return point.x >= normalized.x1 - kEpsilon && point.x <= normalized.x2 + kEpsilon &&
           point.y >= normalized.y1 - kEpsilon && point.y <= normalized.y2 + kEpsilon;
}

// 判断两个矩形是否有重叠或边界接触。
bool intersects(const Rect& lhs, const Rect& rhs) {
    const Rect a = normalize_rect(lhs);
    const Rect b = normalize_rect(rhs);
    return a.x1 <= b.x2 + kEpsilon && a.x2 + kEpsilon >= b.x1 &&
           a.y1 <= b.y2 + kEpsilon && a.y2 + kEpsilon >= b.y1;
}

// 按指定 margin 扩大矩形边界。
Rect expand_rect(const Rect& rect, double margin) {
    const Rect normalized = normalize_rect(rect);
    return Rect{
        normalized.x1 - margin,
        normalized.y1 - margin,
        normalized.x2 + margin,
        normalized.y2 + margin,
    };
}

// 判断线段是否近似水平。
bool is_horizontal(const Segment& segment) {
    return std::abs(segment.start.y - segment.end.y) <= kEpsilon;
}

// 判断线段是否近似垂直。
bool is_vertical(const Segment& segment) {
    return std::abs(segment.start.x - segment.end.x) <= kEpsilon;
}

// 计算线段端点之间的曼哈顿距离。
double manhattan_length(const Segment& segment) {
    return std::abs(segment.start.x - segment.end.x) + std::abs(segment.start.y - segment.end.y);
}

// 将水平或垂直中心线段转换为线宽覆盖矩形。
Rect segment_to_rect(const Segment& segment, double width) {
    if (width < 0.0) {
        throw std::runtime_error("wire width cannot be negative");
    }
    const double half_width = width / 2.0;
    if (is_horizontal(segment)) {
        return normalize_rect(Rect{
            segment.start.x,
            segment.start.y - half_width,
            segment.end.x,
            segment.end.y + half_width,
        });
    }
    if (is_vertical(segment)) {
        return normalize_rect(Rect{
            segment.start.x - half_width,
            segment.start.y,
            segment.end.x + half_width,
            segment.end.y,
        });
    }
    throw std::runtime_error("only horizontal or vertical centerline segments can be converted");
}

}  // namespace sapr::routing
