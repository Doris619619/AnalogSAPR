// 文件职责：声明阶段 1 使用的基础几何类型和纯几何工具函数。
#pragma once

#include "core/model.hpp"

namespace analog_sapr {

// 表示二维点坐标，单位与输入输出保持一致，均为 um。
struct Point {
    double x = 0.0;
    double y = 0.0;
};

// 表示二维中心线段，用于后续把布线中心线转换为金属占用矩形。
struct Segment {
    Point start;
    Point end;
};

// 计算矩形宽度。
double rect_width(const Rect& rect);

// 计算矩形高度。
double rect_height(const Rect& rect);

// 计算矩形面积。
double rect_area(const Rect& rect);

// 规范化矩形坐标，保证 x1 <= x2 且 y1 <= y2。
Rect normalize_rect(const Rect& rect);

// 判断点是否在矩形内，边界视为在内部。
bool contains_point(const Rect& rect, const Point& point);

// 判断两个矩形是否相交，边界接触视为相交。
bool intersects(const Rect& lhs, const Rect& rhs);

// 按 margin 向四周扩张矩形。
Rect expand_rect(const Rect& rect, double margin);

// 判断线段是否为水平线段。
bool is_horizontal(const Segment& segment);

// 判断线段是否为垂直线段。
bool is_vertical(const Segment& segment);

// 计算线段的 Manhattan 长度。
double manhattan_length(const Segment& segment);

// 将水平或垂直中心线段按线宽转换为金属占用矩形。
Rect segment_to_rect(const Segment& segment, double width);

}  // namespace analog_sapr
