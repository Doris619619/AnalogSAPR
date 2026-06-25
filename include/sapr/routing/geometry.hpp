// 文件职责：声明布线子模块使用的基础几何类型和矩形、线段工具函数。
#pragma once

#include "sapr/model.hpp"

namespace sapr::routing {

// 表示布线模型内部使用的二维连续坐标点。
struct Point {
    double x{};
    double y{};
};

// 表示一段水平或垂直的中心线段，用于换算金属占用矩形。
struct Segment {
    Point start;
    Point end;
};

// 计算矩形宽度，输入坐标会先规范化。
double rect_width(const Rect& rect);
// 计算矩形高度，输入坐标会先规范化。
double rect_height(const Rect& rect);
// 计算矩形面积。
double rect_area(const Rect& rect);
// 规范化矩形坐标，保证左下角和右上角顺序一致。
Rect normalize_rect(const Rect& rect);
// 判断点是否落在矩形内，边界视为在内部。
bool contains_point(const Rect& rect, const Point& point);
// 判断两个矩形是否相交，边界接触视为相交。
bool intersects(const Rect& lhs, const Rect& rhs);
// 按指定边距向四周扩张矩形。
Rect expand_rect(const Rect& rect, double margin);
// 判断中心线段是否水平。
bool is_horizontal(const Segment& segment);
// 判断中心线段是否垂直。
bool is_vertical(const Segment& segment);
// 计算线段的曼哈顿长度。
double manhattan_length(const Segment& segment);
// 将中心线段按线宽转换为金属占用矩形。
Rect segment_to_rect(const Segment& segment, double width);

}  // namespace sapr::routing
