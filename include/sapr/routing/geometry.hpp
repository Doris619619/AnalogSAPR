#pragma once

#include "sapr/model.hpp"

namespace sapr::routing {

struct Point {
    double x{};
    double y{};
};

struct Segment {
    Point start;
    Point end;
};

double rect_width(const Rect& rect);
double rect_height(const Rect& rect);
double rect_area(const Rect& rect);
Rect normalize_rect(const Rect& rect);
bool contains_point(const Rect& rect, const Point& point);
bool intersects(const Rect& lhs, const Rect& rhs);
Rect expand_rect(const Rect& rect, double margin);
bool is_horizontal(const Segment& segment);
bool is_vertical(const Segment& segment);
double manhattan_length(const Segment& segment);
Rect segment_to_rect(const Segment& segment, double width);

}  // namespace sapr::routing
