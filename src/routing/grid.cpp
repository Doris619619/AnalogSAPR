// 文件职责：实现多层规则网格的边界、坐标转换和邻居枚举。
#include "sapr/routing/grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sapr::routing {

// 根据连续坐标边界和网格配置构造规则网格。
Grid::Grid(const GridConfig& config, double min_x, double min_y, double max_x, double max_y) : config_(config) {
    if (config_.step <= 0.0) {
        throw std::runtime_error("grid step must be positive");
    }
    if (config_.layer_count <= 0) {
        throw std::runtime_error("grid layer_count must be positive");
    }

    const double low_x = std::min(min_x, max_x);
    const double low_y = std::min(min_y, max_y);
    const double high_x = std::max(min_x, max_x) + config_.margin;
    const double high_y = std::max(min_y, max_y) + config_.margin;

    min_x_ = std::floor(low_x / config_.step) * config_.step;
    min_y_ = std::floor(low_y / config_.step) * config_.step;
    max_x_ = std::ceil(high_x / config_.step) * config_.step;
    max_y_ = std::ceil(high_y / config_.step) * config_.step;
    x_count_ = static_cast<int>(std::round((max_x_ - min_x_) / config_.step)) + 1;
    y_count_ = static_cast<int>(std::round((max_y_ - min_y_) / config_.step)) + 1;
}

// 将连续坐标吸附到最近的网格坐标。
GridPoint Grid::snap_to_grid(const Point& point, int layer) const {
    return GridPoint{
        static_cast<int>(std::llround((point.x - min_x_) / config_.step)),
        static_cast<int>(std::llround((point.y - min_y_) / config_.step)),
        layer,
    };
}

// 将网格坐标恢复为连续坐标。
Point Grid::grid_to_point(const GridPoint& point) const {
    return Point{
        min_x_ + static_cast<double>(point.ix) * config_.step,
        min_y_ + static_cast<double>(point.iy) * config_.step,
    };
}

// 判断网格点是否在网格边界和层范围内。
bool Grid::in_bounds(const GridPoint& point) const {
    return point.ix >= 0 && point.ix < x_count_ &&
           point.iy >= 0 && point.iy < y_count_ &&
           point.layer >= 0 && point.layer < config_.layer_count;
}

// 枚举同一金属层上的四邻居。
std::vector<GridPoint> Grid::planar_neighbors(const GridPoint& point) const {
    const std::vector<GridPoint> candidates = {
        GridPoint{point.ix - 1, point.iy, point.layer},
        GridPoint{point.ix + 1, point.iy, point.layer},
        GridPoint{point.ix, point.iy - 1, point.layer},
        GridPoint{point.ix, point.iy + 1, point.layer},
    };
    std::vector<GridPoint> result;
    for (const auto& candidate : candidates) {
        if (in_bounds(candidate)) {
            result.push_back(candidate);
        }
    }
    return result;
}

// 枚举同一平面位置的上下层邻居。
std::vector<GridPoint> Grid::via_neighbors(const GridPoint& point) const {
    const std::vector<GridPoint> candidates = {
        GridPoint{point.ix, point.iy, point.layer - 1},
        GridPoint{point.ix, point.iy, point.layer + 1},
    };
    std::vector<GridPoint> result;
    for (const auto& candidate : candidates) {
        if (in_bounds(candidate)) {
            result.push_back(candidate);
        }
    }
    return result;
}

// 合并同层邻居和 via 邻居。
std::vector<GridPoint> Grid::neighbors(const GridPoint& point) const {
    std::vector<GridPoint> result = planar_neighbors(point);
    const auto vias = via_neighbors(point);
    result.insert(result.end(), vias.begin(), vias.end());
    return result;
}

// 返回网格间距。
double Grid::step() const { return config_.step; }
// 返回连续坐标 x 下界。
double Grid::min_x() const { return min_x_; }
// 返回连续坐标 y 下界。
double Grid::min_y() const { return min_y_; }
// 返回连续坐标 x 上界。
double Grid::max_x() const { return max_x_; }
// 返回连续坐标 y 上界。
double Grid::max_y() const { return max_y_; }
// 返回金属层数量。
int Grid::layer_count() const { return config_.layer_count; }
// 返回 x 方向网格点数量。
int Grid::x_count() const { return x_count_; }
// 返回 y 方向网格点数量。
int Grid::y_count() const { return y_count_; }

}  // namespace sapr::routing
