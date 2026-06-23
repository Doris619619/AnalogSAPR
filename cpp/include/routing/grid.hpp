// 文件职责：声明多层布线网格模型和网格坐标工具函数。
#pragma once

#include <vector>

#include "core/geometry.hpp"

namespace analog_sapr {

// 表示网格构建参数。
struct GridConfig {
    double step = 1.0;
    double margin = 5.0;
    int layer_count = 7;
};

// 表示三维网格点，ix/iy 是平面网格坐标，layer 是内部层编号。
struct GridPoint {
    int ix = 0;
    int iy = 0;
    int layer = 0;
};

// 表示阶段 1 的规则网格边界和邻接关系。
class Grid {
public:
    // 根据配置和连续坐标范围构造规则网格。
    Grid(const GridConfig& config, double min_x, double min_y, double max_x, double max_y);

    // 将连续坐标吸附到最近的网格点。
    GridPoint snap_to_grid(const Point& point, int layer = 0) const;

    // 将网格点转换回连续坐标。
    Point grid_to_point(const GridPoint& point) const;

    // 判断网格点是否在边界和合法层范围内。
    bool in_bounds(const GridPoint& point) const;

    // 枚举同层上下左右四邻居。
    std::vector<GridPoint> planar_neighbors(const GridPoint& point) const;

    // 枚举同一平面位置的上下层 via 邻居。
    std::vector<GridPoint> via_neighbors(const GridPoint& point) const;

    // 枚举同层四邻居和 via 邻居。
    std::vector<GridPoint> neighbors(const GridPoint& point) const;

    double step() const;
    double min_x() const;
    double min_y() const;
    double max_x() const;
    double max_y() const;
    int layer_count() const;
    int x_count() const;
    int y_count() const;

private:
    GridConfig config_;
    double min_x_ = 0.0;
    double min_y_ = 0.0;
    double max_x_ = 0.0;
    double max_y_ = 0.0;
    int x_count_ = 0;
    int y_count_ = 0;
};

}  // namespace analog_sapr
