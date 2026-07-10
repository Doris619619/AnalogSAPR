// 文件职责：声明多层布线规则网格和网格坐标邻接关系。
#pragma once

#include <vector>

#include "sapr/routing/geometry.hpp"

namespace sapr::routing {

// 表示规则网格的构建参数。
// layer_count 默认 1：只允许 M1，避免高层逃逸掩盖 placement/拓扑问题；可用 CLI --routing-layers 放开到 3/7。
struct GridConfig {
    double step{1.0};
    double margin{5.0};
    int layer_count{1};
};

// 表示三维布线网格点，layer 是金属层编号。
struct GridPoint {
    int ix{};
    int iy{};
    int layer{};
};

// 管理规则网格边界、坐标吸附和邻居枚举。
class Grid {
public:
    // 根据连续坐标边界和配置创建规则网格。
    Grid(const GridConfig& config, double min_x, double min_y, double max_x, double max_y);

    // 将连续坐标吸附到最近的网格点。
    [[nodiscard]] GridPoint snap_to_grid(const Point& point, int layer = 0) const;
    // 将网格点转换回连续坐标。
    [[nodiscard]] Point grid_to_point(const GridPoint& point) const;
    // 判断网格点是否位于合法边界内。
    [[nodiscard]] bool in_bounds(const GridPoint& point) const;
    // 枚举同层上下左右四邻居。
    [[nodiscard]] std::vector<GridPoint> planar_neighbors(const GridPoint& point) const;
    // 枚举同一平面位置上的上下层 via 邻居。
    [[nodiscard]] std::vector<GridPoint> via_neighbors(const GridPoint& point) const;
    // 枚举同层邻居和 via 邻居。
    [[nodiscard]] std::vector<GridPoint> neighbors(const GridPoint& point) const;

    // 返回网格间距。
    [[nodiscard]] double step() const;
    // 返回网格连续坐标的 x 下界。
    [[nodiscard]] double min_x() const;
    // 返回网格连续坐标的 y 下界。
    [[nodiscard]] double min_y() const;
    // 返回网格连续坐标的 x 上界。
    [[nodiscard]] double max_x() const;
    // 返回网格连续坐标的 y 上界。
    [[nodiscard]] double max_y() const;
    // 返回金属层数量。
    [[nodiscard]] int layer_count() const;
    // 返回 x 方向网格点数量。
    [[nodiscard]] int x_count() const;
    // 返回 y 方向网格点数量。
    [[nodiscard]] int y_count() const;

private:
    GridConfig config_;
    double min_x_{};
    double min_y_{};
    double max_x_{};
    double max_y_{};
    int x_count_{};
    int y_count_{};
};

}  // namespace sapr::routing
