// 文件职责：声明布线障碍物和网格可通行性查询模型。
#pragma once

#include <string>
#include <vector>

#include "sapr/routing/grid.hpp"

namespace sapr::routing {

// 表示某一金属层上的矩形障碍物。
struct Obstacle {
    Rect rect;
    int layer{};
    std::string reason;
    std::string owner;
};

// 管理障碍物集合，并允许 pin terminal 作为起终点例外。
class ObstacleMap {
public:
    // 添加一个指定金属层上的障碍物。
    void add_obstacle(const Obstacle& obstacle);
    // 添加一个允许通过的 terminal 网格点。
    void add_terminal_point(const GridPoint& point);

    // 查询连续坐标在指定层是否被障碍物阻塞。
    [[nodiscard]] bool is_blocked(const Point& point, int layer) const;
    // 按线宽半径膨胀障碍物后查询连续坐标是否被阻塞。
    [[nodiscard]] bool is_blocked(const Point& point, int layer, double wire_width) const;
    // 查询网格点是否被阻塞，terminal 点会作为例外放行。
    [[nodiscard]] bool is_blocked(const GridPoint& point, const Grid& grid) const;
    // 按线宽半径膨胀障碍物后查询网格点是否被阻塞。
    [[nodiscard]] bool is_blocked(const GridPoint& point, const Grid& grid, double wire_width) const;
    // 返回所有障碍物，供调试和可视化使用。
    [[nodiscard]] const std::vector<Obstacle>& obstacles() const;
    // 估算当前网格中被障碍物阻塞的网格点数量。
    [[nodiscard]] long long estimate_blocked_grid_points(const Grid& grid) const;

private:
    std::vector<Obstacle> obstacles_;
    std::vector<GridPoint> terminal_points_;
};

}  // namespace sapr::routing
