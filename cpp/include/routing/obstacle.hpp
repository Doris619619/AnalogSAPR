// 文件职责：声明多层障碍物和可通行性查询模型。
#pragma once

#include <string>
#include <vector>

#include "core/geometry.hpp"
#include "routing/grid.hpp"

namespace analog_sapr {

// 表示某一金属层上的障碍矩形。
struct Obstacle {
    Rect rect;
    int layer = 0;
    std::string reason;
    std::string owner;
};

// 管理所有障碍物，并支持 terminal 点例外。
class ObstacleMap {
public:
    // 添加一个指定层上的障碍物。
    void add_obstacle(const Obstacle& obstacle);

    // 将某个网格点标记为 pin terminal，terminal 点允许作为起终点。
    void add_terminal_point(const GridPoint& point);

    // 查询连续坐标在指定层是否被阻塞。
    bool is_blocked(const Point& point, int layer) const;

    // 查询网格点是否被阻塞，terminal 点会被视为可用。
    bool is_blocked(const GridPoint& point, const Grid& grid) const;

    // 返回所有障碍物，供调试和后续可视化使用。
    const std::vector<Obstacle>& obstacles() const;

    // 粗略统计当前网格中被障碍物阻塞的点数量。
    long long estimate_blocked_grid_points(const Grid& grid) const;

private:
    std::vector<Obstacle> obstacles_;
    std::vector<GridPoint> terminal_points_;
};

}  // namespace analog_sapr
