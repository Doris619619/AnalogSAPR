// 文件职责：实现障碍物登记、terminal 例外和阻塞查询。
#include "sapr/routing/obstacle.hpp"

namespace sapr::routing {
namespace {

// 判断两个网格点是否完全相同。
bool same_grid_point(const GridPoint& lhs, const GridPoint& rhs) {
    return lhs.ix == rhs.ix && lhs.iy == rhs.iy && lhs.layer == rhs.layer;
}

}  // namespace

// 添加一个障碍物。
void ObstacleMap::add_obstacle(const Obstacle& obstacle) {
    obstacles_.push_back(obstacle);
}

// 添加一个允许作为起终点的 terminal 网格点。
void ObstacleMap::add_terminal_point(const GridPoint& point) {
    terminal_points_.push_back(point);
}

// 查询连续坐标点在指定层是否被阻塞。
bool ObstacleMap::is_blocked(const Point& point, int layer) const {
    for (const auto& obstacle : obstacles_) {
        if (obstacle.layer == layer && contains_point(obstacle.rect, point)) {
            return true;
        }
    }
    return false;
}

// 查询网格点是否被阻塞，并对 terminal 点放行。
bool ObstacleMap::is_blocked(const GridPoint& point, const Grid& grid) const {
    for (const auto& terminal : terminal_points_) {
        if (same_grid_point(point, terminal)) {
            return false;
        }
    }
    return is_blocked(grid.grid_to_point(point), point.layer);
}

// 返回所有障碍物。
const std::vector<Obstacle>& ObstacleMap::obstacles() const {
    return obstacles_;
}

// 粗略统计规则网格内被阻塞的点数。
long long ObstacleMap::estimate_blocked_grid_points(const Grid& grid) const {
    long long blocked = 0;
    for (int layer = 0; layer < grid.layer_count(); ++layer) {
        for (int ix = 0; ix < grid.x_count(); ++ix) {
            for (int iy = 0; iy < grid.y_count(); ++iy) {
                if (is_blocked(GridPoint{ix, iy, layer}, grid)) {
                    ++blocked;
                }
            }
        }
    }
    return blocked;
}

}  // namespace sapr::routing
