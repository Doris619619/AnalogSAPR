// 文件职责：实现多层障碍物记录和可通行性查询。
#include "routing/obstacle.hpp"

namespace analog_sapr {
namespace {

// 判断两个网格点是否完全相同。
bool same_grid_point(const GridPoint& lhs, const GridPoint& rhs) {
    return lhs.ix == rhs.ix && lhs.iy == rhs.iy && lhs.layer == rhs.layer;
}

}  // namespace

void ObstacleMap::add_obstacle(const Obstacle& obstacle) {
    obstacles_.push_back(obstacle);
}

void ObstacleMap::add_terminal_point(const GridPoint& point) {
    terminal_points_.push_back(point);
}

bool ObstacleMap::is_blocked(const Point& point, int layer) const {
    for (const auto& obstacle : obstacles_) {
        if (obstacle.layer == layer && contains_point(obstacle.rect, point)) {
            return true;
        }
    }
    return false;
}

bool ObstacleMap::is_blocked(const GridPoint& point, const Grid& grid) const {
    for (const auto& terminal : terminal_points_) {
        if (same_grid_point(point, terminal)) {
            return false;
        }
    }
    return is_blocked(grid.grid_to_point(point), point.layer);
}

const std::vector<Obstacle>& ObstacleMap::obstacles() const {
    return obstacles_;
}

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

}  // namespace analog_sapr
