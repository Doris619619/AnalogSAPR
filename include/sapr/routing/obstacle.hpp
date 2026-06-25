#pragma once

#include <string>
#include <vector>

#include "sapr/routing/grid.hpp"

namespace sapr::routing {

struct Obstacle {
    Rect rect;
    int layer{};
    std::string reason;
    std::string owner;
};

class ObstacleMap {
public:
    void add_obstacle(const Obstacle& obstacle);
    void add_terminal_point(const GridPoint& point);

    [[nodiscard]] bool is_blocked(const Point& point, int layer) const;
    [[nodiscard]] bool is_blocked(const GridPoint& point, const Grid& grid) const;
    [[nodiscard]] const std::vector<Obstacle>& obstacles() const;
    [[nodiscard]] long long estimate_blocked_grid_points(const Grid& grid) const;

private:
    std::vector<Obstacle> obstacles_;
    std::vector<GridPoint> terminal_points_;
};

}  // namespace sapr::routing
