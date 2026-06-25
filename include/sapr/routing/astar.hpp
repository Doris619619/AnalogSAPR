#pragma once

#include "sapr/routing/grid.hpp"
#include "sapr/routing/obstacle.hpp"
#include "sapr/routing/path.hpp"

namespace sapr::routing {

struct AStarConfig {
    double wirelength_weight{1.0};
    double bend_weight{3.0};
    double via_weight{5.0};
    int max_expanded_nodes{200000};
};

GridPath find_astar_path(
    const Grid& grid,
    const ObstacleMap& obstacles,
    const GridPoint& start,
    const GridPoint& goal,
    const AStarConfig& config = AStarConfig{});

}  // namespace sapr::routing
