// 文件职责：声明基于多层网格和障碍物的 A* 单对端点寻路接口。
#pragma once

#include "sapr/routing/grid.hpp"
#include "sapr/routing/obstacle.hpp"
#include "sapr/routing/path.hpp"

namespace sapr::routing {

// 表示 A* 搜索的代价权重和搜索规模限制。
struct AStarConfig {
    double wirelength_weight{1.0};
    double bend_weight{3.0};
    double via_weight{5.0};
    double wire_width{1.0};
    int max_expanded_nodes{20000};
};

// 在多层网格上从 start 搜索到 goal，并避开障碍物。
GridPath find_astar_path(
    const Grid& grid,
    const ObstacleMap& obstacles,
    const GridPoint& start,
    const GridPoint& goal,
    const AStarConfig& config = AStarConfig{});

}  // namespace sapr::routing
