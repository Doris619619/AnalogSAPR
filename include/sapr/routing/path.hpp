// 文件职责：声明 A* 候选路径和路径指标的数据结构。
#pragma once

#include <string>
#include <vector>

#include "sapr/routing/grid.hpp"

namespace sapr::routing {

// 汇总一条网格路径的线长、拐弯、via 和代价。
struct PathMetrics {
    double wirelength{};
    int bend_count{};
    int via_count{};
    double cost{};
};

// 表示 A* 搜索得到的一条网格路径。
struct GridPath {
    bool success{};
    std::string message;
    std::vector<GridPoint> points;
    PathMetrics metrics;
};

// 表示某条 net 上一对 terminal 之间的一条候选路径。
struct RouteCandidate {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    GridPath path;
};

}  // namespace sapr::routing
