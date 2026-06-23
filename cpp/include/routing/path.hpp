// 文件职责：定义阶段 2 A* 布线候选路径和路径指标数据结构。
#pragma once

#include <string>
#include <vector>

#include "routing/grid.hpp"

namespace analog_sapr {

// 表示一条网格路径的基础指标。
struct PathMetrics {
    double wirelength = 0.0;
    int bend_count = 0;
    int via_count = 0;
    double cost = 0.0;
};

// 表示 A* 搜索得到的一条候选路径。
struct GridPath {
    bool success = false;
    std::string message;
    std::vector<GridPoint> points;
    PathMetrics metrics;
};

// 表示某条 net 上一对 terminal 之间的候选拓扑路径。
struct RouteCandidate {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    GridPath path;
};

}  // namespace analog_sapr
