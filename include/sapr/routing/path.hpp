#pragma once

#include <string>
#include <vector>

#include "sapr/routing/grid.hpp"

namespace sapr::routing {

struct PathMetrics {
    double wirelength{};
    int bend_count{};
    int via_count{};
    double cost{};
};

struct GridPath {
    bool success{};
    std::string message;
    std::vector<GridPoint> points;
    PathMetrics metrics;
};

struct RouteCandidate {
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    GridPath path;
};

}  // namespace sapr::routing
