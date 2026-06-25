#include "sapr/routing/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace sapr::routing {
namespace {

enum class Direction {
    None = 0,
    Left = 1,
    Right = 2,
    Down = 3,
    Up = 4,
    Via = 5,
};

struct SearchState {
    GridPoint point;
    Direction direction{Direction::None};
};

struct QueueNode {
    SearchState state;
    double priority{};
};

struct StateRecord {
    double g_cost{std::numeric_limits<double>::infinity()};
    SearchState previous;
    bool has_previous{};
    PathMetrics metrics;
};

struct QueueCompare {
    bool operator()(const QueueNode& lhs, const QueueNode& rhs) const {
        return lhs.priority > rhs.priority;
    }
};

std::int64_t state_key(const SearchState& state, const Grid& grid) {
    const std::int64_t direction = static_cast<std::int64_t>(state.direction);
    const std::int64_t layer = static_cast<std::int64_t>(state.point.layer);
    const std::int64_t iy = static_cast<std::int64_t>(state.point.iy);
    const std::int64_t ix = static_cast<std::int64_t>(state.point.ix);
    return (((ix * grid.y_count() + iy) * grid.layer_count() + layer) * 8) + direction;
}

bool same_point(const GridPoint& lhs, const GridPoint& rhs) {
    return lhs.ix == rhs.ix && lhs.iy == rhs.iy && lhs.layer == rhs.layer;
}

Direction direction_between(const GridPoint& from, const GridPoint& to) {
    if (from.layer != to.layer) return Direction::Via;
    if (to.ix < from.ix) return Direction::Left;
    if (to.ix > from.ix) return Direction::Right;
    if (to.iy < from.iy) return Direction::Down;
    if (to.iy > from.iy) return Direction::Up;
    return Direction::None;
}

bool is_bend(Direction previous, Direction current) {
    if (previous == Direction::None || previous == Direction::Via || current == Direction::Via) {
        return false;
    }
    return previous != current;
}

double heuristic(const GridPoint& current, const GridPoint& goal, const Grid& grid, const AStarConfig& config) {
    const double planar = (std::abs(current.ix - goal.ix) + std::abs(current.iy - goal.iy)) * grid.step();
    const int layer_distance = std::abs(current.layer - goal.layer);
    return config.wirelength_weight * planar + config.via_weight * static_cast<double>(layer_distance);
}

GridPath reconstruct_path(
    const SearchState& goal_state,
    const std::unordered_map<std::int64_t, StateRecord>& records,
    const Grid& grid) {
    GridPath result;
    result.success = true;
    result.message = "success";

    SearchState current = goal_state;
    while (true) {
        result.points.push_back(current.point);
        const auto record_it = records.find(state_key(current, grid));
        if (record_it == records.end() || !record_it->second.has_previous) {
            if (record_it != records.end()) {
                result.metrics = record_it->second.metrics;
            }
            break;
        }
        current = record_it->second.previous;
    }
    std::reverse(result.points.begin(), result.points.end());

    const auto goal_record = records.find(state_key(goal_state, grid));
    if (goal_record != records.end()) {
        result.metrics = goal_record->second.metrics;
    }
    return result;
}

}  // namespace

GridPath find_astar_path(
    const Grid& grid,
    const ObstacleMap& obstacles,
    const GridPoint& start,
    const GridPoint& goal,
    const AStarConfig& config) {
    if (!grid.in_bounds(start) || !grid.in_bounds(goal)) {
        return GridPath{false, "start or goal is outside grid bounds", {}, {}};
    }

    SearchState start_state{start, Direction::None};
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
    std::unordered_map<std::int64_t, StateRecord> records;

    StateRecord start_record;
    start_record.g_cost = 0.0;
    records[state_key(start_state, grid)] = start_record;
    open.push(QueueNode{start_state, heuristic(start, goal, grid, config)});

    int expanded = 0;
    while (!open.empty()) {
        const QueueNode current_node = open.top();
        open.pop();

        const auto current_key = state_key(current_node.state, grid);
        const auto current_record_it = records.find(current_key);
        if (current_record_it == records.end()) {
            continue;
        }

        if (same_point(current_node.state.point, goal)) {
            return reconstruct_path(current_node.state, records, grid);
        }

        ++expanded;
        if (expanded > config.max_expanded_nodes) {
            return GridPath{false, "A* exceeded max expanded nodes", {}, {}};
        }

        const StateRecord current_record = current_record_it->second;
        for (const auto& neighbor : grid.neighbors(current_node.state.point)) {
            if (!same_point(neighbor, goal) && obstacles.is_blocked(neighbor, grid)) {
                continue;
            }

            const Direction next_direction = direction_between(current_node.state.point, neighbor);
            const bool via_move = next_direction == Direction::Via;
            const bool bend_move = is_bend(current_node.state.direction, next_direction);
            const double wirelength_delta = via_move ? 0.0 : grid.step();
            const double cost_delta =
                config.wirelength_weight * wirelength_delta +
                (bend_move ? config.bend_weight : 0.0) +
                (via_move ? config.via_weight : 0.0);

            SearchState next_state{neighbor, next_direction};
            const double next_g = current_record.g_cost + cost_delta;
            const auto next_key = state_key(next_state, grid);
            auto next_record_it = records.find(next_key);
            if (next_record_it == records.end() || next_g < next_record_it->second.g_cost) {
                StateRecord next_record;
                next_record.g_cost = next_g;
                next_record.previous = current_node.state;
                next_record.has_previous = true;
                next_record.metrics = current_record.metrics;
                next_record.metrics.wirelength += wirelength_delta;
                next_record.metrics.bend_count += bend_move ? 1 : 0;
                next_record.metrics.via_count += via_move ? 1 : 0;
                next_record.metrics.cost = next_g;
                records[next_key] = next_record;

                const double priority = next_g + heuristic(neighbor, goal, grid, config);
                open.push(QueueNode{next_state, priority});
            }
        }
    }

    return GridPath{false, "A* could not find a feasible path", {}, {}};
}

}  // namespace sapr::routing
