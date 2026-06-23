// 文件职责：实现阶段 2 的候选布线拓扑生成逻辑。
#include "routing/topology.hpp"

#include <algorithm>
#include <set>
#include <sstream>

#include "routing/astar.hpp"

namespace analog_sapr {
namespace {

// 将路径编码成字符串，用于去重。
std::string path_signature(const GridPath& path) {
    std::ostringstream output;
    for (const auto& point : path.points) {
        output << point.ix << "," << point.iy << "," << point.layer << ";";
    }
    return output.str();
}

// 返回用于产生不同候选路径的 A* 权重组合。
std::vector<AStarConfig> candidate_astar_configs() {
    return {
        AStarConfig{1.0, 3.0, 5.0, 200000},
        AStarConfig{1.0, 8.0, 5.0, 200000},
        AStarConfig{1.0, 3.0, 12.0, 200000},
        AStarConfig{1.0, 1.0, 3.0, 200000},
    };
}

// 从全局 pin 表中查找 terminal，缺失时返回空指针。
const GlobalPin* find_global_pin(const RoutingContext& context, const std::string& terminal) {
    const auto pin_it = context.global_pins().find(terminal);
    if (pin_it == context.global_pins().end()) {
        return nullptr;
    }
    return &pin_it->second;
}

}  // namespace

std::vector<RouteCandidate> generate_route_candidates(
    const Circuit& circuit,
    const RoutingContext& context,
    const CandidateConfig& config) {
    std::vector<RouteCandidate> candidates;
    const auto& grid = context.grid();
    const auto& obstacles = context.obstacles();

    for (const auto& [net_name, net] : circuit.nets) {
        if (net.terminals.size() < 2) {
            continue;
        }

        const std::string& root_terminal = net.terminals.front();
        const GlobalPin* root_pin = find_global_pin(context, root_terminal);
        if (root_pin == nullptr) {
            candidates.push_back(RouteCandidate{net_name, root_terminal, "", GridPath{false, "root terminal 缺少全局 pin", {}, {}}});
            continue;
        }

        for (std::size_t terminal_index = 1; terminal_index < net.terminals.size(); ++terminal_index) {
            const std::string& target_terminal = net.terminals[terminal_index];
            const GlobalPin* target_pin = find_global_pin(context, target_terminal);
            if (target_pin == nullptr) {
                candidates.push_back(RouteCandidate{net_name, root_terminal, target_terminal, GridPath{false, "target terminal 缺少全局 pin", {}, {}}});
                continue;
            }

            const GridPoint start = grid.snap_to_grid(root_pin->location, root_pin->layer);
            const GridPoint goal = grid.snap_to_grid(target_pin->location, target_pin->layer);
            std::set<std::string> seen_paths;
            int emitted = 0;
            GridPath first_failure;
            bool has_failure = false;
            for (const auto& astar_config : candidate_astar_configs()) {
                if (emitted >= config.max_candidates_per_pair) {
                    break;
                }
                GridPath path = find_astar_path(grid, obstacles, start, goal, astar_config);
                if (!path.success) {
                    if (!has_failure) {
                        first_failure = path;
                        has_failure = true;
                    }
                    continue;
                }

                const std::string signature = path_signature(path);
                if (seen_paths.insert(signature).second) {
                    candidates.push_back(RouteCandidate{net_name, root_terminal, target_terminal, path});
                    ++emitted;
                }
            }
            if (emitted == 0 && has_failure) {
                candidates.push_back(RouteCandidate{net_name, root_terminal, target_terminal, first_failure});
            }
        }
    }

    return candidates;
}

}  // namespace analog_sapr
