// 文件职责：实现从 net terminal 生成多条 A* 候选路径的逻辑。
#include "sapr/routing/topology.hpp"

#include <set>
#include <sstream>
#include <string>
#include <utility>

#include "sapr/routing/astar.hpp"

namespace sapr::routing {
namespace {

// 将路径编码为字符串，用于候选路径去重。
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
        AStarConfig{1.0, 3.0, 5.0, 1.0, 20000},
        AStarConfig{1.0, 8.0, 5.0, 1.0, 20000},
        AStarConfig{1.0, 3.0, 12.0, 1.0, 20000},
        AStarConfig{1.0, 1.0, 3.0, 1.0, 20000},
    };
}

// 从 RoutingContext 的全局 pin 表中查找 terminal。
const GlobalPin* find_global_pin(const RoutingContext& context, const std::string& terminal) {
    const auto pin_it = context.global_pins().find(terminal);
    if (pin_it == context.global_pins().end()) {
        return nullptr;
    }
    return &pin_it->second;
}

// 返回稳定的 net 遍历顺序，优先使用输入文件顺序。
std::vector<std::string> ordered_net_names(const Circuit& circuit) {
    if (!circuit.net_order.empty()) {
        return circuit.net_order;
    }
    std::vector<std::string> names;
    names.reserve(circuit.nets.size());
    for (const auto& [name, _] : circuit.nets) {
        names.push_back(name);
    }
    return names;
}

}  // namespace

// 为每条 net 生成 root terminal 到其它 terminal 的 A* 候选路径。
std::vector<RouteCandidate> generate_route_candidates(
    const Circuit& circuit,
    const RoutingContext& context,
    const CandidateConfig& config) {
    std::vector<RouteCandidate> candidates;
    const auto& grid = context.grid();
    const auto& obstacles = context.obstacles();

    for (const auto& net_name : ordered_net_names(circuit)) {
        const auto net_it = circuit.nets.find(net_name);
        if (net_it == circuit.nets.end()) {
            continue;
        }
        const Net& net = net_it->second;
        if (net.terminals.size() < 2) {
            continue;
        }

        const std::string& root_terminal = net.terminals.front();
        const GlobalPin* root_pin = find_global_pin(context, root_terminal);
        if (root_pin == nullptr) {
            RouteCandidate candidate;
            candidate.net = net_name;
            candidate.from_terminal = root_terminal;
            candidate.path = GridPath{false, "root terminal has no global pin", {}, {}};
            candidate.segment_id = net_name + ":" + root_terminal + "->";
            candidate.current_density_ok = false;
            candidates.push_back(std::move(candidate));
            continue;
        }

        for (std::size_t terminal_index = 1; terminal_index < net.terminals.size(); ++terminal_index) {
            const std::string& target_terminal = net.terminals[terminal_index];
            const GlobalPin* target_pin = find_global_pin(context, target_terminal);
            if (target_pin == nullptr) {
                RouteCandidate candidate;
                candidate.net = net_name;
                candidate.from_terminal = root_terminal;
                candidate.to_terminal = target_terminal;
                candidate.path = GridPath{false, "target terminal has no global pin", {}, {}};
                candidate.segment_id = net_name + ":" + root_terminal + "->" + target_terminal;
                candidate.current_density_ok = false;
                candidates.push_back(std::move(candidate));
                continue;
            }

            const GridPoint start = grid.snap_to_grid(root_pin->location, root_pin->layer);
            const GridPoint goal = grid.snap_to_grid(target_pin->location, target_pin->layer);
            const double wire_width = context.default_width_for_net(net_name);
            std::set<std::string> seen_paths;
            int emitted = 0;
            GridPath first_failure;
            bool has_failure = false;
            for (auto astar_config : candidate_astar_configs()) {
                if (emitted >= config.max_candidates_per_pair) {
                    break;
                }
                astar_config.wire_width = wire_width;
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
                    RouteCandidate candidate;
                    candidate.net = net_name;
                    candidate.from_terminal = root_terminal;
                    candidate.to_terminal = target_terminal;
                    candidate.path = std::move(path);
                    candidate.segment_id = net_name + ":" + root_terminal + "->" + target_terminal;
                    candidate.wire_width = wire_width;
                    candidate.current_density_ok = path.success;
                    candidates.push_back(candidate);
                    ++emitted;
                }
            }
            if (emitted == 0 && has_failure) {
                RouteCandidate candidate;
                candidate.net = net_name;
                candidate.from_terminal = root_terminal;
                candidate.to_terminal = target_terminal;
                candidate.path = std::move(first_failure);
                candidate.segment_id = net_name + ":" + root_terminal + "->" + target_terminal;
                candidate.wire_width = wire_width;
                candidate.current_density_ok = false;
                candidates.push_back(candidate);
            }
        }
    }

    return candidates;
}

}  // namespace sapr::routing
