// 实现当前可运行的链式布局与 Manhattan 布线基线求解器。
#include "sapr/optimizer.hpp"

#include <stdexcept>

#include "sapr/constraints.hpp"
#include "sapr/router.hpp"
#include "sapr/tree.hpp"

namespace sapr {
namespace {

// 按链式树顺序进行确定性的逐行 packing。
std::unordered_map<std::string, Placement> pack_chain(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config,
    std::vector<std::string>& order) {
    std::unordered_map<std::string, Placement> placements;
    double x = 0.0;
    double y = 0.0;
    double row_height = 0.0;
    auto module_id = tree.root;
    while (module_id.has_value()) {
        const auto& module = circuit.modules.at(*module_id);
        if (x > 0.0 && x + module.width > config.row_width) {
            x = 0.0;
            y += row_height + config.spacing;
            row_height = 0.0;
        }
        placements[*module_id] = {*module_id, x, y, 0, "R0"};
        order.push_back(*module_id);
        x += module.width + config.spacing;
        row_height = std::max(row_height, module.height);
        module_id = tree.nodes.at(*module_id).left;
    }
    return placements;
}

}  // namespace

// 使用链式 packing 和 Manhattan routing 生成基线解。
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config) {
    const auto errors = validate_circuit(circuit);
    if (!errors.empty()) {
        std::string message = "invalid circuit:";
        for (const auto& error : errors) message += "\n- " + error;
        throw std::runtime_error(message);
    }
    const auto tree = make_chain_tree(circuit);
    Solution solution;
    solution.placements = pack_chain(circuit, tree, config, solution.placement_order);
    solution.routes = route_manhattan(circuit, solution.placements);
    return solution;
}

}  // namespace sapr

