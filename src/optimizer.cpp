// 实现论文 placement-aware 增强 B*-tree packing、routing adapter 和模拟退火主循环。
#include "sapr/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_set>

#include "sapr/constraints.hpp"
#include "sapr/geometry.hpp"
#include "sapr/router.hpp"
#include "sapr/tree.hpp"

namespace sapr {
namespace {

// 表示候选解在模拟退火中的完整评价。
struct CandidateState {
    EnhancedBStarTree tree;
    RoutingEvaluationRequest request;
    RoutingFeedback feedback;
    double cost{};
};

// 返回指定模块是否是对称 pair 的代表模块。
const SymmetryGroupNode* symmetry_pair_for_representative(const EnhancedBStarTree& tree, const std::string& module) {
    for (const auto& group : tree.symmetry_groups) {
        if (!group.self_symmetric && group.representative == module) return &group;
    }
    return nullptr;
}

// 返回模块旋转后的尺寸。
std::pair<double, double> module_size(const Circuit& circuit, const std::string& module, int angle) {
    return placed_size(circuit.modules.at(module), {module, 0.0, 0.0, angle, "R0"});
}

// 返回代表节点在 packing 中占用的整体尺寸，包含对称镜像模块和轴间预留空间。
std::pair<double, double> occupied_size(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const BStarNode& node,
    const SolverConfig& config) {
    auto size = module_size(circuit, node.module, node.angle);
    const auto* group = symmetry_pair_for_representative(tree, node.module);
    if (group == nullptr || !group->mirror.has_value()) return size;
    const auto mirror_size = module_size(circuit, *group->mirror, node.angle);
    const double gap = std::max(config.spacing, node.right_space.required_space() + group->space_cluster.required_space());
    if (group->axis == Axis::Vertical) return {size.first + gap + mirror_size.first, std::max(size.second, mirror_size.second)};
    return {std::max(size.first, mirror_size.first), size.second + gap + mirror_size.second};
}

// 将 angle 转成基础 Cadence orient 字符串。
std::string orient_for_angle(int angle) {
    switch ((angle % 360 + 360) % 360) {
        case 0: return "R0";
        case 90: return "R90";
        case 180: return "R180";
        case 270: return "R270";
    }
    return "R0";
}

// 生成对称 pair 中镜像模块的 placement。
Placement mirror_placement(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const BStarNode& node,
    const Placement& representative,
    const SolverConfig& config) {
    const auto* group = symmetry_pair_for_representative(tree, node.module);
    if (group == nullptr || !group->mirror.has_value()) return representative;
    const auto rep_size = module_size(circuit, node.module, representative.angle);
    const double gap = std::max(config.spacing, node.right_space.required_space() + group->space_cluster.required_space());
    Placement mirror{*group->mirror, representative.x, representative.y, representative.angle, representative.orient};
    if (group->axis == Axis::Vertical) {
        mirror.x = representative.x + rep_size.first + gap;
        mirror.orient = "MY";
    } else {
        mirror.y = representative.y + rep_size.second + gap;
        mirror.orient = "MX";
    }
    return mirror;
}

// 按原始模块顺序整理输出顺序，保证文件稳定。
std::vector<std::string> ordered_placements(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    std::vector<std::string> order;
    for (const auto& module : circuit.module_order) {
        if (request.placements.contains(module)) order.push_back(module);
    }
    return order;
}

// 计算当前 metrics 的论文归一化总代价。
double normalized_cost(const Metrics& metrics, const Metrics& base, const SolverConfig& config) {
    const double area = metrics.area / std::max(base.area, 1.0);
    const double wire = metrics.wirelength / std::max(base.wirelength, 1.0);
    const double bend = static_cast<double>(metrics.bend_count) / std::max(base.bend_count, 1);
    const double via = static_cast<double>(metrics.via_count) / std::max(base.via_count, 1);
    return config.area_weight * area + config.wirelength_weight * wire + config.bend_weight * bend +
           config.via_weight * via + metrics.penalty;
}

// 评价一棵增强 B*-tree 对应的候选状态。
CandidateState evaluate_candidate(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config,
    const Metrics& base_metrics) {
    CandidateState state;
    state.tree = tree;
    state.request = pack_enhanced_tree(circuit, state.tree, config);
    state.feedback = evaluate_with_routing_adapter(circuit, state.request);
    state.cost = normalized_cost(state.feedback.metrics, base_metrics, config);
    return state;
}

// 将候选状态转换为最终 Solution。
Solution make_solution(const CandidateState& state) {
    Solution solution;
    solution.placements = state.request.placements;
    solution.placement_order = state.request.placement_order;
    solution.routes = state.feedback.routes;
    return solution;
}

}  // namespace

// 按增强 B*-tree 和 ASF 对称组生成当前候选布局。
RoutingEvaluationRequest pack_enhanced_tree(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config) {
    RoutingEvaluationRequest request;
    if (!tree.root.has_value()) return request;

    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, double, double)> place_node = [&](const std::string& id, double x, double y) {
        if (visited.contains(id)) return;
        visited.insert(id);
        const auto& node = tree.nodes.at(id);
        Placement placement{id, x, y, node.angle, orient_for_angle(node.angle)};
        request.placements[id] = placement;

        const auto* pair_group = symmetry_pair_for_representative(tree, id);
        if (pair_group != nullptr && pair_group->mirror.has_value()) {
            request.placements[*pair_group->mirror] = mirror_placement(circuit, tree, node, placement, config);
        }

        const auto occupied = occupied_size(circuit, tree, node, config);
        if (node.left.has_value()) {
            place_node(*node.left, x + occupied.first + node.right_space.required_space() + config.spacing, y);
        }
        if (node.right.has_value()) {
            place_node(*node.right, x, y + occupied.second + node.top_space.required_space() + config.spacing);
        }
    };

    place_node(*tree.root, 0.0, 0.0);
    request.placement_order = ordered_placements(circuit, request);
    request.space_nodes = collect_space_nodes(tree);
    return request;
}

// 使用 routing adapter 评价当前 placement candidate。
RoutingFeedback evaluate_with_routing_adapter(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    RoutingFeedback feedback;
    feedback.routes = route_manhattan(circuit, request.placements);
    Solution solution{request.placements, request.placement_order, feedback.routes};
    feedback.metrics = measure(circuit, solution);
    for (const auto& space : request.space_nodes) feedback.required_space_by_node[space.id] = space.required_space();
    return feedback;
}

// 使用论文 placement 框架、SA 和 routing adapter 生成布局布线解。
Solution solve_placement_aware(const Circuit& circuit, const SolverConfig& config) {
    const auto errors = validate_circuit(circuit);
    if (!errors.empty()) {
        std::string message = "invalid circuit:";
        for (const auto& error : errors) message += "\n- " + error;
        throw std::runtime_error(message);
    }

    const auto initial_tree = make_enhanced_tree(circuit);
    auto initial_request = pack_enhanced_tree(circuit, initial_tree, config);
    auto initial_feedback = evaluate_with_routing_adapter(circuit, initial_request);
    const Metrics base_metrics = initial_feedback.metrics;
    CandidateState current{initial_tree,
                           std::move(initial_request),
                           std::move(initial_feedback),
                           normalized_cost(base_metrics, base_metrics, config)};
    CandidateState best = current;

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    double temperature = config.initial_temperature;
    for (int iteration = 0; iteration < config.sa_iterations; ++iteration) {
        auto next_tree = current.tree;
        perturb_placement_tree(next_tree, rng);
        auto next = evaluate_candidate(circuit, next_tree, config, base_metrics);
        const double delta = next.cost - current.cost;
        const bool accept = delta <= 0.0 || probability(rng) < std::exp(-delta / std::max(temperature, 1e-9));
        if (accept) current = std::move(next);
        if (current.cost < best.cost) best = current;
        temperature *= config.cooling_rate;
    }

    return make_solution(best);
}

// 保持历史 CLI/API 名称，当前默认指向论文 placement-aware 求解流程。
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config) {
    return solve_placement_aware(circuit, config);
}

}  // namespace sapr
