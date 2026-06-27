// 实现论文 placement-aware 增强 B*-tree contour packing、routing adapter 和模拟退火主循环。
#include "sapr/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "sapr/constraints.hpp"
#include "sapr/geometry.hpp"
#include "sapr/router.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
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

// 表示 contour packing 已占用的 group bounding box。
struct PackedRect {
    double x1{};
    double y1{};
    double x2{};
    double y2{};
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
    const double right_space = node.right_space.required_space();
    const double top_space = node.top_space.required_space();
    const auto* group = symmetry_pair_for_representative(tree, node.module);
    if (group == nullptr || !group->mirror.has_value()) return {size.first + right_space, size.second + top_space};

    const auto mirror_size = module_size(circuit, *group->mirror, node.angle);
    const double gap = std::max(config.spacing, right_space + group->space_cluster.required_space());
    if (group->axis == Axis::Vertical) {
        return {size.first + gap + mirror_size.first + group->space_group.required_space(),
                std::max(size.second, mirror_size.second) + top_space};
    }
    return {std::max(size.first, mirror_size.first) + right_space,
            size.second + gap + mirror_size.second + group->space_group.required_space()};
}

// 返回 contour 在指定 x 区间内的最高 y。
double contour_height(const std::vector<PackedRect>& packed, double x, double width) {
    double result = 0.0;
    const double x2 = x + width;
    for (const auto& rect : packed) {
        if (x < rect.x2 && x2 > rect.x1) result = std::max(result, rect.y2);
    }
    return result;
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

// 返回已放置模块的 active blocker 近似全局矩形。
Rect placed_active_rect(const Module& module, const Placement& placement) {
    return routing::transform_active_to_global(module, placement);
}

// 填充 routing request 中面向 DP/A* 的全局 pin、blocker 和 LCP 候选位置。
void populate_routing_context(const Circuit& circuit, RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::pair<double, double>> module_centers;
    for (const auto& module_id : request.placement_order) {
        const auto& placement = request.placements.at(module_id);
        const auto size = placed_size(circuit.modules.at(module_id), placement);
        module_centers[module_id] = {placement.x + size.first / 2.0, placement.y + size.second / 2.0};
        request.active_region_blockers.push_back(placed_active_rect(circuit.modules.at(module_id), placement));
    }
    for (const auto& key : circuit.pin_order) {
        const auto& pin = circuit.pins.at(key);
        const auto& placement = request.placements.at(pin.module);
        const auto xy = placed_pin(circuit.modules.at(pin.module), pin, placement);
        request.placed_pins.push_back({key, pin.module, pin.name, xy.first, xy.second, pin.layer});
    }
    for (auto space : request.space_nodes) {
        const auto center = module_centers.contains(space.owner) ? module_centers.at(space.owner) : std::pair<double, double>{0.0, 0.0};
        const double offset = std::max(space.required_space(), 1.0);
        space.location_candidates.clear();
        space.location_candidates.push_back({center.first + offset, center.second, space.id + ":edge"});
        space.location_candidates.push_back({center.first, center.second + offset, space.id + ":top"});
        space.location_candidates.push_back({center.first + offset / 2.0, center.second + offset / 2.0, space.id + ":center"});
        for (auto point : space.linking_points) {
            point.location_candidates.clear();
            if (space.kind == SpaceNodeKind::Top) {
                point.location_candidates.push_back({center.first, center.second + offset, point.id + ":top"});
            } else if (space.kind == SpaceNodeKind::Cluster) {
                point.location_candidates.push_back({center.first + offset / 2.0, center.second, point.id + ":axis"});
            } else {
                point.location_candidates.push_back({center.first + offset, center.second, point.id + ":right"});
            }
            point.location_candidates.push_back({center.first + offset / 2.0, center.second + offset / 2.0, point.id + ":center"});
            point.location_candidates.push_back({center.first, center.second + offset, point.id + ":pin_projection"});
            request.linking_points.push_back(std::move(point));
        }
    }
    std::unordered_map<std::string, NetTopology> topologies;
    for (const auto& [name, net] : circuit.nets) {
        topologies[name].net = name;
        topologies[name].pins = net.terminals;
    }
    for (const auto& point : request.linking_points) {
        for (const auto& segment : point.segments) {
            auto& topology = topologies[segment.net];
            topology.net = segment.net;
            topology.linking_points.push_back(point);
            topology.segments.push_back(segment);
        }
    }
    for (auto& [_, topology] : topologies) {
        if (!topology.segments.empty()) request.net_topologies.push_back(std::move(topology));
    }
}

// 计算当前 metrics 的论文归一化总代价。
double compute_phi_cost(Metrics& metrics, const Metrics& base, const SolverConfig& config) {
    metrics.normalized_area = metrics.area / std::max(base.area, 1.0);
    metrics.normalized_wirelength = metrics.wirelength / std::max(base.wirelength, 1.0);
    metrics.normalized_bend = static_cast<double>(metrics.bend_count) / std::max(base.bend_count, 1);
    metrics.normalized_via = static_cast<double>(metrics.via_count) / std::max(base.via_count, 1);
    metrics.phi_cost = config.area_weight * metrics.normalized_area +
                       config.wirelength_weight * metrics.normalized_wirelength +
                       config.bend_weight * metrics.normalized_bend +
                       config.via_weight * metrics.normalized_via +
                       metrics.penalty;
    return metrics.phi_cost;
}

// 评价一棵增强 B*-tree 对应的候选状态。
CandidateState evaluate_candidate(
    const Circuit& circuit,
    EnhancedBStarTree tree,
    const SolverConfig& config,
    const Metrics& base_metrics) {
    CandidateState state;
    state.tree = std::move(tree);
    state.request = pack_enhanced_tree(circuit, state.tree, config);
    state.feedback = evaluate_with_routing_adapter(circuit, state.request);
    apply_routing_feedback(state.tree, state.feedback);
    state.cost = compute_phi_cost(state.feedback.metrics, base_metrics, config);
    return state;
}

// 将候选状态转换为最终 Solution。
Solution make_solution(const CandidateState& state) {
    Solution solution;
    solution.placements = state.request.placements;
    solution.placement_order = state.request.placement_order;
    solution.routes = state.feedback.routes;
    solution.metrics = state.feedback.metrics;
    solution.routing_cost = state.feedback.routing_cost;
    solution.routing_candidate_count = state.feedback.routing_candidate_count;
    return solution;
}

// 检查 FLOW 约束是否在当前临时路由中有可追踪的端点。
int count_flow_violations(const Circuit& circuit, const RoutingEvaluationRequest& request, const std::vector<RouteSegment>& routes) {
    std::unordered_map<std::string, PlacedPin> pins;
    for (const auto& pin : request.placed_pins) pins[pin.key] = pin;
    int violations = 0;
    for (const auto& flow : circuit.constraints.flows) {
        if (!pins.contains(flow.out_pin) || !pins.contains(flow.in_pin)) {
            ++violations;
            continue;
        }
        bool has_route = false;
        for (const auto& route : routes) {
            if (route.net == flow.net) {
                has_route = true;
                break;
            }
        }
        if (!has_route) ++violations;
    }
    return violations;
}

// 检查当前 route 是否违反线宽范围。
int count_current_density_violations(const Circuit& circuit, const std::vector<RouteSegment>& routes) {
    int violations = 0;
    for (const auto& route : routes) {
        const auto found = circuit.constraints.wire_widths.find(route.net);
        if (found == circuit.constraints.wire_widths.end()) continue;
        if (route.width < found->second.min_width || route.width > found->second.max_width) ++violations;
    }
    return violations;
}

}  // namespace

// 按增强 B*-tree 和 ASF 对称组生成当前候选布局。
RoutingEvaluationRequest pack_enhanced_tree(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config) {
    RoutingEvaluationRequest request;
    if (!tree.root.has_value()) return request;

    std::vector<PackedRect> packed;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, double, double)> place_node = [&](const std::string& id, double desired_x, double desired_y) {
        if (visited.contains(id)) return;
        visited.insert(id);
        const auto& node = tree.nodes.at(id);
        const auto occupied = occupied_size(circuit, tree, node, config);
        const double x = desired_x;
        const double y = std::max(desired_y, contour_height(packed, x, occupied.first));

        Placement placement{id, x, y, node.angle, orient_for_angle(node.angle)};
        request.placements[id] = placement;
        const auto* pair_group = symmetry_pair_for_representative(tree, id);
        if (pair_group != nullptr && pair_group->mirror.has_value()) {
            request.placements[*pair_group->mirror] = mirror_placement(circuit, tree, node, placement, config);
        }

        packed.push_back({x, y, x + occupied.first, y + occupied.second});
        if (node.left.has_value()) {
            place_node(*node.left, x + occupied.first + config.spacing, y);
        }
        if (node.right.has_value()) {
            place_node(*node.right, x, y + occupied.second + config.spacing);
        }
    };

    place_node(*tree.root, 0.0, 0.0);
    request.placement_order = ordered_placements(circuit, request);
    request.space_nodes = collect_space_nodes(tree);
    populate_routing_context(circuit, request);
    return request;
}

// 使用 routing adapter 评价当前 placement candidate。
RoutingFeedback evaluate_with_routing_adapter(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    RoutingFeedback feedback;
    const auto routing_evaluation = evaluate_routing(circuit, request);
    const auto detailed = run_detailed_routing(circuit, request, routing_evaluation);
    feedback.routes = detailed.routes;
    if (feedback.routes.empty() && routing_evaluation.failed_nets > 0) {
        feedback.routes = selected_candidates_to_segments(routing_evaluation);
    }
    if (feedback.routes.empty() && routing_evaluation.failed_nets > 0) {
        feedback.routes = route_manhattan(circuit, request.placements);
    }
    Solution solution;
    solution.placements = request.placements;
    solution.placement_order = request.placement_order;
    solution.routes = feedback.routes;
    feedback.metrics = measure(circuit, solution);
    feedback.metrics.wirelength = routing_evaluation.global_routing.total_metrics.wirelength;
    feedback.metrics.bend_count = routing_evaluation.global_routing.total_metrics.bend_count;
    feedback.metrics.via_count = routing_evaluation.global_routing.total_metrics.via_count;
    feedback.metrics.flow_violations = count_flow_violations(circuit, request, feedback.routes);
    feedback.metrics.current_density_violations = count_current_density_violations(circuit, feedback.routes);
    feedback.metrics.flow_violations += detailed.flow_violations;
    feedback.metrics.current_density_violations += detailed.current_density_violations;
    feedback.metrics.design_rule_violations = detailed.design_rule_violations;
    feedback.metrics.design_rule_penalty = detailed.design_rule_penalty;
    feedback.metrics.routing_failures = routing_evaluation.failed_nets;
    feedback.metrics.congestion_penalty = 0.0;
    feedback.metrics.flow_penalty = routing_evaluation.global_routing.flow_penalty;
    feedback.metrics.current_density_penalty = routing_evaluation.global_routing.current_density_penalty;
    feedback.metrics.coupling_penalty = routing_evaluation.global_routing.coupling_penalty + detailed.coupling_penalty;
    feedback.metrics.routing_failure_penalty = routing_evaluation.global_routing.routing_failure_penalty;
    feedback.metrics.penalty += routing_evaluation.global_routing.total_penalty + detailed.design_rule_penalty + detailed.coupling_penalty;
    feedback.routing_cost = routing_evaluation.routing_cost;
    feedback.routing_candidate_count = routing_evaluation.candidates.size();

    for (const auto& space : request.space_nodes) {
        feedback.required_space_by_node[space.id] = space.required_space();
        feedback.coupling_space_by_node[space.id] = routing_evaluation.global_routing.coupling_penalty > 0.0 ? 1.0 : 0.0;
    }
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

    auto initial_tree = make_enhanced_tree(circuit);
    auto initial_request = pack_enhanced_tree(circuit, initial_tree, config);
    auto initial_feedback = evaluate_with_routing_adapter(circuit, initial_request);
    apply_routing_feedback(initial_tree, initial_feedback);
    const Metrics base_metrics = initial_feedback.metrics;
    const double initial_cost = compute_phi_cost(initial_feedback.metrics, base_metrics, config);
    CandidateState current{initial_tree,
                           std::move(initial_request),
                           std::move(initial_feedback),
                           initial_cost};
    CandidateState best = current;

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    double temperature = config.initial_temperature;
    for (int iteration = 0; iteration < config.sa_iterations; ++iteration) {
        auto next_tree = current.tree;
        perturb_placement_tree(next_tree, rng);
        auto next = evaluate_candidate(circuit, std::move(next_tree), config, base_metrics);
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
