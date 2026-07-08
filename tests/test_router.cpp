// 验证几何变换、增强 B*-tree、ASF 约束、contour packing 和 routing adapter 的核心行为。
#include "test_support.hpp"

#include <filesystem>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sapr/geometry.hpp"
#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/router.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/tree.hpp"

namespace {

// 判断两个矩形是否有面积重叠。
bool overlaps(const sapr::Rect& left, const sapr::Rect& right) {
    return left.x1 < right.x2 && left.x2 > right.x1 && left.y1 < right.y2 && left.y2 > right.y1;
}

// 判断 LCP 是否满足论文要求的同 net 且至少两段约束。
bool valid_lcp_topology(const sapr::LinkingControlPoint& point) {
    if (point.segments.size() < 2) return false;
    const auto& net = point.segments.front().net;
    for (const auto& segment : point.segments) {
        if (segment.net != net) return false;
    }
    return true;
}

// 从 placement 生成模块包围盒。
sapr::Rect placement_box(const sapr::Circuit& circuit, const sapr::Placement& placement) {
    const auto size = sapr::placed_size(circuit.modules.at(placement.module), placement);
    return {placement.x, placement.y, placement.x + size.first, placement.y + size.second};
}

// 构造单器件多端同坐标 net，用于稳定验证自动 LCP leaf/root 语义。
sapr::Circuit make_same_location_lcp_circuit(int pin_count) {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{1.0, 1.0, 2.0, 2.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    sapr::Net net{"N", sapr::Priority::Normal, {}};
    for (int index = 0; index < pin_count; ++index) {
        const std::string pin = "P" + std::to_string(index);
        const std::string terminal = "M." + pin;
        circuit.pins.emplace(terminal, sapr::Pin{"M", pin, 5.0, 5.0, "M1"});
        circuit.pin_order.push_back(terminal);
        net.terminals.push_back(terminal);
    }
    circuit.nets.emplace("N", net);
    circuit.net_order.push_back("N");
    return circuit;
}

// 统计包含指定 token 的 LCP 数量。
int count_lcps_with_token(const sapr::RoutingEvaluationRequest& request, const std::string& token) {
    int count = 0;
    for (const auto& point : request.linking_points) {
        if (point.id.find(token) != std::string::npos) ++count;
    }
    return count;
}

// 判断自动 LCP 拓扑里是否存在指定 FLOW 的有向可达路径。
bool topology_has_flow_path(const sapr::RoutingEvaluationRequest& request, const std::string& out_pin, const std::string& in_pin) {
    std::unordered_map<std::string, std::vector<std::string>> adjacency;
    for (const auto& topology : request.net_topologies) {
        for (const auto& segment : topology.segments) adjacency[segment.from].push_back(segment.to);
    }
    std::vector<std::string> frontier{out_pin};
    std::unordered_set<std::string> visited;
    while (!frontier.empty()) {
        const auto current = frontier.back();
        frontier.pop_back();
        if (current == in_pin) return true;
        if (!visited.insert(current).second) continue;
        const auto next = adjacency.find(current);
        if (next != adjacency.end()) frontier.insert(frontier.end(), next->second.begin(), next->second.end());
    }
    return false;
}

}  // namespace

// 验证关键几何、增强树、对称 packing、扰动和求解结果保持稳定语义。
void run_router_tests() {
    const auto circuit = sapr::load_circuit(std::filesystem::path(SAPR_SOURCE_DIR) / "input");
    const auto& module = circuit.modules.at("M1");
    const auto& pin = circuit.pins.at("M1.G");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 0, "R0"}) == std::pair<double, double>{0.8, 1.5}, "R0 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 90, "R90"}) == std::pair<double, double>{1.5, 0.8}, "R90 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 180, "R180"}) == std::pair<double, double>{3.2, 1.5}, "R180 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 270, "R270"}) == std::pair<double, double>{1.5, 3.2}, "R270 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 0, "MY"}) == std::pair<double, double>{3.2, 1.5}, "MY pin transform failed");
    const auto routing_r90_pin =
        sapr::routing::transform_pin_to_global(module, pin, {"M1", 0, 0, 90, "R90"});
    require(approx(routing_r90_pin.x, 1.5) && approx(routing_r90_pin.y, 0.8), "routing R90 transform should match placement geometry");
    const auto routing_r90_bbox =
        sapr::routing::transform_module_bbox_to_global(module, {"M1", 0, 0, 90, "R90"});
    require(
        approx(routing_r90_bbox.x1, 0.0) && approx(routing_r90_bbox.y1, 0.0) &&
            approx(routing_r90_bbox.x2, 3.0) && approx(routing_r90_bbox.y2, 4.0),
        "routing R90 bbox should stay anchored like placement output");

    const auto chain = sapr::make_chain_tree(circuit);
    require(chain.root == std::optional<std::string>{"M1"}, "chain root should be M1");
    require(chain.nodes.at("M1").left == std::optional<std::string>{"M2"}, "chain left child should preserve order");
    require(chain.nodes.at("M1").right_space.id == "M1:right", "right space should be created");

    auto tree = sapr::make_enhanced_tree(circuit);
    require(sapr::is_valid_tree(tree), "enhanced tree should be valid");
    require(sapr::self_symmetry_on_rightmost_branch(tree), "self-symmetry module should stay on right-most branch");
    require(tree.nodes.contains("M1"), "symmetry representative should be in tree");
    require(!tree.nodes.contains("M2"), "symmetry mirror should be generated by ASF packing");
    require(!tree.symmetry_groups.empty(), "symmetry groups should be recorded");
    require(tree.symmetry_groups.front().space_group_bundle.spaces.size() == 2, "space node group should model mirrored right/top spaces");
    require(tree.symmetry_groups.front().space_cluster_bundle.spaces.size() == 4, "space node cluster should model four ASF spaces");
    require(sapr::collect_space_nodes(tree).size() >= tree.representative_order.size() * 2, "space nodes should be collectable");

    sapr::SpaceNode space{"space",
                          "M1",
                          sapr::SpaceNodeKind::Right,
                          {{"p",
                            "space",
                            {{"N", "A", "p", 1.0, 4.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:A->p"},
                             {"N", "p", "B", 1.0, 4.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:p->B"}},
                            {}}},
                          0.0,
                          {},
                          0.0};
    require(approx(space.required_space(), 4.0), "space formula should match paper equation");
    space.allocated_space = 9.0;
    require(approx(space.required_space(), 9.0), "feedback allocation should override smaller formula result");

    const sapr::SolverConfig config{5.0, 40.0, 7, 3};
    const auto request = sapr::pack_enhanced_tree(circuit, tree, config);
    require(request.placements.contains("M1"), "representative placement should exist");
    require(request.placements.contains("M2"), "mirror placement should exist");
    require(request.placements.at("M2").orient == "MY", "vertical symmetry pair should mirror on Y axis");
    require(request.placements.at("M2").x > request.placements.at("M1").x, "mirror should be placed beside representative");
    require(request.placements.at("M5").module == "M5", "self-symmetry module should remain placeable");
    require(request.placed_pins.size() == circuit.pin_order.size(), "routing request should expose placed pins");
    require(!request.active_region_blockers.empty(), "routing request should expose blockers");
    require(!request.linking_points.empty(), "routing request should expose LCPs");
    require(!request.net_topologies.empty(), "routing request should expose net topology records");
    for (const auto& point : request.linking_points) {
        require(point.segments.size() >= 2, "each LCP should connect at least two segments");
        require(point.location_candidates.size() >= 3, "each LCP should expose multiple physical candidates");
    }

    std::vector<sapr::Rect> boxes;
    for (const auto& module_id : request.placement_order) boxes.push_back(placement_box(circuit, request.placements.at(module_id)));
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        for (std::size_t j = i + 1; j < boxes.size(); ++j) require(!overlaps(boxes[i], boxes[j]), "contour packing should avoid overlap");
    }

    auto feedback_tree = tree;
    sapr::RoutingFeedback feedback;
    feedback.required_space_by_node["M1:right"] = 25.0;
    sapr::apply_routing_feedback(feedback_tree, feedback);
    const auto expanded = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    require(expanded.placements.at("M3").x > request.placements.at("M3").x, "routing feedback should affect next packing");

    auto perturbed = tree;
    std::mt19937 rng(3);
    for (int i = 0; i < 10; ++i) {
        sapr::perturb_placement_tree(perturbed, rng);
        require(sapr::is_valid_tree(perturbed), "perturbed tree should stay valid");
        require(sapr::self_symmetry_on_rightmost_branch(perturbed), "perturbation should preserve ASF self branch");
        for (const auto& space_node : sapr::collect_space_nodes(perturbed)) {
            for (const auto& point : space_node.linking_points) {
                require(valid_lcp_topology(point), "LCP perturbation should preserve same-net topology");
            }
        }
    }

    const auto solution = sapr::solve_placement_aware(circuit, config);
    require(solution.metrics.has_value(), "solution should carry routing metric snapshot");
    require(solution.routing_candidate_count.value_or(0) > circuit.net_order.size(), "routing should evaluate topology candidates");
    for (const auto& route : solution.routes) {
        require(route.x1 != route.x2 || route.y1 != route.y2, "zero-length route should be filtered");
    }
    require(solution.placements.size() == circuit.modules.size(), "all modules should be placed");
    require(approx(sapr::default_width(circuit, "VDD"), 5.0), "constrained width should use midpoint");
    require(approx(sapr::default_width(circuit, "OUT"), 1.0), "unconstrained width should be one");

    const auto repeat = sapr::solve_placement_aware(circuit, config);
    require(solution.placement_order == repeat.placement_order, "fixed seed should preserve placement order");
    for (const auto& id : solution.placement_order) {
        require(approx(solution.placements.at(id).x, repeat.placements.at(id).x), "fixed seed should preserve x");
        require(approx(solution.placements.at(id).y, repeat.placements.at(id).y), "fixed seed should preserve y");
        require(solution.placements.at(id).orient == repeat.placements.at(id).orient, "fixed seed should preserve orient");
    }

    const auto four_pin_circuit = make_same_location_lcp_circuit(4);
    const auto four_pin_tree = sapr::make_enhanced_tree(four_pin_circuit);
    const auto four_pin_request = sapr::pack_enhanced_tree(four_pin_circuit, four_pin_tree, config);
    require(count_lcps_with_token(four_pin_request, ":leaf:") == 2, "4-pin net should create two leaf LCPs");
    require(count_lcps_with_token(four_pin_request, ":root") == 1, "4-pin net should create one root LCP");
    require(four_pin_request.linking_points.size() == 3, "4-pin actual LCP count should include root");
    bool has_lcp_to_lcp = false;
    for (const auto& topology : four_pin_request.net_topologies) {
        for (const auto& segment : topology.segments) {
            if (segment.from.find(":leaf:") != std::string::npos && segment.to.find(":root") != std::string::npos) {
                has_lcp_to_lcp = true;
            }
        }
    }
    require(has_lcp_to_lcp, "root topology should include leaf-to-root LCP segment");
    for (const auto& point : four_pin_request.linking_points) {
        require(point.location_candidates.size() >= 3, "automatic LCP should expose at least three candidates");
    }

    auto flow_lcp_circuit = make_same_location_lcp_circuit(4);
    flow_lcp_circuit.constraints.flows.push_back({"N", "M.P0", "M.P3"});
    const auto flow_lcp_tree = sapr::make_enhanced_tree(flow_lcp_circuit);
    const auto flow_lcp_request = sapr::pack_enhanced_tree(flow_lcp_circuit, flow_lcp_tree, config);
    require(
        topology_has_flow_path(flow_lcp_request, "M.P0", "M.P3"),
        "automatic LCP topology should preserve FLOW reachability from source to sink");

    auto blocked_lcp_circuit = make_same_location_lcp_circuit(4);
    blocked_lcp_circuit.modules.at("M").active = {0.0, 0.0, 10.0, 10.0};
    const auto blocked_lcp_tree = sapr::make_enhanced_tree(blocked_lcp_circuit);
    const auto blocked_lcp_request = sapr::pack_enhanced_tree(blocked_lcp_circuit, blocked_lcp_tree, config);
    for (const auto& point : blocked_lcp_request.linking_points) {
        for (const auto& candidate : point.location_candidates) {
            require(
                candidate.validity_level != "strict",
                "active-blocked LCP candidates should not be classified as strict");
        }
    }

    const auto seven_pin_circuit = make_same_location_lcp_circuit(7);
    const auto seven_pin_tree = sapr::make_enhanced_tree(seven_pin_circuit);
    const auto seven_pin_request = sapr::pack_enhanced_tree(seven_pin_circuit, seven_pin_tree, config);
    require(count_lcps_with_token(seven_pin_request, ":leaf:") == 3, "7-pin same-location net should split into 3 balanced leaves");
    for (const auto& point : seven_pin_request.linking_points) {
        if (point.id.find(":leaf:") == std::string::npos) continue;
        require(point.segments.size() <= 3, "leaf cluster should respect max pins per leaf");
    }

    const auto metrics = sapr::measure(circuit, solution);
    require(metrics.area > 0.0, "placement-aware area should be positive");
    require(solution.metrics->wirelength > 0.0, "placement-aware routing metric wirelength should be positive");
    require(solution.metrics->penalty >= 0.0, "adapter penalty should be non-negative");
    require(solution.metrics->routing_failure_penalty <= solution.metrics->penalty, "routing failure penalty should be part of total penalty");
}
