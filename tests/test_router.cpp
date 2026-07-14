// 验证几何变换、增强 B*-tree、ASF 约束、contour packing 和 routing adapter 的核心行为。
#include "test_support.hpp"

#include <algorithm>
#include <array>
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
    std::unordered_set<std::string> segment_ids;
    for (const auto& segment : point.segments) {
        if (segment.net != net) return false;
        if (segment.from != point.id && segment.to != point.id) return false;
        const std::string key = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
        if (!segment_ids.insert(key).second) return false;
    }
    return true;
}

// 验证所有 LCP 的 id 唯一，且被删除 LCP 不会残留在线段端点中。
bool lcp_references_are_consistent(const sapr::EnhancedBStarTree& tree, const std::string& removed_id = "") {
    std::unordered_set<std::string> point_ids;
    for (const auto& space : sapr::collect_space_nodes(tree)) {
        for (const auto& point : space.linking_points) {
            if (!point_ids.insert(point.id).second || !valid_lcp_topology(point)) return false;
            for (const auto& segment : point.segments) {
                if (!removed_id.empty() && (segment.from == removed_id || segment.to == removed_id)) return false;
            }
        }
    }
    return true;
}

// 构造包含一个四线段 LCP 的树，用于确定性覆盖 split 的端点重连语义。
sapr::EnhancedBStarTree make_split_test_tree(const sapr::Circuit& circuit) {
    auto tree = sapr::make_enhanced_tree(circuit);
    auto& asf_tree = tree.symmetry_groups.front().asf_bstar_tree;
    auto& asf_node = asf_tree.nodes.at(asf_tree.representative_order.front());
    auto& space = asf_node.space_node_groups.front().spaces.front();
    space.linking_points = {{"S", "", {{"N", "A", "S", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:A->S"},
                                       {"N", "B", "S", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:B->S"},
                                       {"N", "S", "C", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:S->C"},
                                       {"N", "S", "D", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:S->D"}},
                              {}}};
    space.linking_points.front().space_node_id = space.id;
    return tree;
}

// 构造两个同网 LCP 的树，用于确定性覆盖 merge 的全局端点重连语义。
sapr::EnhancedBStarTree make_merge_test_tree(const sapr::Circuit& circuit) {
    auto tree = sapr::make_enhanced_tree(circuit);
    auto& asf_tree = tree.symmetry_groups.front().asf_bstar_tree;
    auto& asf_node = asf_tree.nodes.at(asf_tree.representative_order.front());
    auto& first_space = asf_node.space_node_groups.front().spaces.at(0);
    auto& second_space = asf_node.space_node_groups.front().spaces.at(1);
    first_space.linking_points = {{"A", "", {{"N", "P", "A", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:P->A"},
                                                        {"N", "A", "X", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:A->X"}},
                                               {}}};
    second_space.linking_points = {{"B", "", {{"N", "Q", "B", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:Q->B"},
                                                         {"N", "B", "Y", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:B->Y"}},
                                                {}}};
    first_space.linking_points.front().space_node_id = first_space.id;
    second_space.linking_points.front().space_node_id = second_space.id;
    return tree;
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

    const sapr::Module transform_module{"T", 4.0, 3.0, sapr::Rect{0.0, 0.0, 4.0, 3.0}, 0.0, 0.0, ""};
    const sapr::Pin transform_pin{"T", "P", 0.5, 0.75, "M1"};
    struct TransformExpectation {
        sapr::Placement placement;
        double x;
        double y;
        double width;
        double height;
    };
    const std::array<TransformExpectation, 8> transform_expectations = {{
        {{"T", 3.0, 5.0, 0, "R0"}, 3.5, 5.75, 4.0, 3.0},
        {{"T", 3.0, 5.0, 90, "R90"}, 5.25, 5.5, 3.0, 4.0},
        {{"T", 3.0, 5.0, 180, "R180"}, 6.5, 7.25, 4.0, 3.0},
        {{"T", 3.0, 5.0, 270, "R270"}, 3.75, 8.5, 3.0, 4.0},
        {{"T", 3.0, 5.0, 0, "MX"}, 3.5, 7.25, 4.0, 3.0},
        {{"T", 3.0, 5.0, 0, "MY"}, 6.5, 5.75, 4.0, 3.0},
        {{"T", 3.0, 5.0, 90, "MXR90"}, 3.75, 5.5, 3.0, 4.0},
        {{"T", 3.0, 5.0, 270, "MYR90"}, 5.25, 8.5, 3.0, 4.0},
    }};
    for (const auto& expected : transform_expectations) {
        const auto [placed_x, placed_y] = sapr::placed_pin(transform_module, transform_pin, expected.placement);
        const auto routing_point = sapr::routing::transform_pin_to_global(transform_module, transform_pin, expected.placement);
        const auto [placed_width, placed_height] = sapr::placed_size(transform_module, expected.placement);
        require(approx(placed_x, expected.x) && approx(placed_y, expected.y), "Cadence orient pin transform failed");
        require(approx(routing_point.x, expected.x) && approx(routing_point.y, expected.y), "routing transform must match placement geometry");
        require(approx(placed_width, expected.width) && approx(placed_height, expected.height), "Cadence orient size transform failed");
    }

    const auto chain = sapr::make_chain_tree(circuit);
    require(chain.root == std::optional<std::string>{"M1"}, "chain root should be M1");
    require(chain.nodes.at("M1").left == std::optional<std::string>{"M2"}, "chain left child should preserve order");
    require(chain.nodes.at("M1").right_space.id == "M1:right", "right space should be created");

    const auto no_symmetry_circuit =
        sapr::load_circuit(std::filesystem::path(SAPR_SOURCE_DIR) / "cases" / "4ring_2_left6_no_symmetry" / "input");
    auto no_symmetry_tree = sapr::make_enhanced_tree(no_symmetry_circuit);
    require(sapr::has_ordinary_right_child(no_symmetry_tree), "ordinary modules should keep two-dimensional right branches");
    sapr::initialize_lcp_topology(no_symmetry_circuit, no_symmetry_tree, sapr::SolverConfig{});
    const auto no_symmetry_request = sapr::pack_enhanced_tree(no_symmetry_circuit, no_symmetry_tree, sapr::SolverConfig{});
    const double first_y = no_symmetry_request.placements.at(no_symmetry_request.placement_order.front()).y;
    const bool has_second_row = std::any_of(
        no_symmetry_request.placement_order.begin(),
        no_symmetry_request.placement_order.end(),
        [&](const std::string& id) { return !approx(no_symmetry_request.placements.at(id).y, first_y); });
    require(has_second_row, "enhanced B*-tree packing should place ordinary modules in more than one row");
    // LCP segment 线宽必须跟 WIRE_WIDTH.min 一致，不能被默认 1um 抬高，否则 A* 膨胀会困死 pin access。
    require(!no_symmetry_request.linking_points.empty(), "4ring case should generate LCP topology");
    for (const auto& point : no_symmetry_request.linking_points) {
        for (const auto& segment : point.segments) {
            const auto width = no_symmetry_circuit.constraints.wire_widths.find(segment.net);
            require(width != no_symmetry_circuit.constraints.wire_widths.end(), "LCP segment net should have WIRE_WIDTH");
            require(approx(segment.min_width, width->second.min_width), "LCP segment min_width should match WIRE_WIDTH.min");
            require(approx(segment.max_width, width->second.max_width), "LCP segment max_width should match WIRE_WIDTH.max");
            require(segment.min_width + 1e-9 < 1.0, "LCP segment min_width must not be lifted to default 1um");
        }
    }

    const auto two_group_circuit = sapr::load_circuit(
        std::filesystem::path(SAPR_SOURCE_DIR) / "cases" / "4ring_2_complex_symmetry_perturbed_2groups" / "input");
    const auto two_group_tree = sapr::make_enhanced_tree(two_group_circuit);
    require(sapr::is_valid_tree(two_group_tree), "two hierarchy nodes should form a valid global tree");
    for (const auto& [id, node] : two_group_tree.nodes) {
        (void)id;
        require(!(node.left.has_value() && node.right.has_value() && node.left == node.right),
                "global tree must not attach the same child as both left and right");
    }
    for (const auto& group : two_group_tree.symmetry_groups) {
        require(!group.asf_bstar_tree.nodes.empty(), "multi-pair symmetry group should build a non-empty ASF tree");
        for (const auto& [id, node] : group.asf_bstar_tree.nodes) {
            (void)id;
            require(!(node.left.has_value() && node.right.has_value() && node.left == node.right),
                    "ASF tree must not attach the same child as both left and right");
        }
    }
    auto two_group_perturbed = two_group_tree;
    std::mt19937 two_group_rng(23);
    bool saw_asf_move = false;
    for (int i = 0; i < 50; ++i) {
        const auto report = sapr::perturb_placement_tree(two_group_perturbed, two_group_rng);
        saw_asf_move = saw_asf_move || report.move.starts_with("asf-module-");
        require(sapr::is_valid_tree(two_group_perturbed), "ASF internal perturbation should preserve tree legality");
        for (const auto& group : two_group_perturbed.symmetry_groups) {
            const auto& asf_tree = group.asf_bstar_tree;
            for (const auto& [representative, mirror] : asf_tree.mirror_map) {
                require(asf_tree.nodes.contains(representative), "ASF tree should keep representative nodes");
                require(!asf_tree.nodes.contains(mirror), "ASF tree must not directly contain mirror nodes");
            }
            std::unordered_set<std::string> right_most(
                asf_tree.right_most_branch.begin(),
                asf_tree.right_most_branch.end());
            for (const auto& [id, node] : asf_tree.nodes) {
                if (node.space_node_cluster.has_value()) {
                    require(right_most.contains(id), "space_node_cluster should only exist on the ASF right-most branch");
                }
                require(node.space_node_groups.size() == 2, "each ASF representative/self node should keep two space_node_group containers");
            }
        }
    }
    require(saw_asf_move, "SA perturbation pool should include ASF internal moves for hierarchy-heavy cases");

    auto tree = sapr::make_enhanced_tree(circuit);
    require(sapr::is_valid_tree(tree), "enhanced tree should be valid");
    require(sapr::self_symmetry_on_rightmost_branch(tree), "self-symmetry module should stay on right-most branch");
    require(tree.nodes.contains("sg1"), "symmetry group should enter global tree as a hierarchy node");
    require(tree.symmetry_groups.front().asf_bstar_tree.nodes.contains("M1"), "symmetry representative should be in ASF tree");
    require(!tree.nodes.contains("M2"), "symmetry mirror should be generated by ASF packing");
    require(!tree.symmetry_groups.empty(), "symmetry groups should be recorded");
    require(tree.symmetry_groups.front().asf_bstar_tree.nodes.at("M1").space_node_groups.size() == 2, "space node group should model mirrored right/top spaces");
    require(tree.symmetry_groups.front().asf_bstar_tree.nodes.at("M1").space_node_cluster->spaces.size() == 4, "space node cluster should model four ASF spaces");
    require(sapr::collect_space_nodes(tree).size() >= tree.representative_order.size() * 2, "space nodes should be collectable");

    struct MirrorExpectation {
        int angle;
        const char* orient;
    };
    const std::array<MirrorExpectation, 4> mirror_expectations = {{
        {0, "MY"},
        {90, "MXR90"},
        {180, "MX"},
        {270, "MYR90"},
    }};
    for (const auto& expected : mirror_expectations) {
        auto rotated_pair_tree = tree;
        rotated_pair_tree.symmetry_groups.front().asf_bstar_tree.nodes.at("M1").angle = expected.angle;
        const auto rotated_pair_request = sapr::pack_enhanced_tree(circuit, rotated_pair_tree, sapr::SolverConfig{});
        const auto& representative = rotated_pair_request.placements.at("M1");
        const auto& mirror = rotated_pair_request.placements.at("M2");
        const auto representative_box = placement_box(circuit, representative);
        const auto mirror_box = placement_box(circuit, mirror);
        const double axis_x = (representative_box.x1 + mirror_box.x2) / 2.0;
        require(mirror.orient == expected.orient, "ASF mirror orient should preserve representative rotation");
        require(
            approx(representative_box.x1, 2.0 * axis_x - mirror_box.x2) &&
                approx(representative_box.x2, 2.0 * axis_x - mirror_box.x1) &&
                approx(representative_box.y1, mirror_box.y1) && approx(representative_box.y2, mirror_box.y2),
            "ASF mirror bbox should stay vertically symmetric after rotation");
        const auto [representative_pin_x, representative_pin_y] =
            sapr::placed_pin(circuit.modules.at("M1"), circuit.pins.at("M1.G"), representative);
        const auto [mirror_pin_x, mirror_pin_y] =
            sapr::placed_pin(circuit.modules.at("M2"), circuit.pins.at("M2.G"), mirror);
        require(
            approx(representative_pin_x, 2.0 * axis_x - mirror_pin_x) && approx(representative_pin_y, mirror_pin_y),
            "ASF mirror pin should stay vertically symmetric after rotation");
    }

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
                          0.0,
                          {}};
    require(approx(space.formula_required_space(), 4.0), "space formula value should match paper equation before feedback");
    require(approx(space.required_space(), 4.0), "space formula should match paper equation");
    space.allocated_space = 9.0;
    require(approx(space.formula_required_space(), 4.0), "feedback allocation should not change formula value");
    require(approx(space.required_space(), 9.0), "feedback allocation should override smaller formula result");

    const sapr::SolverConfig config{5.0, 40.0, 7, 0};
    sapr::initialize_lcp_topology(circuit, tree, config);
    const auto request = sapr::pack_enhanced_tree(circuit, tree, config);
    require(request.placements.contains("M1"), "representative placement should exist");
    require(request.placements.contains("M2"), "mirror placement should exist");
    require(request.placements.at("M2").orient == "MY", "vertical symmetry pair should mirror on Y axis");
    require(request.placements.at("M2").x < request.placements.at("M1").x, "mirror should be placed on the left side of the representative");
    require(request.placements.at("M5").module == "M5", "self-symmetry module should remain placeable");
    require(request.placed_pins.size() == circuit.pin_order.size(), "routing request should expose placed pins");
    require(!request.active_region_blockers.empty(), "routing request should expose blockers");
    require(!request.linking_points.empty(), "routing request should expose LCPs");
    require(!request.net_topologies.empty(), "routing request should expose net topology records");
    const auto find_space = [&](const std::string& id) -> const sapr::SpaceNode* {
        for (const auto& candidate : request.space_nodes) {
            if (candidate.id == id) return &candidate;
        }
        return nullptr;
    };
    const auto* right_space = find_space("sg1/M1:space_node_group_outer:representative_right");
    const auto* top_space = find_space("sg1/M1:space_node_group_top:representative_top");
    require(right_space != nullptr && right_space->physical_region.has_value(), "right space should expose a physical region");
    require(top_space != nullptr && top_space->physical_region.has_value(), "top space should expose a physical region");
    const auto m1_box = placement_box(circuit, request.placements.at("M1"));
    require(approx(right_space->physical_region->x1, m1_box.x2), "right space region should start at module right edge");
    require(approx(top_space->physical_region->y1, m1_box.y2), "top space region should start at module top edge");
    for (const auto& point : request.linking_points) {
        require(point.segments.size() >= 2, "each LCP should connect at least two segments");
        require(point.location_candidates.size() >= 3, "each LCP should expose multiple physical candidates");
        require(
            std::any_of(point.location_candidates.begin(), point.location_candidates.end(), [](const auto& candidate) {
                return candidate.source == "space_grid" || candidate.source == "space_random";
            }),
            "each LCP should include space-node generated candidates");
        require(
            std::all_of(point.location_candidates.begin(), point.location_candidates.end(), [](const auto& candidate) {
                return candidate.validity_level != "strict" || candidate.inside_space_region;
            }),
            "strict LCP candidates should stay inside the assigned space region");
    }

    std::vector<sapr::Rect> boxes;
    for (const auto& module_id : request.placement_order) boxes.push_back(placement_box(circuit, request.placements.at(module_id)));
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        for (std::size_t j = i + 1; j < boxes.size(); ++j) require(!overlaps(boxes[i], boxes[j]), "contour packing should avoid overlap");
    }

    auto feedback_tree = tree;
    sapr::RoutingFeedback feedback;
    feedback.required_space_by_node["sg1/M1:space_node_group_outer:representative_right"] = 25.0;
    feedback.coupling_space_by_node["sg1/M1:space_node_group_outer:representative_right"] = 2.0;
    sapr::apply_routing_feedback(feedback_tree, feedback);
    const auto expanded = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    const auto expanded_right_space = std::find_if(expanded.space_nodes.begin(), expanded.space_nodes.end(), [](const auto& space_node) {
        return space_node.id == "sg1/M1:space_node_group_outer:representative_right";
    });
    require(expanded_right_space != expanded.space_nodes.end(), "expanded request should keep M1 right space");
    require(
        approx(expanded_right_space->allocated_space, 25.0) &&
            approx(expanded_right_space->coupling_extra_space, 2.0) &&
            approx(expanded_right_space->required_space(), 27.0),
        "routing feedback should keep base allocation and coupling space separate");
    require(
        expanded_right_space->physical_region->x2 - expanded_right_space->physical_region->x1 >= 27.0,
        "space physical region width should grow with required space feedback");

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

    bool split_executed = false;
    for (unsigned int seed = 0; seed < 1000 && !split_executed; ++seed) {
        auto split_tree = make_split_test_tree(circuit);
        std::mt19937 split_rng(seed);
        const auto report = sapr::perturb_placement_tree(split_tree, split_rng);
        if (report.move != "lcp-split" || !report.changed) continue;
        split_executed = true;
        require(sapr::count_tree_lcps(split_tree) == 2, "split should replace one LCP with two LCPs");
        require(lcp_references_are_consistent(split_tree), "split should reconnect every moved segment to its new LCP");
        bool has_split_endpoint = false;
        for (const auto& space_node : sapr::collect_space_nodes(split_tree)) {
            for (const auto& point : space_node.linking_points) {
                for (const auto& segment : point.segments) {
                    has_split_endpoint = has_split_endpoint || segment.from.find("S:split") != std::string::npos ||
                                         segment.to.find("S:split") != std::string::npos;
                }
            }
        }
        require(has_split_endpoint, "split should change moved segment endpoints to the created LCP");
    }
    require(split_executed, "test seeds should exercise a successful LCP split");

    bool merge_executed = false;
    for (unsigned int seed = 0; seed < 1000 && !merge_executed; ++seed) {
        auto merge_tree = make_merge_test_tree(circuit);
        std::mt19937 merge_rng(seed);
        const auto report = sapr::perturb_placement_tree(merge_tree, merge_rng);
        if (report.move != "lcp-merge" || !report.changed) continue;
        merge_executed = true;
        require(sapr::count_tree_lcps(merge_tree) == 1, "merge should combine two LCPs into one LCP");
        const auto spaces_after_merge = sapr::collect_space_nodes(merge_tree);
        std::string surviving_id;
        for (const auto& space_node : spaces_after_merge) {
            if (!space_node.linking_points.empty()) surviving_id = space_node.linking_points.front().id;
        }
        const std::string removed_id = surviving_id == "A" ? "B" : "A";
        require(
            lcp_references_are_consistent(merge_tree, removed_id),
            "merge should reconnect all references to the retained LCP before deleting the old LCP");
    }
    require(merge_executed, "test seeds should exercise a successful LCP merge");

    const auto solution = sapr::solve_placement_aware(circuit, config);
    require(solution.metrics.has_value(), "solution should carry routing metric snapshot");
    require(
        std::all_of(solution.sa_progress.begin(), solution.sa_progress.end(), [](const auto& entry) { return entry.changed; }),
        "SA should resample instead of evaluating an unchanged perturbation");
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
    auto four_pin_tree = sapr::make_enhanced_tree(four_pin_circuit);
    sapr::initialize_lcp_topology(four_pin_circuit, four_pin_tree, config);
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
    auto flow_lcp_tree = sapr::make_enhanced_tree(flow_lcp_circuit);
    sapr::initialize_lcp_topology(flow_lcp_circuit, flow_lcp_tree, config);
    const auto flow_lcp_request = sapr::pack_enhanced_tree(flow_lcp_circuit, flow_lcp_tree, config);
    require(
        topology_has_flow_path(flow_lcp_request, "M.P0", "M.P3"),
        "automatic LCP topology should preserve FLOW reachability from source to sink");

    auto blocked_lcp_circuit = make_same_location_lcp_circuit(4);
    blocked_lcp_circuit.modules.at("M").active = {0.0, 0.0, 10.0, 10.0};
    auto blocked_lcp_tree = sapr::make_enhanced_tree(blocked_lcp_circuit);
    sapr::initialize_lcp_topology(blocked_lcp_circuit, blocked_lcp_tree, config);
    const auto blocked_lcp_request = sapr::pack_enhanced_tree(blocked_lcp_circuit, blocked_lcp_tree, config);
    for (const auto& point : blocked_lcp_request.linking_points) {
        for (const auto& candidate : point.location_candidates) {
            require(
                candidate.reason.find("active_blocker") == std::string::npos || candidate.validity_level != "strict",
                "active-blocked LCP candidates should not be classified as strict");
        }
    }

    const auto seven_pin_circuit = make_same_location_lcp_circuit(7);
    auto seven_pin_tree = sapr::make_enhanced_tree(seven_pin_circuit);
    sapr::initialize_lcp_topology(seven_pin_circuit, seven_pin_tree, config);
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
