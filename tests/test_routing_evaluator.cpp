// 文件职责：验证 routing evaluator 可以在样例输入上完成 A*/DP 布线评估。
#include <algorithm>
#include <filesystem>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/routing/dp_router.hpp"
#include "sapr/routing/geometry.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/grid.hpp"
#include "sapr/routing/path_geometry.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "sapr/tree.hpp"
#include "test_support.hpp"

namespace {

// 返回测试使用的输入目录。
std::filesystem::path source_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "input";
}

// 返回 4ring mixed LCP/direct net 回归用例输入目录。
std::filesystem::path mixed_lcp_direct_case_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "cases" / "4ring_2_left6_no_symmetry" / "input";
}

// 构造一个 active 小于 bbox 的最小电路，用于检查 DRC blocker 是否使用真实 active。
sapr::Circuit make_active_region_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{4.0, 4.0, 6.0, 6.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 0.0, 1.0, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 9.0, 1.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 构造 active 覆盖完整 bbox 的最小电路，用于验证 full-bbox active 仍然是布线障碍。
sapr::Circuit make_full_active_region_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{0.0, 0.0, 10.0, 10.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 0.0, 5.0, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 10.0, 5.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 构造 pin 位于左下边界附近的 full-active 电路，用于验证 grid 会纳入向外逃逸空间。
sapr::Circuit make_boundary_access_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{0.0, 0.0, 10.0, 10.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 5.0, 0.1, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 9.9, 5.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 判断 route 金属是否穿过 active 的核心区域，排除边缘 pin access 的短接入段。
bool route_crosses_active_core(const sapr::RouteSegment& route, const sapr::Rect& active) {
    const sapr::Rect normalized = sapr::routing::normalize_rect(active);
    const sapr::Rect core{
        normalized.x1 + 2.0,
        normalized.y1 + 2.0,
        normalized.x2 - 2.0,
        normalized.y2 - 2.0,
    };
    const sapr::Rect metal = sapr::routing::segment_to_rect(
        sapr::routing::Segment{sapr::routing::Point{route.x1, route.y1}, sapr::routing::Point{route.x2, route.y2}},
        route.width);
    return sapr::routing::intersects(metal, core);
}

// 构造一条水平 selected candidate，让 detailed routing 只测试 DRC 统计逻辑。
sapr::RoutingEvaluation make_line_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    double y) {
    sapr::routing::RoutingContext context(circuit, placements);
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = "M.A";
    candidate.to_terminal = "M.B";
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, 0),
    };
    candidate.path.metrics.wirelength = 9.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.selected_candidates.push_back(candidate);

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);
    global.total_metrics.wirelength = 9.0;

    sapr::RoutingEvaluation evaluation{
        std::move(context),
        {candidate},
        std::move(global),
        std::nullopt,
        9.0,
        0,
        false,
    };
    return evaluation;
}

// 构造带 LCP 拓扑的最小 detailed routing request。
sapr::Circuit make_priority_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 12.0, 12.0, sapr::Rect{5.0, 5.0, 6.0, 6.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    const std::vector<std::pair<std::string, double>> pins{
        {"A", 0.0}, {"B", 9.0}, {"C", 0.0}, {"D", 9.0}, {"E", 0.0}, {"F", 9.0},
    };
    for (const auto& [pin, x] : pins) circuit.pins.emplace("M." + pin, sapr::Pin{"M", pin, x, 1.0, "M1"});
    circuit.pin_order = {"M.A", "M.B", "M.C", "M.D", "M.E", "M.F"};
    circuit.nets.emplace("SYM", sapr::Net{"SYM", sapr::Priority::Symmetry, {"M.A", "M.B"}});
    circuit.nets.emplace("CRT", sapr::Net{"CRT", sapr::Priority::Critical, {"M.C", "M.D"}});
    circuit.nets.emplace("NOR", sapr::Net{"NOR", sapr::Priority::Normal, {"M.E", "M.F"}});
    circuit.net_order = {"SYM", "CRT", "NOR"};
    return circuit;
}

// 构造两条不同 net 共享同一几何通道的最小电路，用于验证短路冲突会被拒绝。
sapr::Circuit make_conflict_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 12.0, 12.0, sapr::Rect{5.0, 5.0, 6.0, 6.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    const std::vector<std::string> names{"A", "B", "C", "D"};
    for (const auto& name : names) {
        const double x = (name == "A" || name == "C") ? 0.0 : 9.0;
        circuit.pins.emplace("M." + name, sapr::Pin{"M", name, x, 1.0, "M1"});
    }
    circuit.pin_order = {"M.A", "M.B", "M.C", "M.D"};
    circuit.nets.emplace("N1", sapr::Net{"N1", sapr::Priority::Critical, {"M.A", "M.B"}});
    circuit.nets.emplace("N2", sapr::Net{"N2", sapr::Priority::Normal, {"M.C", "M.D"}});
    circuit.net_order = {"N1", "N2"};
    return circuit;
}

// 构造一条人工水平候选，用于精确测试短路合法化。
sapr::routing::RouteCandidate make_horizontal_candidate(
    const sapr::routing::RoutingContext& context,
    const std::string& net,
    const std::string& from,
    const std::string& to,
    double y,
    int layer = 0) {
    sapr::routing::RouteCandidate candidate;
    candidate.net = net;
    candidate.from_terminal = from;
    candidate.to_terminal = to;
    candidate.segment_id = net + ":" + from + "->" + to;
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, layer),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, layer),
    };
    candidate.path.metrics.wirelength = 9.0;
    return candidate;
}

sapr::RoutingEvaluation make_priority_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    const auto make_candidate = [&](const std::string& net, const std::string& from, const std::string& to, double y) {
        sapr::routing::RouteCandidate candidate;
        candidate.net = net;
        candidate.from_terminal = from;
        candidate.to_terminal = to;
        candidate.wire_width = 1.0;
        candidate.path.success = true;
        candidate.path.points = {
            context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, 0),
            context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, 0),
        };
        candidate.path.metrics.wirelength = 9.0;
        return candidate;
    };
    const auto normal = make_candidate("NOR", "M.E", "M.F", 1.0);
    const auto critical = make_candidate("CRT", "M.C", "M.D", 4.0);
    const auto symmetry = make_candidate("SYM", "M.A", "M.B", 7.0);

    sapr::routing::GlobalRoutingResult global;
    for (const auto& candidate : {normal, critical, symmetry}) {
        sapr::routing::NetRouteChoice choice;
        choice.net = candidate.net;
        choice.selected_candidates.push_back(candidate);
        global.net_routes.push_back(std::move(choice));
    }
    global.total_metrics.wirelength = 27.0;

    return sapr::RoutingEvaluation{
        std::move(context),
        {normal, critical, symmetry},
        std::move(global),
        std::nullopt,
        27.0,
        0,
        false,
    };
}

sapr::RoutingEvaluationRequest make_lcp_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    bool with_location) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"M"};
    request.active_region_blockers.push_back(
        sapr::routing::transform_active_to_global(circuit.modules.at("M"), placements.at("M")));

    sapr::LinkingControlPoint lcp;
    lcp.id = "LCP1";
    lcp.space_node_id = "S1";
    if (with_location) lcp.location_candidates.push_back({4.0, 1.0, "LCP1:first", "strict", false, 0.0, "test"});
    lcp.segments.push_back({"N", "M.A", "LCP1", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Out, "N:left"});
    lcp.segments.push_back({"N", "LCP1", "M.B", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::In, "N:right"});

    sapr::SpaceNode space;
    space.id = "S1";
    space.owner = "M";
    space.kind = sapr::SpaceNodeKind::Right;
    space.linking_points.push_back(lcp);
    request.space_nodes.push_back(space);
    request.linking_points.push_back(lcp);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {lcp}, lcp.segments});
    return request;
}

// 构造一组经过 LCP 的 selected candidates，用于验证 top-down traceback。
sapr::RoutingEvaluationRequest make_reverse_lcp_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    bool with_location) {
    auto request = make_lcp_request(circuit, placements, with_location);
    request.linking_points.clear();
    request.space_nodes.clear();
    request.net_topologies.clear();

    sapr::LinkingControlPoint lcp;
    lcp.id = "LCP1";
    lcp.space_node_id = "S1";
    if (with_location) lcp.location_candidates.push_back({4.0, 1.0, "LCP1:first", "strict", false, 0.0, "test"});
    lcp.segments.push_back({"N", "M.B", "LCP1", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::In, "N:reverse_left"});
    lcp.segments.push_back({"N", "LCP1", "M.A", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Out, "N:reverse_right"});

    sapr::SpaceNode space;
    space.id = "S1";
    space.owner = "M";
    space.kind = sapr::SpaceNodeKind::Right;
    space.linking_points.push_back(lcp);
    request.space_nodes.push_back(space);
    request.linking_points.push_back(lcp);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {lcp}, lcp.segments});
    return request;
}

sapr::RoutingEvaluation make_lcp_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    sapr::routing::RouteCandidate first;
    first.net = "N";
    first.from_terminal = "M.A";
    first.to_terminal = "LCP1";
    first.segment_id = "N:left";
    first.lcp_candidate_id = "LCP1:first";
    first.wire_width = 2.0;
    first.path.success = true;
    first.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{4.0, 1.0}, 0),
    };
    first.path.metrics.wirelength = 4.0;

    sapr::routing::RouteCandidate second = first;
    second.from_terminal = "LCP1";
    second.to_terminal = "M.B";
    second.segment_id = "N:right";
    second.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{4.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.0}, 0),
    };
    second.path.metrics.wirelength = 5.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.selected_candidates = {first, second};
    choice.metrics.wirelength = 9.0;

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);
    global.total_metrics.wirelength = 9.0;

    sapr::RoutingEvaluation evaluation{
        std::move(context),
        {first, second},
        std::move(global),
        std::nullopt,
        9.0,
        0,
        false,
    };
    return evaluation;
}

// 构造 LCP-LCP 线段的 detailed routing 输入，用于验证两端 space 都收到 feedback。
sapr::RoutingEvaluationRequest make_lcp_pair_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request = make_lcp_request(circuit, placements, true);
    request.linking_points.clear();
    request.space_nodes.clear();
    request.net_topologies.clear();

    sapr::LinkingControlPoint left;
    left.id = "LCP_A";
    left.space_node_id = "S_A";
    left.location_candidates.push_back({2.0, 1.0, "LCP_A:first", "strict", false, 0.0, "test"});

    sapr::LinkingControlPoint right;
    right.id = "LCP_B";
    right.space_node_id = "S_B";
    right.location_candidates.push_back({7.0, 1.0, "LCP_B:first", "strict", false, 0.0, "test"});

    sapr::WireSegmentRef segment{"N", "LCP_A", "LCP_B", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:lcp_pair"};
    left.segments.push_back(segment);
    right.segments.push_back(segment);

    sapr::SpaceNode left_space;
    left_space.id = "S_A";
    left_space.owner = "M";
    left_space.kind = sapr::SpaceNodeKind::Right;
    left_space.linking_points.push_back(left);

    sapr::SpaceNode right_space = left_space;
    right_space.id = "S_B";
    right_space.linking_points.clear();
    right_space.linking_points.push_back(right);

    request.space_nodes.push_back(left_space);
    request.space_nodes.push_back(right_space);
    request.linking_points.push_back(left);
    request.linking_points.push_back(right);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {left, right}, {segment}});
    return request;
}

// 构造一条 LCP-LCP selected candidate。
sapr::RoutingEvaluation make_lcp_pair_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = "LCP_A";
    candidate.to_terminal = "LCP_B";
    candidate.segment_id = "N:lcp_pair";
    candidate.lcp_id = "LCP_A";
    candidate.lcp_candidate_id = "LCP_A:first";
    candidate.source_lcp_id = "LCP_A";
    candidate.source_lcp_candidate_id = "LCP_A:first";
    candidate.target_lcp_id = "LCP_B";
    candidate.target_lcp_candidate_id = "LCP_B:first";
    candidate.wire_width = 2.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{2.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{7.0, 1.0}, 0),
    };
    candidate.path.metrics.wirelength = 5.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.success = true;
    choice.selected_candidates.push_back(candidate);

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);

    return {std::move(context), {candidate}, std::move(global), std::nullopt, 5.0, 0, false};
}

// 构造一条成功的人工候选路径，用于测试 LCP 多候选位置一致性选择。
sapr::routing::RouteCandidate make_manual_lcp_candidate(
    const sapr::routing::RoutingContext& context,
    const std::string& from,
    const std::string& to,
    const std::string& location_id,
    double wirelength,
    double y) {
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = from;
    candidate.to_terminal = to;
    candidate.segment_id = from + "->" + to;
    candidate.lcp_id = "LCP1";
    candidate.lcp_candidate_id = location_id;
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{wirelength, y}, 0),
    };
    candidate.path.metrics.wirelength = wirelength;
    return candidate;
}

}  // namespace

// 运行 routing evaluator 的集成测试。
void run_routing_evaluator_tests() {
    const auto circuit = sapr::load_circuit(source_input_dir());
    sapr::SolverConfig config;
    config.sa_iterations = 0;
    const auto solution = sapr::solve_baseline(circuit, config);
    require(solution.metrics.has_value(), "solution should expose optimizer metrics");
    const auto& metrics = *solution.metrics;
    const double expected_phi =
        config.area_weight * metrics.normalized_area +
        config.wirelength_weight * metrics.normalized_wirelength +
        config.bend_weight * metrics.normalized_bend +
        config.via_weight * metrics.normalized_via +
        metrics.row_width_penalty +
        metrics.penalty;
    require(metrics.phi_cost > 0.0, "optimizer should expose positive phi cost");
    require(approx(metrics.phi_cost, expected_phi), "phi cost should match the paper total-cost formula");
    const double expected_dedup_penalty =
        metrics.flow_penalty +
        metrics.current_density_penalty +
        metrics.coupling_penalty +
        metrics.design_rule_penalty +
        metrics.routing_failure_penalty;
    require(approx(metrics.penalty, expected_dedup_penalty), "SA penalty should use deduplicated routing penalty terms");
    require(metrics.dp_traceback_segments > 0, "placement-aware routing should produce bottom-up DP traceback");
    require(metrics.dp_nodes > 0, "bottom-up DP should visit B*-tree nodes");
    require(metrics.dp_states > 0, "bottom-up DP should keep candidate states");
    require(metrics.dp_states >= metrics.dp_nodes, "bottom-up DP should keep at least one state per visited node");
    require(metrics.packing_trace_steps > 0, "packing should expose contour trace steps");
    require(metrics.packing_time_dp_used, "bottom-up DP should consume packing-time local wire segments");
    require(metrics.packing_time_dp_segments > 0, "packing-time DP should expose local wire segment count");
    require(metrics.space_feedback_nodes >= 0, "optimizer should expose routing space feedback count");
    require(metrics.routing_feedback_iterations >= 1, "candidate evaluation should run at least one feedback iteration");
    require(
        metrics.routing_feedback_iterations <= config.routing_feedback_iterations,
        "candidate evaluation should honor max routing feedback iterations");
    if (metrics.routing_feedback_iterations < config.routing_feedback_iterations) {
        require(metrics.routing_feedback_converged, "early feedback-loop stop should mean convergence");
    }
    auto single_loop_config = config;
    single_loop_config.sa_iterations = 0;
    single_loop_config.routing_feedback_iterations = 1;
    const auto single_loop_solution = sapr::solve_placement_aware(circuit, single_loop_config);
    require(single_loop_solution.metrics.has_value(), "single-loop solution should expose metrics");
    require(
        single_loop_solution.metrics->routing_feedback_iterations == 1,
        "routing_feedback_iterations=1 should keep current single-pass behavior");
    const auto boundary_tree = sapr::make_enhanced_tree(circuit);
    const auto request_for_dp = sapr::pack_enhanced_tree(circuit, boundary_tree, config);
    require(!request_for_dp.packing_trace.steps.empty(), "pack_enhanced_tree should record contour trace");
    require(boundary_tree.root.has_value(), "boundary margin test requires a root node");
    const auto& auto_root = request_for_dp.placements.at(*boundary_tree.root);
    require(auto_root.x > 0.0 && auto_root.y > 0.0, "auto boundary margin should move the root inside the chip");

    auto zero_margin_config = config;
    zero_margin_config.boundary_margin = 0.0;
    const auto zero_margin_request = sapr::pack_enhanced_tree(circuit, boundary_tree, zero_margin_config);
    auto explicit_margin_config = config;
    explicit_margin_config.boundary_margin = 1.5;
    const auto explicit_margin_request = sapr::pack_enhanced_tree(circuit, boundary_tree, explicit_margin_config);
    const auto& zero_root = zero_margin_request.placements.at(*boundary_tree.root);
    const auto& explicit_root = explicit_margin_request.placements.at(*boundary_tree.root);
    require(approx(explicit_root.x - zero_root.x, 1.5), "explicit boundary margin should shift root x");
    require(approx(explicit_root.y - zero_root.y, 1.5), "explicit boundary margin should shift root y");
    for (const auto& [id, zero_placement] : zero_margin_request.placements) {
        const auto& shifted = explicit_margin_request.placements.at(id);
        require(approx(shifted.x - zero_placement.x, 1.5), "boundary margin should preserve relative x placement");
        require(approx(shifted.y - zero_placement.y, 1.5), "boundary margin should preserve relative y placement");
    }
    require(
        approx(explicit_margin_request.packing_trace.steps.front().occupied_bbox.x1, 1.5) &&
            approx(explicit_margin_request.packing_trace.steps.front().occupied_bbox.y1, 1.5),
        "packing trace root bbox should start at the explicit boundary margin");
    require(
        request_for_dp.packing_trace.steps.size() == request_for_dp.tree.nodes.size(),
        "packing trace should contain one step per representative tree node");
    for (const auto& step : request_for_dp.packing_trace.steps) {
        require(step.occupied_bbox.x1 <= step.occupied_bbox.x2, "packing trace bbox x range should be valid");
        require(step.occupied_bbox.y1 <= step.occupied_bbox.y2, "packing trace bbox y range should be valid");
        require(step.right_space >= 0.0, "packing trace should record non-negative right routing space");
        require(step.top_space >= 0.0, "packing trace should record non-negative top routing space");
        require(step.coupling_extra_space >= 0.0, "packing trace should record non-negative coupling space");
    }
    const auto local_segment_count = std::accumulate(
        request_for_dp.packing_trace.steps.begin(),
        request_for_dp.packing_trace.steps.end(),
        std::size_t{0},
        [](std::size_t total, const auto& step) { return total + step.local_wire_segments.size(); });
    require(local_segment_count > 0, "packing trace should expose packing-time local routing segments");
    std::unordered_map<std::string, const sapr::WireSegmentRef*> segment_by_key;
    for (const auto& topology : request_for_dp.net_topologies) {
        for (const auto& segment : topology.segments) {
            const std::string key = segment.id.empty() ? segment.net + "|" + segment.from + "|" + segment.to : segment.id;
            segment_by_key[key] = &segment;
        }
    }
    std::unordered_map<std::string, std::string> lcp_owner_by_id;
    std::unordered_map<std::string, std::string> owner_by_space;
    for (const auto& space : request_for_dp.space_nodes) {
        owner_by_space[space.id] = space.owner;
        for (const auto& point : space.linking_points) lcp_owner_by_id[point.id] = space.owner;
    }
    for (const auto& point : request_for_dp.linking_points) {
        const auto found = owner_by_space.find(point.space_node_id);
        if (found != owner_by_space.end()) lcp_owner_by_id[point.id] = found->second;
    }
    for (const auto& topology : request_for_dp.net_topologies) {
        for (const auto& point : topology.linking_points) {
            std::unordered_set<std::string> segment_ids;
            int lcp_incident_segments = 0;
            for (const auto& segment : point.segments) {
                if (segment.from == point.id || segment.to == point.id) ++lcp_incident_segments;
                require(segment_ids.insert(segment.id).second, "LCP topology should not duplicate root-to-LCP segments");
            }
            require(lcp_incident_segments >= 2, "each logical LCP should connect at least two same-net segments");
        }
    }
    const auto terminal_owner = [&](const std::string& terminal) {
        const auto dot = terminal.find('.');
        if (dot != std::string::npos) return terminal.substr(0, dot);
        const auto found = lcp_owner_by_id.find(terminal);
        return found == lcp_owner_by_id.end() ? std::string{} : found->second;
    };
    const auto step_by_node = [&](const std::optional<std::string>& id) -> const sapr::PackingContourStep* {
        if (!id.has_value()) return nullptr;
        const auto found = std::find_if(
            request_for_dp.packing_trace.steps.begin(),
            request_for_dp.packing_trace.steps.end(),
            [&](const auto& step) { return step.tree_node == *id; });
        return found == request_for_dp.packing_trace.steps.end() ? nullptr : &*found;
    };
    const auto inside_modules = [](const std::string& from, const std::string& to, const auto& modules) {
        return modules.contains(from) && modules.contains(to);
    };
    for (const auto& step : request_for_dp.packing_trace.steps) {
        const std::unordered_set<std::string> current_modules(step.subtree_modules.begin(), step.subtree_modules.end());
        const auto* left_step = step_by_node(step.left);
        const auto* right_step = step_by_node(step.right);
        const std::unordered_set<std::string> left_modules =
            left_step == nullptr ? std::unordered_set<std::string>{}
                                 : std::unordered_set<std::string>(left_step->subtree_modules.begin(), left_step->subtree_modules.end());
        const std::unordered_set<std::string> right_modules =
            right_step == nullptr ? std::unordered_set<std::string>{}
                                  : std::unordered_set<std::string>(right_step->subtree_modules.begin(), right_step->subtree_modules.end());
        for (const auto& key : step.local_wire_segments) {
            require(segment_by_key.contains(key), "packing-time local segment should refer to a known topology segment");
            const auto* segment = segment_by_key.at(key);
            const auto from_owner = terminal_owner(segment->from);
            const auto to_owner = terminal_owner(segment->to);
            require(inside_modules(from_owner, to_owner, current_modules), "local segment endpoints should be inside current subtree");
            require(!inside_modules(from_owner, to_owner, left_modules), "local segment should not be fully inside left child");
            require(!inside_modules(from_owner, to_owner, right_modules), "local segment should not be fully inside right child");
        }
    }
    const auto root_step = std::find_if(
        request_for_dp.packing_trace.steps.begin(),
        request_for_dp.packing_trace.steps.end(),
        [&](const auto& step) { return request_for_dp.tree.root.has_value() && step.tree_node == *request_for_dp.tree.root; });
    require(root_step != request_for_dp.packing_trace.steps.end(), "packing trace should include root step");
    std::unordered_set<std::string> root_modules(root_step->subtree_modules.begin(), root_step->subtree_modules.end());
    for (const auto& node : request_for_dp.tree.nodes) {
        require(root_modules.contains(node.module), "root packing trace step should cover all placed representative modules");
    }
    const auto dp_evaluation = sapr::evaluate_routing(circuit, request_for_dp);
    require(dp_evaluation.bottom_up_dp.has_value(), "routing evaluator should expose bottom-up DP result");
    require(!dp_evaluation.bottom_up_dp->traceback_candidates.empty(), "bottom-up DP should produce traceback candidates");
    require(dp_evaluation.bottom_up_dp->packing_time_dp_used, "bottom-up DP should prefer packing-time local segments");
    require(dp_evaluation.bottom_up_dp->packing_time_dp_segments > 0, "bottom-up DP should count packing-time local segments");
    require(
        !dp_evaluation.bottom_up_dp->best_state.covered_wire_segments.empty(),
        "bottom-up DP state should record covered wire segments");
    require(
        !dp_evaluation.bottom_up_dp->best_state.selected_transitions.empty(),
        "bottom-up DP state should record selected transitions");
    require(
        dp_evaluation.bottom_up_dp->best_state.packing_step_index >= 0,
        "bottom-up DP state should record packing step index when contour trace exists");
    const bool has_space_trace = std::any_of(
        dp_evaluation.bottom_up_dp->best_state.selected_transitions.begin(),
        dp_evaluation.bottom_up_dp->best_state.selected_transitions.end(),
        [](const auto& transition) { return transition.find("@space=") != std::string::npos; });
    require(has_space_trace, "LCP DP transitions should record source space node id");
    for (const auto& node_result : dp_evaluation.bottom_up_dp->node_results) {
        require(node_result.states.size() <= 8, "bottom-up DP should honor max_states_per_node");
    }
    const auto mixed_circuit = sapr::load_circuit(mixed_lcp_direct_case_input_dir());
    const auto mixed_request = sapr::pack_enhanced_tree(mixed_circuit, sapr::make_chain_tree(mixed_circuit), config);
    const auto mixed_evaluation = sapr::evaluate_routing(mixed_circuit, mixed_request);
    require(
        mixed_evaluation.bottom_up_dp.has_value() && mixed_evaluation.bottom_up_dp->success,
        "mixed LCP/direct case should use successful bottom-up DP");
    require(mixed_evaluation.failed_nets == 0, "successful DP should not drop direct-only nets from global routing");
    std::unordered_set<std::string> mixed_routed_nets;
    for (const auto& segment : sapr::selected_candidates_to_segments(mixed_evaluation)) {
        mixed_routed_nets.insert(segment.net);
    }
    for (const auto& net : mixed_circuit.net_order) {
        require(mixed_routed_nets.contains(net), "mixed LCP/direct case should keep every net in selected routes");
    }
    auto no_local_segment_request = request_for_dp;
    for (auto& step : no_local_segment_request.packing_trace.steps) {
        step.local_wire_segments.clear();
        step.cross_child_wire_segments.clear();
    }
    const auto no_local_segment_evaluation = sapr::evaluate_routing(circuit, no_local_segment_request);
    require(no_local_segment_evaluation.bottom_up_dp.has_value(), "fallback DP should expose a result without local segments");
    require(
        !no_local_segment_evaluation.bottom_up_dp->traceback_candidates.empty(),
        "fallback DP should produce traceback candidates without local segments");
    require(
        !no_local_segment_evaluation.bottom_up_dp->packing_time_dp_used,
        "cleared local segments should force fallback DP transition inference");
    auto feedback_tree = sapr::make_enhanced_tree(circuit);
    const auto first_feedback_request = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    const auto first_feedback = sapr::evaluate_with_routing_adapter(circuit, first_feedback_request);
    sapr::apply_routing_feedback(feedback_tree, first_feedback);
    const auto second_feedback_request = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    require(
        second_feedback_request.packing_trace.steps.size() == first_feedback_request.packing_trace.steps.size(),
        "routing feedback should preserve representative packing trace size");
    const auto trace_space_sum = [](const sapr::RoutingEvaluationRequest& request) {
        double total = 0.0;
        for (const auto& step : request.packing_trace.steps) {
            total += step.right_space + step.top_space + step.coupling_extra_space;
        }
        return total;
    };
    require(
        trace_space_sum(second_feedback_request) + 1e-9 >= trace_space_sum(first_feedback_request),
        "routing feedback should not reduce contour routing space demand");
    if (first_feedback.metrics.space_feedback_nodes > 0) {
        require(
            !first_feedback.required_space_by_node.empty() || !first_feedback.coupling_space_by_node.empty(),
            "space feedback count should correspond to emitted feedback maps");
    }
    auto no_trace_request = request_for_dp;
    no_trace_request.packing_trace.steps.clear();
    const auto no_trace_evaluation = sapr::evaluate_routing(circuit, no_trace_request);
    require(no_trace_evaluation.bottom_up_dp.has_value(), "fallback DP should still expose a result");
    require(!no_trace_evaluation.bottom_up_dp->traceback_candidates.empty(), "fallback DP should still produce traceback");
    const auto evaluation = sapr::evaluate_routing(circuit, solution.placements);
    const auto selected_segments = sapr::selected_candidates_to_segments(evaluation);
    sapr::RoutingEvaluationRequest request;
    request.placements = solution.placements;
    request.placement_order = solution.placement_order;
    const auto detailed = sapr::run_detailed_routing(circuit, request, evaluation);

    require(!evaluation.candidates.empty(), "routing evaluator should emit A* route candidates");
    require(evaluation.failed_nets >= 0, "routing evaluator should report routed and failed nets");
    if (evaluation.failed_nets > 0) {
        require(
            evaluation.global_routing.routing_failure_penalty >= 100000.0 * static_cast<double>(evaluation.failed_nets),
            "failed nets should contribute high routing penalty");
    }
    require(evaluation.routing_cost > 0.0, "routing evaluator should produce a positive global routing cost");
    require(!selected_segments.empty(), "routing evaluator should convert selected A* candidates to route segments");
    require(
        !detailed.routes.empty() || detailed.traceback_failures > 0 || detailed.design_rule_violations > 0,
        "detailed routing should either emit legal route segments or report why traceback was discarded");
    require(detailed.coupling_penalty >= 0.0, "detailed routing should report coupling penalty");
    require(detailed.design_rule_penalty >= 0.0, "detailed routing should report DRC penalty");
    require(detailed.design_rule_violations >= 0, "detailed routing should report DRC violations");
    require(detailed.current_density_violations == 0, "detailed routing should keep route widths inside constraints");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.flow_penalty, "flow penalty should be part of total penalty");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.current_density_penalty, "current-density penalty should be part of total penalty");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.coupling_penalty, "coupling penalty should be part of total penalty");
    require(evaluation.global_routing.routing_failure_penalty >=
                100000.0 * static_cast<double>(evaluation.failed_nets),
            "routing failures should contribute high penalty");
    require(selected_segments.size() > circuit.net_order.size(), "A*/DP route output should not collapse to one segment per net");
    for (const auto& segment : selected_segments) {
        const auto width = circuit.constraints.wire_widths.find(segment.net);
        if (width != circuit.constraints.wire_widths.end()) {
            require(segment.width >= width->second.min_width, "selected segment width should satisfy min width");
            require(segment.width <= width->second.max_width, "selected segment width should satisfy max width");
        }
    }
    for (const auto& segment : detailed.routes) {
        const auto width = circuit.constraints.wire_widths.find(segment.net);
        if (width != circuit.constraints.wire_widths.end()) {
            require(segment.width >= width->second.min_width, "detailed segment width should satisfy min width");
            require(segment.width <= width->second.max_width, "detailed segment width should satisfy max width");
        }
        require(segment.x1 != segment.x2 || segment.y1 != segment.y2, "detailed routing should not emit zero-length segments");
    }

    const double expected_cost =
        evaluation.global_routing.total_metrics.wirelength +
        3.0 * static_cast<double>(evaluation.global_routing.total_metrics.bend_count) +
        evaluation.global_routing.total_penalty;
    require(approx(evaluation.routing_cost, expected_cost), "global routing cost must not include via count");

    sapr::routing::Grid bounded_grid(sapr::routing::GridConfig{1.0, 5.0, 2}, 0.0, 0.0, 10.0, 10.0);
    require(approx(bounded_grid.min_x(), 0.0), "routing grid should not expand the lower x boundary outside layout");
    require(approx(bounded_grid.min_y(), 0.0), "routing grid should not expand the lower y boundary outside layout");

    const auto conflict_circuit = make_conflict_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> conflict_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    sapr::routing::RoutingContext conflict_context(conflict_circuit, conflict_placements);
    const auto shared_path = std::vector<sapr::routing::GridPoint>{
        conflict_context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.0}, 0),
        conflict_context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.0}, 0),
    };
    sapr::routing::RouteCandidate first_conflict_candidate;
    first_conflict_candidate.net = "N1";
    first_conflict_candidate.from_terminal = "M.A";
    first_conflict_candidate.to_terminal = "M.B";
    first_conflict_candidate.path.success = true;
    first_conflict_candidate.path.points = shared_path;
    first_conflict_candidate.path.metrics.wirelength = 9.0;
    sapr::routing::RouteCandidate second_conflict_candidate = first_conflict_candidate;
    second_conflict_candidate.net = "N2";
    second_conflict_candidate.from_terminal = "M.C";
    second_conflict_candidate.to_terminal = "M.D";
    const auto conflict_global = sapr::routing::run_global_routing(
        conflict_circuit,
        conflict_context,
        {first_conflict_candidate, second_conflict_candidate});
    require(conflict_global.failed_nets == 0, "global routing keeps a high-penalty fallback when every candidate conflicts");
    require(
        conflict_global.coupling_penalty >= 100000.0,
        "different nets sharing routing grid points should receive a high conflict penalty");

    sapr::RouteSegment first_metal{"N1", "M1", 0.0, 1.0, 9.0, 1.0, 1.0};
    sapr::RouteSegment second_metal{"N2", "M1", 0.0, 2.0, 9.0, 2.0, 1.0};
    require(
        sapr::routing::same_layer_short(first_metal, second_metal),
        "same-layer metal rectangles should detect shorts even when centerlines use different tracks");

    const auto preferred_first = make_horizontal_candidate(conflict_context, "N1", "M.A", "M.B", 1.0);
    const auto short_second = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 2.0);
    const auto legal_second = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 4.0);
    const auto short_aware_global = sapr::routing::run_global_routing(
        conflict_circuit,
        conflict_context,
        {preferred_first, short_second, legal_second});
    require(short_aware_global.failed_nets == 0, "short-aware global routing should keep legal fallback candidates");
    require(
        short_aware_global.net_routes.size() == 2 &&
            short_aware_global.net_routes[1].selected_candidates.front().path.points.front().iy ==
                legal_second.path.points.front().iy,
        "global routing should prefer a non-short candidate over a shorter same-layer short");

    sapr::routing::GlobalRoutingResult detailed_global;
    sapr::routing::NetRouteChoice first_choice;
    first_choice.net = "N1";
    first_choice.selected_candidates.push_back(preferred_first);
    detailed_global.net_routes.push_back(first_choice);
    sapr::routing::NetRouteChoice second_choice;
    second_choice.net = "N2";
    second_choice.selected_candidates.push_back(short_second);
    detailed_global.net_routes.push_back(second_choice);
    sapr::RoutingEvaluation short_detail_eval{
        sapr::routing::RoutingContext(conflict_circuit, conflict_placements),
        {preferred_first, short_second, legal_second},
        std::move(detailed_global),
        std::nullopt,
        18.0,
        0,
        false,
    };
    sapr::RoutingEvaluationRequest short_detail_request;
    short_detail_request.placements = conflict_placements;
    short_detail_request.placement_order = {"M"};
    const auto legalized_detail = sapr::run_detailed_routing(conflict_circuit, short_detail_request, short_detail_eval);
    require(legalized_detail.design_rule_violations == 0, "detailed routing should legalize shorts with an alternative candidate");
    require(!legalized_detail.routes.empty(), "detailed routing should keep legalized routes instead of clearing all output");

    sapr::routing::GlobalRoutingResult layer_global;
    layer_global.net_routes.push_back(first_choice);
    layer_global.net_routes.push_back(second_choice);
    sapr::RoutingEvaluation layer_detail_eval{
        sapr::routing::RoutingContext(conflict_circuit, conflict_placements),
        {preferred_first, short_second},
        std::move(layer_global),
        std::nullopt,
        18.0,
        0,
        false,
    };
    const auto layer_detail = sapr::run_detailed_routing(conflict_circuit, short_detail_request, layer_detail_eval);
    require(layer_detail.design_rule_violations == 0, "detailed routing should legalize shorts by reassigning layer when no alternative path exists");
    require(
        std::any_of(layer_detail.routes.begin(), layer_detail.routes.end(), [](const auto& route) {
            return route.net == "N2" && route.layer != "M1";
        }),
        "layer reassignment should move the conflicting net away from the original metal layer");

    const auto drc_circuit = make_active_region_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> drc_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    sapr::RoutingEvaluationRequest drc_request;
    drc_request.placements = drc_placements;
    drc_request.placement_order = {"M"};
    drc_request.active_region_blockers.push_back(
        sapr::routing::transform_active_to_global(drc_circuit.modules.at("M"), drc_placements.at("M")));

    auto bbox_margin_eval = make_line_evaluation(drc_circuit, drc_placements, 1.0);
    const auto bbox_margin_detail = sapr::run_detailed_routing(drc_circuit, drc_request, bbox_margin_eval);
    require(bbox_margin_detail.design_rule_violations == 0, "bbox margin route should not count as active-region DRC");

    auto active_crossing_eval = make_line_evaluation(drc_circuit, drc_placements, 5.0);
    const auto active_crossing_detail = sapr::run_detailed_routing(drc_circuit, drc_request, active_crossing_eval);
    require(active_crossing_detail.design_rule_violations > 0, "active crossing route should count as DRC");

    const auto full_active_circuit = make_full_active_region_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> full_active_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    const auto full_active_eval = sapr::evaluate_routing(full_active_circuit, full_active_placements);
    const auto full_active_segments = sapr::selected_candidates_to_segments(full_active_eval);
    require(!full_active_segments.empty(), "full-bbox active route should still find a path through pin access");
    const auto full_active_rect =
        sapr::routing::transform_active_to_global(full_active_circuit.modules.at("M"), full_active_placements.at("M"));
    for (const auto& segment : full_active_segments) {
        require(
            !route_crosses_active_core(segment, full_active_rect),
            "full-bbox active should block long routes through the module core");
    }

    const auto boundary_circuit = make_boundary_access_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> boundary_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    const sapr::routing::RoutingContext boundary_context(boundary_circuit, boundary_placements);
    require(boundary_context.grid().min_x() >= 0.0, "routing grid should not extend left of the chip boundary");
    require(boundary_context.grid().min_y() >= 0.0, "routing grid should not extend below the chip boundary");
    const auto boundary_eval = sapr::evaluate_routing(boundary_circuit, boundary_placements);
    const auto boundary_segments = sapr::selected_candidates_to_segments(boundary_eval);
    require(!boundary_segments.empty(), "boundary pin access should still find a routed path");
    const auto boundary_active_rect =
        sapr::routing::transform_active_to_global(boundary_circuit.modules.at("M"), boundary_placements.at("M"));
    for (const auto& segment : boundary_segments) {
        require(
            std::min(segment.x1, segment.x2) >= -1e-9 &&
                std::min(segment.y1, segment.y2) >= -1e-9,
            "route centerline should remain inside the non-negative chip boundary");
        require(
            !route_crosses_active_core(segment, boundary_active_rect),
            "boundary pin access should not become a long route through active core");
    }

    const auto priority_circuit = make_priority_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> priority_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    sapr::RoutingEvaluationRequest priority_request;
    priority_request.placements = priority_placements;
    priority_request.placement_order = {"M"};
    const auto priority_eval = make_priority_evaluation(priority_circuit, priority_placements);
    const auto priority_detail = sapr::run_detailed_routing(priority_circuit, priority_request, priority_eval);
    require(priority_detail.routes.size() >= 3, "priority detailed routing should emit one route per selected candidate");
    require(priority_detail.routes[0].net == "SYM", "symmetry net should be detailed-routed before other priorities");
    require(priority_detail.routes[1].net == "CRT", "critical net should be detailed-routed after symmetry net");
    require(priority_detail.routes[2].net == "NOR", "normal net should be detailed-routed after priority nets");

    auto lcp_request = make_lcp_request(drc_circuit, drc_placements, true);
    auto lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto lcp_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, lcp_eval);
    require(!lcp_detail.report.traces.empty(), "detailed routing should expose LCP traceback traces");
    require(!lcp_detail.report.traces.front().segments.empty(), "trace should map selected candidates to route segments");
    require(lcp_detail.space_nodes_with_routes == 1, "LCP detailed route should mark its space node as used");
    require(lcp_detail.required_space_by_node.contains("S1"), "LCP detailed route should report required routing space");
    require(lcp_detail.required_space_by_node.at("S1") >= 3.0, "required space should include route width and spacing");
    require(lcp_detail.detailed_cost > 0.0, "detailed routing should report local detailed cost");

    const auto lcp_pair_request = make_lcp_pair_request(drc_circuit, drc_placements);
    const auto lcp_pair_eval = make_lcp_pair_evaluation(drc_circuit, drc_placements);
    const auto lcp_pair_detail = sapr::run_detailed_routing(drc_circuit, lcp_pair_request, lcp_pair_eval);
    require(lcp_pair_detail.required_space_by_node.contains("S_A"), "LCP-LCP route should feedback source space");
    require(lcp_pair_detail.required_space_by_node.contains("S_B"), "LCP-LCP route should feedback target space");

    auto flow_circuit = drc_circuit;
    flow_circuit.constraints.flows.push_back({"N", "M.A", "M.B"});
    const auto flow_ok_detail = sapr::run_detailed_routing(flow_circuit, lcp_request, lcp_eval);
    require(flow_ok_detail.flow_violations == 0, "out-pin to in-pin LCP traceback should satisfy FLOW");
    auto reverse_lcp_request = make_reverse_lcp_request(flow_circuit, drc_placements, true);
    const auto reverse_lcp_eval = sapr::evaluate_routing(flow_circuit, reverse_lcp_request);
    const bool has_reverse_flow_candidate = std::any_of(
        reverse_lcp_eval.candidates.begin(),
        reverse_lcp_eval.candidates.end(),
        [](const auto& candidate) { return !candidate.flow_ok && candidate.flow_penalty > 0.0; });
    require(has_reverse_flow_candidate, "reversed LCP segments should get FLOW penalty during candidate generation");
    require(
        reverse_lcp_eval.global_routing.flow_penalty > 0.0,
        "reversed LCP segments should contribute FLOW penalty during global routing");

    auto reverse_flow_eval = make_lcp_evaluation(flow_circuit, drc_placements);
    auto& reverse_choice = reverse_flow_eval.global_routing.net_routes.front();
    reverse_choice.selected_candidates[0].from_terminal = "LCP1";
    reverse_choice.selected_candidates[0].to_terminal = "M.A";
    reverse_choice.selected_candidates[1].from_terminal = "M.B";
    reverse_choice.selected_candidates[1].to_terminal = "LCP1";
    const auto reverse_flow_detail = sapr::run_detailed_routing(flow_circuit, lcp_request, reverse_flow_eval);
    require(reverse_flow_detail.flow_violations > 0, "reversed LCP traceback should violate FLOW");
    require(reverse_flow_detail.flow_penalty > 0.0, "FLOW violation should add detailed flow penalty");
    require(!reverse_flow_detail.report.flow_segments.empty(), "FLOW violation should be reported with segment source");

    auto width_violation_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    width_violation_eval.global_routing.net_routes.front().selected_candidates.front().wire_width = 1.0;
    const auto width_violation_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, width_violation_eval);
    require(width_violation_detail.current_density_violations > 0, "segment width below min should violate current-density proxy");
    require(width_violation_detail.current_density_penalty > 0.0, "width violation should add current-density penalty");
    require(
        !width_violation_detail.report.current_density_segments.empty(),
        "current-density violation should be reported with segment source");

    auto missing_lcp_request = make_lcp_request(drc_circuit, drc_placements, false);
    const auto missing_lcp_global_eval = sapr::evaluate_routing(drc_circuit, missing_lcp_request);
    require(missing_lcp_global_eval.failed_nets > 0, "missing LCP location should fail during global routing");
    require(
        missing_lcp_global_eval.global_routing.routing_failure_penalty > 0.0,
        "missing LCP location should add global routing failure penalty");
    const bool has_fake_fallback_location = std::any_of(
        missing_lcp_global_eval.candidates.begin(),
        missing_lcp_global_eval.candidates.end(),
        [](const auto& candidate) {
            return candidate.lcp_candidate_id.find(":fallback") != std::string::npos || candidate.path.success;
        });
    require(!has_fake_fallback_location, "missing LCP location should not create a fake successful fallback candidate");
    auto missing_lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto missing_lcp_detail = sapr::run_detailed_routing(drc_circuit, missing_lcp_request, missing_lcp_eval);
    require(missing_lcp_detail.traceback_failures > 0, "missing LCP location should become a traceback failure");
    require(missing_lcp_detail.routing_failure_penalty > 0.0, "traceback failures should add routing failure penalty");

    sapr::routing::RoutingContext lcp_context(drc_circuit, drc_placements);
    std::vector<sapr::routing::RouteCandidate> multi_location_candidates{
        make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:a", 1.0, 1.0),
        make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:b", 100.0, 2.0),
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:a", 100.0, 3.0),
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:b", 1.0, 4.0),
    };
    const auto lcp_global = sapr::routing::run_global_routing(drc_circuit, lcp_context, multi_location_candidates);
    require(lcp_global.failed_nets == 0, "consistent LCP location selection should find a route");
    require(lcp_global.net_routes.front().selected_candidates.size() == 2, "LCP net should select one candidate per segment");
    const auto selected_location = lcp_global.net_routes.front().selected_candidates.front().lcp_candidate_id;
    for (const auto& candidate : lcp_global.net_routes.front().selected_candidates) {
        require(candidate.lcp_candidate_id == selected_location, "all selected segments of one LCP should share one location");
    }

    auto missing_segment_request = make_lcp_request(drc_circuit, drc_placements, true);
    missing_segment_request.tree.root = "M";
    missing_segment_request.tree.nodes.push_back({"M", "M", std::nullopt, std::nullopt});
    const std::vector<sapr::routing::RouteCandidate> incomplete_candidates{
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:first", 5.0, 1.0),
    };
    const auto missing_segment_dp =
        sapr::routing::run_bottom_up_routing_dp(drc_circuit, missing_segment_request, lcp_context, incomplete_candidates);
    require(!missing_segment_dp.success, "bottom-up DP should fail when a required wire segment has no candidate");
    require(
        missing_segment_dp.best_state.penalty >= 100000.0,
        "missing required wire segment should add routing failure penalty");
    require(
        !missing_segment_dp.best_state.failure_messages.empty(),
        "missing required wire segment should be recorded in DP failure messages");

    auto coupling_eval = make_line_evaluation(drc_circuit, drc_placements, 1.0);
    auto coupling_candidate = coupling_eval.global_routing.net_routes.front().selected_candidates.front();
    coupling_candidate.net = "P";
    coupling_candidate.path.points = {
        coupling_eval.context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.5}, 0),
        coupling_eval.context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.5}, 0),
    };
    sapr::routing::NetRouteChoice coupling_choice;
    coupling_choice.net = "P";
    coupling_choice.selected_candidates.push_back(coupling_candidate);
    coupling_eval.global_routing.net_routes.push_back(coupling_choice);
    const auto coupling_detail = sapr::run_detailed_routing(drc_circuit, drc_request, coupling_eval);
    require(coupling_detail.design_rule_violations == 0, "same-layer different-net overlap should be legalized before final DRC");
    require(!coupling_detail.routes.empty(), "legalized detailed routing should keep route output");
    require(
        std::any_of(coupling_detail.routes.begin(), coupling_detail.routes.end(), [](const auto& route) {
            return route.net == "P" && route.layer != "M1";
        }),
        "overlapping same-layer route should be reassigned when no alternative candidate exists");
}
