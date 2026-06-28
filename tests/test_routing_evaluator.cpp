// 文件职责：验证 routing evaluator 可以在样例输入上完成 A*/DP 布线评估。
#include <filesystem>
#include <optional>
#include <unordered_map>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "test_support.hpp"

namespace {

// 返回测试使用的输入目录。
std::filesystem::path source_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "input";
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
        9.0,
        0,
    };
    return evaluation;
}

// 构造带 LCP 拓扑的最小 detailed routing request。
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
    if (with_location) lcp.location_candidates.push_back({4.0, 1.0, "LCP1:first"});
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
        9.0,
        0,
    };
    return evaluation;
}

}  // namespace

// 运行 routing evaluator 的集成测试。
void run_routing_evaluator_tests() {
    const auto circuit = sapr::load_circuit(source_input_dir());
    sapr::SolverConfig config;
    config.sa_iterations = 3;
    const auto solution = sapr::solve_baseline(circuit, config);
    require(solution.metrics.has_value(), "solution should expose optimizer metrics");
    const auto& metrics = *solution.metrics;
    const double expected_phi =
        config.area_weight * metrics.normalized_area +
        config.wirelength_weight * metrics.normalized_wirelength +
        config.bend_weight * metrics.normalized_bend +
        config.via_weight * metrics.normalized_via +
        metrics.penalty;
    require(metrics.phi_cost > 0.0, "optimizer should expose positive phi cost");
    require(approx(metrics.phi_cost, expected_phi), "phi cost should match the paper total-cost formula");
    const auto evaluation = sapr::evaluate_routing(circuit, solution.placements);
    const auto selected_segments = sapr::selected_candidates_to_segments(evaluation);
    sapr::RoutingEvaluationRequest request;
    request.placements = solution.placements;
    request.placement_order = solution.placement_order;
    const auto detailed = sapr::run_detailed_routing(circuit, request, evaluation);

    require(!evaluation.candidates.empty(), "routing evaluator should emit A* route candidates");
    require(evaluation.failed_nets == 0, "routing evaluator should route all nets in the sample input");
    require(evaluation.routing_cost > 0.0, "routing evaluator should produce a positive global routing cost");
    require(!selected_segments.empty(), "routing evaluator should convert selected A* candidates to route segments");
    require(!detailed.routes.empty(), "detailed routing should emit selected route segments");
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

    auto lcp_request = make_lcp_request(drc_circuit, drc_placements, true);
    auto lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto lcp_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, lcp_eval);
    require(!lcp_detail.report.traces.empty(), "detailed routing should expose LCP traceback traces");
    require(!lcp_detail.report.traces.front().segments.empty(), "trace should map selected candidates to route segments");
    require(lcp_detail.space_nodes_with_routes == 1, "LCP detailed route should mark its space node as used");
    require(lcp_detail.required_space_by_node.contains("S1"), "LCP detailed route should report required routing space");
    require(lcp_detail.required_space_by_node.at("S1") >= 3.0, "required space should include route width and spacing");

    auto missing_lcp_request = make_lcp_request(drc_circuit, drc_placements, false);
    auto missing_lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto missing_lcp_detail = sapr::run_detailed_routing(drc_circuit, missing_lcp_request, missing_lcp_eval);
    require(missing_lcp_detail.traceback_failures > 0, "missing LCP location should become a traceback failure");
    require(missing_lcp_detail.routing_failure_penalty > 0.0, "traceback failures should add routing failure penalty");
}
