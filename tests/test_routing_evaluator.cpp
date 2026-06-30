// 文件职责：验证 routing evaluator 可以在样例输入上完成 A*/DP 布线评估。
#include <algorithm>
#include <filesystem>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/routing/dp_router.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "sapr/tree.hpp"
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
        std::nullopt,
        9.0,
        0,
        false,
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
        std::nullopt,
        9.0,
        0,
        false,
    };
    return evaluation;
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
    require(metrics.dp_used, "placement-aware routing should use bottom-up DP traceback");
    require(metrics.dp_nodes > 0, "bottom-up DP should visit B*-tree nodes");
    require(metrics.dp_states > 0, "bottom-up DP should keep candidate states");
    require(metrics.dp_traceback_segments > 0, "bottom-up DP should produce traceback candidates");
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
    const auto request_for_dp = sapr::pack_enhanced_tree(circuit, sapr::make_enhanced_tree(circuit), config);
    require(!request_for_dp.packing_trace.steps.empty(), "pack_enhanced_tree should record contour trace");
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
    for (const auto& space : request_for_dp.space_nodes) {
        for (const auto& point : space.linking_points) lcp_owner_by_id[point.id] = space.owner;
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
    require(dp_evaluation.used_bottom_up_dp, "routing evaluator should use bottom-up DP when tree snapshot exists");
    require(dp_evaluation.bottom_up_dp.has_value(), "routing evaluator should expose bottom-up DP result");
    require(dp_evaluation.bottom_up_dp->success, "bottom-up DP should succeed on sample input");
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
    auto no_local_segment_request = request_for_dp;
    for (auto& step : no_local_segment_request.packing_trace.steps) {
        step.local_wire_segments.clear();
        step.cross_child_wire_segments.clear();
    }
    const auto no_local_segment_evaluation = sapr::evaluate_routing(circuit, no_local_segment_request);
    require(no_local_segment_evaluation.used_bottom_up_dp, "bottom-up DP should run without packing-time local segments");
    require(no_local_segment_evaluation.bottom_up_dp.has_value(), "fallback DP should expose a result without local segments");
    require(no_local_segment_evaluation.bottom_up_dp->success, "fallback DP should succeed without local segments");
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
    require(no_trace_evaluation.used_bottom_up_dp, "bottom-up DP should fallback when packing trace is absent");
    require(no_trace_evaluation.bottom_up_dp.has_value(), "fallback DP should still expose a result");
    require(no_trace_evaluation.bottom_up_dp->success, "fallback DP should still succeed on sample input");
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
    require(lcp_detail.detailed_cost > 0.0, "detailed routing should report local detailed cost");

    auto flow_circuit = drc_circuit;
    flow_circuit.constraints.flows.push_back({"N", "M.A", "M.B"});
    const auto flow_ok_detail = sapr::run_detailed_routing(flow_circuit, lcp_request, lcp_eval);
    require(flow_ok_detail.flow_violations == 0, "out-pin to in-pin LCP traceback should satisfy FLOW");

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
    require(coupling_detail.coupling_penalty > 100.0, "coupling penalty should scale with parallel overlap length");
    require(!coupling_detail.report.coupling_pairs.empty(), "coupling report should include net pair and overlap length");
}
