// 文件职责：验证 routing evaluator 可以在样例输入上完成 A*/DP 布线评估。
#include <filesystem>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/routing_evaluator.hpp"
#include "test_support.hpp"

namespace {

// 返回测试使用的输入目录。
std::filesystem::path source_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "input";
}

}  // namespace

// 运行 routing evaluator 的集成测试。
void run_routing_evaluator_tests() {
    const auto circuit = sapr::load_circuit(source_input_dir());
    sapr::SolverConfig config;
    config.sa_iterations = 3;
    const auto solution = sapr::solve_baseline(circuit, config);
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

    const double expected_cost =
        evaluation.global_routing.total_metrics.wirelength +
        3.0 * static_cast<double>(evaluation.global_routing.total_metrics.bend_count) +
        evaluation.global_routing.total_penalty;
    require(approx(evaluation.routing_cost, expected_cost), "global routing cost must not include via count");
}
