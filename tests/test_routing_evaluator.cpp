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
    const auto solution = sapr::solve_baseline(circuit);
    const auto evaluation = sapr::evaluate_routing(circuit, solution.placements);

    require(!evaluation.candidates.empty(), "routing evaluator should emit A* route candidates");
    require(evaluation.failed_nets == 0, "routing evaluator should route all nets in the sample input");
    require(evaluation.routing_cost > 0.0, "routing evaluator should produce a positive global routing cost");

    const double expected_cost =
        evaluation.global_routing.total_metrics.wirelength +
        3.0 * static_cast<double>(evaluation.global_routing.total_metrics.bend_count) +
        evaluation.global_routing.total_penalty;
    require(approx(evaluation.routing_cost, expected_cost), "global routing cost must not include via count");
}
