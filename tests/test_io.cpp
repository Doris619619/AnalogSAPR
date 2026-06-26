// 验证仓库输入解析、注释字段和结果写入回读。
#include "test_support.hpp"

#include <filesystem>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"

// 验证标准样例和 solution round-trip。
void run_io_tests() {
    const std::filesystem::path source = SAPR_SOURCE_DIR;
    const auto circuit = sapr::load_circuit(source / "input");
    require(circuit.modules.size() == 7, "module count should be 7");
    require(circuit.pins.size() == 24, "pin count should be 24");
    require(circuit.nets.size() == 7, "net count should be 7");
    require(circuit.module_order.front() == "M1", "module order should be stable");
    require(approx(circuit.modules.at("M1").ox, -0.3), "M1 ox should be parsed from comment");
    require(approx(circuit.modules.at("M1").oy, -0.21), "M1 oy should be parsed from comment");

    sapr::SolverConfig config;
    config.sa_iterations = 20;
    const auto solution = sapr::solve_baseline(circuit, config);
    const std::filesystem::path roundtrip = std::filesystem::path(SAPR_BINARY_DIR) / "test-roundtrip";
    std::filesystem::remove_all(roundtrip);
    sapr::write_solution(solution, roundtrip);
    const auto loaded = sapr::load_solution(roundtrip);
    require(loaded.placement_order == solution.placement_order, "placement order should survive round-trip");
    require(loaded.routes.size() == solution.routes.size(), "route count should survive round-trip");
    std::filesystem::remove_all(roundtrip);
}
