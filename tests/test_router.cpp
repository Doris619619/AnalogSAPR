// 验证旋转几何、链式树、Manhattan routing 和基线指标。
#include "test_support.hpp"

#include <filesystem>

#include "sapr/geometry.hpp"
#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/router.hpp"
#include "sapr/tree.hpp"

// 验证关键几何和算法结果保持迁移前语义。
void run_router_tests() {
    const auto circuit = sapr::load_circuit(std::filesystem::path(SAPR_SOURCE_DIR) / "input");
    const auto& module = circuit.modules.at("M1");
    const auto& pin = circuit.pins.at("M1.G");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 0, "R0"}) == std::pair<double, double>{0.8, 1.5}, "R0 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 90, "R90"}) == std::pair<double, double>{1.5, 0.8}, "R90 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 180, "R180"}) == std::pair<double, double>{3.2, 1.5}, "R180 pin transform failed");
    require(sapr::placed_pin(module, pin, {"M1", 0, 0, 270, "R270"}) == std::pair<double, double>{1.5, 3.2}, "R270 pin transform failed");

    const auto tree = sapr::make_chain_tree(circuit);
    require(tree.root == std::optional<std::string>{"M1"}, "chain root should be M1");
    require(tree.nodes.at("M1").left == std::optional<std::string>{"M2"}, "chain left child should preserve order");
    require(tree.nodes.at("M1").right_space.id == "M1:right", "right space should be created");

    sapr::SpaceNode space{"space", "M1", "right", {{"p", "space", {{"N", 1.0, 4.0, std::nullopt}, {"N", 1.0, 4.0, std::nullopt}}}}};
    require(approx(space.required_space(), 4.0), "space formula should match paper equation");

    const auto solution = sapr::solve_baseline(circuit);
    for (const auto& route : solution.routes) {
        require(route.x1 != route.x2 || route.y1 != route.y2, "zero-length route should be filtered");
    }
    require(approx(sapr::default_width(circuit, "VDD"), 5.0), "constrained width should use midpoint");
    require(approx(sapr::default_width(circuit, "OUT"), 1.0), "unconstrained width should be one");

    const auto metrics = sapr::measure(circuit, solution);
    require(approx(metrics.area, 2103.2), "baseline area changed");
    require(approx(metrics.wirelength, 136.82), "baseline wirelength changed");
    require(metrics.bend_count == 1, "baseline bend count changed");
    require(metrics.via_count == 1, "baseline via count changed");
    require(approx(metrics.penalty, 0.0), "baseline penalty changed");
}

