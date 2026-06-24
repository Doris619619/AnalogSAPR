// 验证缺失引用、FLOW 和线宽范围等输入错误。
#include "test_support.hpp"

#include <filesystem>

#include "sapr/constraints.hpp"
#include "sapr/io.hpp"

// 验证合法样例和典型非法引用均能得到预期报告。
void run_constraint_tests() {
    const auto circuit = sapr::load_circuit(std::filesystem::path(SAPR_SOURCE_DIR) / "input");
    require(sapr::validate_circuit(circuit).empty(), "sample circuit should be valid");

    auto missing_pin = circuit;
    missing_pin.nets.at("VDD").terminals.push_back("M9.X");
    const auto pin_errors = sapr::validate_circuit(missing_pin);
    require(!pin_errors.empty(), "missing pin should be reported");

    auto invalid_width = circuit;
    require(invalid_width.constraints.wire_widths.contains("VDD"), "sample should contain VDD width constraint");
    invalid_width.constraints.wire_widths.at("VDD").min_width = 10.0;
    invalid_width.constraints.wire_widths.at("VDD").max_width = 2.0;
    const auto width_errors = sapr::validate_circuit(invalid_width);
    require(!width_errors.empty(), "invalid width range should be reported");

    auto invalid_flow = circuit;
    require(!invalid_flow.constraints.flows.empty(), "sample should contain flow constraints");
    invalid_flow.constraints.flows.front().out_pin = "M2.G";
    const auto flow_errors = sapr::validate_circuit(invalid_flow);
    require(!flow_errors.empty(), "flow pin outside net should be reported");
}
