// 声明输入电路与求解结果的约束校验接口。
#pragma once

#include <string>
#include <vector>

#include "sapr/model.hpp"

namespace sapr {

// 校验输入对象之间的引用和数值范围。
std::vector<std::string> validate_circuit(const Circuit& circuit);

// 校验结果是否覆盖全部器件并满足已实现的线宽规则。
std::vector<std::string> validate_solution(const Circuit& circuit, const Solution& solution);

}  // namespace sapr

