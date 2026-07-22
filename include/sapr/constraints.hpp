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

// 返回器件之间必须保留的最小边缘距离。
double device_spacing(const Circuit& circuit);
// 返回指定金属层 active 到布线的最小边缘距离。
double active_route_spacing(const Circuit& circuit, const std::string& layer);
// 返回指定金属层异网金属之间的最小边缘距离。
double diff_net_route_spacing(const Circuit& circuit, const std::string& layer);
// 判断指定金属层是否禁止跨越 active；允许跨越时同时豁免 active 间距。
bool active_region_blocked(const Circuit& circuit, const std::string& layer);

}  // namespace sapr

