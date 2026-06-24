// 声明 baseline Manhattan 布线和指标计算接口。
#pragma once

#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"

namespace sapr {

// 返回线网的约束中值线宽或默认线宽。
double default_width(const Circuit& circuit, const std::string& net_name);

// 根据放置结果生成确定性的 Manhattan 布线。
std::vector<RouteSegment> route_manhattan(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements);

// 计算面积、线长、bend 和 via 等基础指标。
Metrics measure(const Circuit& circuit, const Solution& solution);

}  // namespace sapr

