// 文件职责：声明自动 LCP 拓扑生成、space 绑定和物理候选点生成入口。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 自动生成 placement-aware LCP 拓扑和候选点，并写入 routing request。
void generate_automatic_lcps(const Circuit& circuit, RoutingEvaluationRequest& request);

}  // namespace sapr
