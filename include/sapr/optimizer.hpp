// 声明可运行的 baseline 联合布局布线求解器。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 使用链式 packing 和 Manhattan routing 生成基线解。
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config = {});

}  // namespace sapr

