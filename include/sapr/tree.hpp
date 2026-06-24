// 声明当前 baseline 使用的增强 B*-tree 构造接口。
#pragma once

#include "sapr/model.hpp"

namespace sapr {

// 按输入顺序构造左孩子链式增强 B*-tree。
EnhancedBStarTree make_chain_tree(const Circuit& circuit);

}  // namespace sapr

