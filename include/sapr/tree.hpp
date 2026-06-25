// 声明论文增强 B*-tree、ASF 对称组和 placement 扰动辅助接口。
#pragma once

#include <random>
#include <vector>

#include "sapr/model.hpp"

namespace sapr {

// 返回 space node 类型的稳定文本名称，供调试和测试使用。
std::string space_kind_name(SpaceNodeKind kind);

// 按输入和约束构造论文增强 B*-tree，普通模块和对称代表进入主树。
EnhancedBStarTree make_enhanced_tree(const Circuit& circuit);

// 按输入顺序构造左孩子链式增强 B*-tree，作为兼容 baseline 的确定性拓扑。
EnhancedBStarTree make_chain_tree(const Circuit& circuit);

// 收集树中全部 space node，作为 routing adapter 的评价输入。
std::vector<SpaceNode> collect_space_nodes(const EnhancedBStarTree& tree);

// 检查增强 B*-tree 是否仍是单根、无环且 parent/child 关系一致。
bool is_valid_tree(const EnhancedBStarTree& tree);

// 对增强 B*-tree 执行一次论文 placement 侧扰动：delete-insert、swap 或 rotate。
void perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng);

}  // namespace sapr
