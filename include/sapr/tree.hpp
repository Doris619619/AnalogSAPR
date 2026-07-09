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

// 统计当前树状态中持久化保存的 LCP 数量。
std::size_t count_tree_lcps(const EnhancedBStarTree& tree);

// 判断普通模块主树中是否存在 right child，用于验证二维 B*-tree 搜索自由度。
bool has_ordinary_right_child(const EnhancedBStarTree& tree);

// 检查增强 B*-tree 是否仍是单根、无环且 parent/child 关系一致。
bool is_valid_tree(const EnhancedBStarTree& tree);

// 检查所有自对称代表是否位于主树的 right-most branch。
bool self_symmetry_on_rightmost_branch(const EnhancedBStarTree& tree);

// 将 routing adapter 返回的资源预留反馈写回对应 space node。
void apply_routing_feedback(EnhancedBStarTree& tree, const RoutingFeedback& feedback);

// 对增强 B*-tree 执行一次论文 placement 侧扰动：delete-insert、swap 或 rotate。
PerturbationReport perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng);

}  // namespace sapr
