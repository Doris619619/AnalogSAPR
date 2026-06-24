// 实现 linking-control point、space node 与 baseline 链式树构造。
#include "sapr/tree.hpp"

#include <algorithm>

namespace sapr {

// 返回该控制点需要的最大线宽。
double LinkingControlPoint::required_width() const {
    double result = 0.0;
    for (const auto& segment : segments) result = std::max(result, segment.max_width);
    return result;
}

// 按论文公式计算该空间节点需要预留的宽度。
double SpaceNode::required_space() const {
    double result = 0.0;
    for (const auto& point : linking_points) {
        result += point.required_width() * static_cast<double>(std::max<std::size_t>(point.segments.size(), 2)) / 2.0;
    }
    return result;
}

// 按输入顺序构造左孩子链式增强 B*-tree。
EnhancedBStarTree make_chain_tree(const Circuit& circuit) {
    EnhancedBStarTree tree;
    if (!circuit.module_order.empty()) tree.root = circuit.module_order.front();
    for (std::size_t index = 0; index < circuit.module_order.size(); ++index) {
        const auto& module = circuit.module_order[index];
        BStarNode node;
        node.module = module;
        if (index + 1 < circuit.module_order.size()) node.left = circuit.module_order[index + 1];
        node.right_space = {module + ":right", module, "right", {}};
        node.top_space = {module + ":top", module, "top", {}};
        tree.nodes[module] = std::move(node);
    }
    return tree;
}

}  // namespace sapr

