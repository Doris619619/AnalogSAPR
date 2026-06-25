// 实现论文增强 B*-tree、ASF 对称组摘要、space node 公式和 placement 扰动。
#include "sapr/tree.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "sapr/router.hpp"

namespace sapr {
namespace {

// 返回指定线网的线宽约束，缺省时使用可布线的单位线宽。
std::pair<double, double> width_range_for_net(const Circuit& circuit, const std::string& net) {
    const auto found = circuit.constraints.wire_widths.find(net);
    if (found == circuit.constraints.wire_widths.end()) return {1.0, 1.0};
    return {found->second.min_width, found->second.max_width};
}

// 返回线段在 FLOW 约束中的方向标记，供后续 routing 侧常数时间检查使用。
std::optional<std::string> flow_direction_for_net(const Circuit& circuit, const std::string& net) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net == net) return flow.out_pin + "->" + flow.in_pin;
    }
    return std::nullopt;
}

// 为模块创建论文中的右侧和上侧 space node。
BStarNode make_node(const Circuit& circuit, const std::string& module) {
    BStarNode node;
    node.module = module;
    node.right_space = {module + ":right", module, SpaceNodeKind::Right, {}};
    node.top_space = {module + ":top", module, SpaceNodeKind::Top, {}};
    for (const auto& net_name : circuit.net_order) {
        const auto& net = circuit.nets.at(net_name);
        bool touches_module = false;
        for (const auto& terminal : net.terminals) {
            const auto dot = terminal.find('.');
            if (dot != std::string::npos && terminal.substr(0, dot) == module) {
                touches_module = true;
                break;
            }
        }
        if (!touches_module || net.terminals.size() < 2) continue;
        const auto [min_width, max_width] = width_range_for_net(circuit, net_name);
        LinkingControlPoint point{module + ":" + net_name + ":lcp", node.right_space.id,
                                  {{net_name, min_width, max_width, flow_direction_for_net(circuit, net_name)}}};
        node.right_space.linking_points.push_back(std::move(point));
    }
    return node;
}

// 将节点作为父节点的左或右孩子接入，并维护 parent 反向关系。
void attach_child(EnhancedBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    if (as_left) {
        tree.nodes.at(parent).left = child;
    } else {
        tree.nodes.at(parent).right = child;
    }
    tree.nodes.at(child).parent = parent;
}

// 根据代表节点顺序重建一棵交替使用左/右孩子的确定性树。
void rebuild_balanced_chain(EnhancedBStarTree& tree) {
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        node.parent.reset();
        node.left.reset();
        node.right.reset();
    }
    tree.root.reset();
    if (tree.representative_order.empty()) return;
    tree.root = tree.representative_order.front();
    for (std::size_t index = 1; index < tree.representative_order.size(); ++index) {
        const std::string& parent = tree.representative_order[(index - 1) / 2];
        const std::string& child = tree.representative_order[index];
        attach_child(tree, parent, child, index % 2 == 1);
    }
}

// 收集从 root 可达的代表节点数量。
std::size_t reachable_count(const EnhancedBStarTree& tree) {
    if (!tree.root.has_value()) return 0;
    std::unordered_set<std::string> seen;
    std::function<void(const std::string&)> visit = [&](const std::string& id) {
        if (seen.contains(id)) return;
        seen.insert(id);
        const auto& node = tree.nodes.at(id);
        if (node.left.has_value()) visit(*node.left);
        if (node.right.has_value()) visit(*node.right);
    };
    visit(*tree.root);
    return seen.size();
}

}  // namespace

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

// 返回 space node 类型的稳定文本名称，供调试和测试使用。
std::string space_kind_name(SpaceNodeKind kind) {
    switch (kind) {
        case SpaceNodeKind::Right: return "right";
        case SpaceNodeKind::Top: return "top";
        case SpaceNodeKind::Group: return "group";
        case SpaceNodeKind::Cluster: return "cluster";
    }
    return "unknown";
}

// 按输入和约束构造论文增强 B*-tree，普通模块和对称代表进入主树。
EnhancedBStarTree make_enhanced_tree(const Circuit& circuit) {
    EnhancedBStarTree tree;
    std::unordered_set<std::string> mirrored_modules;
    for (const auto& pair : circuit.constraints.symmetry_pairs) mirrored_modules.insert(pair.right);

    for (const auto& module : circuit.module_order) {
        if (mirrored_modules.contains(module)) continue;
        tree.nodes[module] = make_node(circuit, module);
        tree.representative_order.push_back(module);
    }

    for (const auto& pair : circuit.constraints.symmetry_pairs) {
        tree.symmetry_groups.push_back({pair.name,
                                        pair.axis,
                                        pair.left,
                                        pair.right,
                                        false,
                                        {pair.name + ":space_group", pair.left, SpaceNodeKind::Group, {}},
                                        {pair.name + ":space_cluster", pair.left, SpaceNodeKind::Cluster, {}}});
    }
    for (const auto& self : circuit.constraints.symmetry_selfs) {
        tree.symmetry_groups.push_back({self.name,
                                        self.axis,
                                        self.module,
                                        std::nullopt,
                                        true,
                                        {self.name + ":space_group", self.module, SpaceNodeKind::Group, {}},
                                        {self.name + ":space_cluster", self.module, SpaceNodeKind::Cluster, {}}});
    }

    rebuild_balanced_chain(tree);
    return tree;
}

// 按输入顺序构造左孩子链式增强 B*-tree，作为兼容 baseline 的确定性拓扑。
EnhancedBStarTree make_chain_tree(const Circuit& circuit) {
    EnhancedBStarTree tree;
    for (const auto& module : circuit.module_order) {
        tree.nodes[module] = make_node(circuit, module);
        tree.representative_order.push_back(module);
    }
    if (!tree.representative_order.empty()) tree.root = tree.representative_order.front();
    for (std::size_t index = 1; index < tree.representative_order.size(); ++index) {
        attach_child(tree, tree.representative_order[index - 1], tree.representative_order[index], true);
    }
    return tree;
}

// 收集树中全部 space node，作为 routing adapter 的评价输入。
std::vector<SpaceNode> collect_space_nodes(const EnhancedBStarTree& tree) {
    std::vector<SpaceNode> result;
    for (const auto& id : tree.representative_order) {
        const auto& node = tree.nodes.at(id);
        result.push_back(node.right_space);
        result.push_back(node.top_space);
    }
    for (const auto& group : tree.symmetry_groups) {
        result.push_back(group.space_group);
        result.push_back(group.space_cluster);
    }
    return result;
}

// 检查增强 B*-tree 是否仍是单根、无环且 parent/child 关系一致。
bool is_valid_tree(const EnhancedBStarTree& tree) {
    if (tree.representative_order.empty()) return !tree.root.has_value();
    if (!tree.root.has_value() || !tree.nodes.contains(*tree.root)) return false;
    if (tree.nodes.at(*tree.root).parent.has_value()) return false;
    for (const auto& id : tree.representative_order) {
        if (!tree.nodes.contains(id)) return false;
        const auto& node = tree.nodes.at(id);
        for (const auto& child : {node.left, node.right}) {
            if (!child.has_value()) continue;
            if (!tree.nodes.contains(*child)) return false;
            if (tree.nodes.at(*child).parent != id) return false;
        }
    }
    return reachable_count(tree) == tree.representative_order.size();
}

// 对增强 B*-tree 执行一次论文 placement 侧扰动：delete-insert、swap 或 rotate。
void perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng) {
    if (tree.representative_order.empty()) return;
    std::uniform_int_distribution<int> move_dist(0, 2);
    const int move = move_dist(rng);
    if (move == 0 && tree.representative_order.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, tree.representative_order.size() - 1);
        const auto first = index_dist(rng);
        auto second = index_dist(rng);
        if (first == second) second = (second + 1) % tree.representative_order.size();
        std::swap(tree.representative_order[first], tree.representative_order[second]);
        rebuild_balanced_chain(tree);
    } else if (move == 1 && tree.representative_order.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, tree.representative_order.size() - 1);
        const auto from = index_dist(rng);
        auto to = index_dist(rng);
        if (from == to) to = (to + 1) % tree.representative_order.size();
        const auto id = tree.representative_order[from];
        tree.representative_order.erase(tree.representative_order.begin() + static_cast<std::ptrdiff_t>(from));
        tree.representative_order.insert(tree.representative_order.begin() + static_cast<std::ptrdiff_t>(to), id);
        rebuild_balanced_chain(tree);
    } else {
        std::uniform_int_distribution<std::size_t> index_dist(0, tree.representative_order.size() - 1);
        auto& node = tree.nodes.at(tree.representative_order[index_dist(rng)]);
        node.angle = (node.angle + 90) % 360;
    }
    if (!is_valid_tree(tree)) throw std::runtime_error("invalid enhanced B*-tree after perturbation");
}

}  // namespace sapr
