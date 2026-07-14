// 文件职责：实现 enhanced B*-tree、vertical ASF-B*-tree、space node、LCP 扰动和 placement 扰动。
#include "sapr/tree.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace sapr {
namespace {

// 构造一个 concrete space node，供普通 B*-tree 与 ASF space node 容器复用。
SpaceNode make_space_node(const std::string& id, const std::string& owner, SpaceNodeKind kind) {
    return {id, owner, kind, {}, 0.0, {}, 0.0, {}};
}

// 构造全局 enhanced B*-tree 中的普通模块节点，并初始化右侧/上方 space node。
BStarNode make_module_node(const std::string& module) {
    BStarNode node;
    node.id = module;
    node.kind = BStarNodeKind::Module;
    node.module = module;
    node.right_space = make_space_node(module + ":right", module, SpaceNodeKind::Right);
    node.top_space = make_space_node(module + ":top", module, SpaceNodeKind::Top);
    return node;
}

// 构造全局 enhanced B*-tree 中代表 symmetry island 的层次节点。
BStarNode make_hierarchy_node(const std::string& group_name) {
    BStarNode node;
    node.id = group_name;
    node.kind = BStarNodeKind::Hierarchy;
    node.hierarchy_group = group_name;
    return node;
}

// 构造论文中的 outer space_node_group，分别描述 representative 外侧和 mirror 外侧空间。
SpaceNodeGroup make_space_node_group_outer(
    const std::string& group_name,
    const std::string& module,
    const std::optional<std::string>& mirror) {
    SpaceNodeGroup group;
    group.name = group_name + "/" + module + ":space_node_group_outer";
    group.spaces.push_back(make_space_node(group.name + ":representative_right", module, SpaceNodeKind::Group));
    group.spaces.push_back(make_space_node(group.name + ":mirror_left", mirror.value_or(module), SpaceNodeKind::Group));
    return group;
}

// 构造论文中的 top space_node_group，分别描述 representative 与 mirror 上方空间。
SpaceNodeGroup make_space_node_group_top(
    const std::string& group_name,
    const std::string& module,
    const std::optional<std::string>& mirror) {
    SpaceNodeGroup group;
    group.name = group_name + "/" + module + ":space_node_group_top";
    group.spaces.push_back(make_space_node(group.name + ":representative_top", module, SpaceNodeKind::Group));
    group.spaces.push_back(make_space_node(group.name + ":mirror_top", mirror.value_or(module), SpaceNodeKind::Group));
    return group;
}

// 构造 right-most branch 上的 space_node_cluster，包含靠对称轴空间和上方空间。
SpaceNodeCluster make_space_node_cluster(
    const std::string& group_name,
    const std::string& module,
    const std::optional<std::string>& mirror) {
    SpaceNodeCluster cluster;
    cluster.name = group_name + "/" + module + ":space_node_cluster";
    cluster.spaces.push_back(make_space_node(cluster.name + ":representative_axis", module, SpaceNodeKind::Cluster));
    cluster.spaces.push_back(make_space_node(cluster.name + ":mirror_axis", mirror.value_or(module), SpaceNodeKind::Cluster));
    cluster.spaces.push_back(make_space_node(cluster.name + ":representative_top", module, SpaceNodeKind::Cluster));
    cluster.spaces.push_back(make_space_node(cluster.name + ":mirror_top", mirror.value_or(module), SpaceNodeKind::Cluster));
    return cluster;
}

// 构造 ASF-B*-tree 节点；mirror 只记录映射关系，不直接进入树结构。
AsfBStarNode make_asf_node(
    const std::string& group_name,
    const std::string& module,
    const std::optional<std::string>& mirror,
    bool self_symmetric) {
    AsfBStarNode node;
    node.module = module;
    node.mirror_module = mirror;
    node.is_self_symmetric = self_symmetric;
    node.space_node_groups.push_back(make_space_node_group_outer(group_name, module, mirror));
    node.space_node_groups.push_back(make_space_node_group_top(group_name, module, mirror));
    return node;
}

// 连接全局 enhanced B*-tree 的父子节点，并同步 parent 指针。
void attach_child(EnhancedBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    if (as_left) {
        tree.nodes.at(parent).left = child;
    } else {
        tree.nodes.at(parent).right = child;
    }
    tree.nodes.at(child).parent = parent;
}

// 连接 ASF-B*-tree 的父子节点，并同步 parent 指针。
void attach_child(AsfBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    if (as_left) {
        tree.nodes.at(parent).left = child;
    } else {
        tree.nodes.at(parent).right = child;
    }
    tree.nodes.at(child).parent = parent;
}

// 按当前拓扑刷新全局树的遍历顺序，作为后续扰动和重建的稳定顺序。
void refresh_representative_order(EnhancedBStarTree& tree) {
    tree.representative_order.clear();
    if (!tree.root.has_value()) return;
    std::unordered_set<std::string> seen;
    std::function<void(const std::string&)> visit = [&](const std::string& id) {
        if (!tree.nodes.contains(id) || !seen.insert(id).second) return;
        tree.representative_order.push_back(id);
        const auto& node = tree.nodes.at(id);
        if (node.left.has_value()) visit(*node.left);
        if (node.right.has_value()) visit(*node.right);
    };
    visit(*tree.root);
}

// 按当前拓扑刷新 ASF 树的 representative/self 节点顺序。
void refresh_representative_order(AsfBStarTree& tree) {
    tree.representative_order.clear();
    if (!tree.root.has_value()) return;
    std::unordered_set<std::string> seen;
    std::function<void(const std::string&)> visit = [&](const std::string& id) {
        if (!tree.nodes.contains(id) || !seen.insert(id).second) return;
        tree.representative_order.push_back(id);
        const auto& node = tree.nodes.at(id);
        if (node.left.has_value()) visit(*node.left);
        if (node.right.has_value()) visit(*node.right);
    };
    visit(*tree.root);
}

// 刷新 ASF 树中从 root 沿 right child 向下的 right-most branch。
void refresh_right_most_branch(AsfBStarTree& tree) {
    tree.right_most_branch.clear();
    auto current = tree.root;
    while (current.has_value() && tree.nodes.contains(*current)) {
        tree.right_most_branch.push_back(*current);
        current = tree.nodes.at(*current).right;
    }
}

/* 将离开贴轴分支的 cluster LCP 转移到同一 ASF 节点仍存在的 outer/top space，避免 SA 扰动丢失拓扑端点。 */
void migrate_cluster_lcps_to_groups(AsfBStarNode& node) {
    if (!node.space_node_cluster.has_value() || node.space_node_groups.size() < 2) return;
    auto& cluster = *node.space_node_cluster;
    if (cluster.spaces.size() < 4 || node.space_node_groups[0].spaces.size() < 2 ||
        node.space_node_groups[1].spaces.size() < 2) {
        return;
    }

    const std::array<std::pair<std::size_t, std::size_t>, 4> target_indices{{
        {0, 0},  /* 代表侧轴间空间转移到代表侧外部空间。 */
        {0, 1},  /* 镜像侧轴间空间转移到镜像侧外部空间。 */
        {1, 0},  /* 代表侧顶部 cluster 空间转移到代表侧顶部 group 空间。 */
        {1, 1},  /* 镜像侧顶部 cluster 空间转移到镜像侧顶部 group 空间。 */
    }};
    for (std::size_t index = 0; index < target_indices.size(); ++index) {
        const auto [group_index, space_index] = target_indices[index];
        auto& target = node.space_node_groups[group_index].spaces[space_index];
        for (auto& point : cluster.spaces[index].linking_points) {
            point.space_node_id = target.id;
            target.linking_points.push_back(std::move(point));
        }
    }
}

/* 根据 right-most branch 维护 space_node_cluster，保证 cluster 只存在于贴轴分支。 */
void refresh_space_node_clusters(AsfBStarTree& tree) {
    std::unordered_set<std::string> right_branch(tree.right_most_branch.begin(), tree.right_most_branch.end());
    for (auto& [id, node] : tree.nodes) {
        if (right_branch.contains(id)) {
            if (!node.space_node_cluster.has_value()) {
                node.space_node_cluster = make_space_node_cluster(tree.group_name, node.module, node.mirror_module);
            }
        } else {
            migrate_cluster_lcps_to_groups(node);
            node.space_node_cluster.reset();
        }
    }
}

// 根据输入顺序重建 ASF-B*-tree，并强制 self-symmetric 节点位于 right-most branch。
void rebuild_asf_bstar_tree(AsfBStarTree& tree) {
    const auto old_order = tree.representative_order;
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        node.parent.reset();
        node.left.reset();
        node.right.reset();
    }
    tree.root.reset();

    std::vector<std::string> ordinary;
    std::vector<std::string> self_nodes;
    for (const auto& id : old_order) {
        if (!tree.nodes.contains(id)) continue;
        if (tree.nodes.at(id).is_self_symmetric) {
            self_nodes.push_back(id);
        } else {
            ordinary.push_back(id);
        }
    }
    if (ordinary.empty() && self_nodes.empty()) return;

    tree.root = ordinary.empty() ? self_nodes.front() : ordinary.front();
    if (ordinary.size() > 1) {
        const std::size_t split = (ordinary.size() + 1) / 2;
        if (split > 1) {
            attach_child(tree, ordinary.front(), ordinary[1], true);
            for (std::size_t index = 2; index < split; ++index) {
                attach_child(tree, ordinary[index - 1], ordinary[index], true);
            }
        }
        if (split < ordinary.size()) {
            attach_child(tree, ordinary.front(), ordinary[split], false);
            for (std::size_t index = split + 1; index < ordinary.size(); ++index) {
                attach_child(tree, ordinary[index - 1], ordinary[index], true);
            }
        }
    }

    std::string right_parent = *tree.root;
    while (tree.nodes.at(right_parent).right.has_value()) right_parent = *tree.nodes.at(right_parent).right;
    for (const auto& self : self_nodes) {
        if (self == right_parent) continue;
        attach_child(tree, right_parent, self, false);
        right_parent = self;
    }
    refresh_representative_order(tree);
    refresh_right_most_branch(tree);
    refresh_space_node_clusters(tree);
}

// 根据 representative_order 重建全局 enhanced B*-tree 的基础拓扑。
void rebuild_global_bstar_tree(EnhancedBStarTree& tree) {
    const auto order = tree.representative_order;
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        node.parent.reset();
        node.left.reset();
        node.right.reset();
    }
    tree.root.reset();
    if (order.empty()) return;
    tree.root = order.front();
    if (order.size() > 1) {
        const std::size_t split = (order.size() + 1) / 2;
        if (split > 1) {
            attach_child(tree, order.front(), order[1], true);
            for (std::size_t index = 2; index < split; ++index) {
                attach_child(tree, order[index - 1], order[index], true);
            }
        }
        if (split < order.size()) {
            attach_child(tree, order.front(), order[split], false);
            for (std::size_t index = split + 1; index < order.size(); ++index) {
                attach_child(tree, order[index - 1], order[index], true);
            }
        }
    }
    refresh_representative_order(tree);
}

// 检查 ASF 树中所有 self-symmetric 节点是否都位于 right-most branch。
bool asf_selfs_on_right_most_branch(const AsfBStarTree& tree) {
    std::unordered_set<std::string> right_branch(tree.right_most_branch.begin(), tree.right_most_branch.end());
    for (const auto& self : tree.self_nodes) {
        if (!right_branch.contains(self)) return false;
    }
    return true;
}

// 校验 ASF-B*-tree 的连通性、parent/child 一致性和 self-symmetric branch 约束。
bool is_valid_asf_tree(const AsfBStarTree& tree) {
    if (tree.nodes.empty()) return !tree.root.has_value() && tree.representative_order.empty();
    if (!tree.root.has_value() || !tree.nodes.contains(*tree.root)) return false;
    if (tree.nodes.at(*tree.root).parent.has_value()) return false;
    std::unordered_set<std::string> seen;
    std::function<bool(const std::string&)> visit = [&](const std::string& id) {
        if (!tree.nodes.contains(id) || !seen.insert(id).second) return false;
        const auto& node = tree.nodes.at(id);
        for (const auto& child : {node.left, node.right}) {
            if (!child.has_value()) continue;
            if (!tree.nodes.contains(*child)) return false;
            if (tree.nodes.at(*child).parent != id) return false;
            if (!visit(*child)) return false;
        }
        return true;
    };
    if (!visit(*tree.root) || seen.size() != tree.nodes.size()) return false;
    return asf_selfs_on_right_most_branch(tree);
}

// 从 ASF 树中摘除节点，并把原有子树重新接回，供 delete-insert 扰动使用。
void detach_node_preserving_children(AsfBStarTree& tree, const std::string& id) {
    auto& node = tree.nodes.at(id);
    const auto parent = node.parent;
    const auto left = node.left;
    const auto right = node.right;
    std::optional<std::string> replacement;
    if (left.has_value()) {
        replacement = left;
        tree.nodes.at(*left).parent = parent;
        if (right.has_value()) {
            std::string tail = *left;
            while (tree.nodes.at(tail).right.has_value()) tail = *tree.nodes.at(tail).right;
            tree.nodes.at(tail).right = right;
            tree.nodes.at(*right).parent = tail;
        }
    } else if (right.has_value()) {
        replacement = right;
        tree.nodes.at(*right).parent = parent;
    }

    if (parent.has_value()) {
        auto& parent_node = tree.nodes.at(*parent);
        if (parent_node.left == id) parent_node.left = replacement;
        if (parent_node.right == id) parent_node.right = replacement;
    } else {
        tree.root = replacement;
    }
    node.parent.reset();
    node.left.reset();
    node.right.reset();
}

// 将 ASF 树节点插入到指定父节点的 left/right child 位置。
void insert_node_at_child(AsfBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    auto& child_node = tree.nodes.at(child);
    child_node.parent.reset();
    child_node.left.reset();
    child_node.right.reset();
    auto& parent_node = tree.nodes.at(parent);
    auto displaced = as_left ? parent_node.left : parent_node.right;
    if (as_left) {
        parent_node.left = child;
        child_node.left = displaced;
    } else {
        parent_node.right = child;
        child_node.right = displaced;
    }
    child_node.parent = parent;
    if (displaced.has_value()) tree.nodes.at(*displaced).parent = child;
}

// 判断 ASF 树中 candidate 是否为 target 的祖先，避免扰动形成环。
bool is_ancestor_of(const AsfBStarTree& tree, const std::string& candidate, const std::string& target) {
    auto current = tree.nodes.at(target).parent;
    while (current.has_value()) {
        if (*current == candidate) return true;
        current = tree.nodes.at(*current).parent;
    }
    return false;
}

// 交换 ASF 树中两个节点的位置；祖先关系下只交换旋转角以保持拓扑合法。
void swap_tree_positions(AsfBStarTree& tree, const std::string& first, const std::string& second) {
    if (first == second) return;
    if (is_ancestor_of(tree, first, second) || is_ancestor_of(tree, second, first)) {
        std::swap(tree.nodes.at(first).angle, tree.nodes.at(second).angle);
        return;
    }

    const auto first_parent = tree.nodes.at(first).parent;
    const auto second_parent = tree.nodes.at(second).parent;
    if (!first_parent.has_value() || !second_parent.has_value()) {
        std::swap(tree.nodes.at(first).angle, tree.nodes.at(second).angle);
        return;
    }
    const bool first_as_left = tree.nodes.at(*first_parent).left == first;
    const bool second_as_left = tree.nodes.at(*second_parent).left == second;
    detach_node_preserving_children(tree, first);
    detach_node_preserving_children(tree, second);
    if (tree.nodes.contains(*second_parent)) insert_node_at_child(tree, *second_parent, first, second_as_left);
    if (tree.nodes.contains(*first_parent)) insert_node_at_child(tree, *first_parent, second, first_as_left);
}

// 从全局 enhanced B*-tree 中摘除节点，并保持剩余子树连通。
void detach_node_preserving_children(EnhancedBStarTree& tree, const std::string& id) {
    auto& node = tree.nodes.at(id);
    const auto parent = node.parent;
    const auto left = node.left;
    const auto right = node.right;
    std::optional<std::string> replacement;
    if (left.has_value()) {
        replacement = left;
        tree.nodes.at(*left).parent = parent;
        if (right.has_value()) {
            std::string tail = *left;
            while (tree.nodes.at(tail).right.has_value()) tail = *tree.nodes.at(tail).right;
            tree.nodes.at(tail).right = right;
            tree.nodes.at(*right).parent = tail;
        }
    } else if (right.has_value()) {
        replacement = right;
        tree.nodes.at(*right).parent = parent;
    }

    if (parent.has_value()) {
        auto& parent_node = tree.nodes.at(*parent);
        if (parent_node.left == id) parent_node.left = replacement;
        if (parent_node.right == id) parent_node.right = replacement;
    } else {
        tree.root = replacement;
    }
    node.parent.reset();
    node.left.reset();
    node.right.reset();
}

// 将全局树节点插入到指定父节点的 left/right child 位置。
void insert_node_at_child(EnhancedBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    auto& child_node = tree.nodes.at(child);
    child_node.parent.reset();
    child_node.left.reset();
    child_node.right.reset();
    auto& parent_node = tree.nodes.at(parent);
    auto displaced = as_left ? parent_node.left : parent_node.right;
    if (as_left) {
        parent_node.left = child;
        child_node.left = displaced;
    } else {
        parent_node.right = child;
        child_node.right = displaced;
    }
    child_node.parent = parent;
    if (displaced.has_value()) tree.nodes.at(*displaced).parent = child;
}

// 判断全局树中 candidate 是否为 target 的祖先，避免扰动形成环。
bool is_ancestor_of(const EnhancedBStarTree& tree, const std::string& candidate, const std::string& target) {
    auto current = tree.nodes.at(target).parent;
    while (current.has_value()) {
        if (*current == candidate) return true;
        current = tree.nodes.at(*current).parent;
    }
    return false;
}

// 交换全局 enhanced B*-tree 中两个节点的位置。
void swap_tree_positions(EnhancedBStarTree& tree, const std::string& first, const std::string& second) {
    if (first == second) return;
    if (is_ancestor_of(tree, first, second) || is_ancestor_of(tree, second, first)) {
        std::swap(tree.nodes.at(first).angle, tree.nodes.at(second).angle);
        return;
    }

    const auto first_parent = tree.nodes.at(first).parent;
    const auto second_parent = tree.nodes.at(second).parent;
    if (!first_parent.has_value() || !second_parent.has_value()) {
        std::swap(tree.nodes.at(first).angle, tree.nodes.at(second).angle);
        return;
    }
    const bool first_as_left = tree.nodes.at(*first_parent).left == first;
    const bool second_as_left = tree.nodes.at(*second_parent).left == second;
    detach_node_preserving_children(tree, first);
    detach_node_preserving_children(tree, second);
    if (tree.nodes.contains(*second_parent)) insert_node_at_child(tree, *second_parent, first, second_as_left);
    if (tree.nodes.contains(*first_parent)) insert_node_at_child(tree, *first_parent, second, first_as_left);
}

// 统计从 root 可达的全局树节点数，用于树合法性校验。
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

// 给匹配的 space node 写入 routing resource 反馈；基础预留与耦合附加空间保持分离。
bool update_space_node(SpaceNode& space, const RoutingFeedback& feedback) {
    const auto found = feedback.required_space_by_node.find(space.id);
    const auto coupling_found = feedback.coupling_space_by_node.find(space.id);
    bool updated = false;
    if (found != feedback.required_space_by_node.end()) {
        space.allocated_space = std::max(space.allocated_space, found->second);
        updated = true;
    }
    if (coupling_found != feedback.coupling_space_by_node.end()) {
        space.coupling_extra_space = std::max(space.coupling_extra_space, coupling_found->second);
        updated = true;
    }
    return updated;
}

// 遍历 ASF 节点内所有 concrete space node，包括 group 和 cluster 内部空间。
void for_each_asf_space(AsfBStarNode& node, const std::function<void(SpaceNode&)>& visit) {
    for (auto& group : node.space_node_groups) {
        for (auto& space : group.spaces) visit(space);
    }
    if (node.space_node_cluster.has_value()) {
        for (auto& space : node.space_node_cluster->spaces) visit(space);
    }
}

// 只读遍历 ASF 节点内所有 concrete space node。
void for_each_asf_space(const AsfBStarNode& node, const std::function<void(const SpaceNode&)>& visit) {
    for (const auto& group : node.space_node_groups) {
        for (const auto& space : group.spaces) visit(space);
    }
    if (node.space_node_cluster.has_value()) {
        for (const auto& space : node.space_node_cluster->spaces) visit(space);
    }
}

// 收集全局树中可参与 module perturbation 的节点。
std::vector<std::string> movable_nodes(const EnhancedBStarTree& tree) {
    std::vector<std::string> result;
    for (const auto& id : tree.representative_order) {
        if (tree.nodes.contains(id)) result.push_back(id);
    }
    return result;
}

// 收集 ASF 树中可参与内部扰动的 representative/self 节点。
std::vector<std::string> movable_nodes(const AsfBStarTree& tree) {
    std::vector<std::string> result;
    for (const auto& id : tree.representative_order) {
        if (tree.nodes.contains(id)) result.push_back(id);
    }
    return result;
}

// ASF 内部扰动后刷新派生结构，并验证 right-most branch 等约束。
bool finalize_asf_perturbation(AsfBStarTree& tree) {
    refresh_representative_order(tree);
    refresh_right_most_branch(tree);
    refresh_space_node_clusters(tree);
    return is_valid_asf_tree(tree);
}

// 执行 ASF-B*-tree 内部扰动，只移动 representative/self 节点而不直接移动 mirror。
bool perturb_asf_bstar_tree(AsfBStarTree& tree, std::mt19937& rng, std::string& move) {
    const auto movable = movable_nodes(tree);
    if (movable.empty()) return false;
    AsfBStarTree before = tree;
    std::uniform_int_distribution<int> move_dist(0, 2);
    const int selected = move_dist(rng);
    bool changed = false;
    if (selected == 0 && movable.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        const auto first = movable[index_dist(rng)];
        auto second = movable[index_dist(rng)];
        if (first == second) second = movable[(index_dist(rng) + 1) % movable.size()];
        swap_tree_positions(tree, first, second);
        move = "asf-module-swap";
        changed = true;
    } else if (selected == 1 && movable.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        const auto id = movable[index_dist(rng)];
        std::vector<std::string> targets;
        for (const auto& candidate : movable) {
            if (candidate != id && !is_ancestor_of(tree, id, candidate)) targets.push_back(candidate);
        }
        if (!targets.empty()) {
            std::uniform_int_distribution<std::size_t> target_dist(0, targets.size() - 1);
            std::uniform_int_distribution<int> side_dist(0, 1);
            detach_node_preserving_children(tree, id);
            insert_node_at_child(tree, targets[target_dist(rng)], id, side_dist(rng) == 0);
            move = "asf-module-delete-insert";
            changed = true;
        }
    } else {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        auto& node = tree.nodes.at(movable[index_dist(rng)]);
        node.angle = (node.angle + 90) % 360;
        move = "asf-module-rotate";
        changed = true;
    }
    if (!changed) return false;
    if (!finalize_asf_perturbation(tree)) {
        tree = std::move(before);
        return false;
    }
    return true;
}

// 暂存 symmetry group 构建状态，保留输入顺序用于生成 ASF 树。
struct GroupBuildState {
    SymmetryGroupNode group;
    std::vector<std::string> input_order;
};

}  // namespace

// 返回 LCP 内所有 segment 的最大线宽，用于论文中的 routing resource allocation。
double LinkingControlPoint::required_width() const {
    double result = 0.0;
    for (const auto& segment : segments) result = std::max(result, segment.max_width);
    return result;
}

// 按论文公式 sum(Lj * Kj / 2) 计算 space node 的 LCP 需求空间。
double SpaceNode::formula_required_space() const {
    double formula = 0.0;
    for (const auto& point : linking_points) {
        formula += point.required_width() * static_cast<double>(std::max<std::size_t>(point.segments.size(), 2)) / 2.0;
    }
    return formula;
}

// 返回最终需求空间，取 LCP 公式需求和 routing feedback 下界的较大值。
double SpaceNode::required_space() const {
    double result = allocated_space + coupling_extra_space;
    return std::max(result, formula_required_space());
}

// 将 space node 类型转换为稳定文本，供 debug 和 trace 输出使用。
std::string space_kind_name(SpaceNodeKind kind) {
    switch (kind) {
        case SpaceNodeKind::Right: return "right";
        case SpaceNodeKind::Top: return "top";
        case SpaceNodeKind::Group: return "group";
        case SpaceNodeKind::Cluster: return "cluster";
    }
    return "unknown";
}

// 从 circuit 构造 enhanced B*-tree；symmetry group 会形成 ASF 树并作为 hierarchy node 进入全局树。
EnhancedBStarTree make_enhanced_tree(const Circuit& circuit) {
    EnhancedBStarTree tree;
    std::unordered_map<std::string, GroupBuildState> groups_by_name;
    std::vector<std::string> group_order;
    std::unordered_set<std::string> grouped_modules;

    auto ensure_group = [&](const std::string& name, Axis axis) -> GroupBuildState& {
        auto [it, inserted] = groups_by_name.emplace(name, GroupBuildState{});
        if (inserted) {
            group_order.push_back(name);
            it->second.group.name = name;
            it->second.group.axis = axis;
            it->second.group.hierarchy_node_id = name;
            it->second.group.asf_bstar_tree.group_name = name;
            it->second.group.asf_bstar_tree.axis = axis;
        } else if (it->second.group.axis != axis) {
            throw std::runtime_error("symmetry group " + name + " mixes axes");
        }
        if (axis != Axis::Vertical) {
            throw std::runtime_error("only vertical symmetry is supported for ASF-B*-tree");
        }
        return it->second;
    };

    for (const auto& pair : circuit.constraints.symmetry_pairs) {
        auto& state = ensure_group(pair.name, pair.axis);
        auto& asf = state.group.asf_bstar_tree;
        asf.nodes[pair.left] = make_asf_node(pair.name, pair.left, pair.right, false);
        asf.mirror_map[pair.left] = pair.right;
        state.input_order.push_back(pair.left);
        state.group.stored_modules.push_back(pair.left);
        state.group.stored_modules.push_back(pair.right);
        grouped_modules.insert(pair.left);
        grouped_modules.insert(pair.right);
    }
    for (const auto& self : circuit.constraints.symmetry_selfs) {
        auto& state = ensure_group(self.name, self.axis);
        auto& asf = state.group.asf_bstar_tree;
        asf.nodes[self.module] = make_asf_node(self.name, self.module, std::nullopt, true);
        asf.self_nodes.push_back(self.module);
        state.input_order.push_back(self.module);
        state.group.stored_modules.push_back(self.module);
        grouped_modules.insert(self.module);
    }

    std::unordered_set<std::string> inserted_groups;
    for (const auto& module : circuit.module_order) {
        std::string group_for_module;
        for (const auto& [name, state] : groups_by_name) {
            if (std::find(state.group.stored_modules.begin(), state.group.stored_modules.end(), module) !=
                state.group.stored_modules.end()) {
                group_for_module = name;
                break;
            }
        }
        if (!group_for_module.empty()) {
            if (inserted_groups.insert(group_for_module).second) {
                tree.nodes[group_for_module] = make_hierarchy_node(group_for_module);
                tree.representative_order.push_back(group_for_module);
            }
            continue;
        }
        if (!grouped_modules.contains(module)) {
            tree.nodes[module] = make_module_node(module);
            tree.representative_order.push_back(module);
        }
    }

    for (const auto& name : group_order) {
        auto& state = groups_by_name.at(name);
        auto& asf = state.group.asf_bstar_tree;
        asf.representative_order.clear();
        for (const auto& module : state.input_order) {
            if (asf.nodes.contains(module) &&
                std::find(asf.representative_order.begin(), asf.representative_order.end(), module) ==
                    asf.representative_order.end()) {
                asf.representative_order.push_back(module);
            }
        }
        rebuild_asf_bstar_tree(asf);
        tree.symmetry_groups.push_back(std::move(state.group));
    }

    rebuild_global_bstar_tree(tree);
    return tree;
}

// 构造简单链式 B*-tree，主要用于测试和基线流程。
EnhancedBStarTree make_chain_tree(const Circuit& circuit) {
    EnhancedBStarTree tree;
    for (const auto& module : circuit.module_order) {
        tree.nodes[module] = make_module_node(module);
        tree.representative_order.push_back(module);
    }
    if (!tree.representative_order.empty()) tree.root = tree.representative_order.front();
    for (std::size_t index = 1; index < tree.representative_order.size(); ++index) {
        attach_child(tree, tree.representative_order[index - 1], tree.representative_order[index], true);
    }
    return tree;
}

// 收集 routing 可消费的 concrete space node；ASF group/cluster 会展开为内部具体空间。
std::vector<SpaceNode> collect_space_nodes(const EnhancedBStarTree& tree) {
    std::vector<SpaceNode> result;
    for (const auto& id : tree.representative_order) {
        const auto& node = tree.nodes.at(id);
        if (node.kind != BStarNodeKind::Module) continue;
        result.push_back(node.right_space);
        result.push_back(node.top_space);
    }
    for (const auto& group : tree.symmetry_groups) {
        for (const auto& [id, node] : group.asf_bstar_tree.nodes) {
            (void)id;
            for_each_asf_space(node, [&](const SpaceNode& space) { result.push_back(space); });
        }
    }
    return result;
}

// 统计树上所有 space node 中的 LCP 数量。
std::size_t count_tree_lcps(const EnhancedBStarTree& tree) {
    std::size_t count = 0;
    for (const auto& space : collect_space_nodes(tree)) count += space.linking_points.size();
    return count;
}

// 检查全局树中是否存在普通模块的 right child。
bool has_ordinary_right_child(const EnhancedBStarTree& tree) {
    for (const auto& id : tree.representative_order) {
        const auto& node = tree.nodes.at(id);
        if (node.kind != BStarNodeKind::Module) continue;
        if (node.right.has_value() && tree.nodes.at(*node.right).kind == BStarNodeKind::Module) return true;
    }
    return false;
}

// 校验全局 enhanced B*-tree 的连通性、parent/child 一致性和 ASF 自对称约束。
bool is_valid_tree(const EnhancedBStarTree& tree) {
    if (tree.nodes.empty()) return !tree.root.has_value() && tree.representative_order.empty();
    if (!tree.root.has_value() || !tree.nodes.contains(*tree.root)) return false;
    if (tree.nodes.at(*tree.root).parent.has_value()) return false;
    std::unordered_set<std::string> seen;
    std::function<bool(const std::string&)> visit = [&](const std::string& id) {
        if (!tree.nodes.contains(id) || !seen.insert(id).second) return false;
        const auto& node = tree.nodes.at(id);
        for (const auto& child : {node.left, node.right}) {
            if (!child.has_value()) continue;
            if (!tree.nodes.contains(*child)) return false;
            if (tree.nodes.at(*child).parent != id) return false;
            if (!visit(*child)) return false;
        }
        return true;
    };
    if (!visit(*tree.root) || seen.size() != tree.nodes.size() || reachable_count(tree) != tree.nodes.size()) return false;
    return self_symmetry_on_rightmost_branch(tree);
}

// 检查所有 symmetry group 内部的 self-symmetric 节点是否满足 right-most branch 约束。
bool self_symmetry_on_rightmost_branch(const EnhancedBStarTree& tree) {
    for (const auto& group : tree.symmetry_groups) {
        if (!asf_selfs_on_right_most_branch(group.asf_bstar_tree)) return false;
    }
    return true;
}

// 将 routing feedback 写回普通 space 和 ASF concrete space，供下一轮 packing 使用。
void apply_routing_feedback(EnhancedBStarTree& tree, const RoutingFeedback& feedback) {
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        if (node.kind != BStarNodeKind::Module) continue;
        update_space_node(node.right_space, feedback);
        update_space_node(node.top_space, feedback);
    }
    for (auto& group : tree.symmetry_groups) {
        for (auto& [id, node] : group.asf_bstar_tree.nodes) {
            (void)id;
            for_each_asf_space(node, [&](SpaceNode& space) { update_space_node(space, feedback); });
        }
    }
}

// 收集所有可被 LCP perturbation 修改的 concrete space node。
std::vector<SpaceNode*> collect_mutable_spaces(EnhancedBStarTree& tree) {
    std::vector<SpaceNode*> spaces;
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        if (node.kind != BStarNodeKind::Module) continue;
        spaces.push_back(&node.right_space);
        spaces.push_back(&node.top_space);
    }
    for (auto& group : tree.symmetry_groups) {
        for (auto& [id, node] : group.asf_bstar_tree.nodes) {
            (void)id;
            for_each_asf_space(node, [&](SpaceNode& space) { spaces.push_back(&space); });
        }
    }
    return spaces;
}

struct LcpSlot {
    SpaceNode* space{};
    std::size_t index{};
};

// 收集 LCP 所在的 space node 和下标，供 LCP hybrid perturbation 选择操作对象。
std::vector<LcpSlot> collect_lcp_slots(std::vector<SpaceNode*>& spaces) {
    std::vector<LcpSlot> slots;
    for (auto* space : spaces) {
        for (std::size_t index = 0; index < space->linking_points.size(); ++index) {
            slots.push_back({space, index});
        }
    }
    return slots;
}

// 校验 LCP 是否属于单一 net、至少连接两个 segment，且每条 segment 都连接到该 LCP。
bool is_valid_lcp(const LinkingControlPoint& point) {
    if (point.id.empty() || point.space_node_id.empty()) return false;
    if (point.segments.size() < 2) return false;
    const std::string& net = point.segments.front().net;
    std::unordered_set<std::string> segment_ids;
    for (const auto& segment : point.segments) {
        if (segment.net != net) return false;
        if (segment.from != point.id && segment.to != point.id) return false;
        const std::string key = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
        if (!segment_ids.insert(key).second) return false;
    }
    return true;
}

// 以端点为准更新线段标识，避免拓扑端点改变后仍复用旧的 segment id。
void refresh_segment_id(WireSegmentRef& segment) {
    segment.id = segment.net + ":" + segment.from + "->" + segment.to;
}

// 将线段中引用的一个 LCP 端点替换为另一个 LCP，并同步其标识。
void replace_lcp_endpoint(WireSegmentRef& segment, const std::string& from_id, const std::string& to_id) {
    bool changed = false;
    if (segment.from == from_id) {
        segment.from = to_id;
        changed = true;
    }
    if (segment.to == from_id) {
        segment.to = to_id;
        changed = true;
    }
    if (changed) refresh_segment_id(segment);
}

// 移除合并后被折叠的自环及重复线段，保持每个 LCP 的线段集合可解释。
void normalize_lcp_segments(LinkingControlPoint& point) {
    std::unordered_set<std::string> seen;
    auto& segments = point.segments;
    segments.erase(
        std::remove_if(segments.begin(), segments.end(), [&](const WireSegmentRef& segment) {
            if (segment.from == point.id && segment.to == point.id) return true;
            const std::string key = segment.net + ":" + segment.from + "->" + segment.to;
            return !seen.insert(key).second;
        }),
        segments.end());
}

// 检查整棵树中 LCP 的归属、标识和局部线段拓扑是否一致。
bool has_valid_lcp_topology(EnhancedBStarTree& tree) {
    std::unordered_set<std::string> point_ids;
    for (auto* space : collect_mutable_spaces(tree)) {
        for (const auto& point : space->linking_points) {
            if (point.space_node_id != space->id || !point_ids.insert(point.id).second || !is_valid_lcp(point)) return false;
        }
    }
    return true;
}

// 在候选树中按稳定 id 定位 LCP，避免 vector 扩容或删除导致槽位失效。
LinkingControlPoint* find_lcp_by_id(EnhancedBStarTree& tree, const std::string& id) {
    for (auto* space : collect_mutable_spaces(tree)) {
        for (auto& point : space->linking_points) {
            if (point.id == id) return &point;
        }
    }
    return nullptr;
}

// 返回候选树中指定 id 的 LCP 所在 space node。
SpaceNode* find_lcp_space(EnhancedBStarTree& tree, const std::string& id) {
    for (auto* space : collect_mutable_spaces(tree)) {
        for (const auto& point : space->linking_points) {
            if (point.id == id) return space;
        }
    }
    return nullptr;
}

// 判断 space node 是否具备可落点的基本身份信息。
bool can_materialize_space(const SpaceNode& space) {
    return !space.id.empty() && !space.owner.empty();
}

// 将一个 LCP 移入新的 space node，并维护归属 id。
void move_lcp_to_space(LinkingControlPoint& point, SpaceNode& target) {
    point.space_node_id = target.id;
    target.linking_points.push_back(std::move(point));
}

// 论文 LCP hybrid perturbation 的 delete/insert：把一个 LCP 移动到另一个 concrete space node。
bool perturb_lcp_delete_insert(EnhancedBStarTree& tree, std::mt19937& rng) {
    auto spaces = collect_mutable_spaces(tree);
    auto slots = collect_lcp_slots(spaces);
    if (slots.empty() || spaces.size() < 2) return false;
    std::uniform_int_distribution<std::size_t> slot_dist(0, slots.size() - 1);
    std::vector<SpaceNode*> targets;
    for (auto* space : spaces) {
        if (space != nullptr && can_materialize_space(*space)) targets.push_back(space);
    }
    if (targets.size() < 2) return false;
    std::uniform_int_distribution<std::size_t> space_dist(0, targets.size() - 1);
    const auto slot = slots[slot_dist(rng)];
    SpaceNode* target = targets[space_dist(rng)];
    if (target == slot.space && targets.size() > 1) {
        const auto found = std::find(targets.begin(), targets.end(), target);
        const std::size_t index = found == targets.end() ? 0 : static_cast<std::size_t>(found - targets.begin());
        target = targets[(index + 1) % targets.size()];
    }

    auto point = std::move(slot.space->linking_points[slot.index]);
    slot.space->linking_points.erase(slot.space->linking_points.begin() + static_cast<std::ptrdiff_t>(slot.index));
    move_lcp_to_space(point, *target);
    return true;
}

// 论文 LCP hybrid perturbation 的 swap：交换两个 LCP 所属 space node。
bool perturb_lcp_swap(EnhancedBStarTree& tree, std::mt19937& rng) {
    auto spaces = collect_mutable_spaces(tree);
    auto slots = collect_lcp_slots(spaces);
    if (slots.size() < 2) return false;
    std::uniform_int_distribution<std::size_t> slot_dist(0, slots.size() - 1);
    const auto first = slots[slot_dist(rng)];
    auto second = slots[slot_dist(rng)];
    if (first.space == second.space && first.index == second.index) second = slots[(slot_dist(rng) + 1) % slots.size()];
    std::swap(first.space->linking_points[first.index], second.space->linking_points[second.index]);
    first.space->linking_points[first.index].space_node_id = first.space->id;
    second.space->linking_points[second.index].space_node_id = second.space->id;
    return true;
}

// 论文 LCP hybrid perturbation 的 split：把同一 net 的多段 LCP 拆成两个合法 LCP。
bool perturb_lcp_split(EnhancedBStarTree& tree, std::mt19937& rng) {
    auto spaces = collect_mutable_spaces(tree);
    auto slots = collect_lcp_slots(spaces);
    std::vector<LcpSlot> splittable;
    for (const auto& slot : slots) {
        if (slot.space->linking_points[slot.index].segments.size() >= 4) splittable.push_back(slot);
    }
    if (splittable.empty()) return false;
    std::uniform_int_distribution<std::size_t> slot_dist(0, splittable.size() - 1);
    const auto slot = splittable[slot_dist(rng)];
    const auto original_id = slot.space->linking_points[slot.index].id;
    const auto target_space_id = slot.space->id;
    EnhancedBStarTree candidate = tree;
    auto* space = find_lcp_space(candidate, original_id);
    if (space == nullptr || space->id != target_space_id) return false;
    auto* point = find_lcp_by_id(candidate, original_id);
    if (point == nullptr) return false;

    std::unordered_set<std::string> point_ids;
    for (auto* candidate_space : collect_mutable_spaces(candidate)) {
        for (const auto& candidate_point : candidate_space->linking_points) point_ids.insert(candidate_point.id);
    }
    std::string split_id = original_id + ":split";
    for (int suffix = 2; point_ids.contains(split_id); ++suffix) split_id = original_id + ":split" + std::to_string(suffix);

    const std::size_t half = point->segments.size() / 2;
    LinkingControlPoint split = *point;
    split.id = split_id;
    split.space_node_id = space->id;
    split.segments.assign(point->segments.begin() + static_cast<std::ptrdiff_t>(half), point->segments.end());
    point->segments.erase(point->segments.begin() + static_cast<std::ptrdiff_t>(half), point->segments.end());
    for (auto& segment : split.segments) replace_lcp_endpoint(segment, original_id, split.id);
    normalize_lcp_segments(*point);
    normalize_lcp_segments(split);
    space->linking_points.push_back(std::move(split));
    if (!has_valid_lcp_topology(candidate)) return false;
    tree = std::move(candidate);
    return true;
}

// 论文 LCP hybrid perturbation 的 merge：合并同一 net 的两个 LCP，并保留全部 segment。
bool perturb_lcp_merge(EnhancedBStarTree& tree, std::mt19937& rng) {
    auto spaces = collect_mutable_spaces(tree);
    auto slots = collect_lcp_slots(spaces);
    if (slots.size() < 2) return false;
    std::uniform_int_distribution<std::size_t> slot_dist(0, slots.size() - 1);
    const auto first = slots[slot_dist(rng)];
    for (std::size_t attempts = 0; attempts < slots.size(); ++attempts) {
        const auto second = slots[(slot_dist(rng) + attempts) % slots.size()];
        if (first.space == second.space && first.index == second.index) continue;
        const auto& first_point = first.space->linking_points[first.index];
        const auto& second_point = second.space->linking_points[second.index];
        if (first_point.segments.empty() || second_point.segments.empty()) continue;
        if (first_point.segments.front().net != second_point.segments.front().net) continue;
        const std::string retained_id = first_point.id;
        const std::string removed_id = second_point.id;
        EnhancedBStarTree candidate = tree;
        for (auto* candidate_space : collect_mutable_spaces(candidate)) {
            for (auto& point : candidate_space->linking_points) {
                for (auto& segment : point.segments) replace_lcp_endpoint(segment, removed_id, retained_id);
            }
        }
        auto* retained = find_lcp_by_id(candidate, retained_id);
        auto* removed = find_lcp_by_id(candidate, removed_id);
        auto* removed_space = find_lcp_space(candidate, removed_id);
        if (retained == nullptr || removed == nullptr || removed_space == nullptr) return false;
        retained->segments.insert(retained->segments.end(), removed->segments.begin(), removed->segments.end());
        for (auto* candidate_space : collect_mutable_spaces(candidate)) {
            for (auto& point : candidate_space->linking_points) normalize_lcp_segments(point);
        }
        const auto remove_at = std::find_if(
            removed_space->linking_points.begin(), removed_space->linking_points.end(),
            [&](const LinkingControlPoint& point) { return point.id == removed_id; });
        if (remove_at == removed_space->linking_points.end()) return false;
        removed_space->linking_points.erase(remove_at);
        if (!has_valid_lcp_topology(candidate)) return false;
        tree = std::move(candidate);
        return true;
    }
    return false;
}

// 对全局树、ASF 内部树和 LCP 执行一次 SA 扰动，并保持树结构合法。
PerturbationReport perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng) {
    PerturbationReport report;
    report.lcp_before = count_tree_lcps(tree);
    if (tree.representative_order.empty()) return report;
    auto spaces = collect_mutable_spaces(tree);
    const bool has_tree_lcp = !collect_lcp_slots(spaces).empty();
    const bool has_asf_tree = std::any_of(tree.symmetry_groups.begin(), tree.symmetry_groups.end(), [](const auto& group) {
        return !group.asf_bstar_tree.nodes.empty();
    });
    // 无 ASF 的普通树仍只提供 7 类既有扰动，避免新增 ASF 预留编号改变同 seed 的抽样序列。
    const int max_move = has_tree_lcp ? (has_asf_tree ? 8 : 6) : (has_asf_tree ? 5 : 2);
    std::uniform_int_distribution<int> move_dist(0, max_move);
    const int move = move_dist(rng);
    const auto movable = movable_nodes(tree);
    if (move == 0 && movable.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        const auto first = movable[index_dist(rng)];
        auto second = movable[index_dist(rng)];
        if (first == second) second = movable[(index_dist(rng) + 1) % movable.size()];
        swap_tree_positions(tree, first, second);
        report.move = "module-swap";
        report.changed = true;
    } else if (move == 1 && movable.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        const auto id = movable[index_dist(rng)];
        std::vector<std::string> targets;
        for (const auto& candidate : movable) {
            if (candidate != id && !is_ancestor_of(tree, id, candidate)) targets.push_back(candidate);
        }
        if (!targets.empty()) {
            std::uniform_int_distribution<std::size_t> target_dist(0, targets.size() - 1);
            std::uniform_int_distribution<int> side_dist(0, 1);
            // 在修改树前按固定顺序消费随机数，保证同 seed 不受 C++ 参数求值顺序影响。
            const auto target = targets[target_dist(rng)];
            const bool as_left = side_dist(rng) == 0;
            detach_node_preserving_children(tree, id);
            insert_node_at_child(tree, target, id, as_left);
            report.move = "module-delete-insert";
            report.changed = true;
        }
    } else if (move == 2) {
        std::vector<std::string> rotatable;
        for (const auto& id : movable) {
            if (tree.nodes.at(id).kind == BStarNodeKind::Module) rotatable.push_back(id);
        }
        if (!rotatable.empty()) {
            std::uniform_int_distribution<std::size_t> index_dist(0, rotatable.size() - 1);
            auto& node = tree.nodes.at(rotatable[index_dist(rng)]);
            node.angle = (node.angle + 90) % 360;
            report.move = "module-rotate";
            report.changed = true;
        }
    } else if (has_tree_lcp && move == 3) {
        report.move = "lcp-delete-insert";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_delete_insert(tree, rng);
    } else if (has_tree_lcp && move == 4) {
        report.move = "lcp-swap";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_swap(tree, rng);
    } else if (has_tree_lcp && move == 5) {
        report.move = "lcp-split";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_split(tree, rng);
    } else if (has_tree_lcp && move == 6) {
        report.move = "lcp-merge";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_merge(tree, rng);
    } else if (has_asf_tree) {
        std::vector<std::size_t> candidates;
        for (std::size_t index = 0; index < tree.symmetry_groups.size(); ++index) {
            if (!tree.symmetry_groups[index].asf_bstar_tree.nodes.empty()) candidates.push_back(index);
        }
        if (!candidates.empty()) {
            std::uniform_int_distribution<std::size_t> group_dist(0, candidates.size() - 1);
            auto& group = tree.symmetry_groups[candidates[group_dist(rng)]];
            std::string asf_move;
            report.changed = perturb_asf_bstar_tree(group.asf_bstar_tree, rng, asf_move);
            report.move = report.changed ? asf_move + ":" + group.name : "none";
        }
    }
    if (report.move.empty()) report.move = "none";
    refresh_representative_order(tree);
    if (!is_valid_tree(tree)) throw std::runtime_error("invalid enhanced B*-tree after perturbation");
    report.lcp_after = count_tree_lcps(tree);
    return report;
}

}  // namespace sapr
