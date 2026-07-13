// 文件职责：实现 enhanced B*-tree、ASF 对称约束、space node、LCP 扰动和真实二维 placement 扰动。
#include "sapr/tree.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <unordered_set>

namespace sapr {
namespace {

// 创建一个带稳定候选容器的 space node。
SpaceNode make_space_node(const std::string& id, const std::string& owner, SpaceNodeKind kind) {
    return {id, owner, kind, {}, 0.0, {}, 0.0, {}};
}

// 为对称组构造论文 Fig. 5 中的 mirrored space node group。
SpaceNodeBundle make_space_group_bundle(const std::string& group_name, const std::string& owner) {
    return {{{group_name + ":group:right_left", owner, SpaceNodeKind::Group, {}, 0.0, {}, 0.0, {}},
             {group_name + ":group:top_pair", owner, SpaceNodeKind::Group, {}, 0.0, {}, 0.0, {}}}};
}

// 为对称组构造论文 Fig. 5 中的四空间 cluster。
SpaceNodeBundle make_space_cluster_bundle(const std::string& group_name, const std::string& owner) {
    return {{{group_name + ":cluster:left_right", owner, SpaceNodeKind::Cluster, {}, 0.0, {}, 0.0, {}},
             {group_name + ":cluster:left_top", owner, SpaceNodeKind::Cluster, {}, 0.0, {}, 0.0, {}},
             {group_name + ":cluster:right_right", owner, SpaceNodeKind::Cluster, {}, 0.0, {}, 0.0, {}},
             {group_name + ":cluster:right_top", owner, SpaceNodeKind::Cluster, {}, 0.0, {}, 0.0, {}}}};
}

// 为模块创建论文中的右侧和上侧 space node，并初始化触碰线网的 LCP 容器。
BStarNode make_node(const Circuit& circuit, const std::string& module) {
    (void)circuit;
    BStarNode node;
    node.module = module;
    node.right_space = make_space_node(module + ":right", module, SpaceNodeKind::Right);
    node.top_space = make_space_node(module + ":top", module, SpaceNodeKind::Top);
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

// 返回当前代表是否为自对称模块。
bool is_self_module(const EnhancedBStarTree& tree, const std::string& module) {
    for (const auto& group : tree.symmetry_groups) {
        if (group.self_symmetric && group.representative == module) return true;
    }
    return false;
}

// 判断节点是否允许普通 module move 直接移动。
bool is_movable_module(const EnhancedBStarTree& tree, const std::string& module) {
    return tree.nodes.contains(module) && !is_self_module(tree, module);
}

// 按当前真实树结构刷新稳定遍历顺序。
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

// 删除一个节点并把其子树接回原父节点，保证其余节点仍保持单根可达。
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

// 将 child 插入到 parent 的指定方向，原方向子树挂到 child 同侧。
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

// 判断 candidate 是否位于 root 到 target 的路径上，避免插入形成环。
bool is_ancestor_of(const EnhancedBStarTree& tree, const std::string& candidate, const std::string& target) {
    auto current = tree.nodes.at(target).parent;
    while (current.has_value()) {
        if (*current == candidate) return true;
        current = tree.nodes.at(*current).parent;
    }
    return false;
}

// 通过两次安全的 delete-insert 交换节点位置，保持 module id 与物理语义一致。
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

// 初始构造近似平衡二维 ordinary B*-tree，并把 self-symmetry 节点接到 right-most branch。
void rebuild_asf_tree(EnhancedBStarTree& tree) {
    std::vector<std::string> old_order = tree.representative_order;
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        node.parent.reset();
        node.left.reset();
        node.right.reset();
    }
    tree.root.reset();

    std::vector<std::string> ordinary;
    std::vector<std::string> selfs;
    for (const auto& id : old_order) {
        if (!tree.nodes.contains(id)) continue;
        if (is_self_module(tree, id)) selfs.push_back(id);
        else ordinary.push_back(id);
    }
    if (ordinary.empty() && selfs.empty()) return;

    tree.root = ordinary.empty() ? selfs.front() : ordinary.front();
    if (ordinary.size() > 1) {
        const std::size_t split = (ordinary.size() + 1) / 2;
        attach_child(tree, ordinary.front(), ordinary[1], true);
        for (std::size_t index = 2; index < split; ++index) {
            attach_child(tree, ordinary[index - 1], ordinary[index], true);
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
    for (const auto& self : selfs) {
        if (self == right_parent) continue;
        attach_child(tree, right_parent, self, false);
        right_parent = self;
    }
    refresh_representative_order(tree);
}

// 只修复 ASF self-symmetry right-most branch，不改变 ordinary tree 的二维拓扑。
void repair_asf_rightmost_branch(EnhancedBStarTree& tree) {
    std::vector<std::string> selfs;
    for (const auto& [id, _] : tree.nodes) {
        if (is_self_module(tree, id)) selfs.push_back(id);
    }
    std::sort(selfs.begin(), selfs.end());
    for (const auto& self : selfs) detach_node_preserving_children(tree, self);
    if (!tree.root.has_value() && !selfs.empty()) tree.root = selfs.front();
    if (tree.root.has_value()) {
        std::string right_parent = *tree.root;
        while (tree.nodes.at(right_parent).right.has_value()) right_parent = *tree.nodes.at(right_parent).right;
        for (const auto& self : selfs) {
            if (self == right_parent) continue;
            attach_child(tree, right_parent, self, false);
            right_parent = self;
        }
    }
    refresh_representative_order(tree);
}

// 收集当前可移动 ordinary/代表节点。
std::vector<std::string> movable_modules(const EnhancedBStarTree& tree) {
    std::vector<std::string> result;
    for (const auto& id : tree.representative_order) {
        if (is_movable_module(tree, id)) result.push_back(id);
    }
    return result;
}

// 返回从 root 可达的代表节点数量。
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

}  // namespace

// 返回该控制点需要的最大线宽。
double LinkingControlPoint::required_width() const {
    double result = 0.0;
    for (const auto& segment : segments) result = std::max(result, segment.max_width);
    return result;
}

// 按论文公式计算该空间节点需要预留的宽度。
double SpaceNode::required_space() const {
    double result = allocated_space + coupling_extra_space;
    double formula = 0.0;
    for (const auto& point : linking_points) {
        formula += point.required_width() * static_cast<double>(std::max<std::size_t>(point.segments.size(), 2)) / 2.0;
    }
    return std::max(result, formula);
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

// 按输入和约束构造增强 B*-tree，普通模块和对称代表进入主树。
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
                                        {pair.left},
                                        std::nullopt,
                                        make_space_node(pair.name + ":space_group", pair.left, SpaceNodeKind::Group),
                                        make_space_node(pair.name + ":space_cluster", pair.left, SpaceNodeKind::Cluster),
                                        {pair.left},
                                        {{pair.left, pair.right}},
                                        {},
                                        {},
                                        make_space_group_bundle(pair.name, pair.left),
                                        make_space_cluster_bundle(pair.name, pair.left)});
    }
    for (const auto& self : circuit.constraints.symmetry_selfs) {
        tree.symmetry_groups.push_back({self.name,
                                        self.axis,
                                        self.module,
                                        std::nullopt,
                                        true,
                                        {self.module},
                                        self.module,
                                        make_space_node(self.name + ":space_group", self.module, SpaceNodeKind::Group),
                                        make_space_node(self.name + ":space_cluster", self.module, SpaceNodeKind::Cluster),
                                        {self.module},
                                        {},
                                        {self.module},
                                        {self.module},
                                        make_space_group_bundle(self.name, self.module),
                                        make_space_cluster_bundle(self.name, self.module)});
    }

    rebuild_asf_tree(tree);
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
        for (const auto& space : group.space_group_bundle.spaces) result.push_back(space);
        for (const auto& space : group.space_cluster_bundle.spaces) result.push_back(space);
    }
    return result;
}

// 统计当前树状态中持久化保存的 LCP 数量。
std::size_t count_tree_lcps(const EnhancedBStarTree& tree) {
    std::size_t count = 0;
    for (const auto& space : collect_space_nodes(tree)) count += space.linking_points.size();
    return count;
}

// 判断普通模块主树中是否存在 right child，用于验证二维 B*-tree 搜索自由度。
bool has_ordinary_right_child(const EnhancedBStarTree& tree) {
    for (const auto& id : tree.representative_order) {
        if (!is_movable_module(tree, id)) continue;
        const auto& node = tree.nodes.at(id);
        if (node.right.has_value() && is_movable_module(tree, *node.right)) return true;
    }
    return false;
}

// 检查增强 B*-tree 是否仍是单根、无环且 parent/child 关系一致。
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
    return visit(*tree.root) && seen.size() == tree.nodes.size() &&
           reachable_count(tree) == tree.nodes.size() && self_symmetry_on_rightmost_branch(tree);
}

// 检查所有自对称代表是否位于主树的 right-most branch。
bool self_symmetry_on_rightmost_branch(const EnhancedBStarTree& tree) {
    std::unordered_set<std::string> right_branch;
    auto current = tree.root;
    while (current.has_value()) {
        right_branch.insert(*current);
        current = tree.nodes.at(*current).right;
    }
    for (const auto& group : tree.symmetry_groups) {
        if (group.self_symmetric && !right_branch.contains(group.representative)) return false;
    }
    return true;
}

// 将 routing adapter 返回的资源预留反馈写回对应 space node。
void apply_routing_feedback(EnhancedBStarTree& tree, const RoutingFeedback& feedback) {
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        update_space_node(node.right_space, feedback);
        update_space_node(node.top_space, feedback);
    }
    for (auto& group : tree.symmetry_groups) {
        update_space_node(group.space_group, feedback);
        update_space_node(group.space_cluster, feedback);
        for (auto& space : group.space_group_bundle.spaces) update_space_node(space, feedback);
        for (auto& space : group.space_cluster_bundle.spaces) update_space_node(space, feedback);
    }
}

// 收集增强 B*-tree 中所有可容纳 LCP 的 space node。
std::vector<SpaceNode*> collect_mutable_spaces(EnhancedBStarTree& tree) {
    std::vector<SpaceNode*> spaces;
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        spaces.push_back(&node.right_space);
        spaces.push_back(&node.top_space);
    }
    for (auto& group : tree.symmetry_groups) {
        spaces.push_back(&group.space_group);
        spaces.push_back(&group.space_cluster);
        for (auto& space : group.space_group_bundle.spaces) spaces.push_back(&space);
        for (auto& space : group.space_cluster_bundle.spaces) spaces.push_back(&space);
    }
    return spaces;
}

// 表示一个 LCP 在某个 space node 中的位置。
struct LcpSlot {
    SpaceNode* space{};
    std::size_t index{};
};

// 收集当前树中全部 LCP 的可变位置。
std::vector<LcpSlot> collect_lcp_slots(std::vector<SpaceNode*>& spaces) {
    std::vector<LcpSlot> slots;
    for (auto* space : spaces) {
        for (std::size_t index = 0; index < space->linking_points.size(); ++index) {
            slots.push_back({space, index});
        }
    }
    return slots;
}

// 判断 LCP 是否仍满足论文要求的同一 net 且至少两条 wire segment 的约束。
bool is_valid_lcp(const LinkingControlPoint& point) {
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

// 将一个 LCP 移入新的 space node，并维护归属 id。
void move_lcp_to_space(LinkingControlPoint& point, SpaceNode& target) {
    point.space_node_id = target.id;
    target.linking_points.push_back(std::move(point));
}

// 执行论文 LCP delete-insert 扰动。
bool perturb_lcp_delete_insert(EnhancedBStarTree& tree, std::mt19937& rng) {
    auto spaces = collect_mutable_spaces(tree);
    auto slots = collect_lcp_slots(spaces);
    if (slots.empty() || spaces.size() < 2) return false;
    std::uniform_int_distribution<std::size_t> slot_dist(0, slots.size() - 1);
    std::uniform_int_distribution<std::size_t> space_dist(0, spaces.size() - 1);
    const auto slot = slots[slot_dist(rng)];
    SpaceNode* target = spaces[space_dist(rng)];
    if (target == slot.space && spaces.size() > 1) target = spaces[(space_dist(rng) + 1) % spaces.size()];

    auto point = std::move(slot.space->linking_points[slot.index]);
    slot.space->linking_points.erase(slot.space->linking_points.begin() + static_cast<std::ptrdiff_t>(slot.index));
    move_lcp_to_space(point, *target);
    return true;
}

// 执行论文 LCP swap 扰动。
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

// 执行论文 LCP split 扰动。
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

// 执行论文 LCP merge 扰动。
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

// 对增强 B*-tree 执行一次论文 placement 侧扰动：module 或 LCP 的 hybrid move。
PerturbationReport perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng) {
    PerturbationReport report;
    report.lcp_before = count_tree_lcps(tree);
    if (tree.representative_order.empty()) return report;
    auto spaces = collect_mutable_spaces(tree);
    const bool has_tree_lcp = !collect_lcp_slots(spaces).empty();
    std::uniform_int_distribution<int> move_dist(0, has_tree_lcp ? 6 : 2);
    const int move = move_dist(rng);
    const auto movable = movable_modules(tree);
    if (move == 0 && movable.size() > 1) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        const auto first = movable[index_dist(rng)];
        auto second = movable[index_dist(rng)];
        if (first == second) second = movable[(index_dist(rng) + 1) % movable.size()];
        swap_tree_positions(tree, first, second);
        repair_asf_rightmost_branch(tree);
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
            const auto target = targets[target_dist(rng)];
            const bool as_left = side_dist(rng) == 0;
            detach_node_preserving_children(tree, id);
            insert_node_at_child(tree, target, id, as_left);
            repair_asf_rightmost_branch(tree);
            report.move = "module-delete-insert";
            report.changed = true;
        }
    } else if (move == 2 && !movable.empty()) {
        std::uniform_int_distribution<std::size_t> index_dist(0, movable.size() - 1);
        auto& node = tree.nodes.at(movable[index_dist(rng)]);
        node.angle = (node.angle + 90) % 360;
        report.move = "module-rotate";
        report.changed = true;
    } else if (move == 3) {
        report.move = "lcp-delete-insert";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_delete_insert(tree, rng);
    } else if (move == 4) {
        report.move = "lcp-swap";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_swap(tree, rng);
    } else if (move == 5) {
        report.move = "lcp-split";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_split(tree, rng);
    } else {
        report.move = "lcp-merge";
        report.used_lcp_move = true;
        report.changed = perturb_lcp_merge(tree, rng);
    }
    if (report.move.empty()) report.move = "none";
    refresh_representative_order(tree);
    if (!is_valid_tree(tree)) throw std::runtime_error("invalid enhanced B*-tree after perturbation");
    report.lcp_after = count_tree_lcps(tree);
    return report;
}

}  // namespace sapr
