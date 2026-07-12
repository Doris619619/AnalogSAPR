// 文件职责：实现 enhanced B*-tree、vertical ASF-B*-tree、space node、LCP 扰动和 placement 扰动。
#include "sapr/tree.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace sapr {
namespace {

SpaceNode make_space_node(const std::string& id, const std::string& owner, SpaceNodeKind kind) {
    return {id, owner, kind, {}, 0.0, {}, 0.0, {}};
}

BStarNode make_module_node(const std::string& module) {
    BStarNode node;
    node.id = module;
    node.kind = BStarNodeKind::Module;
    node.module = module;
    node.right_space = make_space_node(module + ":right", module, SpaceNodeKind::Right);
    node.top_space = make_space_node(module + ":top", module, SpaceNodeKind::Top);
    return node;
}

BStarNode make_hierarchy_node(const std::string& group_name) {
    BStarNode node;
    node.id = group_name;
    node.kind = BStarNodeKind::Hierarchy;
    node.hierarchy_group = group_name;
    return node;
}

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

void attach_child(EnhancedBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    if (as_left) {
        tree.nodes.at(parent).left = child;
    } else {
        tree.nodes.at(parent).right = child;
    }
    tree.nodes.at(child).parent = parent;
}

void attach_child(AsfBStarTree& tree, const std::string& parent, const std::string& child, bool as_left) {
    if (as_left) {
        tree.nodes.at(parent).left = child;
    } else {
        tree.nodes.at(parent).right = child;
    }
    tree.nodes.at(child).parent = parent;
}

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

void refresh_right_most_branch(AsfBStarTree& tree) {
    tree.right_most_branch.clear();
    auto current = tree.root;
    while (current.has_value() && tree.nodes.contains(*current)) {
        tree.right_most_branch.push_back(*current);
        current = tree.nodes.at(*current).right;
    }
}

void refresh_space_node_clusters(AsfBStarTree& tree) {
    std::unordered_set<std::string> right_branch(tree.right_most_branch.begin(), tree.right_most_branch.end());
    for (auto& [id, node] : tree.nodes) {
        if (right_branch.contains(id)) {
            if (!node.space_node_cluster.has_value()) {
                node.space_node_cluster = make_space_node_cluster(tree.group_name, node.module, node.mirror_module);
            }
        } else {
            node.space_node_cluster.reset();
        }
    }
}

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

bool asf_selfs_on_right_most_branch(const AsfBStarTree& tree) {
    std::unordered_set<std::string> right_branch(tree.right_most_branch.begin(), tree.right_most_branch.end());
    for (const auto& self : tree.self_nodes) {
        if (!right_branch.contains(self)) return false;
    }
    return true;
}

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

bool is_ancestor_of(const EnhancedBStarTree& tree, const std::string& candidate, const std::string& target) {
    auto current = tree.nodes.at(target).parent;
    while (current.has_value()) {
        if (*current == candidate) return true;
        current = tree.nodes.at(*current).parent;
    }
    return false;
}

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

void for_each_asf_space(AsfBStarNode& node, const std::function<void(SpaceNode&)>& visit) {
    for (auto& group : node.space_node_groups) {
        for (auto& space : group.spaces) visit(space);
    }
    if (node.space_node_cluster.has_value()) {
        for (auto& space : node.space_node_cluster->spaces) visit(space);
    }
}

void for_each_asf_space(const AsfBStarNode& node, const std::function<void(const SpaceNode&)>& visit) {
    for (const auto& group : node.space_node_groups) {
        for (const auto& space : group.spaces) visit(space);
    }
    if (node.space_node_cluster.has_value()) {
        for (const auto& space : node.space_node_cluster->spaces) visit(space);
    }
}

std::vector<std::string> movable_nodes(const EnhancedBStarTree& tree) {
    std::vector<std::string> result;
    for (const auto& id : tree.representative_order) {
        if (tree.nodes.contains(id)) result.push_back(id);
    }
    return result;
}

struct GroupBuildState {
    SymmetryGroupNode group;
    std::vector<std::string> input_order;
};

}  // namespace

double LinkingControlPoint::required_width() const {
    double result = 0.0;
    for (const auto& segment : segments) result = std::max(result, segment.max_width);
    return result;
}

double SpaceNode::required_space() const {
    double result = allocated_space + coupling_extra_space;
    double formula = 0.0;
    for (const auto& point : linking_points) {
        formula += point.required_width() * static_cast<double>(std::max<std::size_t>(point.segments.size(), 2)) / 2.0;
    }
    return std::max(result, formula);
}

std::string space_kind_name(SpaceNodeKind kind) {
    switch (kind) {
        case SpaceNodeKind::Right: return "right";
        case SpaceNodeKind::Top: return "top";
        case SpaceNodeKind::Group: return "group";
        case SpaceNodeKind::Cluster: return "cluster";
    }
    return "unknown";
}

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

std::size_t count_tree_lcps(const EnhancedBStarTree& tree) {
    std::size_t count = 0;
    for (const auto& space : collect_space_nodes(tree)) count += space.linking_points.size();
    return count;
}

bool has_ordinary_right_child(const EnhancedBStarTree& tree) {
    for (const auto& id : tree.representative_order) {
        const auto& node = tree.nodes.at(id);
        if (node.kind != BStarNodeKind::Module) continue;
        if (node.right.has_value() && tree.nodes.at(*node.right).kind == BStarNodeKind::Module) return true;
    }
    return false;
}

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

bool self_symmetry_on_rightmost_branch(const EnhancedBStarTree& tree) {
    for (const auto& group : tree.symmetry_groups) {
        if (!asf_selfs_on_right_most_branch(group.asf_bstar_tree)) return false;
    }
    return true;
}

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

std::vector<LcpSlot> collect_lcp_slots(std::vector<SpaceNode*>& spaces) {
    std::vector<LcpSlot> slots;
    for (auto* space : spaces) {
        for (std::size_t index = 0; index < space->linking_points.size(); ++index) {
            slots.push_back({space, index});
        }
    }
    return slots;
}

bool is_valid_lcp(const LinkingControlPoint& point) {
    if (point.segments.size() < 2) return false;
    const std::string& net = point.segments.front().net;
    std::unordered_set<std::string> segment_ids;
    for (const auto& segment : point.segments) {
        if (segment.net != net) return false;
        const std::string key = segment.id.empty() ? segment.net + ":" + segment.from + "->" + segment.to : segment.id;
        if (!segment_ids.insert(key).second) return false;
    }
    return true;
}

void move_lcp_to_space(LinkingControlPoint& point, SpaceNode& target) {
    point.space_node_id = target.id;
    target.linking_points.push_back(std::move(point));
}

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
    auto& point = slot.space->linking_points[slot.index];
    const std::size_t half = point.segments.size() / 2;
    LinkingControlPoint split = point;
    split.id = point.id + ":split";
    split.segments.assign(point.segments.begin() + static_cast<std::ptrdiff_t>(half), point.segments.end());
    point.segments.erase(point.segments.begin() + static_cast<std::ptrdiff_t>(half), point.segments.end());
    if (!is_valid_lcp(point) || !is_valid_lcp(split)) return false;
    slot.space->linking_points.push_back(std::move(split));
    return true;
}

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
        const auto second_segments = second_point.segments;
        first.space->linking_points[first.index].segments.insert(
            first.space->linking_points[first.index].segments.end(),
            second_segments.begin(),
            second_segments.end());
        const bool valid = is_valid_lcp(first.space->linking_points[first.index]);
        second.space->linking_points.erase(second.space->linking_points.begin() + static_cast<std::ptrdiff_t>(second.index));
        return valid;
    }
    return false;
}

PerturbationReport perturb_placement_tree(EnhancedBStarTree& tree, std::mt19937& rng) {
    PerturbationReport report;
    report.lcp_before = count_tree_lcps(tree);
    if (tree.representative_order.empty()) return report;
    auto spaces = collect_mutable_spaces(tree);
    const bool has_tree_lcp = !collect_lcp_slots(spaces).empty();
    std::uniform_int_distribution<int> move_dist(0, has_tree_lcp ? 6 : 2);
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
            detach_node_preserving_children(tree, id);
            insert_node_at_child(tree, targets[target_dist(rng)], id, side_dist(rng) == 0);
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
