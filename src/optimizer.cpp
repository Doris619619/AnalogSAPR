// 实现论文 placement-aware 增强 B*-tree contour packing、routing adapter 和模拟退火主循环。
#include "sapr/optimizer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "sapr/constraints.hpp"
#include "sapr/geometry.hpp"
#include "sapr/lcp_generator.hpp"
#include "sapr/router.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "sapr/tree.hpp"

namespace sapr {
namespace {

// 表示候选解在模拟退火中的完整评价。
struct CandidateState {
    EnhancedBStarTree tree;
    RoutingEvaluationRequest request;
    RoutingFeedback feedback;
    double cost{};
};

// 表示 contour packing 已占用的 group bounding box。
struct PackedRect {
    double x1{};
    double y1{};
    double x2{};
    double y2{};
};

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

void write_json_string(std::ostringstream& out, const std::string& value) {
    out << '"' << json_escape(value) << '"';
}

void write_json_optional_string(std::ostringstream& out, const std::optional<std::string>& value) {
    if (value.has_value()) {
        write_json_string(out, *value);
    } else {
        out << "null";
    }
}

void write_json_string_array(std::ostringstream& out, const std::vector<std::string>& values) {
    out << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ',';
        write_json_string(out, values[index]);
    }
    out << ']';
}

// 收集 space node 内的 LCP 标识，供 enhanced B*-tree 调试图展示空间节点结构。
// 将电流方向写成稳定字符串，供 B* 树可视化解释 LCP 拓扑。
std::string current_direction_name(CurrentDirection direction) {
    switch (direction) {
        case CurrentDirection::In: return "in";
        case CurrentDirection::Out: return "out";
        case CurrentDirection::Unknown: return "unknown";
    }
    return "unknown";
}

// 在 trace 只用于可视化时，根据常见电源/输出网名恢复优先级标签。
std::string trace_priority_name(const std::string& net) {
    std::string upper = net;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    if (upper == "VDD" || upper == "AVDD" || upper == "VCC" || upper == "GND" || upper == "AGND" ||
        upper == "VSS" || upper == "OUT" || upper == "VOUT") {
        return "critical";
    }
    return "normal";
}

// 写出一条 LCP wire segment，保留端点和电流方向，供结构图绘制 routing arc。
void write_wire_segment_json(std::ostringstream& out, const WireSegmentRef& segment) {
    out << "{\"id\": ";
    write_json_string(out, segment.id);
    out << ", \"from\": ";
    write_json_string(out, segment.from);
    out << ", \"to\": ";
    write_json_string(out, segment.to);
    out << ", \"current_direction\": ";
    write_json_string(out, current_direction_name(segment.current_direction));
    out << '}';
}

// 导出 net -> pin/LCP/pin 拓扑，让 B* 树结构图能显示论文中的布线弧线。
void write_routing_topologies_json(std::ostringstream& out, const RoutingEvaluationRequest& request) {
    out << "  \"routing_topologies\": [\n";
    for (std::size_t topology_index = 0; topology_index < request.net_topologies.size(); ++topology_index) {
        const auto& topology = request.net_topologies[topology_index];
        if (topology_index != 0) out << ",\n";
        out << "    {\"net\": ";
        write_json_string(out, topology.net);
        out << ", \"priority\": ";
        write_json_string(out, trace_priority_name(topology.net));
        out << ", \"pins\": ";
        write_json_string_array(out, topology.pins);
        out << ", \"linking_points\": [";
        std::unordered_set<std::string> emitted_lcp_ids;
        bool first_lcp = true;
        for (const auto& point : topology.linking_points) {
            if (!emitted_lcp_ids.insert(point.id).second) continue;
            if (!first_lcp) out << ',';
            first_lcp = false;
            out << "{\"id\": ";
            write_json_string(out, point.id);
            out << ", \"space_node_id\": ";
            write_json_string(out, point.space_node_id);
            out << '}';
        }
        out << "], \"segments\": [";
        for (std::size_t segment_index = 0; segment_index < topology.segments.size(); ++segment_index) {
            if (segment_index != 0) out << ',';
            write_wire_segment_json(out, topology.segments[segment_index]);
        }
        out << "]}";
    }
    out << "\n  ],\n";
}

std::vector<std::string> lcp_ids_for_json(const SpaceNode& space) {
    std::vector<std::string> ids;
    ids.reserve(space.linking_points.size());
    for (const auto& point : space.linking_points) ids.push_back(point.id);
    return ids;
}

// 按论文树侧语义导出 space node：left space node 对应几何右侧空间，right space node 对应几何上方空间。
void write_btree_space_node_json(
    std::ostringstream& out,
    const char* field_name,
    const char* tree_side,
    const char* geometry_space,
    const SpaceNode& space) {
    out << ", \"" << field_name << "\": {\"id\": ";
    write_json_string(out, space.id);
    out << ", \"tree_side\": ";
    write_json_string(out, tree_side);
    out << ", \"geometry_space\": ";
    write_json_string(out, geometry_space);
    out << ", \"required_space\": " << space.required_space()
        << ", \"lcp_count\": " << space.linking_points.size()
        << ", \"lcp_ids\": ";
    write_json_string_array(out, lcp_ids_for_json(space));
    out << '}';
}

// 将最终 best candidate 的 B*-tree、packing trace 和 metrics 导出为轻量 JSON。
std::string make_btree_trace_json(const CandidateState& state) {
    std::ostringstream out;
    const auto metrics = state.feedback.metrics;
    out << "{\n";
    out << "  \"root\": ";
    write_json_optional_string(out, state.tree.root);
    out << ",\n";
    out << "  \"nodes\": [\n";
    bool first_node = true;
    for (const auto& id : state.tree.representative_order) {
        const auto found = state.tree.nodes.find(id);
        if (found == state.tree.nodes.end()) continue;
        const auto& node = found->second;
        if (!first_node) out << ",\n";
        first_node = false;
        out << "    {\"id\": ";
        write_json_string(out, id);
        out << ", \"module\": ";
        write_json_string(out, node.module);
        out << ", \"parent\": ";
        write_json_optional_string(out, node.parent);
        out << ", \"left\": ";
        write_json_optional_string(out, node.left);
        out << ", \"right\": ";
        write_json_optional_string(out, node.right);
        out << ", \"angle\": " << node.angle
            << ", \"right_space\": " << node.right_space.required_space()
            << ", \"top_space\": " << node.top_space.required_space()
            << ", \"right_lcp_count\": " << node.right_space.linking_points.size()
            << ", \"top_lcp_count\": " << node.top_space.linking_points.size();
        write_btree_space_node_json(out, "left_space_node", "left_space_node", "right_space", node.right_space);
        write_btree_space_node_json(out, "right_space_node", "right_space_node", "top_space", node.top_space);
        out << '}';
    }
    out << "\n  ],\n";
    out << "  \"placements\": [\n";
    for (std::size_t index = 0; index < state.request.placement_order.size(); ++index) {
        const auto& module = state.request.placement_order[index];
        const auto found = state.request.placements.find(module);
        if (found == state.request.placements.end()) continue;
        const auto& placement = found->second;
        if (index != 0) out << ",\n";
        out << "    {\"module\": ";
        write_json_string(out, module);
        out << ", \"x\": " << placement.x
            << ", \"y\": " << placement.y
            << ", \"angle\": " << placement.angle
            << ", \"orient\": ";
        write_json_string(out, placement.orient);
        out << '}';
    }
    out << "\n  ],\n";
    out << "  \"packing_steps\": [\n";
    for (std::size_t index = 0; index < state.request.packing_trace.steps.size(); ++index) {
        const auto& step = state.request.packing_trace.steps[index];
        if (index != 0) out << ",\n";
        out << "    {\"index\": " << index
            << ", \"tree_node\": ";
        write_json_string(out, step.tree_node);
        out << ", \"module\": ";
        write_json_string(out, step.module);
        out << ", \"x\": " << step.x
            << ", \"y\": " << step.y
            << ", \"occupied_bbox\": {\"x1\": " << step.occupied_bbox.x1
            << ", \"y1\": " << step.occupied_bbox.y1
            << ", \"x2\": " << step.occupied_bbox.x2
            << ", \"y2\": " << step.occupied_bbox.y2
            << "}, \"desired_x\": " << step.desired_x
            << ", \"desired_y\": " << step.desired_y
            << ", \"contour_y\": " << step.contour_y
            << ", \"right_space\": " << step.right_space
            << ", \"top_space\": " << step.top_space
            << ", \"coupling_extra_space\": " << step.coupling_extra_space
            << ", \"left\": ";
        write_json_optional_string(out, step.left);
        out << ", \"right\": ";
        write_json_optional_string(out, step.right);
        out << ", \"subtree_modules\": ";
        write_json_string_array(out, step.subtree_modules);
        out << ", \"local_wire_segments\": ";
        write_json_string_array(out, step.local_wire_segments);
        out << ", \"cross_child_wire_segments\": ";
        write_json_string_array(out, step.cross_child_wire_segments);
        out << '}';
    }
    out << "\n  ],\n";
    write_routing_topologies_json(out, state.request);
    out << "  \"metrics\": {"
        << "\"area\": " << metrics.area
        << ", \"wirelength\": " << metrics.wirelength
        << ", \"penalty\": " << metrics.penalty
        << ", \"failed_nets\": " << metrics.routing_failures
        << ", \"dp_used\": " << (metrics.dp_used ? "true" : "false")
        << ", \"packing_trace_steps\": " << metrics.packing_trace_steps
        << ", \"packing_time_dp_used\": " << (metrics.packing_time_dp_used ? "true" : "false")
        << ", \"packing_time_dp_segments\": " << metrics.packing_time_dp_segments
        << "}\n";
    out << "}\n";
    return out.str();
}

// 返回指定模块是否是对称 pair 的代表模块。
const SymmetryGroupNode* symmetry_pair_for_representative(const EnhancedBStarTree& tree, const std::string& module) {
    for (const auto& group : tree.symmetry_groups) {
        if (!group.self_symmetric && group.representative == module) return &group;
    }
    return nullptr;
}

// 返回模块旋转后的尺寸。
std::pair<double, double> module_size(const Circuit& circuit, const std::string& module, int angle) {
    return placed_size(circuit.modules.at(module), {module, 0.0, 0.0, angle, "R0"});
}

// 返回代表节点在 packing 中占用的整体尺寸，包含对称镜像模块和轴间预留空间。
std::pair<double, double> occupied_size(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const BStarNode& node,
    const SolverConfig& config) {
    auto size = module_size(circuit, node.module, node.angle);
    const double right_space = node.right_space.required_space();
    const double top_space = node.top_space.required_space();
    const auto* group = symmetry_pair_for_representative(tree, node.module);
    if (group == nullptr || !group->mirror.has_value()) return {size.first + right_space, size.second + top_space};

    const auto mirror_size = module_size(circuit, *group->mirror, node.angle);
    const double gap = std::max(config.spacing, right_space + group->space_cluster.required_space());
    if (group->axis == Axis::Vertical) {
        return {size.first + gap + mirror_size.first + group->space_group.required_space(),
                std::max(size.second, mirror_size.second) + top_space};
    }
    return {std::max(size.first, mirror_size.first) + right_space,
            size.second + gap + mirror_size.second + group->space_group.required_space()};
}

// 返回 contour 在指定 x 区间内的最高 y。
double contour_height(const std::vector<PackedRect>& packed, double x, double width) {
    double result = 0.0;
    const double x2 = x + width;
    for (const auto& rect : packed) {
        if (x < rect.x2 && x2 > rect.x1) result = std::max(result, rect.y2);
    }
    return result;
}

// 将 angle 转成基础 Cadence orient 字符串。
std::string orient_for_angle(int angle) {
    switch ((angle % 360 + 360) % 360) {
        case 0: return "R0";
        case 90: return "R90";
        case 180: return "R180";
        case 270: return "R270";
    }
    return "R0";
}

// 生成对称 pair 中镜像模块的 placement。
Placement mirror_placement(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const BStarNode& node,
    const Placement& representative,
    const SolverConfig& config) {
    const auto* group = symmetry_pair_for_representative(tree, node.module);
    if (group == nullptr || !group->mirror.has_value()) return representative;
    const auto rep_size = module_size(circuit, node.module, representative.angle);
    const double gap = std::max(config.spacing, node.right_space.required_space() + group->space_cluster.required_space());
    Placement mirror{*group->mirror, representative.x, representative.y, representative.angle, representative.orient};
    if (group->axis == Axis::Vertical) {
        mirror.x = representative.x + rep_size.first + gap;
        mirror.orient = "MY";
    } else {
        mirror.y = representative.y + rep_size.second + gap;
        mirror.orient = "MX";
    }
    return mirror;
}

// 按原始模块顺序整理输出顺序，保证文件稳定。
std::vector<std::string> ordered_placements(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    std::vector<std::string> order;
    for (const auto& module : circuit.module_order) {
        if (request.placements.contains(module)) order.push_back(module);
    }
    return order;
}

// 从增强 B*-tree 复制 routing evaluator 所需的轻量拓扑快照。
RoutingTreeSnapshot make_routing_tree_snapshot(const EnhancedBStarTree& tree) {
    RoutingTreeSnapshot snapshot;
    snapshot.root = tree.root;
    for (const auto& [id, node] : tree.nodes) {
        snapshot.nodes.push_back(RoutingTreeNodeRef{id, node.module, node.left, node.right});
    }
    return snapshot;
}

// 返回已放置模块的 active blocker 近似全局矩形。
// 收集指定 B*-tree node 子树中的代表模块，用于把 DP state 对齐到 packing contour step。
std::vector<std::string> subtree_modules_for_trace(const EnhancedBStarTree& tree, const std::string& id) {
    std::vector<std::string> modules;
    const auto found = tree.nodes.find(id);
    if (found == tree.nodes.end()) return modules;
    modules.push_back(found->second.module.empty() ? id : found->second.module);
    if (found->second.left.has_value()) {
        auto left = subtree_modules_for_trace(tree, *found->second.left);
        modules.insert(modules.end(), left.begin(), left.end());
    }
    if (found->second.right.has_value()) {
        auto right = subtree_modules_for_trace(tree, *found->second.right);
        modules.insert(modules.end(), right.begin(), right.end());
    }
    return modules;
}

void append_unique(std::vector<std::string>& values, const std::string& value) {
    if (value.empty()) return;
    if (std::find(values.begin(), values.end(), value) == values.end()) values.push_back(value);
}

std::string segment_key_for_trace(const WireSegmentRef& segment) {
    if (!segment.id.empty()) return segment.id;
    return segment.net + "|" + segment.from + "|" + segment.to;
}

std::unordered_map<std::string, std::string> lcp_owner_map_for_trace(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::string> owners;
    std::unordered_map<std::string, std::string> owner_by_space;
    for (const auto& space : request.space_nodes) {
        owner_by_space[space.id] = space.owner;
        for (const auto& point : space.linking_points) owners[point.id] = space.owner;
    }
    for (const auto& point : request.linking_points) {
        const auto found = owner_by_space.find(point.space_node_id);
        if (found != owner_by_space.end()) owners[point.id] = found->second;
    }
    return owners;
}

std::string terminal_owner_for_trace(
    const std::string& terminal,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const auto dot = terminal.find('.');
    if (dot != std::string::npos) return terminal.substr(0, dot);
    const auto found = lcp_owner_by_id.find(terminal);
    return found == lcp_owner_by_id.end() ? std::string{} : found->second;
}

std::unordered_set<std::string> module_set_for_trace(const std::vector<std::string>& modules) {
    return {modules.begin(), modules.end()};
}

bool segment_inside_trace_modules(
    const WireSegmentRef& segment,
    const std::unordered_set<std::string>& modules,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const auto from = terminal_owner_for_trace(segment.from, lcp_owner_by_id);
    const auto to = terminal_owner_for_trace(segment.to, lcp_owner_by_id);
    return !from.empty() && !to.empty() && modules.contains(from) && modules.contains(to);
}

bool segment_crosses_child_modules(
    const WireSegmentRef& segment,
    const std::unordered_set<std::string>& left,
    const std::unordered_set<std::string>& right,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    if (left.empty() || right.empty()) return false;
    const auto from = terminal_owner_for_trace(segment.from, lcp_owner_by_id);
    const auto to = terminal_owner_for_trace(segment.to, lcp_owner_by_id);
    return (left.contains(from) && right.contains(to)) || (left.contains(to) && right.contains(from));
}

// 根据 packing step 的 subtree 边界显式记录该 step 需要完成的 routing DP transition。
void annotate_packing_time_segments(const EnhancedBStarTree& tree, RoutingEvaluationRequest& request) {
    const auto lcp_owner_by_id = lcp_owner_map_for_trace(request);
    for (auto& step : request.packing_trace.steps) {
        const auto current_modules = module_set_for_trace(step.subtree_modules);
        const auto left_modules = step.left.has_value()
                                      ? module_set_for_trace(subtree_modules_for_trace(tree, *step.left))
                                      : std::unordered_set<std::string>{};
        const auto right_modules = step.right.has_value()
                                       ? module_set_for_trace(subtree_modules_for_trace(tree, *step.right))
                                       : std::unordered_set<std::string>{};
        for (const auto& topology : request.net_topologies) {
            for (const auto& segment : topology.segments) {
                if (!segment_inside_trace_modules(segment, current_modules, lcp_owner_by_id)) continue;
                if (segment_inside_trace_modules(segment, left_modules, lcp_owner_by_id)) continue;
                if (segment_inside_trace_modules(segment, right_modules, lcp_owner_by_id)) continue;
                const auto key = segment_key_for_trace(segment);
                append_unique(step.local_wire_segments, key);
                if (segment_crosses_child_modules(segment, left_modules, right_modules, lcp_owner_by_id)) {
                    append_unique(step.cross_child_wire_segments, key);
                }
            }
        }
    }
}

Rect placed_active_rect(const Module& module, const Placement& placement) {
    return routing::transform_active_to_global(module, placement);
}

// 填充 routing request 中面向 DP/A* 的全局 pin、blocker 和 LCP 候选位置。
void populate_routing_context(const Circuit& circuit, RoutingEvaluationRequest& request) {
    for (const auto& module_id : request.placement_order) {
        const auto& placement = request.placements.at(module_id);
        request.active_region_blockers.push_back(placed_active_rect(circuit.modules.at(module_id), placement));
    }
    for (const auto& key : circuit.pin_order) {
        const auto& pin = circuit.pins.at(key);
        const auto& placement = request.placements.at(pin.module);
        const auto xy = placed_pin(circuit.modules.at(pin.module), pin, placement);
        request.placed_pins.push_back({key, pin.module, pin.name, xy.first, xy.second, pin.layer});
    }
    generate_automatic_lcps(circuit, request);
    std::unordered_map<std::string, NetTopology> topologies;
    for (const auto& [name, net] : circuit.nets) {
        topologies[name].net = name;
        topologies[name].pins = net.terminals;
    }
    for (const auto& point : request.linking_points) {
        std::unordered_set<std::string> emitted_segments;
        for (const auto& segment : point.segments) {
            if (!emitted_segments.insert(segment.id).second) continue;
            auto& topology = topologies[segment.net];
            topology.net = segment.net;
            if (std::none_of(topology.linking_points.begin(), topology.linking_points.end(), [&](const auto& existing) {
                    return existing.id == point.id;
                })) {
                topology.linking_points.push_back(point);
            }
            if (std::none_of(topology.segments.begin(), topology.segments.end(), [&](const auto& existing) {
                    return existing.id == segment.id;
                })) {
                topology.segments.push_back(segment);
            }
        }
    }
    for (auto& [_, topology] : topologies) {
        if (!topology.segments.empty()) request.net_topologies.push_back(std::move(topology));
    }
}

// 计算当前 metrics 的论文归一化总代价。
double compute_phi_cost(Metrics& metrics, const Metrics& base, const SolverConfig& config) {
    metrics.normalized_area = metrics.area / std::max(base.area, 1.0);
    metrics.normalized_wirelength = metrics.wirelength / std::max(base.wirelength, 1.0);
    metrics.normalized_bend = static_cast<double>(metrics.bend_count) / std::max(base.bend_count, 1);
    metrics.normalized_via = static_cast<double>(metrics.via_count) / std::max(base.via_count, 1);
    metrics.phi_cost = config.area_weight * metrics.normalized_area +
                       config.wirelength_weight * metrics.normalized_wirelength +
                       config.bend_weight * metrics.normalized_bend +
                       config.via_weight * metrics.normalized_via +
                       metrics.penalty;
    return metrics.phi_cost;
}

// 评价一棵增强 B*-tree 对应的候选状态。
// 判断当前候选是否已经得到可直接输出的合法 detailed routing。
bool has_clean_detailed_solution(const RoutingFeedback& feedback) {
    const auto& metrics = feedback.metrics;
    return !feedback.routes.empty() &&
           metrics.penalty <= 1e-9 &&
           metrics.design_rule_violations == 0 &&
           metrics.routing_failures == 0 &&
           metrics.traceback_failures == 0;
}

bool feedback_expands_space(
    const RoutingEvaluationRequest& request,
    const RoutingFeedback& feedback,
    double tolerance) {
    for (const auto& space : request.space_nodes) {
        const auto required = feedback.required_space_by_node.find(space.id);
        if (required != feedback.required_space_by_node.end() &&
            required->second > space.required_space() + tolerance) {
            return true;
        }
        const auto coupling = feedback.coupling_space_by_node.find(space.id);
        if (coupling != feedback.coupling_space_by_node.end() &&
            coupling->second > space.coupling_extra_space + tolerance) {
            return true;
        }
    }
    return false;
}

// 在同一个 candidate 内执行有限轮 routing feedback -> re-pack 闭环。
CandidateState evaluate_candidate_with_feedback_loop(
    const Circuit& circuit,
    EnhancedBStarTree tree,
    const SolverConfig& config,
    const Metrics* base_metrics) {
    CandidateState state;
    state.tree = std::move(tree);
    const int max_iterations = std::max(1, config.routing_feedback_iterations);
    bool converged = false;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        state.request = pack_enhanced_tree(circuit, state.tree, config);
        state.feedback = evaluate_with_routing_adapter(circuit, state.request);
        const bool expands_space =
            feedback_expands_space(state.request, state.feedback, config.routing_feedback_tolerance);
        apply_routing_feedback(state.tree, state.feedback);
        state.feedback.metrics.routing_feedback_iterations = iteration + 1;
        state.feedback.metrics.routing_feedback_converged = !expands_space;
        if (!expands_space) {
            converged = true;
            break;
        }
    }
    state.feedback.metrics.routing_feedback_converged = converged;
    if (base_metrics != nullptr) state.cost = compute_phi_cost(state.feedback.metrics, *base_metrics, config);
    return state;
}

CandidateState evaluate_candidate(
    const Circuit& circuit,
    EnhancedBStarTree tree,
    const SolverConfig& config,
    const Metrics& base_metrics) {
    return evaluate_candidate_with_feedback_loop(circuit, std::move(tree), config, &base_metrics);
}

// 将候选状态转换为最终 Solution。
Solution make_solution(const CandidateState& state) {
    Solution solution;
    solution.placements = state.request.placements;
    solution.placement_order = state.request.placement_order;
    solution.routes = state.feedback.routes;
    solution.metrics = state.feedback.metrics;
    solution.routing_cost = state.feedback.routing_cost;
    solution.routing_candidate_count = state.feedback.routing_candidate_count;
    solution.detailed_route_count = static_cast<std::size_t>(state.feedback.metrics.detailed_routes);
    solution.traceback_failures = state.feedback.metrics.traceback_failures;
    solution.space_nodes_with_routes = state.feedback.metrics.space_nodes_with_routes;
    solution.dp_nodes = state.feedback.metrics.dp_nodes;
    solution.dp_states = state.feedback.metrics.dp_states;
    solution.dp_pruned_states = state.feedback.metrics.dp_pruned_states;
    solution.dp_traceback_segments = state.feedback.metrics.dp_traceback_segments;
    solution.packing_trace_steps = state.feedback.metrics.packing_trace_steps;
    solution.space_feedback_nodes = state.feedback.metrics.space_feedback_nodes;
    solution.routing_feedback_iterations = state.feedback.metrics.routing_feedback_iterations;
    solution.packing_time_dp_segments = state.feedback.metrics.packing_time_dp_segments;
    solution.routing_feedback_converged = state.feedback.metrics.routing_feedback_converged;
    solution.packing_time_dp_used = state.feedback.metrics.packing_time_dp_used;
    solution.dp_used = state.feedback.metrics.dp_used;
    solution.btree_trace_json = make_btree_trace_json(state);
    solution.routing_warnings = state.feedback.metrics.routing_warnings;
    return solution;
}

// 检查 FLOW 约束是否在当前临时路由中有可追踪的端点。
int count_flow_violations(const Circuit& circuit, const RoutingEvaluationRequest& request, const std::vector<RouteSegment>& routes) {
    std::unordered_map<std::string, PlacedPin> pins;
    for (const auto& pin : request.placed_pins) pins[pin.key] = pin;
    int violations = 0;
    for (const auto& flow : circuit.constraints.flows) {
        if (!pins.contains(flow.out_pin) || !pins.contains(flow.in_pin)) {
            ++violations;
            continue;
        }
        bool has_route = false;
        for (const auto& route : routes) {
            if (route.net == flow.net) {
                has_route = true;
                break;
            }
        }
        if (!has_route) ++violations;
    }
    return violations;
}

// 检查当前 route 是否违反线宽范围。
int count_current_density_violations(const Circuit& circuit, const std::vector<RouteSegment>& routes) {
    int violations = 0;
    for (const auto& route : routes) {
        const auto found = circuit.constraints.wire_widths.find(route.net);
        if (found == circuit.constraints.wire_widths.end()) continue;
        if (route.width < found->second.min_width || route.width > found->second.max_width) ++violations;
    }
    return violations;
}

}  // namespace

// 按增强 B*-tree 和 ASF 对称组生成当前候选布局。
RoutingEvaluationRequest pack_enhanced_tree(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config) {
    RoutingEvaluationRequest request;
    if (!tree.root.has_value()) return request;

    std::vector<PackedRect> packed;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, double, double)> place_node = [&](const std::string& id, double desired_x, double desired_y) {
        if (visited.contains(id)) return;
        visited.insert(id);
        const auto& node = tree.nodes.at(id);
        const auto occupied = occupied_size(circuit, tree, node, config);
        const double x = desired_x;
        const double contour_y = contour_height(packed, x, occupied.first);
        const double y = std::max(desired_y, contour_y);

        Placement placement{id, x, y, node.angle, orient_for_angle(node.angle)};
        request.placements[id] = placement;
        const auto* pair_group = symmetry_pair_for_representative(tree, id);
        if (pair_group != nullptr && pair_group->mirror.has_value()) {
            request.placements[*pair_group->mirror] = mirror_placement(circuit, tree, node, placement, config);
        }
        const double group_coupling_space =
            pair_group == nullptr ? 0.0 : pair_group->space_group.coupling_extra_space + pair_group->space_cluster.coupling_extra_space;

        request.packing_trace.steps.push_back(PackingContourStep{
            id,
            node.module,
            x,
            y,
            Rect{x, y, x + occupied.first, y + occupied.second},
            desired_x,
            desired_y,
            contour_y,
            node.right_space.required_space(),
            node.top_space.required_space(),
            node.right_space.coupling_extra_space + node.top_space.coupling_extra_space + group_coupling_space,
            node.left,
            node.right,
            subtree_modules_for_trace(tree, id),
            {},
            {},
        });
        packed.push_back({x, y, x + occupied.first, y + occupied.second});
        if (node.left.has_value()) {
            place_node(*node.left, x + occupied.first + config.spacing, y);
        }
        if (node.right.has_value()) {
            place_node(*node.right, x, y + occupied.second + config.spacing);
        }
    };

    place_node(*tree.root, 0.0, 0.0);
    request.placement_order = ordered_placements(circuit, request);
    request.space_nodes = collect_space_nodes(tree);
    request.tree = make_routing_tree_snapshot(tree);
    populate_routing_context(circuit, request);
    annotate_packing_time_segments(tree, request);
    return request;
}

// 使用 routing adapter 评价当前 placement candidate。
RoutingFeedback evaluate_with_routing_adapter(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    RoutingFeedback feedback;
    const auto routing_evaluation = evaluate_routing(circuit, request);
    const auto detailed = run_detailed_routing(circuit, request, routing_evaluation);
    feedback.routes = detailed.routes;
    Solution solution;
    solution.placements = request.placements;
    solution.placement_order = request.placement_order;
    solution.routes = feedback.routes;
    feedback.metrics = measure(circuit, solution);
    feedback.metrics.wirelength = routing_evaluation.global_routing.total_metrics.wirelength;
    feedback.metrics.bend_count = routing_evaluation.global_routing.total_metrics.bend_count;
    feedback.metrics.via_count = routing_evaluation.global_routing.total_metrics.via_count;
    feedback.metrics.flow_violations = count_flow_violations(circuit, request, feedback.routes);
    feedback.metrics.current_density_violations = count_current_density_violations(circuit, feedback.routes);
    feedback.metrics.flow_violations += detailed.flow_violations;
    feedback.metrics.current_density_violations += detailed.current_density_violations;
    feedback.metrics.design_rule_violations = detailed.design_rule_violations;
    feedback.metrics.design_rule_penalty = detailed.design_rule_penalty;
    feedback.metrics.routing_failures = routing_evaluation.failed_nets;
    feedback.metrics.congestion_penalty = 0.0;
    feedback.metrics.flow_penalty = std::max(routing_evaluation.global_routing.flow_penalty, detailed.flow_penalty);
    feedback.metrics.current_density_penalty =
        std::max(routing_evaluation.global_routing.current_density_penalty, detailed.current_density_penalty);
    feedback.metrics.coupling_penalty =
        detailed.routes.empty() ? routing_evaluation.global_routing.coupling_penalty : detailed.coupling_penalty;
    feedback.metrics.routing_failure_penalty =
        routing_evaluation.global_routing.routing_failure_penalty + detailed.routing_failure_penalty;
    feedback.metrics.detailed_routing_penalty = detailed.design_rule_penalty + detailed.routing_failure_penalty;
    feedback.metrics.detailed_cost = detailed.detailed_cost;
    feedback.metrics.detailed_routes = static_cast<int>(detailed.routes.size());
    feedback.metrics.traceback_failures = detailed.traceback_failures;
    feedback.metrics.routing_warnings = detailed.report.warnings;
    for (const auto& net_route : routing_evaluation.global_routing.net_routes) {
        if (!net_route.success && !net_route.message.empty()) {
            feedback.metrics.routing_warnings.push_back(
                net_route.net + ": global routing failed: " + net_route.message);
        }
    }
    feedback.metrics.space_nodes_with_routes = detailed.space_nodes_with_routes;
    feedback.metrics.packing_trace_steps = static_cast<int>(request.packing_trace.steps.size());
    feedback.metrics.dp_used = routing_evaluation.used_bottom_up_dp;
    if (routing_evaluation.bottom_up_dp.has_value()) {
        feedback.metrics.dp_nodes = routing_evaluation.bottom_up_dp->dp_nodes;
        feedback.metrics.dp_states = routing_evaluation.bottom_up_dp->dp_states;
        feedback.metrics.dp_pruned_states = routing_evaluation.bottom_up_dp->dp_pruned_states;
        feedback.metrics.dp_traceback_segments = static_cast<int>(routing_evaluation.bottom_up_dp->traceback_candidates.size());
        feedback.metrics.packing_time_dp_segments = routing_evaluation.bottom_up_dp->packing_time_dp_segments;
        feedback.metrics.packing_time_dp_used = routing_evaluation.bottom_up_dp->packing_time_dp_used;
    }
    feedback.metrics.penalty =
        feedback.metrics.flow_penalty + feedback.metrics.current_density_penalty +
        feedback.metrics.coupling_penalty + feedback.metrics.design_rule_penalty +
        feedback.metrics.routing_failure_penalty;
    feedback.routing_cost = routing_evaluation.routing_cost;
    feedback.routing_candidate_count = routing_evaluation.candidates.size();

    int space_feedback_nodes = 0;
    for (const auto& space : request.space_nodes) {
        const auto detailed_space = detailed.required_space_by_node.find(space.id);
        const double required_space =
            detailed_space == detailed.required_space_by_node.end()
                ? space.required_space()
                : std::max(space.required_space(), detailed_space->second);
        feedback.required_space_by_node[space.id] = required_space;
        const auto detailed_coupling = detailed.coupling_space_by_node.find(space.id);
        const double coupling_space =
            detailed_coupling == detailed.coupling_space_by_node.end()
                ? (routing_evaluation.global_routing.coupling_penalty > 0.0 ? 1.0 : 0.0)
                : detailed_coupling->second;
        feedback.coupling_space_by_node[space.id] = coupling_space;
        const bool has_required_feedback =
            detailed_space != detailed.required_space_by_node.end() && detailed_space->second > space.required_space() + 1e-9;
        const bool has_coupling_feedback = coupling_space > 1e-9;
        if (has_required_feedback || has_coupling_feedback) ++space_feedback_nodes;
    }
    feedback.metrics.space_feedback_nodes = space_feedback_nodes;
    return feedback;
}

// 使用论文 placement 框架、SA 和 routing adapter 生成布局布线解。
Solution solve_placement_aware(const Circuit& circuit, const SolverConfig& config) {
    const auto errors = validate_circuit(circuit);
    if (!errors.empty()) {
        std::string message = "invalid circuit:";
        for (const auto& error : errors) message += "\n- " + error;
        throw std::runtime_error(message);
    }

    auto initial_tree = make_enhanced_tree(circuit);
    auto current = evaluate_candidate_with_feedback_loop(circuit, std::move(initial_tree), config, nullptr);
    const Metrics base_metrics = current.feedback.metrics;
    current.cost = compute_phi_cost(current.feedback.metrics, base_metrics, config);
    CandidateState best = current;
    if (has_clean_detailed_solution(best.feedback)) {
        return make_solution(best);
    }

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    double temperature = config.initial_temperature;
    for (int iteration = 0; iteration < config.sa_iterations; ++iteration) {
        auto next_tree = current.tree;
        perturb_placement_tree(next_tree, rng);
        auto next = evaluate_candidate(circuit, std::move(next_tree), config, base_metrics);
        const double delta = next.cost - current.cost;
        const bool accept = delta <= 0.0 || probability(rng) < std::exp(-delta / std::max(temperature, 1e-9));
        if (accept) current = std::move(next);
        if (current.cost < best.cost) best = current;
        if (has_clean_detailed_solution(best.feedback)) {
            break;
        }
        temperature *= config.cooling_rate;
    }

    return make_solution(best);
}

// 保持历史 CLI/API 名称，当前默认指向论文 placement-aware 求解流程。
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config) {
    return solve_placement_aware(circuit, config);
}

}  // namespace sapr
