// 文件职责：实现 enhanced B*-tree 可视化 JSON 导出与 SA 轮次元数据注入。
#include "sapr/btree_trace.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace sapr {
namespace {

// 转义 JSON 字符串中的特殊字符。
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

// 写出 JSON 字符串字面量。
void write_json_string(std::ostringstream& out, const std::string& value) {
    out << '"' << json_escape(value) << '"';
}

// 写出可选字符串，空值写 null。
void write_json_optional_string(std::ostringstream& out, const std::optional<std::string>& value) {
    if (value.has_value()) {
        write_json_string(out, *value);
    } else {
        out << "null";
    }
}

// 写出字符串数组。
void write_json_string_array(std::ostringstream& out, const std::vector<std::string>& values) {
    out << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ',';
        write_json_string(out, values[index]);
    }
    out << ']';
}

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

// 收集 space node 内的 LCP 标识。
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

// 写出 SA 单轮元数据对象。
void write_sa_iteration_json(std::ostringstream& out, const SaBtreeIterationTrace& iteration) {
    out << "  \"sa_iteration\": {"
        << "\"index\": " << iteration.iteration
        << ", \"total\": " << iteration.sa_iterations
        << ", \"move\": ";
    write_json_string(out, iteration.move);
    out << ", \"changed\": " << (iteration.changed ? "true" : "false")
        << ", \"accept\": " << (iteration.accept ? "true" : "false")
        << ", \"next_cost\": " << iteration.next_cost
        << ", \"current_cost_before\": " << iteration.current_cost_before
        << ", \"temperature\": " << iteration.temperature
        << "}";
}

}  // namespace

// 将候选树、packing 与 routing topology 导出为 btree_trace JSON。
std::string make_btree_trace_json(
    const EnhancedBStarTree& tree,
    const RoutingEvaluationRequest& request,
    const Metrics& metrics) {
    std::ostringstream out;
    out << "{\n";
    out << "  \"root\": ";
    write_json_optional_string(out, tree.root);
    out << ",\n";
    out << "  \"nodes\": [\n";
    bool first_node = true;
    for (const auto& id : tree.representative_order) {
        const auto found = tree.nodes.find(id);
        if (found == tree.nodes.end()) continue;
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
    for (std::size_t index = 0; index < request.placement_order.size(); ++index) {
        const auto& module = request.placement_order[index];
        const auto found = request.placements.find(module);
        if (found == request.placements.end()) continue;
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
    for (std::size_t index = 0; index < request.packing_trace.steps.size(); ++index) {
        const auto& step = request.packing_trace.steps[index];
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
    write_routing_topologies_json(out, request);
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

// 在已有 btree_trace JSON 中写入 SA 单轮扰动元数据，供结构图标题展示。
std::string enrich_btree_trace_with_sa_iteration(
    const std::string& btree_trace_json,
    const SaBtreeIterationTrace& iteration) {
    std::ostringstream meta;
    write_sa_iteration_json(meta, iteration);
    const std::string meta_block = meta.str();

    // 在首个 '{' 后插入 sa_iteration 字段，避免依赖完整 JSON 解析。
    const auto open = btree_trace_json.find('{');
    if (open == std::string::npos) {
        std::ostringstream fallback;
        fallback << "{\n" << meta_block << "\n}\n";
        return fallback.str();
    }
    std::string enriched;
    enriched.reserve(btree_trace_json.size() + meta_block.size() + 8);
    enriched.append(btree_trace_json, 0, open + 1);
    enriched.push_back('\n');
    enriched.append(meta_block);
    enriched.append(",\n");
    std::size_t rest = open + 1;
    while (rest < btree_trace_json.size() &&
           (btree_trace_json[rest] == '\n' || btree_trace_json[rest] == '\r' || btree_trace_json[rest] == ' ')) {
        ++rest;
    }
    // 保持与原 JSON 相同的两空格缩进风格。
    if (rest < btree_trace_json.size() && btree_trace_json[rest] == '"') {
        enriched.append("  ");
    }
    enriched.append(btree_trace_json, rest, std::string::npos);
    return enriched;
}

// 生成带 SA 元数据的单轮候选树可视化记录。
SaBtreeIterationTrace make_sa_btree_iteration_trace(
    const EnhancedBStarTree& tree,
    const RoutingEvaluationRequest& request,
    const Metrics& metrics,
    const SaBtreeIterationTrace& meta) {
    SaBtreeIterationTrace result = meta;
    const auto base_json = make_btree_trace_json(tree, request, metrics);
    result.btree_trace_json = enrich_btree_trace_with_sa_iteration(base_json, result);
    return result;
}

}  // namespace sapr
