// 定义模拟电路、约束、增强 B*-tree、布局布线结果和评价指标的公共数据模型。
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace sapr {

// 表示线网的布线优先级。
enum class Priority { Critical, Symmetry, Normal };

// 表示对称约束的轴方向。
enum class Axis { Vertical, Horizontal };

// 表示增强 B*-tree 中预留布线空间的论文语义。
enum class SpaceNodeKind { Right, Top, Group, Cluster };

// 表示 LCP 线段相对 FLOW 约束的逻辑电流方向。
enum class CurrentDirection { Unknown, In, Out };

// 表示二维轴对齐矩形。
struct Rect {
    double x1{};
    double y1{};
    double x2{};
    double y2{};

    // 返回矩形宽度。
    [[nodiscard]] double width() const { return x2 - x1; }
    // 返回矩形高度。
    [[nodiscard]] double height() const { return y2 - y1; }
};

// 表示一个待放置器件。
struct Module {
    std::string id;
    double width{};
    double height{};
    Rect active;
    double ox{};
    double oy{};
    std::string info;
};

// 表示器件上的一个引脚。
struct Pin {
    std::string module;
    std::string name;
    double x{};
    double y{};
    std::string layer;

    // 返回全局唯一的 module.pin 键。
    [[nodiscard]] std::string key() const { return module + "." + name; }
};

// 表示一个多端线网。
struct Net {
    std::string name;
    Priority priority{Priority::Normal};
    std::vector<std::string> terminals;
};

// 表示一对器件的对称约束。
struct SymmetryPair {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string left;
    std::string right;
};

// 表示器件自身的对称约束。
struct SymmetrySelf {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string module;
};

// 表示线网上的电流流向约束。
struct FlowConstraint {
    std::string net;
    std::string out_pin;
    std::string in_pin;
};

// 表示线网允许的线宽范围。
struct WireWidthConstraint {
    std::string net;
    double min_width{};
    double max_width{};
};

// 汇总电路的全部约束。
struct Constraints {
    std::vector<SymmetryPair> symmetry_pairs;
    std::vector<SymmetrySelf> symmetry_selfs;
    std::unordered_map<std::string, WireWidthConstraint> wire_widths;
    std::vector<FlowConstraint> flows;
};

// 表示完整的算法输入，并保存稳定的原始顺序。
struct Circuit {
    std::unordered_map<std::string, Module> modules;
    std::unordered_map<std::string, Pin> pins;
    std::unordered_map<std::string, Net> nets;
    Constraints constraints;
    std::vector<std::string> module_order;
    std::vector<std::string> pin_order;
    std::vector<std::string> net_order;
};

// 表示一个器件的放置结果。
struct Placement {
    std::string module;
    double x{};
    double y{};
    int angle{};
    std::string orient{"R0"};
};

// 表示一条中心线布线段。
struct RouteSegment {
    std::string net;
    std::string layer;
    double x1{};
    double y1{};
    double x2{};
    double y2{};
    double width{};
};

// 表示当前解的基础评价指标和论文约束违例统计。
struct Metrics {
    double area{};
    double wirelength{};
    int bend_count{};
    int via_count{};
    double phi_cost{};
    double normalized_area{};
    double normalized_wirelength{};
    double normalized_bend{};
    double normalized_via{};
    double penalty{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double routing_failure_penalty{};
    double design_rule_penalty{};
    double detailed_routing_penalty{};
    double detailed_cost{};
    double row_width_overflow{};
    double row_width_penalty{};
    int flow_violations{};
    int current_density_violations{};
    int design_rule_violations{};
    int routing_failures{};
    int detailed_routes{};
    int traceback_failures{};
    int space_nodes_with_routes{};
    int dp_nodes{};
    int dp_states{};
    int dp_pruned_states{};
    int dp_traceback_segments{};
    int packing_trace_steps{};
    int space_feedback_nodes{};
    int routing_feedback_iterations{};
    int packing_time_dp_segments{};
    bool routing_feedback_converged{};
    bool packing_time_dp_used{};
    bool dp_used{};
    double congestion_penalty{};
    std::vector<std::string> routing_warnings;
};

// 记录 SA 单轮候选树可视化所需的扰动与接受信息。
struct SaBtreeIterationTrace {
    int iteration{};
    int sa_iterations{};
    std::string move;
    bool changed{};
    bool accept{};
    double next_cost{};
    double current_cost_before{};
    double temperature{};
    std::string btree_trace_json;
    // 保存本轮候选解的布局布线文本输出所需数据，用于逐轮渲染 layout。
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<RouteSegment> routes;
};

// 汇总布局和布线结果，并可保存求解时的路由评价快照。
struct Solution {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<RouteSegment> routes;
    std::optional<Metrics> metrics;
    std::optional<double> routing_cost;
    std::optional<std::size_t> routing_candidate_count;
    std::optional<std::size_t> detailed_route_count;
    std::optional<int> traceback_failures;
    std::optional<int> space_nodes_with_routes;
    std::optional<int> dp_nodes;
    std::optional<int> dp_states;
    std::optional<int> dp_pruned_states;
    std::optional<int> dp_traceback_segments;
    std::optional<int> packing_trace_steps;
    std::optional<int> space_feedback_nodes;
    std::optional<int> routing_feedback_iterations;
    std::optional<int> packing_time_dp_segments;
    std::optional<bool> routing_feedback_converged;
    std::optional<bool> packing_time_dp_used;
    std::optional<bool> dp_used;
    std::optional<std::string> btree_trace_json;
    std::optional<std::string> routing_debug_json;
    std::vector<std::string> routing_warnings;
    std::vector<SaBtreeIterationTrace> sa_btree_iterations;
};

// 配置求解器的确定性参数和论文代价函数权重。
struct SolverConfig {
    double spacing{5.0};
    double row_width{40.0};
    unsigned int seed{1};
    int sa_iterations{250};
    double initial_temperature{5.0};
    double cooling_rate{0.96};
    double area_weight{1.0};
    double wirelength_weight{1.0};
    double bend_weight{0.2};
    double via_weight{0.2};
    double row_width_weight{1.0};
    int routing_feedback_iterations{2};
    double routing_feedback_tolerance{1e-6};
    bool debug_search{};
    double boundary_margin{-1.0};
    double boundary_clearance{0.0};
    bool dump_sa_btree{true};
};

// 表示一次 SA 扰动的调试摘要，供命令行诊断搜索状态是否真实变化。
struct PerturbationReport {
    std::string move;
    bool changed{};
    bool used_lcp_move{};
    std::size_t lcp_before{};
    std::size_t lcp_after{};
};

// 表示 linking-control point 连接的一条逻辑线段。
struct WireSegmentRef {
    std::string net;
    std::string from;
    std::string to;
    double min_width{};
    double max_width{};
    std::optional<std::string> direction;
    CurrentDirection current_direction{CurrentDirection::Unknown};
    std::string id;
};

// 表示 linking-control point 在所属 space node 内的候选物理位置。
struct PhysicalLocationCandidate {
    double x{};
    double y{};
    std::string id;
    std::string validity_level{"strict"};
    bool is_fallback{};
    double penalty{};
    std::string reason;
    std::string source;
    bool inside_space_region{};
};

// 表示增强 B*-tree 中的拓扑控制点。
struct LinkingControlPoint {
    std::string id;
    std::string space_node_id;
    std::vector<WireSegmentRef> segments;
    std::vector<PhysicalLocationCandidate> location_candidates;

    // 返回该控制点需要的最大线宽。
    [[nodiscard]] double required_width() const;
};

// 表示器件右侧、上侧或对称组内的预留布线空间。
struct SpaceNode {
    std::string id;
    std::string owner;
    SpaceNodeKind kind{SpaceNodeKind::Right};
    std::vector<LinkingControlPoint> linking_points;
    double allocated_space{};
    std::vector<PhysicalLocationCandidate> location_candidates;
    double coupling_extra_space{};
    std::optional<Rect> physical_region;

    // 按论文公式计算该空间节点需要预留的宽度。
    [[nodiscard]] double required_space() const;
};

// 表示 ASF 对称组中的一组镜像或轴间 space node。
struct SpaceNodeBundle {
    std::vector<SpaceNode> spaces;
};

// 表示增强 B*-tree 的一个器件节点。
struct BStarNode {
    std::string module;
    std::optional<std::string> parent;
    std::optional<std::string> left;
    std::optional<std::string> right;
    int angle{};
    SpaceNode right_space;
    SpaceNode top_space;
};

// 表示一个 ASF-B*-tree 对称组在主树中的层次约束摘要。
struct SymmetryGroupNode {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string representative;
    std::optional<std::string> mirror;
    bool self_symmetric{};
    std::vector<std::string> stored_modules;
    std::optional<std::string> right_most_module;
    SpaceNode space_group;
    SpaceNode space_cluster;
    std::vector<std::string> half_tree_nodes;
    std::unordered_map<std::string, std::string> mirror_map;
    std::vector<std::string> self_nodes;
    std::vector<std::string> right_most_branch;
    SpaceNodeBundle space_group_bundle;
    SpaceNodeBundle space_cluster_bundle;
};

// 表示一条 net 在增强 B*-tree 中的 LCP 拓扑摘要。
struct NetTopology {
    std::string net;
    std::vector<std::string> pins;
    std::vector<LinkingControlPoint> linking_points;
    std::vector<WireSegmentRef> segments;
};

// 表示 routing evaluator 所需的轻量 B*-tree 节点快照。
struct RoutingTreeNodeRef {
    std::string id;
    std::string module;
    std::optional<std::string> left;
    std::optional<std::string> right;
};

// 表示当前 placement candidate 对应的 B*-tree 拓扑快照。
struct RoutingTreeSnapshot {
    std::optional<std::string> root;
    std::vector<RoutingTreeNodeRef> nodes;
};

// 表示路由评价时已经展开到全局坐标的引脚。
struct PlacedPin {
    std::string key;
    std::string module;
    std::string pin;
    double x{};
    double y{};
    std::string layer;
};

// 表示一次候选布局的路由侧评价输入。
// 表示一次 contour packing 中某个 B*-tree node 的中间状态。
struct PackingContourStep {
    std::string tree_node;
    std::string module;
    double x{};
    double y{};
    Rect occupied_bbox;
    double desired_x{};
    double desired_y{};
    double contour_y{};
    double right_space{};
    double top_space{};
    double coupling_extra_space{};
    std::optional<std::string> left;
    std::optional<std::string> right;
    std::vector<std::string> subtree_modules;
    std::vector<std::string> local_wire_segments;
    std::vector<std::string> cross_child_wire_segments;
};

// 表示当前候选布局的 contour packing 过程快照。
struct PackingContourTrace {
    std::vector<PackingContourStep> steps;
};

struct RoutingEvaluationRequest {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<SpaceNode> space_nodes;
    std::vector<PlacedPin> placed_pins;
    std::vector<LinkingControlPoint> linking_points;
    std::vector<NetTopology> net_topologies;
    std::vector<Rect> active_region_blockers;
    RoutingTreeSnapshot tree;
    PackingContourTrace packing_trace;
    unsigned int lcp_candidate_seed{};
};

// 表示路由 adapter 返回给 placement/SA 的反馈。
struct RoutingFeedback {
    std::vector<RouteSegment> routes;
    Metrics metrics;
    std::unordered_map<std::string, double> required_space_by_node;
    std::unordered_map<std::string, double> coupling_space_by_node;
    double routing_cost{};
    std::size_t routing_candidate_count{};
    std::optional<std::string> routing_debug_json;
};

// 表示 detailed routing 回溯中的一个拓扑节点。
struct DetailedRouteNode {
    std::string id;
    std::string kind;
    std::string space_node_id;
    double x{};
    double y{};
    std::string layer;
};

// 表示 detailed routing 输出线段与 LCP/space-node 拓扑的映射关系。
struct DetailedRouteSegment {
    std::size_t route_index{};
    int dp_state_id{-1};
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::string tree_node;
    std::string segment_id;
    std::string lcp_id;
    std::string lcp_candidate_id;
    std::string space_node_id;
};

// 表示一条 net 的 top-down detailed routing 回溯摘要。
struct DetailedRouteTrace {
    std::string net;
    std::vector<DetailedRouteNode> nodes;
    std::vector<DetailedRouteSegment> segments;
    std::vector<std::string> warnings;
};

// 汇总 detailed routing 的可解释报告。
struct DetailedRoutingReport {
    std::vector<DetailedRouteTrace> traces;
    std::vector<std::string> warnings;
    std::vector<std::string> coupling_pairs;
    std::vector<std::string> design_rule_segments;
    // 记录详细布线阶段违反 FLOW 方向的 net 或 segment。
    std::vector<std::string> flow_segments;
    // 记录详细布线阶段违反 WIRE_WIDTH/current-density 代理约束的 segment。
    std::vector<std::string> current_density_segments;
};

// 表示 top-down performance-aware detailed routing 的输出。
struct DetailedRoutingResult {
    std::vector<RouteSegment> routes;
    DetailedRoutingReport report;
    std::unordered_map<std::string, double> required_space_by_node;
    std::unordered_map<std::string, double> coupling_space_by_node;
    double detailed_cost{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double design_rule_penalty{};
    double routing_failure_penalty{};
    double detailed_routing_penalty{};
    int flow_violations{};
    int current_density_violations{};
    int design_rule_violations{};
    int traceback_failures{};
    int space_nodes_with_routes{};
    bool used_global_fallback{};
};

// 表示当前阶段的增强 B*-tree 拓扑。
struct EnhancedBStarTree {
    std::optional<std::string> root;
    std::unordered_map<std::string, BStarNode> nodes;
    std::vector<std::string> representative_order;
    std::vector<SymmetryGroupNode> symmetry_groups;
};

}  // namespace sapr
