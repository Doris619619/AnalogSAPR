// 定义模拟电路、约束、增强 B*-tree、布局布线结果和评价指标的公共数据模型。
#pragma once

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

// 汇总布局和布线结果，并保存放置输出顺序。
struct Solution {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<RouteSegment> routes;
};

// 表示当前解的基础评价指标。
struct Metrics {
    double area{};
    double wirelength{};
    int bend_count{};
    int via_count{};
    double penalty{};
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
};

// 表示 linking-control point 连接的一条逻辑线段。
struct WireSegmentRef {
    std::string net;
    double min_width{};
    double max_width{};
    std::optional<std::string> direction;
};

// 表示增强 B*-tree 中的拓扑控制点。
struct LinkingControlPoint {
    std::string id;
    std::string space_node_id;
    std::vector<WireSegmentRef> segments;

    // 返回该控制点需要的最大线宽。
    [[nodiscard]] double required_width() const;
};

// 表示器件右侧、上侧或对称组内的预留布线空间。
struct SpaceNode {
    std::string id;
    std::string owner;
    SpaceNodeKind kind{SpaceNodeKind::Right};
    std::vector<LinkingControlPoint> linking_points;

    // 按论文公式计算该空间节点需要预留的宽度。
    [[nodiscard]] double required_space() const;
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

// 表示一个 ASF-B*-tree 对称组在主树中的约束摘要。
struct SymmetryGroupNode {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string representative;
    std::optional<std::string> mirror;
    bool self_symmetric{};
    SpaceNode space_group;
    SpaceNode space_cluster;
};

// 表示一次候选布局的路由侧评价输入。
struct RoutingEvaluationRequest {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<SpaceNode> space_nodes;
};

// 表示路由 adapter 返回给 placement/SA 的反馈。
struct RoutingFeedback {
    std::vector<RouteSegment> routes;
    Metrics metrics;
    std::unordered_map<std::string, double> required_space_by_node;
};

// 表示当前阶段的增强 B*-tree 拓扑。
struct EnhancedBStarTree {
    std::optional<std::string> root;
    std::unordered_map<std::string, BStarNode> nodes;
    std::vector<std::string> representative_order;
    std::vector<SymmetryGroupNode> symmetry_groups;
};

}  // namespace sapr
