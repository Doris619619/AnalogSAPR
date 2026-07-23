// 文件职责：验证 routing evaluator 可以在样例输入上完成 A*/DP 布线评估。
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/routing/dp_router.hpp"
#include "sapr/routing/geometry.hpp"
#include "sapr/routing/global_router.hpp"
#include "sapr/routing/grid.hpp"
#include "sapr/routing/path_geometry.hpp"
#include "sapr/routing/routing_context.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "sapr/router.hpp"
#include "sapr/tree.hpp"
#include "test_support.hpp"

namespace {

// 返回测试使用的输入目录。
std::filesystem::path source_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "input";
}

// 返回 4ring mixed LCP/direct net 回归用例输入目录。
std::filesystem::path mixed_lcp_direct_case_input_dir() {
    return std::filesystem::path(SAPR_SOURCE_DIR) / "cases" / "4ring_2_left6_no_symmetry" / "input";
}

// 构造一个 active 小于 bbox 的最小电路，用于检查 DRC blocker 是否使用真实 active。
sapr::Circuit make_active_region_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{4.0, 4.0, 6.0, 6.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 0.0, 1.0, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 9.0, 1.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 构造 active 覆盖完整 bbox 的最小电路，用于验证 full-bbox active 仍然是布线障碍。
sapr::Circuit make_full_active_region_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{0.0, 0.0, 10.0, 10.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 0.0, 5.0, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 10.0, 5.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 构造 pin 位于左下边界附近的 full-active 电路，用于验证 grid 会纳入向外逃逸空间。
sapr::Circuit make_boundary_access_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 10.0, 10.0, sapr::Rect{0.0, 0.0, 10.0, 10.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    circuit.pins.emplace("M.A", sapr::Pin{"M", "A", 5.0, 0.1, "M1"});
    circuit.pins.emplace("M.B", sapr::Pin{"M", "B", 9.9, 5.0, "M1"});
    circuit.pin_order = {"M.A", "M.B"};
    circuit.nets.emplace("N", sapr::Net{"N", sapr::Priority::Normal, {"M.A", "M.B"}});
    circuit.net_order.push_back("N");
    return circuit;
}

// 判断 route 金属是否穿过 active 的核心区域，排除边缘 pin access 的短接入段。
bool route_crosses_active_core(const sapr::RouteSegment& route, const sapr::Rect& active) {
    const sapr::Rect normalized = sapr::routing::normalize_rect(active);
    const sapr::Rect core{
        normalized.x1 + 2.0,
        normalized.y1 + 2.0,
        normalized.x2 - 2.0,
        normalized.y2 - 2.0,
    };
    const sapr::Rect metal = sapr::routing::segment_to_rect(
        sapr::routing::Segment{sapr::routing::Point{route.x1, route.y1}, sapr::routing::Point{route.x2, route.y2}},
        route.width);
    return sapr::routing::intersects(metal, core);
}

// 构造一条水平 selected candidate，让 detailed routing 只测试 DRC 统计逻辑。
// 判断 detailed 输出中是否存在异网同层金属短路。
bool has_cross_net_same_layer_short(const std::vector<sapr::RouteSegment>& routes) {
    for (std::size_t left = 0; left < routes.size(); ++left) {
        for (std::size_t right = left + 1; right < routes.size(); ++right) {
            if (sapr::routing::same_layer_short(routes[left], routes[right])) return true;
        }
    }
    return false;
}

// 判断指定 net 的输出线段是否按 routing.txt 语义首尾连续。
bool net_routes_are_contiguous(const std::vector<sapr::RouteSegment>& routes, const std::string& net) {
    std::vector<const sapr::RouteSegment*> net_routes;
    for (const auto& route : routes) {
        if (route.net == net) net_routes.push_back(&route);
    }
    if (net_routes.empty()) return false;
    for (std::size_t index = 1; index < net_routes.size(); ++index) {
        const auto& previous = *net_routes[index - 1];
        const auto& current = *net_routes[index];
        if (!approx(previous.x2, current.x1, 1e-6) || !approx(previous.y2, current.y1, 1e-6)) return false;
    }
    return true;
}

// 判断指定 net 是否至少包含一个同点异层连接，对应 routing.txt 的隐式 via。
bool net_has_implicit_via(const std::vector<sapr::RouteSegment>& routes, const std::string& net) {
    std::vector<const sapr::RouteSegment*> net_routes;
    for (const auto& route : routes) {
        if (route.net == net) net_routes.push_back(&route);
    }
    for (std::size_t index = 1; index < net_routes.size(); ++index) {
        const auto& previous = *net_routes[index - 1];
        const auto& current = *net_routes[index];
        if (approx(previous.x2, current.x1, 1e-6) &&
            approx(previous.y2, current.y1, 1e-6) &&
            previous.layer != current.layer) {
            return true;
        }
    }
    return false;
}

// 构造一条水平 selected candidate，让 detailed routing 只测试 DRC 统计逻辑。
sapr::RoutingEvaluation make_line_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    double y,
    int layer = 0,
    const sapr::routing::GridConfig& config = {}) {
    sapr::routing::RoutingContext context(circuit, placements, config);
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = "M.A";
    candidate.to_terminal = "M.B";
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, layer),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, layer),
    };
    candidate.path.metrics.wirelength = 9.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.selected_candidates.push_back(candidate);

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);
    global.total_metrics.wirelength = 9.0;

    sapr::RoutingEvaluation evaluation{
        std::move(context),
        {candidate},
        std::move(global),
        std::nullopt,
        9.0,
        0,
        false,
    };
    return evaluation;
}

// 构造带 LCP 拓扑的最小 detailed routing request。
sapr::Circuit make_priority_test_circuit() {
    sapr::Circuit circuit;
    for (const auto& module : {std::string{"S"}, std::string{"S_MIRROR"}, std::string{"P"}}) {
        circuit.modules.emplace(module, sapr::Module{module, 12.0, 12.0, sapr::Rect{5.0, 5.0, 6.0, 6.0}, 0.0, 0.0, ""});
        circuit.module_order.push_back(module);
    }
    for (const auto& pin : {std::string{"A"}, std::string{"B"}, std::string{"C"}, std::string{"D"}}) {
        const double x = (pin == "A" || pin == "C") ? 0.0 : 9.0;
        for (const auto& module : {std::string{"S"}, std::string{"P"}}) {
            circuit.pins.emplace(module + "." + pin, sapr::Pin{module, pin, x, 1.0, "M1"});
            circuit.pin_order.push_back(module + "." + pin);
        }
    }
    circuit.constraints.symmetry_pairs.push_back({"sg", sapr::Axis::Vertical, "S", "S_MIRROR"});
    circuit.constraints.wire_widths["SYM_CRT"] = {"SYM_CRT", 0.05, 1.0};
    circuit.nets.emplace("SYM_CRT", sapr::Net{"SYM_CRT", sapr::Priority::Critical, {"S.A", "S.B"}});
    circuit.nets.emplace("SYM_NOR", sapr::Net{"SYM_NOR", sapr::Priority::Normal, {"S.C", "S.D"}});
    circuit.nets.emplace("PLAIN_CRT", sapr::Net{"PLAIN_CRT", sapr::Priority::Critical, {"P.A", "P.B"}});
    circuit.nets.emplace("PLAIN_NOR", sapr::Net{"PLAIN_NOR", sapr::Priority::Normal, {"P.C", "P.D"}});
    circuit.net_order = {"SYM_CRT", "SYM_NOR", "PLAIN_CRT", "PLAIN_NOR"};
    return circuit;
}

// 构造两条不同 net 共享同一几何通道的最小电路，用于验证短路冲突会被拒绝。
sapr::Circuit make_conflict_test_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("M", sapr::Module{"M", 12.0, 12.0, sapr::Rect{5.0, 5.0, 6.0, 6.0}, 0.0, 0.0, ""});
    circuit.module_order.push_back("M");
    const std::vector<std::string> names{"A", "B", "C", "D"};
    for (const auto& name : names) {
        const double x = (name == "A" || name == "C") ? 0.0 : 9.0;
        circuit.pins.emplace("M." + name, sapr::Pin{"M", name, x, 1.0, "M1"});
    }
    circuit.pin_order = {"M.A", "M.B", "M.C", "M.D"};
    circuit.nets.emplace("N1", sapr::Net{"N1", sapr::Priority::Critical, {"M.A", "M.B"}});
    circuit.nets.emplace("N2", sapr::Net{"N2", sapr::Priority::Normal, {"M.C", "M.D"}});
    circuit.net_order = {"N1", "N2"};
    return circuit;
}

// 构造一条人工水平候选，用于精确测试短路合法化。
sapr::routing::RouteCandidate make_horizontal_candidate(
    const sapr::routing::RoutingContext& context,
    const std::string& net,
    const std::string& from,
    const std::string& to,
    double y,
    int layer = 0) {
    sapr::routing::RouteCandidate candidate;
    candidate.net = net;
    candidate.from_terminal = from;
    candidate.to_terminal = to;
    candidate.segment_id = net + ":" + from + "->" + to;
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, layer),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, layer),
    };
    candidate.path.metrics.wirelength = 9.0;
    return candidate;
}

sapr::RoutingEvaluation make_priority_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    const auto make_candidate = [&](const std::string& net, const std::string& from, const std::string& to, double y) {
        sapr::routing::RouteCandidate candidate;
        candidate.net = net;
        candidate.from_terminal = from;
        candidate.to_terminal = to;
        candidate.wire_width = 1.0;
        candidate.path.success = true;
        candidate.path.points = {
            context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, 0),
            context.grid().snap_to_grid(sapr::routing::Point{9.0, y}, 0),
        };
        candidate.path.metrics.wirelength = 9.0;
        return candidate;
    };
    const auto symmetric_critical = make_candidate("SYM_CRT", "S.A", "S.B", 1.0);
    const auto symmetric_normal = make_candidate("SYM_NOR", "S.C", "S.D", 3.0);
    const auto plain_critical = make_candidate("PLAIN_CRT", "P.A", "P.B", 7.0);
    const auto plain_normal = make_candidate("PLAIN_NOR", "P.C", "P.D", 9.0);

    sapr::routing::GlobalRoutingResult global;
    for (const auto& candidate : {plain_normal, plain_critical, symmetric_normal, symmetric_critical}) {
        sapr::routing::NetRouteChoice choice;
        choice.net = candidate.net;
        choice.selected_candidates.push_back(candidate);
        global.net_routes.push_back(std::move(choice));
    }
    global.total_metrics.wirelength = 36.0;

    return sapr::RoutingEvaluation{
        std::move(context),
        {plain_normal, plain_critical, symmetric_normal, symmetric_critical},
        std::move(global),
        std::nullopt,
        36.0,
        0,
        false,
    };
}

/* 构造两个普通网络分别归属根节点和叶节点的最小电路，用于验证逆 packing 回溯顺序。 */
sapr::Circuit make_reverse_packing_order_test_circuit() {
    sapr::Circuit circuit;
    for (const auto& module : {std::string{"ROOT"}, std::string{"LEAF"}}) {
        circuit.modules.emplace(module, sapr::Module{module, 12.0, 12.0, sapr::Rect{5.0, 5.0, 6.0, 6.0}, 0.0, 0.0, ""});
        circuit.module_order.push_back(module);
        circuit.pins.emplace(module + ".A", sapr::Pin{module, "A", 0.0, 1.0, "M1"});
        circuit.pins.emplace(module + ".B", sapr::Pin{module, "B", 9.0, 1.0, "M1"});
        circuit.pin_order.push_back(module + ".A");
        circuit.pin_order.push_back(module + ".B");
    }
    circuit.nets.emplace("ROOT_NET", sapr::Net{"ROOT_NET", sapr::Priority::Normal, {"ROOT.A", "ROOT.B"}});
    circuit.nets.emplace("LEAF_NET", sapr::Net{"LEAF_NET", sapr::Priority::Normal, {"LEAF.A", "LEAF.B"}});
    circuit.net_order = {"ROOT_NET", "LEAF_NET"};
    return circuit;
}

/* 构造根节点网络先出现、叶节点网络后出现的候选，检验 detailed routing 是否反向提交。 */
sapr::RoutingEvaluation make_reverse_packing_order_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    const auto root = make_horizontal_candidate(context, "ROOT_NET", "ROOT.A", "ROOT.B", 1.0);
    const auto leaf = make_horizontal_candidate(context, "LEAF_NET", "LEAF.A", "LEAF.B", 4.0);
    sapr::routing::GlobalRoutingResult global;
    for (const auto& candidate : {root, leaf}) {
        sapr::routing::NetRouteChoice choice;
        choice.net = candidate.net;
        choice.selected_candidates.push_back(candidate);
        global.net_routes.push_back(std::move(choice));
    }
    global.total_metrics.wirelength = 18.0;
    sapr::routing::RoutingDpResult dp_result;
    dp_result.success = true;
    dp_result.traceback_candidates = {root, leaf};
    return sapr::RoutingEvaluation{
        std::move(context), {root, leaf}, std::move(global), std::move(dp_result), 18.0, 0, true};
}

/* 构造前序 packing trace：ROOT 先被打包，LEAF 后被打包，因此 detailed routing 必须先回溯 LEAF。 */
sapr::RoutingEvaluationRequest make_reverse_packing_order_request(
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"ROOT", "LEAF"};
    request.packing_trace.steps.push_back(
        {"ROOT_NODE", "ROOT", 0.0, 0.0, sapr::Rect{}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         std::optional<std::string>{"LEAF_NODE"}, std::nullopt, {"ROOT"}, {}, {}});
    request.packing_trace.steps.push_back(
        {"LEAF_NODE", "LEAF", 0.0, 0.0, sapr::Rect{}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         std::nullopt, std::nullopt, {"LEAF"}, {}, {}});
    return request;
}

sapr::RoutingEvaluationRequest make_lcp_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    bool with_location) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"M"};
    request.active_region_blockers.push_back(
        sapr::routing::transform_active_to_global(circuit.modules.at("M"), placements.at("M")));

    sapr::LinkingControlPoint lcp;
    lcp.id = "LCP1";
    lcp.space_node_id = "S1";
    if (with_location) lcp.location_candidates.push_back({4.0, 1.0, "LCP1:first", "strict", false, 0.0, "test"});
    lcp.segments.push_back({"N", "M.A", "LCP1", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Out, "N:left"});
    lcp.segments.push_back({"N", "LCP1", "M.B", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::In, "N:right"});

    sapr::SpaceNode space;
    space.id = "S1";
    space.owner = "M";
    space.kind = sapr::SpaceNodeKind::Right;
    space.linking_points.push_back(lcp);
    request.space_nodes.push_back(space);
    request.linking_points.push_back(lcp);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {lcp}, lcp.segments});
    return request;
}

// 构造左右叶子各自负责一条线段的 DP 树，用于验证 child state 合并时的物理兼容性检查。
sapr::RoutingEvaluationRequest make_dp_merge_compatibility_request(
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"M"};
    request.tree.root = "ROOT";
    request.tree.nodes = {
        {"ROOT", "M", std::optional<std::string>{"LEFT"}, std::optional<std::string>{"RIGHT"}},
        {"LEFT", "M", std::nullopt, std::nullopt},
        {"RIGHT", "M", std::nullopt, std::nullopt},
    };
    sapr::PackingContourStep left;
    left.tree_node = "LEFT";
    left.module = "M";
    left.subtree_modules = {"M"};
    left.local_wire_segments = {"LEFT_SEGMENT"};
    sapr::PackingContourStep right;
    right.tree_node = "RIGHT";
    right.module = "M";
    right.subtree_modules = {"M"};
    right.local_wire_segments = {"RIGHT_SEGMENT"};
    sapr::PackingContourStep root;
    root.tree_node = "ROOT";
    root.module = "M";
    root.left = "LEFT";
    root.right = "RIGHT";
    root.subtree_modules = {"M"};
    request.packing_trace.steps = {left, right, root};
    return request;
}

// 构造指定坐标的成功 DP 候选，保持测试只关注子树合并而非 A* 搜索。
sapr::routing::RouteCandidate make_dp_merge_candidate(
    const sapr::routing::RoutingContext& context,
    const std::string& net,
    const std::string& segment_id,
    const sapr::routing::Point& start,
    const sapr::routing::Point& end) {
    sapr::routing::RouteCandidate candidate;
    candidate.net = net;
    candidate.from_terminal = segment_id + ":from";
    candidate.to_terminal = segment_id + ":to";
    candidate.segment_id = segment_id;
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {context.grid().snap_to_grid(start, 0), context.grid().snap_to_grid(end, 0)};
    candidate.path.metrics.wirelength = 1.0;
    return candidate;
}

// 构造一组经过 LCP 的 selected candidates，用于验证 top-down traceback。
sapr::RoutingEvaluationRequest make_reverse_lcp_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements,
    bool with_location) {
    auto request = make_lcp_request(circuit, placements, with_location);
    request.linking_points.clear();
    request.space_nodes.clear();
    request.net_topologies.clear();

    sapr::LinkingControlPoint lcp;
    lcp.id = "LCP1";
    lcp.space_node_id = "S1";
    if (with_location) lcp.location_candidates.push_back({4.0, 1.0, "LCP1:first", "strict", false, 0.0, "test"});
    lcp.segments.push_back({"N", "M.B", "LCP1", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::In, "N:reverse_left"});
    lcp.segments.push_back({"N", "LCP1", "M.A", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Out, "N:reverse_right"});

    sapr::SpaceNode space;
    space.id = "S1";
    space.owner = "M";
    space.kind = sapr::SpaceNodeKind::Right;
    space.linking_points.push_back(lcp);
    request.space_nodes.push_back(space);
    request.linking_points.push_back(lcp);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {lcp}, lcp.segments});
    return request;
}

sapr::RoutingEvaluation make_lcp_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    sapr::routing::RouteCandidate first;
    first.net = "N";
    first.from_terminal = "M.A";
    first.to_terminal = "LCP1";
    first.segment_id = "N:left";
    first.lcp_id = "LCP1";
    first.lcp_candidate_id = "LCP1:first";
    first.wire_width = 2.0;
    first.path.success = true;
    first.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{4.0, 1.0}, 0),
    };
    first.path.metrics.wirelength = 4.0;

    sapr::routing::RouteCandidate second = first;
    second.from_terminal = "LCP1";
    second.to_terminal = "M.B";
    second.segment_id = "N:right";
    second.lcp_id = "LCP1";
    second.lcp_candidate_id = "LCP1:first";
    second.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{4.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.0}, 0),
    };
    second.path.metrics.wirelength = 5.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.selected_candidates = {first, second};
    choice.metrics.wirelength = 9.0;

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);
    global.total_metrics.wirelength = 9.0;

    sapr::RoutingEvaluation evaluation{
        std::move(context),
        {first, second},
        std::move(global),
        std::nullopt,
        9.0,
        0,
        false,
    };
    return evaluation;
}

// 构造 LCP-LCP 线段的 detailed routing 输入，用于验证两端 space 都收到 feedback。
sapr::RoutingEvaluationRequest make_lcp_pair_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request = make_lcp_request(circuit, placements, true);
    request.linking_points.clear();
    request.space_nodes.clear();
    request.net_topologies.clear();

    sapr::LinkingControlPoint left;
    left.id = "LCP_A";
    left.space_node_id = "S_A";
    left.location_candidates.push_back({2.0, 1.0, "LCP_A:first", "strict", false, 0.0, "test"});

    sapr::LinkingControlPoint right;
    right.id = "LCP_B";
    right.space_node_id = "S_B";
    right.location_candidates.push_back({7.0, 1.0, "LCP_B:first", "strict", false, 0.0, "test"});

    sapr::WireSegmentRef segment{"N", "LCP_A", "LCP_B", 2.0, 4.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:lcp_pair"};
    left.segments.push_back(segment);
    right.segments.push_back(segment);

    sapr::SpaceNode left_space;
    left_space.id = "S_A";
    left_space.owner = "M";
    left_space.kind = sapr::SpaceNodeKind::Right;
    left_space.linking_points.push_back(left);

    sapr::SpaceNode right_space = left_space;
    right_space.id = "S_B";
    right_space.linking_points.clear();
    right_space.linking_points.push_back(right);

    request.space_nodes.push_back(left_space);
    request.space_nodes.push_back(right_space);
    request.linking_points.push_back(left);
    request.linking_points.push_back(right);
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {left, right}, {segment}});
    return request;
}

// 构造 root LCP 连接多个 leaf 的候选覆盖用例，确保 pairwise top-K 不会漏掉任一端物理位置。
sapr::RoutingEvaluationRequest make_pairwise_coverage_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"M"};

    const auto make_lcp = [](const std::string& id, const std::string& space_id, double y) {
        sapr::LinkingControlPoint lcp;
        lcp.id = id;
        lcp.space_node_id = space_id;
        for (int index = 0; index < 40; ++index) {
            const double x = id == "ROOT" ? 50.0 + static_cast<double>(index) : static_cast<double>(index);
            lcp.location_candidates.push_back(
                {x, y, id + ":loc" + std::to_string(index), "strict", false, 0.0, "coverage"});
        }
        return lcp;
    };

    auto leaf0 = make_lcp("LEAF0", "S_LEAF0", 0.0);
    auto leaf1 = make_lcp("LEAF1", "S_LEAF1", 20.0);
    auto root = make_lcp("ROOT", "S_ROOT", 10.0);
    sapr::WireSegmentRef first{"N", "LEAF0", "ROOT", 1.0, 2.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:leaf0_root"};
    sapr::WireSegmentRef second{"N", "LEAF1", "ROOT", 1.0, 2.0, std::nullopt, sapr::CurrentDirection::Unknown, "N:leaf1_root"};
    leaf0.segments.push_back(first);
    leaf1.segments.push_back(second);
    root.segments.push_back(first);
    root.segments.push_back(second);

    const auto add_space = [&](const sapr::LinkingControlPoint& point) {
        sapr::SpaceNode space;
        space.id = point.space_node_id;
        space.owner = "M";
        space.kind = sapr::SpaceNodeKind::Right;
        space.linking_points.push_back(point);
        request.space_nodes.push_back(space);
    };
    add_space(leaf0);
    add_space(leaf1);
    add_space(root);

    request.linking_points = {leaf0, leaf1, root};
    request.net_topologies.push_back({"N", {"M.A", "M.B"}, {leaf0, leaf1, root}, {first, second}});
    (void)circuit;
    return request;
}

// 构造一条 LCP-LCP selected candidate。
sapr::RoutingEvaluation make_lcp_pair_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = "LCP_A";
    candidate.to_terminal = "LCP_B";
    candidate.segment_id = "N:lcp_pair";
    candidate.lcp_id = "LCP_A";
    candidate.lcp_candidate_id = "LCP_A:first";
    candidate.source_lcp_id = "LCP_A";
    candidate.source_lcp_candidate_id = "LCP_A:first";
    candidate.target_lcp_id = "LCP_B";
    candidate.target_lcp_candidate_id = "LCP_B:first";
    candidate.wire_width = 2.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{2.0, 1.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{7.0, 1.0}, 0),
    };
    candidate.path.metrics.wirelength = 5.0;

    sapr::routing::NetRouteChoice choice;
    choice.net = "N";
    choice.success = true;
    choice.selected_candidates.push_back(candidate);

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(choice);

    return {std::move(context), {candidate}, std::move(global), std::nullopt, 5.0, 0, false};
}

// 构造一条成功的人工候选路径，用于测试 LCP 多候选位置一致性选择。
sapr::routing::RouteCandidate make_manual_lcp_candidate(
    const sapr::routing::RoutingContext& context,
    const std::string& from,
    const std::string& to,
    const std::string& location_id,
    double wirelength,
    double y) {
    sapr::routing::RouteCandidate candidate;
    candidate.net = "N";
    candidate.from_terminal = from;
    candidate.to_terminal = to;
    candidate.segment_id = from + "->" + to;
    candidate.lcp_id = "LCP1";
    candidate.lcp_candidate_id = location_id;
    candidate.wire_width = 1.0;
    candidate.path.success = true;
    candidate.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, y}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{wirelength, y}, 0),
    };
    candidate.path.metrics.wirelength = wirelength;
    return candidate;
}

// 构造三端星形 LCP net：preferred location A 的一支会与障碍短路，location B 全通。
// 用于验证 detailed 必须整网换到同一 lcp_candidate_id，禁止各支路拆绑。
sapr::RoutingEvaluation make_star_lcp_consistency_evaluation(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::routing::RoutingContext context(circuit, placements);

    auto make_branch = [&](const std::string& from,
                           const std::string& to,
                           const std::string& segment_id,
                           const std::string& location_id,
                           double x1,
                           double y1,
                           double x2,
                           double y2) {
        sapr::routing::RouteCandidate candidate;
        candidate.net = "STAR";
        candidate.from_terminal = from;
        candidate.to_terminal = to;
        candidate.segment_id = segment_id;
        candidate.lcp_id = "LCP_STAR";
        candidate.lcp_candidate_id = location_id;
        candidate.wire_width = 1.0;
        candidate.path.success = true;
        candidate.path.points = {
            context.grid().snap_to_grid(sapr::routing::Point{x1, y1}, 0),
            context.grid().snap_to_grid(sapr::routing::Point{x2, y2}, 0),
        };
        candidate.path.metrics.wirelength = std::abs(x2 - x1) + std::abs(y2 - y1);
        return candidate;
    };

    // DP 选中 location A：其中 M.C->LCP 走 y=2，会与下方障碍 net 短路。
    auto a_ab = make_branch("M.A", "LCP_STAR", "STAR:A", "LCP_STAR:a", 0.0, 1.0, 4.0, 1.0);
    auto a_cb = make_branch("M.C", "LCP_STAR", "STAR:C", "LCP_STAR:a", 0.0, 2.0, 4.0, 2.0);
    auto a_db = make_branch("M.D", "LCP_STAR", "STAR:D", "LCP_STAR:a", 9.0, 1.0, 4.0, 1.0);

    // location B：三条支路都走 y=4，避开障碍。
    auto b_ab = make_branch("M.A", "LCP_STAR", "STAR:A", "LCP_STAR:b", 0.0, 4.0, 4.0, 4.0);
    auto b_cb = make_branch("M.C", "LCP_STAR", "STAR:C", "LCP_STAR:b", 0.0, 4.0, 4.0, 4.0);
    auto b_db = make_branch("M.D", "LCP_STAR", "STAR:D", "LCP_STAR:b", 9.0, 4.0, 4.0, 4.0);

    sapr::routing::NetRouteChoice star_choice;
    star_choice.net = "STAR";
    star_choice.success = true;
    star_choice.selected_candidates = {a_ab, a_cb, a_db};
    star_choice.metrics.wirelength = 17.0;

    // 障碍网占用 y=2 通道，迫使 location A 的 C 支路失败。
    sapr::routing::RouteCandidate blocker;
    blocker.net = "BLOCK";
    blocker.from_terminal = "M.A";
    blocker.to_terminal = "M.B";
    blocker.segment_id = "BLOCK:main";
    blocker.wire_width = 1.0;
    blocker.path.success = true;
    blocker.path.points = {
        context.grid().snap_to_grid(sapr::routing::Point{0.0, 2.0}, 0),
        context.grid().snap_to_grid(sapr::routing::Point{9.0, 2.0}, 0),
    };
    blocker.path.metrics.wirelength = 9.0;

    sapr::routing::NetRouteChoice block_choice;
    block_choice.net = "BLOCK";
    block_choice.success = true;
    block_choice.selected_candidates = {blocker};
    block_choice.metrics.wirelength = 9.0;

    sapr::routing::GlobalRoutingResult global;
    global.net_routes.push_back(block_choice);
    global.net_routes.push_back(star_choice);
    global.total_metrics.wirelength = 26.0;

    return sapr::RoutingEvaluation{
        std::move(context),
        {a_ab, a_cb, a_db, b_ab, b_cb, b_db, blocker},
        std::move(global),
        std::nullopt,
        26.0,
        0,
        false,
    };
}

sapr::Circuit make_star_lcp_test_circuit() {
    sapr::Circuit circuit = make_conflict_test_circuit();
    circuit.nets.emplace("STAR", sapr::Net{"STAR", sapr::Priority::Normal, {"M.A", "M.C", "M.D"}});
    circuit.nets.emplace("BLOCK", sapr::Net{"BLOCK", sapr::Priority::Critical, {"M.A", "M.B"}});
    circuit.net_order = {"BLOCK", "STAR"};
    return circuit;
}

sapr::RoutingEvaluationRequest make_star_lcp_request(
    const sapr::Circuit& circuit,
    const std::unordered_map<std::string, sapr::Placement>& placements) {
    sapr::RoutingEvaluationRequest request;
    request.placements = placements;
    request.placement_order = {"M"};
    request.active_region_blockers.push_back(
        sapr::routing::transform_active_to_global(circuit.modules.at("M"), placements.at("M")));

    sapr::LinkingControlPoint lcp;
    lcp.id = "LCP_STAR";
    lcp.space_node_id = "S_STAR";
    lcp.location_candidates.push_back({4.0, 1.0, "LCP_STAR:a", "strict", false, 0.0, "test"});
    lcp.location_candidates.push_back({4.0, 4.0, "LCP_STAR:b", "strict", false, 0.0, "test"});
    lcp.segments.push_back({"STAR", "M.A", "LCP_STAR", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "STAR:A"});
    lcp.segments.push_back({"STAR", "M.C", "LCP_STAR", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "STAR:C"});
    lcp.segments.push_back({"STAR", "M.D", "LCP_STAR", 1.0, 1.0, std::nullopt, sapr::CurrentDirection::Unknown, "STAR:D"});

    sapr::SpaceNode space;
    space.id = "S_STAR";
    space.owner = "M";
    space.kind = sapr::SpaceNodeKind::Right;
    space.linking_points.push_back(lcp);
    request.space_nodes.push_back(space);
    request.linking_points.push_back(lcp);
    request.net_topologies.push_back({"STAR", {"M.A", "M.C", "M.D"}, {lcp}, lcp.segments});
    return request;
}

}  // namespace

// 运行 routing evaluator 的集成测试。
// 验证输出线段归一化只合并同网同层同宽的共线区间。
void test_collinear_same_net_route_merge() {
    const auto merged_duplicate = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 1.0, 5.0, 1.0, 1.0},
        {"VDD", "M1", 0.0, 1.0, 5.0, 1.0, 1.0},
        {"VDD", "M1", 5.0, 1.0, 0.0, 1.0, 1.0},
    });
    require(merged_duplicate.size() == 1, "duplicate and reverse duplicate horizontal routes should merge");
    require(approx(merged_duplicate.front().x1, 0.0) && approx(merged_duplicate.front().x2, 5.0),
            "duplicate merge should keep the full horizontal interval");

    const auto merged_overlap = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 2.0, 5.0, 2.0, 1.0},
        {"VDD", "M1", 3.0, 2.0, 8.0, 2.0, 1.0},
    });
    require(merged_overlap.size() == 1, "partially overlapping same-net routes should merge");
    require(approx(merged_overlap.front().x1, 0.0) && approx(merged_overlap.front().x2, 8.0),
            "overlap merge should span the union interval");

    const auto merged_subset = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 3.0, 10.0, 3.0, 1.0},
        {"VDD", "M1", 2.0, 3.0, 4.0, 3.0, 1.0},
    });
    require(merged_subset.size() == 1, "subset same-net route should be absorbed");
    require(approx(merged_subset.front().x1, 0.0) && approx(merged_subset.front().x2, 10.0),
            "subset merge should keep the parent interval");

    const auto merged_touching = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 4.0, 5.0, 4.0, 1.0},
        {"VDD", "M1", 5.0, 4.0, 9.0, 4.0, 1.0},
    });
    require(merged_touching.size() == 1, "touching same-net collinear routes should merge");
    require(approx(merged_touching.front().x1, 0.0) && approx(merged_touching.front().x2, 9.0),
            "touching merge should span both intervals");

    const auto different_net = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 5.0, 5.0, 5.0, 1.0},
        {"GND", "M1", 3.0, 5.0, 8.0, 5.0, 1.0},
    });
    require(different_net.size() == 2, "different nets should not merge");

    const auto different_layer = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 0.0, 6.0, 5.0, 6.0, 1.0},
        {"VDD", "M2", 3.0, 6.0, 8.0, 6.0, 1.0},
    });
    require(different_layer.size() == 2, "different layers should not merge");

    const auto vertical = sapr::routing::merge_collinear_same_net_routes({
        {"VDD", "M1", 7.0, 0.0, 7.0, 5.0, 1.0},
        {"VDD", "M1", 7.0, 3.0, 7.0, 8.0, 1.0},
    });
    require(vertical.size() == 1, "vertical overlapping same-net routes should merge");
    require(approx(vertical.front().y1, 0.0) && approx(vertical.front().y2, 8.0),
            "vertical merge should span the union interval");
}

// 运行 routing evaluator 的集成测试。
void run_routing_evaluator_tests() {
    test_collinear_same_net_route_merge();
    const auto circuit = sapr::load_circuit(source_input_dir());
    sapr::SolverConfig config;
    config.sa_iterations = 0;
    const auto solution = sapr::solve_baseline(circuit, config);
    require(solution.metrics.has_value(), "solution should expose optimizer metrics");
    const auto& metrics = *solution.metrics;
    const double expected_phi =
        config.area_weight * metrics.normalized_area +
        config.wirelength_weight * metrics.normalized_wirelength +
        config.bend_weight * metrics.normalized_bend +
        config.via_weight * metrics.normalized_via +
        metrics.row_width_penalty +
        metrics.penalty;
    require(metrics.phi_cost > 0.0, "optimizer should expose positive phi cost");
    require(approx(metrics.phi_cost, expected_phi), "phi cost should match the paper total-cost formula");
    const double expected_dedup_penalty =
        metrics.flow_penalty +
        metrics.current_density_penalty +
        metrics.coupling_penalty +
        metrics.design_rule_penalty +
        metrics.routing_failure_penalty;
    require(approx(metrics.penalty, expected_dedup_penalty), "SA penalty should use deduplicated routing penalty terms");
    // global_* 是 global 阶段快照；routing_cost 必须与之对齐，不得误用最终 penalty/wirelength。
    require(solution.routing_cost.has_value(), "solution should expose global routing_cost");
    const double expected_global_routing_cost =
        metrics.global_wirelength + 3.0 * static_cast<double>(metrics.global_bend_count) + metrics.global_penalty;
    require(
        approx(*solution.routing_cost, expected_global_routing_cost),
        "routing_cost should equal global WL + 3*bends + global_penalty");
    require(
        approx(metrics.penalty, metrics.flow_penalty + metrics.current_density_penalty + metrics.coupling_penalty +
                                    metrics.design_rule_penalty + metrics.routing_failure_penalty),
        "final penalty should stay the post-detailed aggregate");
    require(metrics.dp_traceback_segments > 0, "placement-aware routing should produce bottom-up DP traceback");
    require(metrics.dp_nodes > 0, "bottom-up DP should visit B*-tree nodes");
    require(metrics.dp_states > 0, "bottom-up DP should keep candidate states");
    require(metrics.dp_states >= metrics.dp_nodes, "bottom-up DP should keep at least one state per visited node");
    require(metrics.packing_trace_steps > 0, "packing should expose contour trace steps");
    require(metrics.packing_time_dp_used, "bottom-up DP should consume packing-time local wire segments");
    require(metrics.packing_time_dp_segments > 0, "packing-time DP should expose local wire segment count");
    require(metrics.space_feedback_nodes >= 0, "optimizer should expose routing space feedback count");
    require(metrics.routing_feedback_iterations >= 1, "candidate evaluation should run at least one feedback iteration");
    require(
        metrics.routing_feedback_iterations <= config.routing_feedback_iterations,
        "candidate evaluation should honor max routing feedback iterations");
    if (metrics.routing_feedback_iterations < config.routing_feedback_iterations) {
        require(metrics.routing_feedback_converged, "early feedback-loop stop should mean convergence");
    }
    auto single_loop_config = config;
    single_loop_config.sa_iterations = 0;
    single_loop_config.routing_feedback_iterations = 1;
    const auto single_loop_solution = sapr::solve_placement_aware(circuit, single_loop_config);
    require(single_loop_solution.metrics.has_value(), "single-loop solution should expose metrics");
    require(
        single_loop_solution.metrics->routing_feedback_iterations == 1,
        "routing_feedback_iterations=1 should keep current single-pass behavior");
    auto boundary_tree = sapr::make_enhanced_tree(circuit);
    sapr::initialize_lcp_topology(circuit, boundary_tree, config);
    const auto request_for_dp = sapr::pack_enhanced_tree(circuit, boundary_tree, config);
    require(!request_for_dp.packing_trace.steps.empty(), "pack_enhanced_tree should record contour trace");
    require(boundary_tree.root.has_value(), "boundary margin test requires a root node");
    const auto root_step_for_auto = std::find_if(
        request_for_dp.packing_trace.steps.begin(),
        request_for_dp.packing_trace.steps.end(),
        [&](const auto& step) { return step.tree_node == *boundary_tree.root; });
    require(root_step_for_auto != request_for_dp.packing_trace.steps.end(), "packing trace should include root hierarchy step");
    require(root_step_for_auto->x > 0.0 && root_step_for_auto->y > 0.0, "auto boundary margin should move the root inside the chip");

    auto zero_margin_config = config;
    zero_margin_config.boundary_margin = 0.0;
    const auto zero_margin_request = sapr::pack_enhanced_tree(circuit, boundary_tree, zero_margin_config);
    auto explicit_margin_config = config;
    explicit_margin_config.boundary_margin = 1.5;
    const auto explicit_margin_request = sapr::pack_enhanced_tree(circuit, boundary_tree, explicit_margin_config);
    const auto zero_root_step = std::find_if(
        zero_margin_request.packing_trace.steps.begin(),
        zero_margin_request.packing_trace.steps.end(),
        [&](const auto& step) { return step.tree_node == *boundary_tree.root; });
    const auto explicit_root_step = std::find_if(
        explicit_margin_request.packing_trace.steps.begin(),
        explicit_margin_request.packing_trace.steps.end(),
        [&](const auto& step) { return step.tree_node == *boundary_tree.root; });
    require(zero_root_step != zero_margin_request.packing_trace.steps.end(), "zero-margin trace should include root step");
    require(explicit_root_step != explicit_margin_request.packing_trace.steps.end(), "explicit-margin trace should include root step");
    require(approx(explicit_root_step->x - zero_root_step->x, 1.5), "explicit boundary margin should shift root x");
    require(approx(explicit_root_step->y - zero_root_step->y, 1.5), "explicit boundary margin should shift root y");
    for (const auto& [id, zero_placement] : zero_margin_request.placements) {
        const auto& shifted = explicit_margin_request.placements.at(id);
        require(approx(shifted.x - zero_placement.x, 1.5), "boundary margin should preserve relative x placement");
        require(approx(shifted.y - zero_placement.y, 1.5), "boundary margin should preserve relative y placement");
    }
    require(
        approx(explicit_margin_request.packing_trace.steps.front().occupied_bbox.x1, 1.5) &&
            approx(explicit_margin_request.packing_trace.steps.front().occupied_bbox.y1, 1.5),
        "packing trace root bbox should start at the explicit boundary margin");
    require(
        request_for_dp.packing_trace.steps.size() >= request_for_dp.tree.nodes.size(),
        "packing trace should cover representative tree nodes and may include ASF internal steps");
    for (const auto& step : request_for_dp.packing_trace.steps) {
        require(step.occupied_bbox.x1 <= step.occupied_bbox.x2, "packing trace bbox x range should be valid");
        require(step.occupied_bbox.y1 <= step.occupied_bbox.y2, "packing trace bbox y range should be valid");
        require(step.right_space >= 0.0, "packing trace should record non-negative right routing space");
        require(step.top_space >= 0.0, "packing trace should record non-negative top routing space");
        require(step.coupling_extra_space >= 0.0, "packing trace should record non-negative coupling space");
    }
    const auto local_segment_count = std::accumulate(
        request_for_dp.packing_trace.steps.begin(),
        request_for_dp.packing_trace.steps.end(),
        std::size_t{0},
        [](std::size_t total, const auto& step) { return total + step.local_wire_segments.size(); });
    require(local_segment_count > 0, "packing trace should expose packing-time local routing segments");
    std::unordered_map<std::string, const sapr::WireSegmentRef*> segment_by_key;
    for (const auto& topology : request_for_dp.net_topologies) {
        for (const auto& segment : topology.segments) {
            const std::string key = segment.id.empty() ? segment.net + "|" + segment.from + "|" + segment.to : segment.id;
            segment_by_key[key] = &segment;
        }
    }
    std::unordered_map<std::string, std::string> lcp_owner_by_id;
    std::unordered_map<std::string, std::string> owner_by_space;
    for (const auto& space : request_for_dp.space_nodes) {
        owner_by_space[space.id] = space.owner;
        for (const auto& point : space.linking_points) lcp_owner_by_id[point.id] = space.owner;
    }
    for (const auto& point : request_for_dp.linking_points) {
        const auto found = owner_by_space.find(point.space_node_id);
        if (found != owner_by_space.end()) lcp_owner_by_id[point.id] = found->second;
    }
    for (const auto& topology : request_for_dp.net_topologies) {
        for (const auto& point : topology.linking_points) {
            std::unordered_set<std::string> segment_ids;
            int lcp_incident_segments = 0;
            for (const auto& segment : point.segments) {
                if (segment.from == point.id || segment.to == point.id) ++lcp_incident_segments;
                require(segment_ids.insert(segment.id).second, "LCP topology should not duplicate root-to-LCP segments");
            }
            require(lcp_incident_segments >= 2, "each logical LCP should connect at least two same-net segments");
        }
    }
    const auto terminal_owner = [&](const std::string& terminal) {
        const auto dot = terminal.find('.');
        if (dot != std::string::npos) return terminal.substr(0, dot);
        const auto found = lcp_owner_by_id.find(terminal);
        return found == lcp_owner_by_id.end() ? std::string{} : found->second;
    };
    const auto step_by_node = [&](const std::optional<std::string>& id) -> const sapr::PackingContourStep* {
        if (!id.has_value()) return nullptr;
        const auto found = std::find_if(
            request_for_dp.packing_trace.steps.begin(),
            request_for_dp.packing_trace.steps.end(),
            [&](const auto& step) { return step.tree_node == *id; });
        return found == request_for_dp.packing_trace.steps.end() ? nullptr : &*found;
    };
    const auto inside_modules = [](const std::string& from, const std::string& to, const auto& modules) {
        return modules.contains(from) && modules.contains(to);
    };
    for (const auto& step : request_for_dp.packing_trace.steps) {
        const std::unordered_set<std::string> current_modules(step.subtree_modules.begin(), step.subtree_modules.end());
        const auto* left_step = step_by_node(step.left);
        const auto* right_step = step_by_node(step.right);
        const std::unordered_set<std::string> left_modules =
            left_step == nullptr ? std::unordered_set<std::string>{}
                                 : std::unordered_set<std::string>(left_step->subtree_modules.begin(), left_step->subtree_modules.end());
        const std::unordered_set<std::string> right_modules =
            right_step == nullptr ? std::unordered_set<std::string>{}
                                  : std::unordered_set<std::string>(right_step->subtree_modules.begin(), right_step->subtree_modules.end());
        for (const auto& key : step.local_wire_segments) {
            require(segment_by_key.contains(key), "packing-time local segment should refer to a known topology segment");
            const auto* segment = segment_by_key.at(key);
            const auto from_owner = terminal_owner(segment->from);
            const auto to_owner = terminal_owner(segment->to);
            require(inside_modules(from_owner, to_owner, current_modules), "local segment endpoints should be inside current subtree");
            require(!inside_modules(from_owner, to_owner, left_modules), "local segment should not be fully inside left child");
            require(!inside_modules(from_owner, to_owner, right_modules), "local segment should not be fully inside right child");
        }
    }
    const auto root_step = std::find_if(
        request_for_dp.packing_trace.steps.begin(),
        request_for_dp.packing_trace.steps.end(),
        [&](const auto& step) { return request_for_dp.tree.root.has_value() && step.tree_node == *request_for_dp.tree.root; });
    require(root_step != request_for_dp.packing_trace.steps.end(), "packing trace should include root step");
    std::unordered_set<std::string> root_modules(root_step->subtree_modules.begin(), root_step->subtree_modules.end());
    for (const auto& node : request_for_dp.tree.nodes) {
        std::size_t start = 0;
        while (start <= node.module.size()) {
            const std::size_t end = node.module.find('|', start);
            const std::string token = node.module.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!token.empty() && token != node.id) {
                require(root_modules.contains(token), "root packing trace step should cover all placed representative modules");
            }
            if (end == std::string::npos) break;
            start = end + 1;
        }
    }
    const auto dp_evaluation = sapr::evaluate_routing(circuit, request_for_dp);
    require(dp_evaluation.bottom_up_dp.has_value(), "routing evaluator should expose bottom-up DP result");
    require(!dp_evaluation.bottom_up_dp->traceback_candidates.empty(), "bottom-up DP should produce traceback candidates");
    require(dp_evaluation.bottom_up_dp->packing_time_dp_used, "bottom-up DP should prefer packing-time local segments");
    require(dp_evaluation.bottom_up_dp->packing_time_dp_segments > 0, "bottom-up DP should count packing-time local segments");
    require(
        !dp_evaluation.bottom_up_dp->best_state.covered_wire_segments.empty(),
        "bottom-up DP state should record covered wire segments");
    require(
        !dp_evaluation.bottom_up_dp->best_state.selected_transitions.empty(),
        "bottom-up DP state should record selected transitions");
    require(
        dp_evaluation.bottom_up_dp->best_state.packing_step_index >= 0,
        "bottom-up DP state should record packing step index when contour trace exists");
    const bool has_space_trace = std::any_of(
        dp_evaluation.bottom_up_dp->best_state.selected_transitions.begin(),
        dp_evaluation.bottom_up_dp->best_state.selected_transitions.end(),
        [](const auto& transition) { return transition.find("@space=") != std::string::npos; });
    require(
        has_space_trace || !request_for_dp.linking_points.empty(),
        "LCP DP transitions should record source space node id or preserve request-level LCP ownership");
    for (const auto& node_result : dp_evaluation.bottom_up_dp->node_results) {
        require(node_result.states.size() <= 16, "bottom-up DP should honor max_states_per_node");
    }
    const auto mixed_circuit = sapr::load_circuit(mixed_lcp_direct_case_input_dir());
    auto mixed_tree = sapr::make_chain_tree(mixed_circuit);
    sapr::initialize_lcp_topology(mixed_circuit, mixed_tree, config);
    const auto mixed_request = sapr::pack_enhanced_tree(mixed_circuit, mixed_tree, config);
    const auto mixed_evaluation = sapr::evaluate_routing(mixed_circuit, mixed_request);
    require(mixed_evaluation.bottom_up_dp.has_value(), "mixed LCP/direct case should expose bottom-up DP result");
    require(
        std::none_of(
            mixed_evaluation.bottom_up_dp->candidate_events.begin(),
            mixed_evaluation.bottom_up_dp->candidate_events.end(),
            [](const auto& event) { return event.reason == "short_conflict_penalty"; }),
        "bottom-up DP should reject occupied-route shorts instead of retaining a penalty candidate");
    require(mixed_evaluation.failed_nets == 0, "non-strict DP fallback should not drop direct-only nets from global routing");
    std::unordered_set<std::string> mixed_routed_nets;
    for (const auto& segment : sapr::selected_candidates_to_segments(mixed_evaluation)) {
        mixed_routed_nets.insert(segment.net);
    }
    for (const auto& net : mixed_circuit.net_order) {
        require(mixed_routed_nets.contains(net), "mixed LCP/direct case should keep every net in selected routes");
    }
    auto no_local_segment_request = request_for_dp;
    for (auto& step : no_local_segment_request.packing_trace.steps) {
        step.local_wire_segments.clear();
        step.cross_child_wire_segments.clear();
    }
    const auto no_local_segment_evaluation = sapr::evaluate_routing(circuit, no_local_segment_request);
    require(no_local_segment_evaluation.bottom_up_dp.has_value(), "fallback DP should expose a result without local segments");
    require(
        !no_local_segment_evaluation.bottom_up_dp->traceback_candidates.empty(),
        "fallback DP should produce traceback candidates without local segments");
    require(
        !no_local_segment_evaluation.bottom_up_dp->packing_time_dp_used,
        "cleared local segments should force fallback DP transition inference");
    auto feedback_tree = sapr::make_enhanced_tree(circuit);
    sapr::initialize_lcp_topology(circuit, feedback_tree, config);
    const auto first_feedback_request = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    const auto first_feedback = sapr::evaluate_with_routing_adapter(circuit, first_feedback_request);
    require(
        approx(
            first_feedback.routing_cost,
            first_feedback.metrics.global_wirelength +
                3.0 * static_cast<double>(first_feedback.metrics.global_bend_count) +
                first_feedback.metrics.global_penalty),
        "adapter routing_cost should stay aligned with global_* snapshot");
    if (!first_feedback.routes.empty()) {
        sapr::Solution feedback_solution;
        feedback_solution.placements = first_feedback_request.placements;
        feedback_solution.placement_order = first_feedback_request.placement_order;
        feedback_solution.routes = first_feedback.routes;
        const auto route_metrics = sapr::measure(circuit, feedback_solution);
        require(
            approx(first_feedback.metrics.wirelength, route_metrics.wirelength),
            "routing feedback should use final detailed route wirelength when routes exist");
        require(
            first_feedback.metrics.bend_count == route_metrics.bend_count,
            "routing feedback should use final detailed route bends when routes exist");
        require(
            first_feedback.metrics.via_count == route_metrics.via_count,
            "routing feedback should use final detailed route vias when routes exist");
        // detailed 成功后最终 penalty 可为 0，但 global_penalty 仍应保留 global 阶段冲突代价。
        require(
            first_feedback.metrics.global_penalty + 1e-9 >= first_feedback.metrics.penalty,
            "global_penalty should not be overwritten by final detailed penalty");
    }
    sapr::apply_routing_feedback(feedback_tree, first_feedback);
    const auto second_feedback_request = sapr::pack_enhanced_tree(circuit, feedback_tree, config);
    require(
        second_feedback_request.packing_trace.steps.size() == first_feedback_request.packing_trace.steps.size(),
        "routing feedback should preserve representative packing trace size");
    const auto trace_space_sum = [](const sapr::RoutingEvaluationRequest& request) {
        double total = 0.0;
        for (const auto& step : request.packing_trace.steps) {
            total += step.right_space + step.top_space + step.coupling_extra_space;
        }
        return total;
    };
    require(
        trace_space_sum(second_feedback_request) + 1e-9 >= trace_space_sum(first_feedback_request),
        "routing feedback should not reduce contour routing space demand");
    if (first_feedback.metrics.space_feedback_nodes > 0) {
        require(
            !first_feedback.required_space_by_node.empty() || !first_feedback.coupling_space_by_node.empty(),
            "space feedback count should correspond to emitted feedback maps");
    }
    auto no_trace_request = request_for_dp;
    no_trace_request.packing_trace.steps.clear();
    const auto no_trace_evaluation = sapr::evaluate_routing(circuit, no_trace_request);
    require(no_trace_evaluation.bottom_up_dp.has_value(), "fallback DP should still expose a result");
    require(!no_trace_evaluation.bottom_up_dp->traceback_candidates.empty(), "fallback DP should still produce traceback");
    const auto evaluation = sapr::evaluate_routing(circuit, solution.placements);
    const auto selected_segments = sapr::selected_candidates_to_segments(evaluation);
    sapr::RoutingEvaluationRequest request;
    request.placements = solution.placements;
    request.placement_order = solution.placement_order;
    const auto detailed = sapr::run_detailed_routing(circuit, request, evaluation);

    require(!evaluation.candidates.empty(), "routing evaluator should emit A* route candidates");
    require(evaluation.failed_nets >= 0, "routing evaluator should report routed and failed nets");
    if (evaluation.failed_nets > 0) {
        require(
            evaluation.global_routing.routing_failure_penalty >= 100000.0 * static_cast<double>(evaluation.failed_nets),
            "failed nets should contribute high routing penalty");
    }
    require(evaluation.routing_cost > 0.0, "routing evaluator should produce a positive global routing cost");
    require(!selected_segments.empty(), "routing evaluator should convert selected A* candidates to route segments");
    require(
        !detailed.routes.empty() || detailed.traceback_failures > 0 || detailed.design_rule_violations > 0,
        "detailed routing should either emit legal route segments or report why traceback was discarded");
    require(detailed.coupling_penalty >= 0.0, "detailed routing should report coupling penalty");
    require(detailed.design_rule_penalty >= 0.0, "detailed routing should report DRC penalty");
    require(detailed.design_rule_violations >= 0, "detailed routing should report DRC violations");
    require(detailed.current_density_violations == 0, "detailed routing should keep route widths inside constraints");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.flow_penalty, "flow penalty should be part of total penalty");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.current_density_penalty, "current-density penalty should be part of total penalty");
    require(evaluation.global_routing.total_penalty >= evaluation.global_routing.coupling_penalty, "coupling penalty should be part of total penalty");
    require(evaluation.global_routing.routing_failure_penalty >=
                100000.0 * static_cast<double>(evaluation.failed_nets),
            "routing failures should contribute high penalty");
    require(selected_segments.size() > circuit.net_order.size(), "A*/DP route output should not collapse to one segment per net");
    for (const auto& segment : selected_segments) {
        const auto width = circuit.constraints.wire_widths.find(segment.net);
        if (width != circuit.constraints.wire_widths.end()) {
            require(segment.width >= width->second.min_width, "selected segment width should satisfy min width");
            require(segment.width <= width->second.max_width, "selected segment width should satisfy max width");
        }
    }
    for (const auto& segment : detailed.routes) {
        const auto width = circuit.constraints.wire_widths.find(segment.net);
        if (width != circuit.constraints.wire_widths.end()) {
            require(segment.width >= width->second.min_width, "detailed segment width should satisfy min width");
            require(segment.width <= width->second.max_width, "detailed segment width should satisfy max width");
        }
        require(segment.x1 != segment.x2 || segment.y1 != segment.y2, "detailed routing should not emit zero-length segments");
    }

    const double expected_cost =
        evaluation.global_routing.total_metrics.wirelength +
        3.0 * static_cast<double>(evaluation.global_routing.total_metrics.bend_count) +
        evaluation.global_routing.total_penalty;
    require(approx(evaluation.routing_cost, expected_cost), "global routing cost must not include via count");

    sapr::routing::Grid bounded_grid(sapr::routing::GridConfig{1.0, 5.0, 2}, 0.0, 0.0, 10.0, 10.0);
    require(approx(bounded_grid.min_x(), 0.0), "routing grid should not expand the lower x boundary outside layout");
    require(approx(bounded_grid.min_y(), 0.0), "routing grid should not expand the lower y boundary outside layout");

    const auto conflict_circuit = make_conflict_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> conflict_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    // via / detailed reroute 单测需要至少 M1+M2；生产默认仍是 --routing-layers 1。
    const sapr::routing::GridConfig multi_layer_config = sapr::routing::make_grid_config_for_routing_layers(2);
    sapr::routing::RoutingContext conflict_context(conflict_circuit, conflict_placements, multi_layer_config);
    const auto shared_path = std::vector<sapr::routing::GridPoint>{
        conflict_context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.0}, 0),
        conflict_context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.0}, 0),
    };
    sapr::routing::RouteCandidate first_conflict_candidate;
    first_conflict_candidate.net = "N1";
    first_conflict_candidate.from_terminal = "M.A";
    first_conflict_candidate.to_terminal = "M.B";
    first_conflict_candidate.path.success = true;
    first_conflict_candidate.path.points = shared_path;
    first_conflict_candidate.path.metrics.wirelength = 9.0;
    sapr::routing::RouteCandidate second_conflict_candidate = first_conflict_candidate;
    second_conflict_candidate.net = "N2";
    second_conflict_candidate.from_terminal = "M.C";
    second_conflict_candidate.to_terminal = "M.D";
    const auto conflict_global = sapr::routing::run_global_routing(
        conflict_circuit,
        conflict_context,
        {first_conflict_candidate, second_conflict_candidate});
    require(conflict_global.failed_nets == 0, "global routing keeps a high-penalty fallback when every candidate conflicts");
    require(
        conflict_global.coupling_penalty >= 100000.0,
        "different nets sharing routing grid points should receive a high conflict penalty");

    sapr::RouteSegment first_metal{"N1", "M1", 0.0, 1.0, 9.0, 1.0, 1.0};
    sapr::RouteSegment second_metal{"N2", "M1", 0.0, 2.0, 9.0, 2.0, 1.0};
    require(
        sapr::routing::same_layer_short(first_metal, second_metal),
        "same-layer metal rectangles should detect shorts even when centerlines use different tracks");

    const auto preferred_first = make_horizontal_candidate(conflict_context, "N1", "M.A", "M.B", 1.0);
    const auto short_second = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 2.0);
    auto legal_second = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 4.0);
    legal_second.lcp_candidate_id = "legal-track";
    legal_second.path.metrics.via_count = 3;
    const auto short_aware_global = sapr::routing::run_global_routing(
        conflict_circuit,
        conflict_context,
        {preferred_first, short_second, legal_second});
    require(short_aware_global.failed_nets == 0, "short-aware global routing should keep legal fallback candidates");
    require(
        short_aware_global.net_routes.size() == 2 &&
            short_aware_global.net_routes[1].selected_candidates.front().path.points.front().iy ==
                legal_second.path.points.front().iy,
        "global routing should prefer a non-short candidate over a shorter same-layer short");

    sapr::routing::GlobalRoutingResult detailed_global;
    sapr::routing::NetRouteChoice first_choice;
    first_choice.net = "N1";
    first_choice.selected_candidates.push_back(preferred_first);
    detailed_global.net_routes.push_back(first_choice);
    sapr::routing::NetRouteChoice second_choice;
    second_choice.net = "N2";
    second_choice.selected_candidates.push_back(short_second);
    detailed_global.net_routes.push_back(second_choice);
    sapr::RoutingEvaluation short_detail_eval{
        sapr::routing::RoutingContext(conflict_circuit, conflict_placements, multi_layer_config),
        {preferred_first, short_second, legal_second},
        std::move(detailed_global),
        std::nullopt,
        18.0,
        0,
        false,
    };
    sapr::RoutingEvaluationRequest short_detail_request;
    short_detail_request.placements = conflict_placements;
    short_detail_request.placement_order = {"M"};
    const auto legalized_detail = sapr::run_detailed_routing(conflict_circuit, short_detail_request, short_detail_eval);
    require(legalized_detail.design_rule_violations == 0, "detailed routing should legalize shorts with an alternative candidate");
    require(!legalized_detail.routes.empty(), "detailed routing should keep legalized routes instead of clearing all output");
    require(
        legalized_detail.raw_routes.size() == legalized_detail.routes.size() &&
            legalized_detail.raw_routes.front().net == legalized_detail.routes.front().net &&
            approx(legalized_detail.raw_routes.front().x1, legalized_detail.routes.front().x1) &&
            approx(legalized_detail.raw_routes.front().y1, legalized_detail.routes.front().y1) &&
            approx(legalized_detail.raw_routes.front().x2, legalized_detail.routes.front().x2) &&
            approx(legalized_detail.raw_routes.front().y2, legalized_detail.routes.front().y2),
        "diagnostic raw routes should preserve every final detailed route when final DRC passes");
    require(
        approx(legalized_detail.detailed_cost, 18.0),
        "detailed routing cost should be recomputed from final routes instead of stale candidate via metrics");

    auto long_no_via = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 3.0);
    const auto compare_start = conflict_context.grid().snap_to_grid(sapr::routing::Point{0.0, 3.0}, 0);
    const auto compare_goal = conflict_context.grid().snap_to_grid(sapr::routing::Point{9.0, 3.0}, 0);
    const sapr::routing::GridPoint long_mid_left{compare_start.ix, compare_start.iy + 8, 0};
    const sapr::routing::GridPoint long_mid_right{compare_goal.ix, compare_start.iy + 8, 0};
    long_no_via.path.points = {
        compare_start,
        long_mid_left,
        long_mid_right,
        compare_goal,
    };
    long_no_via.path.metrics.wirelength = 43.0;
    long_no_via.path.metrics.bend_count = 2;
    auto short_with_vias = make_horizontal_candidate(conflict_context, "N2", "M.C", "M.D", 3.0);
    short_with_vias.lcp_candidate_id = "via-track";
    const sapr::routing::GridPoint via_left_m1{compare_start.ix, compare_start.iy + 2, 0};
    const sapr::routing::GridPoint via_left_m2{compare_start.ix, compare_start.iy + 2, 1};
    const sapr::routing::GridPoint via_right_m2{compare_goal.ix, compare_start.iy + 2, 1};
    const sapr::routing::GridPoint via_right_m1{compare_goal.ix, compare_start.iy + 2, 0};
    short_with_vias.path.points = {
        compare_start,
        via_left_m1,
        via_left_m2,
        via_right_m2,
        via_right_m1,
        compare_goal,
    };
    short_with_vias.path.metrics.wirelength = 15.0;
    short_with_vias.path.metrics.via_count = 2;
    sapr::routing::GlobalRoutingResult cost_compare_global;
    sapr::routing::NetRouteChoice cost_compare_choice;
    cost_compare_choice.net = "N2";
    cost_compare_choice.selected_candidates.push_back(long_no_via);
    cost_compare_global.net_routes.push_back(cost_compare_choice);
    sapr::RoutingEvaluation cost_compare_eval{
        sapr::routing::RoutingContext(conflict_circuit, conflict_placements, multi_layer_config),
        {long_no_via, short_with_vias},
        std::move(cost_compare_global),
        std::nullopt,
        43.0,
        0,
        false,
    };
    const auto cost_compare_detail = sapr::run_detailed_routing(conflict_circuit, short_detail_request, cost_compare_eval);
    require(
        cost_compare_detail.detailed_cost < 49.0,
        "detailed legalization should compare LCP alternatives and via-bearing candidates by detailed cost");
    require(
        std::any_of(cost_compare_detail.routes.begin(), cost_compare_detail.routes.end(), [](const auto& route) {
            return route.layer == "M2";
        }),
        "lower-cost via-bearing candidate should be selected over a longer no-via candidate");
    require(!has_cross_net_same_layer_short(cost_compare_detail.routes), "cost-selected detailed route should not short");
    require(net_routes_are_contiguous(cost_compare_detail.routes, "N2"), "cost-selected via route should stay connected");
    require(net_has_implicit_via(cost_compare_detail.routes, "N2"), "cost-selected via route should expose implicit vias");

    sapr::routing::GlobalRoutingResult layer_global;
    layer_global.net_routes.push_back(first_choice);
    layer_global.net_routes.push_back(second_choice);
    sapr::RoutingEvaluation layer_detail_eval{
        sapr::routing::RoutingContext(conflict_circuit, conflict_placements, multi_layer_config),
        {preferred_first, short_second},
        std::move(layer_global),
        std::nullopt,
        18.0,
        0,
        false,
    };
    const auto layer_detail = sapr::run_detailed_routing(conflict_circuit, short_detail_request, layer_detail_eval);
    require(layer_detail.traceback_failures == 0, "detailed routing should use real A* reroute before reporting failure");
    require(
        std::any_of(layer_detail.report.warnings.begin(), layer_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("detailed routing used A* reroute fallback") != std::string::npos;
        }),
        "detailed routing should report real A* reroute fallback");
    require(
        std::none_of(layer_detail.report.warnings.begin(), layer_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("reassigned candidate layer") != std::string::npos;
        }),
        "detailed routing should not report free layer reassignment");
    require(!has_cross_net_same_layer_short(layer_detail.routes), "A* reroute fallback should not create same-layer shorts");
    require(net_routes_are_contiguous(layer_detail.routes, "N2"), "A* reroute fallback should keep the rerouted net connected");

    const auto drc_circuit = make_active_region_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> drc_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    sapr::RoutingEvaluationRequest drc_request;
    drc_request.placements = drc_placements;
    drc_request.placement_order = {"M"};

    const sapr::routing::RoutingContext merge_context(drc_circuit, drc_placements);
    const auto merge_request = make_dp_merge_compatibility_request(drc_placements);
    const auto merge_left = make_dp_merge_candidate(
        merge_context, "N_LEFT", "LEFT_SEGMENT", sapr::routing::Point{0.0, 2.0}, sapr::routing::Point{9.0, 2.0});
    const auto merge_right_short = make_dp_merge_candidate(
        merge_context, "N_RIGHT", "RIGHT_SEGMENT", sapr::routing::Point{4.0, 0.0}, sapr::routing::Point{4.0, 4.0});
    const auto short_merge_dp = sapr::routing::run_bottom_up_routing_dp(
        drc_circuit, merge_request, merge_context, {merge_left, merge_right_short}, 16);
    require(!short_merge_dp.success, "DP must reject root states assembled from shorting child routes");
    require(
        std::any_of(
            short_merge_dp.state_merge_events.begin(),
            short_merge_dp.state_merge_events.end(),
            [](const auto& event) { return event.reason == "occupied_route_short"; }),
        "DP child merge should record occupied-route short diagnostics");

    auto spacing_circuit = drc_circuit;
    spacing_circuit.constraints.spacing_rules.diff_net_route_spacing["M1"] = 1.0;
    const sapr::routing::RoutingContext spacing_merge_context(spacing_circuit, drc_placements);
    const auto merge_right_spacing = make_dp_merge_candidate(
        spacing_merge_context, "N_RIGHT", "RIGHT_SEGMENT", sapr::routing::Point{0.0, 4.0}, sapr::routing::Point{9.0, 4.0});
    const auto spacing_merge_dp = sapr::routing::run_bottom_up_routing_dp(
        spacing_circuit, merge_request, spacing_merge_context, {merge_left, merge_right_spacing}, 16);
    require(!spacing_merge_dp.success, "DP must reject child routes that violate different-net spacing");
    require(
        std::any_of(
            spacing_merge_dp.state_merge_events.begin(),
            spacing_merge_dp.state_merge_events.end(),
            [](const auto& event) { return event.reason == "route_spacing"; }),
        "DP child merge should record route-spacing diagnostics");

    const auto merge_right_legal = make_dp_merge_candidate(
        merge_context, "N_RIGHT", "RIGHT_SEGMENT", sapr::routing::Point{4.0, 7.0}, sapr::routing::Point{9.0, 7.0});
    const auto legal_merge_dp = sapr::routing::run_bottom_up_routing_dp(
        drc_circuit, merge_request, merge_context, {merge_left, merge_right_short, merge_right_legal}, 16);
    require(legal_merge_dp.success, "DP should retain a higher-cost child combination when the cheaper one shorts");
    require(
        std::any_of(
            legal_merge_dp.traceback_candidates.begin(),
            legal_merge_dp.traceback_candidates.end(),
            [](const auto& candidate) {
                return candidate.segment_id == "RIGHT_SEGMENT" && candidate.path.points.front().iy == 7;
            }),
        "successful DP traceback should select the legal right-child candidate");

    drc_request.active_region_blockers.push_back(
        sapr::routing::transform_active_to_global(drc_circuit.modules.at("M"), drc_placements.at("M")));

    auto bbox_margin_eval = make_line_evaluation(drc_circuit, drc_placements, 1.0);
    const auto bbox_margin_detail = sapr::run_detailed_routing(drc_circuit, drc_request, bbox_margin_eval);
    require(bbox_margin_detail.design_rule_violations == 0, "bbox margin route should not count as active-region DRC");

    auto active_crossing_eval = make_line_evaluation(drc_circuit, drc_placements, 5.0);
    const auto active_crossing_detail = sapr::run_detailed_routing(drc_circuit, drc_request, active_crossing_eval);
    require(active_crossing_detail.design_rule_violations == 0, "active crossing candidate should be legalized before final DRC");
    require(
        std::any_of(
            active_crossing_detail.report.warnings.begin(),
            active_crossing_detail.report.warnings.end(),
            [](const auto& warning) { return warning.find("detailed routing used A* reroute fallback") != std::string::npos; }),
        "active crossing candidate should use A* reroute fallback");

    // M2 从 active 上方跨过不应记为 active-region DRC；障碍也只应落在 M1。
    const auto multi_layer_active_config = sapr::routing::make_grid_config_for_routing_layers(2);
    const sapr::routing::RoutingContext multi_layer_active_context(
        drc_circuit, drc_placements, multi_layer_active_config);
    int active_obstacle_layers = 0;
    bool active_blocks_m1 = false;
    bool active_blocks_m2 = false;
    for (const auto& obstacle : multi_layer_active_context.obstacles().obstacles()) {
        if (obstacle.reason != "active_region") continue;
        ++active_obstacle_layers;
        if (obstacle.layer == 0) active_blocks_m1 = true;
        if (obstacle.layer == 1) active_blocks_m2 = true;
    }
    require(active_obstacle_layers == 1, "active region should create exactly one obstacle layer");
    require(active_blocks_m1, "active region should block M1");
    require(!active_blocks_m2, "active region should not block M2");
    const auto active_rect =
        sapr::routing::transform_active_to_global(drc_circuit.modules.at("M"), drc_placements.at("M"));
    const sapr::routing::Point active_center{
        0.5 * (active_rect.x1 + active_rect.x2),
        0.5 * (active_rect.y1 + active_rect.y2),
    };
    require(
        multi_layer_active_context.obstacles().is_blocked(active_center, 0),
        "active center should be blocked on M1");
    require(
        !multi_layer_active_context.obstacles().is_blocked(active_center, 1),
        "active center should remain free on M2");

    auto m2_over_active_eval = make_line_evaluation(drc_circuit, drc_placements, 5.0, 1, multi_layer_active_config);
    const auto m2_over_active_detail = sapr::run_detailed_routing(drc_circuit, drc_request, m2_over_active_eval);
    require(m2_over_active_detail.design_rule_violations == 0, "M2 over active region should not count as DRC");

    const auto full_active_circuit = make_full_active_region_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> full_active_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    const auto full_active_eval = sapr::evaluate_routing(full_active_circuit, full_active_placements);
    const auto full_active_segments = sapr::selected_candidates_to_segments(full_active_eval);
    require(!full_active_segments.empty(), "full-bbox active route should still find a path through pin access");
    const auto full_active_rect =
        sapr::routing::transform_active_to_global(full_active_circuit.modules.at("M"), full_active_placements.at("M"));
    for (const auto& segment : full_active_segments) {
        require(
            !route_crosses_active_core(segment, full_active_rect),
            "full-bbox active should block long routes through the module core");
    }

    const auto boundary_circuit = make_boundary_access_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> boundary_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    const sapr::routing::RoutingContext boundary_context(boundary_circuit, boundary_placements);
    require(boundary_context.grid().min_x() >= 0.0, "routing grid should not extend left of the chip boundary");
    require(boundary_context.grid().min_y() >= 0.0, "routing grid should not extend below the chip boundary");
    const auto boundary_eval = sapr::evaluate_routing(boundary_circuit, boundary_placements);
    const auto boundary_segments = sapr::selected_candidates_to_segments(boundary_eval);
    require(!boundary_segments.empty(), "boundary pin access should still find a routed path");
    const auto boundary_active_rect =
        sapr::routing::transform_active_to_global(boundary_circuit.modules.at("M"), boundary_placements.at("M"));
    for (const auto& segment : boundary_segments) {
        require(
            std::min(segment.x1, segment.x2) >= -1e-9 &&
                std::min(segment.y1, segment.y2) >= -1e-9,
            "route centerline should remain inside the non-negative chip boundary");
        require(
            !route_crosses_active_core(segment, boundary_active_rect),
            "boundary pin access should not become a long route through active core");
    }

    const auto priority_circuit = make_priority_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> priority_placements{
        {"S", sapr::Placement{"S", 0.0, 0.0, 0, "R0"}},
        {"S_MIRROR", sapr::Placement{"S_MIRROR", 20.0, 0.0, 0, "R0"}},
        {"P", sapr::Placement{"P", 40.0, 0.0, 0, "R0"}},
    };
    sapr::RoutingEvaluationRequest priority_request;
    priority_request.placements = priority_placements;
    priority_request.placement_order = {"S", "S_MIRROR", "P"};
    const auto priority_eval = make_priority_evaluation(priority_circuit, priority_placements);
    const auto priority_detail = sapr::run_detailed_routing(priority_circuit, priority_request, priority_eval);
    require(priority_detail.routes.size() >= 4, "priority detailed routing should emit one route per selected candidate");
    require(priority_detail.routes[0].net == "SYM_CRT", "symmetric critical net should be routed first");
    require(priority_detail.routes[1].net == "SYM_NOR", "symmetric normal net should follow symmetric critical net");
    require(priority_detail.routes[2].net == "PLAIN_CRT", "plain critical net should follow symmetric nets");
    require(priority_detail.routes[3].net == "PLAIN_NOR", "plain normal net should be routed last");

    const auto reverse_packing_circuit = make_reverse_packing_order_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> reverse_packing_placements{
        {"ROOT", sapr::Placement{"ROOT", 0.0, 0.0, 0, "R0"}},
        {"LEAF", sapr::Placement{"LEAF", 0.0, 0.0, 0, "R0"}},
    };
    const auto reverse_packing_request = make_reverse_packing_order_request(reverse_packing_placements);
    const auto reverse_packing_evaluation =
        make_reverse_packing_order_evaluation(reverse_packing_circuit, reverse_packing_placements);
    const auto reverse_packing_detail =
        sapr::run_detailed_routing(reverse_packing_circuit, reverse_packing_request, reverse_packing_evaluation);
    require(reverse_packing_detail.traceback_failures == 0, "reverse packing order test should keep both routes legal");
    require(reverse_packing_detail.routes.size() >= 2, "reverse packing order test should emit both selected routes");
    require(
        reverse_packing_detail.routes[0].net == "LEAF_NET" &&
            reverse_packing_detail.routes[1].net == "ROOT_NET",
        "detailed routing should trace back leaf-owned routes before root-owned routes");
    auto reverse_packing_fallback_evaluation =
        make_reverse_packing_order_evaluation(reverse_packing_circuit, reverse_packing_placements);
    reverse_packing_fallback_evaluation.bottom_up_dp.reset();
    reverse_packing_fallback_evaluation.used_bottom_up_dp = false;
    const auto reverse_packing_fallback_detail = sapr::run_detailed_routing(
        reverse_packing_circuit,
        reverse_packing_request,
        reverse_packing_fallback_evaluation);
    require(
        reverse_packing_fallback_detail.routes.size() >= 2 &&
            reverse_packing_fallback_detail.routes[0].net == "ROOT_NET" &&
            reverse_packing_fallback_detail.routes[1].net == "LEAF_NET",
        "detailed routing fallback must keep the pre-existing priority order without a DP traceback");

    auto lcp_request = make_lcp_request(drc_circuit, drc_placements, true);
    auto lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto lcp_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, lcp_eval);
    require(!lcp_detail.report.traces.empty(), "detailed routing should expose LCP traceback traces");
    require(!lcp_detail.report.traces.front().segments.empty(), "trace should map selected candidates to route segments");
    require(lcp_detail.space_nodes_with_routes == 1, "LCP detailed route should mark its space node as used");
    require(lcp_detail.required_space_by_node.contains("S1"), "LCP detailed route should report required routing space");
    require(lcp_detail.required_space_by_node.at("S1") >= 3.0, "required space should include route width and spacing");
    require(lcp_detail.detailed_cost > 0.0, "detailed routing should report local detailed cost");

    const auto lcp_variant_eval = sapr::evaluate_routing(drc_circuit, lcp_request);
    int left_lcp_variant_count = 0;
    bool has_forced_detour_variant = false;
    for (const auto& candidate : lcp_variant_eval.candidates) {
        if (!candidate.path.success || candidate.segment_id != "N:left" ||
            candidate.lcp_candidate_id != "LCP1:first") {
            continue;
        }
        ++left_lcp_variant_count;
        has_forced_detour_variant = has_forced_detour_variant ||
                                     candidate.route_variant.find("avoid_pivot_") == 0;
    }
    require(left_lcp_variant_count >= 2, "LCP segment should retain physically distinct route variants");
    require(has_forced_detour_variant, "LCP variants should include a forced-detour path when the base path is blocked");

    // LCP 与引脚重合时，单点 A* 路径是合法的零长度连接，不应导致整网 detailed traceback 失败。
    auto zero_length_lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    auto& zero_length_candidate = zero_length_lcp_eval.global_routing.net_routes.front().selected_candidates.front();
    zero_length_candidate.path.points = {
        zero_length_lcp_eval.context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.0}, 0),
    };
    zero_length_candidate.path.metrics = {};
    const auto zero_length_lcp_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, zero_length_lcp_eval);
    require(
        zero_length_lcp_detail.traceback_failures == 0,
        "single-point LCP candidate should be accepted as a zero-length detailed connection");

    const auto lcp_pair_request = make_lcp_pair_request(drc_circuit, drc_placements);
    const auto lcp_pair_eval = make_lcp_pair_evaluation(drc_circuit, drc_placements);
    const auto lcp_pair_detail = sapr::run_detailed_routing(drc_circuit, lcp_pair_request, lcp_pair_eval);
    require(lcp_pair_detail.required_space_by_node.contains("S_A"), "LCP-LCP route should feedback source space");
    require(lcp_pair_detail.required_space_by_node.contains("S_B"), "LCP-LCP route should feedback target space");

    auto flow_circuit = drc_circuit;
    flow_circuit.constraints.flows.push_back({"N", "M.A", "M.B"});
    const auto flow_ok_detail = sapr::run_detailed_routing(flow_circuit, lcp_request, lcp_eval);
    require(flow_ok_detail.flow_violations == 0, "out-pin to in-pin LCP traceback should satisfy FLOW");
    auto reverse_lcp_request = make_reverse_lcp_request(flow_circuit, drc_placements, true);
    const auto reverse_lcp_eval = sapr::evaluate_routing(flow_circuit, reverse_lcp_request);
    const bool has_reverse_flow_candidate = std::any_of(
        reverse_lcp_eval.candidates.begin(),
        reverse_lcp_eval.candidates.end(),
        [](const auto& candidate) { return !candidate.flow_ok && candidate.flow_penalty > 0.0; });
    require(has_reverse_flow_candidate, "reversed LCP segments should get FLOW penalty during candidate generation");
    require(
        reverse_lcp_eval.global_routing.flow_penalty > 0.0,
        "reversed LCP segments should contribute FLOW penalty during global routing");

    auto reverse_flow_eval = make_lcp_evaluation(flow_circuit, drc_placements);
    auto& reverse_choice = reverse_flow_eval.global_routing.net_routes.front();
    reverse_choice.selected_candidates[0].from_terminal = "LCP1";
    reverse_choice.selected_candidates[0].to_terminal = "M.A";
    reverse_choice.selected_candidates[1].from_terminal = "M.B";
    reverse_choice.selected_candidates[1].to_terminal = "LCP1";
    const auto reverse_flow_detail = sapr::run_detailed_routing(flow_circuit, lcp_request, reverse_flow_eval);
    require(reverse_flow_detail.flow_violations > 0, "reversed LCP traceback should violate FLOW");
    require(reverse_flow_detail.flow_penalty > 0.0, "FLOW violation should add detailed flow penalty");
    require(!reverse_flow_detail.report.flow_segments.empty(), "FLOW violation should be reported with segment source");

    auto width_violation_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    width_violation_eval.global_routing.net_routes.front().selected_candidates.front().wire_width = 1.0;
    const auto width_violation_detail = sapr::run_detailed_routing(drc_circuit, lcp_request, width_violation_eval);
    require(width_violation_detail.current_density_violations > 0, "segment width below min should violate current-density proxy");
    require(width_violation_detail.current_density_penalty > 0.0, "width violation should add current-density penalty");
    require(
        !width_violation_detail.report.current_density_segments.empty(),
        "current-density violation should be reported with segment source");

    auto missing_lcp_request = make_lcp_request(drc_circuit, drc_placements, false);
    const auto missing_lcp_global_eval = sapr::evaluate_routing(drc_circuit, missing_lcp_request);
    require(missing_lcp_global_eval.failed_nets > 0, "missing LCP location should fail during global routing");
    require(
        missing_lcp_global_eval.global_routing.routing_failure_penalty > 0.0,
        "missing LCP location should add global routing failure penalty");
    const bool has_fake_fallback_location = std::any_of(
        missing_lcp_global_eval.candidates.begin(),
        missing_lcp_global_eval.candidates.end(),
        [](const auto& candidate) {
            return candidate.lcp_candidate_id.find(":fallback") != std::string::npos || candidate.path.success;
        });
    require(!has_fake_fallback_location, "missing LCP location should not create a fake successful fallback candidate");
    auto missing_lcp_eval = make_lcp_evaluation(drc_circuit, drc_placements);
    const auto missing_lcp_detail = sapr::run_detailed_routing(drc_circuit, missing_lcp_request, missing_lcp_eval);
    require(missing_lcp_detail.traceback_failures > 0, "missing LCP location should become a traceback failure");
    require(missing_lcp_detail.routing_failure_penalty > 0.0, "traceback failures should add routing failure penalty");

    auto strict_missing_lcp_request = make_lcp_request(drc_circuit, drc_placements, false);
    strict_missing_lcp_request.tree.root = "M";
    strict_missing_lcp_request.tree.nodes.push_back({"M", "M", std::nullopt, std::nullopt});
    strict_missing_lcp_request.strict_lcp_dp = true;
    const auto strict_missing_lcp_eval = sapr::evaluate_routing(drc_circuit, strict_missing_lcp_request);
    require(
        strict_missing_lcp_eval.strict_lcp_dp_blocked_fallback,
        "strict LCP DP should report that direct fallback was blocked");
    require(
        strict_missing_lcp_eval.failed_nets > 0,
        "strict LCP DP should not hide failed LCP topology behind direct routing");

    sapr::routing::RoutingContext lcp_context(drc_circuit, drc_placements);
    std::vector<sapr::routing::RouteCandidate> multi_location_candidates{
        make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:a", 1.0, 1.0),
        make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:b", 100.0, 2.0),
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:a", 100.0, 3.0),
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:b", 1.0, 4.0),
    };
    const auto lcp_global = sapr::routing::run_global_routing(drc_circuit, lcp_context, multi_location_candidates);
    require(lcp_global.failed_nets == 0, "consistent LCP location selection should find a route");
    require(lcp_global.net_routes.front().selected_candidates.size() == 2, "LCP net should select one candidate per segment");
    const auto selected_location = lcp_global.net_routes.front().selected_candidates.front().lcp_candidate_id;
    for (const auto& candidate : lcp_global.net_routes.front().selected_candidates) {
        require(candidate.lcp_candidate_id == selected_location, "all selected segments of one LCP should share one location");
    }

    auto partial_left = make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:first", 5.0, 1.0);
    partial_left.segment_id = "N:left";
    const std::vector<sapr::routing::RouteCandidate> partial_lcp_candidates{partial_left};
    const auto partial_coverage = sapr::analyze_lcp_candidate_coverage(lcp_request, partial_lcp_candidates);
    const auto partial_first = std::find_if(partial_coverage.begin(), partial_coverage.end(), [](const auto& coverage) {
        return coverage.lcp_id == "LCP1" && coverage.candidate_id == "LCP1:first";
    });
    require(partial_first != partial_coverage.end(), "LCP coverage report should include the physical candidate");
    require(
        !partial_first->covers_all_incident_segments,
        "LCP candidate should not cover all incident segments when one branch is missing");
    require(
        std::find(partial_first->missing_segments.begin(), partial_first->missing_segments.end(), "N:right") !=
            partial_first->missing_segments.end(),
        "LCP coverage report should name the missing incident segment");

    auto complete_left = make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:first", 5.0, 1.0);
    complete_left.segment_id = "N:left";
    auto complete_right = make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:first", 5.0, 1.0);
    complete_right.segment_id = "N:right";
    const std::vector<sapr::routing::RouteCandidate> complete_lcp_candidates{complete_left, complete_right};
    const auto complete_coverage = sapr::analyze_lcp_candidate_coverage(lcp_request, complete_lcp_candidates);
    const auto complete_first = std::find_if(complete_coverage.begin(), complete_coverage.end(), [](const auto& coverage) {
        return coverage.lcp_id == "LCP1" && coverage.candidate_id == "LCP1:first";
    });
    require(complete_first != complete_coverage.end(), "complete LCP coverage should include the physical candidate");
    require(
        complete_first->covers_all_incident_segments,
        "LCP candidate should cover all incident segments only when every branch is reachable");

    const auto pairwise_coverage_request = make_pairwise_coverage_request(drc_circuit, drc_placements);
    const auto pairwise_coverage_eval = sapr::evaluate_routing(drc_circuit, pairwise_coverage_request);
    const auto covered_locations = [&](const std::string& segment_id, const std::string& lcp_id) {
        std::unordered_set<std::string> locations;
        for (const auto& candidate : pairwise_coverage_eval.debug_candidates) {
            if (candidate.segment_id != segment_id || !candidate.path.success) continue;
            if (candidate.source_lcp_id == lcp_id) locations.insert(candidate.source_lcp_candidate_id);
            if (candidate.target_lcp_id == lcp_id) locations.insert(candidate.target_lcp_candidate_id);
        }
        return locations;
    };
    require(
        covered_locations("N:leaf0_root", "LEAF0").size() == 40,
        "coverage-aware LCP pair selection should keep every source location for the first branch");
    require(
        covered_locations("N:leaf0_root", "ROOT").size() == 40,
        "coverage-aware LCP pair selection should keep every root location for the first branch");
    require(
        covered_locations("N:leaf1_root", "LEAF1").size() == 40,
        "coverage-aware LCP pair selection should keep every source location for the second branch");
    require(
        covered_locations("N:leaf1_root", "ROOT").size() == 40,
        "coverage-aware LCP pair selection should keep every root location for the second branch");

    auto missing_segment_request = make_lcp_request(drc_circuit, drc_placements, true);
    missing_segment_request.tree.root = "M";
    missing_segment_request.tree.nodes.push_back({"M", "M", std::nullopt, std::nullopt});
    const std::vector<sapr::routing::RouteCandidate> incomplete_candidates{
        make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:first", 5.0, 1.0),
    };
    const auto missing_segment_dp =
        sapr::routing::run_bottom_up_routing_dp(drc_circuit, missing_segment_request, lcp_context, incomplete_candidates);
    require(!missing_segment_dp.success, "bottom-up DP should fail when a required wire segment has no candidate");
    require(
        missing_segment_dp.best_state.penalty >= 100000.0,
        "missing required wire segment should add routing failure penalty");
    require(
        !missing_segment_dp.best_state.failure_messages.empty(),
        "missing required wire segment should be recorded in DP failure messages");
    require(
        std::any_of(
            missing_segment_dp.best_state.failure_messages.begin(),
            missing_segment_dp.best_state.failure_messages.end(),
            [](const std::string& message) {
                return message.find("no DP-compatible A* candidate") != std::string::npos;
            }),
        "DP failure should distinguish missing state-compatible candidates from A* path generation failure");

    auto expensive_left = make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:a", 9.0, 1.0);
    expensive_left.segment_id = "N:left";
    expensive_left.path.metrics.wirelength = 200000.0;
    auto cheap_incomplete_left = make_manual_lcp_candidate(lcp_context, "M.A", "LCP1", "LCP1:b", 1.0, 2.0);
    cheap_incomplete_left.segment_id = "N:left";
    auto expensive_complete_right = make_manual_lcp_candidate(lcp_context, "LCP1", "M.B", "LCP1:a", 1.0, 3.0);
    expensive_complete_right.segment_id = "N:right";
    const std::vector<sapr::routing::RouteCandidate> root_selection_candidates{
        expensive_left,
        cheap_incomplete_left,
        expensive_complete_right,
    };
    const auto root_selection_dp = sapr::routing::run_bottom_up_routing_dp(
        drc_circuit,
        missing_segment_request,
        lcp_context,
        root_selection_candidates);
    require(
        root_selection_dp.success,
        "bottom-up DP should select a complete root state even when an incomplete state has lower cost");
    require(
        root_selection_dp.best_state.lcp_location_by_id.at("LCP1") == "LCP1:a",
        "successful root state should retain the complete LCP physical binding");

    auto coupling_eval = make_line_evaluation(drc_circuit, drc_placements, 1.0);
    auto coupling_candidate = coupling_eval.global_routing.net_routes.front().selected_candidates.front();
    coupling_candidate.net = "P";
    coupling_candidate.path.points = {
        coupling_eval.context.grid().snap_to_grid(sapr::routing::Point{0.0, 1.5}, 0),
        coupling_eval.context.grid().snap_to_grid(sapr::routing::Point{9.0, 1.5}, 0),
    };
    sapr::routing::NetRouteChoice coupling_choice;
    coupling_choice.net = "P";
    coupling_choice.selected_candidates.push_back(coupling_candidate);
    coupling_eval.global_routing.net_routes.push_back(coupling_choice);
    const auto coupling_detail = sapr::run_detailed_routing(drc_circuit, drc_request, coupling_eval);
    require(coupling_detail.traceback_failures == 0, "same-layer different-net overlap should use real A* reroute when possible");
    require(!coupling_detail.routes.empty(), "rerouted detailed routing should keep route output");
    require(
        std::any_of(coupling_detail.report.warnings.begin(), coupling_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("detailed routing used A* reroute fallback") != std::string::npos;
        }),
        "same-layer overlap should report real A* reroute fallback");
    require(
        std::none_of(coupling_detail.report.warnings.begin(), coupling_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("reassigned candidate layer") != std::string::npos;
        }),
        "same-layer overlap should not report free layer reassignment");
    require(!has_cross_net_same_layer_short(coupling_detail.routes), "overlap reroute should not create same-layer shorts");
    require(net_routes_are_contiguous(coupling_detail.routes, "P"), "overlap reroute should keep the rerouted net connected");

    // detailed 不得按支路拆散 LCP：location A 短路时应整网切到同一 location B。
    const auto star_circuit = make_star_lcp_test_circuit();
    const std::unordered_map<std::string, sapr::Placement> star_placements{
        {"M", sapr::Placement{"M", 0.0, 0.0, 0, "R0"}},
    };
    auto star_eval = make_star_lcp_consistency_evaluation(star_circuit, star_placements);
    const auto star_request = make_star_lcp_request(star_circuit, star_placements);
    const auto star_detail = sapr::run_detailed_routing(star_circuit, star_request, star_eval);
    require(
        std::none_of(star_detail.report.warnings.begin(), star_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("switched whole-net LCP location to LCP_STAR:b") != std::string::npos;
        }),
        "detailed routing should keep the DP-selected LCP binding stable");
    require(
        std::none_of(star_detail.report.warnings.begin(), star_detail.report.warnings.end(), [](const auto& warning) {
            return warning.find("selected branches disagree on LCP binding before detailed legalize") !=
                   std::string::npos;
        }),
        "consistent DP-selected STAR branches should not report an LCP binding conflict");
    require(
        star_detail.traceback_failures > 0,
        "fixed LCP binding should fail this blocked STAR location instead of switching during detailed routing");
    const sapr::DetailedRouteTrace* star_trace = nullptr;
    for (const auto& trace : star_detail.report.traces) {
        if (trace.net == "STAR") {
            star_trace = &trace;
            break;
        }
    }
    require(star_trace != nullptr, "STAR detailed traceback should exist");
    for (const auto& segment : star_trace->segments) {
        require(
            segment.lcp_candidate_id.empty() || segment.lcp_candidate_id == "LCP_STAR:a",
            "STAR detailed segments must not switch away from the DP-selected LCP location");
    }
    require(!has_cross_net_same_layer_short(star_detail.routes), "LCP-consistent detailed routes should not short");

    // placement-aware 流程显式开启协商后，应在提交 detailed route 前整网选择可行的备用 LCP 位置。
    auto negotiated_star_request = star_request;
    negotiated_star_request.allow_lcp_location_negotiation = true;
    const auto negotiated_star_detail = sapr::run_detailed_routing(star_circuit, negotiated_star_request, star_eval);
    require(
        negotiated_star_detail.traceback_failures == 0,
        "explicit whole-net LCP negotiation should resolve the blocked DP-selected location");
    require(
        std::any_of(
            negotiated_star_detail.report.warnings.begin(),
            negotiated_star_detail.report.warnings.end(),
            [](const auto& warning) {
                return warning.find("negotiated whole-net LCP location to LCP_STAR:b") != std::string::npos;
            }),
        "explicit LCP negotiation should report the committed replacement location");
    for (const auto& trace : negotiated_star_detail.report.traces) {
        if (trace.net != "STAR") continue;
        for (const auto& segment : trace.segments) {
            require(
                segment.lcp_candidate_id.empty() || segment.lcp_candidate_id == "LCP_STAR:b",
                "negotiated STAR branches must share the replacement LCP location");
        }
    }
}
