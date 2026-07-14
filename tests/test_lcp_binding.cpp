// 文件职责：验证初始 LCP 绑定及 packing 刷新不会改变 SA 管理的 space 归属。
#include "test_support.hpp"

#include <string>
#include <vector>

#include "sapr/lcp_generator.hpp"
#include "sapr/model.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/tree.hpp"

namespace {

// 构造左右两个模块，三端网全部落在右侧模块上。
sapr::Circuit make_right_biased_three_pin_circuit() {
    sapr::Circuit circuit;
    circuit.modules.emplace("LEFT", sapr::Module{"LEFT", 10.0, 10.0, sapr::Rect{1.0, 1.0, 9.0, 9.0}, 0.0, 0.0, ""});
    circuit.modules.emplace("RIGHT", sapr::Module{"RIGHT", 10.0, 10.0, sapr::Rect{1.0, 1.0, 9.0, 9.0}, 0.0, 0.0, ""});
    circuit.module_order = {"LEFT", "RIGHT"};

    circuit.pins.emplace("RIGHT.A", sapr::Pin{"RIGHT", "A", 2.0, 2.0, "M1"});
    circuit.pins.emplace("RIGHT.B", sapr::Pin{"RIGHT", "B", 5.0, 5.0, "M1"});
    circuit.pins.emplace("RIGHT.C", sapr::Pin{"RIGHT", "C", 8.0, 8.0, "M1"});
    circuit.pin_order = {"RIGHT.A", "RIGHT.B", "RIGHT.C"};

    sapr::Net net{"N", sapr::Priority::Critical, {"RIGHT.A", "RIGHT.B", "RIGHT.C"}};
    circuit.nets.emplace("N", net);
    circuit.net_order.push_back("N");
    return circuit;
}

// 返回指定 LCP 当前绑定的 space id。
std::string space_id_for_lcp(const sapr::RoutingEvaluationRequest& request, const std::string& lcp_id) {
    for (const auto& point : request.linking_points) {
        if (point.id == lcp_id) return point.space_node_id;
    }
    for (const auto& space : request.space_nodes) {
        for (const auto& point : space.linking_points) {
            if (point.id == lcp_id) return space.id;
        }
    }
    return {};
}

}  // namespace

// 运行 LCP 初始绑定与固定归属候选刷新测试。
void run_lcp_binding_tests() {
    const auto circuit = make_right_biased_three_pin_circuit();
    auto tree = sapr::make_enhanced_tree(circuit);
    // 固定树形：LEFT 为根，RIGHT 为其左孩子，使 RIGHT:right 更靠近右侧 pin。
    tree.root = "LEFT";
    tree.nodes.at("LEFT").parent = std::nullopt;
    tree.nodes.at("LEFT").left = "RIGHT";
    tree.nodes.at("LEFT").right = std::nullopt;
    tree.nodes.at("RIGHT").parent = "LEFT";
    tree.nodes.at("RIGHT").left = std::nullopt;
    tree.nodes.at("RIGHT").right = std::nullopt;
    tree.representative_order = {"LEFT", "RIGHT"};

    sapr::RoutingEvaluationRequest request;
    request.placements["LEFT"] = {"LEFT", 0.0, 0.0, 0, "R0"};
    request.placements["RIGHT"] = {"RIGHT", 20.0, 0.0, 0, "R0"};
    request.placement_order = {"LEFT", "RIGHT"};
    request.space_nodes = sapr::collect_space_nodes(tree);
    for (auto& space : request.space_nodes) {
        if (space.id == "LEFT:right") {
            space.physical_region = sapr::Rect{10.0, 0.0, 15.0, 10.0};
        } else if (space.id == "RIGHT:right") {
            space.physical_region = sapr::Rect{30.0, 0.0, 35.0, 10.0};
        } else if (space.id == "LEFT:top") {
            space.physical_region = sapr::Rect{0.0, 10.0, 10.0, 15.0};
        } else if (space.id == "RIGHT:top") {
            space.physical_region = sapr::Rect{20.0, 10.0, 30.0, 15.0};
        }
    }
    request.placed_pins = {
        {"RIGHT.A", "RIGHT", "A", 22.0, 2.0, "M1"},
        {"RIGHT.B", "RIGHT", "B", 25.0, 5.0, "M1"},
        {"RIGHT.C", "RIGHT", "C", 28.0, 8.0, "M1"},
    };

    sapr::generate_initial_lcp_topology(circuit, request);
    require(!request.linking_points.empty(), "three-pin net should create at least one LCP");
    const auto leaf_space = space_id_for_lcp(request, "N:leaf:0");
    require(!leaf_space.empty(), "leaf LCP should be bound to a space");
    require(leaf_space.find("RIGHT") != std::string::npos,
            "right-biased pins should bind leaf LCP to a RIGHT-owned space, got " + leaf_space);

    // 人为把 LCP 移到左侧 space；packing 刷新候选时不得绕过 SA 自动重绑。
    sapr::LinkingControlPoint stuck = request.linking_points.front();
    for (auto& space : request.space_nodes) space.linking_points.clear();
    request.linking_points.clear();
    stuck.space_node_id = "LEFT:right";
    stuck.location_candidates.clear();
    bool inserted = false;
    for (auto& space : request.space_nodes) {
        if (space.id != "LEFT:right") continue;
        space.linking_points.push_back(stuck);
        inserted = true;
        break;
    }
    require(inserted, "LEFT:right should exist for mismatch rebind test");

    sapr::generate_automatic_lcps(circuit, request);
    const auto refreshed = space_id_for_lcp(request, stuck.id);
    require(refreshed == "LEFT:right", "candidate refresh must preserve the LCP space selected by SA");

    // 已在 RIGHT space 上时，即使 ideal 不在窄 region 内，也不应在刷新时迁移。
    for (auto& space : request.space_nodes) space.linking_points.clear();
    request.linking_points.clear();
    sapr::LinkingControlPoint stable = stuck;
    stable.space_node_id = leaf_space;
    stable.location_candidates.clear();
    bool stable_inserted = false;
    for (auto& space : request.space_nodes) {
        if (space.id != leaf_space) continue;
        space.linking_points.push_back(stable);
        stable_inserted = true;
        break;
    }
    require(stable_inserted, "stable initial space should exist");
    sapr::generate_automatic_lcps(circuit, request);
    require(space_id_for_lcp(request, stable.id) == leaf_space,
            "candidate refresh should keep the existing LCP binding");

    // 初始 LCP 必须在 SA 前建立，不能依赖 packing 后的物理坐标。
    auto initial_tree = sapr::make_enhanced_tree(circuit);
    sapr::initialize_lcp_topology(circuit, initial_tree);
    require(sapr::count_tree_lcps(initial_tree) > 0, "initial LCP topology should exist before packing");
}
