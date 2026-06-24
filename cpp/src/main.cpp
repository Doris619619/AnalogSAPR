// 文件职责：提供 C++ 命令行入口，串联输入解析、布线环境构建和阶段调试输出。
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "io/parser.hpp"
#include "io/writer.hpp"
#include "routing/global_router.hpp"
#include "routing/routing_context.hpp"
#include "routing/topology.hpp"

namespace {

// 保存命令行参数解析结果。
struct CliOptions {
    std::filesystem::path input_dir;
    std::filesystem::path placement_path;
    std::filesystem::path output_dir;
    bool help = false;
    bool dump_grid_info = false;
    bool dump_route_candidates = false;
    bool dump_global_routing = false;
};

// 打印工具的用法说明。
void print_usage() {
    std::cout
        << "用法:\n"
        << "  analog_sapr --input <input_dir> --placement <placement.txt> --output <output_dir> "
        << "[--dump-grid-info] [--dump-route-candidates] [--dump-global-routing]\n\n"
        << "阶段 3 目标:\n"
        << "  基于 A* 候选路径执行不计 via cost 的 DP-based global routing。\n";
}

// 解析最小命令行参数集合。
CliOptions parse_args(int argc, char** argv) {
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--input" && i + 1 < argc) {
            options.input_dir = argv[++i];
        } else if (arg == "--placement" && i + 1 < argc) {
            options.placement_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            options.output_dir = argv[++i];
        } else if (arg == "--dump-grid-info") {
            options.dump_grid_info = true;
        } else if (arg == "--dump-route-candidates") {
            options.dump_route_candidates = true;
        } else if (arg == "--dump-global-routing") {
            options.dump_global_routing = true;
        } else {
            throw std::runtime_error("未知或缺少值的参数: " + arg);
        }
    }

    if (!options.help && (options.input_dir.empty() || options.placement_path.empty() || options.output_dir.empty())) {
        throw std::runtime_error("必须提供 --input、--placement 和 --output");
    }
    return options;
}

// 打印阶段 1 网格和障碍物调试信息。
void dump_grid_info(const analog_sapr::RoutingContext& context) {
    const auto& grid = context.grid();
    const auto& obstacles = context.obstacles();

    std::cout << std::fixed << std::setprecision(3)
              << "阶段 1 网格信息\n"
              << "  grid_step: " << grid.step() << "\n"
              << "  x_range: [" << grid.min_x() << ", " << grid.max_x() << "]\n"
              << "  y_range: [" << grid.min_y() << ", " << grid.max_y() << "]\n"
              << "  x_count: " << grid.x_count() << "\n"
              << "  y_count: " << grid.y_count() << "\n"
              << "  layer_count: " << grid.layer_count() << "\n"
              << "  global_pins: " << context.global_pins().size() << "\n"
              << "  obstacles: " << obstacles.obstacles().size() << "\n"
              << "  blocked_grid_points: " << obstacles.estimate_blocked_grid_points(grid) << "\n";

    for (const auto& warning : context.warnings()) {
        std::cout << "  warning: " << warning << "\n";
    }
}

// 打印阶段 2 A* 候选路径调试信息。
void dump_route_candidates(const analog_sapr::Circuit& circuit, const analog_sapr::RoutingContext& context) {
    const auto candidates = analog_sapr::generate_route_candidates(circuit, context);
    int success_count = 0;
    int failed_count = 0;

    std::cout << std::fixed << std::setprecision(3)
              << "阶段 2 A* 候选路径\n";
    for (const auto& candidate : candidates) {
        if (candidate.path.success) {
            ++success_count;
            std::cout << "  ok   net=" << candidate.net
                      << " " << candidate.from_terminal << " -> " << candidate.to_terminal
                      << " points=" << candidate.path.points.size()
                      << " wirelength=" << candidate.path.metrics.wirelength
                      << " bends=" << candidate.path.metrics.bend_count
                      << " vias=" << candidate.path.metrics.via_count
                      << " cost=" << candidate.path.metrics.cost << "\n";
        } else {
            ++failed_count;
            std::cout << "  fail net=" << candidate.net
                      << " " << candidate.from_terminal << " -> " << candidate.to_terminal
                      << " reason=" << candidate.path.message << "\n";
        }
    }

    std::cout << "  summary: success=" << success_count
              << " failed=" << failed_count
              << " total=" << candidates.size() << "\n";
}

// 打印阶段 3 global routing 选择结果，via 只展示不参与 cost。
void dump_global_routing(const analog_sapr::Circuit& circuit, const analog_sapr::RoutingContext& context) {
    const auto candidates = analog_sapr::generate_route_candidates(circuit, context);
    const auto result = analog_sapr::run_global_routing(circuit, context, candidates);

    std::cout << std::fixed << std::setprecision(3)
              << "阶段 3 DP-based Global Routing\n";
    for (const auto& route : result.net_routes) {
        std::cout << "  net=" << route.net
                  << " success=" << (route.success ? "true" : "false")
                  << " pairs=" << route.selected_candidates.size()
                  << " wirelength=" << route.metrics.wirelength
                  << " bends=" << route.metrics.bend_count
                  << " vias=" << route.metrics.via_count
                  << " penalty=" << route.penalty
                  << " cost=" << route.metrics.cost;
        if (!route.message.empty()) {
            std::cout << " message=" << route.message;
        }
        std::cout << "\n";
    }

    std::cout << "  total_wirelength=" << result.total_metrics.wirelength << "\n"
              << "  total_bends=" << result.total_metrics.bend_count << "\n"
              << "  total_vias=" << result.total_metrics.via_count << "\n"
              << "  total_penalty=" << result.total_penalty << "\n"
              << "  failed_nets=" << result.failed_nets << "\n"
              << "  total_cost=" << result.total_metrics.cost << "\n";
}

}  // namespace

// 程序入口：完成阶段 3 的 global routing 调试和 IO 闭环。
int main(int argc, char** argv) {
    try {
        const auto options = parse_args(argc, argv);
        if (options.help) {
            print_usage();
            return 0;
        }

        analog_sapr::Circuit circuit = analog_sapr::load_circuit(options.input_dir);
        auto placements = analog_sapr::load_placements(options.placement_path);
        analog_sapr::RoutingContext routing_context(circuit, placements);

        analog_sapr::RoutingSolution solution;
        solution.placements = std::move(placements);
        analog_sapr::write_routes(solution, options.output_dir);

        std::cout << "阶段 3 DP-based Global Routing 环境构建完成\n"
                  << "  modules: " << circuit.modules.size() << "\n"
                  << "  pins: " << circuit.pins.size() << "\n"
                  << "  nets: " << circuit.nets.size() << "\n"
                  << "  symmetry_pairs: " << circuit.constraints.symmetry_pairs.size() << "\n"
                  << "  symmetry_selfs: " << circuit.constraints.symmetry_selfs.size() << "\n"
                  << "  flows: " << circuit.constraints.flows.size() << "\n"
                  << "  wire_widths: " << circuit.constraints.wire_widths.size() << "\n"
                  << "  placements: " << solution.placements.size() << "\n"
                  << "  global_pins: " << routing_context.global_pins().size() << "\n"
                  << "  obstacles: " << routing_context.obstacles().obstacles().size() << "\n"
                  << "  routing: " << (options.output_dir / "routing.txt").string() << "\n";
        if (options.dump_grid_info) {
            dump_grid_info(routing_context);
        }
        if (options.dump_route_candidates) {
            dump_route_candidates(circuit, routing_context);
        }
        if (options.dump_global_routing) {
            dump_global_routing(circuit, routing_context);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << "\n\n";
        print_usage();
        return 1;
    }
}
