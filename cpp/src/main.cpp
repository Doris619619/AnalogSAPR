// 文件职责：提供阶段 0 的 C++ 命令行入口，串联输入解析、placement 读取和 routing 写出。
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "io/parser.hpp"
#include "io/writer.hpp"

namespace {

// 保存命令行参数解析结果。
struct CliOptions {
    std::filesystem::path input_dir;
    std::filesystem::path placement_path;
    std::filesystem::path output_dir;
    bool help = false;
};

// 打印阶段 0 工具的用法说明。
void print_usage() {
    std::cout
        << "用法:\n"
        << "  analog_sapr --input <input_dir> --placement <placement.txt> --output <output_dir>\n\n"
        << "阶段 0 目标:\n"
        << "  读取 IO_example 兼容输入和 placement，写出合法的空 routing.txt。\n";
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
        } else {
            throw std::runtime_error("未知或缺少值的参数: " + arg);
        }
    }

    if (!options.help && (options.input_dir.empty() || options.placement_path.empty() || options.output_dir.empty())) {
        throw std::runtime_error("必须提供 --input、--placement 和 --output");
    }
    return options;
}

}  // namespace

// 程序入口：完成阶段 0 的 IO 闭环。
int main(int argc, char** argv) {
    try {
        const auto options = parse_args(argc, argv);
        if (options.help) {
            print_usage();
            return 0;
        }

        analog_sapr::Circuit circuit = analog_sapr::load_circuit(options.input_dir);
        auto placements = analog_sapr::load_placements(options.placement_path);

        analog_sapr::RoutingSolution solution;
        solution.placements = std::move(placements);
        analog_sapr::write_routes(solution, options.output_dir);

        std::cout << "阶段 0 IO 闭环完成\n"
                  << "  modules: " << circuit.modules.size() << "\n"
                  << "  pins: " << circuit.pins.size() << "\n"
                  << "  nets: " << circuit.nets.size() << "\n"
                  << "  symmetry_pairs: " << circuit.constraints.symmetry_pairs.size() << "\n"
                  << "  symmetry_selfs: " << circuit.constraints.symmetry_selfs.size() << "\n"
                  << "  flows: " << circuit.constraints.flows.size() << "\n"
                  << "  wire_widths: " << circuit.constraints.wire_widths.size() << "\n"
                  << "  placements: " << solution.placements.size() << "\n"
                  << "  routing: " << (options.output_dir / "routing.txt").string() << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << "\n\n";
        print_usage();
        return 1;
    }
}
