// 实现 sapr validate/run 命令行入口和稳定的指标输出。
// 文件职责：实现命令行入口，负责输入校验、求解运行和 routing debug 输出。
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sapr/constraints.hpp"
#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/router.hpp"
#include "sapr/sa_btree_dump.hpp"

namespace {

// 返回指定选项的值，不存在时返回默认值。
std::string option_value(const std::vector<std::string>& args, const std::string& option, const std::string& fallback) {
    for (std::size_t index = 0; index < args.size(); ++index) {
        if (args[index] == option) {
            if (index + 1 >= args.size()) throw std::runtime_error("missing value for " + option);
            return args[index + 1];
        }
    }
    return fallback;
}

// 将字符串严格解析为 double 参数。
double option_double(const std::vector<std::string>& args, const std::string& option, double fallback) {
    const std::string value = option_value(args, option, "");
    if (value.empty()) return fallback;
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size()) throw std::runtime_error("invalid value for " + option + ": " + value);
    return parsed;
}

// 将字符串严格解析为 int 参数。
int option_int(const std::vector<std::string>& args, const std::string& option, int fallback) {
    const std::string value = option_value(args, option, "");
    if (value.empty()) return fallback;
    std::size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed);
    if (consumed != value.size()) throw std::runtime_error("invalid value for " + option + ": " + value);
    return parsed;
}

// 将字符串严格解析为 unsigned int 参数。
unsigned int option_uint(const std::vector<std::string>& args, const std::string& option, unsigned int fallback) {
    const int parsed = option_int(args, option, static_cast<int>(fallback));
    if (parsed < 0) throw std::runtime_error("invalid value for " + option + ": must be non-negative");
    return static_cast<unsigned int>(parsed);
}

// 判断某个无值命令行选项是否存在。
bool has_option(const std::vector<std::string>& args, const std::string& option) {
    for (const auto& arg : args) {
        if (arg == option) return true;
    }
    return false;
}

// 输出 CLI 使用说明。
void print_usage() {
    std::cerr << "Usage:\n"
              << "  sapr validate [--input input]\n"
              << "  sapr run [--input input] [--output output] [--spacing 5] [--row-width 40]\n"
              << "           [--boundary-margin value] [--boundary-clearance 0]\n"
              << "           [--seed 1] [--sa-iterations 250] [--initial-temperature 5]\n"
              << "           [--cooling-rate 0.96] [--dump-routing-eval]\n"
              << "           [--debug-search]\n"
              << "           [--render-dpi 200] [--render-name name]\n"
              << "           [--dump-btree] [--render-btree-name name]\n"
              << "           [--no-dump-sa-btree]\n";
}

std::string shell_quote(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char ch : value) {
        if (ch == '"') escaped += "\\\"";
        else escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

std::string shell_quote(const std::filesystem::path& value) {
    return shell_quote(value.string());
}

std::filesystem::path renderer_script_path(const char*) {
    const auto source_candidate = std::filesystem::path("tools") / "render_layout.py";
    if (std::filesystem::exists(source_candidate)) return source_candidate;
    const auto build_candidate = std::filesystem::path("..") / "tools" / "render_layout.py";
    if (std::filesystem::exists(build_candidate)) return build_candidate;
    return source_candidate;
}

std::filesystem::path btree_renderer_script_path() {
    const auto source_candidate = std::filesystem::path("tools") / "render_btree.py";
    if (std::filesystem::exists(source_candidate)) return source_candidate;
    const auto build_candidate = std::filesystem::path("..") / "tools" / "render_btree.py";
    if (std::filesystem::exists(build_candidate)) return build_candidate;
    return source_candidate;
}

std::string python_command() {
    const std::vector<std::filesystem::path> candidates{
        std::filesystem::path("..") / ".venv_gds" / "Scripts" / "python.exe",
        std::filesystem::path("..") / ".." / ".venv_gds" / "Scripts" / "python.exe",
        std::filesystem::path(".venv_gds") / "Scripts" / "python.exe",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) return std::filesystem::absolute(candidate).string();
    }
    return "python";
}

int render_solution_png(const std::filesystem::path& input, const std::filesystem::path& output,
                        const std::filesystem::path& renderer, int dpi, const std::string& render_name) {
    std::ostringstream command;
    command << python_command() << ' ' << shell_quote(renderer)
            << " --input " << shell_quote(input)
            << " --output " << shell_quote(output)
            << " --dpi " << dpi;
    if (!render_name.empty()) command << " --name " << shell_quote(render_name);
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        std::cerr << "layout rendering failed. Ensure Python and matplotlib are available.\n";
    }
    return status;
}

int render_btree_png(const std::filesystem::path& trace_json, const std::filesystem::path& output,
                     const std::filesystem::path& renderer, int dpi, const std::string& render_name) {
    std::ostringstream command;
    command << python_command() << ' ' << shell_quote(renderer)
            << " --trace " << shell_quote(trace_json)
            << " --output " << shell_quote(output)
            << " --dpi " << dpi;
    if (!render_name.empty()) command << " --name " << shell_quote(render_name);
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        std::cerr << "B* tree rendering failed. Ensure Python and matplotlib are available.\n";
    }
    return status;
}

std::filesystem::path write_btree_trace_json(const sapr::Solution& solution, const std::filesystem::path& output) {
    if (!solution.btree_trace_json.has_value()) throw std::runtime_error("B* tree trace is not available");
    std::filesystem::create_directories(output);
    const auto trace_path = output / "btree_trace.json";
    std::ofstream trace(trace_path);
    if (!trace) throw std::runtime_error("failed to write " + trace_path.string());
    trace << *solution.btree_trace_json;
    return trace_path;
}

// 写出每次求解的 routing 诊断快照，和 PNG/routing.txt 放在同一输出目录。
std::filesystem::path write_routing_debug_json(const sapr::Solution& solution, const std::filesystem::path& output) {
    if (!solution.routing_debug_json.has_value()) throw std::runtime_error("routing debug report is not available");
    std::filesystem::create_directories(output);
    const auto debug_path = output / "routing_debug.json";
    std::ofstream debug(debug_path);
    if (!debug) throw std::runtime_error("failed to write " + debug_path.string());
    debug << *solution.routing_debug_json;
    return debug_path;
}

// 执行输入校验命令。
int run_validate(const std::vector<std::string>& args) {
    const auto circuit = sapr::load_circuit(option_value(args, "--input", "input"));
    const auto errors = sapr::validate_circuit(circuit);
    if (errors.empty()) {
        std::cout << "OK\n";
        return 0;
    }
    std::cout << "INVALID\n";
    for (const auto& error : errors) std::cout << "- " << error << '\n';
    return 1;
}

// 执行论文 placement-aware 求解并输出结果与指标。
int run_solver(const std::vector<std::string>& args, const char* executable_path) {
    const auto input = std::filesystem::path(option_value(args, "--input", "input"));
    const auto output = std::filesystem::path(option_value(args, "--output", "output"));
    sapr::SolverConfig config;
    config.spacing = option_double(args, "--spacing", config.spacing);
    config.row_width = option_double(args, "--row-width", config.row_width);
    if (has_option(args, "--boundary-margin")) {
        config.boundary_margin = option_double(args, "--boundary-margin", config.boundary_margin);
        if (config.boundary_margin < 0.0) {
            throw std::runtime_error("invalid value for --boundary-margin: must be non-negative");
        }
    }
    config.boundary_clearance =
        option_double(args, "--boundary-clearance", config.boundary_clearance);
    if (config.boundary_clearance < 0.0) {
        throw std::runtime_error("invalid value for --boundary-clearance: must be non-negative");
    }
    config.seed = option_uint(args, "--seed", config.seed);
    config.sa_iterations = option_int(args, "--sa-iterations", config.sa_iterations);
    config.initial_temperature = option_double(args, "--initial-temperature", config.initial_temperature);
    config.cooling_rate = option_double(args, "--cooling-rate", config.cooling_rate);
    config.debug_search = has_option(args, "--debug-search");
    config.dump_sa_btree = !has_option(args, "--no-dump-sa-btree");
    const bool dump_routing_eval = has_option(args, "--dump-routing-eval");
    const int render_dpi = option_int(args, "--render-dpi", 200);
    if (render_dpi <= 0) throw std::runtime_error("invalid value for --render-dpi: must be positive");
    const std::string render_name = option_value(args, "--render-name", "");
    const bool dump_btree = has_option(args, "--dump-btree");
    const std::string render_btree_name = option_value(args, "--render-btree-name", "");

    const auto circuit = sapr::load_circuit(input);
    const auto solution = sapr::solve_baseline(circuit, config);
    const auto errors = sapr::validate_solution(circuit, solution);
    if (!errors.empty()) {
        std::cerr << "invalid solution:\n";
        for (const auto& error : errors) std::cerr << "- " << error << '\n';
        return 1;
    }
    sapr::write_solution(solution, output);
    if (solution.routing_debug_json.has_value()) {
        write_routing_debug_json(solution, output);
    }
    const int render_status = render_solution_png(input, output, renderer_script_path(executable_path), render_dpi, render_name);
    if (render_status != 0) return 1;
    if (config.dump_sa_btree && !solution.sa_btree_iterations.empty()) {
        sapr::write_sa_btree_iterations(
            solution,
            input,
            output,
            btree_renderer_script_path(),
            renderer_script_path(executable_path),
            python_command(),
            render_dpi);
    }
    if (dump_btree) {
        const auto trace_path = write_btree_trace_json(solution, output);
        const int render_status =
            render_btree_png(trace_path, output, btree_renderer_script_path(), render_dpi, render_btree_name);
        if (render_status != 0) return 1;
    }
    const auto metrics = solution.metrics.value_or(sapr::measure(circuit, solution));
    std::cout << std::fixed << std::setprecision(2)
              << "{\n"
              << "  \"area\": " << metrics.area << ",\n"
              << "  \"bend_count\": " << metrics.bend_count << ",\n"
              << "  \"penalty\": " << metrics.penalty << ",\n"
              << "  \"via_count\": " << metrics.via_count << ",\n"
              << "  \"wirelength\": " << metrics.wirelength << "\n"
              << "}\n";
    if (dump_routing_eval) {
        const double routing_cost = solution.routing_cost.value_or(
            metrics.wirelength + 0.2 * static_cast<double>(metrics.bend_count) + metrics.penalty);
        std::cout << std::fixed << std::setprecision(2)
                  << "routing_evaluation:\n"
                  << "  phi_cost: " << metrics.phi_cost << '\n'
                  << "  normalized_area: " << metrics.normalized_area << '\n'
                  << "  normalized_wirelength: " << metrics.normalized_wirelength << '\n'
                  << "  normalized_bend: " << metrics.normalized_bend << '\n'
                  << "  normalized_via: " << metrics.normalized_via << '\n'
                  << "  routing_cost: " << routing_cost << '\n'
                  << "  failed_nets: " << metrics.routing_failures << '\n'
                  << "  candidate_count: " << solution.routing_candidate_count.value_or(0) << '\n'
                  << "  detailed_routes: " << solution.detailed_route_count.value_or(static_cast<std::size_t>(metrics.detailed_routes)) << '\n'
                  << "  traceback_failures: " << solution.traceback_failures.value_or(metrics.traceback_failures) << '\n'
                  << "  space_nodes_with_routes: " << solution.space_nodes_with_routes.value_or(metrics.space_nodes_with_routes) << '\n'
                  << "  dp_used: " << (solution.dp_used.value_or(metrics.dp_used) ? "true" : "false") << '\n'
                  << "  dp_nodes: " << solution.dp_nodes.value_or(metrics.dp_nodes) << '\n'
                  << "  dp_states: " << solution.dp_states.value_or(metrics.dp_states) << '\n'
                  << "  dp_pruned_states: " << solution.dp_pruned_states.value_or(metrics.dp_pruned_states) << '\n'
                  << "  dp_traceback_segments: " << solution.dp_traceback_segments.value_or(metrics.dp_traceback_segments) << '\n'
                  << "  packing_trace_steps: " << solution.packing_trace_steps.value_or(metrics.packing_trace_steps) << '\n'
                  << "  packing_time_dp_used: "
                  << (solution.packing_time_dp_used.value_or(metrics.packing_time_dp_used) ? "true" : "false") << '\n'
                  << "  packing_time_dp_segments: "
                  << solution.packing_time_dp_segments.value_or(metrics.packing_time_dp_segments) << '\n'
                  << "  space_feedback_nodes: " << solution.space_feedback_nodes.value_or(metrics.space_feedback_nodes) << '\n'
                  << "  routing_feedback_iterations: "
                  << solution.routing_feedback_iterations.value_or(metrics.routing_feedback_iterations) << '\n'
                  << "  routing_feedback_converged: "
                  << (solution.routing_feedback_converged.value_or(metrics.routing_feedback_converged) ? "true" : "false") << '\n'
                  << "  global_wirelength: " << metrics.wirelength << '\n'
                  << "  global_bends: " << metrics.bend_count << '\n'
                  << "  global_vias: " << metrics.via_count << '\n'
                  << "  global_penalty: " << metrics.penalty << '\n'
                  << "  flow_penalty: " << metrics.flow_penalty << '\n'
                  << "  current_density_penalty: " << metrics.current_density_penalty << '\n'
                  << "  coupling_penalty: " << metrics.coupling_penalty << '\n'
                  << "  design_rule_violations: " << metrics.design_rule_violations << '\n'
                  << "  design_rule_penalty: " << metrics.design_rule_penalty << '\n'
                  << "  detailed_cost: " << metrics.detailed_cost << '\n'
                  << "  detailed_routing_penalty: " << metrics.detailed_routing_penalty << '\n'
                  << "  routing_failure_penalty: " << metrics.routing_failure_penalty << '\n';
        const auto& warnings = solution.routing_warnings.empty() ? metrics.routing_warnings : solution.routing_warnings;
        if (!warnings.empty()) {
            std::cout << "  routing_warnings:\n";
            for (const auto& warning : warnings) std::cout << "    - " << warning << '\n';
        }
    }
    return 0;
}

}  // namespace

// 解析命令行并将异常转换为非零退出码。
int main(int argc, char* argv[]) {
    try {
        if (argc < 2) {
            print_usage();
            return 2;
        }
        std::vector<std::string> args(argv + 2, argv + argc);
        const std::string command = argv[1];
        if (command == "validate") return run_validate(args);
        if (command == "run") return run_solver(args, argv[0]);
        print_usage();
        return 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
