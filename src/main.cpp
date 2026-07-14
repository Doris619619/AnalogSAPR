// 实现 sapr validate/run 命令行入口和稳定的指标输出。
// 文件职责：实现命令行入口，负责输入校验、求解运行、metrics/routing debug 输出。
#include <algorithm>
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
              << "           [--cooling-rate 0.96] [--routing-layers 2]\n"
              << "           [--sa-convergence-tolerance 1e-6] [--sa-convergence-patience 20]\n"
              << "           [--dump-routing-eval]\n"
              << "           [--debug-search]\n"
              << "           [--render-dpi 200] [--render-name name]\n"
              << "           [--dump-btree] [--no-dump-btree] [--render-btree-name name]\n"
              << "           [--no-dump-sa-btree] [--dump-sa-btree-json-only]\n";
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

// 定位 SA 进度 Excel 导出脚本。
std::filesystem::path sa_trace_xlsx_script_path() {
    const auto source_candidate = std::filesystem::path("tools") / "export_sa_trace_xlsx.py";
    if (std::filesystem::exists(source_candidate)) return source_candidate;
    const auto build_candidate = std::filesystem::path("..") / "tools" / "export_sa_trace_xlsx.py";
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

// 渲染最终解 B* 树图；默认叠加全部 net 拓扑弧。
int render_btree_png(const std::filesystem::path& trace_json, const std::filesystem::path& output,
                     const std::filesystem::path& renderer, int dpi, const std::string& render_name) {
    std::ostringstream command;
    command << python_command() << ' ' << shell_quote(renderer)
            << " --trace " << shell_quote(trace_json)
            << " --output " << shell_quote(output)
            << " --dpi " << dpi
            << " --show-routing";
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

// 转义 JSON 字符串字面量。
std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
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

// 将终端 `[sa]` 轻量进度写入 output/sa_trace.json，与 btree_trace.json 同级。
std::filesystem::path write_sa_trace_json(const sapr::Solution& solution, const std::filesystem::path& output) {
    std::filesystem::create_directories(output);
    const auto trace_path = output / "sa_trace.json";
    std::ofstream out(trace_path);
    if (!out) throw std::runtime_error("failed to write " + trace_path.string());
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"sa_iterations\": " << (solution.sa_progress.empty() ? 0 : solution.sa_progress.front().sa_iterations)
        << ",\n";
    out << "  \"recorded\": " << solution.sa_progress.size() << ",\n";
    out << "  \"terminated_early\": " << (solution.sa_terminated_early ? "true" : "false") << ",\n";
    out << "  \"termination_reason\": \"" << json_escape(solution.sa_termination_reason) << "\",\n";
    out << "  \"iterations\": [\n";
    for (std::size_t index = 0; index < solution.sa_progress.size(); ++index) {
        const auto& entry = solution.sa_progress[index];
        if (index > 0) out << ",\n";
        out << "    {"
            << "\"index\": " << entry.iteration
            << ", \"total\": " << entry.sa_iterations
            << ", \"move\": \"" << json_escape(entry.move) << '"'
            << ", \"changed\": " << (entry.changed ? "true" : "false")
            << ", \"accept\": " << (entry.accept ? "true" : "false")
            << ", \"next_cost\": " << entry.next_cost
            << ", \"current_cost\": " << entry.current_cost
            << ", \"best_cost\": " << entry.best_cost
            << ", \"temperature\": " << entry.temperature
            << '}';
    }
    out << "\n  ]\n";
    out << "}\n";
    return trace_path;
}

// 汇总 SA 每轮候选与最终解的 routing feedback 收敛信息，写入独立 JSON。
std::filesystem::path write_sa_feedback_converge_json(
    const sapr::Solution& solution,
    const sapr::Metrics& metrics,
    const std::filesystem::path& output) {
    std::filesystem::create_directories(output);
    const auto path = output / "sa_feedback_converge.json";
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to write " + path.string());

    int converged_count = 0;
    int not_converged_count = 0;
    int max_feedback_iterations = 0;
    for (const auto& entry : solution.sa_progress) {
        if (entry.routing_feedback_converged) {
            ++converged_count;
        } else {
            ++not_converged_count;
        }
        max_feedback_iterations =
            std::max(max_feedback_iterations, entry.routing_feedback_iterations);
    }

    const bool final_converged =
        solution.routing_feedback_converged.value_or(metrics.routing_feedback_converged);
    const int final_feedback_iterations =
        solution.routing_feedback_iterations.value_or(metrics.routing_feedback_iterations);
    const int final_space_feedback_nodes =
        solution.space_feedback_nodes.value_or(metrics.space_feedback_nodes);

    out << "{\n";
    out << "  \"sa_iterations\": "
        << (solution.sa_progress.empty() ? 0 : solution.sa_progress.front().sa_iterations) << ",\n";
    out << "  \"recorded\": " << solution.sa_progress.size() << ",\n";
    out << "  \"summary\": {\n";
    out << "    \"converged_count\": " << converged_count << ",\n";
    out << "    \"not_converged_count\": " << not_converged_count << ",\n";
    out << "    \"max_routing_feedback_iterations\": " << max_feedback_iterations << ",\n";
    out << "    \"final_routing_feedback_converged\": " << (final_converged ? "true" : "false")
        << ",\n";
    out << "    \"final_routing_feedback_iterations\": " << final_feedback_iterations << ",\n";
    out << "    \"final_space_feedback_nodes\": " << final_space_feedback_nodes << "\n";
    out << "  },\n";
    out << "  \"iterations\": [\n";
    for (std::size_t index = 0; index < solution.sa_progress.size(); ++index) {
        const auto& entry = solution.sa_progress[index];
        if (index > 0) out << ",\n";
        out << "    {"
            << "\"index\": " << entry.iteration
            << ", \"move\": \"" << json_escape(entry.move) << '"'
            << ", \"accept\": " << (entry.accept ? "true" : "false")
            << ", \"routing_feedback_iterations\": " << entry.routing_feedback_iterations
            << ", \"routing_feedback_converged\": "
            << (entry.routing_feedback_converged ? "true" : "false")
            << ", \"space_feedback_nodes\": " << entry.space_feedback_nodes
            << '}';
    }
    out << "\n  ]\n";
    out << "}\n";
    return path;
}

// 基于已写出的 sa_trace.json 再导出 Excel 表格；失败时仅告警，不影响主流程。
int export_sa_trace_xlsx(const std::filesystem::path& trace_json, const std::filesystem::path& output) {
    const auto script = sa_trace_xlsx_script_path();
    if (!std::filesystem::exists(script)) {
        std::cerr << "warning: SA Excel export script not found; skip sa_trace.xlsx\n";
        return 1;
    }
    const auto xlsx_path = output / "sa_trace.xlsx";
    std::ostringstream command;
    command << python_command() << ' ' << shell_quote(script)
            << " --trace " << shell_quote(trace_json)
            << " --output " << shell_quote(xlsx_path);
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        std::cerr << "warning: failed to export sa_trace.xlsx; JSON is still available\n";
    }
    return status;
}

// 将终端打印的基础指标与 routing_evaluation 摘要写入 output/metrics.json（与 png、btree 同级）。
// routing_cost/global_* 固定为 global 阶段；顶层与 final_penalty/detailed_* 为最终口径。
std::filesystem::path write_metrics_json(const sapr::Solution& solution, const sapr::Metrics& metrics,
                                        const std::filesystem::path& output) {
    std::filesystem::create_directories(output);
    const auto metrics_path = output / "metrics.json";
    std::ofstream out(metrics_path);
    if (!out) throw std::runtime_error("failed to write " + metrics_path.string());

    // routing_cost 仅表示 global 阶段代价；缺省时用 global_* 与 GlobalRouterConfig 默认 bend_weight=3 回退。
    const double routing_cost = solution.routing_cost.value_or(
        metrics.global_wirelength + 3.0 * static_cast<double>(metrics.global_bend_count) + metrics.global_penalty);
    const auto& warnings =
        solution.routing_warnings.empty() ? metrics.routing_warnings : solution.routing_warnings;

    out << std::fixed << std::setprecision(2)
        << "{\n"
        << "  \"area\": " << metrics.area << ",\n"
        << "  \"bend_count\": " << metrics.bend_count << ",\n"
        << "  \"penalty\": " << metrics.penalty << ",\n"
        << "  \"via_count\": " << metrics.via_count << ",\n"
        << "  \"wirelength\": " << metrics.wirelength << ",\n"
        << "  \"routing_evaluation\": {\n"
        << "    \"phi_cost\": " << metrics.phi_cost << ",\n"
        << "    \"normalized_area\": " << metrics.normalized_area << ",\n"
        << "    \"normalized_wirelength\": " << metrics.normalized_wirelength << ",\n"
        << "    \"normalized_bend\": " << metrics.normalized_bend << ",\n"
        << "    \"normalized_via\": " << metrics.normalized_via << ",\n"
        << "    \"routing_cost\": " << routing_cost << ",\n"
        << "    \"failed_nets\": " << metrics.routing_failures << ",\n"
        << "    \"candidate_count\": " << solution.routing_candidate_count.value_or(0) << ",\n"
        << "    \"detailed_routes\": "
        << solution.detailed_route_count.value_or(static_cast<std::size_t>(metrics.detailed_routes)) << ",\n"
        << "    \"traceback_failures\": "
        << solution.traceback_failures.value_or(metrics.traceback_failures) << ",\n"
        << "    \"space_nodes_with_routes\": "
        << solution.space_nodes_with_routes.value_or(metrics.space_nodes_with_routes) << ",\n"
        << "    \"dp_used\": " << (solution.dp_used.value_or(metrics.dp_used) ? "true" : "false") << ",\n"
        << "    \"dp_nodes\": " << solution.dp_nodes.value_or(metrics.dp_nodes) << ",\n"
        << "    \"dp_states\": " << solution.dp_states.value_or(metrics.dp_states) << ",\n"
        << "    \"dp_pruned_states\": "
        << solution.dp_pruned_states.value_or(metrics.dp_pruned_states) << ",\n"
        << "    \"dp_traceback_segments\": "
        << solution.dp_traceback_segments.value_or(metrics.dp_traceback_segments) << ",\n"
        << "    \"packing_trace_steps\": "
        << solution.packing_trace_steps.value_or(metrics.packing_trace_steps) << ",\n"
        << "    \"packing_time_dp_used\": "
        << (solution.packing_time_dp_used.value_or(metrics.packing_time_dp_used) ? "true" : "false") << ",\n"
        << "    \"packing_time_dp_segments\": "
        << solution.packing_time_dp_segments.value_or(metrics.packing_time_dp_segments) << ",\n"
        << "    \"space_feedback_nodes\": "
        << solution.space_feedback_nodes.value_or(metrics.space_feedback_nodes) << ",\n"
        << "    \"routing_feedback_iterations\": "
        << solution.routing_feedback_iterations.value_or(metrics.routing_feedback_iterations) << ",\n"
        << "    \"routing_feedback_converged\": "
        << (solution.routing_feedback_converged.value_or(metrics.routing_feedback_converged) ? "true" : "false")
        << ",\n"
        << "    \"global_wirelength\": " << metrics.global_wirelength << ",\n"
        << "    \"global_bends\": " << metrics.global_bend_count << ",\n"
        << "    \"global_vias\": " << metrics.global_via_count << ",\n"
        << "    \"global_penalty\": " << metrics.global_penalty << ",\n"
        << "    \"final_penalty\": " << metrics.penalty << ",\n"
        << "    \"flow_penalty\": " << metrics.flow_penalty << ",\n"
        << "    \"current_density_penalty\": " << metrics.current_density_penalty << ",\n"
        << "    \"coupling_penalty\": " << metrics.coupling_penalty << ",\n"
        << "    \"design_rule_violations\": " << metrics.design_rule_violations << ",\n"
        << "    \"design_rule_penalty\": " << metrics.design_rule_penalty << ",\n"
        << "    \"detailed_cost\": " << metrics.detailed_cost << ",\n"
        << "    \"detailed_routing_penalty\": " << metrics.detailed_routing_penalty << ",\n"
        << "    \"routing_failure_penalty\": " << metrics.routing_failure_penalty << ",\n"
        << "    \"routing_warnings\": [";
    for (std::size_t index = 0; index < warnings.size(); ++index) {
        if (index != 0) out << ", ";
        out << '"' << json_escape(warnings[index]) << '"';
    }
    out << "]\n"
        << "  }\n"
        << "}\n";
    return metrics_path;
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
    config.sa_convergence_tolerance =
        option_double(args, "--sa-convergence-tolerance", config.sa_convergence_tolerance);
    config.sa_convergence_patience =
        option_int(args, "--sa-convergence-patience", config.sa_convergence_patience);
    if (config.sa_convergence_tolerance < 0.0) {
        throw std::runtime_error("--sa-convergence-tolerance must be non-negative");
    }
    if (config.sa_convergence_patience < 0) {
        throw std::runtime_error("--sa-convergence-patience must be non-negative");
    }
    config.routing_layers = option_int(args, "--routing-layers", config.routing_layers);
    if (config.routing_layers < 1 || config.routing_layers > 7) {
        throw std::runtime_error("invalid value for --routing-layers: must be in [1, 7]");
    }
    config.debug_search = has_option(args, "--debug-search");
    // json-only 优先于 no-dump：仍收集并写出每轮文本，但跳过 PNG。
    const bool dump_sa_btree_json_only = has_option(args, "--dump-sa-btree-json-only");
    config.dump_sa_btree = dump_sa_btree_json_only || !has_option(args, "--no-dump-sa-btree");
    const bool dump_routing_eval = has_option(args, "--dump-routing-eval");
    const int render_dpi = option_int(args, "--render-dpi", 200);
    if (render_dpi <= 0) throw std::runtime_error("invalid value for --render-dpi: must be positive");
    const std::string render_name = option_value(args, "--render-name", "");
    // 最终解 btree 默认开启；--no-dump-btree 可关闭。保留 --dump-btree 兼容旧命令。
    const bool dump_btree = !has_option(args, "--no-dump-btree");
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
    if (!solution.sa_progress.empty()) {
        const auto sa_trace_path = write_sa_trace_json(solution, output);
        export_sa_trace_xlsx(sa_trace_path, output);
    }
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
            render_dpi,
            !dump_sa_btree_json_only);
    }
    if (dump_btree) {
        if (!solution.btree_trace_json.has_value()) {
            std::cerr << "warning: final B* tree trace is not available; skip btree dump\n";
        } else {
            const auto trace_path = write_btree_trace_json(solution, output);
            const int btree_status =
                render_btree_png(trace_path, output, btree_renderer_script_path(), render_dpi, render_btree_name);
            if (btree_status != 0) return 1;
        }
    }
    const auto metrics = solution.metrics.value_or(sapr::measure(circuit, solution));
    write_metrics_json(solution, metrics, output);
    write_sa_feedback_converge_json(solution, metrics, output);
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
            metrics.global_wirelength + 3.0 * static_cast<double>(metrics.global_bend_count) +
            metrics.global_penalty);
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
                  << "  global_wirelength: " << metrics.global_wirelength << '\n'
                  << "  global_bends: " << metrics.global_bend_count << '\n'
                  << "  global_vias: " << metrics.global_via_count << '\n'
                  << "  global_penalty: " << metrics.global_penalty << '\n'
                  << "  final_penalty: " << metrics.penalty << '\n'
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
