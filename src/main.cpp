// 实现 sapr validate/run 命令行入口和稳定的指标输出。
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sapr/constraints.hpp"
#include "sapr/io.hpp"
#include "sapr/optimizer.hpp"
#include "sapr/router.hpp"
#include "sapr/routing_evaluator.hpp"

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
              << "           [--seed 1] [--sa-iterations 250] [--initial-temperature 5]\n"
              << "           [--cooling-rate 0.96] [--dump-routing-eval]\n";
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
int run_solver(const std::vector<std::string>& args) {
    const auto input = std::filesystem::path(option_value(args, "--input", "input"));
    const auto output = std::filesystem::path(option_value(args, "--output", "output"));
    sapr::SolverConfig config;
    config.spacing = option_double(args, "--spacing", config.spacing);
    config.row_width = option_double(args, "--row-width", config.row_width);
    config.seed = option_uint(args, "--seed", config.seed);
    config.sa_iterations = option_int(args, "--sa-iterations", config.sa_iterations);
    config.initial_temperature = option_double(args, "--initial-temperature", config.initial_temperature);
    config.cooling_rate = option_double(args, "--cooling-rate", config.cooling_rate);
    const bool dump_routing_eval = has_option(args, "--dump-routing-eval");

    const auto circuit = sapr::load_circuit(input);
    const auto solution = sapr::solve_baseline(circuit, config);
    const auto errors = sapr::validate_solution(circuit, solution);
    if (!errors.empty()) {
        std::cerr << "invalid solution:\n";
        for (const auto& error : errors) std::cerr << "- " << error << '\n';
        return 1;
    }
    sapr::write_solution(solution, output);
    const auto metrics = sapr::measure(circuit, solution);
    std::cout << std::fixed << std::setprecision(2)
              << "{\n"
              << "  \"area\": " << metrics.area << ",\n"
              << "  \"bend_count\": " << metrics.bend_count << ",\n"
              << "  \"penalty\": " << metrics.penalty << ",\n"
              << "  \"via_count\": " << metrics.via_count << ",\n"
              << "  \"wirelength\": " << metrics.wirelength << "\n"
              << "}\n";
    if (dump_routing_eval) {
        const auto evaluation = sapr::evaluate_routing(circuit, solution.placements);
        std::cout << std::fixed << std::setprecision(2)
                  << "routing_evaluation:\n"
                  << "  routing_cost: " << evaluation.routing_cost << '\n'
                  << "  failed_nets: " << evaluation.failed_nets << '\n'
                  << "  candidate_count: " << evaluation.candidates.size() << '\n'
                  << "  global_wirelength: " << evaluation.global_routing.total_metrics.wirelength << '\n'
                  << "  global_bends: " << evaluation.global_routing.total_metrics.bend_count << '\n'
                  << "  global_vias: " << evaluation.global_routing.total_metrics.via_count << '\n'
                  << "  global_penalty: " << evaluation.global_routing.total_penalty << '\n';
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
        if (command == "run") return run_solver(args);
        print_usage();
        return 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
