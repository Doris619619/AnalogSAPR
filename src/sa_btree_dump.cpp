// 文件职责：将 SA 每轮候选解落到 output/btree/iter_XX，并批量渲染结构图与布局图。
#include "sapr/sa_btree_dump.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "sapr/io.hpp"

namespace sapr {
namespace {

// 为 Windows 命令行安全包裹路径参数。
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

// 生成零填充的迭代文件名前缀，例如 iter_01。
std::string iteration_stem(int iteration, int total) {
    const int width = total >= 100 ? 3 : 2;
    std::ostringstream name;
    name << "iter_" << std::setw(width) << std::setfill('0') << iteration;
    return name.str();
}

// 调用 render_btree.py 输出结构图；可选择叠加所有 net routing topology。
void render_structure_png(
    const std::filesystem::path& trace_json,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& renderer_script,
    const std::string& python_command,
    const std::string& name,
    const std::string& output_basename,
    bool show_routing,
    int dpi) {
    std::ostringstream command;
    command << python_command << ' ' << shell_quote(renderer_script.string())
            << " --trace " << shell_quote(trace_json.string())
            << " --output " << shell_quote(output_dir.string())
            << " --dpi " << dpi
            << " --name " << shell_quote(name)
            << " --structure-only"
            << " --output-basename " << shell_quote(output_basename);
    if (show_routing) command << " --show-routing";
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        throw std::runtime_error("SA btree structure rendering failed for " + output_basename);
    }
}

// 调用 render_layout.py 渲染当前 SA 候选解对应的 placement/routing。
void render_layout_png(
    const std::filesystem::path& input_dir,
    const std::filesystem::path& iteration_dir,
    const std::filesystem::path& renderer_script,
    const std::string& python_command,
    const std::string& name,
    int dpi) {
    std::ostringstream command;
    command << python_command << ' ' << shell_quote(renderer_script.string())
            << " --input " << shell_quote(input_dir.string())
            << " --output " << shell_quote(iteration_dir.string())
            << " --dpi " << dpi
            << " --name " << shell_quote(name);
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        throw std::runtime_error("SA layout rendering failed for " + name);
    }
}

// 将单轮候选解转成标准 Solution，复用已有 placement/routing 文本写出逻辑。
Solution make_iteration_solution(const SaBtreeIterationTrace& iteration) {
    Solution solution;
    solution.placements = iteration.placements;
    solution.placement_order = iteration.placement_order;
    solution.routes = iteration.routes;
    return solution;
}

}  // namespace

// 写出 SA 每轮 JSON/layout 文本；可选跳过 PNG，便于长 SA 只留文本诊断。
std::size_t write_sa_btree_iterations(
    const Solution& solution,
    const std::filesystem::path& input_dir,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& btree_renderer_script,
    const std::filesystem::path& layout_renderer_script,
    const std::string& python_command,
    int dpi,
    bool render_pngs) {
    if (solution.sa_btree_iterations.empty()) return 0;
    if (render_pngs && dpi <= 0) throw std::runtime_error("invalid dpi for SA btree dump");

    const auto btree_dir = output_dir / "btree";
    std::filesystem::create_directories(btree_dir);

    const int total = static_cast<int>(solution.sa_btree_iterations.size());
    for (const auto& iteration : solution.sa_btree_iterations) {
        const auto stem = iteration_stem(iteration.iteration, total);
        const auto iteration_dir = btree_dir / stem;
        std::filesystem::create_directories(iteration_dir);

        const auto json_path = iteration_dir / (stem + ".json");
        std::ofstream out(json_path);
        if (!out) throw std::runtime_error("failed to write " + json_path.string());
        out << iteration.btree_trace_json;
        out.close();

        write_solution(make_iteration_solution(iteration), iteration_dir);

        if (!render_pngs) continue;

        render_structure_png(
            json_path,
            iteration_dir,
            btree_renderer_script,
            python_command,
            stem,
            "tree",
            false,
            dpi);
        render_structure_png(
            json_path,
            iteration_dir,
            btree_renderer_script,
            python_command,
            stem,
            "tree_with_nets",
            true,
            dpi);
        render_layout_png(
            input_dir,
            iteration_dir,
            layout_renderer_script,
            python_command,
            stem,
            dpi);
    }
    return solution.sa_btree_iterations.size();
}

}  // namespace sapr
