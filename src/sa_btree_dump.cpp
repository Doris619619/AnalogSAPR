// 文件职责：将 SA 每轮 btree JSON 落到 output/btree，并批量渲染结构图。
#include "sapr/sa_btree_dump.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

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

// 调用 render_btree.py 只输出结构图，文件名为 stem.png。
void render_structure_png(
    const std::filesystem::path& trace_json,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& renderer_script,
    const std::string& python_command,
    const std::string& name,
    int dpi) {
    std::ostringstream command;
    command << python_command << ' ' << shell_quote(renderer_script.string())
            << " --trace " << shell_quote(trace_json.string())
            << " --output " << shell_quote(output_dir.string())
            << " --dpi " << dpi
            << " --name " << shell_quote(name)
            << " --structure-only"
            << " --output-basename " << shell_quote(name);
    const int status = std::system(command.str().c_str());
    if (status != 0) {
        throw std::runtime_error("SA btree structure rendering failed for " + name);
    }
}

}  // namespace

// 写出 SA 每轮 JSON，并调用 render_btree.py 生成 iter_XX.png。
std::size_t write_sa_btree_iterations(
    const Solution& solution,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& renderer_script,
    const std::string& python_command,
    int dpi) {
    if (solution.sa_btree_iterations.empty()) return 0;
    if (dpi <= 0) throw std::runtime_error("invalid dpi for SA btree dump");

    const auto btree_dir = output_dir / "btree";
    std::filesystem::create_directories(btree_dir);

    const int total = static_cast<int>(solution.sa_btree_iterations.size());
    for (const auto& iteration : solution.sa_btree_iterations) {
        const auto stem = iteration_stem(iteration.iteration, total);
        const auto json_path = btree_dir / (stem + ".json");
        std::ofstream out(json_path);
        if (!out) throw std::runtime_error("failed to write " + json_path.string());
        out << iteration.btree_trace_json;
        out.close();

        render_structure_png(
            json_path,
            btree_dir,
            renderer_script,
            python_command,
            stem,
            dpi);
    }
    return solution.sa_btree_iterations.size();
}

}  // namespace sapr
