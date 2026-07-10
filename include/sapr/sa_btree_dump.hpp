// 文件职责：将 SA 每轮 btree trace 写入 output/btree 并触发结构图渲染。
#pragma once

#include <filesystem>
#include <string>

#include "sapr/model.hpp"

namespace sapr {

// 写出 SA 每轮 JSON/layout 文本；render_pngs 为 true 时再调用渲染脚本生成逐轮调试图。
// 返回成功写出的迭代数量；渲染失败时抛出异常。
std::size_t write_sa_btree_iterations(
    const Solution& solution,
    const std::filesystem::path& input_dir,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& btree_renderer_script,
    const std::filesystem::path& layout_renderer_script,
    const std::string& python_command,
    int dpi,
    bool render_pngs = true);

}  // namespace sapr
