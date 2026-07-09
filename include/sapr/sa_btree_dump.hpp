// 文件职责：将 SA 每轮 btree trace 写入 output/btree 并触发结构图渲染。
#pragma once

#include <filesystem>
#include <string>

#include "sapr/model.hpp"

namespace sapr {

// 写出 SA 每轮 JSON，并调用 render_btree.py 生成 iter_XX.png。
// 返回成功写出的迭代数量；渲染失败时抛出异常。
std::size_t write_sa_btree_iterations(
    const Solution& solution,
    const std::filesystem::path& output_dir,
    const std::filesystem::path& renderer_script,
    const std::string& python_command,
    int dpi);

}  // namespace sapr
