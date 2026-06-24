// 声明仓库文本格式的读取与写出接口。
#pragma once

#include <filesystem>

#include "sapr/model.hpp"

namespace sapr {

// 从输入目录读取完整电路。
Circuit load_circuit(const std::filesystem::path& input_dir);

// 从输出目录读取已有布局布线结果。
Solution load_solution(const std::filesystem::path& output_dir);

// 将布局布线结果写入标准输出文件。
void write_solution(const Solution& solution, const std::filesystem::path& output_dir);

}  // namespace sapr

