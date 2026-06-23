// 文件职责：声明标准 IO 文本格式的写出接口。
#pragma once

#include <filesystem>

#include "core/model.hpp"

namespace analog_sapr {

// 将布线结果写出到 output/routing.txt。
void write_routes(const RoutingSolution& solution, const std::filesystem::path& output_dir);

}  // namespace analog_sapr
