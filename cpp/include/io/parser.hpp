// 文件职责：声明标准 IO 文本格式的读取接口。
#pragma once

#include <filesystem>

#include "core/model.hpp"

namespace analog_sapr {

// 从 input 目录读取 modules、pins、nets 和 constraints 文件。
Circuit load_circuit(const std::filesystem::path& input_dir);

// 从 placement.txt 读取布局结果，作为布局模块和布线模块的阶段 0 接口。
std::unordered_map<std::string, Placement> load_placements(const std::filesystem::path& placement_path);

}  // namespace analog_sapr
