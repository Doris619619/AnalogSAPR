// 文件职责：声明金属层名称和内部层编号之间的转换函数。
#pragma once

#include <string>
#include <vector>

namespace analog_sapr {

// 返回阶段 1 默认支持的金属层名称列表。
const std::vector<std::string>& supported_layers();

// 将 M1~M7 转换为 0~6 的内部编号。
int layer_to_index(const std::string& layer);

// 将 0~6 的内部编号转换为 M1~M7。
std::string index_to_layer(int index);

// 判断层编号是否在阶段 1 支持范围内。
bool is_valid_layer_index(int index);

}  // namespace analog_sapr
