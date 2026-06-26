// 文件职责：声明金属层名称与内部层编号之间的转换接口。
#pragma once

#include <string>
#include <vector>

namespace sapr::routing {

// 返回当前布线模型支持的金属层名称列表。
const std::vector<std::string>& supported_layers();
// 将金属层名称转换为内部层编号。
int layer_to_index(const std::string& layer);
// 将内部层编号转换为金属层名称。
std::string index_to_layer(int index);
// 判断内部层编号是否有效。
bool is_valid_layer_index(int index);

}  // namespace sapr::routing
