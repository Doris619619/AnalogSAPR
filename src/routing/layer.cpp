// 文件职责：实现布线金属层名称与内部层编号之间的转换。
#include "sapr/routing/layer.hpp"

#include <stdexcept>

namespace sapr::routing {

// 返回阶段性支持的默认金属层列表。
const std::vector<std::string>& supported_layers() {
    static const std::vector<std::string> layers = {"M1", "M2", "M3", "M4", "M5", "M6", "M7"};
    return layers;
}

// 将 M1 到 M7 的层名映射为从 0 开始的内部编号。
int layer_to_index(const std::string& layer) {
    const auto& layers = supported_layers();
    for (int index = 0; index < static_cast<int>(layers.size()); ++index) {
        if (layers[index] == layer) {
            return index;
        }
    }
    throw std::runtime_error("unsupported routing layer: " + layer);
}

// 将内部层编号映射回 M1 到 M7 的层名。
std::string index_to_layer(int index) {
    const auto& layers = supported_layers();
    if (!is_valid_layer_index(index)) {
        throw std::runtime_error("unsupported routing layer index: " + std::to_string(index));
    }
    return layers[index];
}

// 判断内部层编号是否落在支持范围内。
bool is_valid_layer_index(int index) {
    return index >= 0 && index < static_cast<int>(supported_layers().size());
}

}  // namespace sapr::routing
