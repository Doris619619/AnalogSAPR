// 文件职责：实现金属层名称和内部层编号之间的转换。
#include "routing/layer.hpp"

#include <stdexcept>

namespace analog_sapr {

const std::vector<std::string>& supported_layers() {
    static const std::vector<std::string> layers = {"M1", "M2", "M3", "M4", "M5", "M6", "M7"};
    return layers;
}

int layer_to_index(const std::string& layer) {
    const auto& layers = supported_layers();
    for (int i = 0; i < static_cast<int>(layers.size()); ++i) {
        if (layers[i] == layer) {
            return i;
        }
    }
    throw std::runtime_error("不支持的金属层: " + layer);
}

std::string index_to_layer(int index) {
    const auto& layers = supported_layers();
    if (!is_valid_layer_index(index)) {
        throw std::runtime_error("不支持的金属层编号: " + std::to_string(index));
    }
    return layers[index];
}

bool is_valid_layer_index(int index) {
    return index >= 0 && index < static_cast<int>(supported_layers().size());
}

}  // namespace analog_sapr
