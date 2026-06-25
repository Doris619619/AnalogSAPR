#include "sapr/routing/layer.hpp"

#include <stdexcept>

namespace sapr::routing {

const std::vector<std::string>& supported_layers() {
    static const std::vector<std::string> layers = {"M1", "M2", "M3", "M4", "M5", "M6", "M7"};
    return layers;
}

int layer_to_index(const std::string& layer) {
    const auto& layers = supported_layers();
    for (int index = 0; index < static_cast<int>(layers.size()); ++index) {
        if (layers[index] == layer) {
            return index;
        }
    }
    throw std::runtime_error("unsupported routing layer: " + layer);
}

std::string index_to_layer(int index) {
    const auto& layers = supported_layers();
    if (!is_valid_layer_index(index)) {
        throw std::runtime_error("unsupported routing layer index: " + std::to_string(index));
    }
    return layers[index];
}

bool is_valid_layer_index(int index) {
    return index >= 0 && index < static_cast<int>(supported_layers().size());
}

}  // namespace sapr::routing
