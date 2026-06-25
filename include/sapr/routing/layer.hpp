#pragma once

#include <string>
#include <vector>

namespace sapr::routing {

const std::vector<std::string>& supported_layers();
int layer_to_index(const std::string& layer);
std::string index_to_layer(int index);
bool is_valid_layer_index(int index);

}  // namespace sapr::routing
