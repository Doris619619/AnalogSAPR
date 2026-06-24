// 声明放置后的器件和引脚几何变换接口。
#pragma once

#include <utility>

#include "sapr/model.hpp"

namespace sapr {

// 计算引脚经过器件旋转和平移后的全局坐标。
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement);

// 返回器件旋转后的宽度和高度。
std::pair<double, double> placed_size(const Module& module, const Placement& placement);

}  // namespace sapr

