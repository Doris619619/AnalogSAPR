/* 文件职责：声明布局 placement 的统一几何变换接口。 */
#pragma once

#include <utility>

#include "sapr/model.hpp"

namespace sapr {

/* 将器件 BB 左下角局部点按 placement 变换为全局坐标。 */
std::pair<double, double> transform_placed_point(
    const Module& module,
    double local_x,
    double local_y,
    const Placement& placement);

/* 将器件引脚按 placement 变换为全局坐标。 */
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement);

/* 返回器件经过 placement 变换后的外接尺寸。 */
std::pair<double, double> placed_size(const Module& module, const Placement& placement);

}  // namespace sapr
