// 文件职责：声明器件局部坐标到全局布局坐标的转换函数。
#pragma once

#include "core/geometry.hpp"
#include "core/model.hpp"

namespace analog_sapr {

// 根据 placement 的 orient 将器件局部点转换到全局坐标。
Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement);

// 将 pin 的器件局部坐标转换为全局坐标。
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement);

// 将器件 active region 转换为全局坐标矩形。
Rect transform_active_to_global(const Module& module, const Placement& placement);

// 将器件 BB 转换为全局坐标矩形。
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement);

}  // namespace analog_sapr
