// 文件职责：声明器件局部坐标到全局布局坐标的转换接口。
#pragma once

#include "sapr/model.hpp"
#include "sapr/routing/geometry.hpp"

namespace sapr::routing {

// 将器件局部坐标点按 placement 变换到全局坐标。
Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement);
// 将 pin 的局部坐标转换为全局坐标。
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement);
// 将器件 active region 转换为全局矩形。
Rect transform_active_to_global(const Module& module, const Placement& placement);
// 将器件外接矩形转换为全局矩形。
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement);

}  // namespace sapr::routing
