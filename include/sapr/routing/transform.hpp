#pragma once

#include "sapr/model.hpp"
#include "sapr/routing/geometry.hpp"

namespace sapr::routing {

Point transform_local_point_to_global(const Module& module, const Point& local_point, const Placement& placement);
Point transform_pin_to_global(const Module& module, const Pin& pin, const Placement& placement);
Rect transform_active_to_global(const Module& module, const Placement& placement);
Rect transform_module_bbox_to_global(const Module& module, const Placement& placement);

}  // namespace sapr::routing
