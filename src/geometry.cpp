/* 文件职责：实现 packing 与 routing 共用的 placement 几何变换。 */
#include "sapr/geometry.hpp"

#include <stdexcept>

namespace sapr {

/* 按完整 Cadence orient 将局部点变换为全局坐标。 */
std::pair<double, double> transform_placed_point(
    const Module& module,
    double local_x,
    double local_y,
    const Placement& placement) {
    double x = local_x;
    double y = local_y;
    if (placement.orient == "MX") {
        y = module.height - y;
    } else if (placement.orient == "MY") {
        x = module.width - x;
    } else if (placement.orient == "MXR90") {
        x = y;
        y = local_x;
    } else if (placement.orient == "MYR90") {
        x = module.height - local_y;
        y = module.width - local_x;
    } else {
        switch ((placement.angle % 360 + 360) % 360) {
            case 0: break;
            case 90: x = module.height - local_y; y = local_x; break;
            case 180: x = module.width - local_x; y = module.height - local_y; break;
            case 270: x = local_y; y = module.width - local_x; break;
            default: throw std::runtime_error("unsupported placement angle: " + std::to_string(placement.angle));
        }
    }
    return {placement.x + x, placement.y + y};
}

/* 复用局部点变换路径计算引脚全局坐标。 */
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_placed_point(module, pin.x, pin.y, placement);
}

/* 按完整 orientation 返回器件的变换后尺寸。 */
std::pair<double, double> placed_size(const Module& module, const Placement& placement) {
    if (placement.orient == "MX" || placement.orient == "MY") return {module.width, module.height};
    if (placement.orient == "MXR90" || placement.orient == "MYR90") return {module.height, module.width};
    const int angle = (placement.angle % 360 + 360) % 360;
    if (angle == 0 || angle == 180) return {module.width, module.height};
    if (angle == 90 || angle == 270) return {module.height, module.width};
    throw std::runtime_error("unsupported placement angle: " + std::to_string(placement.angle));
}

}  // namespace sapr
