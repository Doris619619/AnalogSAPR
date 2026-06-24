// 实现器件旋转后的尺寸与引脚全局坐标计算。
#include "sapr/geometry.hpp"

#include <stdexcept>

namespace sapr {

// 计算引脚经过器件旋转和平移后的全局坐标。
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement) {
    double x = pin.x;
    double y = pin.y;
    switch ((placement.angle % 360 + 360) % 360) {
        case 0: break;
        case 90: x = module.height - pin.y; y = pin.x; break;
        case 180: x = module.width - pin.x; y = module.height - pin.y; break;
        case 270: x = pin.y; y = module.width - pin.x; break;
        default: throw std::runtime_error("unsupported placement angle: " + std::to_string(placement.angle));
    }
    return {placement.x + x, placement.y + y};
}

// 返回器件旋转后的宽度和高度。
std::pair<double, double> placed_size(const Module& module, const Placement& placement) {
    const int angle = (placement.angle % 360 + 360) % 360;
    if (angle == 0 || angle == 180) return {module.width, module.height};
    if (angle == 90 || angle == 270) return {module.height, module.width};
    throw std::runtime_error("unsupported placement angle: " + std::to_string(placement.angle));
}

}  // namespace sapr

