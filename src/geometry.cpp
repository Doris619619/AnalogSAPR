// Placement geometry transformations shared by packing and routing.
#include "sapr/geometry.hpp"

#include <stdexcept>

namespace sapr {

// Transform a local point with the full Cadence orientation.
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

// Transform a pin through the shared local point path.
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement) {
    return transform_placed_point(module, pin.x, pin.y, placement);
}

// Return the transformed module size for the full orientation.
std::pair<double, double> placed_size(const Module& module, const Placement& placement) {
    if (placement.orient == "MX" || placement.orient == "MY") return {module.width, module.height};
    if (placement.orient == "MXR90" || placement.orient == "MYR90") return {module.height, module.width};
    const int angle = (placement.angle % 360 + 360) % 360;
    if (angle == 0 || angle == 180) return {module.width, module.height};
    if (angle == 90 || angle == 270) return {module.height, module.width};
    throw std::runtime_error("unsupported placement angle: " + std::to_string(placement.angle));
}

}  // namespace sapr
