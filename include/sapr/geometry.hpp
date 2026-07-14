// Placement geometry transformation interface.
#pragma once

#include <utility>

#include "sapr/model.hpp"

namespace sapr {

// Transform a module BB-local point to global placement coordinates.
std::pair<double, double> transform_placed_point(
    const Module& module,
    double local_x,
    double local_y,
    const Placement& placement);

// Transform a module pin to global placement coordinates.
std::pair<double, double> placed_pin(const Module& module, const Pin& pin, const Placement& placement);

// Return the module size after placement transformation.
std::pair<double, double> placed_size(const Module& module, const Placement& placement);

}  // namespace sapr
