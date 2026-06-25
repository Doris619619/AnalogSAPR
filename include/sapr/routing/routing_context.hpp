#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/grid.hpp"
#include "sapr/routing/obstacle.hpp"

namespace sapr::routing {

struct GlobalPin {
    std::string key;
    std::string module;
    std::string pin;
    Point location;
    int layer{};
};

class RoutingContext {
public:
    RoutingContext(
        const Circuit& circuit,
        const std::unordered_map<std::string, Placement>& placements,
        const GridConfig& config = GridConfig{});

    RoutingContext(const RoutingContext&) = delete;
    RoutingContext& operator=(const RoutingContext&) = delete;
    RoutingContext(RoutingContext&&) noexcept = default;
    RoutingContext& operator=(RoutingContext&&) noexcept = delete;

    [[nodiscard]] const Grid& grid() const;
    [[nodiscard]] const ObstacleMap& obstacles() const;
    [[nodiscard]] const std::unordered_map<std::string, GlobalPin>& global_pins() const;
    [[nodiscard]] double default_width_for_net(const std::string& net) const;
    [[nodiscard]] const std::vector<std::string>& warnings() const;

private:
    const Circuit& circuit_;
    std::unique_ptr<Grid> grid_;
    ObstacleMap obstacles_;
    std::unordered_map<std::string, GlobalPin> global_pins_;
    std::vector<std::string> warnings_;
};

}  // namespace sapr::routing
