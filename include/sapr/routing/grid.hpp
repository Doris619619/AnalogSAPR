#pragma once

#include <vector>

#include "sapr/routing/geometry.hpp"

namespace sapr::routing {

struct GridConfig {
    double step{1.0};
    double margin{5.0};
    int layer_count{7};
};

struct GridPoint {
    int ix{};
    int iy{};
    int layer{};
};

class Grid {
public:
    Grid(const GridConfig& config, double min_x, double min_y, double max_x, double max_y);

    [[nodiscard]] GridPoint snap_to_grid(const Point& point, int layer = 0) const;
    [[nodiscard]] Point grid_to_point(const GridPoint& point) const;
    [[nodiscard]] bool in_bounds(const GridPoint& point) const;
    [[nodiscard]] std::vector<GridPoint> planar_neighbors(const GridPoint& point) const;
    [[nodiscard]] std::vector<GridPoint> via_neighbors(const GridPoint& point) const;
    [[nodiscard]] std::vector<GridPoint> neighbors(const GridPoint& point) const;

    [[nodiscard]] double step() const;
    [[nodiscard]] double min_x() const;
    [[nodiscard]] double min_y() const;
    [[nodiscard]] double max_x() const;
    [[nodiscard]] double max_y() const;
    [[nodiscard]] int layer_count() const;
    [[nodiscard]] int x_count() const;
    [[nodiscard]] int y_count() const;

private:
    GridConfig config_;
    double min_x_{};
    double min_y_{};
    double max_x_{};
    double max_y_{};
    int x_count_{};
    int y_count_{};
};

}  // namespace sapr::routing
