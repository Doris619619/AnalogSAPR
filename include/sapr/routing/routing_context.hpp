// 文件职责：声明由电路和 placement 构建出的布线环境汇总模型。
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "sapr/model.hpp"
#include "sapr/routing/grid.hpp"
#include "sapr/routing/obstacle.hpp"

namespace sapr::routing {

// 根据允许的金属层数构造 GridConfig；层数必须落在 1..supported_layers().size()。
GridConfig make_grid_config_for_routing_layers(int routing_layers);

// 根据布局尺寸和线宽约束返回 routing context 实际使用的网格配置。
GridConfig effective_grid_config_for_layout(
    const Circuit& circuit,
    const GridConfig& config,
    double width,
    double height);

// 表示转换到全局坐标后的 pin 信息。
struct GlobalPin {
    std::string key;
    std::string module;
    std::string pin;
    Point location;
    int layer{};
};

// 汇总规则网格、障碍物和全局 pin，供 A* 与 DP 布线查询。
class RoutingContext {
public:
    // 根据电路输入和 placement 构建布线上下文。
    RoutingContext(
        const Circuit& circuit,
        const std::unordered_map<std::string, Placement>& placements,
        const GridConfig& config = GridConfig{},
        const std::vector<Point>& extra_routing_points = {});

    // 禁止复制，避免复制内部网格所有权。
    RoutingContext(const RoutingContext&) = delete;
    // 禁止复制赋值，避免复制内部网格所有权。
    RoutingContext& operator=(const RoutingContext&) = delete;
    // 允许移动构造，便于 evaluator 按值返回。
    RoutingContext(RoutingContext&&) noexcept = default;
    // 禁止移动赋值，因为上下文持有 circuit 引用。
    RoutingContext& operator=(RoutingContext&&) noexcept = delete;

    // 返回构建好的规则网格。
    [[nodiscard]] const Grid& grid() const;
    // 返回障碍物地图。
    [[nodiscard]] const ObstacleMap& obstacles() const;
    // 返回全局 active blocker 矩形，用于 detailed routing 切分 pin access 线段。
    [[nodiscard]] const std::vector<Rect>& active_regions() const;
    // 返回所有成功转换的全局 pin。
    [[nodiscard]] const std::unordered_map<std::string, GlobalPin>& global_pins() const;
    // 返回指定 net 的默认线宽。
    [[nodiscard]] double default_width_for_net(const std::string& net) const;
    // 返回构建上下文时收集到的非致命警告。
    [[nodiscard]] const std::vector<std::string>& warnings() const;

private:
    const Circuit& circuit_;
    std::unique_ptr<Grid> grid_;
    ObstacleMap obstacles_;
    std::vector<Rect> active_regions_;
    std::unordered_map<std::string, GlobalPin> global_pins_;
    std::vector<std::string> warnings_;
};

}  // namespace sapr::routing
