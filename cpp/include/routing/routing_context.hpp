// 文件职责：声明阶段 1 的布线环境汇总模型。
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/geometry.hpp"
#include "core/model.hpp"
#include "routing/grid.hpp"
#include "routing/obstacle.hpp"

namespace analog_sapr {

// 表示转换到全局坐标后的 pin。
struct GlobalPin {
    std::string key;
    std::string module;
    std::string pin;
    Point location;
    int layer = 0;
};

// 汇总 Circuit 和 Placement 生成的阶段 1 布线环境。
class RoutingContext {
public:
    // 根据电路输入和布局结果构建网格、障碍物和全局 pin。
    RoutingContext(
        const Circuit& circuit,
        const std::unordered_map<std::string, Placement>& placements,
        const GridConfig& config = GridConfig{});

    // 返回构建好的规则网格。
    const Grid& grid() const;

    // 返回障碍物地图。
    const ObstacleMap& obstacles() const;

    // 返回所有成功转换到全局坐标的 pin。
    const std::unordered_map<std::string, GlobalPin>& global_pins() const;

    // 返回指定 net 的默认线宽，优先使用 WIRE_WIDTH 的 min_width。
    double default_width_for_net(const std::string& net) const;

    // 返回构建过程中遇到但未中断流程的警告。
    const std::vector<std::string>& warnings() const;

private:
    const Circuit& circuit_;
    std::unique_ptr<Grid> grid_;
    ObstacleMap obstacles_;
    std::unordered_map<std::string, GlobalPin> global_pins_;
    std::vector<std::string> warnings_;
};

}  // namespace analog_sapr
