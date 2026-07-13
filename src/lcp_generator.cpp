// 文件职责：实现 placement 后的自动 LCP 拓扑、space 绑定和物理候选点生成。
#include "sapr/lcp_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sapr/geometry.hpp"
#include "sapr/routing/layer.hpp"

namespace sapr {
namespace {

constexpr double kDefaultWidth = 1.0;
constexpr double kGridUnit = 1.0;
constexpr int kMaxPinsPerLeafLcp = 3;
constexpr std::size_t kMaxCandidatesPerLcp = 16;
constexpr std::size_t kMinCandidatesPerLcp = 3;
constexpr std::size_t kRandomSpaceCandidatesPerLcp = 1;

enum class NetDirection {
    Neutral,
    Horizontal,
    Vertical,
};

struct PinInfo {
    std::string terminal;
    std::string module;
    double x{};
    double y{};
    int layer{};
};

struct Cluster {
    std::vector<int> pins;
};

struct WorkingLcp {
    LinkingControlPoint point;
    std::string net;
    double required_width{kDefaultWidth};
    double ideal_x{};
    double ideal_y{};
    Rect support_bbox{};
    NetDirection direction{NetDirection::Neutral};
    unsigned int candidate_seed{};
};

struct SpaceBindingState {
    double used_width{};
    int used_lcp_count{};
};

// 查找 net 对应的 FLOW 约束，供自动 LCP 拓扑保持电流方向。
std::optional<FlowConstraint> flow_constraint_for_net(const Circuit& circuit, const std::string& net) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net == net) return flow;
    }
    return std::nullopt;
}

// 返回 net 的线宽范围；有 WIRE_WIDTH 时直接用约束 min/max，不再用 kDefaultWidth 抬高 min。
// 否则 LCP A* 会按 1um 膨胀 active，而 pin access corridor 仍按细网格短 escape，导致 pin 出不去。
std::pair<double, double> width_range_for_net(const Circuit& circuit, const std::string& net) {
    const auto found = circuit.constraints.wire_widths.find(net);
    if (found == circuit.constraints.wire_widths.end()) return {kDefaultWidth, kDefaultWidth};
    double min_width = found->second.min_width;
    double max_width = found->second.max_width;
    if (min_width <= 0.0) min_width = kDefaultWidth;
    if (max_width > 0.0 && min_width > max_width) min_width = max_width;
    if (max_width <= 0.0) max_width = min_width;
    return {min_width, max_width};
}

// 返回 FLOW 约束的文本方向，供后续 routing 检查沿用。
std::optional<std::string> flow_direction_for_net(const Circuit& circuit, const std::string& net) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net == net) return flow.out_pin + "->" + flow.in_pin;
    }
    return std::nullopt;
}

// 返回 endpoint 在 FLOW 约束中的电流角色。
CurrentDirection current_direction_for_endpoint(const Circuit& circuit, const std::string& net, const std::string& endpoint) {
    for (const auto& flow : circuit.constraints.flows) {
        if (flow.net != net) continue;
        if (flow.out_pin == endpoint) return CurrentDirection::Out;
        if (flow.in_pin == endpoint) return CurrentDirection::In;
    }
    return CurrentDirection::Unknown;
}

// 计算 pin 集合的中位坐标，用于 leaf/root LCP 的理想点。
double median_value(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

// 将点纳入 bbox；首个点会初始化 bbox。
void include_point(Rect& box, bool& has_box, double x, double y) {
    if (!has_box) {
        box = {x, y, x, y};
        has_box = true;
        return;
    }
    box.x1 = std::min(box.x1, x);
    box.y1 = std::min(box.y1, y);
    box.x2 = std::max(box.x2, x);
    box.y2 = std::max(box.y2, y);
}

// 将矩形纳入 bbox。
void include_rect(Rect& box, bool& has_box, Rect rect) {
    include_point(box, has_box, rect.x1, rect.y1);
    include_point(box, has_box, rect.x2, rect.y2);
}

// 估计 net 的几何方向；全重合时保持 neutral。
NetDirection direction_for_pins(const std::vector<PinInfo>& pins) {
    if (pins.empty()) return NetDirection::Neutral;
    double min_x = pins.front().x;
    double max_x = pins.front().x;
    double min_y = pins.front().y;
    double max_y = pins.front().y;
    for (const auto& pin : pins) {
        min_x = std::min(min_x, pin.x);
        max_x = std::max(max_x, pin.x);
        min_y = std::min(min_y, pin.y);
        max_y = std::max(max_y, pin.y);
    }
    const double spread_x = max_x - min_x;
    const double spread_y = max_y - min_y;
    if (spread_x < kGridUnit && spread_y < kGridUnit) return NetDirection::Neutral;
    if (spread_x >= 1.5 * std::max(kGridUnit, spread_y)) return NetDirection::Horizontal;
    if (spread_y >= 1.5 * std::max(kGridUnit, spread_x)) return NetDirection::Vertical;
    return NetDirection::Neutral;
}

// 计算目标 leaf LCP 数量；root 不计入该数量。
int target_leaf_count(const std::vector<PinInfo>& pins, double required_width) {
    const int pin_count = static_cast<int>(pins.size());
    if (pin_count <= 2) return 0;
    if (pin_count == 3) return 1;
    double min_x = pins.front().x;
    double max_x = pins.front().x;
    double min_y = pins.front().y;
    double max_y = pins.front().y;
    for (const auto& pin : pins) {
        min_x = std::min(min_x, pin.x);
        max_x = std::max(max_x, pin.x);
        min_y = std::min(min_y, pin.y);
        max_y = std::max(max_y, pin.y);
    }
    const double spread_x = max_x - min_x;
    const double spread_y = max_y - min_y;
    int geometry_bonus = 0;
    if (!(spread_x < kGridUnit && spread_y < kGridUnit)) {
        const double minor = std::max(kGridUnit, std::min(spread_x, spread_y));
        const double aspect = std::max(spread_x, spread_y) / minor;
        geometry_bonus = aspect >= 3.0 ? 1 : 0;
    }
    const int width_bonus = required_width >= 3.0 * kDefaultWidth ? 1 : 0;
    const int base = (pin_count + kMaxPinsPerLeafLcp - 1) / kMaxPinsPerLeafLcp;
    return std::clamp(base + geometry_bonus + width_bonus, 1, pin_count - 1);
}

// 计算稳定的 Manhattan/layer 混合距离。
double pin_distance(const PinInfo& a, const PinInfo& b, double layer_penalty) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + layer_penalty * std::abs(a.layer - b.layer);
}

// 返回 pairwise Manhattan 距离中位数，用来归一化 layer penalty。
double median_pairwise_distance(const std::vector<PinInfo>& pins) {
    std::vector<double> distances;
    for (std::size_t i = 0; i < pins.size(); ++i) {
        for (std::size_t j = i + 1; j < pins.size(); ++j) {
            distances.push_back(std::abs(pins[i].x - pins[j].x) + std::abs(pins[i].y - pins[j].y));
        }
    }
    return median_value(std::move(distances));
}

// 判断所有 pin 是否在相同几何/layer 位置，避免 MST tie-break 影响稳定性。
bool all_pairwise_distance_zero(const std::vector<PinInfo>& pins, double layer_penalty) {
    for (std::size_t i = 0; i < pins.size(); ++i) {
        for (std::size_t j = i + 1; j < pins.size(); ++j) {
            if (pin_distance(pins[i], pins[j], layer_penalty) > 1e-9) return false;
        }
    }
    return true;
}

// 按 terminal 原始顺序均匀分组；前面的 cluster 接收余数。
std::vector<Cluster> balanced_order_clusters(int pin_count, int cluster_count) {
    std::vector<Cluster> clusters;
    clusters.reserve(static_cast<std::size_t>(cluster_count));
    int cursor = 0;
    for (int index = 0; index < cluster_count; ++index) {
        const int remaining_pins = pin_count - cursor;
        const int remaining_clusters = cluster_count - index;
        const int size = (remaining_pins + remaining_clusters - 1) / remaining_clusters;
        Cluster cluster;
        for (int offset = 0; offset < size; ++offset) cluster.pins.push_back(cursor++);
        clusters.push_back(std::move(cluster));
    }
    return clusters;
}

// 用 Prim MST 删除最长边得到目标数量的 cluster。
std::vector<Cluster> mst_clusters(const std::vector<PinInfo>& pins, int cluster_count, double layer_penalty) {
    const int count = static_cast<int>(pins.size());
    if (cluster_count <= 1) {
        Cluster cluster;
        for (int index = 0; index < count; ++index) cluster.pins.push_back(index);
        return {std::move(cluster)};
    }
    if (all_pairwise_distance_zero(pins, layer_penalty)) return balanced_order_clusters(count, cluster_count);

    std::vector<double> best(count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(count, -1);
    std::vector<bool> used(count, false);
    best[0] = 0.0;
    for (int step = 0; step < count; ++step) {
        int current = -1;
        for (int index = 0; index < count; ++index) {
            if (used[index]) continue;
            if (current == -1 || best[index] < best[current] ||
                (std::abs(best[index] - best[current]) <= 1e-9 && pins[index].terminal < pins[current].terminal)) {
                current = index;
            }
        }
        used[current] = true;
        for (int next = 0; next < count; ++next) {
            if (used[next] || next == current) continue;
            const double distance = pin_distance(pins[current], pins[next], layer_penalty);
            if (distance < best[next] ||
                (std::abs(distance - best[next]) <= 1e-9 && pins[current].terminal < pins[parent[next]].terminal)) {
                best[next] = distance;
                parent[next] = current;
            }
        }
    }

    struct Edge {
        int a{};
        int b{};
        double distance{};
    };
    std::vector<Edge> edges;
    for (int index = 1; index < count; ++index) edges.push_back({index, parent[index], best[index]});
    std::sort(edges.begin(), edges.end(), [&](const Edge& lhs, const Edge& rhs) {
        if (std::abs(lhs.distance - rhs.distance) > 1e-9) return lhs.distance > rhs.distance;
        return pins[lhs.a].terminal + pins[lhs.b].terminal < pins[rhs.a].terminal + pins[rhs.b].terminal;
    });
    std::unordered_set<std::string> removed;
    for (int index = 0; index < cluster_count - 1 && index < static_cast<int>(edges.size()); ++index) {
        removed.insert(std::to_string(std::min(edges[index].a, edges[index].b)) + ":" +
                       std::to_string(std::max(edges[index].a, edges[index].b)));
    }

    std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(count));
    for (const auto& edge : edges) {
        const std::string key = std::to_string(std::min(edge.a, edge.b)) + ":" + std::to_string(std::max(edge.a, edge.b));
        if (removed.contains(key)) continue;
        adjacency[edge.a].push_back(edge.b);
        adjacency[edge.b].push_back(edge.a);
    }
    std::vector<Cluster> clusters;
    std::vector<bool> seen(static_cast<std::size_t>(count), false);
    for (int start = 0; start < count; ++start) {
        if (seen[start]) continue;
        Cluster cluster;
        std::vector<int> stack{start};
        seen[start] = true;
        while (!stack.empty()) {
            const int current = stack.back();
            stack.pop_back();
            cluster.pins.push_back(current);
            for (int next : adjacency[current]) {
                if (seen[next]) continue;
                seen[next] = true;
                stack.push_back(next);
            }
        }
        std::sort(cluster.pins.begin(), cluster.pins.end());
        clusters.push_back(std::move(cluster));
    }
    std::sort(clusters.begin(), clusters.end(), [](const Cluster& lhs, const Cluster& rhs) {
        return lhs.pins.front() < rhs.pins.front();
    });
    return clusters;
}

// 对过大的 cluster 继续稳定切分，保证 leaf cluster 不超过上限。
std::vector<Cluster> repair_large_clusters(const std::vector<Cluster>& clusters, int pin_count) {
    std::vector<Cluster> repaired;
    const int max_leaf_count = std::max(1, pin_count - 1);
    for (const auto& cluster : clusters) {
        if (static_cast<int>(cluster.pins.size()) <= kMaxPinsPerLeafLcp || static_cast<int>(repaired.size()) >= max_leaf_count) {
            repaired.push_back(cluster);
            continue;
        }
        const int remaining_slots = max_leaf_count - static_cast<int>(repaired.size());
        const int needed = static_cast<int>((cluster.pins.size() + kMaxPinsPerLeafLcp - 1) / kMaxPinsPerLeafLcp);
        const int split_count = std::max(1, std::min(needed, remaining_slots));
        int cursor = 0;
        for (int index = 0; index < split_count; ++index) {
            const int remaining_pins = static_cast<int>(cluster.pins.size()) - cursor;
            const int remaining_clusters = split_count - index;
            const int size = (remaining_pins + remaining_clusters - 1) / remaining_clusters;
            Cluster split;
            for (int offset = 0; offset < size; ++offset) split.pins.push_back(cluster.pins[static_cast<std::size_t>(cursor++)]);
            repaired.push_back(std::move(split));
        }
    }
    return repaired;
}

// 生成一条拓扑 segment。
WireSegmentRef make_segment(
    const Circuit& circuit,
    const std::string& net,
    const std::string& from,
    const std::string& to,
    double min_width,
    double max_width) {
    return {net,
            from,
            to,
            min_width,
            max_width,
            flow_direction_for_net(circuit, net),
            current_direction_for_endpoint(circuit, net, from),
            net + ":" + from + "->" + to};
}

// 返回 module 的已放置 bbox。
Rect module_box_for_pin(
    const Circuit& circuit,
    const std::unordered_map<std::string, Placement>& placements,
    const std::string& module) {
    const auto& placement = placements.at(module);
    const auto size = placed_size(circuit.modules.at(module), placement);
    return {placement.x, placement.y, placement.x + size.first, placement.y + size.second};
}

// 构建一个自动 leaf/root LCP 的候选上下文 bbox。
Rect support_bbox_for_pins(
    const Circuit& circuit,
    const RoutingEvaluationRequest& request,
    const std::vector<PinInfo>& pins,
    const std::vector<int>& indices) {
    Rect box{};
    bool has_box = false;
    for (int index : indices) {
        include_point(box, has_box, pins[static_cast<std::size_t>(index)].x, pins[static_cast<std::size_t>(index)].y);
        include_rect(box, has_box, module_box_for_pin(circuit, request.placements, pins[static_cast<std::size_t>(index)].module));
    }
    return box;
}

// 计算 cluster pin 的中位理想点。
std::pair<double, double> cluster_ideal(const std::vector<PinInfo>& pins, const std::vector<int>& indices) {
    std::vector<double> xs;
    std::vector<double> ys;
    for (int index : indices) {
        xs.push_back(pins[static_cast<std::size_t>(index)].x);
        ys.push_back(pins[static_cast<std::size_t>(index)].y);
    }
    return {median_value(std::move(xs)), median_value(std::move(ys))};
}

// 返回 space 的几何锚点。
std::pair<double, double> space_anchor(
    const SpaceNode& space,
    const std::unordered_map<std::string, Rect>& module_boxes) {
    if (space.physical_region.has_value()) {
        const Rect region = *space.physical_region;
        return {(region.x1 + region.x2) / 2.0, (region.y1 + region.y2) / 2.0};
    }
    const auto found = module_boxes.find(space.owner);
    if (found == module_boxes.end()) return {0.0, 0.0};
    const Rect box = found->second;
    const double cx = (box.x1 + box.x2) / 2.0;
    const double cy = (box.y1 + box.y2) / 2.0;
    if (space.kind == SpaceNodeKind::Top) return {cx, box.y2};
    if (space.kind == SpaceNodeKind::Right) return {box.x2, cy};
    return {cx, cy};
}

// 计算 space 绑定分数。
double space_score(
    const SpaceNode& space,
    const WorkingLcp& lcp,
    const SpaceBindingState& state,
    const std::unordered_map<std::string, Rect>& module_boxes) {
    const auto anchor = space_anchor(space, module_boxes);
    const double distance = std::abs(anchor.first - lcp.ideal_x) + std::abs(anchor.second - lcp.ideal_y);
    const double capacity_width = std::max(space.required_space(), kGridUnit);
    const double remaining_width = std::max(0.0, capacity_width - state.used_width);
    const double width_penalty = remaining_width < lcp.required_width ? 10000.0 : 0.0;
    double direction_penalty = 0.0;
    if (lcp.direction == NetDirection::Horizontal && space.kind == SpaceNodeKind::Top) direction_penalty = 50.0;
    if (lcp.direction == NetDirection::Vertical && space.kind != SpaceNodeKind::Top) direction_penalty = 50.0;
    return distance + static_cast<double>(state.used_lcp_count) * lcp.required_width + width_penalty + direction_penalty;
}

// 将 LCP 稳定绑定到分数最低的 space node。
void bind_lcps_to_spaces(
    std::vector<WorkingLcp>& lcps,
    std::vector<SpaceNode>& spaces,
    const std::unordered_map<std::string, Rect>& module_boxes) {
    std::sort(lcps.begin(), lcps.end(), [](const WorkingLcp& lhs, const WorkingLcp& rhs) {
        if (std::abs(lhs.required_width - rhs.required_width) > 1e-9) return lhs.required_width > rhs.required_width;
        if (lhs.net != rhs.net) return lhs.net < rhs.net;
        return lhs.point.id < rhs.point.id;
    });
    std::unordered_map<std::string, SpaceBindingState> states;
    for (auto& lcp : lcps) {
        SpaceNode* best_space = nullptr;
        double best_score = std::numeric_limits<double>::infinity();
        for (auto& space : spaces) {
            const double score = space_score(space, lcp, states[space.id], module_boxes);
            if (score < best_score || (std::abs(score - best_score) <= 1e-9 && best_space != nullptr && space.id < best_space->id)) {
                best_score = score;
                best_space = &space;
            }
        }
        if (best_space == nullptr) continue;
        lcp.point.space_node_id = best_space->id;
        auto& state = states[best_space->id];
        state.used_width += lcp.required_width;
        ++state.used_lcp_count;
    }
}

// 固定坐标去重 key，避免 round 策略随平台漂移。
std::string candidate_key(double x, double y, const std::string& validity) {
    const auto gx = static_cast<long long>(std::floor(x / kGridUnit + 0.5));
    const auto gy = static_cast<long long>(std::floor(y / kGridUnit + 0.5));
    return std::to_string(gx) + ":" + std::to_string(gy) + ":" + validity;
}

// 判断点是否落在矩形区域内。
bool point_inside_rect(double x, double y, const Rect& rect) {
    return x >= rect.x1 - 1e-9 && x <= rect.x2 + 1e-9 && y >= rect.y1 - 1e-9 && y <= rect.y2 + 1e-9;
}

// 返回两个矩形的交集；没有交集时返回空值。
std::optional<Rect> intersect_rect(Rect lhs, Rect rhs) {
    Rect result{std::max(lhs.x1, rhs.x1), std::max(lhs.y1, rhs.y1), std::min(lhs.x2, rhs.x2), std::min(lhs.y2, rhs.y2)};
    if (result.x2 < result.x1 || result.y2 < result.y1) return std::nullopt;
    return result;
}

// 将坐标限制到指定矩形中。
std::pair<double, double> clamp_to_rect(double x, double y, Rect rect) {
    return {std::clamp(x, rect.x1, rect.x2), std::clamp(y, rect.y1, rect.y2)};
}

// 按全局 seed、LCP id 和 space id 派生稳定随机种子。
std::uint32_t stable_candidate_seed(const WorkingLcp& lcp) {
    std::uint32_t seed = lcp.candidate_seed == 0 ? 1U : lcp.candidate_seed;
    auto mix = [&](const std::string& value) {
        for (unsigned char ch : value) {
            seed ^= static_cast<std::uint32_t>(ch) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        }
    };
    mix(lcp.point.id);
    mix(lcp.point.space_node_id);
    return seed;
}

// 加入一个候选点并按 grid key 去重。
void add_candidate(
    std::vector<PhysicalLocationCandidate>& candidates,
    std::unordered_set<std::string>& seen,
    double x,
    double y,
    const std::string& id,
    const std::string& validity,
    bool fallback,
    double penalty,
    const std::string& reason,
    const std::string& source,
    bool inside_space_region) {
    const std::string key = candidate_key(x, y, validity);
    if (!seen.insert(key).second) return;
    candidates.push_back({x, y, id, validity, fallback, penalty, reason, source, inside_space_region});
}

// 判断候选点按所需线宽扩张后是否碰到 active blocker。
bool candidate_hits_blocker(double x, double y, double width, const std::vector<Rect>& blockers) {
    const double half = std::max(width, kGridUnit) / 2.0;
    const Rect candidate{x - half, y - half, x + half, y + half};
    for (const auto& blocker : blockers) {
        if (candidate.x1 < blocker.x2 && candidate.x2 > blocker.x1 &&
            candidate.y1 < blocker.y2 && candidate.y2 > blocker.y1) {
            return true;
        }
    }
    return false;
}

// 加入候选点前做基础合法性分类，避免把明显落在 active 区内的点标为 strict。
void add_classified_candidate(
    std::vector<PhysicalLocationCandidate>& candidates,
    std::unordered_set<std::string>& seen,
    const WorkingLcp& lcp,
    const std::optional<Rect>& space_region,
    const std::vector<Rect>& blockers,
    double x,
    double y,
    const std::string& id,
    const std::string& validity,
    bool fallback,
    double penalty,
    const std::string& reason,
    const std::string& source) {
    const bool inside_space = !space_region.has_value() || point_inside_rect(x, y, *space_region);
    std::string final_validity = validity;
    bool final_fallback = fallback;
    double final_penalty = penalty;
    std::string final_reason = reason;
    if (!inside_space) {
        final_penalty += 2000.0;
        final_reason += "_outside_space";
        if (validity == "strict") {
            final_validity = "relaxed";
        } else {
            final_validity = "emergency";
            final_fallback = true;
            final_penalty += 8000.0;
        }
    }
    const bool blocked = candidate_hits_blocker(x, y, lcp.required_width, blockers);
    add_candidate(
        candidates,
        seen,
        x,
        y,
        id,
        blocked ? "emergency" : final_validity,
        final_fallback || blocked,
        final_penalty + (blocked ? 10000.0 : 0.0),
        blocked ? final_reason + "_active_blocker" : final_reason,
        source,
        inside_space);
}

// 为单个 LCP 生成 strict/relaxed/emergency 候选。
// 在所属 space region 内生成优先候选和确定性随机采样点。
void add_space_region_candidates(
    std::vector<PhysicalLocationCandidate>& candidates,
    std::unordered_set<std::string>& seen,
    const WorkingLcp& lcp,
    const std::optional<Rect>& space_region,
    const std::vector<Rect>& blockers) {
    if (!space_region.has_value()) return;
    const Rect region = *space_region;
    const Rect expanded_support{
        lcp.support_bbox.x1 - kGridUnit,
        lcp.support_bbox.y1 - kGridUnit,
        lcp.support_bbox.x2 + kGridUnit,
        lcp.support_bbox.y2 + kGridUnit,
    };
    const Rect sample = intersect_rect(region, expanded_support).value_or(region);
    const auto [ideal_x, ideal_y] = clamp_to_rect(lcp.ideal_x, lcp.ideal_y, region);
    const std::vector<std::pair<std::string, std::pair<double, double>>> anchors{
        {"space_clamped_ideal", {ideal_x, ideal_y}},
        {"space_center", {(region.x1 + region.x2) / 2.0, (region.y1 + region.y2) / 2.0}},
        {"space_sample_center", {(sample.x1 + sample.x2) / 2.0, (sample.y1 + sample.y2) / 2.0}},
    };
    for (const auto& [name, point] : anchors) {
        add_classified_candidate(
            candidates,
            seen,
            lcp,
            space_region,
            blockers,
            point.first,
            point.second,
            lcp.point.id + ":" + name,
            "strict",
            false,
            0.0,
            name,
            "space_grid");
    }
    const std::vector<std::pair<std::string, std::pair<double, double>>> boundary_anchors{
        {"space_left_mid", {region.x1, (region.y1 + region.y2) / 2.0}},
        {"space_right_mid", {region.x2, (region.y1 + region.y2) / 2.0}},
        {"space_bottom_mid", {(region.x1 + region.x2) / 2.0, region.y1}},
        {"space_top_mid", {(region.x1 + region.x2) / 2.0, region.y2}},
        {"space_lower_left", {region.x1, region.y1}},
        {"space_upper_right", {region.x2, region.y2}},
    };
    for (const auto& [name, point] : boundary_anchors) {
        add_classified_candidate(
            candidates,
            seen,
            lcp,
            space_region,
            blockers,
            point.first,
            point.second,
            lcp.point.id + ":" + name,
            "relaxed",
            false,
            15.0,
            name,
            "space_boundary");
    }

    std::mt19937 rng(stable_candidate_seed(lcp));
    std::uniform_real_distribution<double> x_dist(sample.x1, sample.x2);
    std::uniform_real_distribution<double> y_dist(sample.y1, sample.y2);
    for (std::size_t index = 0; index < kRandomSpaceCandidatesPerLcp; ++index) {
        add_classified_candidate(
            candidates,
            seen,
            lcp,
            space_region,
            blockers,
            x_dist(rng),
            y_dist(rng),
            lcp.point.id + ":space_random:" + std::to_string(index),
            "strict",
            false,
            5.0 + static_cast<double>(index) * 0.1,
            "space_random",
            "space_random");
    }
}

void generate_candidates_for_lcp(
    WorkingLcp& lcp,
    const std::vector<SpaceNode>& spaces,
    const std::unordered_map<std::string, Rect>& module_boxes,
    const std::vector<Rect>& blockers) {
    std::vector<PhysicalLocationCandidate> candidates;
    std::unordered_set<std::string> seen;
    const Rect box = lcp.support_bbox;
    const double cx = (box.x1 + box.x2) / 2.0;
    const double cy = (box.y1 + box.y2) / 2.0;
    const double offset = std::max(lcp.required_width, kGridUnit);
    const auto space_it = std::find_if(spaces.begin(), spaces.end(), [&](const SpaceNode& space) {
        return space.id == lcp.point.space_node_id;
    });
    const std::optional<Rect> space_region = space_it == spaces.end() ? std::nullopt : space_it->physical_region;
    std::pair<double, double> anchor{lcp.ideal_x, lcp.ideal_y};
    if (space_it != spaces.end()) anchor = space_anchor(*space_it, module_boxes);

    add_space_region_candidates(candidates, seen, lcp, space_region, blockers);
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, lcp.ideal_x, lcp.ideal_y, lcp.point.id + ":median", "strict", false, 0.0, "median", "median");
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, cx, cy, lcp.point.id + ":bbox_center", "strict", false, 0.0, "bbox_center", "bbox_center");
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, anchor.first, anchor.second, lcp.point.id + ":space_projection", "strict", false, 0.0, "space_projection", "space_projection");
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, lcp.ideal_x, cy, lcp.point.id + ":aligned_x", "relaxed", false, 10.0, "aligned_x", "aligned");
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, cx, lcp.ideal_y, lcp.point.id + ":aligned_y", "relaxed", false, 10.0, "aligned_y", "aligned");
    add_classified_candidate(candidates, seen, lcp, space_region, blockers, box.x2 + offset, cy, lcp.point.id + ":width_offset", "relaxed", false, 20.0, "width_offset", "width_offset");

    const std::vector<std::pair<double, double>> emergency_offsets{
        {kGridUnit, 0.0},
        {-kGridUnit, 0.0},
        {0.0, kGridUnit},
        {0.0, -kGridUnit},
        {2.0 * kGridUnit, 0.0},
        {0.0, 2.0 * kGridUnit},
    };
    std::size_t emergency_index = 0;
    while (candidates.size() < kMinCandidatesPerLcp && emergency_index < emergency_offsets.size()) {
        const auto [dx, dy] = emergency_offsets[emergency_index++];
        add_candidate(
            candidates,
            seen,
            anchor.first + dx,
            anchor.second + dy,
            lcp.point.id + ":emergency:" + std::to_string(emergency_index),
            "emergency",
            true,
            10000.0,
            "emergency_offset",
            "emergency",
            false);
    }
    std::sort(candidates.begin(), candidates.end(), [&](const auto& lhs, const auto& rhs) {
        const auto cost = [&](const PhysicalLocationCandidate& candidate) {
            return std::abs(candidate.x - lcp.ideal_x) + std::abs(candidate.y - lcp.ideal_y) + candidate.penalty;
        };
        const double lhs_cost = cost(lhs);
        const double rhs_cost = cost(rhs);
        if (std::abs(lhs_cost - rhs_cost) > 1e-9) return lhs_cost < rhs_cost;
        return lhs.id < rhs.id;
    });
    if (candidates.size() > kMaxCandidatesPerLcp) candidates.resize(kMaxCandidatesPerLcp);
    lcp.point.location_candidates = std::move(candidates);
}

// 从 request placed_pins 收集指定 net 的全局 pin。
std::vector<PinInfo> pins_for_net(const Net& net, const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, PlacedPin> placed;
    for (const auto& pin : request.placed_pins) placed[pin.key] = pin;
    std::vector<PinInfo> result;
    for (const auto& terminal : net.terminals) {
        const auto found = placed.find(terminal);
        if (found == placed.end()) continue;
        result.push_back({terminal, found->second.module, found->second.x, found->second.y, routing::layer_to_index(found->second.layer)});
    }
    return result;
}

// 生成一个 net 的 leaf/root LCP 拓扑。
// 判断 leaf cluster 是否包含指定 terminal。
bool cluster_contains_terminal(const std::vector<PinInfo>& pins, const Cluster& cluster, const std::string& terminal) {
    for (int pin_index : cluster.pins) {
        if (pins[static_cast<std::size_t>(pin_index)].terminal == terminal) return true;
    }
    return false;
}

// 按 FLOW source/sink 给 pin-leaf 边定向，普通 pin 仍保持 pin 到 leaf 的默认汇聚语义。
std::pair<std::string, std::string> oriented_pin_leaf_endpoints(
    const std::optional<FlowConstraint>& flow,
    const std::string& pin,
    const std::string& leaf_id) {
    if (flow.has_value() && pin == flow->in_pin) return {leaf_id, pin};
    return {pin, leaf_id};
}

// 按 FLOW source/sink 给 leaf-root 边定向，确保 out_pin 可以经 root 追踪到 in_pin。
std::pair<std::string, std::string> oriented_leaf_root_endpoints(
    const std::optional<FlowConstraint>& flow,
    bool has_source,
    bool has_sink,
    const std::string& leaf_id,
    const std::string& root_id) {
    if (flow.has_value() && has_sink && !has_source) return {root_id, leaf_id};
    return {leaf_id, root_id};
}

void append_net_lcps(
    const Circuit& circuit,
    const Net& net,
    const RoutingEvaluationRequest& request,
    std::vector<WorkingLcp>& lcps) {
    const auto pins = pins_for_net(net, request);
    if (pins.size() <= 2) return;
    const auto [min_width, max_width] = width_range_for_net(circuit, net.name);
    const double layer_penalty = std::max(min_width * 4.0, median_pairwise_distance(pins) * 0.10);
    const int target_count = target_leaf_count(pins, min_width);
    auto clusters = repair_large_clusters(mst_clusters(pins, target_count, layer_penalty), static_cast<int>(pins.size()));
    if (clusters.empty()) return;
    const NetDirection direction = direction_for_pins(pins);
    const auto flow = flow_constraint_for_net(circuit, net.name);
    std::vector<std::pair<std::string, std::pair<double, double>>> leaf_ideals;
    std::vector<int> leaf_weights;
    std::vector<std::pair<bool, bool>> leaf_flow_roles;
    Rect root_box{};
    bool has_root_box = false;

    for (std::size_t index = 0; index < clusters.size(); ++index) {
        const auto& cluster = clusters[index];
        const std::string leaf_id = net.name + ":leaf:" + std::to_string(index);
        WorkingLcp lcp;
        lcp.net = net.name;
        lcp.required_width = min_width;
        lcp.direction = direction;
        lcp.candidate_seed = request.lcp_candidate_seed;
        lcp.point.id = leaf_id;
        lcp.support_bbox = support_bbox_for_pins(circuit, request, pins, cluster.pins);
        const auto ideal = cluster_ideal(pins, cluster.pins);
        lcp.ideal_x = ideal.first;
        lcp.ideal_y = ideal.second;
        for (int pin_index : cluster.pins) {
            const auto& pin = pins[static_cast<std::size_t>(pin_index)];
            const auto [from, to] = oriented_pin_leaf_endpoints(flow, pin.terminal, leaf_id);
            lcp.point.segments.push_back(make_segment(circuit, net.name, from, to, min_width, max_width));
        }
        include_rect(root_box, has_root_box, lcp.support_bbox);
        leaf_ideals.push_back({leaf_id, ideal});
        leaf_weights.push_back(static_cast<int>(cluster.pins.size()));
        leaf_flow_roles.push_back({
            flow.has_value() && cluster_contains_terminal(pins, cluster, flow->out_pin),
            flow.has_value() && cluster_contains_terminal(pins, cluster, flow->in_pin),
        });
        lcps.push_back(std::move(lcp));
    }

    if (clusters.size() <= 1) return;
    WorkingLcp root;
    root.net = net.name;
    root.required_width = min_width;
    root.direction = direction;
    root.candidate_seed = request.lcp_candidate_seed;
    root.point.id = net.name + ":root";
    root.support_bbox = root_box;
    std::vector<double> weighted_xs;
    std::vector<double> weighted_ys;
    for (std::size_t index = 0; index < leaf_ideals.size(); ++index) {
        for (int count = 0; count < leaf_weights[index]; ++count) {
            weighted_xs.push_back(leaf_ideals[index].second.first);
            weighted_ys.push_back(leaf_ideals[index].second.second);
        }
        const auto [from, to] = oriented_leaf_root_endpoints(
            flow,
            leaf_flow_roles[index].first,
            leaf_flow_roles[index].second,
            leaf_ideals[index].first,
            root.point.id);
        root.point.segments.push_back(make_segment(circuit, net.name, from, to, min_width, max_width));
    }
    root.ideal_x = median_value(std::move(weighted_xs));
    root.ideal_y = median_value(std::move(weighted_ys));
    lcps.push_back(std::move(root));
}

// 生成 module bbox 查找表。
std::unordered_map<std::string, Rect> module_boxes_for_request(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, Rect> boxes;
    for (const auto& [module, placement] : request.placements) {
        const auto size = placed_size(circuit.modules.at(module), placement);
        boxes[module] = {placement.x, placement.y, placement.x + size.first, placement.y + size.second};
    }
    return boxes;
}

// 从已放置 pin 表中构建快速查找表，供持久化 LCP 刷新候选点时恢复几何上下文。
std::unordered_map<std::string, PinInfo> placed_pin_map(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, PinInfo> result;
    for (const auto& pin : request.placed_pins) {
        result[pin.key] = {pin.key, pin.module, pin.x, pin.y, routing::layer_to_index(pin.layer)};
    }
    return result;
}

// 根据已有 LCP 的 segment 端点恢复候选点生成所需的轻量工作对象。
WorkingLcp working_lcp_from_existing(
    const LinkingControlPoint& point,
    const SpaceNode& space,
    const std::unordered_map<std::string, PinInfo>& pins,
    const std::unordered_map<std::string, Rect>& module_boxes,
    unsigned int candidate_seed) {
    WorkingLcp lcp;
    lcp.point = point;
    lcp.point.location_candidates.clear();
    lcp.candidate_seed = candidate_seed;
    if (!point.segments.empty()) {
        lcp.net = point.segments.front().net;
        // 从 0 起只取 segment.min_width，避免默认 1um 或 max_width 抬高障碍膨胀。
        lcp.required_width = 0.0;
        for (const auto& segment : point.segments) {
            lcp.required_width = std::max(lcp.required_width, segment.min_width);
        }
        if (lcp.required_width <= 0.0) lcp.required_width = kDefaultWidth;
    }

    Rect box{};
    bool has_box = false;
    std::vector<PinInfo> endpoint_pins;
    for (const auto& segment : point.segments) {
        for (const auto& endpoint : {segment.from, segment.to}) {
            const auto found = pins.find(endpoint);
            if (found == pins.end()) continue;
            endpoint_pins.push_back(found->second);
            include_point(box, has_box, found->second.x, found->second.y);
            const auto owner_box = module_boxes.find(found->second.module);
            if (owner_box != module_boxes.end()) include_rect(box, has_box, owner_box->second);
        }
    }
    if (!has_box) {
        for (const auto& candidate : point.location_candidates) {
            include_point(box, has_box, candidate.x, candidate.y);
        }
    }
    if (!has_box) {
        const auto owner_box = module_boxes.find(space.owner);
        if (owner_box != module_boxes.end()) {
            box = owner_box->second;
            has_box = true;
        }
    }
    if (!has_box) box = {0.0, 0.0, 0.0, 0.0};
    lcp.support_bbox = box;
    lcp.ideal_x = (box.x1 + box.x2) / 2.0;
    lcp.ideal_y = (box.y1 + box.y2) / 2.0;
    lcp.direction = endpoint_pins.empty() ? NetDirection::Neutral : direction_for_pins(endpoint_pins);
    return lcp;
}

}  // namespace

// 自动生成 placement-aware LCP 拓扑和候选点，并写入 routing request。
void generate_initial_lcp_topology(const Circuit& circuit, RoutingEvaluationRequest& request) {
    request.linking_points.clear();
    for (auto& space : request.space_nodes) {
        space.linking_points.clear();
        space.location_candidates.clear();
    }

    std::vector<WorkingLcp> lcps;
    for (const auto& net_name : circuit.net_order) {
        const auto found = circuit.nets.find(net_name);
        if (found == circuit.nets.end()) continue;
        append_net_lcps(circuit, found->second, request, lcps);
    }
    const auto module_boxes = module_boxes_for_request(circuit, request);
    bind_lcps_to_spaces(lcps, request.space_nodes, module_boxes);
    for (auto& lcp : lcps) {
        generate_candidates_for_lcp(lcp, request.space_nodes, module_boxes, request.active_region_blockers);
    }

    for (auto& lcp : lcps) {
        request.linking_points.push_back(lcp.point);
        auto space = std::find_if(request.space_nodes.begin(), request.space_nodes.end(), [&](const SpaceNode& node) {
            return node.id == lcp.point.space_node_id;
        });
        if (space != request.space_nodes.end()) space->linking_points.push_back(std::move(lcp.point));
    }
}

// 基于已有 LCP 拓扑刷新物理候选点，保持 space_node_id 和 segments 不变。
void refresh_lcp_location_candidates(const Circuit& circuit, RoutingEvaluationRequest& request) {
    request.linking_points.clear();
    const auto module_boxes = module_boxes_for_request(circuit, request);
    const auto pins = placed_pin_map(request);
    for (auto& space : request.space_nodes) {
        space.location_candidates.clear();
        for (auto& point : space.linking_points) {
            point.space_node_id = space.id;
            auto lcp = working_lcp_from_existing(point, space, pins, module_boxes, request.lcp_candidate_seed);
            generate_candidates_for_lcp(lcp, request.space_nodes, module_boxes, request.active_region_blockers);
            point = std::move(lcp.point);
            request.linking_points.push_back(point);
        }
    }
}

// 保持旧入口语义，供现有调用点继续使用。
void generate_automatic_lcps(const Circuit& circuit, RoutingEvaluationRequest& request) {
    const bool has_existing_lcp = std::any_of(request.space_nodes.begin(), request.space_nodes.end(), [](const SpaceNode& space) {
        return !space.linking_points.empty();
    });
    if (has_existing_lcp) {
        refresh_lcp_location_candidates(circuit, request);
    } else {
        generate_initial_lcp_topology(circuit, request);
    }
}

}  // namespace sapr
