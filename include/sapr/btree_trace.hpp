// 文件职责：导出 enhanced B*-tree 可视化 JSON，并附加 SA 轮次元数据。
#pragma once

#include <string>

#include "sapr/model.hpp"

namespace sapr {

// 将候选树、packing 与 routing topology 导出为 btree_trace JSON。
std::string make_btree_trace_json(
    const EnhancedBStarTree& tree,
    const RoutingEvaluationRequest& request,
    const Metrics& metrics);

// 在已有 btree_trace JSON 中写入 SA 单轮扰动元数据，供结构图标题展示。
std::string enrich_btree_trace_with_sa_iteration(
    const std::string& btree_trace_json,
    const SaBtreeIterationTrace& iteration);

// 生成带 SA 元数据的单轮候选树可视化记录。
SaBtreeIterationTrace make_sa_btree_iteration_trace(
    const EnhancedBStarTree& tree,
    const RoutingEvaluationRequest& request,
    const Metrics& metrics,
    const SaBtreeIterationTrace& meta);

}  // namespace sapr
