// 瀹炵幇璁烘枃 placement-aware 澧炲己 B*-tree contour packing銆乺outing adapter 鍜屾ā鎷熼€€鐏富寰幆銆?
#include "sapr/optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sapr/btree_trace.hpp"
#include "sapr/constraints.hpp"
#include "sapr/geometry.hpp"
#include "sapr/lcp_generator.hpp"
#include "sapr/router.hpp"
#include "sapr/routing/layer.hpp"
#include "sapr/routing/routing_context.hpp"
#include "sapr/routing/transform.hpp"
#include "sapr/routing_evaluator.hpp"
#include "sapr/tree.hpp"

namespace sapr {
namespace {

// 琛ㄧず鍊欓€夎В鍦ㄦā鎷熼€€鐏腑鐨勫畬鏁磋瘎浠枫€?
struct CandidateState {
    EnhancedBStarTree tree;
    RoutingEvaluationRequest request;
    RoutingFeedback feedback;
    double cost{};
};

// 琛ㄧず contour packing 宸插崰鐢ㄧ殑 group bounding box銆?
struct PackedRect {
    double x1{};
    double y1{};
    double x2{};
    double y2{};
};

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

void write_json_string(std::ostringstream& out, const std::string& value) {
    out << '"' << json_escape(value) << '"';
}

void write_json_string_array(std::ostringstream& out, const std::vector<std::string>& values) {
    out << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) out << ',';
        write_json_string(out, values[index]);
    }
    out << ']';
}

// 灏嗙揣鍑?JSON 鏍煎紡鍖栦负缂╄繘鏂囨湰锛屾彁鍗?routing_debug.json 鍙鎬с€?
std::string pretty_print_json(const std::string& compact) {
    std::ostringstream out;
    int indent = 0;
    bool in_string = false;
    bool escape = false;
    const auto write_indent = [&]() {
        for (int i = 0; i < indent; ++i) out << "  ";
    };

    for (std::size_t index = 0; index < compact.size(); ++index) {
        const char ch = compact[index];
        if (in_string) {
            out << ch;
            if (escape) {
                escape = false;
            } else if (ch == '\\') {
                escape = true;
            } else if (ch == '"') {
                in_string = false;
            }
            continue;
        }

        switch (ch) {
            case '"':
                in_string = true;
                out << ch;
                break;
            case '{':
            case '[':
                out << ch;
                if (index + 1 < compact.size() && compact[index + 1] != '}' && compact[index + 1] != ']') {
                    out << '\n';
                    ++indent;
                    write_indent();
                }
                break;
            case '}':
            case ']': {
                // 绌哄鍣ㄤ繚鎸?[] / {}锛岄伩鍏嶆媶鎴愬琛屻€?
                const char open = ch == '}' ? '{' : '[';
                std::size_t prev = index;
                while (prev > 0) {
                    --prev;
                    const char prior = compact[prev];
                    if (prior == ' ' || prior == '\n' || prior == '\r' || prior == '\t') continue;
                    break;
                }
                if (prev < index && compact[prev] == open) {
                    out << ch;
                } else {
                    out << '\n';
                    --indent;
                    write_indent();
                    out << ch;
                }
                break;
            }
            case ',':
                out << ch << '\n';
                write_indent();
                while (index + 1 < compact.size() && compact[index + 1] == ' ') ++index;
                break;
            case ':':
                out << ": ";
                while (index + 1 < compact.size() && compact[index + 1] == ' ') ++index;
                break;
            case ' ':
            case '\n':
            case '\r':
            case '\t':
                break;
            default:
                out << ch;
                break;
        }
    }
    out << '\n';
    return out.str();
}

// 鏀堕泦 space node 鍐呯殑 LCP 鏍囪瘑锛屼緵 enhanced B*-tree 璋冭瘯鍥惧睍绀虹┖闂磋妭鐐圭粨鏋勩€?
// 灏嗙數娴佹柟鍚戝啓鎴愮ǔ瀹氬瓧绗︿覆锛屼緵 B* 鏍戝彲瑙嗗寲瑙ｉ噴 LCP 鎷撴墤銆?
std::string current_direction_name(CurrentDirection direction) {
    switch (direction) {
        case CurrentDirection::In: return "in";
        case CurrentDirection::Out: return "out";
        case CurrentDirection::Unknown: return "unknown";
    }
    return "unknown";
}

// 鍐欏嚭鍙€夌煩褰㈠尯鍩燂紝渚?debug JSON 鏍囨敞 space node 鐗╃悊閫氶亾銆?
void write_optional_rect_json(std::ostringstream& out, const std::optional<Rect>& rect) {
    if (!rect.has_value()) {
        out << "null";
        return;
    }
    out << "{\"x1\": " << rect->x1
        << ", \"y1\": " << rect->y1
        << ", \"x2\": " << rect->x2
        << ", \"y2\": " << rect->y2
        << '}';
}

// 鏌ユ壘 LCP 鎵€灞?space node锛屼緵 debug JSON 杈撳嚭鐗╃悊閫氶亾銆?
const SpaceNode* find_space_for_lcp(const RoutingEvaluationRequest& request, const std::string& space_node_id) {
    for (const auto& space : request.space_nodes) {
        if (space.id == space_node_id) return &space;
    }
    return nullptr;
}

void write_wire_segment_json(std::ostringstream& out, const WireSegmentRef& segment) {
    out << "{\"id\": ";
    write_json_string(out, segment.id);
    out << ", \"from\": ";
    write_json_string(out, segment.from);
    out << ", \"to\": ";
    write_json_string(out, segment.to);
    out << ", \"current_direction\": ";
    write_json_string(out, current_direction_name(segment.current_direction));
    out << '}';
}

// 鍐欏嚭涓€涓?LCP 鐗╃悊鍊欓€夌偣锛屽府鍔╂帓鏌?LCP 鏄惁璐村埌 terminal pin 鎴?fallback 浣嶇疆銆?
void write_location_candidate_json(std::ostringstream& out, const PhysicalLocationCandidate& candidate) {
    out << "{\"id\": ";
    write_json_string(out, candidate.id);
    out << ", \"x\": " << candidate.x
        << ", \"y\": " << candidate.y
        << ", \"validity_level\": ";
    write_json_string(out, candidate.validity_level);
    out << ", \"is_fallback\": " << (candidate.is_fallback ? "true" : "false")
        << ", \"penalty\": " << candidate.penalty
        << ", \"reason\": ";
    write_json_string(out, candidate.reason);
    out << ", \"candidate_source\": ";
    write_json_string(out, candidate.source);
    out << ", \"inside_space_region\": " << (candidate.inside_space_region ? "true" : "false");
    out << '}';
}

// 鍐欏嚭甯﹀€欓€夊潗鏍囩殑 LCP 鎷撴墤锛屼緵 routing debug 瀵圭収 DP 鍜?detailed 閫夋嫨銆?
void write_debug_topologies_json(std::ostringstream& out, const RoutingEvaluationRequest& request) {
    out << "  \"lcp_topologies\": [\n";
    for (std::size_t topology_index = 0; topology_index < request.net_topologies.size(); ++topology_index) {
        const auto& topology = request.net_topologies[topology_index];
        if (topology_index != 0) out << ",\n";
        out << "    {\"net\": ";
        write_json_string(out, topology.net);
        out << ", \"pins\": ";
        write_json_string_array(out, topology.pins);
        out << ", \"linking_points\": [";
        for (std::size_t point_index = 0; point_index < topology.linking_points.size(); ++point_index) {
            if (point_index != 0) out << ',';
            const auto& point = topology.linking_points[point_index];
            out << "{\"id\": ";
            write_json_string(out, point.id);
            out << ", \"space_node_id\": ";
            write_json_string(out, point.space_node_id);
            out << ", \"physical_region\": ";
            const auto* space = find_space_for_lcp(request, point.space_node_id);
            write_optional_rect_json(out, space == nullptr ? std::optional<Rect>{} : space->physical_region);
            out << ", \"location_candidates\": [";
            for (std::size_t candidate_index = 0; candidate_index < point.location_candidates.size(); ++candidate_index) {
                if (candidate_index != 0) out << ',';
                write_location_candidate_json(out, point.location_candidates[candidate_index]);
            }
            out << "]}";
        }
        out << "], \"segments\": [";
        for (std::size_t segment_index = 0; segment_index < topology.segments.size(); ++segment_index) {
            if (segment_index != 0) out << ',';
            write_wire_segment_json(out, topology.segments[segment_index]);
        }
        out << "]}";
    }
    out << "\n  ]";
}

// 鍐欏嚭 routing.txt 璇箟鐨勪竴娈垫渶缁堥噾灞炵嚎锛屼究浜庡拰 PNG 涓婄殑寮傚父绾挎浜掔浉瀹氫綅銆?
void write_debug_space_nodes_json(std::ostringstream& out, const RoutingEvaluationRequest& request) {
    out << "  \"space_nodes\": [\n";
    for (std::size_t index = 0; index < request.space_nodes.size(); ++index) {
        if (index != 0) out << ",\n";
        const auto& space = request.space_nodes[index];
        out << "    {\"id\": ";
        write_json_string(out, space.id);
        out << ", \"owner\": ";
        write_json_string(out, space.owner);
        out << ", \"kind\": ";
        write_json_string(out, space_kind_name(space.kind));
        out << ", \"formula_required_space\": " << space.formula_required_space()
            << ", \"allocated_space\": " << space.allocated_space
            << ", \"coupling_extra_space\": " << space.coupling_extra_space
            << ", \"final_required_space\": " << space.required_space()
            << ", \"required_space\": " << space.required_space()
            << ", \"lcp_count\": " << space.linking_points.size()
            << ", \"physical_region\": ";
        write_optional_rect_json(out, space.physical_region);
        out << '}';
    }
    out << "\n  ]";
}

void write_route_segment_json(std::ostringstream& out, const RouteSegment& route) {
    out << "{\"net\": ";
    write_json_string(out, route.net);
    out << ", \"layer\": ";
    write_json_string(out, route.layer);
    out << ", \"x1\": " << route.x1
        << ", \"y1\": " << route.y1
        << ", \"x2\": " << route.x2
        << ", \"y2\": " << route.y2
        << ", \"width\": " << route.width
        << '}';
}

// 鍐欏嚭 A*/DP 鍊欓€夋憳瑕侊紝鐢ㄤ簬姣旇緝 DP traceback 鍜?detailed legalize 鍚庣殑瀹為檯閫夋嫨銆?
// 灏?A*/DP 鐨勬枃鏈秷鎭綊涓€鎴愮ǔ瀹氬け璐ョ被鍒€?
std::string route_candidate_failure_reason(const routing::RouteCandidate& candidate) {
    if (candidate.path.success) return "";
    if (candidate.path.message.find("multi_terminal_missing") != std::string::npos ||
        candidate.path.message.find("multi-terminal") != std::string::npos) {
        return "multi_terminal_missing";
    }
    if (candidate.path.message.find("active_blocker") != std::string::npos) return "active_blocker";
    if (candidate.path.message.find("outside_space") != std::string::npos) return "outside_space";
    if (candidate.path.message.find("exceeded max expanded nodes") != std::string::npos ||
        candidate.path.message.find("could not find a feasible path") != std::string::npos) {
        return "path_fail";
    }
    return "path_fail";
}

void write_route_candidate_json(std::ostringstream& out, const routing::RouteCandidate& candidate) {
    out << "{\"net\": ";
    write_json_string(out, candidate.net);
    out << ", \"from\": ";
    write_json_string(out, candidate.from_terminal);
    out << ", \"to\": ";
    write_json_string(out, candidate.to_terminal);
    out << ", \"segment_id\": ";
    write_json_string(out, candidate.segment_id);
    out << ", \"lcp_candidate_id\": ";
    write_json_string(out, candidate.lcp_candidate_id);
    out << ", \"source_lcp_candidate_id\": ";
    write_json_string(out, candidate.source_lcp_candidate_id);
    out << ", \"target_lcp_candidate_id\": ";
    write_json_string(out, candidate.target_lcp_candidate_id);
    out << ", \"path_success\": " << (candidate.path.success ? "true" : "false")
        << ", \"path_message\": ";
    write_json_string(out, candidate.path.message);
    out << ", \"failure_reason\": ";
    write_json_string(out, route_candidate_failure_reason(candidate));
    out << ", \"wirelength\": " << candidate.path.metrics.wirelength
        << ", \"bend_count\": " << candidate.path.metrics.bend_count
        << ", \"via_count\": " << candidate.path.metrics.via_count
        << ", \"path_points\": [";
    for (std::size_t index = 0; index < candidate.path.points.size(); ++index) {
        if (index != 0) out << ',';
        const auto& point = candidate.path.points[index];
        out << "{\"ix\": " << point.ix
            << ", \"iy\": " << point.iy
            << ", \"layer\": ";
        write_json_string(out, routing::index_to_layer(point.layer));
        out << '}';
    }
    out << "]}";
}

// 鍐欏嚭 DP 瀵瑰€欓€夌殑閫夋嫨鎴栨嫆缁濅簨浠讹紝瑙ｉ噴 path fail銆丩CP 缁戝畾鍐茬獊鍜岀煭璺?penalty銆?
void write_dp_candidate_event_json(std::ostringstream& out, const routing::RoutingDpCandidateEvent& event) {
    out << "{\"group_key\": ";
    write_json_string(out, event.group_key);
    out << ", \"net\": ";
    write_json_string(out, event.net);
    out << ", \"from\": ";
    write_json_string(out, event.from_terminal);
    out << ", \"to\": ";
    write_json_string(out, event.to_terminal);
    out << ", \"segment_id\": ";
    write_json_string(out, event.segment_id);
    out << ", \"lcp_candidate_id\": ";
    write_json_string(out, event.lcp_candidate_id);
    out << ", \"state_lcp_candidate_id\": ";
    write_json_string(out, event.state_lcp_candidate_id);
    out << ", \"reason\": ";
    write_json_string(out, event.reason);
    out << ", \"selected\": " << (event.selected ? "true" : "false")
        << '}';
}

// 写出最终 bottom-up DP 状态，供 strict LCP-DP 失败时定位未覆盖 terminal 与绑定冲突。
void write_dp_result_json(std::ostringstream& out, const std::optional<routing::RoutingDpResult>& result) {
    if (!result.has_value()) {
        out << "null";
        return;
    }
    const auto& dp = *result;
    out << "{\"success\": " << (dp.success ? "true" : "false")
        << ", \"dp_nodes\": " << dp.dp_nodes
        << ", \"dp_states\": " << dp.dp_states
        << ", \"dp_pruned_states\": " << dp.dp_pruned_states
        << ", \"traceback_candidate_count\": " << dp.traceback_candidates.size()
        << ", \"best_state\": {\"id\": " << dp.best_state.id
        << ", \"tree_node\": ";
    write_json_string(out, dp.best_state.tree_node);
    out << ", \"covered_terminals\": ";
    write_json_string_array(out, dp.best_state.covered_terminals);
    out << ", \"covered_wire_segments\": ";
    write_json_string_array(out, dp.best_state.covered_wire_segments);
    out << ", \"selected_transitions\": ";
    write_json_string_array(out, dp.best_state.selected_transitions);
    out << ", \"failure_messages\": ";
    write_json_string_array(out, dp.best_state.failure_messages);
    out << "}}";
}

// 鍐欏嚭 detailed traceback 涓嚭鐜扮殑 pin/LCP 鑺傜偣銆?
void write_detailed_node_json(std::ostringstream& out, const DetailedRouteNode& node) {
    out << "{\"id\": ";
    write_json_string(out, node.id);
    out << ", \"kind\": ";
    write_json_string(out, node.kind);
    out << ", \"space_node_id\": ";
    write_json_string(out, node.space_node_id);
    out << ", \"x\": " << node.x
        << ", \"y\": " << node.y
        << ", \"layer\": ";
    write_json_string(out, node.layer);
    out << '}';
}

// 鍐欏嚭 detailed route segment 鍒?LCP/space-node/DP state 鐨勬槧灏勩€?
void write_detailed_segment_json(std::ostringstream& out, const DetailedRouteSegment& segment) {
    out << "{\"route_index\": " << segment.route_index
        << ", \"dp_state_id\": " << segment.dp_state_id
        << ", \"net\": ";
    write_json_string(out, segment.net);
    out << ", \"from\": ";
    write_json_string(out, segment.from_terminal);
    out << ", \"to\": ";
    write_json_string(out, segment.to_terminal);
    out << ", \"tree_node\": ";
    write_json_string(out, segment.tree_node);
    out << ", \"segment_id\": ";
    write_json_string(out, segment.segment_id);
    out << ", \"lcp_id\": ";
    write_json_string(out, segment.lcp_id);
    out << ", \"lcp_candidate_id\": ";
    write_json_string(out, segment.lcp_candidate_id);
    out << ", \"space_node_id\": ";
    write_json_string(out, segment.space_node_id);
    out << '}';
}

// 鍐欏嚭鍗曚釜 net 鐨?detailed traceback 鎽樿鍜岃鍛娿€?
void write_detailed_trace_json(std::ostringstream& out, const DetailedRouteTrace& trace) {
    out << "{\"net\": ";
    write_json_string(out, trace.net);
    out << ", \"nodes\": [";
    for (std::size_t index = 0; index < trace.nodes.size(); ++index) {
        if (index != 0) out << ',';
        write_detailed_node_json(out, trace.nodes[index]);
    }
    out << "], \"segments\": [";
    for (std::size_t index = 0; index < trace.segments.size(); ++index) {
        if (index != 0) out << ',';
        write_detailed_segment_json(out, trace.segments[index]);
    }
    out << "], \"warnings\": ";
    write_json_string_array(out, trace.warnings);
    out << '}';
}

// 姹囨€讳竴娆?routing evaluation 鍜?detailed routing 鐨勮瘖鏂俊鎭€?
// 缁熻 request 涓墍鏈?LCP 鐗╃悊鍊欓€夋暟閲忋€?
std::size_t count_lcp_location_candidates(const RoutingEvaluationRequest& request) {
    std::size_t total = 0;
    for (const auto& point : request.linking_points) total += point.location_candidates.size();
    return total;
}

// 缁熻宸查€氳繃 A* 涓?multi-terminal 杩囨护鐨勫敮涓€ LCP 鐗╃悊鍊欓€夋暟閲忋€?
std::size_t count_reachable_lcp_locations(const RoutingEvaluation& evaluation) {
    std::unordered_set<std::string> reachable;
    const auto& candidates = evaluation.debug_candidates.empty() ? evaluation.candidates : evaluation.debug_candidates;
    for (const auto& candidate : candidates) {
        if (!candidate.path.success) continue;
        if (!candidate.lcp_id.empty() && !candidate.lcp_candidate_id.empty()) {
            reachable.insert(candidate.lcp_id + "@" + candidate.lcp_candidate_id);
        }
        if (!candidate.source_lcp_id.empty() && !candidate.source_lcp_candidate_id.empty()) {
            reachable.insert(candidate.source_lcp_id + "@" + candidate.source_lcp_candidate_id);
        }
        if (!candidate.target_lcp_id.empty() && !candidate.target_lcp_candidate_id.empty()) {
            reachable.insert(candidate.target_lcp_id + "@" + candidate.target_lcp_candidate_id);
        }
    }
    return reachable.size();
}

// 统计能够覆盖全部 incident segments 的 LCP 物理候选数量。
std::size_t count_full_lcp_coverages(const std::vector<LcpCandidateCoverage>& coverages) {
    return static_cast<std::size_t>(std::count_if(coverages.begin(), coverages.end(), [](const auto& coverage) {
        return coverage.covers_all_incident_segments;
    }));
}

// 统计仍缺少 incident segments 的 LCP 物理候选数量。
std::size_t count_missing_lcp_coverages(const std::vector<LcpCandidateCoverage>& coverages) {
    return static_cast<std::size_t>(std::count_if(coverages.begin(), coverages.end(), [](const auto& coverage) {
        return !coverage.covers_all_incident_segments;
    }));
}

// 返回第一条未全覆盖的 LCP 候选，供 summary 快速定位。
std::string first_uncovered_lcp_location(const std::vector<LcpCandidateCoverage>& coverages) {
    const auto found = std::find_if(coverages.begin(), coverages.end(), [](const auto& coverage) {
        return !coverage.covers_all_incident_segments;
    });
    return found == coverages.end() ? std::string{} : found->lcp_id + "@" + found->candidate_id;
}

// 写出每个 LCP 物理候选点的 incident segment 覆盖诊断。
void write_lcp_candidate_coverage_json(std::ostringstream& out, const std::vector<LcpCandidateCoverage>& coverages) {
    out << "  \"lcp_candidate_coverage\": [";
    for (std::size_t index = 0; index < coverages.size(); ++index) {
        if (index != 0) out << ',';
        const auto& coverage = coverages[index];
        out << "{\"lcp_id\": ";
        write_json_string(out, coverage.lcp_id);
        out << ", \"candidate_id\": ";
        write_json_string(out, coverage.candidate_id);
        out << ", \"required_segments\": ";
        write_json_string_array(out, coverage.required_segments);
        out << ", \"reachable_segments\": ";
        write_json_string_array(out, coverage.reachable_segments);
        out << ", \"missing_segments\": ";
        write_json_string_array(out, coverage.missing_segments);
        out << ", \"covers_all_incident_segments\": "
            << (coverage.covers_all_incident_segments ? "true" : "false")
            << '}';
    }
    out << "]";
}

bool used_lcp_direct_fallback(const RoutingEvaluationRequest& request, const RoutingEvaluation& evaluation) {
    return !evaluation.strict_lcp_dp_blocked_fallback &&
           !request.linking_points.empty() && evaluation.bottom_up_dp.has_value() && !evaluation.bottom_up_dp->success;
}

// 鎸?net 缁熻娌℃湁浠讳綍鎴愬姛 LCP candidate 鐨勬嫇鎵戞暟閲忋€?
std::size_t count_lcp_fallback_nets(const RoutingEvaluationRequest& request, const RoutingEvaluation& evaluation) {
    const auto& candidates = evaluation.debug_candidates.empty() ? evaluation.candidates : evaluation.debug_candidates;
    std::unordered_set<std::string> nets_with_successful_lcp;
    for (const auto& candidate : candidates) {
        if (!candidate.path.success) continue;
        if (candidate.lcp_id.empty() && candidate.source_lcp_id.empty() && candidate.target_lcp_id.empty()) continue;
        nets_with_successful_lcp.insert(candidate.net);
    }
    std::size_t failed = 0;
    for (const auto& topology : request.net_topologies) {
        if (!topology.linking_points.empty() && !nets_with_successful_lcp.contains(topology.net)) ++failed;
    }
    return failed;
}

std::string make_routing_debug_json(
    const RoutingEvaluationRequest& request,
    const RoutingEvaluation& evaluation,
    const DetailedRoutingResult& detailed,
    const Metrics& metrics) {
    std::ostringstream out;
    const auto& debug_candidates = evaluation.debug_candidates.empty() ? evaluation.candidates : evaluation.debug_candidates;
    const auto lcp_coverages = analyze_lcp_candidate_coverage(request, debug_candidates);
    const std::string first_uncovered = first_uncovered_lcp_location(lcp_coverages);
    out << "{\n";
    // summary 涓?routing_cost/global_* 鏉ヨ嚜 global 闃舵锛沠inal_penalty/detailed_cost 鏉ヨ嚜鏈€缁堝彛寰勩€?
    out << "  \"summary\": {"
        << "\"routing_cost\": " << evaluation.routing_cost
        << ", \"global_wirelength\": " << metrics.global_wirelength
        << ", \"global_bends\": " << metrics.global_bend_count
        << ", \"global_vias\": " << metrics.global_via_count
        << ", \"global_penalty\": " << metrics.global_penalty
        << ", \"final_penalty\": " << metrics.penalty
        << ", \"candidate_count\": " << evaluation.candidates.size()
        << ", \"dp_used\": " << (evaluation.used_bottom_up_dp ? "true" : "false")
        << ", \"failed_nets\": " << evaluation.failed_nets
        << ", \"detailed_routes\": " << detailed.routes.size()
        << ", \"space_nodes_with_routes\": " << detailed.space_nodes_with_routes
        << ", \"traceback_failures\": " << detailed.traceback_failures
        << ", \"design_rule_violations\": " << detailed.design_rule_violations
        << ", \"flow_violations\": " << detailed.flow_violations
        << ", \"current_density_violations\": " << detailed.current_density_violations
        << ", \"detailed_cost\": " << detailed.detailed_cost
        << ", \"phi_cost\": " << metrics.phi_cost
        << ", \"lcp_candidates_total\": " << count_lcp_location_candidates(request)
        << ", \"lcp_candidates_reachable\": " << count_reachable_lcp_locations(evaluation)
        << ", \"lcp_locations_covering_all_segments\": " << count_full_lcp_coverages(lcp_coverages)
        << ", \"lcp_locations_missing_segments\": " << count_missing_lcp_coverages(lcp_coverages)
        << ", \"fallback_blocked_by_strict_lcp_dp\": "
        << (evaluation.strict_lcp_dp_blocked_fallback ? "true" : "false")
        << ", \"first_uncovered_lcp_location\": ";
    write_json_string(out, first_uncovered);
    out
        << ", \"lcp_nets_fallback_count\": " << count_lcp_fallback_nets(request, evaluation)
        << ", \"fallback_reason\": ";
    write_json_string(out, used_lcp_direct_fallback(request, evaluation) ? "no_multi_terminal_reachable_lcp_candidate" : "");
    out
        << "},\n";
    out << "  \"bottom_up_dp\": ";
    write_dp_result_json(out, evaluation.bottom_up_dp);
    out << ",\n";
    write_debug_topologies_json(out, request);
    out << ",\n";
    write_debug_space_nodes_json(out, request);
    out << ",\n";
    write_lcp_candidate_coverage_json(out, lcp_coverages);
    out << ",\n  \"final_candidates\": [";
    for (std::size_t index = 0; index < evaluation.candidates.size(); ++index) {
        if (index != 0) out << ',';
        write_route_candidate_json(out, evaluation.candidates[index]);
    }
    out << "],\n  \"all_candidates\": [";
    for (std::size_t index = 0; index < evaluation.debug_candidates.size(); ++index) {
        if (index != 0) out << ',';
        write_route_candidate_json(out, evaluation.debug_candidates[index]);
    }
    out << "]";
    out << ",\n  \"dp_traceback_candidates\": [";
    if (evaluation.bottom_up_dp.has_value()) {
        for (std::size_t index = 0; index < evaluation.bottom_up_dp->traceback_candidates.size(); ++index) {
            if (index != 0) out << ',';
            write_route_candidate_json(out, evaluation.bottom_up_dp->traceback_candidates[index]);
        }
    }
    out << "],\n  \"dp_candidate_events\": [";
    if (evaluation.bottom_up_dp.has_value()) {
        for (std::size_t index = 0; index < evaluation.bottom_up_dp->candidate_events.size(); ++index) {
            if (index != 0) out << ',';
            write_dp_candidate_event_json(out, evaluation.bottom_up_dp->candidate_events[index]);
        }
    }
    out << "],\n  \"final_routes\": [";
    for (std::size_t index = 0; index < detailed.routes.size(); ++index) {
        if (index != 0) out << ',';
        write_route_segment_json(out, detailed.routes[index]);
    }
    out << "],\n  \"detailed_traces\": [";
    for (std::size_t index = 0; index < detailed.report.traces.size(); ++index) {
        if (index != 0) out << ',';
        write_detailed_trace_json(out, detailed.report.traces[index]);
    }
    out << "],\n  \"warnings\": ";
    write_json_string_array(out, detailed.report.warnings);
    out << ",\n  \"design_rule_segments\": ";
    write_json_string_array(out, detailed.report.design_rule_segments);
    out << ",\n  \"coupling_pairs\": ";
    write_json_string_array(out, detailed.report.coupling_pairs);
    out << "\n}\n";
    return pretty_print_json(out.str());
}

// 杩斿洖鎸囧畾妯″潡鏄惁鏄绉?pair 鐨勪唬琛ㄦā鍧椼€?
const SymmetryGroupNode* symmetry_group_for_hierarchy(const EnhancedBStarTree& tree, const std::string& hierarchy) {
    for (const auto& group : tree.symmetry_groups) {
        if (group.hierarchy_node_id == hierarchy) return &group;
    }
    return nullptr;
}

// 杩斿洖妯″潡鏃嬭浆鍚庣殑灏哄銆?
std::pair<double, double> module_size(const Circuit& circuit, const std::string& module, int angle) {
    return placed_size(circuit.modules.at(module), {module, 0.0, 0.0, angle, "R0"});
}

// 杩斿洖浠ｈ〃鑺傜偣鍦?packing 涓崰鐢ㄧ殑鏁翠綋灏哄锛屽寘鍚绉伴暅鍍忔ā鍧楀拰杞撮棿棰勭暀绌洪棿銆?
std::pair<double, double> occupied_size(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const BStarNode& node,
    const SolverConfig& config) {
    (void)tree;
    (void)config;
    if (node.kind == BStarNodeKind::Hierarchy) {
        return {0.0, 0.0};
    }
    auto size = module_size(circuit, node.module, node.angle);
    const double right_space = node.right_space.required_space();
    const double top_space = node.top_space.required_space();
    return {size.first + right_space, size.second + top_space};
}

// 杩斿洖 contour 鍦ㄦ寚瀹?x 鍖洪棿鍐呯殑鏈€楂?y銆?
double contour_height(const std::vector<PackedRect>& packed, double x, double width) {
    double result = 0.0;
    const double x2 = x + width;
    for (const auto& rect : packed) {
        if (x < rect.x2 && x2 > rect.x1) result = std::max(result, rect.y2);
    }
    return result;
}

// 灏?angle 杞垚鍩虹 Cadence orient 瀛楃涓层€?
std::string orient_for_angle(int angle) {
    switch ((angle % 360 + 360) % 360) {
        case 0: return "R0";
        case 90: return "R90";
        case 180: return "R180";
        case 270: return "R270";
    }
    return "R0";
}

// 返回 representative 绕垂直对称轴镜像后应使用的完整 Cadence orient。
std::string vertical_mirror_orient_for_angle(int angle) {
    switch ((angle % 360 + 360) % 360) {
        case 0: return "MY";
        case 90: return "MXR90";
        case 180: return "MX";
        case 270: return "MYR90";
    }
    return "MY";
}

// 鐢熸垚瀵圭О pair 涓暅鍍忔ā鍧楃殑 placement銆?
struct AsfPackingResult {
    std::unordered_map<std::string, Placement> placements;
    std::vector<SpaceNode> space_nodes;
    PackingContourTrace trace;
    Rect bbox;
    std::vector<std::string> stored_modules;
};

double required_space_for_group(const SpaceNodeGroup& group) {
    double result = 0.0;
    for (const auto& space : group.spaces) result = std::max(result, space.required_space());
    return result;
}

double required_space_for_cluster(const std::optional<SpaceNodeCluster>& cluster) {
    if (!cluster.has_value()) return 0.0;
    double result = 0.0;
    for (const auto& space : cluster->spaces) result = std::max(result, space.required_space());
    return result;
}

std::pair<double, double> asf_node_occupied_size(const Circuit& circuit, const AsfBStarNode& node, const SolverConfig& config) {
    const auto size = module_size(circuit, node.module, node.angle);
    const double outer = node.space_node_groups.empty() ? 0.0 : required_space_for_group(node.space_node_groups.front());
    const double top = node.space_node_groups.size() < 2 ? 0.0 : required_space_for_group(node.space_node_groups[1]);
    const double cluster = required_space_for_cluster(node.space_node_cluster);
    return {size.first + std::max({outer, cluster, config.spacing}), size.second + std::max(top, config.spacing)};
}

Rect placement_rect(const Circuit& circuit, const Placement& placement) {
    const auto size = placed_size(circuit.modules.at(placement.module), placement);
    return {placement.x, placement.y, placement.x + size.first, placement.y + size.second};
}

// 验证 ASF 局部坐标系中每个 pair 的 bbox 和同名引脚关于 x=0 垂直镜像。
bool validate_asf_vertical_symmetry(
    const Circuit& circuit,
    const AsfBStarTree& asf,
    const std::unordered_map<std::string, Placement>& placements,
    std::string& error) {
    constexpr double kTolerance = 1e-8;
    const auto same = [](double left, double right) { return std::abs(left - right) <= kTolerance; };
    for (const auto& [representative, mirror] : asf.mirror_map) {
        const auto rep_placement = placements.find(representative);
        const auto mirror_placement = placements.find(mirror);
        if (rep_placement == placements.end() || mirror_placement == placements.end()) {
            error = "missing ASF pair placement for " + representative + " and " + mirror;
            return false;
        }
        const Rect rep_box = placement_rect(circuit, rep_placement->second);
        const Rect mirror_box = placement_rect(circuit, mirror_placement->second);
        if (!same(rep_box.x1, -mirror_box.x2) || !same(rep_box.x2, -mirror_box.x1) ||
            !same(rep_box.y1, mirror_box.y1) || !same(rep_box.y2, mirror_box.y2)) {
            error = "bbox mismatch for " + representative + " and " + mirror;
            return false;
        }

        std::unordered_map<std::string, const Pin*> mirror_pins;
        for (const auto& terminal : circuit.pin_order) {
            const auto& pin = circuit.pins.at(terminal);
            if (pin.module == mirror) mirror_pins[pin.name] = &pin;
        }
        for (const auto& terminal : circuit.pin_order) {
            const auto& pin = circuit.pins.at(terminal);
            if (pin.module != representative) continue;
            const auto matched = mirror_pins.find(pin.name);
            if (matched == mirror_pins.end()) {
                error = "missing mirror pin " + mirror + "." + pin.name;
                return false;
            }
            const auto [rep_x, rep_y] = placed_pin(circuit.modules.at(representative), pin, rep_placement->second);
            const auto [mirror_x, mirror_y] = placed_pin(circuit.modules.at(mirror), *matched->second, mirror_placement->second);
            if (!same(rep_x, -mirror_x) || !same(rep_y, mirror_y)) {
                error = "pin mismatch for " + representative + "." + pin.name;
                return false;
            }
        }
    }
    return true;
}

void translate_rect(Rect& rect, double dx, double dy) {
    rect.x1 += dx;
    rect.y1 += dy;
    rect.x2 += dx;
    rect.y2 += dy;
}

void translate_asf_result(AsfPackingResult& result, double dx, double dy) {
    for (auto& [_, placement] : result.placements) {
        placement.x += dx;
        placement.y += dy;
    }
    translate_rect(result.bbox, dx, dy);
    for (auto& space : result.space_nodes) {
        if (space.physical_region.has_value()) translate_rect(*space.physical_region, dx, dy);
    }
    for (auto& step : result.trace.steps) {
        step.x += dx;
        step.y += dy;
        step.desired_x += dx;
        step.desired_y += dy;
        step.contour_y += dy;
        translate_rect(step.occupied_bbox, dx, dy);
    }
}

void set_region(SpaceNode& space, const Rect& region) {
    constexpr double kMinRegionWidth = 1.0;
    Rect fixed = region;
    if (fixed.x2 < fixed.x1) std::swap(fixed.x1, fixed.x2);
    if (fixed.y2 < fixed.y1) std::swap(fixed.y1, fixed.y2);
    if (fixed.x2 - fixed.x1 < 1e-9) fixed.x2 = fixed.x1 + kMinRegionWidth;
    if (fixed.y2 - fixed.y1 < 1e-9) fixed.y2 = fixed.y1 + kMinRegionWidth;
    space.physical_region = fixed;
}

void append_asf_spaces_for_node(
    const Circuit& circuit,
    const AsfBStarNode& source_node,
    const std::unordered_map<std::string, Placement>& placements,
    std::vector<SpaceNode>& spaces) {
    const Rect rep = placement_rect(circuit, placements.at(source_node.module));
    const Rect mirror = source_node.mirror_module.has_value()
                            ? placement_rect(circuit, placements.at(*source_node.mirror_module))
                            : rep;
    const double outer_width = source_node.space_node_groups.empty() ? 1.0 : std::max(1.0, required_space_for_group(source_node.space_node_groups.front()));
    const double top_width = source_node.space_node_groups.size() < 2 ? 1.0 : std::max(1.0, required_space_for_group(source_node.space_node_groups[1]));
    const double cluster_width = std::max(1.0, required_space_for_cluster(source_node.space_node_cluster));

    if (!source_node.space_node_groups.empty()) {
        auto group = source_node.space_node_groups.front();
        if (group.spaces.size() >= 2) {
            set_region(group.spaces[0], Rect{rep.x2, rep.y1, rep.x2 + outer_width, rep.y2});
            set_region(group.spaces[1], Rect{mirror.x1 - outer_width, mirror.y1, mirror.x1, mirror.y2});
        }
        spaces.insert(spaces.end(), group.spaces.begin(), group.spaces.end());
    }
    if (source_node.space_node_groups.size() >= 2) {
        auto group = source_node.space_node_groups[1];
        if (group.spaces.size() >= 2) {
            set_region(group.spaces[0], Rect{rep.x1, rep.y2, rep.x2, rep.y2 + top_width});
            set_region(group.spaces[1], Rect{mirror.x1, mirror.y2, mirror.x2, mirror.y2 + top_width});
        }
        spaces.insert(spaces.end(), group.spaces.begin(), group.spaces.end());
    }
    if (source_node.space_node_cluster.has_value()) {
        auto cluster = *source_node.space_node_cluster;
        if (cluster.spaces.size() >= 4) {
            const double axis_x = 0.0;
            set_region(cluster.spaces[0], Rect{axis_x, rep.y1, rep.x1, rep.y2});
            set_region(cluster.spaces[1], Rect{mirror.x2, mirror.y1, axis_x, mirror.y2});
            set_region(cluster.spaces[2], Rect{rep.x1, rep.y2, rep.x2, rep.y2 + cluster_width});
            set_region(cluster.spaces[3], Rect{mirror.x1, mirror.y2, mirror.x2, mirror.y2 + cluster_width});
        }
        spaces.insert(spaces.end(), cluster.spaces.begin(), cluster.spaces.end());
    }
}

AsfPackingResult pack_asf_bstar_tree(const Circuit& circuit, const SymmetryGroupNode& group, const SolverConfig& config) {
    AsfPackingResult result;
    result.stored_modules = group.stored_modules;
    const auto& asf = group.asf_bstar_tree;
    if (!asf.root.has_value()) return result;

    std::vector<PackedRect> packed;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, double, double)> place_node = [&](const std::string& id, double desired_x, double desired_y) {
        if (visited.contains(id)) return;
        visited.insert(id);
        const auto& node = asf.nodes.at(id);
        const auto occupied = asf_node_occupied_size(circuit, node, config);
        const double x = desired_x;
        const double contour_y = contour_height(packed, x, occupied.first);
        const double y = std::max(desired_y, contour_y);
        Placement placement{id, x, y, node.angle, orient_for_angle(node.angle)};
        result.placements[id] = placement;
        const auto rep_size = module_size(circuit, id, node.angle);
        if (node.mirror_module.has_value()) {
            const auto mirror_size = module_size(circuit, *node.mirror_module, node.angle);
            Placement mirror{
                *node.mirror_module,
                -x - mirror_size.first,
                y,
                node.angle,
                vertical_mirror_orient_for_angle(node.angle)};
            (void)rep_size;
            result.placements[*node.mirror_module] = mirror;
        }

        std::vector<std::string> step_modules{id};
        if (node.mirror_module.has_value()) step_modules.push_back(*node.mirror_module);
        std::optional<std::string> left_trace;
        std::optional<std::string> right_trace;
        if (node.left.has_value()) left_trace = "asf:" + group.name + "/" + *node.left;
        if (node.right.has_value()) right_trace = "asf:" + group.name + "/" + *node.right;
        const double group_space = [&]() {
            double value = 0.0;
            for (const auto& space_node_group : node.space_node_groups) value += required_space_for_group(space_node_group);
            value += required_space_for_cluster(node.space_node_cluster);
            return value;
        }();
        result.trace.steps.push_back(PackingContourStep{
            "asf:" + group.name + "/" + id,
            id,
            x,
            y,
            Rect{x, y, x + occupied.first, y + occupied.second},
            desired_x,
            desired_y,
            contour_y,
            group_space,
            0.0,
            0.0,
            left_trace,
            right_trace,
            step_modules,
            {},
            {},
        });
        packed.push_back({x, y, x + occupied.first, y + occupied.second});
        if (node.left.has_value()) place_node(*node.left, x + occupied.first + config.spacing, y);
        if (node.right.has_value()) place_node(*node.right, x, y + occupied.second + config.spacing);
    };

    const double axis_clearance = std::max(1.0, config.spacing);
    place_node(*asf.root, axis_clearance, 0.0);

    std::string symmetry_error;
    if (!validate_asf_vertical_symmetry(circuit, asf, result.placements, symmetry_error)) {
        throw std::runtime_error("symmetry_transform_mismatch: " + symmetry_error);
    }

    bool first = true;
    Rect bbox{};
    for (const auto& [module, placement] : result.placements) {
        const Rect rect = placement_rect(circuit, placement);
        if (first) {
            bbox = rect;
            first = false;
        } else {
            bbox.x1 = std::min(bbox.x1, rect.x1);
            bbox.y1 = std::min(bbox.y1, rect.y1);
            bbox.x2 = std::max(bbox.x2, rect.x2);
            bbox.y2 = std::max(bbox.y2, rect.y2);
        }
    }
    result.bbox = bbox;
    for (const auto& id : asf.representative_order) {
        append_asf_spaces_for_node(circuit, asf.nodes.at(id), result.placements, result.space_nodes);
    }
    for (const auto& space : result.space_nodes) {
        if (!space.physical_region.has_value()) continue;
        const auto& region = *space.physical_region;
        if (first) {
            bbox = region;
            first = false;
        } else {
            bbox.x1 = std::min(bbox.x1, region.x1);
            bbox.y1 = std::min(bbox.y1, region.y1);
            bbox.x2 = std::max(bbox.x2, region.x2);
            bbox.y2 = std::max(bbox.y2, region.y2);
        }
    }
    result.bbox = bbox;
    translate_asf_result(result, -result.bbox.x1, -result.bbox.y1);
    return result;
}

// 鎸夊師濮嬫ā鍧楅『搴忔暣鐞嗚緭鍑洪『搴忥紝淇濊瘉鏂囦欢绋冲畾銆?
std::vector<std::string> ordered_placements(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    std::vector<std::string> order;
    for (const auto& module : circuit.module_order) {
        if (request.placements.contains(module)) order.push_back(module);
    }
    return order;
}

// 浠庡寮?B*-tree 澶嶅埗 routing evaluator 鎵€闇€鐨勮交閲忔嫇鎵戝揩鐓с€?
RoutingTreeSnapshot make_routing_tree_snapshot(const EnhancedBStarTree& tree) {
    RoutingTreeSnapshot snapshot;
    snapshot.root = tree.root;
    for (const auto& [id, node] : tree.nodes) {
        std::string module = node.module;
        if (node.kind == BStarNodeKind::Hierarchy) {
            const auto* group = symmetry_group_for_hierarchy(tree, id);
            if (group != nullptr) {
                for (const auto& stored : group->stored_modules) {
                    if (!module.empty()) module += "|";
                    module += stored;
                }
            }
        }
        snapshot.nodes.push_back(RoutingTreeNodeRef{id, module, node.left, node.right});
    }
    return snapshot;
}

// 杩斿洖宸叉斁缃ā鍧楃殑 active blocker 杩戜技鍏ㄥ眬鐭╁舰銆?
// 鏀堕泦鎸囧畾 B*-tree node 瀛愭爲涓殑浠ｈ〃妯″潡锛岀敤浜庢妸 DP state 瀵归綈鍒?packing contour step銆?
std::vector<std::string> subtree_modules_for_trace(const EnhancedBStarTree& tree, const std::string& id) {
    std::vector<std::string> modules;
    const auto found = tree.nodes.find(id);
    if (found == tree.nodes.end()) return modules;
    if (found->second.kind == BStarNodeKind::Hierarchy) {
        const auto* group = symmetry_group_for_hierarchy(tree, id);
        if (group != nullptr) {
            modules.insert(modules.end(), group->stored_modules.begin(), group->stored_modules.end());
        }
    } else {
        modules.push_back(found->second.module.empty() ? id : found->second.module);
    }
    if (found->second.left.has_value()) {
        auto left = subtree_modules_for_trace(tree, *found->second.left);
        modules.insert(modules.end(), left.begin(), left.end());
    }
    if (found->second.right.has_value()) {
        auto right = subtree_modules_for_trace(tree, *found->second.right);
        modules.insert(modules.end(), right.begin(), right.end());
    }
    return modules;
}

void append_unique(std::vector<std::string>& values, const std::string& value) {
    if (value.empty()) return;
    if (std::find(values.begin(), values.end(), value) == values.end()) values.push_back(value);
}

std::string segment_key_for_trace(const WireSegmentRef& segment) {
    if (!segment.id.empty()) return segment.id;
    return segment.net + "|" + segment.from + "|" + segment.to;
}

std::unordered_map<std::string, std::string> lcp_owner_map_for_trace(const RoutingEvaluationRequest& request) {
    std::unordered_map<std::string, std::string> owners;
    std::unordered_map<std::string, std::string> owner_by_space;
    for (const auto& space : request.space_nodes) {
        owner_by_space[space.id] = space.owner;
        for (const auto& point : space.linking_points) owners[point.id] = space.owner;
    }
    for (const auto& point : request.linking_points) {
        const auto found = owner_by_space.find(point.space_node_id);
        if (found != owner_by_space.end()) owners[point.id] = found->second;
    }
    return owners;
}

std::string terminal_owner_for_trace(
    const std::string& terminal,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const auto dot = terminal.find('.');
    if (dot != std::string::npos) return terminal.substr(0, dot);
    const auto found = lcp_owner_by_id.find(terminal);
    return found == lcp_owner_by_id.end() ? std::string{} : found->second;
}

std::unordered_set<std::string> module_set_for_trace(const std::vector<std::string>& modules) {
    return {modules.begin(), modules.end()};
}

std::string raw_tree_id_for_trace(const std::string& id) {
    constexpr const char* kGlobalPrefix = "global:";
    const std::string prefix{kGlobalPrefix};
    if (id.rfind(prefix, 0) == 0) return id.substr(prefix.size());
    return id;
}

bool segment_inside_trace_modules(
    const WireSegmentRef& segment,
    const std::unordered_set<std::string>& modules,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    const auto from = terminal_owner_for_trace(segment.from, lcp_owner_by_id);
    const auto to = terminal_owner_for_trace(segment.to, lcp_owner_by_id);
    return !from.empty() && !to.empty() && modules.contains(from) && modules.contains(to);
}

bool segment_crosses_child_modules(
    const WireSegmentRef& segment,
    const std::unordered_set<std::string>& left,
    const std::unordered_set<std::string>& right,
    const std::unordered_map<std::string, std::string>& lcp_owner_by_id) {
    if (left.empty() || right.empty()) return false;
    const auto from = terminal_owner_for_trace(segment.from, lcp_owner_by_id);
    const auto to = terminal_owner_for_trace(segment.to, lcp_owner_by_id);
    return (left.contains(from) && right.contains(to)) || (left.contains(to) && right.contains(from));
}

// 鏍规嵁 packing step 鐨?subtree 杈圭晫鏄惧紡璁板綍璇?step 闇€瑕佸畬鎴愮殑 routing DP transition銆?
void annotate_packing_time_segments(const EnhancedBStarTree& tree, RoutingEvaluationRequest& request) {
    const auto lcp_owner_by_id = lcp_owner_map_for_trace(request);
    for (auto& step : request.packing_trace.steps) {
        const auto current_modules = module_set_for_trace(step.subtree_modules);
        const auto left_modules = step.left.has_value()
                                      ? module_set_for_trace(subtree_modules_for_trace(tree, raw_tree_id_for_trace(*step.left)))
                                      : std::unordered_set<std::string>{};
        const auto right_modules = step.right.has_value()
                                       ? module_set_for_trace(subtree_modules_for_trace(tree, raw_tree_id_for_trace(*step.right)))
                                       : std::unordered_set<std::string>{};
        for (const auto& topology : request.net_topologies) {
            for (const auto& segment : topology.segments) {
                if (!segment_inside_trace_modules(segment, current_modules, lcp_owner_by_id)) continue;
                if (segment_inside_trace_modules(segment, left_modules, lcp_owner_by_id)) continue;
                if (segment_inside_trace_modules(segment, right_modules, lcp_owner_by_id)) continue;
                const auto key = segment_key_for_trace(segment);
                append_unique(step.local_wire_segments, key);
                if (segment_crosses_child_modules(segment, left_modules, right_modules, lcp_owner_by_id)) {
                    append_unique(step.cross_child_wire_segments, key);
                }
            }
        }
    }
}

Rect placed_active_rect(const Module& module, const Placement& placement) {
    return routing::transform_active_to_global(module, placement);
}

// 杩斿洖褰撳墠 placement 涓嬫ā鍧楃殑澶栨帴鐭╁舰銆?
Rect placed_module_rect(const Circuit& circuit, const std::unordered_map<std::string, Placement>& placements, const std::string& module) {
    const auto placement = placements.find(module);
    if (placement == placements.end() || !circuit.modules.contains(module)) return {0.0, 0.0, 0.0, 0.0};
    const auto size = placed_size(circuit.modules.at(module), placement->second);
    return {placement->second.x, placement->second.y, placement->second.x + size.first, placement->second.y + size.second};
}

// 涓烘瘡涓?space node 鐢熸垚褰撳墠 placement 涓嬪彲閲囨牱鐨勭墿鐞嗛€氶亾鍖哄煙銆?
void assign_space_physical_regions(
    const Circuit& circuit,
    RoutingEvaluationRequest& request,
    const SolverConfig& config) {
    constexpr double kMinRegionWidth = 1.0;
    for (auto& space : request.space_nodes) {
        if (space.physical_region.has_value()) continue;
        if (space.owner.empty() || !request.placements.contains(space.owner) || !circuit.modules.contains(space.owner)) {
            space.physical_region = Rect{0.0, 0.0, kMinRegionWidth, kMinRegionWidth};
            continue;
        }
        const Rect owner = placed_module_rect(circuit, request.placements, space.owner);
        const double reserved_width = std::max({space.required_space(), config.spacing, kMinRegionWidth});
        if (space.kind == SpaceNodeKind::Right) {
            space.physical_region = Rect{owner.x2, owner.y1, owner.x2 + reserved_width, owner.y2};
        } else if (space.kind == SpaceNodeKind::Top) {
            space.physical_region = Rect{owner.x1, owner.y2, owner.x2, owner.y2 + reserved_width};
        } else {
            space.physical_region =
                Rect{owner.x1 - reserved_width, owner.y1 - reserved_width, owner.x2 + reserved_width, owner.y2 + reserved_width};
        }
    }
}

// 濉厖 routing request 涓潰鍚?DP/A* 鐨勫叏灞€ pin銆乥locker 鍜?LCP 鍊欓€変綅缃€?
void populate_routing_context(const Circuit& circuit, RoutingEvaluationRequest& request) {
    for (const auto& module_id : request.placement_order) {
        const auto& placement = request.placements.at(module_id);
        request.active_region_blockers.push_back(placed_active_rect(circuit.modules.at(module_id), placement));
    }
    for (const auto& key : circuit.pin_order) {
        const auto& pin = circuit.pins.at(key);
        const auto& placement = request.placements.at(pin.module);
        const auto xy = placed_pin(circuit.modules.at(pin.module), pin, placement);
        request.placed_pins.push_back({key, pin.module, pin.name, xy.first, xy.second, pin.layer});
    }
    const bool has_existing_lcp = std::any_of(request.space_nodes.begin(), request.space_nodes.end(), [](const SpaceNode& space) {
        return !space.linking_points.empty();
    });
    // packing 只刷新既有 LCP 的物理候选；初始 LCP 必须由 SA 前的初始化步骤建立。
    if (has_existing_lcp) refresh_lcp_location_candidates(circuit, request);
    std::unordered_map<std::string, NetTopology> topologies;
    for (const auto& [name, net] : circuit.nets) {
        topologies[name].net = name;
        topologies[name].pins = net.terminals;
    }
    for (const auto& point : request.linking_points) {
        std::unordered_set<std::string> emitted_segments;
        for (const auto& segment : point.segments) {
            if (!emitted_segments.insert(segment.id).second) continue;
            auto& topology = topologies[segment.net];
            topology.net = segment.net;
            if (std::none_of(topology.linking_points.begin(), topology.linking_points.end(), [&](const auto& existing) {
                    return existing.id == point.id;
                })) {
                topology.linking_points.push_back(point);
            }
            if (std::none_of(topology.segments.begin(), topology.segments.end(), [&](const auto& existing) {
                    return existing.id == segment.id;
                })) {
                topology.segments.push_back(segment);
            }
        }
    }
    for (auto& [_, topology] : topologies) {
        if (!topology.segments.empty()) request.net_topologies.push_back(std::move(topology));
    }
}

// 璁＄畻褰撳墠 metrics 鐨勮鏂囧綊涓€鍖栨€讳唬浠枫€?
// 灏?request 涓凡缁忕敓鎴愭垨鍒锋柊鐨?LCP 鍐欏洖 tree 鐨勫搴?space node銆?
void write_request_lcps_to_tree(EnhancedBStarTree& tree, const RoutingEvaluationRequest& request) {
    auto clear_space = [](SpaceNode& space) {
        space.linking_points.clear();
        space.location_candidates.clear();
    };
    for (auto& [id, node] : tree.nodes) {
        (void)id;
        if (node.kind == BStarNodeKind::Module) {
            clear_space(node.right_space);
            clear_space(node.top_space);
        }
    }
    for (auto& group : tree.symmetry_groups) {
        for (auto& [id, node] : group.asf_bstar_tree.nodes) {
            (void)id;
            for (auto& space_node_group : node.space_node_groups) {
                for (auto& space : space_node_group.spaces) clear_space(space);
            }
            if (node.space_node_cluster.has_value()) {
                for (auto& space : node.space_node_cluster->spaces) clear_space(space);
            }
        }
    }

    auto copy_if_matching = [](SpaceNode& target, const SpaceNode& source) {
        if (target.id != source.id) return;
        target.linking_points = source.linking_points;
        target.location_candidates = source.location_candidates;
    };
    for (const auto& source : request.space_nodes) {
        for (auto& [id, node] : tree.nodes) {
            (void)id;
            if (node.kind == BStarNodeKind::Module) {
                copy_if_matching(node.right_space, source);
                copy_if_matching(node.top_space, source);
            }
        }
        for (auto& group : tree.symmetry_groups) {
            for (auto& [id, node] : group.asf_bstar_tree.nodes) {
                (void)id;
                for (auto& space_node_group : node.space_node_groups) {
                    for (auto& space : space_node_group.spaces) copy_if_matching(space, source);
                }
                if (node.space_node_cluster.has_value()) {
                    for (auto& space : node.space_node_cluster->spaces) copy_if_matching(space, source);
                }
            }
        }
    }
}

// 根据当前 placement bbox 计算 row-width 溢出惩罚。
void apply_row_width_metrics(const Circuit& circuit, const RoutingEvaluationRequest& request, Metrics& metrics, const SolverConfig& config) {
    if (config.row_width <= 0.0 || request.placements.empty()) {
        metrics.row_width_overflow = 0.0;
        metrics.row_width_penalty = 0.0;
        return;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    for (const auto& [module, placement] : request.placements) {
        const auto size = placed_size(circuit.modules.at(module), placement);
        min_x = std::min(min_x, placement.x);
        max_x = std::max(max_x, placement.x + size.first);
    }
    const double width = std::max(0.0, max_x - min_x);
    metrics.row_width_overflow = std::max(0.0, width - config.row_width);
    metrics.row_width_penalty = config.row_width_weight * metrics.row_width_overflow / std::max(config.row_width, 1.0);
}

double compute_phi_cost(Metrics& metrics, const Metrics& base, const SolverConfig& config) {
    metrics.normalized_area = metrics.area / std::max(base.area, 1.0);
    metrics.normalized_wirelength = metrics.wirelength / std::max(base.wirelength, 1.0);
    metrics.normalized_bend = static_cast<double>(metrics.bend_count) / std::max(base.bend_count, 1);
    metrics.normalized_via = static_cast<double>(metrics.via_count) / std::max(base.via_count, 1);
    metrics.phi_cost = config.area_weight * metrics.normalized_area +
                       config.wirelength_weight * metrics.normalized_wirelength +
                       config.bend_weight * metrics.normalized_bend +
                       config.via_weight * metrics.normalized_via +
                       metrics.row_width_penalty +
                       metrics.penalty;
    return metrics.phi_cost;
}

// 璇勪环涓€妫靛寮?B*-tree 瀵瑰簲鐨勫€欓€夌姸鎬併€?
// 鍒ゆ柇褰撳墠鍊欓€夋槸鍚﹀凡缁忓緱鍒板彲鐩存帴杈撳嚭鐨勫悎娉?detailed routing銆?
bool feedback_expands_space(
    const RoutingEvaluationRequest& request,
    const RoutingFeedback& feedback,
    double tolerance) {
    for (const auto& space : request.space_nodes) {
        const auto required = feedback.required_space_by_node.find(space.id);
        if (required != feedback.required_space_by_node.end() &&
            required->second > space.allocated_space + tolerance) {
            return true;
        }
        const auto coupling = feedback.coupling_space_by_node.find(space.id);
        if (coupling != feedback.coupling_space_by_node.end() &&
            coupling->second > space.coupling_extra_space + tolerance) {
            return true;
        }
    }
    return false;
}

// 鍦ㄥ悓涓€涓?candidate 鍐呮墽琛屾湁闄愯疆 routing feedback -> re-pack 闂幆銆?
CandidateState evaluate_candidate_with_feedback_loop(
    const Circuit& circuit,
    EnhancedBStarTree tree,
    const SolverConfig& config,
    const Metrics* base_metrics) {
    CandidateState state;
    state.tree = std::move(tree);
    const int max_iterations = std::max(1, config.routing_feedback_iterations);
    bool converged = false;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        state.request = pack_enhanced_tree(circuit, state.tree, config);
        // 将 pack 阶段可能发生的几何失配重绑写回 tree，避免后续 SA 仍粘在旧 space。
        write_request_lcps_to_tree(state.tree, state.request);
        state.feedback = evaluate_with_routing_adapter(circuit, state.request);
        apply_row_width_metrics(circuit, state.request, state.feedback.metrics, config);
        const bool expands_space =
            feedback_expands_space(state.request, state.feedback, config.routing_feedback_tolerance);
        apply_routing_feedback(state.tree, state.feedback);
        state.feedback.metrics.routing_feedback_iterations = iteration + 1;
        state.feedback.metrics.routing_feedback_converged = !expands_space;
        if (!expands_space) {
            converged = true;
            break;
        }
    }
    state.feedback.metrics.routing_feedback_converged = converged;
    if (base_metrics != nullptr) state.cost = compute_phi_cost(state.feedback.metrics, *base_metrics, config);
    return state;
}

CandidateState evaluate_candidate(
    const Circuit& circuit,
    EnhancedBStarTree tree,
    const SolverConfig& config,
    const Metrics& base_metrics) {
    return evaluate_candidate_with_feedback_loop(circuit, std::move(tree), config, &base_metrics);
}

// 灏嗗€欓€夌姸鎬佽浆鎹负鏈€缁?Solution锛涘彲閫夐檮甯?SA 杩涘害涓庢瘡杞?btree 鍙鍖栬褰曘€?
Solution make_solution(
    const CandidateState& state,
    std::vector<SaProgressEntry> sa_progress = {},
    std::vector<SaBtreeIterationTrace> sa_btree_iterations = {}) {
    Solution solution;
    solution.placements = state.request.placements;
    solution.placement_order = state.request.placement_order;
    solution.routes = state.feedback.routes;
    solution.metrics = state.feedback.metrics;
    solution.routing_cost = state.feedback.routing_cost;
    solution.routing_candidate_count = state.feedback.routing_candidate_count;
    solution.detailed_route_count = static_cast<std::size_t>(state.feedback.metrics.detailed_routes);
    solution.traceback_failures = state.feedback.metrics.traceback_failures;
    solution.space_nodes_with_routes = state.feedback.metrics.space_nodes_with_routes;
    solution.dp_nodes = state.feedback.metrics.dp_nodes;
    solution.dp_states = state.feedback.metrics.dp_states;
    solution.dp_pruned_states = state.feedback.metrics.dp_pruned_states;
    solution.dp_traceback_segments = state.feedback.metrics.dp_traceback_segments;
    solution.packing_trace_steps = state.feedback.metrics.packing_trace_steps;
    solution.space_feedback_nodes = state.feedback.metrics.space_feedback_nodes;
    solution.routing_feedback_iterations = state.feedback.metrics.routing_feedback_iterations;
    solution.packing_time_dp_segments = state.feedback.metrics.packing_time_dp_segments;
    solution.routing_feedback_converged = state.feedback.metrics.routing_feedback_converged;
    solution.packing_time_dp_used = state.feedback.metrics.packing_time_dp_used;
    solution.dp_used = state.feedback.metrics.dp_used;
    solution.btree_trace_json = make_btree_trace_json(state.tree, state.request, state.feedback.metrics);
    solution.routing_debug_json = state.feedback.routing_debug_json;
    solution.routing_warnings = state.feedback.metrics.routing_warnings;
    solution.sa_progress = std::move(sa_progress);
    solution.sa_btree_iterations = std::move(sa_btree_iterations);
    return solution;
}

// 妫€鏌?FLOW 绾︽潫鏄惁鍦ㄥ綋鍓嶄复鏃惰矾鐢变腑鏈夊彲杩借釜鐨勭鐐广€?
int count_flow_violations(const Circuit& circuit, const RoutingEvaluationRequest& request, const std::vector<RouteSegment>& routes) {
    std::unordered_map<std::string, PlacedPin> pins;
    for (const auto& pin : request.placed_pins) pins[pin.key] = pin;
    int violations = 0;
    for (const auto& flow : circuit.constraints.flows) {
        if (!pins.contains(flow.out_pin) || !pins.contains(flow.in_pin)) {
            ++violations;
            continue;
        }
        bool has_route = false;
        for (const auto& route : routes) {
            if (route.net == flow.net) {
                has_route = true;
                break;
            }
        }
        if (!has_route) ++violations;
    }
    return violations;
}

// 妫€鏌ュ綋鍓?route 鏄惁杩濆弽绾垮鑼冨洿銆?
int count_current_density_violations(const Circuit& circuit, const std::vector<RouteSegment>& routes) {
    int violations = 0;
    for (const auto& route : routes) {
        const auto found = circuit.constraints.wire_widths.find(route.net);
        if (found == circuit.constraints.wire_widths.end()) continue;
        if (route.width < found->second.min_width || route.width > found->second.max_width) ++violations;
    }
    return violations;
}

double maximum_routing_width(const Circuit& circuit) {
    double width = 1.0;
    for (const auto& [_, rule] : circuit.constraints.wire_widths) {
        width = std::max(width, rule.max_width);
    }
    return width;
}

double resolve_boundary_margin(
    const Circuit& circuit,
    const SolverConfig& config,
    double layout_width,
    double layout_height) {
    if (config.boundary_clearance < 0.0) {
        throw std::runtime_error("boundary clearance must be non-negative");
    }
    if (config.boundary_margin >= 0.0) return config.boundary_margin;
    if (config.boundary_margin < -1.0) {
        throw std::runtime_error("boundary margin must be -1 for auto mode or non-negative");
    }

    const routing::GridConfig effective = routing::effective_grid_config_for_layout(
        circuit,
        routing::make_grid_config_for_routing_layers(config.routing_layers),
        layout_width,
        layout_height);
    const double pin_access_escape = 2.0 * effective.step;
    return maximum_routing_width(circuit) / 2.0 + config.boundary_clearance + pin_access_escape;
}

void translate_packing_request(RoutingEvaluationRequest& request, double offset) {
    if (offset == 0.0) return;
    for (auto& [_, placement] : request.placements) {
        placement.x += offset;
        placement.y += offset;
    }
    for (auto& step : request.packing_trace.steps) {
        step.x += offset;
        step.y += offset;
        step.desired_x += offset;
        step.desired_y += offset;
        step.contour_y += offset;
        step.occupied_bbox.x1 += offset;
        step.occupied_bbox.y1 += offset;
        step.occupied_bbox.x2 += offset;
        step.occupied_bbox.y2 += offset;
    }
    for (auto& space : request.space_nodes) {
        if (!space.physical_region.has_value()) continue;
        space.physical_region->x1 += offset;
        space.physical_region->y1 += offset;
        space.physical_region->x2 += offset;
        space.physical_region->y2 += offset;
    }
}

// 璁＄畻 space node 鍐?LCP 鎵€闇€鐨勬渶澶х嚎瀹斤紝鐢ㄤ簬 fallback 鍚庢墿澶ч鐣欑┖闂淬€?
double max_lcp_width_for_space(const SpaceNode& space) {
    double width = 1.0;
    for (const auto& point : space.linking_points) width = std::max(width, point.required_width());
    return width;
}

}  // namespace

// 在 SA 开始前按网表创建初始 LCP 拓扑；初始 packing 仅提供 space 几何用于确定初始归属。
void initialize_lcp_topology(const Circuit& circuit, EnhancedBStarTree& tree, const SolverConfig& config) {
    auto request = pack_enhanced_tree(circuit, tree, config);
    generate_initial_lcp_topology(circuit, request);
    write_request_lcps_to_tree(tree, request);
}

// 按增强 B*-tree 和 ASF 对称组生成当前候选布局。
RoutingEvaluationRequest pack_enhanced_tree(
    const Circuit& circuit,
    const EnhancedBStarTree& tree,
    const SolverConfig& config) {
    RoutingEvaluationRequest request;
    if (!tree.root.has_value()) return request;

    std::unordered_map<std::string, AsfPackingResult> asf_results;
    for (const auto& group : tree.symmetry_groups) {
        asf_results[group.hierarchy_node_id] = pack_asf_bstar_tree(circuit, group, config);
    }

    std::vector<PackedRect> packed;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, double, double)> place_node = [&](const std::string& id, double desired_x, double desired_y) {
        if (visited.contains(id)) return;
        visited.insert(id);
        const auto& node = tree.nodes.at(id);
        std::pair<double, double> occupied = occupied_size(circuit, tree, node, config);
        if (node.kind == BStarNodeKind::Hierarchy) {
            const auto found = asf_results.find(node.hierarchy_group);
            if (found != asf_results.end()) {
                occupied = {found->second.bbox.width(), found->second.bbox.height()};
            }
        }
        const double x = desired_x;
        const double contour_y = contour_height(packed, x, occupied.first);
        const double y = std::max(desired_y, contour_y);

        std::vector<std::string> subtree_modules = subtree_modules_for_trace(tree, id);
        if (node.kind == BStarNodeKind::Module) {
            Placement placement{node.module, x, y, node.angle, orient_for_angle(node.angle)};
            request.placements[node.module] = placement;
        } else {
            auto found = asf_results.find(node.hierarchy_group);
            if (found != asf_results.end()) {
                auto local = found->second;
                translate_asf_result(local, x, y);
                for (const auto& [module, placement] : local.placements) request.placements[module] = placement;
                request.space_nodes.insert(request.space_nodes.end(), local.space_nodes.begin(), local.space_nodes.end());
            }
        }

        std::optional<std::string> left_trace;
        std::optional<std::string> right_trace;
        if (node.left.has_value()) left_trace = *node.left;
        if (node.right.has_value()) right_trace = *node.right;
        request.packing_trace.steps.push_back(PackingContourStep{
            id,
            node.kind == BStarNodeKind::Module ? node.module : node.hierarchy_group,
            x,
            y,
            Rect{x, y, x + occupied.first, y + occupied.second},
            desired_x,
            desired_y,
            contour_y,
            node.kind == BStarNodeKind::Module ? node.right_space.required_space() : 0.0,
            node.kind == BStarNodeKind::Module ? node.top_space.required_space() : 0.0,
            node.kind == BStarNodeKind::Module ? node.right_space.coupling_extra_space + node.top_space.coupling_extra_space : 0.0,
            left_trace,
            right_trace,
            subtree_modules,
            {},
            {},
        });
        if (node.kind == BStarNodeKind::Hierarchy) {
            auto found = asf_results.find(node.hierarchy_group);
            if (found != asf_results.end()) {
                auto local = found->second;
                translate_asf_result(local, x, y);
                request.packing_trace.steps.insert(
                    request.packing_trace.steps.end(),
                    local.trace.steps.begin(),
                    local.trace.steps.end());
            }
        }
        packed.push_back({x, y, x + occupied.first, y + occupied.second});
        if (node.left.has_value()) {
            place_node(*node.left, x + occupied.first + config.spacing, y);
        }
        if (node.right.has_value()) {
            place_node(*node.right, x, y + occupied.second + config.spacing);
        }
    };

    place_node(*tree.root, 0.0, 0.0);
    double layout_width = 0.0;
    double layout_height = 0.0;
    for (const auto& rect : packed) {
        layout_width = std::max(layout_width, rect.x2);
        layout_height = std::max(layout_height, rect.y2);
    }
    for (const auto& id : tree.representative_order) {
        const auto& node = tree.nodes.at(id);
        if (node.kind != BStarNodeKind::Module) continue;
        request.space_nodes.push_back(node.right_space);
        request.space_nodes.push_back(node.top_space);
    }
    translate_packing_request(
        request,
        resolve_boundary_margin(circuit, config, layout_width, layout_height));
    request.placement_order = ordered_placements(circuit, request);
    request.lcp_candidate_seed = config.seed;
    request.strict_lcp_dp = config.strict_lcp_dp;
    request.allow_lcp_location_negotiation = config.negotiate_lcp_locations;
    request.routing_layers = config.routing_layers;
    assign_space_physical_regions(circuit, request, config);
    request.tree = make_routing_tree_snapshot(tree);
    populate_routing_context(circuit, request);
    annotate_packing_time_segments(tree, request);
    return request;
}

// 使用最终 RouteSegment 统一汇总布局布线指标，避免多层 detailed 路由内部统计与输出走线不一致。
RoutingFeedback evaluate_with_routing_adapter(const Circuit& circuit, const RoutingEvaluationRequest& request) {
    RoutingFeedback feedback;
    const auto routing_evaluation = evaluate_routing(circuit, request);
    const auto detailed = run_detailed_routing(circuit, request, routing_evaluation);
    feedback.routes = detailed.routes;
    Solution solution;
    solution.placements = request.placements;
    solution.placement_order = request.placement_order;
    solution.routes = feedback.routes;
    feedback.metrics = measure(circuit, solution);
    // 固化 global 阶段快照；有最终 detailed 路由时，最终几何指标保持为上面按 RouteSegment 重算的结果。
    feedback.metrics.global_wirelength = routing_evaluation.global_routing.total_metrics.wirelength;
    feedback.metrics.global_bend_count = routing_evaluation.global_routing.total_metrics.bend_count;
    feedback.metrics.global_via_count = routing_evaluation.global_routing.total_metrics.via_count;
    feedback.metrics.global_penalty = routing_evaluation.global_routing.total_penalty;
    if (detailed.routes.empty()) {
        feedback.metrics.wirelength = feedback.metrics.global_wirelength;
        feedback.metrics.bend_count = feedback.metrics.global_bend_count;
        feedback.metrics.via_count = feedback.metrics.global_via_count;
    }
    feedback.metrics.flow_violations = count_flow_violations(circuit, request, feedback.routes);
    feedback.metrics.current_density_violations = count_current_density_violations(circuit, feedback.routes);
    feedback.metrics.flow_violations += detailed.flow_violations;
    feedback.metrics.current_density_violations += detailed.current_density_violations;
    feedback.metrics.design_rule_violations = detailed.design_rule_violations;
    feedback.metrics.design_rule_penalty = detailed.design_rule_penalty;
    feedback.metrics.routing_failures = routing_evaluation.failed_nets;
    feedback.metrics.congestion_penalty = 0.0;
    feedback.metrics.flow_penalty = std::max(routing_evaluation.global_routing.flow_penalty, detailed.flow_penalty);
    feedback.metrics.current_density_penalty =
        std::max(routing_evaluation.global_routing.current_density_penalty, detailed.current_density_penalty);
    feedback.metrics.coupling_penalty =
        detailed.routes.empty() ? routing_evaluation.global_routing.coupling_penalty : detailed.coupling_penalty;
    feedback.metrics.routing_failure_penalty =
        routing_evaluation.global_routing.routing_failure_penalty + detailed.routing_failure_penalty;
    feedback.metrics.detailed_routing_penalty = detailed.design_rule_penalty + detailed.routing_failure_penalty;
    feedback.metrics.detailed_cost = detailed.detailed_cost;
    feedback.metrics.detailed_routes = static_cast<int>(detailed.routes.size());
    feedback.metrics.traceback_failures = detailed.traceback_failures;
    feedback.metrics.routing_warnings = detailed.report.warnings;
    for (const auto& net_route : routing_evaluation.global_routing.net_routes) {
        if (!net_route.success && !net_route.message.empty()) {
            feedback.metrics.routing_warnings.push_back(
                net_route.net + ": global routing failed: " + net_route.message);
        }
    }
    if (!request.linking_points.empty() && routing_evaluation.bottom_up_dp.has_value() && !routing_evaluation.bottom_up_dp->success) {
        feedback.metrics.routing_warnings.push_back(
            routing_evaluation.strict_lcp_dp_blocked_fallback
                ? "LCP strict DP blocked fallback: no_multi_terminal_reachable_lcp_candidate"
                : "LCP fallback: no_multi_terminal_reachable_lcp_candidate");
    }
    feedback.metrics.space_nodes_with_routes = detailed.space_nodes_with_routes;
    feedback.metrics.packing_trace_steps = static_cast<int>(request.packing_trace.steps.size());
    feedback.metrics.dp_used = routing_evaluation.used_bottom_up_dp;
    if (routing_evaluation.bottom_up_dp.has_value()) {
        feedback.metrics.dp_nodes = routing_evaluation.bottom_up_dp->dp_nodes;
        feedback.metrics.dp_states = routing_evaluation.bottom_up_dp->dp_states;
        feedback.metrics.dp_pruned_states = routing_evaluation.bottom_up_dp->dp_pruned_states;
        feedback.metrics.dp_traceback_segments = static_cast<int>(routing_evaluation.bottom_up_dp->traceback_candidates.size());
        feedback.metrics.packing_time_dp_segments = routing_evaluation.bottom_up_dp->packing_time_dp_segments;
        feedback.metrics.packing_time_dp_used = routing_evaluation.bottom_up_dp->packing_time_dp_used;
    }
    feedback.metrics.penalty =
        feedback.metrics.flow_penalty + feedback.metrics.current_density_penalty +
        feedback.metrics.coupling_penalty + feedback.metrics.design_rule_penalty +
        feedback.metrics.routing_failure_penalty;
    feedback.routing_cost = routing_evaluation.routing_cost;
    feedback.routing_candidate_count = routing_evaluation.candidates.size();

    int space_feedback_nodes = 0;
    const bool lcp_direct_fallback =
        !routing_evaluation.strict_lcp_dp_blocked_fallback &&
        !request.linking_points.empty() && routing_evaluation.bottom_up_dp.has_value() && !routing_evaluation.bottom_up_dp->success;
    for (const auto& space : request.space_nodes) {
        const auto detailed_space = detailed.required_space_by_node.find(space.id);
        // required_space_by_node 的语义是基础预留宽度，不能混入 coupling_extra_space。
        // 否则写回 allocated_space 后，下一轮 required_space() 会把同一 coupling 再加一次。
        double required_space = space.allocated_space;
        if (detailed_space != detailed.required_space_by_node.end()) {
            required_space = std::max(required_space, detailed_space->second);
        }
        if (lcp_direct_fallback && !space.linking_points.empty()) {
            required_space = std::max(
                required_space,
                std::max(0.0, space.required_space() - space.coupling_extra_space) + max_lcp_width_for_space(space));
        }
        feedback.required_space_by_node[space.id] = required_space;
        const auto detailed_coupling = detailed.coupling_space_by_node.find(space.id);
        const double coupling_space =
            detailed_coupling == detailed.coupling_space_by_node.end()
                ? (routing_evaluation.global_routing.coupling_penalty > 0.0 ? 1.0 : 0.0)
                : detailed_coupling->second;
        feedback.coupling_space_by_node[space.id] = coupling_space;
        const bool has_required_feedback =
            required_space > space.allocated_space + 1e-9;
        const bool has_coupling_feedback = coupling_space > 1e-9;
        if (has_required_feedback || has_coupling_feedback) ++space_feedback_nodes;
    }
    feedback.metrics.space_feedback_nodes = space_feedback_nodes;
    feedback.routing_debug_json = make_routing_debug_json(request, routing_evaluation, detailed, feedback.metrics);
    return feedback;
}

// 浣跨敤璁烘枃 placement 妗嗘灦銆丼A 鍜?routing adapter 鐢熸垚甯冨眬甯冪嚎瑙ｃ€?
Solution solve_placement_aware(const Circuit& circuit, const SolverConfig& config) {
    const auto errors = validate_circuit(circuit);
    if (!errors.empty()) {
        std::string message = "invalid circuit:";
        for (const auto& error : errors) message += "\n- " + error;
        throw std::runtime_error(message);
    }

    auto initial_tree = make_enhanced_tree(circuit);
    initialize_lcp_topology(circuit, initial_tree, config);
    if (config.debug_search) {
        std::cerr << "[search] init ordinary_right_child=" << (has_ordinary_right_child(initial_tree) ? "true" : "false")
                  << " tree_lcps=" << count_tree_lcps(initial_tree) << '\n';
    }
    auto current = evaluate_candidate_with_feedback_loop(circuit, std::move(initial_tree), config, nullptr);
    const Metrics base_metrics = current.feedback.metrics;
    current.cost = compute_phi_cost(current.feedback.metrics, base_metrics, config);
    CandidateState best = current;
    if (config.debug_search) {
        std::cerr << "[search] initial cost=" << current.cost
                  << " penalty=" << current.feedback.metrics.penalty
                  << " row_overflow=" << current.feedback.metrics.row_width_overflow
                  << " tree_lcps=" << count_tree_lcps(current.tree) << '\n';
    }
    if (config.sa_iterations <= 0) {
        return make_solution(best);
    }

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> probability(0.0, 1.0);
    double temperature = config.initial_temperature;
    std::vector<SaProgressEntry> sa_progress;
    sa_progress.reserve(static_cast<std::size_t>(config.sa_iterations));
    std::vector<SaBtreeIterationTrace> sa_btree_iterations;
    if (config.dump_sa_btree) sa_btree_iterations.reserve(static_cast<std::size_t>(config.sa_iterations));
    int stagnant_iterations = 0;
    bool sa_terminated_early = false;
    std::string sa_termination_reason;
    for (int iteration = 0; iteration < config.sa_iterations; ++iteration) {
        // 同一 SA 温度轮内重采样，避免不可执行的 LCP 操作消耗一次退火迭代。
        constexpr int max_perturbation_attempts = 8;
        auto next_tree = current.tree;
        PerturbationReport perturbation;
        for (int attempt = 0; attempt < max_perturbation_attempts; ++attempt) {
            next_tree = current.tree;
            perturbation = perturb_placement_tree(next_tree, rng);
            if (perturbation.changed) break;
        }
        auto next = evaluate_candidate(circuit, std::move(next_tree), config, base_metrics);
        const double delta = next.cost - current.cost;
        const bool accept = delta <= 0.0 || probability(rng) < std::exp(-delta / std::max(temperature, 1e-9));
        if (config.dump_sa_btree) {
            SaBtreeIterationTrace meta;
            meta.iteration = iteration + 1;
            meta.sa_iterations = config.sa_iterations;
            meta.move = perturbation.move;
            meta.changed = perturbation.changed;
            meta.accept = accept;
            meta.next_cost = next.cost;
            meta.current_cost_before = current.cost;
            meta.temperature = temperature;
            auto trace = make_sa_btree_iteration_trace(
                next.tree,
                next.request,
                next.feedback.metrics,
                meta);
            trace.placements = next.request.placements;
            trace.placement_order = next.request.placement_order;
            trace.routes = next.feedback.routes;
            sa_btree_iterations.push_back(std::move(trace));
        }
        const double logged_next_cost = next.cost;
        // accept 前先取出本轮候选的 feedback 收敛信息，避免 move 后丢失。
        const int next_feedback_iterations = next.feedback.metrics.routing_feedback_iterations;
        const bool next_feedback_converged = next.feedback.metrics.routing_feedback_converged;
        const int next_space_feedback_nodes = next.feedback.metrics.space_feedback_nodes;
        if (accept) current = std::move(next);
        const double best_improvement = best.cost - current.cost;
        if (best_improvement > config.sa_convergence_tolerance) {
            best = current;
            stagnant_iterations = 0;
        } else {
            if (current.cost < best.cost) best = current;
            if (config.sa_convergence_tolerance > 0.0 && config.sa_convergence_patience > 0) {
                ++stagnant_iterations;
            }
        }
        // 榛樿杈撳嚭杞婚噺 SA 杩涘害锛岄伩鍏嶉暱璇勪及鏃剁粓绔湅璧锋潵鍍忓崱浣忋€?
        std::cerr << "[sa] " << (iteration + 1) << '/' << config.sa_iterations
                  << " move=" << perturbation.move
                  << " accept=" << (accept ? "true" : "false")
                  << " next_cost=" << logged_next_cost
                  << " current_cost=" << current.cost
                  << " best_cost=" << best.cost
                  << '\n'
                  << std::flush;
        SaProgressEntry progress;
        progress.iteration = iteration + 1;
        progress.sa_iterations = config.sa_iterations;
        progress.move = perturbation.move;
        progress.changed = perturbation.changed;
        progress.accept = accept;
        progress.next_cost = logged_next_cost;
        progress.current_cost = current.cost;
        progress.best_cost = best.cost;
        progress.temperature = temperature;
        progress.routing_feedback_iterations = next_feedback_iterations;
        progress.routing_feedback_converged = next_feedback_converged;
        progress.space_feedback_nodes = next_space_feedback_nodes;
        sa_progress.push_back(std::move(progress));
        if (config.debug_search) {
            std::cerr << "[search] iter=" << iteration
                      << " move=" << perturbation.move
                      << " changed=" << (perturbation.changed ? "true" : "false")
                      << " lcp_move=" << (perturbation.used_lcp_move ? "true" : "false")
                      << " lcp_before=" << perturbation.lcp_before
                      << " lcp_after=" << perturbation.lcp_after
                      << " accept=" << (accept ? "true" : "false")
                      << " next_cost=" << logged_next_cost
                      << " current_cost=" << current.cost
                      << " best_cost=" << best.cost
                      << " penalty=" << current.feedback.metrics.penalty
                      << " row_overflow=" << current.feedback.metrics.row_width_overflow
                      << '\n';
        }
        if (config.sa_convergence_tolerance > 0.0 && config.sa_convergence_patience > 0 &&
            stagnant_iterations >= config.sa_convergence_patience) {
            sa_terminated_early = true;
            sa_termination_reason = "best-cost improvement stayed within " +
                std::to_string(config.sa_convergence_tolerance) + " for " +
                std::to_string(config.sa_convergence_patience) + " consecutive iterations";
            std::cerr << "[sa] early-stop: " << sa_termination_reason << '\n' << std::flush;
            break;
        }
        temperature *= config.cooling_rate;
    }

    auto solution = make_solution(best, std::move(sa_progress), std::move(sa_btree_iterations));
    solution.sa_terminated_early = sa_terminated_early;
    solution.sa_termination_reason = std::move(sa_termination_reason);
    return solution;
}

// 淇濇寔鍘嗗彶 CLI/API 鍚嶇О锛屽綋鍓嶉粯璁ゆ寚鍚戣鏂?placement-aware 姹傝В娴佺▼銆?
Solution solve_baseline(const Circuit& circuit, const SolverConfig& config) {
    return solve_placement_aware(circuit, config);
}

}  // namespace sapr
