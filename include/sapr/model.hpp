// 瀹氫箟妯℃嫙鐢佃矾銆佺害鏉熴€佸寮?B*-tree銆佸竷灞€甯冪嚎缁撴灉鍜岃瘎浠锋寚鏍囩殑鍏叡鏁版嵁妯″瀷銆?
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace sapr {

// 琛ㄧず绾跨綉鐨勫竷绾夸紭鍏堢骇銆?
enum class Priority { Critical, Symmetry, Normal };

// 琛ㄧず瀵圭О绾︽潫鐨勮酱鏂瑰悜銆?
enum class Axis { Vertical, Horizontal };

// 琛ㄧず澧炲己 B*-tree 涓鐣欏竷绾跨┖闂寸殑璁烘枃璇箟銆?
enum class SpaceNodeKind { Right, Top, Group, Cluster };

// 琛ㄧず LCP 绾挎鐩稿 FLOW 绾︽潫鐨勯€昏緫鐢垫祦鏂瑰悜銆?
enum class CurrentDirection { Unknown, In, Out };

// 琛ㄧず浜岀淮杞村榻愮煩褰€?
struct Rect {
    double x1{};
    double y1{};
    double x2{};
    double y2{};

    // 杩斿洖鐭╁舰瀹藉害銆?
    [[nodiscard]] double width() const { return x2 - x1; }
    // 杩斿洖鐭╁舰楂樺害銆?
    [[nodiscard]] double height() const { return y2 - y1; }
};

// 琛ㄧず涓€涓緟鏀剧疆鍣ㄤ欢銆?
struct Module {
    std::string id;
    double width{};
    double height{};
    Rect active;
    double ox{};
    double oy{};
    std::string info;
};

// 琛ㄧず鍣ㄤ欢涓婄殑涓€涓紩鑴氥€?
struct Pin {
    std::string module;
    std::string name;
    double x{};
    double y{};
    std::string layer;

    // 杩斿洖鍏ㄥ眬鍞竴鐨?module.pin 閿€?
    [[nodiscard]] std::string key() const { return module + "." + name; }
};

// 琛ㄧず涓€涓绔嚎缃戙€?
struct Net {
    std::string name;
    Priority priority{Priority::Normal};
    std::vector<std::string> terminals;
};

// 琛ㄧず涓€瀵瑰櫒浠剁殑瀵圭О绾︽潫銆?
struct SymmetryPair {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string left;
    std::string right;
};

// 琛ㄧず鍣ㄤ欢鑷韩鐨勫绉扮害鏉熴€?
struct SymmetrySelf {
    std::string name;
    Axis axis{Axis::Vertical};
    std::string module;
};

// 琛ㄧず绾跨綉涓婄殑鐢垫祦娴佸悜绾︽潫銆?
struct FlowConstraint {
    std::string net;
    std::string out_pin;
    std::string in_pin;
};

// 琛ㄧず绾跨綉鍏佽鐨勭嚎瀹借寖鍥淬€?
struct WireWidthConstraint {
    std::string net;
    double min_width{};
    double max_width{};
};

// 姹囨€荤數璺殑鍏ㄩ儴绾︽潫銆?
struct Constraints {
    std::vector<SymmetryPair> symmetry_pairs;
    std::vector<SymmetrySelf> symmetry_selfs;
    std::unordered_map<std::string, WireWidthConstraint> wire_widths;
    std::vector<FlowConstraint> flows;
};

// 琛ㄧず瀹屾暣鐨勭畻娉曡緭鍏ワ紝骞朵繚瀛樼ǔ瀹氱殑鍘熷椤哄簭銆?
struct Circuit {
    std::unordered_map<std::string, Module> modules;
    std::unordered_map<std::string, Pin> pins;
    std::unordered_map<std::string, Net> nets;
    Constraints constraints;
    std::vector<std::string> module_order;
    std::vector<std::string> pin_order;
    std::vector<std::string> net_order;
};

// 琛ㄧず涓€涓櫒浠剁殑鏀剧疆缁撴灉銆?
struct Placement {
    std::string module;
    double x{};
    double y{};
    int angle{};
    std::string orient{"R0"};
};

// 琛ㄧず涓€鏉′腑蹇冪嚎甯冪嚎娈点€?
struct RouteSegment {
    std::string net;
    std::string layer;
    double x1{};
    double y1{};
    double x2{};
    double y2{};
    double width{};
};

// 琛ㄧず褰撳墠瑙ｇ殑鍩虹璇勪环鎸囨爣鍜岃鏂囩害鏉熻繚渚嬬粺璁°€?
// wirelength/bend/via/penalty 涓烘渶缁堝彛寰勶紙浼樺厛 detailed锛夛紱global_* 淇濈暀 global 闃舵蹇収锛岄伩鍏嶄笌鏈€缁堝€兼贩鐢ㄣ€?
struct Metrics {
    double area{};
    double wirelength{};
    int bend_count{};
    int via_count{};
    double phi_cost{};
    double normalized_area{};
    double normalized_wirelength{};
    double normalized_bend{};
    double normalized_via{};
    double penalty{};
    // global 闃舵鍘熷鍑犱綍涓庢儵缃氾紙鍐欏叆 metrics 鍓嶄笉琚?detailed 瑕嗙洊锛夈€?
    double global_wirelength{};
    int global_bend_count{};
    int global_via_count{};
    double global_penalty{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double routing_failure_penalty{};
    double design_rule_penalty{};
    double detailed_routing_penalty{};
    double detailed_cost{};
    double row_width_overflow{};
    double row_width_penalty{};
    int flow_violations{};
    int current_density_violations{};
    int design_rule_violations{};
    int routing_failures{};
    int detailed_routes{};
    int traceback_failures{};
    int space_nodes_with_routes{};
    int dp_nodes{};
    int dp_states{};
    int dp_pruned_states{};
    int dp_traceback_segments{};
    int packing_trace_steps{};
    int space_feedback_nodes{};
    int routing_feedback_iterations{};
    int packing_time_dp_segments{};
    bool routing_feedback_converged{};
    bool packing_time_dp_used{};
    bool dp_used{};
    double congestion_penalty{};
    std::vector<std::string> routing_warnings;
};

// 璁板綍 SA 鍗曡疆杞婚噺杩涘害锛屽瓧娈典笌缁堢 `[sa]` 鏃ュ織瀵归綈銆?
struct SaProgressEntry {
    int iteration{};
    int sa_iterations{};
    std::string move;
    // 标记本轮记录的扰动是否真实改变了候选树，便于识别空扰动。
    bool changed{};
    bool accept{};
    double next_cost{};
    double current_cost{};
    double best_cost{};
    double temperature{};
    // 本轮候选解内部 routing feedback 闭环的收敛摘要。
    int routing_feedback_iterations{};
    bool routing_feedback_converged{};
    int space_feedback_nodes{};
};

// 璁板綍 SA 鍗曡疆鍊欓€夋爲鍙鍖栨墍闇€鐨勬壈鍔ㄤ笌鎺ュ彈淇℃伅銆?
struct SaBtreeIterationTrace {
    int iteration{};
    int sa_iterations{};
    std::string move;
    bool changed{};
    bool accept{};
    double next_cost{};
    double current_cost_before{};
    double temperature{};
    std::string btree_trace_json;
    // 淇濆瓨鏈疆鍊欓€夎В鐨勫竷灞€甯冪嚎鏂囨湰杈撳嚭鎵€闇€鏁版嵁锛岀敤浜庨€愯疆娓叉煋 layout銆?
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<RouteSegment> routes;
};

// 姹囨€诲竷灞€鍜屽竷绾跨粨鏋滐紝骞跺彲淇濆瓨姹傝В鏃剁殑璺敱璇勪环蹇収銆?
struct Solution {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<RouteSegment> routes;
    std::optional<Metrics> metrics;
    std::optional<double> routing_cost;
    std::optional<std::size_t> routing_candidate_count;
    std::optional<std::size_t> detailed_route_count;
    std::optional<int> traceback_failures;
    std::optional<int> space_nodes_with_routes;
    std::optional<int> dp_nodes;
    std::optional<int> dp_states;
    std::optional<int> dp_pruned_states;
    std::optional<int> dp_traceback_segments;
    std::optional<int> packing_trace_steps;
    std::optional<int> space_feedback_nodes;
    std::optional<int> routing_feedback_iterations;
    std::optional<int> packing_time_dp_segments;
    std::optional<bool> routing_feedback_converged;
    std::optional<bool> packing_time_dp_used;
    std::optional<bool> dp_used;
    std::optional<std::string> btree_trace_json;
    std::optional<std::string> routing_debug_json;
    std::vector<std::string> routing_warnings;
    // 涓?btree_trace.json 骞跺垪鍐欏嚭鐨?SA 杞婚噺杩涘害锛坰a_trace.json锛夈€?
    std::vector<SaProgressEntry> sa_progress;
    std::vector<SaBtreeIterationTrace> sa_btree_iterations;
    // 记录 SA 是否因收敛准则提前结束，避免输出将计划迭代数误认为实际迭代数。
    bool sa_terminated_early{};
    std::string sa_termination_reason;
};

// 閰嶇疆姹傝В鍣ㄧ殑纭畾鎬у弬鏁板拰璁烘枃浠ｄ环鍑芥暟鏉冮噸銆?
struct SolverConfig {
    double spacing{5.0};
    double row_width{40.0};
    unsigned int seed{1};
    int sa_iterations{250};
    double initial_temperature{5.0};
    double cooling_rate{0.96};
    // 最优代价改善不超过该阈值时计为无显著改善；设为非正值可关闭提前停止。
    double sa_convergence_tolerance{1e-6};
    // 连续无显著改善达到该轮数后提前停止；设为非正值可关闭提前停止。
    int sa_convergence_patience{20};
    double area_weight{1.0};
    double wirelength_weight{1.0};
    double bend_weight{0.2};
    double via_weight{0.2};
    double row_width_weight{1.0};
    int routing_feedback_iterations{2};
    double routing_feedback_tolerance{1e-6};
    bool debug_search{};
    double boundary_margin{-1.0};
    double boundary_clearance{0.0};
    bool dump_sa_btree{true};
    // 仅用于论文 LCP/对称布线诊断；DP 失败时禁止含 LCP net 退回普通 greedy 布线。
    bool strict_lcp_dp{false};
    // 在写入 detailed route 前协商整网 LCP 物理位置，避免 DP 位置不可实现时只留下失败罚分。
    bool negotiate_lcp_locations{true};
    // 允许使用的金属层数（M1..Mn）；默认两层以对齐论文实验设置并避免单层短路。
    int routing_layers{2};
};

// 琛ㄧず涓€娆?SA 鎵板姩鐨勮皟璇曟憳瑕侊紝渚涘懡浠よ璇婃柇鎼滅储鐘舵€佹槸鍚︾湡瀹炲彉鍖栥€?
struct PerturbationReport {
    std::string move;
    bool changed{};
    bool used_lcp_move{};
    std::size_t lcp_before{};
    std::size_t lcp_after{};
};

// 琛ㄧず linking-control point 杩炴帴鐨勪竴鏉￠€昏緫绾挎銆?
struct WireSegmentRef {
    std::string net;
    std::string from;
    std::string to;
    double min_width{};
    double max_width{};
    std::optional<std::string> direction;
    CurrentDirection current_direction{CurrentDirection::Unknown};
    std::string id;
};

// 琛ㄧず linking-control point 鍦ㄦ墍灞?space node 鍐呯殑鍊欓€夌墿鐞嗕綅缃€?
struct PhysicalLocationCandidate {
    double x{};
    double y{};
    std::string id;
    std::string validity_level{"strict"};
    bool is_fallback{};
    double penalty{};
    std::string reason;
    std::string source;
    bool inside_space_region{};
};

// 琛ㄧず澧炲己 B*-tree 涓殑鎷撴墤鎺у埗鐐广€?
struct LinkingControlPoint {
    std::string id;
    std::string space_node_id;
    std::vector<WireSegmentRef> segments;
    std::vector<PhysicalLocationCandidate> location_candidates;

    // 杩斿洖璇ユ帶鍒剁偣闇€瑕佺殑鏈€澶х嚎瀹姐€?
    [[nodiscard]] double required_width() const;
};

// 琛ㄧず鍣ㄤ欢鍙充晶銆佷笂渚ф垨瀵圭О缁勫唴鐨勯鐣欏竷绾跨┖闂淬€?
struct SpaceNode {
    std::string id;
    std::string owner;
    SpaceNodeKind kind{SpaceNodeKind::Right};
    std::vector<LinkingControlPoint> linking_points;
    double allocated_space{};
    std::vector<PhysicalLocationCandidate> location_candidates;
    double coupling_extra_space{};
    std::optional<Rect> physical_region;

    // 按论文公式返回 LCP 本身需要的 routing resource，不包含 feedback 下界。
    [[nodiscard]] double formula_required_space() const;
    [[nodiscard]] double required_space() const;
};

// 琛ㄧず ASF 瀵圭О缁勪腑鐨勪竴缁勯暅鍍忔垨杞撮棿 space node銆?
struct SpaceNodeGroup {
    std::string name;
    std::vector<SpaceNode> spaces;
};

struct SpaceNodeCluster {
    std::string name;
    std::vector<SpaceNode> spaces;
};

enum class BStarNodeKind { Module, Hierarchy };

// 琛ㄧず澧炲己 B*-tree 鐨勪竴涓櫒浠惰妭鐐广€?
struct BStarNode {
    std::string id;
    BStarNodeKind kind{BStarNodeKind::Module};
    std::string module;
    std::string hierarchy_group;
    std::optional<std::string> parent;
    std::optional<std::string> left;
    std::optional<std::string> right;
    int angle{};
    SpaceNode right_space;
    SpaceNode top_space;
};

struct AsfBStarNode {
    std::string module;
    std::optional<std::string> mirror_module;
    bool is_self_symmetric{};
    std::optional<std::string> parent;
    std::optional<std::string> left;
    std::optional<std::string> right;
    int angle{};
    std::vector<SpaceNodeGroup> space_node_groups;
    std::optional<SpaceNodeCluster> space_node_cluster;
};

struct AsfBStarTree {
    std::string group_name;
    Axis axis{Axis::Vertical};
    std::optional<std::string> root;
    std::unordered_map<std::string, AsfBStarNode> nodes;
    std::vector<std::string> representative_order;
    std::unordered_map<std::string, std::string> mirror_map;
    std::vector<std::string> self_nodes;
    std::vector<std::string> right_most_branch;
};

// 琛ㄧず涓€涓?ASF-B*-tree 瀵圭О缁勫湪涓绘爲涓殑灞傛绾︽潫鎽樿銆?
struct SymmetryGroupNode {
    std::string name;
    Axis axis{Axis::Vertical};
    AsfBStarTree asf_bstar_tree;
    std::string hierarchy_node_id;
    std::vector<std::string> stored_modules;
};

// 琛ㄧず涓€鏉?net 鍦ㄥ寮?B*-tree 涓殑 LCP 鎷撴墤鎽樿銆?
struct NetTopology {
    std::string net;
    std::vector<std::string> pins;
    std::vector<LinkingControlPoint> linking_points;
    std::vector<WireSegmentRef> segments;
};

// 琛ㄧず routing evaluator 鎵€闇€鐨勮交閲?B*-tree 鑺傜偣蹇収銆?
struct RoutingTreeNodeRef {
    std::string id;
    std::string module;
    std::optional<std::string> left;
    std::optional<std::string> right;
};

// 琛ㄧず褰撳墠 placement candidate 瀵瑰簲鐨?B*-tree 鎷撴墤蹇収銆?
struct RoutingTreeSnapshot {
    std::optional<std::string> root;
    std::vector<RoutingTreeNodeRef> nodes;
};

// 琛ㄧず璺敱璇勪环鏃跺凡缁忓睍寮€鍒板叏灞€鍧愭爣鐨勫紩鑴氥€?
struct PlacedPin {
    std::string key;
    std::string module;
    std::string pin;
    double x{};
    double y{};
    std::string layer;
};

// 琛ㄧず涓€娆″€欓€夊竷灞€鐨勮矾鐢变晶璇勪环杈撳叆銆?
// 琛ㄧず涓€娆?contour packing 涓煇涓?B*-tree node 鐨勪腑闂寸姸鎬併€?
struct PackingContourStep {
    std::string tree_node;
    std::string module;
    double x{};
    double y{};
    Rect occupied_bbox;
    double desired_x{};
    double desired_y{};
    double contour_y{};
    double right_space{};
    double top_space{};
    double coupling_extra_space{};
    std::optional<std::string> left;
    std::optional<std::string> right;
    std::vector<std::string> subtree_modules;
    std::vector<std::string> local_wire_segments;
    std::vector<std::string> cross_child_wire_segments;
};

// 琛ㄧず褰撳墠鍊欓€夊竷灞€鐨?contour packing 杩囩▼蹇収銆?
struct PackingContourTrace {
    std::vector<PackingContourStep> steps;
};

struct RoutingEvaluationRequest {
    std::unordered_map<std::string, Placement> placements;
    std::vector<std::string> placement_order;
    std::vector<SpaceNode> space_nodes;
    std::vector<PlacedPin> placed_pins;
    std::vector<LinkingControlPoint> linking_points;
    std::vector<NetTopology> net_topologies;
    std::vector<Rect> active_region_blockers;
    RoutingTreeSnapshot tree;
    PackingContourTrace packing_trace;
    unsigned int lcp_candidate_seed{};
    // strict 模式下，含 LCP topology 的 net 必须由 bottom-up DP traceback 接管。
    bool strict_lcp_dp{};
    // 仅 placement-aware 流程开启：允许在 detailed route 提交前为整网显式重选同一 LCP 位置。
    bool allow_lcp_location_negotiation{};
    // 传入 RoutingContext::GridConfig.layer_count，统一限制 A*/obstacle/detailed reroute。
    int routing_layers{1};
};

// 琛ㄧず璺敱 adapter 杩斿洖缁?placement/SA 鐨勫弽棣堛€?
struct RoutingFeedback {
    std::vector<RouteSegment> routes;
    Metrics metrics;
    // 不含 coupling_extra_space 的基础 routing 预留宽度，写回 SpaceNode::allocated_space。
    std::unordered_map<std::string, double> required_space_by_node;
    // 独立于基础预留宽度的耦合附加空间，写回 SpaceNode::coupling_extra_space。
    std::unordered_map<std::string, double> coupling_space_by_node;
    double routing_cost{};
    std::size_t routing_candidate_count{};
    std::optional<std::string> routing_debug_json;
};

// 琛ㄧず detailed routing 鍥炴函涓殑涓€涓嫇鎵戣妭鐐广€?
struct DetailedRouteNode {
    std::string id;
    std::string kind;
    std::string space_node_id;
    double x{};
    double y{};
    std::string layer;
};

// 琛ㄧず detailed routing 杈撳嚭绾挎涓?LCP/space-node 鎷撴墤鐨勬槧灏勫叧绯汇€?
struct DetailedRouteSegment {
    std::size_t route_index{};
    int dp_state_id{-1};
    std::string net;
    std::string from_terminal;
    std::string to_terminal;
    std::string tree_node;
    std::string segment_id;
    std::string lcp_id;
    std::string lcp_candidate_id;
    std::string space_node_id;
};

// 琛ㄧず涓€鏉?net 鐨?top-down detailed routing 鍥炴函鎽樿銆?
struct DetailedRouteTrace {
    std::string net;
    std::vector<DetailedRouteNode> nodes;
    std::vector<DetailedRouteSegment> segments;
    std::vector<std::string> warnings;
};

// 姹囨€?detailed routing 鐨勫彲瑙ｉ噴鎶ュ憡銆?
struct DetailedRoutingReport {
    std::vector<DetailedRouteTrace> traces;
    std::vector<std::string> warnings;
    std::vector<std::string> coupling_pairs;
    std::vector<std::string> design_rule_segments;
    // 璁板綍璇︾粏甯冪嚎闃舵杩濆弽 FLOW 鏂瑰悜鐨?net 鎴?segment銆?
    std::vector<std::string> flow_segments;
    // 璁板綍璇︾粏甯冪嚎闃舵杩濆弽 WIRE_WIDTH/current-density 浠ｇ悊绾︽潫鐨?segment銆?
    std::vector<std::string> current_density_segments;
};

// 琛ㄧず top-down performance-aware detailed routing 鐨勮緭鍑恒€?
struct DetailedRoutingResult {
    // 保存 detailed 阶段已提交、但可能因最终全局 DRC 被拒绝的原始金属段，仅供 routing_debug.json 诊断。
    std::vector<RouteSegment> raw_routes;
    // 仅保存通过最终 DRC 的可交付金属段，作为 routing.txt 的唯一来源。
    std::vector<RouteSegment> routes;
    DetailedRoutingReport report;
    std::unordered_map<std::string, double> required_space_by_node;
    std::unordered_map<std::string, double> coupling_space_by_node;
    double detailed_wirelength{};
    int detailed_bend_count{};
    int detailed_via_count{};
    double detailed_cost{};
    double flow_penalty{};
    double current_density_penalty{};
    double coupling_penalty{};
    double design_rule_penalty{};
    double routing_failure_penalty{};
    double detailed_routing_penalty{};
    int flow_violations{};
    int current_density_violations{};
    int design_rule_violations{};
    int traceback_failures{};
    int space_nodes_with_routes{};
    bool used_global_fallback{};
};

// 琛ㄧず褰撳墠闃舵鐨勫寮?B*-tree 鎷撴墤銆?
struct EnhancedBStarTree {
    std::optional<std::string> root;
    std::unordered_map<std::string, BStarNode> nodes;
    std::vector<std::string> representative_order;
    std::vector<SymmetryGroupNode> symmetry_groups;
};

}  // namespace sapr
