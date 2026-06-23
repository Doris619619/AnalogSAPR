// 文件职责：定义 AnalogSAPR C++ 阶段 0 使用的核心电路、布局和布线数据结构。
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace analog_sapr {

// 表示二维矩形区域，用于器件边界、有源区和后续障碍物建模。
struct Rect {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

// 表示输入 modules.txt 中的一个器件及其有源区和 cell 原点偏移。
struct Module {
    std::string id;
    double width = 0.0;
    double height = 0.0;
    Rect active;
    double ox = 0.0;
    double oy = 0.0;
    std::string info;
};

// 表示输入 pins.txt 中的一个管脚，坐标相对器件 BB 左下角。
struct Pin {
    std::string module;
    std::string name;
    double x = 0.0;
    double y = 0.0;
    std::string layer;
};

// 表示输入 nets.txt 中的一条线网及其终端列表。
struct Net {
    std::string name;
    std::string priority;
    std::vector<std::string> terminals;
};

// 表示 SYMMETRY_PAIR 约束。
struct SymmetryPair {
    std::string name;
    std::string axis;
    std::string left;
    std::string right;
};

// 表示 SYMMETRY_SELF 约束。
struct SymmetrySelf {
    std::string name;
    std::string axis;
    std::string module;
};

// 表示 FLOW 电流方向约束。
struct FlowConstraint {
    std::string net;
    std::string out_pin;
    std::string in_pin;
};

// 表示 WIRE_WIDTH 线宽范围约束。
struct WireWidthConstraint {
    std::string net;
    double min_width = 0.0;
    double max_width = 0.0;
};

// 汇总所有约束，后续 checker 和 router 会复用该结构。
struct Constraints {
    std::vector<SymmetryPair> symmetry_pairs;
    std::vector<SymmetrySelf> symmetry_selfs;
    std::vector<FlowConstraint> flows;
    std::unordered_map<std::string, WireWidthConstraint> wire_widths;
};

// 表示完整输入电路模型。
struct Circuit {
    std::unordered_map<std::string, Module> modules;
    std::unordered_map<std::string, Pin> pins;
    std::unordered_map<std::string, Net> nets;
    Constraints constraints;
};

// 表示 output/placement.txt 中的一条器件布局记录。
struct Placement {
    std::string module;
    double x = 0.0;
    double y = 0.0;
    int angle = 0;
    std::string orient = "R0";
};

// 表示 output/routing.txt 中的一条中心线布线段。
struct RouteSegment {
    std::string net;
    std::string layer;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    double width = 0.0;
};

// 表示阶段 0 的布线求解输出。
struct RoutingSolution {
    std::unordered_map<std::string, Placement> placements;
    std::vector<RouteSegment> routes;
};

}  // namespace analog_sapr
