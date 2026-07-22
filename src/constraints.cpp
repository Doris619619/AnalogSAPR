// 实现输入引用关系和当前已支持结果约束的校验。
#include "sapr/constraints.hpp"

#include <unordered_map>
#include <unordered_set>

#include "sapr/routing/layer.hpp"

namespace sapr {

namespace {

// 按具体层优先、ALL 兜底查询 PDK 按层数值规则。
double layer_rule(const std::unordered_map<std::string, double>& rules, const std::string& layer) {
    if (const auto found = rules.find(layer); found != rules.end()) return found->second;
    if (const auto fallback = rules.find("ALL"); fallback != rules.end()) return fallback->second;
    return 0.0;
}

// 按具体层优先、ALL 兜底查询 active 穿越开关。
bool block_rule(const std::unordered_map<std::string, bool>& rules, const std::string& layer) {
    if (const auto found = rules.find(layer); found != rules.end()) return found->second;
    if (const auto fallback = rules.find("ALL"); fallback != rules.end()) return fallback->second;
    return true;
}

}  // namespace

// 返回器件之间必须保留的最小边缘距离。
double device_spacing(const Circuit& circuit) {
    return circuit.constraints.spacing_rules.device_spacing;
}

// 返回指定层 active 与布线的最小边缘距离；允许跨越时豁免。
double active_route_spacing(const Circuit& circuit, const std::string& layer) {
    return active_region_blocked(circuit, layer) ? layer_rule(circuit.constraints.spacing_rules.active_route_spacing, layer) : 0.0;
}

// 返回指定层异网金属之间的最小边缘距离。
double diff_net_route_spacing(const Circuit& circuit, const std::string& layer) {
    return layer_rule(circuit.constraints.spacing_rules.diff_net_route_spacing, layer);
}

// 判断指定层是否禁止穿越 active region。
bool active_region_blocked(const Circuit& circuit, const std::string& layer) {
    return block_rule(circuit.constraints.spacing_rules.active_region_block, layer);
}

// 校验输入对象之间的引用和数值范围。
std::vector<std::string> validate_circuit(const Circuit& circuit) {
    std::vector<std::string> errors;
    std::unordered_map<std::string, std::string> terminal_net;
    for (const auto& key : circuit.pin_order) {
        const auto& pin = circuit.pins.at(key);
        if (!circuit.modules.contains(pin.module)) errors.push_back("pin " + key + " references missing module " + pin.module);
    }
    for (const auto& name : circuit.net_order) {
        const auto& net = circuit.nets.at(name);
        for (const auto& terminal : net.terminals) {
            if (!circuit.pins.contains(terminal)) errors.push_back("net " + name + " references missing pin " + terminal);
            const auto [existing, inserted] = terminal_net.emplace(terminal, name);
            if (!inserted && existing->second != name) {
                errors.push_back("terminal " + terminal + " is assigned to multiple nets: " + existing->second + " and " + name);
            }
        }
    }
    for (const auto& pair : circuit.constraints.symmetry_pairs) {
        if (!circuit.modules.contains(pair.left)) errors.push_back("symmetry pair " + pair.name + " references missing module " + pair.left);
        if (!circuit.modules.contains(pair.right)) errors.push_back("symmetry pair " + pair.name + " references missing module " + pair.right);
        if (pair.axis != Axis::Vertical) errors.push_back("symmetry pair " + pair.name + " uses unsupported horizontal symmetry; only vertical symmetry is supported");
    }
    for (const auto& self : circuit.constraints.symmetry_selfs) {
        if (!circuit.modules.contains(self.module)) errors.push_back("symmetry self " + self.name + " references missing module " + self.module);
        if (self.axis != Axis::Vertical) errors.push_back("symmetry self " + self.name + " uses unsupported horizontal symmetry; only vertical symmetry is supported");
    }
    std::unordered_map<std::string, Axis> symmetry_group_axis;
    auto check_group_axis = [&](const std::string& name, Axis axis) {
        const auto [found, inserted] = symmetry_group_axis.emplace(name, axis);
        if (!inserted && found->second != axis) errors.push_back("symmetry group " + name + " mixes vertical and horizontal axes");
    };
    for (const auto& pair : circuit.constraints.symmetry_pairs) check_group_axis(pair.name, pair.axis);
    for (const auto& self : circuit.constraints.symmetry_selfs) check_group_axis(self.name, self.axis);
    for (const auto& flow : circuit.constraints.flows) {
        if (!circuit.nets.contains(flow.net)) errors.push_back("flow references missing net " + flow.net);
        if (!circuit.pins.contains(flow.out_pin)) errors.push_back("flow " + flow.net + " references missing out pin " + flow.out_pin);
        if (!circuit.pins.contains(flow.in_pin)) errors.push_back("flow " + flow.net + " references missing in pin " + flow.in_pin);
        if (circuit.nets.contains(flow.net)) {
            const auto& terminals = circuit.nets.at(flow.net).terminals;
            const std::unordered_set<std::string> terminal_set(terminals.begin(), terminals.end());
            if (circuit.pins.contains(flow.out_pin) && !terminal_set.contains(flow.out_pin)) errors.push_back("flow " + flow.net + " out pin " + flow.out_pin + " is not on the net");
            if (circuit.pins.contains(flow.in_pin) && !terminal_set.contains(flow.in_pin)) errors.push_back("flow " + flow.net + " in pin " + flow.in_pin + " is not on the net");
        }
    }
    for (const auto& [net, width] : circuit.constraints.wire_widths) {
        if (!circuit.nets.contains(net)) errors.push_back("wire width references missing net " + net);
        if (width.min_width <= 0.0 || width.max_width < width.min_width) errors.push_back("wire width for " + net + " has invalid range");
    }
    if (circuit.constraints.spacing_rules.device_spacing < 0.0) errors.push_back("device spacing must be non-negative");
    for (const auto& [layer, spacing] : circuit.constraints.spacing_rules.active_route_spacing) {
        if (layer != "ALL" && !routing::is_valid_layer_index(routing::layer_to_index(layer))) {
            errors.push_back("active route spacing uses unsupported layer " + layer);
        }
        if (spacing < 0.0) errors.push_back("active route spacing must be non-negative for " + layer);
    }
    for (const auto& [layer, spacing] : circuit.constraints.spacing_rules.diff_net_route_spacing) {
        if (layer != "ALL" && !routing::is_valid_layer_index(routing::layer_to_index(layer))) {
            errors.push_back("diff-net route spacing uses unsupported layer " + layer);
        }
        if (spacing < 0.0) errors.push_back("diff-net route spacing must be non-negative for " + layer);
    }
    return errors;
}

// 校验结果是否覆盖全部器件并满足已实现的线宽规则。
std::vector<std::string> validate_solution(const Circuit& circuit, const Solution& solution) {
    std::vector<std::string> errors;
    for (const auto& module : circuit.module_order) {
        if (!solution.placements.contains(module)) errors.push_back("missing placement for module " + module);
    }
    for (const auto& route : solution.routes) {
        if (!circuit.nets.contains(route.net)) errors.push_back("route references missing net " + route.net);
        const auto width = circuit.constraints.wire_widths.find(route.net);
        if (width != circuit.constraints.wire_widths.end() &&
            (route.width < width->second.min_width || route.width > width->second.max_width)) {
            errors.push_back("route " + route.net + " width violates configured range");
        }
    }
    return errors;
}

}  // namespace sapr
