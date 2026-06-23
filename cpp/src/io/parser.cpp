// 文件职责：实现 AnalogSAPR 标准输入和 placement 文件的解析逻辑。
#include "io/parser.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace analog_sapr {
namespace {

// 保存一行去注释后的字段和原始注释，便于解析 modules.txt 中的 ox/oy。
struct ParsedLine {
    std::vector<std::string> parts;
    std::string comment;
};

// 去除字符串首尾空白。
std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

// 按空白字符拆分一行。
std::vector<std::string> split_ws(const std::string& value) {
    std::istringstream stream(value);
    std::vector<std::string> parts;
    std::string item;
    while (stream >> item) {
        parts.push_back(item);
    }
    return parts;
}

// 读取数据行，跳过空行和整行注释，同时保留行内注释。
std::vector<ParsedLine> read_data_lines(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("无法打开文件: " + path.string());
    }

    std::vector<ParsedLine> rows;
    std::string raw;
    while (std::getline(input, raw)) {
        const auto hash_pos = raw.find('#');
        const std::string body = trim(hash_pos == std::string::npos ? raw : raw.substr(0, hash_pos));
        const std::string comment = hash_pos == std::string::npos ? "" : trim(raw.substr(hash_pos + 1));
        if (!body.empty()) {
            rows.push_back({split_ws(body), comment});
        }
    }
    return rows;
}

// 解析逗号分隔的 active 矩形。
Rect parse_rect(const std::string& text, const std::filesystem::path& path) {
    std::vector<double> values;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        values.push_back(std::stod(item));
    }
    if (values.size() != 4) {
        throw std::runtime_error(path.string() + ": active 矩形需要 4 个数字: " + text);
    }
    return Rect{values[0], values[1], values[2], values[3]};
}

// 从注释中读取 ox 或 oy，缺省返回 0。
double parse_optional_offset(const std::string& comment, const std::string& key) {
    const std::regex pattern("\\b" + key + "=([-+]?\\d+(?:\\.\\d+)?)");
    std::smatch match;
    if (std::regex_search(comment, match, pattern)) {
        return std::stod(match[1].str());
    }
    return 0.0;
}

// 生成 module.pin 的全局 pin key。
std::string pin_key(const std::string& module, const std::string& name) {
    return module + "." + name;
}

// 解析 modules.txt。
std::unordered_map<std::string, Module> load_modules(const std::filesystem::path& path) {
    std::unordered_map<std::string, Module> modules;
    for (const auto& row : read_data_lines(path)) {
        if (row.parts.size() < 4) {
            throw std::runtime_error(path.string() + ": modules 行至少需要 4 列");
        }
        Module module;
        module.id = row.parts[0];
        module.width = std::stod(row.parts[1]);
        module.height = std::stod(row.parts[2]);
        module.active = parse_rect(row.parts[3], path);
        module.ox = parse_optional_offset(row.comment, "ox");
        module.oy = parse_optional_offset(row.comment, "oy");
        module.info = row.comment;
        modules[module.id] = module;
    }
    return modules;
}

// 解析 pins.txt。
std::unordered_map<std::string, Pin> load_pins(const std::filesystem::path& path) {
    std::unordered_map<std::string, Pin> pins;
    for (const auto& row : read_data_lines(path)) {
        if (row.parts.size() != 5) {
            throw std::runtime_error(path.string() + ": pins 行需要 5 列");
        }
        Pin pin;
        pin.module = row.parts[0];
        pin.name = row.parts[1];
        pin.x = std::stod(row.parts[2]);
        pin.y = std::stod(row.parts[3]);
        pin.layer = row.parts[4];
        pins[pin_key(pin.module, pin.name)] = pin;
    }
    return pins;
}

// 解析 nets.txt。
std::unordered_map<std::string, Net> load_nets(const std::filesystem::path& path) {
    std::unordered_map<std::string, Net> nets;
    for (const auto& row : read_data_lines(path)) {
        if (row.parts.size() < 3) {
            throw std::runtime_error(path.string() + ": nets 行至少需要 3 列");
        }
        Net net;
        net.name = row.parts[0];
        net.priority = row.parts[1];
        net.terminals.assign(row.parts.begin() + 2, row.parts.end());
        nets[net.name] = net;
    }
    return nets;
}

// 解析 constraints.txt。
Constraints load_constraints(const std::filesystem::path& path) {
    Constraints constraints;
    for (const auto& row : read_data_lines(path)) {
        const auto& parts = row.parts;
        if (parts[0] == "SYMMETRY_PAIR" && parts.size() == 5) {
            constraints.symmetry_pairs.push_back(SymmetryPair{parts[1], parts[2], parts[3], parts[4]});
        } else if (parts[0] == "SYMMETRY_SELF" && parts.size() == 4) {
            constraints.symmetry_selfs.push_back(SymmetrySelf{parts[1], parts[2], parts[3]});
        } else if (parts[0] == "FLOW" && parts.size() == 4) {
            constraints.flows.push_back(FlowConstraint{parts[1], parts[2], parts[3]});
        } else if (parts[0] == "WIRE_WIDTH" && parts.size() == 4) {
            WireWidthConstraint width;
            width.net = parts[1];
            width.min_width = std::stod(parts[2]);
            width.max_width = std::stod(parts[3]);
            constraints.wire_widths[width.net] = width;
        } else {
            throw std::runtime_error(path.string() + ": 不支持的约束行");
        }
    }
    return constraints;
}

}  // namespace

Circuit load_circuit(const std::filesystem::path& input_dir) {
    Circuit circuit;
    circuit.modules = load_modules(input_dir / "modules.txt");
    circuit.pins = load_pins(input_dir / "pins.txt");
    circuit.nets = load_nets(input_dir / "nets.txt");
    circuit.constraints = load_constraints(input_dir / "constraints.txt");
    return circuit;
}

std::unordered_map<std::string, Placement> load_placements(const std::filesystem::path& placement_path) {
    std::unordered_map<std::string, Placement> placements;
    for (const auto& row : read_data_lines(placement_path)) {
        if (row.parts.size() != 5) {
            throw std::runtime_error(placement_path.string() + ": placement 行需要 5 列");
        }
        Placement placement;
        placement.module = row.parts[0];
        placement.x = std::stod(row.parts[1]);
        placement.y = std::stod(row.parts[2]);
        placement.angle = std::stoi(row.parts[3]);
        placement.orient = row.parts[4];
        placements[placement.module] = placement;
    }
    return placements;
}

}  // namespace analog_sapr
