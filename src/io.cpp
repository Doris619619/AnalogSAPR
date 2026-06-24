// 实现仓库标准文本格式的解析、校验前建模和结果写出。
#include "sapr/io.hpp"

#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace sapr {
namespace {

// 表示去除注释后的一行字段及其注释内容。
struct DataLine {
    std::vector<std::string> fields;
    std::string comment;
};

// 去除字符串两端空白。
std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

// 将文件读取为忽略空行和整行注释的数据行。
std::vector<DataLine> read_data_lines(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open file: " + path.string());
    }
    std::vector<DataLine> result;
    std::string raw;
    std::size_t line_number = 0;
    while (std::getline(input, raw)) {
        ++line_number;
        const auto comment_pos = raw.find('#');
        const std::string body = trim(raw.substr(0, comment_pos));
        const std::string comment = comment_pos == std::string::npos ? "" : trim(raw.substr(comment_pos + 1));
        if (body.empty()) {
            continue;
        }
        std::istringstream stream(body);
        DataLine line;
        line.comment = comment;
        std::string field;
        while (stream >> field) {
            line.fields.push_back(field);
        }
        if (line.fields.empty()) {
            throw std::runtime_error(path.string() + ": invalid line " + std::to_string(line_number));
        }
        result.push_back(std::move(line));
    }
    return result;
}

// 将字符串严格解析为浮点数。
double parse_double(const std::string& value, const std::filesystem::path& path) {
    std::size_t consumed = 0;
    try {
        const double parsed = std::stod(value, &consumed);
        if (consumed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error(path.string() + ": invalid number: " + value);
    }
}

// 将字符串严格解析为整数。
int parse_int(const std::string& value, const std::filesystem::path& path) {
    std::size_t consumed = 0;
    try {
        const int parsed = std::stoi(value, &consumed);
        if (consumed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error(path.string() + ": invalid integer: " + value);
    }
}

// 将 priority 文本转换为枚举。
Priority parse_priority(const std::string& value, const std::filesystem::path& path) {
    if (value == "critical") return Priority::Critical;
    if (value == "symmetry") return Priority::Symmetry;
    if (value == "normal") return Priority::Normal;
    throw std::runtime_error(path.string() + ": invalid priority: " + value);
}

// 将 axis 文本转换为枚举。
Axis parse_axis(const std::string& value, const std::filesystem::path& path) {
    if (value == "vertical") return Axis::Vertical;
    if (value == "horizontal") return Axis::Horizontal;
    throw std::runtime_error(path.string() + ": invalid axis: " + value);
}

// 从逗号分隔字段解析 active region。
Rect parse_rect(const std::string& value, const std::filesystem::path& path) {
    std::vector<std::string> parts;
    std::stringstream stream(value);
    std::string part;
    while (std::getline(stream, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() != 4) {
        throw std::runtime_error(path.string() + ": invalid rectangle: " + value);
    }
    return {parse_double(parts[0], path), parse_double(parts[1], path),
            parse_double(parts[2], path), parse_double(parts[3], path)};
}

// 从模块注释中读取 ox 或 oy，缺失时返回零。
double parse_comment_offset(const std::string& comment, const std::string& key) {
    const std::regex pattern("\\b" + key + "=([-+]?\\d+(?:\\.\\d+)?)");
    std::smatch match;
    return std::regex_search(comment, match, pattern) ? std::stod(match[1].str()) : 0.0;
}

// 读取 modules.txt。
void load_modules(const std::filesystem::path& path, Circuit& circuit) {
    for (const auto& line : read_data_lines(path)) {
        if (line.fields.size() < 4) {
            throw std::runtime_error(path.string() + ": expected 4 module columns");
        }
        Module module{line.fields[0], parse_double(line.fields[1], path), parse_double(line.fields[2], path),
                      parse_rect(line.fields[3], path), parse_comment_offset(line.comment, "ox"),
                      parse_comment_offset(line.comment, "oy"), line.comment};
        circuit.module_order.push_back(module.id);
        circuit.modules[module.id] = std::move(module);
    }
}

// 读取 pins.txt。
void load_pins(const std::filesystem::path& path, Circuit& circuit) {
    for (const auto& line : read_data_lines(path)) {
        if (line.fields.size() != 5) {
            throw std::runtime_error(path.string() + ": expected 5 pin columns");
        }
        Pin pin{line.fields[0], line.fields[1], parse_double(line.fields[2], path),
                parse_double(line.fields[3], path), line.fields[4]};
        const std::string key = pin.key();
        circuit.pin_order.push_back(key);
        circuit.pins[key] = std::move(pin);
    }
}

// 读取 nets.txt。
void load_nets(const std::filesystem::path& path, Circuit& circuit) {
    for (const auto& line : read_data_lines(path)) {
        if (line.fields.size() < 3) {
            throw std::runtime_error(path.string() + ": expected at least 3 net columns");
        }
        Net net{line.fields[0], parse_priority(line.fields[1], path),
                {line.fields.begin() + 2, line.fields.end()}};
        circuit.net_order.push_back(net.name);
        circuit.nets[net.name] = std::move(net);
    }
}

// 读取 constraints.txt。
void load_constraints(const std::filesystem::path& path, Circuit& circuit) {
    for (const auto& line : read_data_lines(path)) {
        const auto& f = line.fields;
        if (f[0] == "SYMMETRY_PAIR" && f.size() == 5) {
            circuit.constraints.symmetry_pairs.push_back({f[1], parse_axis(f[2], path), f[3], f[4]});
        } else if (f[0] == "SYMMETRY_SELF" && f.size() == 4) {
            circuit.constraints.symmetry_selfs.push_back({f[1], parse_axis(f[2], path), f[3]});
        } else if (f[0] == "FLOW" && f.size() == 4) {
            circuit.constraints.flows.push_back({f[1], f[2], f[3]});
        } else if (f[0] == "WIRE_WIDTH" && f.size() == 4) {
            circuit.constraints.wire_widths[f[1]] = {f[1], parse_double(f[2], path), parse_double(f[3], path)};
        } else {
            throw std::runtime_error(path.string() + ": unsupported constraint row");
        }
    }
}

// 以固定小数精度写出 placement.txt。
void write_placements(const Solution& solution, const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot write file: " + path.string());
    output << "# module       x           y           angle  orient\n\n" << std::fixed << std::setprecision(3);
    for (const auto& id : solution.placement_order) {
        const auto& p = solution.placements.at(id);
        output << std::left << std::setw(15) << p.module << std::setw(12) << p.x << std::setw(12) << p.y
               << std::setw(7) << p.angle << p.orient << '\n';
    }
}

// 以固定小数精度写出 routing.txt。
void write_routes(const Solution& solution, const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) throw std::runtime_error("cannot write file: " + path.string());
    output << "# net         layer  x1       y1       x2       y2       width\n\n" << std::fixed << std::setprecision(3);
    for (const auto& r : solution.routes) {
        output << std::left << std::setw(13) << r.net << std::setw(6) << r.layer << std::setw(9) << r.x1
               << std::setw(9) << r.y1 << std::setw(9) << r.x2 << std::setw(9) << r.y2 << r.width << '\n';
    }
}

}  // namespace

// 从输入目录读取完整电路。
Circuit load_circuit(const std::filesystem::path& input_dir) {
    Circuit circuit;
    load_modules(input_dir / "modules.txt", circuit);
    load_pins(input_dir / "pins.txt", circuit);
    load_nets(input_dir / "nets.txt", circuit);
    load_constraints(input_dir / "constraints.txt", circuit);
    return circuit;
}

// 从输出目录读取已有布局布线结果。
Solution load_solution(const std::filesystem::path& output_dir) {
    Solution solution;
    const auto placement_path = output_dir / "placement.txt";
    for (const auto& line : read_data_lines(placement_path)) {
        if (line.fields.size() != 5) throw std::runtime_error(placement_path.string() + ": expected 5 placement columns");
        Placement p{line.fields[0], parse_double(line.fields[1], placement_path),
                    parse_double(line.fields[2], placement_path), parse_int(line.fields[3], placement_path), line.fields[4]};
        solution.placement_order.push_back(p.module);
        solution.placements[p.module] = std::move(p);
    }
    const auto routing_path = output_dir / "routing.txt";
    for (const auto& line : read_data_lines(routing_path)) {
        if (line.fields.size() != 7) throw std::runtime_error(routing_path.string() + ": expected 7 route columns");
        solution.routes.push_back({line.fields[0], line.fields[1], parse_double(line.fields[2], routing_path),
                                   parse_double(line.fields[3], routing_path), parse_double(line.fields[4], routing_path),
                                   parse_double(line.fields[5], routing_path), parse_double(line.fields[6], routing_path)});
    }
    return solution;
}

// 将布局布线结果写入标准输出文件。
void write_solution(const Solution& solution, const std::filesystem::path& output_dir) {
    std::filesystem::create_directories(output_dir);
    write_placements(solution, output_dir / "placement.txt");
    write_routes(solution, output_dir / "routing.txt");
}

}  // namespace sapr
