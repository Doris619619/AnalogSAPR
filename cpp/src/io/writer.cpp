// 文件职责：实现阶段 0 的布线结果写出逻辑。
#include "io/writer.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace analog_sapr {

void write_routes(const RoutingSolution& solution, const std::filesystem::path& output_dir) {
    std::filesystem::create_directories(output_dir);
    const auto path = output_dir / "routing.txt";
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("无法写入文件: " + path.string());
    }

    output << "# net         layer  x1       y1       x2       y2       width\n\n";
    output << std::fixed << std::setprecision(3);
    for (const auto& segment : solution.routes) {
        output << std::left << std::setw(12) << segment.net << " "
               << std::setw(5) << segment.layer << " "
               << std::setw(8) << segment.x1 << " "
               << std::setw(8) << segment.y1 << " "
               << std::setw(8) << segment.x2 << " "
               << std::setw(8) << segment.y2 << " "
               << std::setw(8) << segment.width << "\n";
    }
}

}  // namespace analog_sapr
