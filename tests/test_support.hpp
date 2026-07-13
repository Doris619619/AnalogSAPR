// 提供不依赖第三方框架的最小测试断言工具。
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#ifndef SAPR_SOURCE_DIR
#define SAPR_SOURCE_DIR "."
#endif

#ifndef SAPR_BINARY_DIR
#define SAPR_BINARY_DIR "build"
#endif

// 在条件不满足时抛出带说明的测试异常。
inline void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// 判断两个浮点数是否在容差范围内相等。
inline bool approx(double left, double right, double tolerance = 1e-9) {
    return std::abs(left - right) <= tolerance;
}

// 运行 I/O 相关测试。
void run_io_tests();
// 运行约束校验相关测试。
void run_constraint_tests();
// 运行几何、树、路由和指标测试。
void run_router_tests();
// 运行 A*/DP 布线评估集成测试。
void run_routing_evaluator_tests();
// 运行 LCP space 绑定与几何失配重绑测试。
void run_lcp_binding_tests();
