// 组织并运行全部无第三方依赖的 C++ 单元测试。
#include <exception>
#include <iostream>

#include "test_support.hpp"

// 顺序执行测试组并报告失败原因。
int main() {
    try {
        std::cout << "[test] io\n";
        run_io_tests();
        std::cout << "[test] constraints\n";
        run_constraint_tests();
        std::cout << "[test] router\n";
        run_router_tests();
        std::cout << "[test] routing evaluator\n";
        run_routing_evaluator_tests();
        std::cout << "[test] lcp binding\n";
        run_lcp_binding_tests();
        std::cout << "All tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return 1;
    }
}
