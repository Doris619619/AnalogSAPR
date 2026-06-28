# Analog SAPR C++20 项目的构建入口。
cmake_minimum_required(VERSION 3.20)
project(analog_sapr_reproduction VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sapr_core STATIC
    src/constraints.cpp
    src/geometry.cpp
    src/io.cpp
    src/optimizer.cpp
    src/routing/astar.cpp
    src/routing/dp_router.cpp
    src/routing/geometry.cpp
    src/routing/global_router.cpp
    src/routing/grid.cpp
    src/routing/layer.cpp
    src/routing/obstacle.cpp
    src/routing/routing_context.cpp
    src/routing/topology.cpp
    src/routing/transform.cpp
    src/router.cpp
    src/routing_evaluator.cpp
    src/tree.cpp
)
target_include_directories(sapr_core PUBLIC include)

if(MSVC)
    target_compile_options(sapr_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(sapr_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(sapr src/main.cpp)
target_link_libraries(sapr PRIVATE sapr_core)
if(MSVC)
    target_compile_options(sapr PRIVATE /W4 /permissive-)
else()
    target_compile_options(sapr PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(sapr_tests
        tests/test_main.cpp
        tests/test_constraints.cpp
        tests/test_io.cpp
        tests/test_routing_evaluator.cpp
        tests/test_router.cpp
    )
    target_link_libraries(sapr_tests PRIVATE sapr_core)
    target_include_directories(sapr_tests PRIVATE tests)
    target_compile_definitions(sapr_tests PRIVATE
        SAPR_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
        SAPR_BINARY_DIR="${CMAKE_BINARY_DIR}"
    )
    if(MSVC)
        target_compile_options(sapr_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(sapr_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME sapr_unit COMMAND sapr_tests)
    set_tests_properties(sapr_unit PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    add_test(NAME sapr_validate COMMAND sapr validate --input input)
    set_tests_properties(sapr_validate PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    add_test(NAME sapr_run COMMAND sapr run --input ../input --output cli-output --sa-iterations 5)
    set_tests_properties(sapr_run PROPERTIES WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()
