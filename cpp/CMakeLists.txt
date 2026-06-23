cmake_minimum_required(VERSION 3.16)

project(analog_sapr_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(analog_sapr_core
    src/io/parser.cpp
    src/io/writer.cpp
)

target_include_directories(analog_sapr_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(analog_sapr
    src/main.cpp
)

target_link_libraries(analog_sapr
    PRIVATE
        analog_sapr_core
)
