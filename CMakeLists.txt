cmake_minimum_required(VERSION 3.14)
project(ping_pong_robot LANGUAGES CXX)

add_library(ping_pong INTERFACE)
target_include_directories(ping_pong INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(ping_pong INTERFACE cxx_std_17)

add_executable(optimize_launch examples/optimize_launch.cpp)
target_link_libraries(optimize_launch PRIVATE ping_pong)
