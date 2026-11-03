cmake_minimum_required(VERSION 3.14)
project(ping_pong_robot LANGUAGES CXX)

add_library(ping_pong_motion INTERFACE)
target_include_directories(ping_pong_motion INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_features(ping_pong_motion INTERFACE cxx_std_17)

add_executable(motion_example examples/motion_example.cpp)
target_link_libraries(motion_example PRIVATE ping_pong_motion)

add_executable(inverse_example examples/inverse_example.cpp)
target_link_libraries(inverse_example PRIVATE ping_pong_motion)

include(CTest)
if(BUILD_TESTING)
    add_executable(motion_tests tests/motion_tests.cpp)
    target_link_libraries(motion_tests PRIVATE ping_pong_motion)
    add_test(NAME motion_tests COMMAND motion_tests)
    add_executable(inverse_tests tests/inverse_tests.cpp)
    target_link_libraries(inverse_tests PRIVATE ping_pong_motion)
    add_test(NAME inverse_tests COMMAND inverse_tests)
endif()
