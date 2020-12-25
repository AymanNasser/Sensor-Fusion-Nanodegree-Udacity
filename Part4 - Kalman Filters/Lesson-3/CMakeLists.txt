cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(KF)

add_executable (main src/main.cpp src/kalman_filter.cpp src/tracking.cpp)
target_include_directories(main PRIVATE ${include})





