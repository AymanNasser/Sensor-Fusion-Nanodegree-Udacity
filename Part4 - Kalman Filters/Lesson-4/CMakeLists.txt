cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(UKF)

add_executable (ufk_main src/main.cpp src/ukf.cpp )
target_include_directories(ufk_main PRIVATE ${include})





