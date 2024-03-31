cmake_minimum_required(VERSION 3.16)
project(webgpu_hpp VERSION 6108 LANGUAGES CXX)

add_library(webgpu_hpp INTERFACE headers/webgpu.hpp)

message(" public headers ${PUBLIC_HEADER}")

add_subdirectory(imgui/ SYSTEM EXCLUDE_FROM_ALL)

target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})
