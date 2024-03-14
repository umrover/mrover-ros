cmake_minimum_required(VERSION 3.16)
project(webgpu_hpp VERSION 6108 LANGUAGES CXX)

add_library(webgpu_hpp INTERFACE webgpu.hpp)
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR})
